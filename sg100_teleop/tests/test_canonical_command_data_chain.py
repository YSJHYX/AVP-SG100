"""AVP -> SG100Retargeter -> backend 数据链端到端测试（无硬件）。

锁定三件事：

1. 同一个 authoritative canonical q_hand_cmd[11] 同时进入 MuJoCo
   backend（ctrl）与 restricted SG100 real backend，数值完全一致；
2. F_cmd（J7 符号翻转等）只发生在 real backend 内部，绝不 in-place
   修改 shared canonical command，MuJoCo 看到的仍是 canonical 值；
3. recording 显式保存 authoritative hand command（含元数据），而不是
   从 MuJoCo qpos slice 反推。
"""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from sg100_teleop.hand.canonical_hand import CanonicalHandState
from sg100_teleop.recording.mujoco_state_recorder import (
    HAND_COMMAND_DOMAIN,
    HAND_COMMAND_UNIT,
    MujocoStateRecorder,
)
from sg100_teleop.retargeting.sg100_retargeter import SG100Retargeter
from sg100_teleop.robot.sg100_driver import HAND_DOF, MODE_JOINT_POSITION
from sg100_teleop.robot.sg100_restricted_avp_backend import (
    SG100RestrictedAVPBackend,
)
from sg100_teleop.robot.sg100_sdk_driver import (
    DEFAULT_SDK_DIR,
    load_vendor_user_ranges,
)
from sg100_teleop.sim.combined_robot_layout import (
    SG100_JOINT_ORDER,
    resolve_robot_layout,
    write_robot_controls,
)


ROOT = Path(__file__).resolve().parents[2]
# Full combined scene: satisfies the recorder's VLA-camera contract AND
# the 17-DoF named robot layout in one model.
SIM_XML = (
    ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/rm65_sg100_avp_grasp_scene.xml"
)

NORMAL_GRASP = CanonicalHandState(
    thumb_opposition=0.7595,
    thumb_abduction=0.8610,
    thumb_distal=0.918,
    index_adduction=0.0,
    index_flexion=0.9795,
    index_distal=0.8,
    middle_flexion=1.0,
    middle_distal=0.8,
    little_outward=0.0,
    little_flexion=1.0,
    little_distal=0.8,
)


class FakeDriver:
    """Restricted-backend contract double（复用既有测试形态）."""

    def __init__(self):
        self.vendor_user_ranges = load_vendor_user_ranges(DEFAULT_SDK_DIR)
        self.zero_calls = []
        self.active_calls = []
        self.hold_calls = 0
        self.fault_active = False
        self.crc_count = 0
        self.feedback = SimpleNamespace(
            valid=True,
            received_monotonic_ns=1_000_000_000,
            joint_modes=(MODE_JOINT_POSITION,) * HAND_DOF,
        )

    @property
    def diagnostics(self):
        return {"crc_error_count": self.crc_count}

    def send_mode7_zero_output_sdk_targets(self, positions):
        self.zero_calls.append(np.asarray(positions).copy())
        return SimpleNamespace(sent_monotonic_ns=1)

    def send_mode7_active_allowlist(self, indices, positions):
        self.active_calls.append(
            (tuple(indices), np.asarray(positions).copy())
        )
        return SimpleNamespace(sent_monotonic_ns=2)

    def hold_passive(self):
        self.hold_calls += 1
        return True

    def read_feedback(self):
        return self.feedback


@pytest.fixture(scope="module")
def sim():
    model = mujoco.MjModel.from_xml_path(str(SIM_XML))
    data = mujoco.MjData(model)
    layout = resolve_robot_layout(model)
    return model, data, layout


# ----------------------------------------------------------------------
# §5 joint order：canonical index ↔ MuJoCo actuator/joint 绑定
# ----------------------------------------------------------------------


def test_canonical_order_binds_mujoco_actuators_by_name(sim):
    model, _, layout = sim
    retargeter = SG100Retargeter()

    assert tuple(retargeter.joint_order) == SG100_JOINT_ORDER

    for i, name in enumerate(SG100_JOINT_ORDER):
        actuator_id = int(layout.hand_actuator_ids[i])
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        bound_name = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
        )
        assert bound_name == name, (
            f"canonical index {i} ({name}) must drive MuJoCo joint "
            f"{name}; actuator bound to {bound_name}"
        )
        # Position servo with identity control scale: ctrl units are
        # joint radians, no extra transform.
        assert tuple(model.actuator_gear[actuator_id][:1]) == (1.0,)
        lo, hi = model.actuator_ctrlrange[actuator_id]
        jlo, jhi = model.jnt_range[joint_id]
        assert lo == pytest.approx(jlo) and hi == pytest.approx(jhi)


# ----------------------------------------------------------------------
# §17 same canonical command: MuJoCo ctrl == real backend pre-F_cmd
# ----------------------------------------------------------------------


def test_same_canonical_command_reaches_mujoco_ctrl_and_real_backend(sim):
    model, data, layout = sim
    retargeter = SG100Retargeter()
    driver = FakeDriver()
    backend = SG100RestrictedAVPBackend(
        driver,
        joint_allowlist=("index_j3",),
        max_velocity_rad_s=10.0,
        max_cycle_delta_rad=10.0,
        tracking_spike_rad=10.0,  # 数值一致性测试：单步直达，不测 envelope
    )

    q_cmd = retargeter.retarget(NORMAL_GRASP)
    assert q_cmd.shape == (HAND_DOF,)
    assert q_cmd.dtype == np.float64

    # Anchor + enable（C/A gate），然后同一变量双投递：
    assert backend.anchor(np.zeros(HAND_DOF)).accepted
    backend.set_enabled(True)

    q_hand_cmd = q_cmd  # runner 中的同一变量（filter identity 语义）
    write_robot_controls(
        data, layout, np.zeros(6), q_hand_cmd
    )
    result = backend.submit(
        q_hand_cmd,
        tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000,
        dt_s=1.0,  # 单步到位：验证的是数值一致性而非限速
    )

    assert result.accepted, result.reason
    mujoco_ctrl_hand = data.ctrl[layout.hand_actuator_ids]
    np.testing.assert_array_equal(
        mujoco_ctrl_hand,
        q_hand_cmd,
        err_msg="MuJoCo ctrl must receive the identical canonical vector",
    )
    np.testing.assert_array_equal(
        result.canonical_qcmd_rad,
        q_hand_cmd,
        err_msg="real backend must receive the identical canonical vector",
    )
    # driver 侧 SDK 目标：allowlist 内的轴 = F_cmd(canonical)；未解锁
    # 轴保持 anchor 缓存的零输出目标（产品语义，不激活其它关节）。
    indices, sdk_deg = driver.active_calls[-1]
    assert indices == backend.active_joint_indices
    expected_sdk_deg = np.degrees(
        backend.hardware_map.gain * q_hand_cmd
        + backend.hardware_map.offset_rad
    )
    for i in backend.active_joint_indices:
        assert sdk_deg[i] == pytest.approx(expected_sdk_deg[i]), i
    inactive = [
        i for i in range(HAND_DOF) if i not in backend.active_joint_indices
    ]
    np.testing.assert_array_equal(
        sdk_deg[inactive],
        np.zeros(len(inactive)),
        err_msg="inactive joints must keep the zero-output anchor target",
    )


# ----------------------------------------------------------------------
# §18 J7：F_cmd 只在 real backend 内部，绝不污染 shared command
# ----------------------------------------------------------------------


def test_j7_mapping_does_not_mutate_shared_canonical_command(sim):
    model, data, layout = sim
    driver = FakeDriver()
    # J7 尚无真机证据：restricted backend 构造必须拒绝它进入
    # allowlist；因此这里直接驱动 internal F_cmd + anchor/submit 路径，
    # 仅验证映射方向与 shared command 的不可变性。
    backend = SG100RestrictedAVPBackend(
        driver,
        joint_allowlist=("index_j3",),
        max_velocity_rad_s=10.0,
        max_cycle_delta_rad=10.0,
        tracking_spike_rad=10.0,  # 同上：验证映射与不可变性，不测 envelope
    )

    shared = np.zeros(HAND_DOF)
    shared[6] = np.deg2rad(-20.0)  # canonical middle_j1 = -20 deg
    shared_copy = shared.copy()

    assert backend.anchor(np.zeros(HAND_DOF)).accepted
    backend.set_enabled(True)
    result = backend.submit(
        shared,
        tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000,
        dt_s=1.0,
    )
    assert result.accepted, result.reason

    # shared canonical command 本身没有被 in-place 修改
    np.testing.assert_array_equal(
        shared, shared_copy, err_msg="shared canonical command was mutated"
    )

    # MuJoCo 消费的仍是 canonical -20 deg
    write_robot_controls(data, layout, np.zeros(6), shared)
    mujoco_j7 = np.degrees(
        data.ctrl[layout.hand_actuator_ids][6]
    )
    assert mujoco_j7 == pytest.approx(-20.0)

    # F_cmd 的 J7 方向：纯映射（不经过 allowlist 发送路径）
    mapped_sdk_deg = backend._validate_and_map(shared)
    assert mapped_sdk_deg[6] == pytest.approx(+20.0), (
        "F_cmd must flip canonical middle_j1 -20 deg to SDK J7 +20 deg"
    )
    assert mapped_sdk_deg[5] == pytest.approx(
        float(np.degrees(shared[5]))
    ), "identity axes must map 1:1"

    # 真机侧：J7 不在 allowlist，其 SDK 目标保持 anchor 零输出，
    # 即 canonical -20 deg 绝不会以任何形式泄露给真机 J7。
    _, sdk_deg = driver.active_calls[-1]
    assert sdk_deg[6] == pytest.approx(0.0)


def test_j7_still_blocked_from_real_allowlist():
    with pytest.raises(ValueError, match="active-command evidence"):
        SG100RestrictedAVPBackend(
            FakeDriver(), joint_allowlist=("middle_j1",)
        )


# ----------------------------------------------------------------------
# §19 recording：authoritative hand command 显式保存
# ----------------------------------------------------------------------


def test_recording_stores_authoritative_hand_command(sim):
    model, data, layout = sim
    import tempfile

    retargeter = SG100Retargeter()
    q_cmd = retargeter.retarget(NORMAL_GRASP)

    with tempfile.TemporaryDirectory() as tmp:
        recorder = MujocoStateRecorder(
            model,
            scene_path=SIM_XML,
            recording_dir=tmp,
            record_fps=30.0,
            hand_qpos_indices=layout.hand_qpos_indices,
        )

        def latch_and_step(k: int) -> None:
            # MuJoCo 仿真响应（qpos）与命令刻意不同：证明两个字段
            # 来源独立，command 不是从 qpos 反推的。
            data.qpos[layout.hand_qpos_indices] = q_cmd * (0.5 + 0.1 * k)
            recorder.set_hand_command(q_cmd)
            data.time = float(data.time) + recorder.period
            recorder.maybe_record(data)

        # start() 立即采样首帧：先注入 qpos 与 command 再启动。
        data.qpos[layout.hand_qpos_indices] = q_cmd * 0.5
        recorder.set_hand_command(q_cmd)
        recorder.start(data)
        for k in (1, 2, 3):
            latch_and_step(k)
        output = recorder.stop()

        with np.load(output, allow_pickle=False) as archive:
            recorded_cmd = archive["hand_qcmd_canonical_rad"]
            recorded_qpos = archive["hand_qpos_mujoco_rad"]
            names = [str(v) for v in archive["hand_joint_names"]]
            unit = str(archive["hand_command_unit"])
            domain = str(archive["hand_command_domain"])

        assert recorded_cmd.shape == (4, HAND_DOF)
        assert recorded_qpos.shape == (4, HAND_DOF)
        np.testing.assert_array_equal(recorded_cmd, np.stack([q_cmd] * 4))
        # command != qpos slice（仿真响应独立存在）
        assert not np.allclose(recorded_cmd, recorded_qpos)
        assert names == list(SG100_JOINT_ORDER)
        assert unit == HAND_COMMAND_UNIT == "rad"
        assert domain == HAND_COMMAND_DOMAIN == "canonical"

        metadata = json.loads(
            recorder.metadata_path.read_text(encoding="utf-8")
            if recorder.metadata_path is not None
            else "{}"
        )
        hand_meta = {
            k: v
            for k, v in metadata.items()
            if k.startswith("hand_")
        }
        assert hand_meta["hand_dof"] == HAND_DOF
        assert hand_meta["hand_joint_names"] == list(SG100_JOINT_ORDER)
        assert hand_meta["hand_command_unit"] == "rad"
        assert hand_meta["hand_command_domain"] == "canonical"
        assert "SG100Retargeter" in hand_meta["hand_command_source"]
        assert hand_meta["hand_command_missing_frames"] == 0


def test_recording_without_hand_config_stays_backward_compatible(sim):
    model, data, _ = sim
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        recorder = MujocoStateRecorder(
            model,
            scene_path=SIM_XML,
            recording_dir=tmp,
            record_fps=30.0,
        )
        recorder.start(data)
        data.time += recorder.period
        recorder.maybe_record(data)
        output = recorder.stop()

        with np.load(output, allow_pickle=False) as archive:
            # 旧字段集完整保留；hand 数组字段不出现
            for name in (
                "timestamps",
                "qpos",
                "mocap_pos",
                "mocap_quat",
            ):
                assert name in archive
            assert "hand_qcmd_canonical_rad" not in archive
