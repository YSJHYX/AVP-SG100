"""On/Off 采集边界与 D/E 安全语义的纯软件合同测试；不连接任何硬件。"""

from __future__ import annotations

from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import pytest

from sg100_teleop.camera.realsense_camera import RealSenseFrame
from sg100_teleop.data_collection.episode_controller import (
    EpisodeController,
    EpisodeState,
)
from sg100_teleop.data_collection.synchronized_sampler import (
    SynchronizedSampler,
)
from sg100_teleop.data_collection.vla_raw_buffer import VlaRawBuffer
from sg100_teleop.data_collection.vla_raw_schema import FreshnessThresholds
from sg100_teleop.robot.sg100_driver import HAND_DOF, MODE_JOINT_POSITION
from sg100_teleop.robot.sg100_restricted_avp_backend import (
    SG100RestrictedAVPBackend,
)
from sg100_teleop.robot.sg100_sdk_driver import (
    DEFAULT_SDK_DIR,
    load_vendor_user_ranges,
)
from sg100_teleop.sim.run_avp_arm_hand_sim import (
    disarm_real_command_authority,
)


class FakeTelemetry:
    def __init__(self, packets: list[dict] | None = None) -> None:
        self.packets = packets or []

    def latest_before(self, timestamp_ns: int):
        eligible = [
            p for p in self.packets
            if p["publish_monotonic_ns"] <= timestamp_ns
        ]
        return dict(eligible[-1]) if eligible else None

    def is_alive(self):
        return bool(self.packets)

    def clear(self):
        pass

    @property
    def stats(self):
        return {"received_packets": len(self.packets), "sequence_gap_count": 0}


def _frame(role: str, timestamp: int, number: int) -> RealSenseFrame:
    return RealSenseFrame(
        role=role,
        host_receive_monotonic_ns=timestamp,
        device_timestamp_ms=float(number) * 3.0,
        frame_number=number,
        color_rgb=np.full((2, 2, 3), number, dtype=np.uint8),
        depth_uint16=np.full((2, 2), number, dtype=np.uint16),
    )


class FakeCamera:
    def __init__(self, role: str, frames: list[RealSenseFrame]) -> None:
        self.role = role
        self.frames = frames

    @property
    def ready(self):
        return bool(self.frames)

    def latest_before(self, timestamp_ns: int):
        eligible = [
            f
            for f in self.frames
            if f.host_receive_monotonic_ns <= timestamp_ns
        ]
        return eligible[-1] if eligible else None

    def clear(self):
        pass

    def metadata(self):
        return {
            "model": "Intel RealSense D455",
            "serial": f"serial-{self.role}",
            "width": 2,
            "height": 2,
            "fps": 30,
            "depth_scale": 0.001,
            "color_intrinsics": np.eye(3),
            "depth_intrinsics": np.eye(3),
            "extrinsics_to_robot": "UNAVAILABLE / NOT_CALIBRATED",
        }


def _packet(timestamp_ns: int) -> dict:
    return {
        "schema_version": 1,
        "publish_sequence": 1,
        "publish_monotonic_ns": timestamp_ns,
        "telemetry_publish_hz_requested": 100.0,
        "tracking_valid": True,
        "avp_anchored": True,
        "motion_enabled": True,
        "estop_latched": False,
        "arm_q_goal_rad": np.arange(6, dtype=float).tolist(),
        "arm_qcmd_sent_rad": (10 + np.arange(6, dtype=float)).tolist(),
        "arm_qcmd_sent_timestamp_ns": timestamp_ns - 30,
        "arm_qcmd_sequence": 7,
        "arm_command_valid": True,
        "arm_qpos_rad": (20 + np.arange(6, dtype=float)).tolist(),
        "arm_qpos_timestamp_ns": timestamp_ns - 40,
        "arm_feedback_sequence": 8,
        "arm_feedback_valid": True,
        "controller_error": 0,
        "arm_status": 1,
        "feedback_channel": "TCP_POLL",
        "sender_hz": 100.0,
        "hand_qcmd_retargeted_rad": (30 + np.arange(11, dtype=float)).tolist(),
        "hand_qcmd_effective_canonical_rad": (
            40 + np.arange(11, dtype=float)
        ).tolist(),
        "hand_qcmd_sdk_deg": (50 + np.arange(11, dtype=float)).tolist(),
        "hand_qcmd_timestamp_ns": timestamp_ns - 50,
        "hand_command_sequence": 9,
        "hand_command_valid": True,
        "hand_feedback_sdk_rad": (60 + np.arange(11, dtype=float)).tolist(),
        "hand_feedback_modes": [7] * 11,
        "hand_feedback_timestamp_ns": timestamp_ns - 60,
        "hand_feedback_sequence": 10,
        "hand_feedback_valid": True,
        "hand_qpos_canonical_rad": None,
        "hand_feedback_canonical_valid": False,
    }


def _controller(tmp_path: Path):
    timestamp = time.monotonic_ns() - 1_000_000
    telemetry = FakeTelemetry([_packet(timestamp)])
    return EpisodeController(
        output_dir=tmp_path,
        hz=30,
        language_instruction="boundary contract",
        telemetry=telemetry,
        head_camera=FakeCamera("head", [_frame("head", timestamp, 1)]),
        wrist_camera=None,
        required_cameras=("head",),
        camera_modalities=("rgb",),
        telemetry_required=False,
    )


# ------------------------------------------------------------- pre-On


def test_ready_run_has_zero_episode_rows_and_media(tmp_path: Path) -> None:
    controller = _controller(tmp_path)
    deadline = time.monotonic() + 0.1
    while time.monotonic() < deadline:
        time.sleep(0.02)  # camera/telemetry warmup 窗口
    assert controller.state is EpisodeState.READY
    assert controller.buffer is None and controller.pending_dir is None
    assert not any(tmp_path.glob("episode_*.npz"))
    assert not any(tmp_path.glob("episode_*_media"))
    assert not any(tmp_path.glob(".pending*"))


# ---------------------------------------------------------- On boundary


def test_on_accepted_first_tick_equals_episode_start(tmp_path: Path) -> None:
    controller = _controller(tmp_path)
    controller.handle_input("On")
    assert controller.state is EpisodeState.RECORDING
    controller.handle_input("Off")
    arrays = controller.candidate_arrays
    assert arrays["timestamp_ns"][0] == controller.episode_start_monotonic_ns
    controller.handle_input("N")


def test_first_formal_source_sample_must_be_post_on() -> None:
    start = 1_000_000_000_000
    pre_on = FakeTelemetry([_packet(start - 10_000_000)])
    buffer = VlaRawBuffer()

    class MemoryWriter:
        backlog = 0
        drop_count = 0

        def enqueue(self, *args):
            return True

    sampler = SynchronizedSampler(
        hz=30,
        start_monotonic_ns=start,
        telemetry=pre_on,
        head_camera=FakeCamera(
            "head", [_frame("head", start - 10_000_000, 1)]
        ),
        wrist_camera=None,
        camera_roles=("head",),
        camera_modalities=("rgb",),
        buffer=buffer,
        media_writer=MemoryWriter(),
        thresholds=FreshnessThresholds(),
    )
    # 只有 pre-On source 时：latest-before 仍选中，但必须标 invalid，
    # 不得静默当作正式 sample
    sampler.sample_tick(1, start + 40_000_000, start + 40_000_000)
    row = buffer.arrays()
    assert not row["arm_command_valid"][0]
    assert not row["telemetry_valid"][0]
    assert not row["head_camera_valid"][0]

    post_on = FakeTelemetry([
        _packet(start - 10_000_000),
        _packet(start + 5_000_000),
    ])
    camera = FakeCamera("head", [
        _frame("head", start - 10_000_000, 1),
        _frame("head", start + 5_000_000, 2),
    ])
    buffer2 = VlaRawBuffer()
    sampler2 = SynchronizedSampler(
        hz=30,
        start_monotonic_ns=start,
        telemetry=post_on,
        head_camera=camera,
        wrist_camera=None,
        camera_roles=("head",),
        camera_modalities=("rgb",),
        buffer=buffer2,
        media_writer=MemoryWriter(),
        thresholds=FreshnessThresholds(),
    )
    sampler2.sample_tick(1, start + 40_000_000, start + 40_000_000)
    row2 = buffer2.arrays()
    assert row2["arm_command_valid"][0]
    assert row2["head_camera_valid"][0]
    assert row2["head_camera_frame_number"][0] == 2


# ---------------------------------------------------------- Off boundary


def _record_and_stop(controller: EpisodeController, telemetry: FakeTelemetry):
    controller.handle_input("On")
    telemetry.packets.append(_packet(time.monotonic_ns()))
    time.sleep(0.09)
    report = controller.handle_input("Off")
    return report


def test_off_stops_row_and_media_growth(tmp_path: Path) -> None:
    controller = _controller(tmp_path)
    report = _record_and_stop(controller, controller.telemetry)
    arrays = controller.candidate_arrays
    n = int(arrays["timestamp_ns"].size)
    assert n >= 2
    media = controller.pending_dir / "head" / "rgb"
    files_after_off = len(tuple(media.glob("*.jpg")))
    rows_after_off = len(controller.buffer.rows)
    # Off 后新 source 到达：不得新增 row / media enqueue
    controller.telemetry.packets.append(_packet(time.monotonic_ns()))
    time.sleep(0.12)
    assert len(controller.buffer.rows) == rows_after_off
    assert len(tuple(media.glob("*.jpg"))) == files_after_off == n
    assert report.metrics["frames"] == n
    controller.handle_input("N")


def test_writer_flush_only_pre_off_queue(tmp_path: Path) -> None:
    controller = _controller(tmp_path)
    _record_and_stop(controller, controller.telemetry)
    n = int(controller.candidate_arrays["timestamp_ns"].size)
    # Off 返回时 writer 已 join：flush 的全部是 Off 前 enqueue 的内容
    assert controller.media_writer.written_ticks == n
    assert controller.media_writer.drop_count == 0
    controller.handle_input("N")


def test_y_saved_episode_stays_within_on_off_boundary(tmp_path: Path) -> None:
    controller = _controller(tmp_path)
    _record_and_stop(controller, controller.telemetry)
    saved = controller.handle_input("Y")
    npz_path, media_path = saved
    with np.load(npz_path, allow_pickle=False) as archive:
        tick = archive["timestamp_ns"]
        start = int(archive["episode_start_monotonic_ns"])
        stop = int(archive["episode_stop_request_monotonic_ns"])
        assert tick[0] >= start
        assert int(tick.max()) <= stop
        # 正式（valid）source sample 必须在 [start, stop] 内；invalid 行允许
        # 保留 pre-On source timestamp 作为 RAW 证据
        for source_key, valid_key in (
            ("arm_qcmd_source_timestamp_ns", "arm_command_valid"),
            ("arm_qpos_source_timestamp_ns", "arm_feedback_valid"),
            ("hand_qcmd_source_timestamp_ns", "hand_command_valid"),
            ("hand_feedback_source_timestamp_ns", "hand_feedback_valid"),
            ("head_camera_host_timestamp_ns", "head_camera_valid"),
        ):
            valid = np.asarray(archive[valid_key], dtype=bool)
            source = np.asarray(archive[source_key], dtype=np.int64)
            formal = source[valid & (source > 0)]
            assert np.all(formal >= start), source_key
            assert np.all(formal <= stop), source_key
    assert npz_path.exists() and media_path.exists()


def test_n_discards_pending_and_keeps_no_permanent_episode(
    tmp_path: Path,
) -> None:
    controller = _controller(tmp_path)
    _record_and_stop(controller, controller.telemetry)
    controller.handle_input("N")
    assert controller.state is EpisodeState.READY
    assert not any(tmp_path.glob("episode_*.npz"))
    assert not any(tmp_path.glob("episode_*_media"))
    assert not any(tmp_path.glob(".pending*"))
    assert controller.next_episode_index == 0


# ------------------------------------------------------------- D / E


class FakeDriver:
    def __init__(self):
        self.vendor_user_ranges = load_vendor_user_ranges(DEFAULT_SDK_DIR)
        self.active_calls = []
        self.hold_calls = 0
        self.fault_active = False
        self.feedback = SimpleNamespace(
            valid=True,
            received_monotonic_ns=1_000_000_000,
            joint_modes=(MODE_JOINT_POSITION,) * HAND_DOF,
        )

    @property
    def diagnostics(self):
        return {"crc_error_count": 0}

    def send_mode7_zero_output_sdk_targets(self, positions):
        return SimpleNamespace(sent_monotonic_ns=1)

    def send_mode7_active_allowlist(self, indices, positions):
        self.active_calls.append((tuple(indices), np.asarray(positions)))
        return SimpleNamespace(sent_monotonic_ns=2)

    def hold_passive(self):
        self.hold_calls += 1
        return True

    def read_feedback(self):
        return self.feedback


class FakeArmBackend:
    def __init__(self):
        self.enabled = False
        self.calls = []

    def set_enabled(self, value: bool) -> None:
        self.enabled = value

    def stop_smooth(self) -> None:
        self.calls.append("stop_smooth")


def _anchored_backend():
    driver = FakeDriver()
    backend = SG100RestrictedAVPBackend(driver, joint_allowlist=("index_j3",))
    assert backend.anchor(np.zeros(HAND_DOF)).accepted
    return driver, backend


def test_d_is_disarm_only_and_double_d_stays_disarmed() -> None:
    arm = FakeArmBackend()
    _, hand = _anchored_backend()
    hand.set_enabled(True)
    arm.enabled = True
    assert disarm_real_command_authority(arm, hand) is True
    assert not arm.enabled and not hand.enabled
    assert arm.calls == ["stop_smooth"]
    # D 不是 toggle：再次 D 仍 disarmed，不产生额外动作
    assert disarm_real_command_authority(arm, hand) is False
    assert not arm.enabled and not hand.enabled


def test_d_then_avp_target_has_no_active_submission() -> None:
    driver, backend = _anchored_backend()
    backend.set_enabled(True)
    disarm_real_command_authority(None, backend)
    target = np.zeros(HAND_DOF)
    target[5] = np.deg2rad(0.25)
    result = backend.submit(
        target, tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    )
    assert not result.accepted
    assert result.reason == "NOT_ENABLED_NO_ACTIVE_COMMAND"
    assert driver.active_calls == []
    assert driver.hold_calls >= 1  # D 进入已有 passive hold 安全路径


def test_rearm_after_d_requires_explicit_a() -> None:
    driver, backend = _anchored_backend()
    backend.set_enabled(True)
    disarm_real_command_authority(None, backend)
    assert not backend.enabled
    # 只有显式 A（set_enabled(True)）才能重新 armed
    backend.set_enabled(True)
    target = np.zeros(HAND_DOF)
    target[5] = np.deg2rad(0.25)
    result = backend.submit(
        target, tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    )
    assert result.accepted and len(driver.active_calls) == 1


def test_e_stop_latch_is_not_cleared_by_d_or_a() -> None:
    driver, backend = _anchored_backend()
    backend.set_enabled(True)
    backend.emergency_stop()
    assert backend.estopped is True and not backend.enabled
    # D：normal disarm，不清 E-stop latch
    disarm_real_command_authority(None, backend)
    assert backend.estopped is True
    result = backend.submit(
        np.zeros(HAND_DOF), tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    )
    assert result.reason == "ESTOP_LATCHED"
    assert driver.active_calls == []
    # A：不能绕过 E-stop
    with pytest.raises(RuntimeError, match="C anchor is required"):
        backend.set_enabled(True)
    assert backend.estopped is True
