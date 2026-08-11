from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from avp_stream.streamer import TrackingData
from scipy.spatial.transform import Rotation

from sg100_teleop.control.joint_command_filter import JointCommandFilter
from sg100_teleop.control.teleop_arming import (
    TeleopArmingController,
    TeleopArmState,
)
from sg100_teleop.kinematics.rm65_ik import RM65IKSolver, so3_log
from sg100_teleop.retargeting.sg100_retargeter import SG100Retargeter
from sg100_teleop.sim.combined_robot_layout import (
    ARM_DOF,
    HAND_DOF,
    ROBOT_JOINT_ORDER,
    SG100_JOINT_ORDER,
    initialize_robot_state,
    resolve_robot_layout,
    write_robot_controls,
)
from sg100_teleop.sim.run_avp_arm_hand_sim import RM65_TELEOP_READY_Q
from sg100_teleop.tests.test_avp_feature_extractor import (
    bend_index,
    build_straight_hand,
)
from sg100_teleop.tests.test_hand_calibration import build_calibration
from sg100_teleop.tracking.avp_adapter import AVPHandAdapter
from sg100_teleop.tracking.avp_hand_adapter import AVP_JOINT_INDEX
from sg100_teleop.tracking.avp_source import get_hand_from_tracking
from sg100_teleop.tracking.avp_wrist_mapper import (
    AVPWristRelativeMapper,
    make_transform,
)
from sg100_teleop.tracking.frame_gate import (
    TrackingFrameGate,
    TrackingFrameStatus,
)


ROOT = Path(__file__).resolve().parents[2]
ROBOT_ONLY_MODEL = (
    ROOT / "sg100_teleop/assets/rm65_sg100/mjcf/rm65_sg100_sim.xml"
)
TABLETOP_MODEL = (
    ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/rm65_sg100_grasp_scene.xml"
)

NS = 1_000_000_000
TRANSLATION_SCALE = 0.5
NONIDENTITY_BASIS = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)


def _bend_finger(joints: np.ndarray, finger: str) -> np.ndarray:
    """Bend a named chain while retaining the existing AVP test convention."""
    result = joints.copy()
    knuckle = result[AVP_JOINT_INDEX[f"{finger}Knuckle"], :3, 3].copy()
    offsets = {
        f"{finger}IntermediateBase": [0.0, 0.020, -0.020],
        f"{finger}IntermediateTip": [0.0, 0.040, -0.040],
        f"{finger}Tip": [0.0, 0.060, -0.060],
    }
    for name, offset in offsets.items():
        result[AVP_JOINT_INDEX[name], :3, 3] = knuckle + offset
    return result


def fist_like_hand() -> np.ndarray:
    hand = bend_index(build_straight_hand())
    hand = _bend_finger(hand, "middle")
    return _bend_finger(hand, "little")


def pinch_like_hand() -> np.ndarray:
    hand = bend_index(build_straight_hand())
    thumb_positions = {
        "thumbIntermediateBase": [0.035, 0.035, -0.005],
        "thumbIntermediateTip": [0.025, 0.050, -0.010],
        "thumbTip": [0.015, 0.060, -0.012],
    }
    for name, position in thumb_positions.items():
        hand[AVP_JOINT_INDEX[name], :3, 3] = position
    return hand


def wrist_transform(
    translation=(0.0, 0.0, 0.0),
    rotation_vector=(0.0, 0.0, 0.0),
) -> np.ndarray:
    return make_transform(
        np.asarray(translation, dtype=np.float64),
        Rotation.from_rotvec(rotation_vector).as_matrix(),
    )


def tracking_frame(
    sequence: int,
    local_hand: np.ndarray,
    wrist: np.ndarray,
    *,
    valid: bool = True,
) -> TrackingData:
    """Build the same wrist-local payload consumed by production TrackingData."""
    local_hand = np.asarray(local_hand, dtype=np.float64)
    return TrackingData(
        {
            "right_wrist": np.asarray(wrist, dtype=np.float64)[None, :, :],
            "right_arm": local_hand,
            "right_fingers": local_hand[:25],
            "frame_sequence": sequence,
            "received_monotonic_ns": sequence * NS,
            "right_hand_valid": valid,
        }
    )


@dataclass(frozen=True)
class FrameResult:
    accepted: bool
    q_arm: np.ndarray
    q_hand: np.ndarray
    tcp_target: np.ndarray | None = None
    ik_converged: bool | None = None


class RawAVPIntegrationHarness:
    """Test-local composition of the existing production processing modules."""

    def __init__(self, model_path: Path):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.layout = resolve_robot_layout(self.model)
        self.ik = RM65IKSolver(self.model, tcp_body_name="sg100_base_link")
        self.adapter = AVPHandAdapter(build_calibration())
        self.retargeter = SG100Retargeter()
        self.gate = TrackingFrameGate(0.5, 0.5, started_monotonic_ns=0)
        self.arming = TeleopArmingController()
        self.mapper = AVPWristRelativeMapper(
            translation_scale=TRANSLATION_SCALE,
            R_tcp_from_avp_local=NONIDENTITY_BASIS,
        )
        self.filter = JointCommandFilter(dof=HAND_DOF, tau=0.06, max_velocity=4.0)

        assert tuple(self.retargeter.joint_order) == SG100_JOINT_ORDER
        assert len(self.layout.actuator_ids) == len(ROBOT_JOINT_ORDER) == 17
        assert self.model.nu == 17
        assert set(self.layout.arm_qpos_indices).isdisjoint(
            set(self.layout.hand_qpos_indices)
        )

        self.q_arm_goal = RM65_TELEOP_READY_Q.copy()
        self.q_hand_goal = np.zeros(HAND_DOF, dtype=np.float64)
        self.q_hand_cmd = self.q_hand_goal.copy()
        initialize_robot_state(
            self.model,
            self.data,
            self.layout,
            self.q_arm_goal,
            self.q_hand_goal,
        )
        self.filter.reset(self.q_hand_goal)

        tcp_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "sg100_base_link",
        )
        self.tcp_anchor = make_transform(
            self.data.xpos[tcp_id].copy(),
            self.data.xmat[tcp_id].reshape(3, 3).copy(),
        )
        self.t_avp_wrist_0 = None
        self.t_tcp_runtime_0 = None
        self.q_hand0_runtime = None

    def request_calibration(self, after_sequence: int) -> bool:
        return self.arming.request_neutral_calibration(after_sequence * NS + 1)

    def process(self, tracking: TrackingData) -> FrameResult:
        now_ns = int(tracking.received_monotonic_ns)
        if self.gate.inspect(tracking, now_ns) != TrackingFrameStatus.CANDIDATE:
            return FrameResult(
                False,
                self.q_arm_goal.copy(),
                self.q_hand_goal.copy(),
            )

        hand = get_hand_from_tracking(tracking, side="right")
        if hand is None:
            self.gate.reject_malformed()
            return FrameResult(
                False,
                self.q_arm_goal.copy(),
                self.q_hand_goal.copy(),
            )

        try:
            canonical = self.adapter.convert(hand)
            q_hand_candidate = self.retargeter.retarget(canonical)
            avp_wrist = np.asarray(hand[0], dtype=np.float64).copy()
            if q_hand_candidate.shape != (HAND_DOF,):
                raise ValueError("unexpected hand command shape")
            if not np.all(np.isfinite(q_hand_candidate)):
                raise ValueError("non-finite hand command")
            if avp_wrist.shape != (4, 4) or not np.all(np.isfinite(avp_wrist)):
                raise ValueError("malformed wrist transform")
        except (ValueError, np.linalg.LinAlgError):
            self.gate.reject_malformed()
            return FrameResult(
                False,
                self.q_arm_goal.copy(),
                self.q_hand_goal.copy(),
            )

        sequence = int(tracking.frame_sequence)
        self.arming.observe_valid_frame(sequence)
        self.gate.accept(now_ns)

        if self.arming.should_calibrate(sequence, now_ns):
            self.mapper.calibrate(avp_wrist, self.tcp_anchor)
            self.t_avp_wrist_0 = avp_wrist.copy()
            self.t_tcp_runtime_0 = self.tcp_anchor.copy()
            self.q_hand0_runtime = q_hand_candidate.copy()
            self.q_hand_goal = q_hand_candidate.copy()
            self.q_hand_cmd = q_hand_candidate.copy()
            self.filter.reset(q_hand_candidate)
            self.arming.complete_calibration()
            return FrameResult(
                True,
                self.q_arm_goal.copy(),
                self.q_hand_goal.copy(),
                self.tcp_anchor.copy(),
                None,
            )

        tcp_target = None
        converged = None
        if self.arming.active:
            tcp_target = self.mapper.map(avp_wrist)
            q_arm_candidate, ik_result = self.ik.solve(
                q_seed=self.q_arm_goal,
                target_position=tcp_target[:3, 3],
                target_rotation=tcp_target[:3, :3],
            )
            converged = ik_result.converged
            if converged:
                self.q_arm_goal = q_arm_candidate.copy()
            self.q_hand_goal = q_hand_candidate.copy()

        return FrameResult(
            True,
            self.q_arm_goal.copy(),
            self.q_hand_goal.copy(),
            None if tcp_target is None else tcp_target.copy(),
            converged,
        )

    def step(self, count: int = 150) -> None:
        for _ in range(count):
            self.q_hand_cmd = self.filter.update(
                self.q_hand_goal,
                self.model.opt.timestep,
            )
            write_robot_controls(
                self.data,
                self.layout,
                self.q_arm_goal,
                self.q_hand_cmd,
            )
            mujoco.mj_step(self.model, self.data)

    def assert_command_valid(self, result: FrameResult) -> None:
        assert result.q_arm.shape == (ARM_DOF,)
        assert result.q_hand.shape == (HAND_DOF,)
        assert np.all(np.isfinite(result.q_arm))
        assert np.all(np.isfinite(result.q_hand))
        assert np.all(result.q_arm >= self.ik.lower - 1e-12)
        assert np.all(result.q_arm <= self.ik.upper + 1e-12)
        hand_ranges = self.model.jnt_range[self.layout.joint_ids[ARM_DOF:]]
        assert np.all(result.q_hand >= hand_ranges[:, 0] - 1e-12)
        assert np.all(result.q_hand <= hand_ranges[:, 1] + 1e-12)


def assert_pose_matches_target(harness, result: FrameResult) -> None:
    assert result.tcp_target is not None
    p_fk, r_fk = harness.ik.fk(result.q_arm)
    np.testing.assert_allclose(p_fk, result.tcp_target[:3, 3], atol=1e-4)
    assert np.linalg.norm(so3_log(result.tcp_target[:3, :3] @ r_fk.T)) <= 1e-3


def bottle_freejoint(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    joint_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "water_bottle_free",
    )
    assert joint_id >= 0
    address = int(model.jnt_qposadr[joint_id])
    return data.qpos[address : address + 7].copy()


def run_raw_pipeline(model_path: Path, *, full_regression: bool):
    harness = RawAVPIntegrationHarness(model_path)
    open_hand = build_straight_hand()
    wrist0 = wrist_transform()

    initial_arm = harness.q_arm_goal.copy()
    initial_hand = harness.q_hand_goal.copy()
    startup = harness.process(tracking_frame(100, open_hand, wrist0))
    assert startup.accepted
    assert harness.arming.state == TeleopArmState.WAITING_FOR_NEUTRAL_CONFIRM
    assert not harness.mapper.calibrated
    np.testing.assert_allclose(startup.q_arm, initial_arm)
    np.testing.assert_allclose(startup.q_hand, initial_hand)

    assert harness.request_calibration(100)
    calibrated = harness.process(tracking_frame(101, open_hand, wrist0))
    assert calibrated.accepted
    assert harness.arming.state == TeleopArmState.TELEOP_ACTIVE
    assert harness.mapper.calibrated
    np.testing.assert_allclose(harness.t_avp_wrist_0, wrist0)
    np.testing.assert_allclose(harness.t_tcp_runtime_0, harness.tcp_anchor)
    assert harness.q_hand0_runtime is not None
    harness.assert_command_valid(calibrated)

    arm_before = harness.q_arm_goal.copy()
    hand_before = harness.q_hand_goal.copy()
    duplicate = harness.process(tracking_frame(101, fist_like_hand(), wrist0))
    assert not duplicate.accepted
    np.testing.assert_allclose(harness.q_arm_goal, arm_before)
    np.testing.assert_allclose(harness.q_hand_goal, hand_before)

    if not full_regression:
        active = harness.process(
            tracking_frame(
                102,
                fist_like_hand(),
                wrist_transform((0.010, 0.0, 0.0), (0.0, 0.0, 0.03)),
            )
        )
        assert active.accepted and active.ik_converged
        harness.assert_command_valid(active)
        robot_before = harness.data.qpos[harness.layout.qpos_indices].copy()
        harness.step()
        robot_after = harness.data.qpos[harness.layout.qpos_indices].copy()
        assert np.max(np.abs(robot_after[:ARM_DOF] - robot_before[:ARM_DOF])) > 0.0
        assert np.max(np.abs(robot_after[ARM_DOF:] - robot_before[ARM_DOF:])) > 0.0
        return harness, {"simultaneous": active}

    translations = {}
    for sequence, axis in zip((102, 103, 104), np.eye(3)):
        previous_arm = harness.q_arm_goal.copy()
        result = harness.process(
            tracking_frame(sequence, open_hand, wrist_transform(0.010 * axis))
        )
        assert result.accepted and result.ik_converged
        harness.assert_command_valid(result)
        assert not np.allclose(result.q_arm, previous_arm, atol=1e-8)
        np.testing.assert_allclose(result.q_hand, harness.q_hand0_runtime, atol=1e-12)
        expected_local = TRANSLATION_SCALE * (NONIDENTITY_BASIS @ (0.010 * axis))
        actual_local = (
            harness.tcp_anchor[:3, :3].T
            @ (result.tcp_target[:3, 3] - harness.tcp_anchor[:3, 3])
        )
        np.testing.assert_allclose(actual_local, expected_local, atol=1e-12)
        assert_pose_matches_target(harness, result)
        translations[sequence] = result

    goals_before_invalid = (harness.q_arm_goal.copy(), harness.q_hand_goal.copy())
    invalid = harness.process(
        tracking_frame(105, fist_like_hand(), wrist0, valid=False)
    )
    assert not invalid.accepted
    np.testing.assert_allclose(harness.q_arm_goal, goals_before_invalid[0])
    np.testing.assert_allclose(harness.q_hand_goal, goals_before_invalid[1])

    rotations = {}
    angle = 0.025
    for sequence, axis in zip((106, 107, 108), np.eye(3)):
        previous_arm = harness.q_arm_goal.copy()
        delta_rotation = Rotation.from_rotvec(angle * axis).as_matrix()
        result = harness.process(
            tracking_frame(
                sequence,
                open_hand,
                wrist_transform(rotation_vector=angle * axis),
            )
        )
        assert result.accepted and result.ik_converged
        harness.assert_command_valid(result)
        assert not np.allclose(result.q_arm, previous_arm, atol=1e-8)
        np.testing.assert_allclose(result.q_hand, harness.q_hand0_runtime, atol=1e-12)
        expected_rotation = (
            harness.tcp_anchor[:3, :3]
            @ NONIDENTITY_BASIS
            @ delta_rotation
            @ NONIDENTITY_BASIS.T
        )
        np.testing.assert_allclose(result.tcp_target[:3, :3], expected_rotation, atol=1e-12)
        assert_pose_matches_target(harness, result)
        rotations[sequence] = result

    articulated = {}
    for sequence, name, geometry in (
        (109, "open", open_hand),
        (110, "fist", fist_like_hand()),
        (111, "pinch", pinch_like_hand()),
    ):
        result = harness.process(tracking_frame(sequence, geometry, wrist0))
        assert result.accepted and result.ik_converged
        harness.assert_command_valid(result)
        assert_pose_matches_target(harness, result)
        articulated[name] = result

    np.testing.assert_allclose(
        articulated["open"].q_arm,
        articulated["fist"].q_arm,
        atol=1e-8,
    )
    np.testing.assert_allclose(
        articulated["fist"].q_arm,
        articulated["pinch"].q_arm,
        atol=1e-8,
    )
    assert np.linalg.norm(articulated["fist"].q_hand - articulated["open"].q_hand) > 0.5
    assert np.all(
        np.abs(
            articulated["fist"].q_hand[[4, 6, 9]]
            - articulated["open"].q_hand[[4, 6, 9]]
        )
        > 0.5
    )
    assert np.linalg.norm(articulated["pinch"].q_hand - articulated["open"].q_hand) > 0.5
    assert np.abs(articulated["pinch"].q_hand[0] - articulated["open"].q_hand[0]) > 0.5
    assert np.abs(articulated["pinch"].q_hand[4] - articulated["open"].q_hand[4]) > 0.5

    before_simultaneous_arm = harness.q_arm_goal.copy()
    before_simultaneous_hand = harness.q_hand_goal.copy()
    simultaneous = harness.process(
        tracking_frame(
            112,
            fist_like_hand(),
            wrist_transform((0.010, 0.0, 0.0), (0.0, 0.0, 0.025)),
        )
    )
    assert simultaneous.accepted and simultaneous.ik_converged
    harness.assert_command_valid(simultaneous)
    assert not np.allclose(simultaneous.q_arm, before_simultaneous_arm, atol=1e-8)
    assert not np.allclose(simultaneous.q_hand, before_simultaneous_hand, atol=1e-8)

    robot_before = harness.data.qpos[harness.layout.qpos_indices].copy()
    harness.step()
    robot_after = harness.data.qpos[harness.layout.qpos_indices].copy()
    assert np.max(np.abs(robot_after[:ARM_DOF] - robot_before[:ARM_DOF])) > 0.0
    assert np.max(np.abs(robot_after[ARM_DOF:] - robot_before[ARM_DOF:])) > 0.0
    assert harness.gate.accepted_frames == 12
    assert harness.gate.duplicate_frames == 1
    assert harness.gate.invalid_frames == 1

    return harness, {
        "translations": translations,
        "rotations": rotations,
        "articulated": articulated,
        "simultaneous": simultaneous,
    }


def run_regression():
    robot_harness, results = run_raw_pipeline(ROBOT_ONLY_MODEL, full_regression=True)
    tabletop_harness, _ = run_raw_pipeline(TABLETOP_MODEL, full_regression=False)

    bottle = bottle_freejoint(tabletop_harness.model, tabletop_harness.data)
    assert np.all(np.isfinite(bottle))
    np.testing.assert_allclose(np.linalg.norm(bottle[3:]), 1.0, atol=1e-8)
    assert tabletop_harness.gate.accepted_frames == 3
    assert tabletop_harness.gate.duplicate_frames == 1
    return robot_harness, results, bottle


def test_raw_avp_to_combined_mujoco():
    run_regression()


def main():
    robot_harness, results, bottle = run_regression()

    q_hand0 = robot_harness.q_hand0_runtime
    print("RAW AVP -> Q17 -> MUJOCO: PASS")
    print(
        "q_hand0_runtime: "
        f"shape={q_hand0.shape}, range=[{q_hand0.min():.6f}, {q_hand0.max():.6f}], "
        f"norm={np.linalg.norm(q_hand0):.6f}"
    )
    print(
        "translation IK: "
        + ", ".join(
            f"seq{sequence}={result.ik_converged}"
            for sequence, result in results["translations"].items()
        )
    )
    print(
        "rotation IK: "
        + ", ".join(
            f"seq{sequence}={result.ik_converged}"
            for sequence, result in results["rotations"].items()
        )
    )
    print(
        "articulation deltas: "
        f"fist={np.linalg.norm(results['articulated']['fist'].q_hand - results['articulated']['open'].q_hand):.6f}, "
        f"pinch={np.linalg.norm(results['articulated']['pinch'].q_hand - results['articulated']['open'].q_hand):.6f}"
    )
    print(f"tabletop bottle quaternion norm={np.linalg.norm(bottle[3:]):.12f}")


if __name__ == "__main__":
    main()
