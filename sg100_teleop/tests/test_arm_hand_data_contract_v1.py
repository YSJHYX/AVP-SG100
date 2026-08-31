"""No-hardware regression tests for rm65_sg100_joint17_v1."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import time

import numpy as np

from sg100_teleop.data_collection.adapter import load_physical_episode
from sg100_teleop.data_collection.camera import (
    CameraCalibration,
    NullCameraProvider,
    RealSenseCameraProvider,
)
from sg100_teleop.data_collection.collector import ArmHandDataCollector
from sg100_teleop.data_collection.schema import (
    PHYSICAL_DIM,
    SCHEMA_NAME,
    MarkovStatus,
    RejectionReason,
)
from sg100_teleop.robot.arm_state import ArmCommandSent, ArmState
from sg100_teleop.robot.rm65_real import RM65RealBackend
from sg100_teleop.tests.test_robot_backend_units import FakeSdkArm


def feedback(timestamp_ns: int, sequence: int, q, *, valid: bool = True) -> ArmState:
    return ArmState(
        q_rad=np.asarray(q, dtype=np.float64),
        qd_rad_s=np.full(6, 0.1),
        received_monotonic_ns=timestamp_ns,
        sequence=sequence,
        controller_error=0,
        arm_status=0,
        valid=valid,
        controller_tcp=np.arange(6, dtype=np.float64),
    )


def sent(bundle: int, timestamp_ns: int, sequence: int, q) -> ArmCommandSent:
    return ArmCommandSent(
        q_rad=np.asarray(q, dtype=np.float64),
        limiter_input_q_rad=np.asarray(q, dtype=np.float64) + 0.05,
        sent_monotonic_ns=timestamp_ns,
        source_timestamp_ns=timestamp_ns - 20,
        target_sequence=sequence,
        command_bundle_id=bundle,
    )


def make_collector(path: Path) -> ArmHandDataCollector:
    return ArmHandDataCollector(
        path,
        initial_hand_qcmd=np.zeros(11),
        arm_lower_rad=np.full(6, -3.2),
        arm_upper_rad=np.full(6, 3.2),
        hand_lower_rad=np.full(11, -2.0),
        hand_upper_rad=np.full(11, 2.0),
        started_timestamp_ns=1,
    )


def test_canonical_transition_uses_measured_feedback_and_actual_sent_command() -> None:
    with tempfile.TemporaryDirectory() as directory:
        collector = make_collector(Path(directory))
        collector.observe_arm_feedback(feedback(100, 1, np.full(6, 0.1)))
        collector.record_control_decision(
            command_bundle_id=1,
            decision_timestamp_ns=150,
            hand_command_applied_timestamp_ns=160,
            hand_qcmd_rad=np.full(11, 0.3),
            audit={
                "raw_avp_wrist_pose": np.eye(4),
                "mapped_relative_wrist_target": np.eye(4),
                "filtered_tcp_target": np.eye(4),
                "ik_q_arm_goal": np.full(6, 0.25),
                "continuity_gate_status": "not_applied",
                "latest_target_q": np.full(6, 0.25),
                "tracking_status": "valid",
                "safety_status": "ok",
            },
        )
        collector.ingest_arm_command_events([sent(1, 170, 1, np.full(6, 0.2))])
        assert not collector.transitions
        collector.observe_arm_feedback(feedback(200, 2, np.full(6, 0.15)))
        assert len(collector.transitions) == 1
        transition = collector.transitions[0]
        np.testing.assert_allclose(transition.state[:6], 0.1)
        np.testing.assert_allclose(transition.state[6:], 0.0)
        np.testing.assert_allclose(transition.action[:6], 0.2)
        np.testing.assert_allclose(transition.action[6:], 0.3)
        np.testing.assert_allclose(transition.next_state[:6], 0.15)
        np.testing.assert_allclose(transition.next_state[6:], 0.3)
        assert transition.state_timestamp_ns < transition.action_effective_timestamp_ns
        assert transition.action_effective_timestamp_ns < transition.next_state_timestamp_ns
        assert collector.markov_diagnostic() is MarkovStatus.INSUFFICIENT_DATA


def test_next_hand_state_is_latest_post_action_command_proxy() -> None:
    with tempfile.TemporaryDirectory() as directory:
        collector = make_collector(Path(directory))
        collector.observe_arm_feedback(feedback(100, 1, np.zeros(6)))
        collector.record_control_decision(
            command_bundle_id=1,
            decision_timestamp_ns=140,
            hand_command_applied_timestamp_ns=150,
            hand_qcmd_rad=np.full(11, 0.2),
        )
        collector.ingest_arm_command_events([sent(1, 160, 1, np.full(6, 0.1))])
        collector.record_control_decision(
            command_bundle_id=2,
            decision_timestamp_ns=170,
            hand_command_applied_timestamp_ns=180,
            hand_qcmd_rad=np.full(11, 0.4),
        )
        collector.observe_arm_feedback(feedback(200, 2, np.full(6, 0.05)))
        np.testing.assert_allclose(collector.transitions[0].next_state[6:], 0.4)


def test_missing_current_hand_qcmd_never_fabricates_zero_baseline() -> None:
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        collector = ArmHandDataCollector(root, started_timestamp_ns=50)
        collector.observe_arm_feedback(feedback(100, 1, np.zeros(6)))
        collector.record_control_decision(
            command_bundle_id=1,
            decision_timestamp_ns=110,
            hand_command_applied_timestamp_ns=115,
            hand_qcmd_rad=np.full(11, 0.2),
        )
        collector.ingest_arm_command_events([sent(1, 120, 1, np.zeros(6))])
        collector.finalize()
        assert collector.hand_proxy_baseline_valid is False
        assert RejectionReason.MISSING_HAND_PROXY_BASELINE in {
            rejection.reason for rejection in collector.rejections
        }
        raw = (root / "arm_hand_raw_events.jsonl").read_text(encoding="utf-8")
        assert '"event_type": "hand_proxy_baseline"' not in raw
        manifest = json.loads(
            (root / "arm_hand_manifest.json").read_text(encoding="utf-8")
        )
        assert manifest["hand_proxy_baseline_serialized"] is False


def test_first_action_without_preroll_feedback_is_hard_rejected() -> None:
    with tempfile.TemporaryDirectory() as directory:
        collector = make_collector(Path(directory))
        collector.record_control_decision(
            command_bundle_id=1,
            decision_timestamp_ns=100,
            hand_command_applied_timestamp_ns=110,
            hand_qcmd_rad=np.zeros(11),
        )
        collector.ingest_arm_command_events([sent(1, 120, 1, np.zeros(6))])
        assert not collector.transitions
        assert RejectionReason.NO_PRE_ACTION_FEEDBACK in {
            rejection.reason for rejection in collector.rejections
        }


def test_hard_rejection_flags_do_not_enter_canonical_manifest() -> None:
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        collector = make_collector(root)
        collector.observe_arm_feedback(feedback(100, 1, np.zeros(6)))
        collector.record_control_decision(
            command_bundle_id=1,
            decision_timestamp_ns=130,
            hand_command_applied_timestamp_ns=140,
            hand_qcmd_rad=np.zeros(11),
            tracking_valid=False,
        )
        collector.ingest_arm_command_events([sent(1, 150, 1, np.zeros(6))])
        episode_path = collector.finalize()
        episode = load_physical_episode(episode_path)
        assert episode.state.shape == (0, PHYSICAL_DIM)
        manifest = json.loads((root / "arm_hand_manifest.json").read_text())
        assert manifest["schema"] == SCHEMA_NAME
        assert manifest["rejection_counts"]["TRACKING_LOST"] == 1
        assert manifest["camera"]["head_camera_present"] is False
        assert manifest["camera"]["wrist_camera_present"] is False
        assert manifest["camera"]["multimodal_training_ready"] is False


def test_invalid_hand_shape_is_hard_rejected_without_crashing() -> None:
    with tempfile.TemporaryDirectory() as directory:
        collector = make_collector(Path(directory))
        collector.record_control_decision(
            command_bundle_id=1,
            decision_timestamp_ns=100,
            hand_command_applied_timestamp_ns=110,
            hand_qcmd_rad=np.zeros(10),
        )
        assert len(collector.transitions) == 0
        assert collector.rejections[-1].reason is RejectionReason.HAND_COMMAND_INVALID


def test_saved_canonical_values_remain_physical_radians() -> None:
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        collector = make_collector(root)
        collector.observe_arm_feedback(feedback(100, 1, np.full(6, 2.5)))
        collector.record_control_decision(
            command_bundle_id=1,
            decision_timestamp_ns=130,
            hand_command_applied_timestamp_ns=140,
            hand_qcmd_rad=np.full(11, 1.5),
        )
        collector.ingest_arm_command_events([sent(1, 150, 1, np.full(6, 2.6))])
        collector.observe_arm_feedback(feedback(180, 2, np.full(6, 2.55)))
        episode = load_physical_episode(collector.finalize())
        assert float(episode.state.max()) == 2.5
        assert float(episode.action.max()) == 2.6
        assert float(episode.next_state.max()) == 2.55
        with np.load(root / "arm_hand_joint17_v1.npz", allow_pickle=False) as archive:
            assert archive["arm_qvel_pre_rad_s"].shape == (1, 6)
            assert archive["previous_action"].shape == (1, 17)
            assert archive["hand_qcmd_velocity_proxy_rad_s"].shape == (1, 11)
            assert archive["controller_tcp_pre"].shape == (1, 6)
            assert archive["tracking_status"].tolist() == ["valid"]


def test_null_camera_and_future_realsense_never_fabricate_images() -> None:
    null = NullCameraProvider()
    assert not null.present and null.latest("head") is None
    assert null.metadata()["multimodal_training_ready"] is False
    future = RealSenseCameraProvider()
    assert not future.present
    try:
        future.latest("wrist")
    except NotImplementedError:
        pass
    else:
        raise AssertionError("future RealSense interface must not emit fake frames")
    calibration = CameraCalibration(
        calibration_id="future",
        parent_frame="base",
        transform_parent_camera=np.eye(4),
        intrinsics=np.eye(3),
        distortion=np.zeros(5),
        depth_scale=0.001,
    )
    calibration.validate()


def test_real_backend_emits_only_successful_post_limiter_command() -> None:
    fake = FakeSdkArm()
    backend = RM65RealBackend(
        "fake",
        sdk=fake,
        send_hz=100.0,
        feedback_poll_hz=100.0,
        qdot_max_rad_s=1.0,
        qddot_max_rad_s2=10.0,
    )
    backend.connect()
    backend.start()
    backend.set_enabled(True)
    try:
        backend.send_joint_target(np.full(6, 0.5), command_bundle_id=77)
        deadline = time.monotonic() + 0.2
        events = ()
        while time.monotonic() < deadline and not events:
            time.sleep(0.01)
            events = backend.drain_command_events()
        assert events
        event = events[0]
        event.validate()
        assert event.command_bundle_id == 77
        assert event.q_rad.shape == (6,)
        assert np.all(np.abs(event.q_rad) < 0.5)
        np.testing.assert_allclose(event.limiter_input_q_rad, 0.5)
        assert event.sent_monotonic_ns > event.source_timestamp_ns
    finally:
        backend.set_enabled(False)
        backend.disconnect()


def main() -> None:
    tests = [value for name, value in globals().items() if name.startswith("test_")]
    for test in tests:
        test()
        print(f"{test.__name__}: PASS")
    print("RM65-SG100 DATA CONTRACT V1: PASS")


if __name__ == "__main__":
    main()
