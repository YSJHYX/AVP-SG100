"""Synthetic, hardware-free tests for the DATA CONTRACT V1 inspector."""

from __future__ import annotations

from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile

import numpy as np

from sg100_teleop.data_collection.collector import ArmHandDataCollector
from sg100_teleop.data_collection.inspect_episode import (
    format_summary,
    format_transition,
    inspect_episode,
    main as inspector_main,
)
from sg100_teleop.data_collection.schema import PHYSICAL_DIM, SCHEMA_NAME
from sg100_teleop.robot.arm_state import ArmCommandSent, ArmState


def _event(event_type: str, timestamp: int, payload: dict) -> dict:
    return {
        "event_type": event_type,
        "sequence_id": timestamp,
        "master_timestamp_ns": timestamp,
        "payload": payload,
    }


def make_episode(
    root: Path,
    *,
    mutate_arrays=None,
    mutate_events=None,
    mutate_manifest=None,
) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    state = np.zeros((2, PHYSICAL_DIM), dtype=np.float64)
    action = np.zeros((2, PHYSICAL_DIM), dtype=np.float64)
    next_state = np.zeros((2, PHYSICAL_DIM), dtype=np.float64)

    state[0, :6] = 0.10
    action[0, :6] = 0.11
    next_state[0, :6] = 0.105
    action[0, 6:] = 0.20
    next_state[0, 6:] = 0.20

    state[1, :6] = 0.12
    state[1, 6:] = 0.20
    action[1, :6] = 0.13
    action[1, 6:] = 0.30
    next_state[1, :6] = 0.125
    next_state[1, 6:] = 0.30

    arrays = {
        "schema_name": np.asarray(SCHEMA_NAME),
        "state": state,
        "action": action,
        "next_state": next_state,
        "command_bundle_id": np.asarray([1, 2], dtype=np.int64),
        "state_timestamp_ns": np.asarray([100, 200], dtype=np.int64),
        "decision_timestamp_ns": np.asarray([110, 210], dtype=np.int64),
        "arm_command_sent_timestamp_ns": np.asarray([120, 220], dtype=np.int64),
        "hand_command_applied_timestamp_ns": np.asarray([115, 215], dtype=np.int64),
        "action_effective_timestamp_ns": np.asarray([120, 220], dtype=np.int64),
        "next_state_timestamp_ns": np.asarray([150, 250], dtype=np.int64),
    }
    if mutate_arrays is not None:
        mutate_arrays(arrays)
    np.savez_compressed(root / "arm_hand_joint17_v1.npz", **arrays)

    events = [
        _event(
            "hand_proxy_baseline",
            50,
            {
                "sequence_id": 50,
                "hand_qcmd": np.zeros(11).tolist(),
                "source": "final_teleop_hand_qcmd",
                "measured": False,
                "hardware_feedback": False,
                "hardware_applied": False,
            },
        ),
        _event(
            "arm_feedback",
            100,
            {"sequence": 1, "q_rad": np.full(6, 0.10).tolist(), "valid": True},
        ),
        _event(
            "control_decision",
            110,
            {
                "command_bundle_id": 1,
                "decision_timestamp_ns": 110,
                "applied_timestamp_ns": 115,
                "q_rad": np.full(11, 0.20).tolist(),
                "tracking_valid": True,
                "safety_ok": True,
                "ik_accepted": True,
                "stale_target": False,
                "hand_command_valid": True,
                "audit": {
                    "ik_q_arm_goal": np.full(6, 0.12).tolist(),
                    "raw_avp_wrist_pose": np.eye(4).tolist(),
                },
            },
        ),
        _event(
            "arm_command_sent",
            120,
            {
                "command_bundle_id": 1,
                "q_rad": np.full(6, 0.11).tolist(),
                "limiter_input_q_rad": np.full(6, 0.12).tolist(),
            },
        ),
        _event(
            "arm_feedback",
            150,
            {"sequence": 2, "q_rad": np.full(6, 0.105).tolist(), "valid": True},
        ),
        _event(
            "arm_feedback",
            200,
            {"sequence": 3, "q_rad": np.full(6, 0.12).tolist(), "valid": True},
        ),
        _event(
            "control_decision",
            210,
            {
                "command_bundle_id": 2,
                "decision_timestamp_ns": 210,
                "applied_timestamp_ns": 215,
                "q_rad": np.full(11, 0.30).tolist(),
                "tracking_valid": True,
                "safety_ok": True,
                "ik_accepted": True,
                "stale_target": False,
                "hand_command_valid": True,
                "audit": {
                    "ik_q_arm_goal": np.full(6, 0.14).tolist(),
                    "raw_avp_wrist_pose": np.eye(4).tolist(),
                },
            },
        ),
        _event(
            "arm_command_sent",
            220,
            {
                "command_bundle_id": 2,
                "q_rad": np.full(6, 0.13).tolist(),
                "limiter_input_q_rad": np.full(6, 0.14).tolist(),
            },
        ),
        _event(
            "arm_feedback",
            250,
            {"sequence": 4, "q_rad": np.full(6, 0.125).tolist(), "valid": True},
        ),
    ]
    if mutate_events is not None:
        mutate_events(events)
    (root / "arm_hand_raw_events.jsonl").write_text(
        "".join(json.dumps(event) + "\n" for event in events),
        encoding="utf-8",
    )
    manifest = {
        "schema": SCHEMA_NAME,
        "physical_dimension": PHYSICAL_DIM,
        "arm_state_source": "rm65_measured_joint_feedback",
        "arm_action_source": "real_joint_command_sent",
        "hand_state_source": "previous_command_proxy",
        "hand_initial_state_source": "serialized_command_proxy_baseline",
        "hand_action_source": "final_teleop_qcmd",
        "hand_proxy_baseline_serialized": True,
        "hand_proxy_baseline_timestamp_ns": 50,
        "hand_hardware_present": False,
        "hand_action_hardware_applied": False,
        "hand_feedback_available": False,
        "hand_feedback_valid": False,
        "representation": "absolute_joint_position",
        "unit": "rad",
        "arm_backend": "real",
        "rm65_real_hardware": True,
        "transition_count": 2,
        "rejection_count": 0,
        "rejection_counts": {},
        "camera": {
            "head_camera_present": False,
            "wrist_camera_present": False,
            "multimodal_training_ready": False,
        },
    }
    if mutate_manifest is not None:
        mutate_manifest(manifest)
    (root / "arm_hand_manifest.json").write_text(
        json.dumps(manifest), encoding="utf-8"
    )
    return root


def _reasons(report) -> set[str]:
    return {item["reason"] for item in report.failures}


def test_valid_episode_and_cli_inputs() -> None:
    with tempfile.TemporaryDirectory() as directory:
        dataset = Path(directory)
        episode = make_episode(dataset / "session" / "arm_hand_data")
        report = inspect_episode(episode)
        assert report.passed
        assert report.summary["causality"]["strict_valid"] == 2
        assert report.summary["bundles"]["canonical_complete"] == 2
        assert report.summary["rm65"]["canonical_action_raw_match"] is True
        assert report.summary["sg100"]["proxy_consistency"] is True
        assert report.summary["hand_proxy_baseline_valid"] is True
        assert report.summary["rm65_real_causal_chain_pass"] is True
        assert report.summary["result"]["pi05_multimodal_training_ready"] is False
        assert inspect_episode(episode / "arm_hand_joint17_v1.npz").passed
        assert inspect_episode(episode.parent).passed
        summary_text = format_summary(report)
        assert "FIRST TRANSITION" in summary_text
        assert "DATA CONTRACT V1: PASS" in summary_text
        assert "TRANSITION 0" in format_transition(report.transition(0))

        stdout = io.StringIO()
        with redirect_stdout(stdout):
            code = inspector_main([str(episode), "--json", "--dump-transition", "0"])
        payload = json.loads(stdout.getvalue())
        assert code == 0 and payload["result"]["data_contract_pass"]
        assert payload["transition"]["bundle_id"] == 1

        stdout = io.StringIO()
        with redirect_stdout(stdout):
            code = inspector_main([str(dataset), "--all", "--json"])
        assert code == 0 and len(json.loads(stdout.getvalue())) == 1


def test_current_collector_output_is_inspectable() -> None:
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        collector = ArmHandDataCollector(
            root, initial_hand_qcmd=np.zeros(11), started_timestamp_ns=50
        )
        collector.observe_arm_feedback(
            ArmState(
                q_rad=np.full(6, 0.10),
                qd_rad_s=np.zeros(6),
                received_monotonic_ns=100,
                sequence=1,
                controller_error=0,
                arm_status=None,
                valid=True,
            )
        )
        collector.record_control_decision(
            command_bundle_id=1,
            decision_timestamp_ns=110,
            hand_command_applied_timestamp_ns=115,
            hand_qcmd_rad=np.full(11, 0.20),
        )
        collector.ingest_arm_command_events(
            [
                ArmCommandSent(
                    q_rad=np.full(6, 0.11),
                    limiter_input_q_rad=np.full(6, 0.12),
                    sent_monotonic_ns=120,
                    source_timestamp_ns=110,
                    target_sequence=1,
                    command_bundle_id=1,
                )
            ]
        )
        collector.observe_arm_feedback(
            ArmState(
                q_rad=np.full(6, 0.105),
                qd_rad_s=np.zeros(6),
                received_monotonic_ns=150,
                sequence=2,
                controller_error=0,
                arm_status=None,
                valid=True,
            )
        )
        collector.finalize()
        report = inspect_episode(root)
        assert report.passed
        assert report.summary["raw_counts"]["arm_feedback"] == 2
        assert report.summary["raw_counts"]["arm_sent_command"] == 1
        assert report.summary["raw_counts"]["hand_command"] == 1
        assert report.summary["raw_counts"]["hand_proxy_baseline"] == 1
        assert report.summary["hand_proxy_baseline_valid"] is True
        assert report.transitions[0]["state"]["baseline_raw_match"] is True


def test_invalid_shape() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(arrays):
            arrays["state"] = arrays["state"][:, :16]

        report = inspect_episode(make_episode(Path(directory), mutate_arrays=mutate))
        assert not report.passed
        assert "INVALID_SHAPE" in _reasons(report)


def test_state_at_or_after_action() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(arrays):
            arrays["state_timestamp_ns"][0] = 120

        report = inspect_episode(make_episode(Path(directory), mutate_arrays=mutate))
        assert not report.passed
        assert "STATE_AFTER_ACTION" in _reasons(report)


def test_action_at_or_after_next_state() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(arrays):
            arrays["next_state_timestamp_ns"][0] = 120

        report = inspect_episode(make_episode(Path(directory), mutate_arrays=mutate))
        assert not report.passed
        assert "NEXT_STATE_BEFORE_ACTION" in _reasons(report)


def test_wrong_bundle_id() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(arrays):
            arrays["command_bundle_id"][0] = 99

        report = inspect_episode(make_episode(Path(directory), mutate_arrays=mutate))
        assert not report.passed
        assert "MISSING_OR_AMBIGUOUS_ARM_COMMAND" in _reasons(report)
        assert "MISSING_OR_AMBIGUOUS_HAND_COMMAND" in _reasons(report)


def test_missing_arm_command() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(events):
            events[:] = [
                event
                for event in events
                if not (
                    event["event_type"] == "arm_command_sent"
                    and event["payload"].get("command_bundle_id") == 1
                )
            ]

        report = inspect_episode(make_episode(Path(directory), mutate_events=mutate))
        assert not report.passed
        assert report.summary["bundles"]["missing_arm_command"] == 1


def test_missing_hand_command() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(events):
            events[:] = [
                event
                for event in events
                if not (
                    event["event_type"] == "control_decision"
                    and event["payload"].get("command_bundle_id") == 1
                )
            ]

        report = inspect_episode(make_episode(Path(directory), mutate_events=mutate))
        assert not report.passed
        assert report.summary["bundles"]["missing_hand_command"] == 1


def test_canonical_arm_action_mismatch() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(arrays):
            arrays["action"][0, 0] += 1e-4

        report = inspect_episode(make_episode(Path(directory), mutate_arrays=mutate))
        assert not report.passed
        assert "ARM_CANONICAL_RAW_MISMATCH" in _reasons(report)


def test_incorrect_hand_proxy() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(arrays):
            arrays["next_state"][0, 6] += 1e-4

        report = inspect_episode(make_episode(Path(directory), mutate_arrays=mutate))
        assert not report.passed
        assert "HAND_NEXT_PROXY_MISMATCH" in _reasons(report)


def test_nan_and_inf_indices() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(arrays):
            arrays["state"][0, 0] = np.nan
            arrays["action"][1, 0] = np.inf
            arrays["next_state"][1, 1] = -np.inf

        report = inspect_episode(make_episode(Path(directory), mutate_arrays=mutate))
        assert not report.passed
        quality = report.summary["data_quality"]
        assert quality["nonfinite_state_indices"] == [0]
        assert quality["nonfinite_action_indices"] == [1]
        assert quality["nonfinite_next_state_indices"] == [1]


def test_stored_valid_mask_disagrees_with_recomputed_result() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(arrays):
            arrays["state_timestamp_ns"][0] = 120
            arrays["valid_mask"] = np.ones(2, dtype=bool)

        report = inspect_episode(make_episode(Path(directory), mutate_arrays=mutate))
        assert not report.passed
        assert report.summary["data_quality"]["stored_valid_mask_consistency"] is False
        assert "STORED_FILTER_RECOMPUTE_MISMATCH" in _reasons(report)


def test_first_transition_requires_raw_pre_action_feedback() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(events):
            events[:] = [
                event
                for event in events
                if not (
                    event["event_type"] == "arm_feedback"
                    and event["master_timestamp_ns"] == 100
                )
            ]

        report = inspect_episode(make_episode(Path(directory), mutate_events=mutate))
        assert not report.passed
        assert report.summary["data_quality"]["first_transition_pre_roll"] is False
        assert "FIRST_TRANSITION_NO_VERIFIABLE_PRE_ROLL" in _reasons(report)


def test_missing_serialized_baseline_is_hard_failure() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(events):
            events[:] = [
                event for event in events
                if event["event_type"] != "hand_proxy_baseline"
            ]

        report = inspect_episode(make_episode(Path(directory), mutate_events=mutate))
        assert not report.passed
        assert "MISSING_HAND_PROXY_BASELINE" in _reasons(report)


def test_baseline_wrong_shape_is_hard_failure() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(events):
            events[0]["payload"]["hand_qcmd"] = [0.0] * 10

        report = inspect_episode(make_episode(Path(directory), mutate_events=mutate))
        assert not report.passed
        assert "INVALID_HAND_PROXY_BASELINE" in _reasons(report)


def test_baseline_nan_is_hard_failure() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(events):
            events[0]["payload"]["hand_qcmd"][0] = float("nan")

        report = inspect_episode(make_episode(Path(directory), mutate_events=mutate))
        assert not report.passed
        assert "INVALID_HAND_PROXY_BASELINE" in _reasons(report)


def test_baseline_must_precede_first_hand_action() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(events):
            events[0]["master_timestamp_ns"] = 115

        report = inspect_episode(make_episode(Path(directory), mutate_events=mutate))
        assert not report.passed
        assert "HAND_BASELINE_AFTER_ACTION" in _reasons(report)


def test_baseline_must_match_first_canonical_hand_state() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(arrays):
            arrays["state"][0, 6] = 0.1

        report = inspect_episode(make_episode(Path(directory), mutate_arrays=mutate))
        assert not report.passed
        assert "HAND_BASELINE_STATE_MISMATCH" in _reasons(report)


def test_legacy_episode_without_baseline_does_not_crash_or_pass() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate_events(events):
            events[:] = [
                event for event in events
                if event["event_type"] != "hand_proxy_baseline"
            ]

        def mutate_manifest(manifest):
            for key in (
                "hand_proxy_baseline_serialized",
                "hand_proxy_baseline_timestamp_ns",
                "hand_initial_state_source",
                "hand_action_hardware_applied",
                "hand_hardware_present",
            ):
                manifest.pop(key, None)
            manifest["hand_action_source"] = "final_hand_qcmd"

        report = inspect_episode(
            make_episode(
                Path(directory),
                mutate_events=mutate_events,
                mutate_manifest=mutate_manifest,
            )
        )
        assert not report.passed
        assert report.summary["sg100"]["initial_proxy_baseline"] == (
            "NOT SERIALIZED — LEGACY EPISODE"
        )
        assert report.transitions[0]["validation"][
            "hand_state_initial_baseline_verifiable"
        ] is None


def test_rm65_only_hardware_result_ignores_absent_sg100_and_cameras() -> None:
    with tempfile.TemporaryDirectory() as directory:
        report = inspect_episode(make_episode(Path(directory)))
        assert report.summary["rm65_real_causal_chain_pass"] is True
        assert report.summary["sg100_hardware_present"] is False
        assert report.summary["head_camera_present"] is False
        assert report.summary["wrist_camera_present"] is False
        assert report.summary["full_arm_hand_hardware_validated"] is False


def test_rm65_causal_error_fails_rm65_only_result() -> None:
    with tempfile.TemporaryDirectory() as directory:
        def mutate(arrays):
            arrays["state_timestamp_ns"][0] = 120

        report = inspect_episode(make_episode(Path(directory), mutate_arrays=mutate))
        assert report.summary["rm65_real_causal_chain_pass"] is False


def test_absent_hand_hardware_is_never_reported_applied() -> None:
    with tempfile.TemporaryDirectory() as directory:
        report = inspect_episode(make_episode(Path(directory)))
        assert report.summary["sg100"]["hardware_present"] is False
        assert report.summary["sg100"]["hardware_execution"] is False
        assert report.summary["sg100"]["action_hardware_applied"] is False


def main() -> None:
    tests = [value for name, value in globals().items() if name.startswith("test_")]
    for test in tests:
        test()
        print(f"{test.__name__}: PASS")
    print("DATA COLLECTION INSPECTOR: PASS")


if __name__ == "__main__":
    main()
