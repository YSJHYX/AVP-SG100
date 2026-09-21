"""Offline tests for the RM65/SG100 communication-latency benchmark."""

from __future__ import annotations

import csv
import json
import threading
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from scripts import communication_latency_benchmark as benchmark


def _rtt_row(latency_ms: float | None, *, timeout: bool = False) -> dict:
    return {
        "latency_ms": latency_ms,
        "valid": latency_ms is not None,
        "timeout": timeout,
    }


def test_workbook_acceptance_thresholds_are_encoded_exactly() -> None:
    criteria = benchmark.CRITERIA
    assert criteria.rtt_p95_ms == 50.0
    assert criteria.rtt_p99_ms == 70.0
    assert criteria.rtt_max_ms == 100.0
    assert criteria.feedback_min_hz == 20.0
    assert criteria.feedback_interval_p99_ms == 50.0
    assert criteria.feedback_interval_max_ms == 100.0
    assert criteria.command_sync_p95_ms == 50.0
    assert criteria.command_sync_max_ms == 100.0
    assert criteria.max_error_rate == 0.001
    assert criteria.interruption_timeout_ms == 100.0
    assert criteria.command_block_deadline_ms == 20.0


def test_timeout_is_retained_and_integrity_uses_full_denominator() -> None:
    rows = [_rtt_row(10.0) for _ in range(999)]
    rows.append(_rtt_row(None, timeout=True))
    summary = benchmark.evaluate_rtt(rows)
    assert summary["total_requests"] == 1000
    assert summary["valid_responses"] == 999
    assert summary["timeout_count"] == 1
    assert summary["latency_ms"]["count"] == 999

    at_limit = benchmark.evaluate_integrity(
        valid_frames=1000,
        invalid_frames=1,
        status_frames=1000,
        crc_errors=1,
        request_count=1000,
        request_timeouts=1,
        request_errors=0,
        estimated_sequence_gaps=1,
        device_error_frames=0,
        control_failure_events=0,
    )
    assert at_limit["pass"] is True

    over_limit = benchmark.evaluate_integrity(
        valid_frames=1000,
        invalid_frames=2,
        status_frames=1000,
        crc_errors=2,
        request_count=1000,
        request_timeouts=2,
        request_errors=0,
        estimated_sequence_gaps=2,
        device_error_frames=0,
        control_failure_events=0,
    )
    assert over_limit["pass"] is False


def test_feedback_reports_interval_and_host_sequence_gap() -> None:
    rows = [
        {"received_ns": 1_000_000_000, "sequence": 10, "valid": True},
        {"received_ns": 1_050_000_000, "sequence": 11, "valid": True},
        {"received_ns": 1_151_000_000, "sequence": 13, "valid": True},
    ]
    summary = benchmark.evaluate_feedback(rows)
    assert summary["interval_ms"]["max"] == pytest.approx(101.0)
    assert summary["estimated_host_callback_sequence_gaps"] == 1
    assert summary["pass"] is False

    samples = tuple(
        benchmark.ObservedHandFeedback(
            received_ns=row["received_ns"],
            sequence=row["sequence"],
            valid=True,
            joint_modes=(benchmark.MODE_JOINT_POSITION,) * benchmark.HAND_DOF,
        )
        for row in rows
    )
    anomalies = benchmark.feedback_anomaly_rows("run-1", samples)
    assert len(anomalies) == 1
    assert "feedback_interval_over_50_ms" in anomalies[0]["error"]
    assert "host_callback_sequence_gap" in anomalies[0]["error"]


def test_command_sync_is_matched_by_bundle_and_thresholded() -> None:
    rows = [
        {"valid": True, "skew_ms": 5.0},
        {"valid": True, "skew_ms": 10.0},
        {"valid": True, "skew_ms": 100.0},
    ]
    result = benchmark.evaluate_command_sync(rows)
    assert result["absolute_send_skew_ms"]["max"] == 100.0
    assert result["acceptance"]["max_le_100_ms"] is True
    assert result["acceptance"]["p95_le_50_ms"] is False


def test_interruption_acceptance_checks_detection_gate_and_replay() -> None:
    last = 1_000_000_000
    passing = benchmark.evaluate_interruption({
        "last_valid_ns": last,
        "timeout_trigger_ns": last + 100_000_000,
        "commands_forbidden_ns": last + 119_000_000,
        "commands_after_gate": 0,
        "historical_targets_replayed": 0,
    })
    assert passing["pass"] is True

    replayed = benchmark.evaluate_interruption({
        "last_valid_ns": last,
        "timeout_trigger_ns": last + 100_000_000,
        "commands_forbidden_ns": last + 121_000_000,
        "commands_after_gate": 1,
        "historical_targets_replayed": 1,
    })
    assert replayed["pass"] is False

    incomplete_gate = benchmark.evaluate_interruption({
        "last_valid_ns": last,
        "timeout_trigger_ns": last + 100_000_000,
        "commands_forbidden_ns": last + 101_000_000,
        "commands_after_gate": 0,
        "post_gate_attempts": 3,
        "post_gate_attempt_rejections": 2,
        "historical_targets_replayed": 0,
    })
    assert incomplete_gate["pass"] is False


def test_closed_rig_command_gate_rejects_before_hardware_calls() -> None:
    class ForbiddenHardware:
        def __getattr__(self, name):
            raise AssertionError(f"closed gate touched hardware attribute {name}")

    rig = benchmark.RealCommunicationRig.__new__(
        benchmark.RealCommunicationRig
    )
    rig.args = SimpleNamespace(motion_frequency_hz=0.1)
    rig.arm_center_rad = np.zeros(benchmark.ARM_DOF)
    rig.hand_center_rad = np.zeros(benchmark.HAND_DOF)
    rig._active_profile = benchmark.PROFILE_STATIONARY
    rig._command_gate_lock = threading.Lock()
    rig._command_gate_open = False
    rig.command_gate_rejections = 0
    rig.arm = ForbiddenHardware()
    rig.hand = ForbiddenHardware()

    sent_ns, error, _ = rig.send_bundle(elapsed_s=0.0, bundle_id=1)
    assert sent_ns is None
    assert error == "command_gate_closed"
    assert rig.command_gate_rejections == 1


def test_sample_coverage_prevents_small_good_subset_from_passing() -> None:
    complete = benchmark.evaluate_sample_coverage(
        configured_duration_s=300.0,
        actual_duration_s=300.0,
        query_hz=20.0,
        command_hz=20.0,
        rm65_query_count=6000,
        sg100_query_count=6000,
        feedback_count=6000,
        command_bundle_count=6000,
    )
    assert complete["pass"] is True

    truncated = benchmark.evaluate_sample_coverage(
        configured_duration_s=300.0,
        actual_duration_s=300.0,
        query_hz=20.0,
        command_hz=20.0,
        rm65_query_count=100,
        sg100_query_count=100,
        feedback_count=100,
        command_bundle_count=100,
    )
    assert truncated["pass"] is False


def test_initial_pose_hand_angles_preserve_canonical_order(tmp_path: Path) -> None:
    angles = {
        name: float(index)
        for index, name in enumerate(reversed(benchmark.HAND_JOINT_NAMES))
    }
    config = tmp_path / "pose.yaml"
    config.write_text(
        "hand:\n  angles_deg:\n"
        + "".join(f"    {name}: {value}\n" for name, value in angles.items()),
        encoding="utf-8",
    )
    loaded = benchmark.load_hand_center_deg(str(config))
    assert loaded is not None
    assert loaded.tolist() == [angles[name] for name in benchmark.HAND_JOINT_NAMES]


def test_parser_restricts_real_hand_motion_to_validated_j6() -> None:
    with pytest.raises(SystemExit):
        benchmark.parse_args(["--hand-joint", "middle_j1"])
    parsed = benchmark.parse_args(["--hand-joint", "index_j3"])
    assert parsed.hand_joint == "index_j3"


def test_parser_caps_benchmark_motion_envelope() -> None:
    with pytest.raises(SystemExit):
        benchmark.parse_args(["--arm-amplitude-deg", "3.1"])
    with pytest.raises(SystemExit):
        benchmark.parse_args(["--hand-amplitude-deg", "0.6"])
    with pytest.raises(SystemExit):
        benchmark.parse_args(["--motion-frequency-hz", "0.6"])


def test_timed_rm65_backend_separates_lock_wait_and_sdk_rtt() -> None:
    class FakeSDK:
        arm_dof = 6

        def rm_get_joint_degree(self):
            return 0, [0.0] * 6

        def rm_get_current_arm_state(self):
            return 0, {
                "joint": [1.0] * 6,
                "pose": [0.0] * 6,
                "err": {"err": ["0"]},
            }

    backend = benchmark.TimedRM65Backend(
        "127.0.0.1", sdk=FakeSDK(), on_event=lambda *_: None
    )
    backend.connect()
    backend._poll_feedback_once()
    samples = backend.drain_latency_query_samples()
    assert len(samples) == 1
    sample = samples[0]
    assert sample.valid is True
    assert sample.timeout is False
    assert sample.request_queued_ns <= sample.request_started_ns
    assert sample.request_started_ns <= sample.response_received_ns
    assert sample.latency_ms is not None and sample.latency_ms >= 0.0
    assert sample.sdk_lock_wait_ms >= 0.0
    assert backend.joint_state().sequence == 2
    backend.disconnect()


def test_timed_rm65_feedback_loop_uses_absolute_query_cadence(
    monkeypatch,
) -> None:
    clock = [0.0]
    starts = []

    class FakeShutdown:
        def is_set(self):
            return len(starts) >= 4

        def wait(self, delay_s):
            clock[0] += delay_s

    backend = benchmark.TimedRM65Backend.__new__(benchmark.TimedRM65Backend)
    backend._feedback_poll_hz = 20.0
    backend._shutdown = FakeShutdown()
    backend._on_event = lambda *_: None

    def poll_once():
        starts.append(clock[0])
        clock[0] += 0.008

    backend._poll_feedback_once = poll_once
    monkeypatch.setattr(benchmark.time, "monotonic", lambda: clock[0])
    backend._feedback_loop()
    assert starts == pytest.approx([0.0, 0.05, 0.10, 0.15])


def test_default_main_is_no_hardware_dry_run(monkeypatch, tmp_path: Path) -> None:
    def forbidden(*args, **kwargs):
        raise AssertionError("dry run must not construct a hardware rig")

    monkeypatch.setattr(benchmark, "RealCommunicationRig", forbidden)
    output = tmp_path / "must-not-exist"
    assert benchmark.main(["--output-dir", str(output)]) == 0
    assert not output.exists()


def test_short_mock_run_is_screening_not_full_acceptance() -> None:
    args = benchmark.parse_args([
        "--simulate",
        "--profiles", "stationary",
        "--repeats", "1",
        "--warmup-s", "1",
        "--duration-s", "1",
        "--interruption-repeats", "1",
    ])
    run, _ = benchmark.build_mock_run(benchmark.PROFILE_STATIONARY, 1)
    interruptions = benchmark.build_mock_interruptions(1)
    report = benchmark.assemble_report(args, [run], interruptions)
    assert report["overall_result"] == "SCREENING_PASS_NOT_FULL_ACCEPTANCE"
    assert report["summary"]["each_run_judged_independently"] is True
    assert report["summary"]["combined_average_used_for_acceptance"] is False


def test_runtime_failure_is_persisted_in_report(monkeypatch, tmp_path: Path) -> None:
    args = benchmark.parse_args([
        "--simulate",
        "--profiles", "stationary",
        "--repeats", "1",
        "--warmup-s", "1",
        "--duration-s", "1",
        "--output-dir", str(tmp_path),
    ])

    def fail(*args, **kwargs):
        raise RuntimeError("synthetic transport failure")

    monkeypatch.setattr(benchmark, "build_mock_run", fail)
    code, run_dir = benchmark.run(args)
    assert code == 1
    report = json.loads((run_dir / "report.json").read_text(encoding="utf-8"))
    assert report["overall_result"] == "FAIL"
    assert report["fatal_error"]["type"] == "RuntimeError"
    assert "synthetic transport failure" in report["fatal_error"]["message"]


def test_anomaly_jsonl_contains_failures_but_not_valid_rows(tmp_path: Path) -> None:
    path = tmp_path / "anomalies.jsonl"
    benchmark.write_anomalies_jsonl(path, [
        {"sample_type": "rm65_rtt", "valid": True},
        {"sample_type": "sg100_rtt_proxy", "valid": False, "error": "timeout"},
        {"sample_type": "crc_error", "valid": False, "error": "crc"},
    ])
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert [row["sample_type"] for row in rows] == [
        "sg100_rtt_proxy",
        "crc_error",
    ]


def test_incremental_writers_keep_header_and_prior_anomalies(
    tmp_path: Path,
) -> None:
    csv_path = tmp_path / "raw.csv"
    benchmark.write_raw_csv(csv_path, [])
    benchmark.append_raw_csv(csv_path, [
        {"sample_type": "first", "valid": True},
    ])
    benchmark.append_raw_csv(csv_path, [
        {"sample_type": "second", "valid": False, "error": "failure"},
    ])
    with csv_path.open(encoding="utf-8", newline="") as stream:
        csv_rows = list(csv.DictReader(stream))
    assert [row["sample_type"] for row in csv_rows] == ["first", "second"]

    anomaly_path = tmp_path / "anomalies.jsonl"
    benchmark.write_anomalies_jsonl(anomaly_path, [])
    benchmark.append_anomalies_jsonl(anomaly_path, [
        {"sample_type": "first", "valid": False},
    ])
    benchmark.append_anomalies_jsonl(anomaly_path, [
        {"sample_type": "second", "valid": False},
    ])
    anomaly_rows = [
        json.loads(line)
        for line in anomaly_path.read_text(encoding="utf-8").splitlines()
    ]
    assert [row["sample_type"] for row in anomaly_rows] == ["first", "second"]


def test_skipping_interruptions_is_incomplete_not_a_metric_failure() -> None:
    args = benchmark.parse_args([
        "--simulate",
        "--profiles", "all",
        "--skip-interruptions",
    ])
    runs = [
        benchmark.build_mock_run(profile, repeat)[0]
        for profile in benchmark.REQUIRED_PROFILES
        for repeat in range(1, 4)
    ]
    report = benchmark.assemble_report(args, runs, [])
    assert report["overall_result"] == "SCREENING_PASS_NOT_FULL_ACCEPTANCE"
