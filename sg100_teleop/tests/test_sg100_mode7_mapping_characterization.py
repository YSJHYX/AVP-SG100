"""Software-only safety and evidence tests for passive MODE7 F_fb mapping."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from sg100_teleop.robot.sg100_driver import (
    HAND_DOF,
    MODE_JOINT_POSITION,
    SG100HandFeedback,
    SG100HardwareMap,
)
from sg100_teleop.tools import sg100_mode7_mapping_characterization as mapping


ROOT = Path(__file__).resolve().parents[2]
# KNOWN_STATIC_RUNS — REMOVED 2026-09-04 with the deleted
# test_static_snapshots_refine_j2_j5_j10_without_selecting_mapping
# (recording/sg100_mode7_probe_* artifacts deleted by user decision).


def _frame(sequence: int, values, *, phase: str = "test") -> mapping.MappingFrame:
    return mapping.MappingFrame(
        phase=phase,
        sequence=sequence,
        received_monotonic_ns=sequence * 10_000_000,
        returned_modes=(MODE_JOINT_POSITION,) * HAND_DOF,
        sdk_callback_positions_deg=tuple(float(value) for value in values),
    )


def _frames(values, *, phase="test", start=1, count=8):
    return [
        _frame(start + index, values, phase=phase)
        for index in range(count)
    ]


# test_static_snapshots_refine_j2_j5_j10_without_selecting_mapping
# — REMOVED 2026-09-04: 依赖的 KNOWN_STATIC_RUNS（sg100_mode7_probe_*
# 五个真机 run）已按用户决定从工作区删除。删除原因 = artifact 缺失，
# 非行为回归。


@pytest.mark.parametrize(
    ("joint_index", "baseline_value", "endpoint_delta", "canonical_sign", "gain"),
    [
        (1, 18.1, -20.0, -1, "+1"),
        (1, 18.1, +20.0, -1, "-1"),
        (4, -0.428, +20.0, +1, "+1"),
        (9, -3.955, +20.0, +1, "+1"),
    ],
)
def test_priority_joint_synthetic_trials_only_produce_candidates(
    joint_index, baseline_value, endpoint_delta, canonical_sign, gain
):
    hardware_map = SG100HardwareMap()
    baseline = np.zeros(HAND_DOF)
    baseline[joint_index] = baseline_value
    endpoint = baseline.copy()
    endpoint[joint_index] += endpoint_delta
    returned = baseline.copy()
    returned[joint_index] += 0.2
    result = mapping.analyze_manual_excursion(
        _frames(baseline, phase="canonical_zero_reference_hold"),
        _frames(endpoint, phase="endpoint_hold", start=20),
        expected_joint_index=joint_index,
        canonical_delta_sign=canonical_sign,
        return_frames=_frames(returned, phase="return_hold", start=40),
        expected_mode7_channel_id=hardware_map.hardware_joint_ids[joint_index],
        zero_reference_confirmed=True,
        measured_excursion_deg=20.0,
        measurement_method="operator_estimate",
        canonical_range_deg=mapping.canonical_ranges_deg(hardware_map)[joint_index],
    )
    assert result["identity_candidate"] == "EXPECTED_CHANNEL_DOMINANT"
    assert result["feedback_sign_gain_candidate"] == gain
    assert result["zero_mapping_evidence"] == (
        "OPERATOR_REFERENCED_REPEATABLE_CANDIDATE"
    )
    assert result["rough_scale_evidence"] == (
        "CANDIDATE_ONLY_PENDING_METROLOGY_REVIEW"
    )
    assert result["F_fb_verdict"] == "PENDING_HUMAN_REVIEW"
    assert result["F_cmd_changed_or_validated"] is False
    assert result["canonical_valid"] is False


def test_ambiguous_identity_blocks_sign_candidate():
    baseline = np.zeros(HAND_DOF)
    endpoint = baseline.copy()
    endpoint[1] = -20.0
    endpoint[2] = -15.0
    result = mapping.analyze_manual_excursion(
        _frames(baseline),
        _frames(endpoint, start=20),
        expected_joint_index=1,
        canonical_delta_sign=-1,
        return_frames=_frames(baseline, start=40),
        zero_reference_confirmed=True,
    )
    assert result["identity_candidate"] == "AMBIGUOUS_CHANNEL_DOMINANCE"
    assert result["feedback_sign_gain_candidate"] == (
        "NOT_EVALUATED_IDENTITY_UNRESOLVED"
    )


def test_single_frame_spike_cannot_establish_identity_or_sign():
    baseline = _frames(np.zeros(HAND_DOF), count=20)
    movement = _frames(np.zeros(HAND_DOF), start=30, count=20)
    spike = np.zeros(HAND_DOF)
    spike[1] = -100.0
    movement[0] = _frame(30, spike)
    result = mapping.analyze_manual_excursion(
        baseline,
        movement,
        expected_joint_index=1,
        canonical_delta_sign=-1,
    )
    assert result["identity_candidate"] == "INSUFFICIENT_EXCURSION"
    assert result["feedback_sign_gain_candidate"] == (
        "NOT_EVALUATED_IDENTITY_UNRESOLVED"
    )


def test_no_physical_zero_or_scale_reference_stays_not_evaluated():
    baseline = np.zeros(HAND_DOF)
    endpoint = baseline.copy()
    endpoint[4] = 20.0
    result = mapping.analyze_manual_excursion(
        _frames(baseline),
        _frames(endpoint, start=20),
        expected_joint_index=4,
        canonical_delta_sign=1,
        return_frames=_frames(baseline, start=40),
    )
    assert result["feedback_value_at_canonical_zero_candidate_deg"] is None
    assert result["zero_mapping_evidence"] == (
        "NOT_EVALUATED_NO_CANONICAL_ZERO_REFERENCE"
    )
    assert result["rough_scale_evidence"] == (
        "NOT_EVALUATED_NO_PHYSICAL_MEASUREMENT"
    )


@pytest.mark.parametrize("value", [float("nan"), float("inf"), 0.0, -1.0, 31.0])
def test_invalid_durations_are_rejected(value):
    with pytest.raises(ValueError):
        mapping.validate_duration("phase", value)


def test_preflight_phrase_mismatch_has_zero_hardware_access(tmp_path, monkeypatch):
    calls = []

    def forbidden(*args, **kwargs):
        calls.append((args, kwargs))
        raise AssertionError("driver must not be constructed")

    monkeypatch.setattr(mapping, "_probe_driver", forbidden)
    code = mapping.run_characterization(
        "stub",
        0,
        Path("."),
        1,
        tmp_path / "not_created",
        measured_excursion_deg=20.0,
        measurement_method="operator_estimate",
        confirm_fn=lambda prompt: "WRONG",
    )
    assert code == 4
    assert calls == []
    assert not (tmp_path / "not_created").exists()


def test_artifact_records_modes_and_refuses_overwrite(tmp_path):
    frame = _frame(1, np.arange(HAND_DOF), phase="return_hold")
    mapping._write_artifacts(tmp_path, {"canonical_valid": False}, [frame])
    header = (tmp_path / "sdk_callback_samples.csv").read_text(
        encoding="utf-8"
    ).splitlines()[0]
    assert "J1_returned_mode" in header
    assert "J11_sdk_callback_deg" in header
    with pytest.raises(FileExistsError):
        mapping._write_artifacts(tmp_path, {}, [frame])


class _TrialDriver:
    def __init__(self, *, cleanup_accepted: bool):
        self.cleanup_accepted = cleanup_accepted
        self.hand_control_calls = 0
        self.send_count = 0
        self.sequence = 100
        self.lifecycle = []
        self.fault_active = False

    @property
    def diagnostics(self):
        return {"hand_control_calls": self.hand_control_calls}

    def connect(self):
        self.lifecycle.append("connect")

    def send_mode7_feedback_probe(self):
        self.send_count += 1
        self.hand_control_calls += 1
        return SimpleNamespace(sent_monotonic_ns=1)

    def hold_passive(self):
        self.lifecycle.append("hold_passive")
        return self.cleanup_accepted

    def read_feedback(self):
        self.sequence += 1
        return SG100HandFeedback(
            q_rad=np.zeros(HAND_DOF),
            received_monotonic_ns=self.sequence * 10_000_000,
            sequence=self.sequence,
            valid=True,
            raw_positions=np.zeros(HAND_DOF),
            joint_modes=(0,) * HAND_DOF,
        )

    def stop(self):
        self.lifecycle.append("stop")

    def disconnect(self):
        self.lifecycle.append("disconnect")


def test_cleanup_rejection_overrides_success_and_returns_nonzero(tmp_path, monkeypatch):
    driver = _TrialDriver(cleanup_accepted=False)
    monkeypatch.setattr(mapping, "_probe_driver", lambda *args, **kwargs: driver)
    monkeypatch.setattr(mapping, "run_phase_a", lambda *args, **kwargs: {})
    tracker_frame = SimpleNamespace(
        sequence=1,
        timestamp_ns=1,
        modes=(MODE_JOINT_POSITION,) * HAND_DOF,
        positions_deg=(0.0,) * HAND_DOF,
    )
    tracker = SimpleNamespace(
        state=mapping.STATE_MODE7_STABLE,
        observation_complete=True,
        frames=[tracker_frame],
        summary=lambda: {"state": mapping.STATE_MODE7_STABLE},
    )
    monkeypatch.setattr(mapping, "_observe_transition", lambda *args, **kwargs: tracker)
    sequence = iter(range(10, 1000))

    def fake_capture(driver_arg, duration, phase, **kwargs):
        values = np.zeros(HAND_DOF)
        if phase in ("outbound_move", "endpoint_hold"):
            values[1] = -20.0
        frames = [
            _frame(next(sequence), values, phase=phase)
            for _ in range(8)
        ]
        sink = kwargs.get("artifact_sink")
        if sink is not None:
            sink.extend(frames)
        return frames

    monkeypatch.setattr(mapping, "capture_mode7_window", fake_capture)
    answers = iter([
        "ZERO J2 thumb_j2",
        "ARM J2 thumb_j2",
        "n",
        "ONLY J2",
        "RETURNED J2",
    ])
    artifact = tmp_path / "cleanup_fail"
    code = mapping.run_characterization(
        "stub",
        0,
        Path("."),
        1,
        artifact,
        measured_excursion_deg=20.0,
        measurement_method="operator_estimate",
        confirm_fn=lambda prompt: next(answers),
    )
    assert code == 6
    assert driver.send_count == 1
    summary = json.loads(
        (artifact / "summary.json").read_text(encoding="utf-8")
    )
    assert summary["status"] == "CLEANUP_FAIL"
    assert summary["pre_cleanup_status"] == (
        "PASSIVE_EVIDENCE_CAPTURED_MAPPING_PENDING"
    )
    assert summary["cleanup"]["mode_null_command_accepted"] is False
    assert summary["canonical_valid"] is False

