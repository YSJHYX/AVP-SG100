#!/usr/bin/env python3
"""Passive SG100 MODE7 feedback-to-canonical mapping characterization.

PREPARED ONLY. This tool is the next hardware stage; it is not an active
target-motion test. One run observes one manually moved physical DOF:

1. establish the proven-safe MODE_NULL/realtime baseline;
2. submit exactly one full-hand MODE7, position=0, PID=0, output=0 frame via
   SG100SDKDriver.send_mode7_feedback_probe();
3. verify all 11 returned modes become and remain MODE7;
4. record the vendor SDK callback MODE7 Joint_Status.position while the
   operator manually moves the selected DOF in the stated canonical direction,
   holds the endpoint, and returns to an explicitly confirmed physical zero;
5. restore MODE_NULL on every exit and require fresh all-mode0 RX evidence.

It never sends a later target, performs homing/zeroing, derives a startup
offset from an arbitrary startup pose, applies the outbound hardware map, or
marks canonical feedback valid. Any operator-referenced zero, sign, identity,
or scale result remains a candidate for human review. The output is evidence
for F_fb only. It does not validate or modify F_cmd.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from sg100_teleop.robot.sg100_driver import (
    DEFAULT_HARDWARE_MAP_PATH,
    DEFAULT_SEMANTICS_PATH,
    HAND_DOF,
    MODE_JOINT_POSITION,
    SG100HardwareMap,
)
from sg100_teleop.robot.sg100_sdk_driver import (
    DEFAULT_BAUDRATE,
    DEFAULT_SDK_DIR,
)
from sg100_teleop.tools.sg100_mode7_feedback_probe import (
    MIN_MODE7_FRAMES,
    MULTI_TURN_ABS_DEG,
    STATE_MODE7_STABLE,
    Mode7ProbeHardFail,
    _observe_transition,
    _probe_driver,
    canonical_ranges_deg,
    format_preflight_table,
    build_full_hand_mode7_control,
    run_phase_a,
)


SCHEMA = "sg100_mode7_passive_mapping_characterization_v2"
STATIC_REVIEW_SCHEMA = "sg100_mode7_static_snapshot_review_v1"
MIN_EXCURSION_DEG = 5.0
MIN_PHASE_DURATION_S = 0.1
MAX_PHASE_DURATION_S = 30.0
DEFAULT_BASELINE_S = 0.5
DEFAULT_LEAD_IN_S = 2.0
DEFAULT_MOVEMENT_WINDOW_S = 6.0
DEFAULT_ENDPOINT_HOLD_S = 0.75
DEFAULT_RETURN_WINDOW_S = 4.0
DEFAULT_RETURN_HOLD_S = 0.75
TRANSPORT_STALL_S = 0.1
MAX_STATIONARY_HOLD_SPAN_DEG = 1.0
MIN_FEEDBACK_RATE_HZ = 50.0
RETURN_TO_ZERO_TOLERANCE_DEG = 2.0
MIN_CHANNEL_DOMINANCE_RATIO = 2.0
MODE_NULL_RESTORE_TIMEOUT_S = 0.5
PRIORITY_JOINT_INDICES = (1, 4, 9)  # J2, J5, J10

# One deliberately chosen canonical direction per physical DOF. These are
# canonical semantics, not SDK command-range directions.
CANONICAL_MOVEMENTS = (
    (+1, "rotate thumb opposition toward palm"),
    (-1, "rotate thumb abduction outward and downward"),
    (+1, "flex thumb distal joint toward palm"),
    (+1, "adduct index base toward thumb"),
    (+1, "flex index joint toward palm"),
    (+1, "flex index distal joint toward palm"),
    (-1, "flex middle base joint toward palm"),
    (+1, "flex middle distal joint toward palm"),
    (-1, "rotate little base joint outward and downward"),
    (+1, "flex little joint toward palm"),
    (+1, "flex little distal joint toward palm"),
)

# These are human-recognizable physical landmarks. They are never inferred
# from the startup callback value and never written into a production map.
CANONICAL_ZERO_REFERENCES = (
    "thumb opposition at the model's neutral boundary",
    "thumb abduction at neutral, before outward/downward rotation",
    "thumb distal joint at the model's neutral reference",
    "index base at neutral, before adduction toward the thumb",
    "index flexion joint straight/unflexed",
    "index distal joint at the model's neutral reference",
    "middle base joint straight/unflexed",
    "middle distal joint at the model's neutral reference",
    "little base at neutral, before outward/downward rotation",
    "little flexion joint straight/unflexed",
    "little distal joint at the model's neutral reference",
)

PRIORITY_QUESTIONS = {
    1: (
        "Repeated +18 to +19 deg static baseline versus canonical <= 0; "
        "distinguish sign, physical-zero value, and channel identity."
    ),
    4: (
        "One -0.428 deg boundary miss but later static snapshots are in "
        "range; test repeatable physical zero and positive flexion sign."
    ),
    9: (
        "Unlabelled snapshots cross zero (-3.955 to +24.027 deg); test "
        "channel identity, positive flexion sign, and repeatable zero."
    ),
}


@dataclasses.dataclass(frozen=True)
class MappingFrame:
    phase: str
    sequence: int
    received_monotonic_ns: int
    returned_modes: tuple[int, ...]
    sdk_callback_positions_deg: tuple[float, ...]

    @property
    def raw_mode7_positions_deg(self) -> tuple[float, ...]:
        """Backward-compatible alias; values are not wire/raw encoders."""

        return self.sdk_callback_positions_deg


def extract_mode7_raw_degrees(feedback) -> np.ndarray:
    """Return SDK-callback degrees only when every returned mode is MODE7.

    ``raw_positions`` is the value exposed at the Python driver boundary
    after the vendor SDK callback has rendered its configured user-angle
    coordinate. It is raw with respect to *our* unvalidated F_fb mapping, but
    it is not a wire encoder value.
    """

    if feedback is None or feedback.joint_modes is None:
        raise Mode7ProbeHardFail("missing mode-bearing feedback")
    modes = tuple(int(mode) for mode in feedback.joint_modes)
    if modes != (MODE_JOINT_POSITION,) * HAND_DOF:
        raise Mode7ProbeHardFail(
            f"unexpected MODE7 loss during passive mapping: {list(modes)}"
        )
    if feedback.raw_positions is None:
        raise Mode7ProbeHardFail("missing raw MODE7 callback positions")
    positions_deg = np.rad2deg(
        np.asarray(feedback.raw_positions, dtype=np.float64)
    )
    if positions_deg.shape != (HAND_DOF,):
        raise Mode7ProbeHardFail("malformed raw MODE7 callback positions")
    if not np.all(np.isfinite(positions_deg)):
        raise Mode7ProbeHardFail("non-finite raw MODE7 callback positions")
    if np.any(np.abs(positions_deg) > MULTI_TURN_ABS_DEG):
        raise Mode7ProbeHardFail("multi-turn value in MODE7 mapping window")
    return positions_deg


def _mapping_frame(feedback, phase: str) -> MappingFrame:
    """Build an auditable frame before applying the all-MODE7 gate."""

    if feedback is None or feedback.joint_modes is None:
        raise Mode7ProbeHardFail("missing mode-bearing feedback")
    modes = tuple(int(mode) for mode in feedback.joint_modes)
    if len(modes) != HAND_DOF:
        raise Mode7ProbeHardFail("malformed returned mode vector")
    if feedback.raw_positions is None:
        raise Mode7ProbeHardFail("missing SDK callback positions")
    positions_deg = np.rad2deg(
        np.asarray(feedback.raw_positions, dtype=np.float64)
    )
    if positions_deg.shape != (HAND_DOF,):
        raise Mode7ProbeHardFail("malformed SDK callback positions")
    if not np.all(np.isfinite(positions_deg)):
        raise Mode7ProbeHardFail("non-finite SDK callback positions")
    return MappingFrame(
        phase=phase,
        sequence=int(feedback.sequence),
        received_monotonic_ns=int(feedback.received_monotonic_ns),
        returned_modes=modes,
        sdk_callback_positions_deg=tuple(float(v) for v in positions_deg),
    )


def capture_mode7_window(
    driver,
    duration_s: float,
    phase: str,
    *,
    after_sequence: int | None = None,
    artifact_sink: list[MappingFrame] | None = None,
) -> list[MappingFrame]:
    """Passively capture fresh all-MODE7 feedback; any fault/mode loss fails."""

    deadline = time.monotonic() + float(duration_s)
    last_progress = time.monotonic()
    last_sequence = after_sequence
    frames: list[MappingFrame] = []
    while time.monotonic() < deadline:
        if driver.fault_active:
            raise Mode7ProbeHardFail(
                f"fault callback during passive mapping phase {phase}"
            )
        feedback = driver.read_feedback()
        if feedback is not None and feedback.sequence != last_sequence:
            frame = _mapping_frame(feedback, phase)
            if artifact_sink is not None:
                artifact_sink.append(frame)
            positions_deg = np.asarray(
                frame.sdk_callback_positions_deg, dtype=np.float64
            )
            last_sequence = int(feedback.sequence)
            last_progress = time.monotonic()
            if frame.returned_modes != (MODE_JOINT_POSITION,) * HAND_DOF:
                raise Mode7ProbeHardFail(
                    "unexpected MODE7 loss during passive mapping: "
                    f"{list(frame.returned_modes)}"
                )
            if np.any(np.abs(positions_deg) > MULTI_TURN_ABS_DEG):
                raise Mode7ProbeHardFail(
                    "multi-turn value in MODE7 mapping window"
                )
            frames.append(frame)
        if time.monotonic() - last_progress > TRANSPORT_STALL_S:
            raise Mode7ProbeHardFail(
                f"fresh feedback stalled during passive mapping phase {phase}"
            )
        time.sleep(0.001)
    if len(frames) < MIN_MODE7_FRAMES:
        raise Mode7ProbeHardFail(
            f"only {len(frames)} fresh MODE7 samples during {phase}"
        )
    return frames


def _frame_matrix(frames: list[MappingFrame], phase: str) -> np.ndarray:
    if len(frames) < MIN_MODE7_FRAMES:
        raise ValueError(
            f"{phase} requires at least {MIN_MODE7_FRAMES} fresh frames"
        )
    if any(
        frame.returned_modes != (MODE_JOINT_POSITION,) * HAND_DOF
        for frame in frames
    ):
        raise ValueError(f"{phase} contains a non-MODE7 sample")
    values = np.asarray(
        [frame.sdk_callback_positions_deg for frame in frames],
        dtype=np.float64,
    )
    if values.shape[1:] != (HAND_DOF,) or not np.all(np.isfinite(values)):
        raise ValueError(f"{phase} contains malformed callback values")
    return values


def summarize_hold(frames: list[MappingFrame], phase: str) -> dict:
    """Summarize a stationary hold without interpreting canonical mapping."""

    values = _frame_matrix(frames, phase)
    timestamps = np.asarray(
        [frame.received_monotonic_ns for frame in frames], dtype=np.int64
    )
    elapsed_s = float(timestamps[-1] - timestamps[0]) * 1e-9
    rate_hz = (
        float((len(frames) - 1) / elapsed_s)
        if len(frames) > 1 and elapsed_s > 0.0 else None
    )
    median = np.median(values, axis=0)
    span = np.ptp(values, axis=0)
    mad = np.median(np.abs(values - median), axis=0)
    return {
        "phase": phase,
        "samples": len(frames),
        "rate_hz": rate_hz,
        "median_sdk_callback_deg": median.tolist(),
        "span_deg": span.tolist(),
        "mad_deg": mad.tolist(),
        "stationary": bool(
            float(np.max(span)) <= MAX_STATIONARY_HOLD_SPAN_DEG
        ),
    }


def require_stationary_hold(
    frames: list[MappingFrame], phase: str
) -> dict:
    summary = summarize_hold(frames, phase)
    if not summary["stationary"]:
        raise Mode7ProbeHardFail(
            f"{phase} was not stationary: max span "
            f"{max(summary['span_deg']):.3f} deg"
        )
    if (
        summary["rate_hz"] is None
        or summary["rate_hz"] < MIN_FEEDBACK_RATE_HZ
    ):
        raise Mode7ProbeHardFail(
            f"{phase} feedback rate is below {MIN_FEEDBACK_RATE_HZ:.0f} Hz"
        )
    return summary


def analyze_manual_excursion(
    baseline_frames: list[MappingFrame],
    movement_frames: list[MappingFrame],
    expected_joint_index: int,
    canonical_delta_sign: int,
    *,
    return_frames: list[MappingFrame] | None = None,
    expected_mode7_channel_id: int | None = None,
    zero_reference_confirmed: bool = False,
    measured_excursion_deg: float | None = None,
    measurement_method: str | None = None,
    canonical_range_deg: tuple[float, float] | None = None,
) -> dict:
    """Produce conservative F_fb candidates; never promotes a mapping.

    With an explicit ``return_frames`` window, ``movement_frames`` is an
    endpoint hold and the median is used. Without it, the legacy robust-peak
    path is retained for offline compatibility only; it cannot establish an
    independent return-to-zero result.
    """

    if not 0 <= expected_joint_index < HAND_DOF:
        raise ValueError("expected_joint_index must be 0..10")
    if canonical_delta_sign not in (-1, 1):
        raise ValueError("canonical_delta_sign must be -1 or +1")
    channel_id = (
        expected_joint_index + 1
        if expected_mode7_channel_id is None
        else int(expected_mode7_channel_id)
    )
    if not 1 <= channel_id <= HAND_DOF:
        raise ValueError("expected MODE7 channel must be 1..11")
    expected_channel_index = channel_id - 1

    baseline = _frame_matrix(baseline_frames, "canonical_zero_reference")
    movement = _frame_matrix(movement_frames, "endpoint_hold")
    baseline_median = np.median(baseline, axis=0)
    deltas = movement - baseline_median
    if return_frames is None:
        # Use the median of the strongest same-sign samples per channel so a
        # single spike cannot become an identity/sign candidate.
        representative_deltas = []
        selected_count = max(
            MIN_MODE7_FRAMES, int(math.ceil(len(movement) * 0.25))
        )
        selected_count = min(selected_count, len(movement))
        for channel_index in range(HAND_DOF):
            column = deltas[:, channel_index]
            strongest = np.argsort(np.abs(column))[-selected_count:]
            representative_deltas.append(float(np.median(column[strongest])))
        representative = np.asarray(representative_deltas)
        analysis_method = "ROBUST_STRONGEST_QUARTILE_LEGACY_WINDOW"
    else:
        representative = np.median(deltas, axis=0)
        analysis_method = "INDEPENDENT_ENDPOINT_HOLD_MEDIAN"

    magnitudes = np.abs(representative)
    dominant_index = int(np.argmax(magnitudes))
    sorted_magnitudes = np.sort(magnitudes)
    second_magnitude = float(sorted_magnitudes[-2])
    dominant_magnitude = float(sorted_magnitudes[-1])
    dominance_ratio = (
        None if second_magnitude == 0.0
        else dominant_magnitude / second_magnitude
    )
    expected_delta = float(representative[expected_channel_index])
    excursion_sufficient = abs(expected_delta) >= MIN_EXCURSION_DEG
    if not excursion_sufficient:
        identity_candidate = "INSUFFICIENT_EXCURSION"
    elif dominant_index != expected_channel_index:
        identity_candidate = "UNEXPECTED_CHANNEL_DOMINANT"
    elif (
        dominance_ratio is not None
        and dominance_ratio < MIN_CHANNEL_DOMINANCE_RATIO
    ):
        identity_candidate = "AMBIGUOUS_CHANNEL_DOMINANCE"
    else:
        identity_candidate = "EXPECTED_CHANNEL_DOMINANT"

    if identity_candidate != "EXPECTED_CHANNEL_DOMINANT":
        sign_candidate = "NOT_EVALUATED_IDENTITY_UNRESOLVED"
    elif not excursion_sufficient:
        sign_candidate = "INSUFFICIENT_EXCURSION"
    else:
        observed_sign = 1 if expected_delta > 0.0 else -1
        sign_candidate = "+1" if observed_sign == canonical_delta_sign else "-1"

    if return_frames is None:
        tail_count = max(MIN_MODE7_FRAMES, len(movement_frames) // 5)
        tail_count = min(tail_count, len(movement_frames))
        return_delta = (
            np.median(movement[-tail_count:], axis=0) - baseline_median
        )
        return_source = "LEGACY_MOVEMENT_WINDOW_TAIL_NOT_INDEPENDENT"
        return_within_tolerance = None
    else:
        returned = _frame_matrix(return_frames, "return_hold")
        return_delta = np.median(returned, axis=0) - baseline_median
        return_source = "INDEPENDENT_RETURN_HOLD_MEDIAN"
        return_within_tolerance = bool(
            abs(float(return_delta[expected_channel_index]))
            <= RETURN_TO_ZERO_TOLERANCE_DEG
        )

    identity_supported = identity_candidate == "EXPECTED_CHANNEL_DOMINANT"
    if zero_reference_confirmed and identity_supported:
        zero_candidate = float(baseline_median[expected_channel_index])
        if return_within_tolerance is True:
            zero_verdict = "OPERATOR_REFERENCED_REPEATABLE_CANDIDATE"
        elif return_within_tolerance is False:
            zero_verdict = "OPERATOR_REFERENCE_NOT_REPEATABLE"
        else:
            zero_verdict = "OPERATOR_REFERENCED_CANDIDATE_NO_INDEPENDENT_RETURN"
    elif not zero_reference_confirmed:
        zero_candidate = None
        zero_verdict = "NOT_EVALUATED_NO_CANONICAL_ZERO_REFERENCE"
    else:
        zero_candidate = None
        zero_verdict = "NOT_EVALUATED_IDENTITY_UNRESOLVED"

    scale_ratio = None
    scale_verdict = "NOT_EVALUATED_NO_PHYSICAL_MEASUREMENT"
    if measured_excursion_deg is not None and identity_supported:
        physical_delta = float(measured_excursion_deg)
        if not np.isfinite(physical_delta) or physical_delta <= 0.0:
            raise ValueError("measured_excursion_deg must be finite and > 0")
        scale_ratio = abs(expected_delta) / physical_delta
        scale_verdict = "CANDIDATE_ONLY_PENDING_METROLOGY_REVIEW"
    elif measured_excursion_deg is not None:
        scale_verdict = "NOT_EVALUATED_IDENTITY_UNRESOLVED"

    baseline_range_compatible = None
    baseline_distance_outside = None
    if canonical_range_deg is not None:
        low, high = (float(v) for v in canonical_range_deg)
        baseline_value = float(baseline_median[expected_channel_index])
        baseline_range_compatible = bool(low <= baseline_value <= high)
        baseline_distance_outside = (
            0.0 if baseline_range_compatible
            else float(low - baseline_value if baseline_value < low
                       else baseline_value - high)
        )

    return {
        "analysis_method": analysis_method,
        "expected_canonical_joint_index": expected_joint_index,
        "expected_mode7_channel_from_vendor_layout": channel_id,
        "expected_channel": channel_id,
        "observed_dominant_channel": dominant_index + 1,
        "identity_candidate": identity_candidate,
        "channel_dominance_ratio": dominance_ratio,
        "second_largest_channel_delta_deg": second_magnitude,
        "canonical_delta_sign": canonical_delta_sign,
        "observed_expected_channel_peak_delta_deg": expected_delta,
        "observed_expected_channel_endpoint_delta_deg": expected_delta,
        "feedback_sign_gain_candidate": sign_candidate,
        "excursion_sufficient_for_sign_candidate": excursion_sufficient,
        "all_channel_peak_delta_deg": representative.tolist(),
        "all_channel_endpoint_delta_deg": representative.tolist(),
        "physical_excursion_deg": measured_excursion_deg,
        "physical_excursion_measurement_method": measurement_method,
        "feedback_to_physical_scale_ratio_candidate": scale_ratio,
        "rough_scale_evidence": scale_verdict,
        "zero_reference_confirmed": bool(zero_reference_confirmed),
        "feedback_value_at_canonical_zero_candidate_deg": zero_candidate,
        "zero_mapping_evidence": zero_verdict,
        "baseline_strict_canonical_range_compatible": (
            baseline_range_compatible
        ),
        "baseline_canonical_distance_outside_deg": baseline_distance_outside,
        "return_evidence_source": return_source,
        "return_to_baseline_delta_deg": return_delta.tolist(),
        "expected_channel_return_delta_deg": float(
            return_delta[expected_channel_index]
        ),
        "return_within_tolerance": return_within_tolerance,
        "return_tolerance_deg": RETURN_TO_ZERO_TOLERANCE_DEG,
        "F_fb_verdict": "PENDING_HUMAN_REVIEW",
        "F_cmd_changed_or_validated": False,
        "canonical_valid": False,
    }


def review_static_probe_artifacts(
    recording_root: Path,
    *,
    hardware_map: SG100HardwareMap | None = None,
    joint_indices: tuple[int, ...] = PRIORITY_JOINT_INDICES,
) -> dict:
    """Aggregate valid entry snapshots without treating them as F_fb tests."""

    hardware_map = hardware_map or SG100HardwareMap()
    ranges = canonical_ranges_deg(hardware_map)
    accepted_runs: list[str] = []
    ignored_runs: list[dict] = []
    samples: dict[int, list[float]] = {index: [] for index in joint_indices}
    for path in sorted(Path(recording_root).glob(
        "sg100_mode7_probe_*/summary.json"
    )):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            ignored_runs.append({
                "run": path.parent.name,
                "reason": f"UNREADABLE_SUMMARY: {exc}",
            })
            continue
        per_joint = payload.get("per_joint")
        status_ok = payload.get("MODE7_STATUS_MODE") == "PASS"
        observation_ok = payload.get("OBSERVATION_COMPLETE") is True
        stable_count = int(payload.get("stable_mode7_samples_captured") or 0)
        if (
            not isinstance(per_joint, list)
            or len(per_joint) != HAND_DOF
            or not status_ok
            or not observation_ok
            or stable_count < MIN_MODE7_FRAMES
        ):
            ignored_runs.append({
                "run": path.parent.name,
                "reason": "NOT_A_COMPLETE_STABLE_ALL_MODE7_SNAPSHOT",
            })
            continue
        try:
            values = [
                float(item["mode7_median_deg"])
                for item in per_joint
            ]
        except (KeyError, TypeError, ValueError):
            ignored_runs.append({
                "run": path.parent.name,
                "reason": "MALFORMED_PER_JOINT_BASELINES",
            })
            continue
        if not np.all(np.isfinite(values)):
            ignored_runs.append({
                "run": path.parent.name,
                "reason": "NON_FINITE_BASELINE",
            })
            continue
        accepted_runs.append(path.parent.name)
        for index in joint_indices:
            samples[index].append(values[index])

    per_joint_review = []
    for index in joint_indices:
        values = samples[index]
        low, high = ranges[index]
        compatible = [low <= value <= high for value in values]
        below = sum(value < low for value in values)
        above = sum(value > high for value in values)
        if not values:
            classification = "NO_COMPLETE_STATIC_SNAPSHOTS"
        elif all(compatible):
            classification = "ALL_STATIC_SNAPSHOTS_RANGE_COMPATIBLE"
        elif above == len(values):
            classification = (
                "PERSISTENT_ABOVE_CANONICAL_RANGE_STATIC_SNAPSHOTS_ONLY"
            )
        elif below == len(values):
            classification = (
                "PERSISTENT_BELOW_CANONICAL_RANGE_STATIC_SNAPSHOTS_ONLY"
            )
        else:
            classification = (
                "CANONICAL_BOUNDARY_CROSSING_STATIC_SNAPSHOTS_ONLY"
            )
        per_joint_review.append({
            "joint": f"J{index + 1}",
            "canonical_joint": hardware_map.software_joint_order[index],
            "expected_mode7_channel_from_vendor_layout": (
                hardware_map.hardware_joint_ids[index]
            ),
            "canonical_range_deg": [low, high],
            "static_mode7_medians_deg": values,
            "snapshot_count": len(values),
            "strict_canonical_compatible_count": sum(compatible),
            "below_canonical_count": below,
            "above_canonical_count": above,
            "across_run_min_deg": min(values) if values else None,
            "across_run_max_deg": max(values) if values else None,
            "classification": classification,
            "identity_inference": "NOT_ALLOWED_UNLABELLED_STATIC_SNAPSHOTS",
            "sign_inference": "NOT_ALLOWED_UNLABELLED_STATIC_SNAPSHOTS",
            "zero_inference": "NOT_ALLOWED_WITHOUT_PHYSICAL_ZERO_REFERENCE",
            "candidate_selected": False,
        })
    return {
        "schema": STATIC_REVIEW_SCHEMA,
        "source_kind": "UNLABELLED_MODE7_ENTRY_SNAPSHOTS",
        "accepted_runs": accepted_runs,
        "ignored_runs": ignored_runs,
        "per_joint": per_joint_review,
        "F_fb_verdict": "PENDING_LABELLED_SINGLE_DOF_CHARACTERIZATION",
        "F_cmd_changed_or_validated": False,
        "canonical_valid": False,
    }


def format_static_review(review: dict) -> str:
    lines = [
        "Supplemental static MODE7 snapshots (context only; no F_fb "
        "inference):",
        "joint  runs  in_range  observed_deg             classification",
    ]
    for item in review["per_joint"]:
        observed = (
            "NONE" if not item["snapshot_count"]
            else f"[{item['across_run_min_deg']:+.4f},"
                 f" {item['across_run_max_deg']:+.4f}]"
        )
        lines.append(
            f"{item['joint']:<5}  {item['snapshot_count']:>4}  "
            f"{item['strict_canonical_compatible_count']:>3}/"
            f"{item['snapshot_count']:<3}  {observed:<24} "
            f"{item['classification']}"
        )
    lines.append(
        "identity/sign/zero remain pending labelled single-DOF movement; "
        "no candidate is auto-selected."
    )
    return "\n".join(lines)


def _sha256(path: Path) -> str | None:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return None


def runtime_fingerprints(
    sdk_dir: Path,
    hardware_map_path: Path,
    semantics_path: Path,
) -> dict:
    sdk_dir = Path(sdk_dir)
    candidates = (
        sdk_dir / "joint_angle_map.json",
        sdk_dir / "joint_angle_map_v3.0.json",
    )
    existing_maps = [path for path in candidates if path.exists()]
    sdk_libraries = sorted(
        path for path in (sdk_dir / "lib").glob("**/*")
        if path.is_file() and "sg100" in path.name.lower()
    ) if (sdk_dir / "lib").exists() else []
    return {
        "sdk_dir": str(sdk_dir.resolve()),
        "joint_angle_maps": [
            {"path": str(path.resolve()), "sha256": _sha256(path)}
            for path in existing_maps
        ],
        "runtime_joint_angle_map_selection": (
            "UNVERIFIED_MULTIPLE_CANDIDATES"
            if len(existing_maps) > 1 else
            "ONLY_DISCOVERED_CANDIDATE_NOT_RUNTIME_PROOF"
            if len(existing_maps) == 1 else
            "NO_CANDIDATE_DISCOVERED"
        ),
        "sdk_libraries": [
            {"path": str(path.resolve()), "sha256": _sha256(path)}
            for path in sdk_libraries
        ],
        "sg100_hardware_map": {
            "path": str(hardware_map_path.resolve()),
            "sha256": _sha256(hardware_map_path),
            "authority": "F_cmd_ONLY_NOT_F_fb",
        },
        "sg100_joint_semantics": {
            "path": str(semantics_path.resolve()),
            "sha256": _sha256(semantics_path),
        },
    }


def validate_duration(name: str, value: float) -> float:
    value = float(value)
    if (
        not math.isfinite(value)
        or value < MIN_PHASE_DURATION_S
        or value > MAX_PHASE_DURATION_S
    ):
        raise ValueError(
            f"{name} must be finite and within "
            f"[{MIN_PHASE_DURATION_S}, {MAX_PHASE_DURATION_S}] seconds"
        )
    return value


def _write_artifacts(
    artifact_dir: Path,
    report: dict,
    frames: list[MappingFrame],
) -> None:
    artifact_dir.mkdir(parents=True, exist_ok=True)
    summary_path = artifact_dir / "summary.json"
    samples_path = artifact_dir / "sdk_callback_samples.csv"
    if summary_path.exists() or samples_path.exists():
        raise FileExistsError(
            f"refusing to overwrite mapping artifact: {artifact_dir}"
        )
    summary_path.write_text(
        json.dumps(report, indent=2, default=str), encoding="utf-8"
    )
    header = (
        "phase,sequence,received_monotonic_ns,"
        + ",".join(f"J{joint}_returned_mode" for joint in range(1, 12))
        + ","
        + ",".join(
            f"J{joint}_sdk_callback_deg" for joint in range(1, 12)
        )
    )
    rows = [header]
    for frame in frames:
        rows.append(
            f"{frame.phase},{frame.sequence},{frame.received_monotonic_ns},"
            + ",".join(str(mode) for mode in frame.returned_modes)
            + ","
            + ",".join(
                f"{value:.9f}"
                for value in frame.sdk_callback_positions_deg
            )
        )
    samples_path.write_text(
        "\n".join(rows) + "\n", encoding="utf-8", newline="\n"
    )


def _transition_mapping_frames(tracker) -> list[MappingFrame]:
    return [
        MappingFrame(
            phase="mode7_transition",
            sequence=(
                int(frame.sequence)
                if frame.sequence is not None else -1
            ),
            received_monotonic_ns=int(frame.timestamp_ns),
            returned_modes=tuple(int(mode) for mode in frame.modes),
            sdk_callback_positions_deg=tuple(
                float(value) for value in frame.positions_deg
            ),
        )
        for frame in tracker.frames
    ]


def restore_mode_null_and_confirm(driver) -> dict:
    """Submit MODE_NULL and require fresh all-mode0 RX evidence."""

    result = {
        "mode_null_command_accepted": False,
        "mode_null_rx_confirmed": False,
        "fresh_mode_null_frames": 0,
    }
    try:
        accepted = bool(driver.hold_passive())
        result["mode_null_command_accepted"] = accepted
        if not accepted:
            result["reason"] = "MODE_NULL_COMMAND_REJECTED"
            return result
        deadline = time.monotonic() + MODE_NULL_RESTORE_TIMEOUT_S
        last_sequence = None
        consecutive = 0
        while time.monotonic() < deadline:
            feedback = driver.read_feedback()
            if feedback is None or feedback.sequence == last_sequence:
                time.sleep(0.001)
                continue
            last_sequence = int(feedback.sequence)
            modes = (
                tuple(int(mode) for mode in feedback.joint_modes)
                if feedback.joint_modes is not None else ()
            )
            if modes == (0,) * HAND_DOF:
                consecutive += 1
                result["fresh_mode_null_frames"] = consecutive
                if consecutive >= MIN_MODE7_FRAMES:
                    result["mode_null_rx_confirmed"] = True
                    return result
            else:
                consecutive = 0
            time.sleep(0.001)
        result["reason"] = "MODE_NULL_RX_CONFIRMATION_TIMEOUT"
        return result
    except BaseException as exc:
        result["reason"] = "MODE_NULL_CLEANUP_EXCEPTION"
        result["exception"] = repr(exc)
        return result


def run_characterization(
    port: str,
    baudrate: int,
    sdk_dir: Path,
    joint_index: int,
    artifact_dir: Path,
    *,
    baseline_s: float = DEFAULT_BASELINE_S,
    lead_in_s: float = DEFAULT_LEAD_IN_S,
    movement_window_s: float = DEFAULT_MOVEMENT_WINDOW_S,
    endpoint_hold_s: float = DEFAULT_ENDPOINT_HOLD_S,
    return_window_s: float = DEFAULT_RETURN_WINDOW_S,
    return_hold_s: float = DEFAULT_RETURN_HOLD_S,
    measured_excursion_deg: float | None = None,
    measurement_method: str | None = None,
    confirm_fn=input,
) -> int:
    """Execute one passive manual-DOF observation (future hardware only).

    All argument and operator-identity checks happen before driver creation.
    No successful exit is returned until MODE_NULL command acceptance and
    fresh all-mode0 RX restoration have both been observed.
    """

    hardware_map = SG100HardwareMap()
    if not 0 <= joint_index < HAND_DOF:
        raise ValueError("joint_index must be 0..10")
    baseline_s = validate_duration("baseline_s", baseline_s)
    lead_in_s = validate_duration("lead_in_s", lead_in_s)
    movement_window_s = validate_duration(
        "movement_window_s", movement_window_s
    )
    endpoint_hold_s = validate_duration("endpoint_hold_s", endpoint_hold_s)
    return_window_s = validate_duration("return_window_s", return_window_s)
    return_hold_s = validate_duration("return_hold_s", return_hold_s)
    if measured_excursion_deg is None:
        raise ValueError(
            "measured_excursion_deg is required for an executable trial"
        )
    measured_excursion_deg = float(measured_excursion_deg)
    if (
        not math.isfinite(measured_excursion_deg)
        or measured_excursion_deg < 5.0
        or measured_excursion_deg > 60.0
    ):
        raise ValueError(
            "measured_excursion_deg must be finite and within [5, 60]"
        )
    if measurement_method not in {"operator_estimate", "external_measurement"}:
        raise ValueError(
            "measurement_method must be operator_estimate or "
            "external_measurement"
        )
    joint_name = hardware_map.software_joint_order[joint_index]
    channel_id = hardware_map.hardware_joint_ids[joint_index]
    canonical_sign, instruction = CANONICAL_MOVEMENTS[joint_index]
    zero_reference = CANONICAL_ZERO_REFERENCES[joint_index]
    joint_label = f"J{joint_index + 1} {joint_name}"
    zero_phrase = f"ZERO {joint_label}"
    arm_phrase = f"ARM {joint_label}"
    print(
        f"Place ONLY {joint_label} at this physical canonical-zero "
        f"landmark: {zero_reference}."
    )
    if confirm_fn(f"Type exactly '{zero_phrase}': ").strip() != zero_phrase:
        print("[ABORT] canonical-zero landmark not explicitly confirmed")
        return 4
    if confirm_fn(f"Type exactly '{arm_phrase}': ").strip() != arm_phrase:
        print("[ABORT] joint-specific MODE7 trial not armed")
        return 4

    artifact_dir = Path(artifact_dir)
    if artifact_dir.exists():
        raise FileExistsError(
            f"refusing to reuse existing artifact directory: {artifact_dir}"
        )
    artifact_dir.mkdir(parents=True, exist_ok=False)

    driver = None
    all_frames: list[MappingFrame] = []
    report: dict = {
        "schema": SCHEMA,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "port": port,
        "baudrate": int(baudrate),
        "joint": f"J{joint_index + 1}",
        "canonical_joint": joint_name,
        "expected_mode7_channel_from_vendor_layout": channel_id,
        "canonical_manual_direction": instruction,
        "canonical_delta_sign": canonical_sign,
        "canonical_zero_reference": zero_reference,
        "operator_canonical_zero_reference_confirmed": True,
        "physical_excursion_deg": measured_excursion_deg,
        "physical_excursion_measurement_method": measurement_method,
        "units": {
            "positions": "degree at vendor SDK Python callback boundary",
            "not_wire_encoder": True,
        },
        "runtime_fingerprints": runtime_fingerprints(
            Path(sdk_dir),
            DEFAULT_HARDWARE_MAP_PATH,
            DEFAULT_SEMANTICS_PATH,
        ),
        "durations_s": {
            "canonical_zero_reference_hold": baseline_s,
            "still_lead_in": lead_in_s,
            "outbound_move": movement_window_s,
            "endpoint_hold": endpoint_hold_s,
            "return_move": return_window_s,
            "return_hold": return_hold_s,
        },
        "initial_mode7_configuration_count": 0,
        "later_position_target_updates": 0,
        "active_motion_commands": 0,
        "F_cmd_changed_or_validated": False,
        "F_fb_verdict": "PENDING",
        "canonical_valid": False,
        "status": "NOT_STARTED",
    }
    result_code = 11
    mode7_submitted = False
    try:
        driver = _probe_driver(port, baudrate, sdk_dir)
        driver.connect()
        report["mode_null_baseline"] = run_phase_a(driver, baseline_s)
        print(format_preflight_table(build_full_hand_mode7_control()))
        print(
            f"Manual phase for {joint_label}: move ONLY this DOF about "
            f"{measured_excursion_deg:.1f} degrees: {instruction}; hold at "
            "the endpoint, then return to the confirmed physical zero."
        )

        calls_before = driver.diagnostics["hand_control_calls"]
        submission = driver.send_mode7_feedback_probe()
        calls_after = driver.diagnostics["hand_control_calls"]
        if submission is None or calls_after - calls_before != 1:
            raise Mode7ProbeHardFail(
                "initial MODE7 frame was not accepted exactly once"
            )
        mode7_submitted = True
        report["initial_mode7_configuration_count"] = 1
        tracker = _observe_transition(
            driver,
            submitted_monotonic_ns=submission.sent_monotonic_ns,
        )
        report["transition"] = tracker.summary()
        all_frames.extend(_transition_mapping_frames(tracker))
        if tracker.state != STATE_MODE7_STABLE or not tracker.observation_complete:
            raise Mode7ProbeHardFail(
                f"MODE7 transition not stable/complete: {tracker.state}"
            )

        moved_answer = confirm_fn(
            "Did the hand move by itself when MODE7 was entered? [y/N] "
        ).strip().lower()
        if moved_answer in ("y", "yes"):
            report["mode7_entry_no_visible_motion"] = False
            report["status"] = "UNEXPECTED_MODE7_ENTRY_MOTION"
            result_code = 7
        elif moved_answer in ("", "n", "no"):
            report["mode7_entry_no_visible_motion"] = True
        else:
            report["mode7_entry_no_visible_motion"] = None
            report["status"] = "ENTRY_MOTION_RESPONSE_INVALID"
            result_code = 7
        if result_code == 7:
            raise Mode7ProbeHardFail(report["status"])

        transition_sequence = tracker.frames[-1].sequence
        baseline_frames = capture_mode7_window(
            driver,
            baseline_s,
            "canonical_zero_reference_hold",
            after_sequence=transition_sequence,
            artifact_sink=all_frames,
        )
        report["canonical_zero_reference_hold"] = require_stationary_hold(
            baseline_frames, "canonical_zero_reference_hold"
        )
        print(
            f"Lead-in {lead_in_s:.1f}s: keep every joint still."
        )
        lead_in_frames = capture_mode7_window(
            driver,
            lead_in_s,
            "still_lead_in",
            after_sequence=baseline_frames[-1].sequence,
            artifact_sink=all_frames,
        )
        report["still_lead_in"] = require_stationary_hold(
            lead_in_frames, "still_lead_in"
        )
        print(
            f"MOVE NOW for {movement_window_s:.1f}s: only {joint_label}, "
            f"{instruction}; finish at the endpoint and HOLD."
        )
        outbound_frames = capture_mode7_window(
            driver,
            movement_window_s,
            "outbound_move",
            after_sequence=lead_in_frames[-1].sequence,
            artifact_sink=all_frames,
        )
        print(f"HOLD endpoint for {endpoint_hold_s:.2f}s.")
        endpoint_frames = capture_mode7_window(
            driver,
            endpoint_hold_s,
            "endpoint_hold",
            after_sequence=outbound_frames[-1].sequence,
            artifact_sink=all_frames,
        )
        report["endpoint_hold"] = require_stationary_hold(
            endpoint_frames, "endpoint_hold"
        )
        print(
            f"RETURN NOW for {return_window_s:.1f}s: return only "
            f"{joint_label} to the confirmed physical zero and HOLD."
        )
        return_move_frames = capture_mode7_window(
            driver,
            return_window_s,
            "return_move",
            after_sequence=endpoint_frames[-1].sequence,
            artifact_sink=all_frames,
        )
        print(f"HOLD returned zero for {return_hold_s:.2f}s.")
        return_hold_frames = capture_mode7_window(
            driver,
            return_hold_s,
            "return_hold",
            after_sequence=return_move_frames[-1].sequence,
            artifact_sink=all_frames,
        )
        report["return_hold"] = require_stationary_hold(
            return_hold_frames, "return_hold"
        )

        only_phrase = f"ONLY J{joint_index + 1}"
        returned_phrase = f"RETURNED J{joint_index + 1}"
        only_confirmed = (
            confirm_fn(
                f"Confirm only {joint_label} moved in the instructed "
                f"direction. Type exactly '{only_phrase}': "
            ).strip() == only_phrase
        )
        returned_confirmed = (
            confirm_fn(
                f"Confirm {joint_label} returned to the same physical-zero "
                f"landmark. Type exactly '{returned_phrase}': "
            ).strip() == returned_phrase
        )
        report["operator_single_dof_direction_confirmed"] = only_confirmed
        report["operator_return_to_zero_confirmed"] = returned_confirmed
        report["analysis"] = analyze_manual_excursion(
            baseline_frames,
            endpoint_frames,
            joint_index,
            canonical_sign,
            return_frames=return_hold_frames,
            expected_mode7_channel_id=channel_id,
            zero_reference_confirmed=True,
            measured_excursion_deg=measured_excursion_deg,
            measurement_method=measurement_method,
            canonical_range_deg=canonical_ranges_deg(hardware_map)[joint_index],
        )
        report["F_fb_verdict"] = "PENDING_HUMAN_REVIEW"
        if not only_confirmed or not returned_confirmed:
            report["status"] = "OPERATOR_ATTRIBUTION_NOT_CONFIRMED"
            result_code = 7
        else:
            report["status"] = "PASSIVE_EVIDENCE_CAPTURED_MAPPING_PENDING"
            result_code = 0
    except Mode7ProbeHardFail as exc:
        if report["status"] in ("NOT_STARTED", "PENDING"):
            report["status"] = "HARD_FAIL"
        report["hard_fail_reason"] = str(exc)
        if result_code not in (6, 7):
            result_code = 5
    except BaseException as exc:
        report["status"] = "UNEXPECTED_EXCEPTION"
        report["exception_type"] = type(exc).__name__
        report["exception_reason"] = str(exc)
        result_code = 11
    finally:
        cleanup = {
            "mode_null_command_accepted": None,
            "mode_null_rx_confirmed": None,
            "stop": "NOT_ATTEMPTED",
            "disconnect": "NOT_ATTEMPTED",
        }
        if driver is not None:
            cleanup.update(restore_mode_null_and_confirm(driver))
            try:
                driver.stop()
                cleanup["stop"] = "ATTEMPTED"
            except BaseException as exc:
                cleanup["stop"] = "FAILED"
                cleanup["stop_error"] = repr(exc)
            try:
                driver.disconnect()
                cleanup["disconnect"] = "ATTEMPTED"
            except BaseException as exc:
                cleanup["disconnect"] = "FAILED"
                cleanup["disconnect_error"] = repr(exc)
        report["cleanup"] = cleanup
        cleanup_ok = bool(
            cleanup["mode_null_command_accepted"]
            and cleanup["mode_null_rx_confirmed"]
            and cleanup["stop"] != "FAILED"
            and cleanup["disconnect"] != "FAILED"
        )
        if mode7_submitted and not cleanup_ok:
            report["pre_cleanup_status"] = report["status"]
            report["status"] = "CLEANUP_FAIL"
            result_code = 6
        report["completed_utc"] = datetime.now(timezone.utc).isoformat()
        try:
            _write_artifacts(artifact_dir, report, all_frames)
        except BaseException as exc:
            print(f"[FATAL] could not preserve mapping artifact: {exc}")
            result_code = 12
    return result_code


def _joint_index(value: str, joint_names: list[str]) -> int:
    normalized = value.strip().lower()
    if normalized.startswith("j") and normalized[1:].isdigit():
        index = int(normalized[1:]) - 1
        if 0 <= index < HAND_DOF:
            return index
    if normalized in joint_names:
        return joint_names.index(normalized)
    raise ValueError("--joint must be J1..J11 or a canonical joint name")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Prepare/run one passive SG100 MODE7 F_fb observation"
    )
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--sdk-dir", type=Path, default=DEFAULT_SDK_DIR)
    parser.add_argument(
        "--joint",
        help="required for execution: J1..J11 or canonical joint name",
    )
    parser.add_argument(
        "--recording-root", type=Path, default=Path("recording")
    )
    parser.add_argument("--baseline-seconds", type=float, default=DEFAULT_BASELINE_S)
    parser.add_argument("--lead-in-seconds", type=float, default=DEFAULT_LEAD_IN_S)
    parser.add_argument(
        "--movement-window-seconds",
        type=float,
        default=DEFAULT_MOVEMENT_WINDOW_S,
    )
    parser.add_argument(
        "--endpoint-hold-seconds",
        type=float,
        default=DEFAULT_ENDPOINT_HOLD_S,
    )
    parser.add_argument(
        "--return-window-seconds",
        type=float,
        default=DEFAULT_RETURN_WINDOW_S,
    )
    parser.add_argument(
        "--return-hold-seconds",
        type=float,
        default=DEFAULT_RETURN_HOLD_S,
    )
    parser.add_argument("--physical-excursion-degrees", type=float)
    parser.add_argument(
        "--measurement-method",
        choices=("operator_estimate", "external_measurement"),
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--enter-mode7", action="store_true")
    parser.add_argument("--manual-characterization", action="store_true")
    args = parser.parse_args(argv)

    hardware_map = SG100HardwareMap()
    opt_ins = (
        args.execute, args.enter_mode7, args.manual_characterization
    )
    if any(opt_ins) and not all(opt_ins):
        parser.error(
            "hardware execution requires all of --execute, --enter-mode7, "
            "and --manual-characterization"
        )
    execute_requested = bool(all(opt_ins))
    if execute_requested and not args.joint:
        parser.error("--joint is required for hardware execution")
    index = None
    if args.joint:
        try:
            index = _joint_index(args.joint, hardware_map.software_joint_order)
        except ValueError as exc:
            parser.error(str(exc))
    if not execute_requested:
        print("[PREPARED ONLY] no hardware opened and no command submitted.")
        review = review_static_probe_artifacts(
            args.recording_root, hardware_map=hardware_map
        )
        print(format_static_review(review))
        selected = (
            [index] if index is not None else list(PRIORITY_JOINT_INDICES)
        )
        print("Prepared single-DOF priority trials:")
        for selected_index in selected:
            sign, instruction = CANONICAL_MOVEMENTS[selected_index]
            name = hardware_map.software_joint_order[selected_index]
            print(
                f"  J{selected_index + 1} {name}: canonical sign "
                f"{sign:+d}; zero={CANONICAL_ZERO_REFERENCES[selected_index]}; "
                f"move={instruction}."
            )
            if selected_index in PRIORITY_QUESTIONS:
                print(f"    question: {PRIORITY_QUESTIONS[selected_index]}")
        print(format_preflight_table(build_full_hand_mode7_control()))
        return 0
    if not args.port:
        parser.error("--port is required for hardware execution")
    if args.physical_excursion_degrees is None:
        parser.error(
            "--physical-excursion-degrees is required for hardware execution"
        )
    if args.measurement_method is None:
        parser.error("--measurement-method is required for hardware execution")
    for name, value in (
        ("--baseline-seconds", args.baseline_seconds),
        ("--lead-in-seconds", args.lead_in_seconds),
        ("--movement-window-seconds", args.movement_window_seconds),
        ("--endpoint-hold-seconds", args.endpoint_hold_seconds),
        ("--return-window-seconds", args.return_window_seconds),
        ("--return-hold-seconds", args.return_hold_seconds),
    ):
        try:
            validate_duration(name, value)
        except ValueError as exc:
            parser.error(str(exc))
    assert index is not None
    artifact_dir = Path("recording") / (
        f"sg100_mode7_mapping_J{index + 1}_"
        + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    )
    return run_characterization(
        args.port,
        args.baudrate,
        args.sdk_dir,
        index,
        artifact_dir,
        baseline_s=args.baseline_seconds,
        lead_in_s=args.lead_in_seconds,
        movement_window_s=args.movement_window_seconds,
        endpoint_hold_s=args.endpoint_hold_seconds,
        return_window_s=args.return_window_seconds,
        return_hold_s=args.return_hold_seconds,
        measured_excursion_deg=args.physical_excursion_degrees,
        measurement_method=args.measurement_method,
    )


if __name__ == "__main__":
    sys.exit(main())
