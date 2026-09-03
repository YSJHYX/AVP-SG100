#!/usr/bin/env python3
"""Passive SG100 MODE7 feedback-to-canonical mapping characterization.

PREPARED ONLY. This tool is the next hardware stage; it is not an active
target-motion test. One run observes one manually moved physical DOF:

1. establish the proven-safe MODE_NULL/realtime baseline;
2. submit exactly one full-hand MODE7, position=0, PID=0, output=0 frame via
   SG100SDKDriver.send_mode7_feedback_probe();
3. verify all 11 returned modes become and remain MODE7;
4. record raw MODE7 Joint_Status.position while the operator manually moves
   the selected DOF in the stated canonical direction and returns it;
5. restore MODE_NULL on every exit.

It never sends a later target, performs homing/zeroing, derives a startup
offset, applies the outbound hardware map, or marks canonical feedback valid.
The output is evidence for F_fb only. It does not validate or modify F_cmd.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
import time
from pathlib import Path

import numpy as np

from sg100_teleop.robot.sg100_driver import (
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
    format_preflight_table,
    build_full_hand_mode7_control,
    run_phase_a,
)


SCHEMA = "sg100_mode7_passive_mapping_characterization_v1"
MIN_EXCURSION_DEG = 5.0
DEFAULT_BASELINE_S = 0.5
DEFAULT_LEAD_IN_S = 2.0
DEFAULT_MOVEMENT_WINDOW_S = 6.0
TRANSPORT_STALL_S = 0.1

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


@dataclasses.dataclass(frozen=True)
class MappingFrame:
    phase: str
    sequence: int
    received_monotonic_ns: int
    raw_mode7_positions_deg: tuple[float, ...]


def extract_mode7_raw_degrees(feedback) -> np.ndarray:
    """Return raw callback degrees only when every returned mode is MODE7."""

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


def capture_mode7_window(
    driver,
    duration_s: float,
    phase: str,
    *,
    after_sequence: int | None = None,
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
            positions_deg = extract_mode7_raw_degrees(feedback)
            last_sequence = int(feedback.sequence)
            last_progress = time.monotonic()
            frames.append(MappingFrame(
                phase=phase,
                sequence=last_sequence,
                received_monotonic_ns=int(feedback.received_monotonic_ns),
                raw_mode7_positions_deg=tuple(float(v) for v in positions_deg),
            ))
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


def analyze_manual_excursion(
    baseline_frames: list[MappingFrame],
    movement_frames: list[MappingFrame],
    expected_joint_index: int,
    canonical_delta_sign: int,
) -> dict:
    """Produce candidates for human review; never promotes a mapping."""

    baseline = np.asarray(
        [frame.raw_mode7_positions_deg for frame in baseline_frames],
        dtype=np.float64,
    )
    movement = np.asarray(
        [frame.raw_mode7_positions_deg for frame in movement_frames],
        dtype=np.float64,
    )
    baseline_median = np.median(baseline, axis=0)
    deltas = movement - baseline_median
    peak_indices = np.argmax(np.abs(deltas), axis=0)
    peak_deltas = np.asarray([
        deltas[sample_index, joint_index]
        for joint_index, sample_index in enumerate(peak_indices)
    ])
    dominant_index = int(np.argmax(np.abs(peak_deltas)))
    expected_delta = float(peak_deltas[expected_joint_index])
    excursion_sufficient = abs(expected_delta) >= MIN_EXCURSION_DEG
    identity_candidate = (
        "EXPECTED_CHANNEL_DOMINANT"
        if dominant_index == expected_joint_index
        else "UNEXPECTED_CHANNEL_DOMINANT"
    )
    if not excursion_sufficient:
        sign_candidate = "INSUFFICIENT_EXCURSION"
    else:
        observed_sign = 1 if expected_delta > 0.0 else -1
        sign_candidate = "+1" if observed_sign == canonical_delta_sign else "-1"
    tail_count = max(MIN_MODE7_FRAMES, len(movement_frames) // 5)
    return_delta = np.median(movement[-tail_count:], axis=0) - baseline_median
    return {
        "expected_channel": expected_joint_index + 1,
        "observed_dominant_channel": dominant_index + 1,
        "identity_candidate": identity_candidate,
        "canonical_delta_sign": canonical_delta_sign,
        "observed_expected_channel_peak_delta_deg": expected_delta,
        "feedback_sign_gain_candidate": sign_candidate,
        "excursion_sufficient_for_sign_candidate": excursion_sufficient,
        "all_channel_peak_delta_deg": peak_deltas.tolist(),
        "expected_physical_excursion_instruction_deg": [15.0, 30.0],
        "rough_scale_evidence": "REVIEW_PEAK_DELTA_AGAINST_OPERATOR_MOTION",
        "return_to_baseline_delta_deg": return_delta.tolist(),
        "F_fb_verdict": "PENDING_HUMAN_REVIEW",
        "F_cmd_changed_or_validated": False,
        "canonical_valid": False,
    }


def _write_artifacts(
    artifact_dir: Path,
    report: dict,
    frames: list[MappingFrame],
) -> None:
    artifact_dir.mkdir(parents=True, exist_ok=True)
    (artifact_dir / "summary.json").write_text(
        json.dumps(report, indent=2, default=str)
    )
    header = (
        "phase,sequence,received_monotonic_ns,"
        + ",".join(f"J{joint}_raw_mode7_deg" for joint in range(1, 12))
    )
    rows = [header]
    for frame in frames:
        rows.append(
            f"{frame.phase},{frame.sequence},{frame.received_monotonic_ns},"
            + ",".join(f"{value:.9f}" for value in frame.raw_mode7_positions_deg)
        )
    (artifact_dir / "raw_mode7_samples.csv").write_text("\n".join(rows) + "\n")


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
    confirm_fn=input,
) -> int:
    """Execute one passive manual-DOF observation (future hardware only)."""

    hardware_map = SG100HardwareMap()
    joint_name = hardware_map.software_joint_order[joint_index]
    canonical_sign, instruction = CANONICAL_MOVEMENTS[joint_index]
    driver = _probe_driver(port, baudrate, sdk_dir)
    all_frames: list[MappingFrame] = []
    report: dict = {
        "schema": SCHEMA,
        "joint": f"J{joint_index + 1}",
        "canonical_joint": joint_name,
        "canonical_manual_direction": instruction,
        "canonical_delta_sign": canonical_sign,
        "initial_mode7_configuration_count": 0,
        "later_position_target_updates": 0,
        "active_motion_commands": 0,
        "F_cmd_changed_or_validated": False,
        "F_fb_verdict": "PENDING",
        "canonical_valid": False,
        "status": "NOT_STARTED",
    }
    try:
        driver.connect()
        report["mode_null_baseline"] = run_phase_a(driver, baseline_s)
        print(format_preflight_table(build_full_hand_mode7_control()))
        print(
            f"Manual phase for J{joint_index + 1} {joint_name}: after the "
            f"lead-in, move ONLY this DOF 15-30 degrees: {instruction}; "
            "then return it to baseline before the window ends."
        )
        answer = confirm_fn(
            "ARM the one zero-output MODE7 submission and timed passive "
            "observation? Type YES: "
        )
        if answer.strip().upper() != "YES":
            report["status"] = "OPERATOR_ABORT_BEFORE_MODE7"
            return 4

        calls_before = driver.diagnostics["hand_control_calls"]
        submission = driver.send_mode7_feedback_probe()
        calls_after = driver.diagnostics["hand_control_calls"]
        if submission is None or calls_after - calls_before != 1:
            raise Mode7ProbeHardFail(
                "initial MODE7 frame was not accepted exactly once"
            )
        report["initial_mode7_configuration_count"] = 1
        tracker = _observe_transition(
            driver,
            submitted_monotonic_ns=submission.sent_monotonic_ns,
        )
        report["transition"] = tracker.summary()
        if tracker.state != STATE_MODE7_STABLE or not tracker.observation_complete:
            raise Mode7ProbeHardFail(
                f"MODE7 transition not stable/complete: {tracker.state}"
            )

        transition_sequence = tracker.frames[-1].sequence
        baseline_frames = capture_mode7_window(
            driver,
            baseline_s,
            "baseline",
            after_sequence=transition_sequence,
        )
        all_frames.extend(baseline_frames)
        print(
            f"Lead-in {lead_in_s:.1f}s; keep the hand still. Then move only "
            f"{joint_name}: {instruction}."
        )
        lead_in_frames = capture_mode7_window(
            driver,
            lead_in_s,
            "lead_in",
            after_sequence=baseline_frames[-1].sequence,
        )
        all_frames.extend(lead_in_frames)
        print(
            f"MOVE NOW for {movement_window_s:.1f}s, then return to baseline."
        )
        movement_frames = capture_mode7_window(
            driver,
            movement_window_s,
            "manual_movement",
            after_sequence=lead_in_frames[-1].sequence,
        )
        all_frames.extend(movement_frames)
        report["analysis"] = analyze_manual_excursion(
            baseline_frames,
            movement_frames,
            joint_index,
            canonical_sign,
        )
        report["F_fb_verdict"] = "PENDING_HUMAN_REVIEW"
        report["status"] = "PASSIVE_EVIDENCE_CAPTURED"
        return 0
    except Mode7ProbeHardFail as exc:
        report["status"] = "HARD_FAIL"
        report["hard_fail_reason"] = str(exc)
        return 5
    finally:
        cleanup = {}
        try:
            cleanup["mode_null_restore"] = bool(driver.hold_passive())
        except BaseException as exc:
            cleanup["mode_null_restore"] = False
            cleanup["mode_null_restore_error"] = repr(exc)
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
        _write_artifacts(artifact_dir, report, all_frames)


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
    parser.add_argument("--joint", default="J1")
    parser.add_argument("--baseline-seconds", type=float, default=DEFAULT_BASELINE_S)
    parser.add_argument("--lead-in-seconds", type=float, default=DEFAULT_LEAD_IN_S)
    parser.add_argument(
        "--movement-window-seconds",
        type=float,
        default=DEFAULT_MOVEMENT_WINDOW_S,
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--enter-mode7", action="store_true")
    parser.add_argument("--manual-characterization", action="store_true")
    args = parser.parse_args(argv)

    hardware_map = SG100HardwareMap()
    try:
        index = _joint_index(args.joint, hardware_map.software_joint_order)
    except ValueError as exc:
        parser.error(str(exc))
    sign, instruction = CANONICAL_MOVEMENTS[index]
    if not (
        args.execute and args.enter_mode7 and args.manual_characterization
    ):
        print("[PREPARED ONLY] no hardware opened and no command submitted.")
        print(
            f"Selected J{index + 1} {hardware_map.software_joint_order[index]}: "
            f"canonical sign {sign:+d}; {instruction}."
        )
        print(format_preflight_table(build_full_hand_mode7_control()))
        return 0
    if not args.port:
        parser.error("--port is required for hardware execution")
    from datetime import datetime
    artifact_dir = Path("recording") / (
        "sg100_mode7_mapping_" + datetime.now().strftime("%Y%m%d_%H%M%S")
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
    )


if __name__ == "__main__":
    sys.exit(main())
