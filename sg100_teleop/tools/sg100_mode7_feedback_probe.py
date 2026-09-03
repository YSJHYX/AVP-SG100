#!/usr/bin/env python3
"""SG100 FULL-HAND MODE7 feedback probe — PREPARED, NOT EXECUTED.

Goal (single question, no active motion): "When all 11 joints report
mode=7, does Joint_Status.position represent the real SG100 joint angle?"

Authoritative contract this probe is built on:
- Hand_Control_t = Joint_Control_t joints[JOINT_COUNT], JOINT_COUNT = 11;
  every Joint_Control_t carries its own id and mode
  (include/sg100_types.h @ V1.5_NO_TACTILE).
- Vendor guidance (user-supplied, 2026-09-03): for realtime joint angle
  operation, populate the 11 joint IDs and use MODE_JOINT_POSITION = 7 for
  EVERY joint; in mode 7 the returned position is expected to be the
  correct joint angle.  => MODE7_ENTRY_CONTRACT_CONFIRMED = True.  The
  entry contract is id+mode only — NO physical target has to be derived
  from MODE_NULL feedback, so the previous single-gate blocker model
  (SAFE_INITIAL_MODE7_TARGET_SOURCE_VALIDATED) is retired.

Two INDEPENDENT states now govern everything:

  MODE7_ENTRY_CONTRACT_CONFIRMED = True
      allows building/sending the full-hand mode7 FEEDBACK probe frame.
  MODE7_PHYSICAL_FEEDBACK_HARDWARE_VALIDATED = False
      still BLOCKS active movement, canonical measured state, dataset
      training, AVP, and H2 until the real hardware probe passes.

Probe frame design (feedback-only, deterministic):
  Hand_Control_t ctrl{}  — zero-initialized, then per joint:
      id = 1..11, mode = MODE_JOINT_POSITION(7),
      position = 0.0, p = 0, i = 0, d = 0, output_limit = 0.
  With kp=ki=kd=0 and output_limit=0 this is a zero-output configuration
  intended to produce no active drive.  The host cannot prove firmware
  actuation semantics from those fields, so no-MOTION remains a probe
  RESULT to verify (operator confirmation + hard-fail monitoring), never
  a software assumption.

ABSOLUTE PROHIBITION: MODE_NULL status.position must NEVER be copied
into a mode7 position target (observed accumulated values: 1854 deg,
42604.57 deg).  No startup-offset derivation, no clamping workaround.
Sign/range characterization stays out of this probe; if mode7 feedback is
validated, future sign work happens in the MODE7 domain, not the
MODE_NULL accumulated domain.

Phases:
  A (hardware, proven-safe baseline): connect -> MODE_NULL x11 ->
     SwitchToRealtimeMode -> ~100 Hz status, fault 0, no movement ->
     record last MODE_NULL positions per joint (diagnostics only).
  B (offline, always available): construct the deterministic full-hand
     mode7 Hand_Control_t and dump the complete serialized preflight
     (id/mode/position/p/i/d/output_limit for all 11 joints).
  C (hardware, dual opt-in --execute --enter-mode7): send the full-hand
     mode7 frame ONCE through the native SetHandControl path (one API
     call updates cached_control_; the vendor CommScheduler re-transmits
     it at ~100 Hz — no custom sender), then run the BOUNDED TRANSITION
     state machine MODE7_COMMAND_SUBMITTED -> WAITING_FOR_MODE7_RX ->
     MODE7_STABLE / TIMEOUT.  The first real run (2026-09-03) proved
     frame 0 can legitimately report mode0 x11 (the known asynchronous
     transient), so mode0 inside the transition window is ALLOWED. Five
     consecutive all-11-mode7 fresh frames mark MODE7_STABLE immediately;
     passive observation then continues to at least 20 total post-command
     fresh frames without any additional send. Fault, transport stall,
     and non-finite data hard-fail; a >±360-deg position on a joint whose
     own RX mode is 7 is a semantic rejection.  Mode0 accumulated values
     like 42604 deg remain allowed transitional data.  Position-domain
     evaluation uses only the final stable all-11-mode7 run.  The operator
     movement prompt ("DID THE HAND MOVE WHEN ENTERING MODE7? [y/N]";
     Enter means no movement observed)
     follows the transition; if a software fault precedes it the result
     reports NO_VISIBLE_ENTRY_MOTION = NOT_CONFIRMED, never PASS.
     Artifacts land in recording/sg100_mode7_probe_<ts>/ (metadata.json,
     transition_samples.csv with every fresh post-submit frame through
     observation completion/rejection/timeout, and
     summary.json distinguishing COMMAND_SUBMITTED / FIRST_RX /
     FIRST_MODE7_ANY / FIRST_MODE7_ALL / MODE7_STABLE / TIMEOUT).
     Never touches canonical_valid — that stays a separate later change.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import struct
import sys
import time
from pathlib import Path

import numpy as np

from sg100_teleop.robot.sg100_driver import (
    FEEDBACK_DOMAIN_MODE7_CANDIDATE,
    HAND_DOF,
    MODE_JOINT_POSITION,
    MODE_NULL,
)
from sg100_teleop.robot.sg100_sdk_driver import (
    DEFAULT_BAUDRATE,
    DEFAULT_SDK_DIR,
    SG100SDKDriver,
)

# ============================================================================
# Governing states (task section 1)
# ============================================================================
MODE7_ENTRY_CONTRACT_CONFIRMED = True
MODE7_ENTRY_CONTRACT_SOURCE = (
    "vendor guidance (user-supplied, 2026-09-03): populate the 11 joint "
    "IDs and use MODE_JOINT_POSITION=7 for every joint; in mode7 the "
    "returned position is expected to be the correct joint angle"
)
MODE7_PHYSICAL_FEEDBACK_HARDWARE_VALIDATED = False

MODE_NULL_TARGET_REUSE = "FORBIDDEN"

# Structural contract (asserted offline and at runtime).
EXPECTED_JOINT_COUNT = 11
EXPECTED_JOINT_CONTROL_SIZE = 16
EXPECTED_HAND_CONTROL_SIZE = 176

# Observation / classification thresholds.
MULTI_TURN_ABS_DEG = 360.0       # any |position| > 360 deg => multi-turn
STABILIZE_SKIP_FRAMES = 0        # input is already the final stable run
RANGE_TOLERANCE_DEG = 30.0       # plausibility margin around cfg range

# MODE7 transition state machine (task sections 3-7).  Real-run evidence
# 2026-09-03: frame 0 after the mode7 send returned mode0 x11 — the known
# asynchronous transient (vendor logs: TX stays mode7 while RX flips to
# mode0 for 1-4 frames, e.g. J5 8.92 -> mode0 42604.57 -> 8.93 deg).
# Requiring frame0 == mode7 was a probe state-machine bug, NOT evidence of
# mode7 rejection.
TRANSITION_TIMEOUT_S = 0.5        # bounded window (~50 fresh frames @100Hz)
STABLE_ALL_MODE7_FRAMES = 5       # exact MODE7_STABLE milestone
MIN_MODE7_FRAMES = STABLE_ALL_MODE7_FRAMES
MIN_POST_COMMAND_FRAMES = 20      # evidence window, not stability latency
POST_STABLE_TIMEOUT_S = 0.5       # bounded tail if feedback is slower
TRANSPORT_STALL_TIMEOUT_S = 0.1   # ~10 missed callbacks at nominal 100 Hz

STATE_COMMAND_SUBMITTED = "MODE7_COMMAND_SUBMITTED"
STATE_WAITING_FOR_MODE7_RX = "WAITING_FOR_MODE7_RX"
STATE_MODE7_STABLE = "MODE7_STABLE"
STATE_SEMANTIC_REJECTED = "MODE7_SEMANTIC_REJECTED"
STATE_TIMEOUT = "TIMEOUT"

# Target of the observation (all 11 joints are probed simultaneously).
PROBE_TARGETS = [f"J{j}" for j in range(1, 12)]

# ---------------------------------------------------------------------------
# Joint_Control_t (16 bytes, #pragma pack(1)), sg100_types.h:
#   id u8 | mode u8 | position f32 | p u16 | i u16 | d u16 | out u16 | rsv 2B
_JOINT_CONTROL_STRUCT = struct.Struct("<BBfHHHH2x")


class Mode7ProbeHardFail(RuntimeError):
    """A hard-fail condition (task section 10) was observed."""


class ModeNullTargetReuseError(RuntimeError):
    """Someone tried to derive a mode7 target from MODE_NULL feedback."""


@dataclasses.dataclass(frozen=True)
class JointSpec:
    """One Joint_Control_t worth of preflight data (SDK user degrees)."""

    sdk_joint_id: int
    mode: int
    position_deg: float
    p: int
    i: int
    d: int
    output_limit: int


# ============================================================================
# Deterministic control construction (task sections 2-4)
# ============================================================================

def assert_hand_control_contract() -> None:
    """Compile-time-equivalent structural assertions for the Python side."""

    assert HAND_DOF == EXPECTED_JOINT_COUNT, (
        f"JOINT_COUNT expected {EXPECTED_JOINT_COUNT}, got {HAND_DOF}"
    )
    assert _JOINT_CONTROL_STRUCT.size == EXPECTED_JOINT_CONTROL_SIZE
    assert (
        EXPECTED_JOINT_COUNT * EXPECTED_JOINT_CONTROL_SIZE
        == EXPECTED_HAND_CONTROL_SIZE
    )


def build_full_hand_mode7_control() -> list[JointSpec]:
    """The deterministic feedback-only probe frame.

    Mirrors the C++ semantics of ``Hand_Control_t ctrl{}`` followed by
    explicit id/mode assignment: every remaining byte is zero, so the
    serialized 176-byte frame is fully deterministic:
    position=0.0 deg, p=i=d=output_limit=0.  This is intended to produce
    no active drive, but no-motion remains a hardware acceptance result.
    Requires MODE7_ENTRY_CONTRACT_CONFIRMED.
    """

    assert_hand_control_contract()
    if not MODE7_ENTRY_CONTRACT_CONFIRMED:
        raise RuntimeError("mode7 entry contract not confirmed")
    return [
        JointSpec(
            sdk_joint_id=j,
            mode=MODE_JOINT_POSITION,
            position_deg=0.0,
            p=0,
            i=0,
            d=0,
            output_limit=0,
        )
        for j in range(1, EXPECTED_JOINT_COUNT + 1)
    ]


def build_null_control() -> list[JointSpec]:
    """Phase-A baseline frame: id=1..11, mode=MODE_NULL, zeros elsewhere."""

    assert_hand_control_contract()
    return [
        JointSpec(j, MODE_NULL, 0.0, 0, 0, 0, 0)
        for j in range(1, EXPECTED_JOINT_COUNT + 1)
    ]


def reject_mode_null_derived_target(value_deg: float, context: str = "") -> None:
    """Absolute prohibition (task section 5).

    MODE_NULL status.position (accumulated values such as 1854 deg or
    42604.57 deg) must never become a mode7 target.  This helper always
    raises so no code path can "validate its way around" the prohibition.
    """

    raise ModeNullTargetReuseError(
        f"MODE_NULL position ({value_deg} deg) must NEVER become a mode7 "
        f"target; no startup-offset derivation, no clamping workaround. "
        f"{context}"
    )


def serialize_hand_control(specs: list[JointSpec]) -> bytes:
    """Serialize 11 JointSpecs into the exact Hand_Control_t wire layout."""

    assert_hand_control_contract()
    if len(specs) != EXPECTED_JOINT_COUNT:
        raise ValueError(f"expected {EXPECTED_JOINT_COUNT} joint specs")
    ids = sorted(spec.sdk_joint_id for spec in specs)
    if ids != list(range(1, EXPECTED_JOINT_COUNT + 1)):
        raise ValueError(f"joint ids must be exactly 1..11, got {ids}")
    for spec in specs:
        for name, value in (
            ("position", spec.position_deg), ("p", spec.p), ("i", spec.i),
            ("d", spec.d), ("output_limit", spec.output_limit),
        ):
            if not np.isfinite(float(value)):
                raise ValueError(
                    f"J{spec.sdk_joint_id} {name} non-finite — "
                    "uninitialized/raw byte leakage"
                )
    return b"".join(
        _JOINT_CONTROL_STRUCT.pack(
            spec.sdk_joint_id,
            spec.mode,
            float(spec.position_deg),
            int(spec.p) & 0xFFFF,
            int(spec.i) & 0xFFFF,
            int(spec.d) & 0xFFFF,
            int(spec.output_limit) & 0xFFFF,
        )
        for spec in specs
    )


def format_preflight_table(specs: list[JointSpec]) -> str:
    """Exact id/mode/position/PID/output_limit preflight for all 11 joints.

    Nothing is hidden: the zero-initialized mode7 field values are printed
    verbatim (task section 4).
    """

    assert_hand_control_contract()
    lines = [
        "MODE7 full-hand probe preflight — Hand_Control_t (deterministic, "
        "SDK user degrees):",
        f"{'J':>3} {'mode':>4} {'position_deg':>13} {'p':>6} {'i':>6} "
        f"{'d':>6} {'output_limit':>13}",
    ]
    for spec in specs:
        lines.append(
            f"J{spec.sdk_joint_id:<2} {spec.mode:>4} {spec.position_deg:>13.4f} "
            f"{spec.p:>6} {spec.i:>6} {spec.d:>6} {spec.output_limit:>13}"
        )
    mode7 = [s for s in specs if s.mode == MODE_JOINT_POSITION]
    null = [s for s in specs if s.mode == MODE_NULL]
    lines.append(f"all 11 mode7: {'YES' if len(mode7) == 11 else 'NO'} "
                 f"(mode7={len(mode7)}, MODE_NULL={len(null)}, other="
                 f"{len(specs) - len(mode7) - len(null)})")
    lines.append(
        "remaining fields deterministic (zero-initialized): "
        "position=0.0, p=0, i=0, d=0, output_limit=0"
    )
    lines.append(
        "zero-output configuration is intended to produce no active drive; "
        "no-motion remains a hardware acceptance criterion"
    )
    lines.append(
        f"serialized size: {EXPECTED_HAND_CONTROL_SIZE} bytes "
        f"({EXPECTED_JOINT_COUNT} x {EXPECTED_JOINT_CONTROL_SIZE})"
    )
    return "\n".join(lines)


# ============================================================================
# Classification helpers (pure, offline-testable; task sections 8/9/12)
# ============================================================================

def verify_all_mode7(
    frames: list[tuple[int, list[int], list[float]]],
) -> tuple[bool, str]:
    """MODE7_STATUS_MODE check decided by RX Joint_Status.mode ONLY.

    frames: (timestamp_ns, modes[11], positions_deg[11]).  The TX command
    intent is deliberately not an input.
    """

    if len(frames) < MIN_MODE7_FRAMES:
        return False, f"only {len(frames)} frames (<{MIN_MODE7_FRAMES})"
    for index, (ts, modes, _) in enumerate(frames):
        if len(modes) != EXPECTED_JOINT_COUNT:
            return False, f"frame {index}: malformed mode list"
        bad = [
            (j + 1, m) for j, m in enumerate(modes)
            if m != MODE_JOINT_POSITION
        ]
        if bad:
            preview = ", ".join(f"J{j}=mode{m}" for j, m in bad[:4])
            return False, f"frame {index} (ts {ts}): {preview}"
    return True, f"all {len(frames)} frames report mode7 on all 11 joints"


def detect_multi_turn(positions_deg) -> bool:
    """True when any |position| exceeds 360 deg (raw multi-turn value)."""

    arr = np.abs(np.asarray(positions_deg, dtype=np.float64))
    return bool(np.any(arr > MULTI_TURN_ABS_DEG))


def classify_position_domain(
    mode7_positions_by_joint,
    user_ranges: list[tuple[float, float]] | None,
) -> str:
    """MODE7_POSITION_DOMAIN classification (task section 12).

    REJECTED  — non-finite value or any multi-turn (>360 deg) sample.
    SUPPORTED — all finite, no multi-turn, and every joint's stabilized
                median sits inside its configured user range with the
                min/max within RANGE_TOLERANCE_DEG of that range.
    AMBIGUOUS — finite and non-multi-turn but range plausibility fails
                (e.g. a joint whose excursion exceeds its configured
                range by more than the tolerance, or unknown ranges).
    """

    series = [np.asarray(p, dtype=np.float64)
              for p in mode7_positions_by_joint]
    for joint_index, values in enumerate(series):
        if not np.all(np.isfinite(values)):
            return "REJECTED"
        if detect_multi_turn(values):
            return "REJECTED"
    if user_ranges is None:
        return "AMBIGUOUS"
    for joint_index, values in enumerate(series):
        low, high = user_ranges[joint_index]
        stabilized = values[min(STABILIZE_SKIP_FRAMES, len(values) - 1):]
        median = float(np.median(stabilized))
        if not (low <= median <= high):
            return "AMBIGUOUS"
        if (float(values.min()) < low - RANGE_TOLERANCE_DEG
                or float(values.max()) > high + RANGE_TOLERANCE_DEG):
            return "AMBIGUOUS"
    return "PHYSICAL_JOINT_ANGLE_SUPPORTED"


def summarize_joints(
    null_last_deg,
    mode7_frames: list[tuple[int, list[int], list[float]]],
    user_ranges: list[tuple[float, float]] | None,
) -> list[dict]:
    """Per-joint MODE_NULL-last vs MODE7 comparison (task section 9)."""

    summary = []
    for joint_index in range(EXPECTED_JOINT_COUNT):
        positions = np.asarray(
            [positions[joint_index] for _, _, positions in mode7_frames],
            dtype=np.float64,
        )
        stabilized = positions[
            min(STABILIZE_SKIP_FRAMES, len(positions) - 1):
        ]
        cfg = (
            [float(user_ranges[joint_index][0]),
             float(user_ranges[joint_index][1])]
            if user_ranges is not None else None
        )
        plausible = None
        if cfg is not None and np.all(np.isfinite(positions)):
            low, high = cfg
            median = float(np.median(stabilized))
            plausible = bool(
                low <= median <= high
                and float(positions.min()) >= low - RANGE_TOLERANCE_DEG
                and float(positions.max()) <= high + RANGE_TOLERANCE_DEG
                and not detect_multi_turn(positions)
            )
        summary.append({
            "joint": f"J{joint_index + 1}",
            "mode_null_last_deg": float(null_last_deg[joint_index]),
            "mode7_first_deg": float(positions[0]),
            "mode7_median_deg": float(np.median(stabilized)),
            "mode7_min_deg": float(positions.min()),
            "mode7_max_deg": float(positions.max()),
            "configured_user_range_deg": cfg,
            "multi_turn_present": bool(detect_multi_turn(positions)),
            "range_plausible": plausible,
        })
    return summary


# ============================================================================
# MODE7 transition state machine (task sections 3-10; pure, offline-testable)
# ============================================================================

@dataclasses.dataclass(frozen=True)
class TransitionFrame:
    """One FRESH (deduplicated) RX callback during the transition window."""

    index: int
    sequence: int | None
    timestamp_ns: int
    relative_time_ms: float
    modes: tuple[int, ...]
    positions_deg: tuple[float, ...]

    @property
    def mode7_count(self) -> int:
        return sum(1 for m in self.modes if m == MODE_JOINT_POSITION)

    @property
    def all_mode7(self) -> bool:
        return self.mode7_count == EXPECTED_JOINT_COUNT


class Mode7TransitionTracker:
    """Track MODE7 stability, semantic rejection, or bounded timeout.

    Rules (real-run informed):
    - frame 0 (and any frame inside the bounded window) may report mode0
      x11 — the asynchronous transport/firmware transient.  mode0 is NOT
      a failure during the window; its positions (1854 / 42604-like
      accumulated values) are recorded but never position-checked.
    - Hard fail (immediate) only for non-finite data on any joint or a fault/
      transport failure checked by the caller. Unexpected modes and a
      multi-turn (>±360 deg) value on a joint whose own RX mode is 7 are
      semantic rejections preserved as results rather than exceptions.
    - MODE7_STABLE is timestamped immediately after
      STABLE_ALL_MODE7_FRAMES consecutive fresh frames with all 11 joints
      reporting mode 7 (a single lucky frame is not stable). Passive
      observation continues afterward for position-domain evidence.
    - TIMEOUT (caller-invoked after the bounded window) without stability
      is a FAIL *result*, not an exception — evidence is preserved.
    """

    def __init__(self, stable_frames: int = STABLE_ALL_MODE7_FRAMES):
        self.state = STATE_COMMAND_SUBMITTED
        self.stable_frames_required = int(stable_frames)
        self.frames: list[TransitionFrame] = []
        self.stable_run = 0
        self.best_stable_run = 0
        self.first_mode7_any_index: int | None = None
        self.first_all_mode7_index: int | None = None
        self.stable_reached_at_index: int | None = None
        self.semantic_rejection_reason: str | None = None
        self.observation_complete = False
        self._seen_sequences: set[int] = set()
        self._seen_timestamps: set[int] = set()

    # -- deduplication -----------------------------------------------------
    def is_duplicate(
        self, timestamp_ns: int, sequence: int | None = None
    ) -> bool:
        """Duplicate callback snapshots must not consume frame/stability."""

        if (
            timestamp_ns in self._seen_timestamps
            or (
                sequence is not None
                and int(sequence) in self._seen_sequences
            )
        ):
            return True
        self._seen_timestamps.add(timestamp_ns)
        if sequence is not None:
            self._seen_sequences.add(int(sequence))
        return False

    # -- state machine -----------------------------------------------------
    def observe(
        self,
        timestamp_ns: int,
        modes,
        positions_deg,
        *,
        sequence: int | None = None,
    ) -> TransitionFrame:
        was_stable = self.stable_reached_at_index is not None
        modes_t = tuple(int(m) for m in modes)
        positions_t = tuple(float(p) for p in positions_deg)
        if len(modes_t) != EXPECTED_JOINT_COUNT:
            raise Mode7ProbeHardFail("malformed mode list in transition")
        if len(positions_t) != EXPECTED_JOINT_COUNT:
            raise Mode7ProbeHardFail("malformed position list in transition")
        # Non-finite callback data is a transport/data-integrity hard fail.
        for joint_index, position in enumerate(positions_t):
            if not np.isfinite(position):
                raise Mode7ProbeHardFail(
                    f"non-finite position J{joint_index + 1}"
                )
        index = len(self.frames)
        frame = TransitionFrame(
            index=index,
            sequence=None if sequence is None else int(sequence),
            timestamp_ns=int(timestamp_ns),
            relative_time_ms=0.0,       # filled by the caller (tx-relative)
            modes=modes_t,
            positions_deg=positions_t,
        )
        self.frames.append(frame)
        if self.state == STATE_COMMAND_SUBMITTED:
            self.state = STATE_WAITING_FOR_MODE7_RX
        if frame.mode7_count > 0 and self.first_mode7_any_index is None:
            self.first_mode7_any_index = index
        if frame.all_mode7:
            if self.first_all_mode7_index is None:
                self.first_all_mode7_index = index

        unexpected = [
            (joint_index + 1, mode)
            for joint_index, mode in enumerate(modes_t)
            if mode not in (MODE_NULL, MODE_JOINT_POSITION)
        ]
        if unexpected:
            preview = ", ".join(
                f"J{joint}=mode{mode}" for joint, mode in unexpected[:4]
            )
            self.semantic_rejection_reason = (
                f"unexpected returned joint mode(s): {preview}"
            )
            self.state = STATE_SEMANTIC_REJECTED
            return frame

        # A large value is expected in MODE_NULL, but in a mode-7 field it
        # rejects the candidate position semantics.  This is not a transport
        # exception; preserve the frame and perform normal safe cleanup.
        mode7_multiturn = [
            (joint_index + 1, position)
            for joint_index, (mode, position) in enumerate(
                zip(modes_t, positions_t)
            )
            if (
                mode == MODE_JOINT_POSITION
                and abs(position) > MULTI_TURN_ABS_DEG
            )
        ]
        if mode7_multiturn:
            joint, position = mode7_multiturn[0]
            self.semantic_rejection_reason = (
                f"multi-turn mode7 position J{joint}={position:.2f} deg "
                f"(>±{MULTI_TURN_ABS_DEG:.0f})"
            )
            self.state = STATE_SEMANTIC_REJECTED
            return frame

        if was_stable and not frame.all_mode7:
            self.semantic_rejection_reason = (
                "all-11 mode7 was lost after the stability milestone at "
                f"frame {index}"
            )
            self.state = STATE_SEMANTIC_REJECTED
            return frame

        if frame.all_mode7:
            self.stable_run += 1
            self.best_stable_run = max(
                self.best_stable_run, self.stable_run
            )
            if (
                self.stable_reached_at_index is None
                and self.stable_run >= self.stable_frames_required
            ):
                self.stable_reached_at_index = index
                self.state = STATE_MODE7_STABLE
        else:
            self.stable_run = 0
        return frame

    def set_relative_time(self, tx_monotonic_ns: int) -> None:
        for frame in self.frames:
            object.__setattr__(  # frozen dataclass field update once
                frame, "relative_time_ms",
                max(0.0, (frame.timestamp_ns - tx_monotonic_ns) * 1e-6),
            )

    def mark_timeout(self) -> None:
        if self.state not in (STATE_MODE7_STABLE, STATE_SEMANTIC_REJECTED):
            self.state = STATE_TIMEOUT

    def mark_observation_complete(self) -> None:
        if self.state == STATE_MODE7_STABLE:
            self.observation_complete = True

    def mark_post_stable_incomplete(self) -> None:
        self.semantic_rejection_reason = (
            "post-stable evidence window incomplete: captured "
            f"{len(self.frames)} fresh post-command frames "
            f"(need {MIN_POST_COMMAND_FRAMES})"
        )
        self.state = STATE_SEMANTIC_REJECTED

    @property
    def stable(self) -> bool:
        return self.state == STATE_MODE7_STABLE

    def stable_frames(self) -> list[TransitionFrame]:
        """The final consecutive all-mode7 run that established stability."""

        if self.stable_reached_at_index is None:
            return []
        start = self.stable_reached_at_index - self.stable_frames_required + 1
        stable = []
        for frame in self.frames[start:]:
            if not frame.all_mode7:
                break
            stable.append(frame)
        return stable

    # -- reporting ---------------------------------------------------------
    def history_lines(self) -> list[str]:
        lines = [
            "transition history (fresh frames only):",
            f"{'frame':>5} {'t_ms':>8}  modes J1..J11",
        ]
        for frame in self.frames:
            modes = ",".join(str(m) for m in frame.modes)
            lines.append(
                f"{frame.index:>5} {frame.relative_time_ms:>8.1f}  {modes}"
                f"  (mode7 x{frame.mode7_count})"
            )
        return lines

    def summary(self) -> dict:
        def rel(index: int | None) -> float | None:
            if index is None:
                return None
            return self.frames[index].relative_time_ms

        return {
            "COMMAND_SUBMITTED": True,
            "frames_recorded": len(self.frames),
            "FIRST_RX": rel(0) if self.frames else None,
            "FIRST_MODE7_ANY": rel(self.first_mode7_any_index),
            "FIRST_MODE7_ALL": rel(self.first_all_mode7_index),
            "MODE7_STABLE": rel(self.stable_reached_at_index),
            "TIMEOUT": self.state == STATE_TIMEOUT,
            "SEMANTIC_REJECTION": self.semantic_rejection_reason,
            "OBSERVATION_COMPLETE": self.observation_complete,
            "post_command_frames_target": MIN_POST_COMMAND_FRAMES,
            "stable_mode7_samples_captured": len(self.stable_frames()),
            "best_consecutive_all_mode7": self.best_stable_run,
            "state": self.state,
        }


# ============================================================================
# Phase A — proven-safe MODE_NULL baseline (hardware)
# ============================================================================

def run_phase_a(driver: SG100SDKDriver, duration_s: float) -> dict:
    if not driver.hold_passive():        # MODE_NULL x11 cache seed
        raise Mode7ProbeHardFail("MODE_NULL cache seed rejected")
    driver.enter_realtime()              # NULL seed -> 0x25 -> status flow
    deadline = time.monotonic() + duration_s
    samples = 0
    seq0 = 0
    t0 = time.monotonic()
    last = None
    while time.monotonic() < deadline:
        fb = driver.read_feedback()
        if fb is not None and fb.sequence != seq0:
            seq0 = fb.sequence
            samples += 1
            last = fb
        if driver.fault_active:
            raise Mode7ProbeHardFail("fault callback during phase A")
        time.sleep(0.002)
    elapsed = max(time.monotonic() - t0, 1e-6)
    rate_hz = samples / elapsed
    if rate_hz < 50.0:
        raise Mode7ProbeHardFail(
            f"feedback rate {rate_hz:.1f} Hz far below ~100 Hz"
        )
    if last is None or last.joint_modes is None:
        raise Mode7ProbeHardFail("no mode-bearing feedback in phase A")
    if tuple(last.joint_modes) != (MODE_NULL,) * EXPECTED_JOINT_COUNT:
        raise Mode7ProbeHardFail(
            "phase A baseline did not return MODE_NULL on all 11 joints: "
            f"{list(last.joint_modes)}"
        )
    return {
        "feedback_rate_hz": rate_hz,
        "fault": 0,
        "modes": [int(m) for m in last.joint_modes],
        "null_last_deg": np.rad2deg(last.raw_positions).tolist(),
        "domain": last.feedback_domain,
    }


# ============================================================================
# Phase C — future hardware: send once, observe, hard-fail, classify
# ============================================================================

def _probe_driver(port: str, baudrate: int, sdk_dir: Path) -> SG100SDKDriver:
    """Construct the driver used by the fixed zero-output feedback probe."""

    return SG100SDKDriver(
        port,
        baudrate=baudrate,
        sdk_dir=sdk_dir,
        control_mode=MODE_JOINT_POSITION,
        kp=0.0, ki=0.0, kd=0.0, output_limit=0.0,
    )


def _observe_transition(
    driver: SG100SDKDriver,
    timeout_s: float = TRANSITION_TIMEOUT_S,
    *,
    submitted_monotonic_ns: int = 0,
) -> Mode7TransitionTracker:
    """Run the bounded mode7 transition window (task sections 3-7).

    One SetHandControl invocation already updated the SDK cache; the
    vendor CommScheduler re-transmits it at ~100 Hz (verified SDK
    semantics — no custom sender here).  We only WATCH the RX side:
    fresh (sequence-deduplicated) frames feed the tracker; mode0 frames
    are allowed before stability. The stability latency is fixed at the
    fifth consecutive all-mode7 frame, then observation continues without
    sending until at least MIN_POST_COMMAND_FRAMES total fresh frames exist.
    Loss of all-mode7 after stability is rejection. Fault callback,
    transport stall, and non-finite data hard-fail; unexpected modes or
    mode-gated multi-turn values return a semantic-rejection state. Timeout
    is a FAIL result, not an exception.
    """

    tracker = Mode7TransitionTracker()
    transition_deadline = time.monotonic() + timeout_s
    post_stable_deadline: float | None = None
    last_progress = time.monotonic()
    last_sequence: int | None = None
    try:
        while True:
            now = time.monotonic()
            if driver.fault_active:
                raise Mode7ProbeHardFail(
                    "joint fault callback during transition"
                )
            fb = driver.read_feedback()
            if (
                fb is not None
                and fb.joint_modes is not None
                and fb.sequence != last_sequence
            ):
                last_sequence = fb.sequence
                # read_feedback() may initially return its cached Phase-A
                # snapshot.  It is not evidence about the submitted mode-7
                # command.  Only callbacks strictly after SetHandControl
                # returned are admitted to the tracker and latency metrics.
                if fb.received_monotonic_ns > submitted_monotonic_ns:
                    if not tracker.is_duplicate(
                        fb.received_monotonic_ns, fb.sequence
                    ):
                        tracker.observe(
                            fb.received_monotonic_ns,
                            fb.joint_modes,
                            np.rad2deg(fb.raw_positions),
                            sequence=fb.sequence,
                        )
                        last_progress = now

            after_observe = time.monotonic()
            if driver.fault_active:
                raise Mode7ProbeHardFail(
                    "joint fault callback during transition/observation"
                )
            if (
                tracker.state == STATE_MODE7_STABLE
                and post_stable_deadline is None
            ):
                # The fifth consecutive all-mode7 frame fixes the exact
                # stability latency. Observation then continues passively to
                # collect a useful position-domain sample window.
                post_stable_deadline = (
                    after_observe + POST_STABLE_TIMEOUT_S
                )
            if tracker.state == STATE_SEMANTIC_REJECTED:
                break
            if tracker.state == STATE_MODE7_STABLE:
                if len(tracker.frames) >= MIN_POST_COMMAND_FRAMES:
                    tracker.mark_observation_complete()
                    break
                if (
                    post_stable_deadline is not None
                    and after_observe >= post_stable_deadline
                ):
                    tracker.mark_post_stable_incomplete()
                    break
            elif after_observe >= transition_deadline:
                break
            if now - last_progress >= TRANSPORT_STALL_TIMEOUT_S:
                raise Mode7ProbeHardFail(
                    "transport failure: fresh status sequence stalled"
                )
            time.sleep(0.001)
    except BaseException as exc:
        tracker.set_relative_time(submitted_monotonic_ns)
        setattr(exc, "_mode7_transition_tracker", tracker)
        raise
    tracker.mark_timeout()
    tracker.set_relative_time(submitted_monotonic_ns)
    return tracker


def run_phase_c(
    driver: SG100SDKDriver,
    user_ranges,
    confirm_fn=input,
) -> dict:
    """Send the full-hand mode7 frame ONCE, then observe the transition.

    Returns a result dict carrying the transition summary, the tracker
    (for artifacts), TX/cache audit counters, and the classification.  On
    a hard failure the caller records NO_VISIBLE_ENTRY_MOTION=
    NOT_CONFIRMED (the operator was never asked).
    """

    result: dict = {
        "status": None,
        "fault_status": "NO_FAULT_OBSERVED",
        "NO_VISIBLE_ENTRY_MOTION": "NOT_CONFIRMED",
        "_tracker": None,
    }
    submitted = False
    try:
        print(format_preflight_table(build_full_hand_mode7_control()))
        print()
        print("MODE7_ENTRY_CONTRACT_CONFIRMED="
              f"{MODE7_ENTRY_CONTRACT_CONFIRMED}")
        print("MODE7_PHYSICAL_FEEDBACK_HARDWARE_VALIDATED="
              f"{MODE7_PHYSICAL_FEEDBACK_HARDWARE_VALIDATED}")
        print("MODE_NULL target reuse: FORBIDDEN (never performed)")
        answer = confirm_fn(
            "SEND the full-hand mode7 frame now? Type YES to continue: "
        )
        if answer.strip().upper() != "YES":
            print("[ABORT] operator declined the mode7 send")
            result["status"] = "OPERATOR_ABORT_BEFORE_SEND"
            return result

        # This diagnostic API bypasses canonical mapping/clamping and
        # production gains.  It invokes the existing wrapper once with SDK
        # position=0.0 deg, mode=7, and p=i=d=output_limit=0 for all joints.
        diag_before = driver.diagnostics["hand_control_calls"]
        submission = driver.send_mode7_feedback_probe()
        if submission is None:
            raise Mode7ProbeHardFail(
                "SetHandControl rejected the mode7 feedback-probe frame"
            )
        submitted = True
        diag_after = driver.diagnostics["hand_control_calls"]
        application_calls = diag_after - diag_before
        result["tx_audit"] = {
            "native_path": (
                "SG100SDKDriver.send_mode7_feedback_probe -> "
                "sg100_set_hand_control -> "
                "SG100Device::SetHandControl(Hand_Control_t)"
            ),
            "set_hand_control_calls_this_send": application_calls,
            "latest_cached_requested_joint_modes": list(
                submission.requested_modes
            ),
            "requested_positions_deg": list(
                submission.requested_positions_deg
            ),
            "p": submission.p,
            "i": submission.i,
            "d": submission.d,
            "output_limit": submission.output_limit,
            "scheduler_alive": False,
            "transition_window_s": TRANSITION_TIMEOUT_S,
        }
        if application_calls != 1:
            raise Mode7ProbeHardFail(
                "expected exactly one application SetHandControl update, "
                f"observed {application_calls}"
            )

        try:
            tracker = _observe_transition(
                driver,
                submitted_monotonic_ns=submission.sent_monotonic_ns,
            )
        except BaseException as exc:
            tracker = getattr(exc, "_mode7_transition_tracker", None)
            if tracker is not None:
                result["_tracker"] = tracker
                result.update(tracker.summary())
                result["tx_audit"]["scheduler_alive"] = bool(
                    tracker.frames
                )
            raise

        result["_tracker"] = tracker
        result["tx_audit"]["scheduler_alive"] = bool(tracker.frames)
        summary = tracker.summary()
        result.update(summary)

        print()
        for line in tracker.history_lines():
            print(line)

        if tracker.state == STATE_SEMANTIC_REJECTED:
            print(f"[FAIL] {tracker.semantic_rejection_reason}")
            result["status"] = "MODE7_SEMANTIC_REJECTION_FAIL"
            result["MODE7_STATUS_MODE"] = "FAIL"
            result["MODE7_POSITION_DOMAIN"] = "REJECTED"
            return result

        if tracker.state != STATE_MODE7_STABLE:
            print(
                f"[FAIL] mode7 not stable within "
                f"{TRANSITION_TIMEOUT_S * 1e3:.0f} ms: best consecutive "
                f"all-mode7 run = {tracker.best_stable_run} "
                f"(need {tracker.stable_frames_required}); first "
                f"all-mode7 frame: "
                f"{'NONE' if tracker.first_all_mode7_index is None else tracker.first_all_mode7_index}"
            )
            result["status"] = "TRANSITION_TIMEOUT_FAIL"
            result["MODE7_STATUS_MODE"] = "FAIL"
            result["MODE7_POSITION_DOMAIN"] = "NOT_EVALUATED"
            return result

        # Only the final consecutive run that established stability is used;
        # transition mode0 and earlier isolated mode7 frames are excluded.
        stable = tracker.stable_frames()
        print(
            f"[OK] mode7 stability milestone established by "
            f"{tracker.stable_frames_required} consecutive fresh all-mode7 "
            f"frames (first at {summary['FIRST_MODE7_ALL']:.3f} ms, "
            f"stable at {summary['MODE7_STABLE']:.3f} ms); passive "
            f"observation complete with {len(tracker.frames)} total fresh "
            f"post-command frames and {len(stable)} stable-mode7 samples"
        )
        domain = classify_position_domain(
            [[f.positions_deg[j] for f in stable]
             for j in range(EXPECTED_JOINT_COUNT)],
            user_ranges,
        )
        movement_answer = confirm_fn(
            "DID THE HAND MOVE WHEN ENTERING MODE7? [y/N] "
        ).strip().lower()
        if movement_answer in ("y", "yes"):
            no_motion: bool | None = False
        elif movement_answer in ("", "n", "no"):
            no_motion = True
        else:
            no_motion = None
        if no_motion is None:
            status = "OPERATOR_MOVEMENT_RESPONSE_INVALID"
        elif not no_motion:
            status = "OPERATOR_MOVEMENT_FAIL"
        elif domain != "PHYSICAL_JOINT_ANGLE_SUPPORTED":
            status = "MODE7_POSITION_DOMAIN_NOT_SUPPORTED"
        else:
            status = "COMPLETED_PASS"
        result.update({
            "status": status,
            "MODE7_STATUS_MODE": "PASS",
            "MODE7_POSITION_DOMAIN": domain,
            "NO_VISIBLE_ENTRY_MOTION": (
                "NOT_CONFIRMED" if no_motion is None
                else ("PASS" if no_motion else "FAIL")
            ),
            "MODE7_PHYSICAL_FEEDBACK_HARDWARE_VALIDATED": (
                MODE7_PHYSICAL_FEEDBACK_HARDWARE_VALIDATED
            ),
            "canonical_valid_autoflip": "NOT_PERFORMED",
            "_stable_positions": [
                [f.positions_deg[j] for f in stable]
                for j in range(EXPECTED_JOINT_COUNT)
            ],
        })
        return result
    except BaseException as exc:
        setattr(exc, "_mode7_phase_c_result", result)
        raise
    finally:
        if submitted:
            try:
                result["mode_null_restore_after_mode7"] = bool(
                    driver.hold_passive()
                )
            except BaseException as cleanup_exc:
                result["mode_null_restore_after_mode7"] = False
                result["mode_null_restore_error"] = repr(cleanup_exc)


def _write_artifacts(
    artifact_dir: Path,
    report: dict,
    tracker: Mode7TransitionTracker | None,
) -> None:
    """Persist transition evidence whether the probe passed or failed."""

    artifact_dir.mkdir(parents=True, exist_ok=True)
    (artifact_dir / "metadata.json").write_text(
        json.dumps(
            {
                k: v for k, v in report.items() if k != "phase_c"
            },
            indent=2,
            default=str,
        )
    )
    summary = dict(report.get("phase_c", {}))
    summary.pop("_tracker", None)
    summary.pop("_stable_positions", None)
    (artifact_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, default=str)
    )
    lines = [
        "index,sequence,timestamp_ns,relative_time_ms,"
        + ",".join(f"J{j}_mode" for j in range(1, 12))
        + ","
        + ",".join(f"J{j}_pos_deg" for j in range(1, 12))
    ]
    if tracker is not None:
        for frame in tracker.frames:
            lines.append(
                f"{frame.index},"
                f"{'' if frame.sequence is None else frame.sequence},"
                f"{frame.timestamp_ns},"
                f"{frame.relative_time_ms:.3f},"
                + ",".join(str(m) for m in frame.modes)
                + ","
                + ",".join(f"{p:.4f}" for p in frame.positions_deg)
            )
    (artifact_dir / "transition_samples.csv").write_text(
        "\n".join(lines) + "\n"
    )


def run_full_probe(
    port: str,
    baudrate: int,
    sdk_dir: Path,
    duration_s: float,
    enter_mode7: bool,
    artifact_dir: Path | None = None,
    confirm_fn=input,
) -> int:
    driver = _probe_driver(port, baudrate, sdk_dir)
    report: dict = {
        "port": port,
        "schema": "sg100_mode7_feedback_probe_v4",
        "MODE7_ENTRY_CONTRACT_CONFIRMED": MODE7_ENTRY_CONTRACT_CONFIRMED,
        "MODE7_PHYSICAL_FEEDBACK_HARDWARE_VALIDATED": (
            MODE7_PHYSICAL_FEEDBACK_HARDWARE_VALIDATED
        ),
    }
    phase_c: dict | None = None
    try:
        driver.connect()
        print("[PHASE A] MODE_NULL baseline")
        phase_a = run_phase_a(driver, duration_s)
        report["phase_a"] = phase_a
        print(json.dumps(phase_a, indent=2))
        if not enter_mode7:
            print("[OK] phase A complete (mode7 entry not requested)")
            return 0
        if not MODE7_ENTRY_CONTRACT_CONFIRMED:
            print("[ABORT] mode7 entry contract not confirmed")
            return 4
        print()
        print("[PHASE C] full-hand mode7 feedback observation")
        user_ranges = driver.vendor_user_ranges
        phase_c = run_phase_c(driver, user_ranges, confirm_fn=confirm_fn)
        tracker = phase_c.get("_tracker")
        if phase_c.get("_stable_positions") is not None and tracker:
            phase_c["per_joint"] = summarize_joints(
                phase_a["null_last_deg"],
                [
                    (f.timestamp_ns, list(f.modes), list(f.positions_deg))
                    for f in tracker.stable_frames()
                ],
                user_ranges,
            )
        report["phase_c"] = phase_c
        print(json.dumps(
            {k: v for k, v in phase_c.items() if not k.startswith("_")},
            indent=2, default=str,
        ))
        return {
            "COMPLETED_PASS": 0,
            "OPERATOR_ABORT_BEFORE_SEND": 4,
            "TRANSITION_TIMEOUT_FAIL": 6,
            "MODE7_SEMANTIC_REJECTION_FAIL": 7,
            "OPERATOR_MOVEMENT_FAIL": 8,
            "MODE7_POSITION_DOMAIN_NOT_SUPPORTED": 9,
            "OPERATOR_MOVEMENT_RESPONSE_INVALID": 10,
        }.get(str(phase_c.get("status")), 9)
    except Mode7ProbeHardFail as exc:
        # Software fault BEFORE the movement prompt: the operator was
        # never asked, so no-visible-motion is NOT_CONFIRMED, never PASS.
        print(f"[HARD FAIL] {exc}")
        phase_c = getattr(exc, "_mode7_phase_c_result", phase_c) or {}
        tracker = getattr(exc, "_mode7_transition_tracker", None)
        if tracker is not None and phase_c.get("_tracker") is None:
            phase_c["_tracker"] = tracker
            phase_c.update(tracker.summary())
        phase_c["status"] = "HARD_FAIL"
        phase_c["hard_fail_reason"] = str(exc)
        if "fault callback" in str(exc):
            phase_c["fault_status"] = "FAULT_CALLBACK_ACTIVE"
        phase_c["NO_VISIBLE_ENTRY_MOTION"] = "NOT_CONFIRMED"
        report["phase_c"] = phase_c
        return 5
    except EOFError as exc:
        print("[ABORT] operator input reached EOF")
        phase_c = getattr(exc, "_mode7_phase_c_result", phase_c) or {}
        phase_c["status"] = "OPERATOR_INPUT_EOF_ABORT"
        phase_c["NO_VISIBLE_ENTRY_MOTION"] = "NOT_CONFIRMED"
        report["phase_c"] = phase_c
        return 10
    except KeyboardInterrupt as exc:
        print("[ABORT] keyboard interrupt")
        phase_c = getattr(exc, "_mode7_phase_c_result", phase_c) or {}
        phase_c["status"] = "KEYBOARD_INTERRUPT_ABORT"
        phase_c["NO_VISIBLE_ENTRY_MOTION"] = "NOT_CONFIRMED"
        report["phase_c"] = phase_c
        return 130
    except Exception as exc:
        print(f"[UNEXPECTED FAIL] {type(exc).__name__}: {exc}")
        phase_c = getattr(exc, "_mode7_phase_c_result", phase_c) or {}
        phase_c["status"] = "UNEXPECTED_EXCEPTION"
        phase_c["exception_type"] = type(exc).__name__
        phase_c["exception_reason"] = str(exc)
        phase_c["NO_VISIBLE_ENTRY_MOTION"] = "NOT_CONFIRMED"
        report["phase_c"] = phase_c
        return 11
    finally:
        cleanup: dict[str, object] = {}
        try:
            cleanup["hold_passive"] = bool(driver.hold_passive())
        except BaseException as exc:
            cleanup["hold_passive"] = False
            cleanup["hold_passive_error"] = repr(exc)
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
        if artifact_dir is not None:
            tracker_for_artifact = (
                phase_c.get("_tracker") if phase_c else None
            )
            _write_artifacts(artifact_dir, report, tracker_for_artifact)


# ============================================================================
# CLI
# ============================================================================

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "SG100 FULL-HAND MODE7 feedback probe (prepare-only by "
            "default; hardware needs --port + --execute)"
        )
    )
    parser.add_argument("--port", default=None,
                        help="serial port (required with --execute)")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--sdk-dir", type=Path, default=DEFAULT_SDK_DIR)
    parser.add_argument("--duration", type=float, default=5.0,
                        help="phase A observation seconds")
    parser.add_argument("--execute", action="store_true",
                        help="open the serial port (phase A)")
    parser.add_argument("--enter-mode7", action="store_true",
                        help="ALSO run phase C (send the mode7 frame once;"
                             " requires interactive operator confirmations)")
    parser.add_argument("--explain", action="store_true",
                        help="print the governing states and exit")
    args = parser.parse_args(argv)

    assert_hand_control_contract()

    if args.explain:
        print(f"MODE7_ENTRY_CONTRACT_CONFIRMED="
              f"{MODE7_ENTRY_CONTRACT_CONFIRMED}")
        print(f"  source: {MODE7_ENTRY_CONTRACT_SOURCE}")
        print("MODE7_PHYSICAL_FEEDBACK_HARDWARE_VALIDATED="
              f"{MODE7_PHYSICAL_FEEDBACK_HARDWARE_VALIDATED}"
              " (active motion / canonical / dataset / AVP / H2 stay BLOCKED)")
        print(f"MODE_NULL target reuse: {MODE_NULL_TARGET_REUSE}")
        print("canonical_valid autoflip after probe: NOT_PERFORMED")
        print(f"transition policy: timeout {TRANSITION_TIMEOUT_S * 1e3:.0f} ms, "
              f">= {STABLE_ALL_MODE7_FRAMES} consecutive all-11-mode7 fresh "
              f"frames mark MODE7_STABLE; passive capture continues to >= "
              f"{MIN_POST_COMMAND_FRAMES} total fresh post-command frames; "
              f"mode0 transitional frames ALLOWED before stability")
        return 0

    if not args.execute:
        # Dry-run: ZERO hardware access; print the deterministic preflight.
        print("[DRY-RUN] nothing executed; hardware requires "
              "--port PORT --execute (phase C adds --enter-mode7).")
        print()
        print(format_preflight_table(build_full_hand_mode7_control()))
        return 0

    if not args.port:
        parser.error("--port is required with --execute")
    from datetime import datetime
    artifact_dir = (
        Path("recording")
        / f"sg100_mode7_probe_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    return run_full_probe(
        args.port, args.baudrate, args.sdk_dir, args.duration,
        enter_mode7=args.enter_mode7,
        artifact_dir=artifact_dir,
    )


if __name__ == "__main__":
    sys.exit(main())
