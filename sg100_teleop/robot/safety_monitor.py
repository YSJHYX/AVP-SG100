"""Safety watchdogs and the ordered stop sequence (v3 §A3.5).

Dead-arm detection is VELOCITY-based (v3 §A3.3 failure taxonomy): a
lagging arm is moving (healthy physics of the low-follow mode, however
large its time constant), a dead arm is not.  This makes the detector
immune to tracking-lag changes and removes the need to model the lag
with position thresholds.  The position check survives only as a wide
gross-mismatch bound.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from sg100_teleop.robot.arm_state import ARM_DOF, ArmState, ArmTarget


@dataclass(frozen=True)
class SafetyTrigger:
    """Why the stop sequence fired; surfaced to logs and the operator."""

    name: str
    detail: str


class SafetyMonitor:
    """Pure checks over target/feedback freshness and arm liveness.

    Instances hold no threads and perform no I/O; the real backend runs
    ``check()`` inside its sender thread so that the thread which stops
    streaming is the thread which streams (v3 §A3.5).

    Triggers, in evaluation order:

    1. ``target_missing`` / ``target_stale`` — no target, or the newest
       target is older than ``target_timeout_s``;
    2. ``feedback_stale`` — no valid feedback, or older than
       ``feedback_timeout_s``;
    3. ``controller_error`` — non-zero controller error code;
    4. ``arm_not_moving`` — the command trajectory has been moving
       faster than ``velocity_fault_threshold_rad_s`` for
       ``velocity_fault_window_s`` while the measured arm velocity
       stayed below ``velocity_fault_ratio`` of the commanded one
       (dead stream / stalled arm);
    5. ``follow_error`` — gross position mismatch beyond
       ``gross_follow_error_rad`` (last-resort bound; ordinary tracking
       lag never reaches it).
    """

    def __init__(
        self,
        target_timeout_s: float,
        feedback_timeout_s: float,
        gross_follow_error_rad: float = 0.35,
        velocity_fault_threshold_rad_s: float = 0.1,
        velocity_fault_ratio: float = 0.3,
        velocity_fault_window_s: float = 0.3,
    ) -> None:
        if target_timeout_s <= 0.0 or feedback_timeout_s <= 0.0:
            raise ValueError("watchdog timeouts must be positive")
        if gross_follow_error_rad <= 0.0:
            raise ValueError("gross_follow_error_rad must be positive")
        if velocity_fault_threshold_rad_s <= 0.0:
            raise ValueError("velocity_fault_threshold_rad_s must be positive")
        if not 0.0 < velocity_fault_ratio < 1.0:
            raise ValueError("velocity_fault_ratio must be in (0, 1)")
        if velocity_fault_window_s <= 0.0:
            raise ValueError("velocity_fault_window_s must be positive")

        self.target_timeout_s = float(target_timeout_s)
        self.feedback_timeout_s = float(feedback_timeout_s)
        self.gross_follow_error_rad = float(gross_follow_error_rad)
        self.velocity_fault_threshold_rad_s = float(
            velocity_fault_threshold_rad_s
        )
        self.velocity_fault_ratio = float(velocity_fault_ratio)
        self.velocity_fault_window_s = float(velocity_fault_window_s)
        self._velocity_fault_elapsed_s = 0.0

    def reset_follow_error_state(self) -> None:
        """Clear liveness bookkeeping (call on re-arm)."""
        self._velocity_fault_elapsed_s = 0.0

    def check(
        self,
        *,
        now_monotonic_ns: int,
        latest_target: ArmTarget | None,
        feedback: ArmState | None,
        last_cmd_rad: np.ndarray | None,
        commanded_velocity_rad_s: np.ndarray | None = None,
        actual_velocity_rad_s: np.ndarray | None = None,
        dt_s: float | None = None,
    ) -> SafetyTrigger | None:
        if latest_target is None:
            return SafetyTrigger("target_missing", "no target published")
        target_age_s = (
            now_monotonic_ns - latest_target.source_timestamp_ns
        ) * 1e-9
        if target_age_s > self.target_timeout_s:
            return SafetyTrigger(
                "target_stale", f"target age {target_age_s:.3f} s"
            )

        if feedback is None or not feedback.valid:
            return SafetyTrigger(
                "feedback_stale", "no valid feedback sample"
            )
        feedback_age_s = (
            now_monotonic_ns - feedback.received_monotonic_ns
        ) * 1e-9
        if feedback_age_s > self.feedback_timeout_s:
            return SafetyTrigger(
                "feedback_stale", f"feedback age {feedback_age_s:.3f} s"
            )

        if feedback.controller_error not in (None, 0):
            return SafetyTrigger(
                "controller_error",
                f"controller error code {feedback.controller_error}",
            )

        # ---------------------------------------------
        # Liveness: velocity-based dead-arm detection.
        # ---------------------------------------------
        if (
            commanded_velocity_rad_s is not None
            and actual_velocity_rad_s is not None
            and dt_s is not None
            and dt_s > 0.0
        ):
            v_cmd = float(np.max(np.abs(commanded_velocity_rad_s)))
            v_actual = float(np.max(np.abs(actual_velocity_rad_s)))
            if v_cmd > self.velocity_fault_threshold_rad_s:
                if v_actual < self.velocity_fault_ratio * v_cmd:
                    self._velocity_fault_elapsed_s += dt_s
                    if (
                        self._velocity_fault_elapsed_s
                        >= self.velocity_fault_window_s
                    ):
                        return SafetyTrigger(
                            "arm_not_moving",
                            f"cmd {v_cmd:.2f} rad/s vs actual "
                            f"{v_actual:.2f} rad/s for "
                            f"{self._velocity_fault_elapsed_s:.2f} s",
                        )
                else:
                    self._velocity_fault_elapsed_s = 0.0
            else:
                self._velocity_fault_elapsed_s = 0.0

        # ---------------------------------------------
        # Gross position mismatch (last-resort bound).
        # ---------------------------------------------
        if last_cmd_rad is not None:
            error = float(
                np.max(np.abs(np.asarray(last_cmd_rad) - feedback.q_rad))
            )
            if error > self.gross_follow_error_rad:
                return SafetyTrigger(
                    "follow_error",
                    f"gross cmd-actual {error:.3f} rad",
                )
        return None


class StopSequence:
    """The v3 §A3.5 stop order; the steps must not be reordered.

    1. atomically disable the sender (no further CANFD sends),
    2. drain in-flight sends,
    3. ``rm_set_arm_slow_stop()``; retry once, escalate to the
       immediate stop on repeated failure,
    4. clear the target buffer and disarm.

    Issuing the controller stop before the sender is quiesced would let
    a later CANFD target land after the stop, whose behaviour is
    undocumented — exactly the race v3 fixes.
    """

    def __init__(
        self,
        disable_sender: Callable[[], None],
        drain_sender: Callable[[], None],
        slow_stop: Callable[[], int],
        immediate_stop: Callable[[], int],
        clear_buffer: Callable[[], None],
        on_disarm: Callable[[str], None],
    ) -> None:
        self._disable_sender = disable_sender
        self._drain_sender = drain_sender
        self._slow_stop = slow_stop
        self._immediate_stop = immediate_stop
        self._clear_buffer = clear_buffer
        self._on_disarm = on_disarm

    def execute(self, reason: str) -> None:
        self._disable_sender()
        self._drain_sender()
        if self._slow_stop() != 0:
            if self._slow_stop() != 0:
                self._immediate_stop()
        self._clear_buffer()
        self._on_disarm(reason)
