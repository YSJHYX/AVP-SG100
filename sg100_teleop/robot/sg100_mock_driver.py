"""MockSG100Driver: deterministic SG100 hardware simulation.

Purpose: unit/CI tests for the driver boundary, the command
provenance chain, and the DATA CONTRACT hand semantics — no hardware.

Model:
- a ~100 Hz "communication thread" is emulated lazily: each
  ``read_feedback`` call synthesizes the 10 ms-grained samples that
  elapsed since the previous call, so timestamps and rates behave
  like the real SDK cache;
- joint positions follow the last accepted command with a first-order
  response (tau configurable) plus measurement noise;
- fault injection knobs cover every DATA CONTRACT corner: send
  failures, NaN feedback, duplicate samples, frozen (stale) feedback,
  and out-of-order timestamps.
"""

from __future__ import annotations

import threading
import time
from collections import deque

import numpy as np

from sg100_teleop.robot.sg100_driver import (
    HAND_DOF,
    SG100CommandSent,
    SG100Driver,
    SG100HandFeedback,
    SG100HardwareMap,
)


class MockSG100Driver(SG100Driver):
    """In-process SG100 stand-in with fault injection."""

    def __init__(
        self,
        hardware_map: SG100HardwareMap | None = None,
        *,
        initial_q_rad: np.ndarray | None = None,
        follow_tau_s: float = 0.04,
        noise_std_rad: float = 0.0,
        send_failure: bool = False,
        nan_feedback: bool = False,
        duplicate_feedback: bool = False,
        frozen_feedback: bool = False,
        time_reversal: bool = False,
        feedback_period_s: float = 0.01,
        start_monotonic_ns: int | None = None,
        query_driven: bool = False,
    ) -> None:
        self.hardware_map = (
            hardware_map
            if hardware_map is not None
            else SG100HardwareMap()
        )
        self.follow_tau_s = float(follow_tau_s)
        self.noise_std_rad = float(noise_std_rad)
        self.send_failure = bool(send_failure)
        self.nan_feedback = bool(nan_feedback)
        self.duplicate_feedback = bool(duplicate_feedback)
        self.frozen_feedback = bool(frozen_feedback)
        self.time_reversal = bool(time_reversal)
        self.feedback_period_s = float(feedback_period_s)
        self.query_driven = bool(query_driven)

        self._q_actual = (
            np.zeros(HAND_DOF)
            if initial_q_rad is None
            else np.asarray(initial_q_rad, dtype=np.float64).copy()
        )
        self._q_target_actual = self._q_actual.copy()
        self._connected = False
        self._running = False
        self._realtime = False
        self._sequence = 0
        self._read_calls = 0
        self._last_feedback: SG100HandFeedback | None = None
        self._last_sample_ns = int(
            time.monotonic_ns()
            if start_monotonic_ns is None
            else start_monotonic_ns
        )
        self._last_feedback_ns = self._last_sample_ns
        self._events: deque[SG100CommandSent] = deque()
        self._events_lock = threading.Lock()
        self._rng = np.random.default_rng(20260901)
        self._status_requests_sent = 0
        self._pending_status_requests = 0

    # -- lifecycle -------------------------------------------------------

    def connect(self) -> None:
        self._connected = True

    def start(self) -> None:
        if not self._connected:
            raise RuntimeError("connect() before start()")
        self._running = True

    def stop(self) -> None:
        # Mirror the real backend's safe passive hold: MODE_NULL means no
        # drive output, so the hand stops following any previous target.
        # The transport stays alive (feedback keeps flowing) exactly like
        # the vendor scheduler continues after a MODE_NULL seed; only
        # disconnect() tears the link down.
        self.hold_passive()

    def disconnect(self) -> None:
        self._running = False
        self._realtime = False
        self._connected = False

    # -- realtime state (mirrors SG100SDKDriver.enter_realtime) -----------

    def enter_realtime(self, timeout_s: float = 2.0) -> None:
        """Enable command acceptance; entering itself never moves the hand.

        Like the real backend, entry seeds a MODE_NULL (no-drive) hold, so
        the physical pose is unchanged until the first explicit target.
        """

        if not self._connected:
            raise RuntimeError("connect() before enter_realtime()")
        if self._realtime:
            return
        self.hold_passive()
        self._realtime = True

    @property
    def realtime_active(self) -> bool:
        return self._realtime

    def hold_passive(self) -> bool:
        """MODE_NULL hold: freeze at the current pose (no drive output)."""

        self._q_target_actual = self._q_actual.copy()
        return True

    # -- feedback ----------------------------------------------------------

    def read_feedback(self) -> SG100HandFeedback | None:
        """Synthesize the elapsed ~100 Hz samples, return the newest."""

        if not self._running:
            return None
        if self.query_driven:
            if self._pending_status_requests == 0:
                return None
            self._pending_status_requests -= 1

        self._read_calls += 1

        if (
            self.duplicate_feedback or self.frozen_feedback
        ) and self._last_feedback is not None and self._read_calls % 2 == 0:
            # Emulates a cache that did not refresh: the same sample
            # (same sequence) is returned again.  Callers must dedupe
            # by sequence, never treat this as a new measurement.
            return self._last_feedback

        now_ns = time.monotonic_ns()
        elapsed_s = max(
            0.0,
            (now_ns - self._last_sample_ns) * 1e-9,
        )
        steps = max(1, int(elapsed_s / self.feedback_period_s))

        for _ in range(steps):
            if not self.frozen_feedback:
                dt = self.feedback_period_s
                alpha = 1.0 - np.exp(
                    -dt / max(self.follow_tau_s, 1e-6)
                )
                noise = (
                    self._rng.normal(
                        0.0, self.noise_std_rad, HAND_DOF
                    )
                    if self.noise_std_rad > 0.0
                    else np.zeros(HAND_DOF)
                )
                self._q_actual = self._q_actual + alpha * (
                    self._q_target_actual - self._q_actual
                ) + noise

        self._sequence += 1

        received_ns = now_ns
        if self.time_reversal and self._sequence % 5 == 0:
            # Simulates an out-of-order host timestamp.
            received_ns = now_ns - int(50e6)

        raw = self._q_actual.copy()
        if self.nan_feedback and self._sequence % 4 == 0:
            raw[3] = np.nan

        canonical = self.hardware_map.actual_to_canonical(raw)

        feedback = SG100HandFeedback(
            q_rad=canonical,
            received_monotonic_ns=received_ns,
            sequence=self._sequence,
            valid=bool(np.all(np.isfinite(canonical))),
            raw_positions=raw.copy(),
        )
        self._last_sample_ns = now_ns
        self._last_feedback = feedback
        self._last_feedback_ns = received_ns
        return feedback

    # -- command -----------------------------------------------------------

    def send_positions_rad(
        self,
        q_rad: np.ndarray,
        *,
        command_bundle_id: int | None = None,
    ) -> SG100CommandSent | None:
        if not self._running:
            return None
        if not self._realtime:
            # Fail-closed before realtime entry, mirroring the real
            # backend whose cached control stays MODE_NULL until then.
            return None

        actual = self.hardware_map.canonical_to_actual(q_rad)

        if self.send_failure:
            return None

        self._q_target_actual = actual.copy()
        sent_ns = time.monotonic_ns()
        event = SG100CommandSent(
            q_rad=self.hardware_map.clamp_canonical(
                np.asarray(q_rad, dtype=np.float64)
            ),
            sent_monotonic_ns=sent_ns,
            command_bundle_id=command_bundle_id,
        )
        with self._events_lock:
            self._events.append(event)
        return event

    def drain_command_events(self) -> tuple[SG100CommandSent, ...]:
        with self._events_lock:
            events = tuple(self._events)
            self._events.clear()
        return events

    def request_status(self) -> bool:
        if not self._running:
            return False
        self._status_requests_sent += 1
        self._pending_status_requests += 1
        return True

    @property
    def diagnostics(self) -> dict[str, int]:
        callbacks = self._sequence
        return {
            "hand_status_callbacks": callbacks,
            "status_requests_sent": self._status_requests_sent,
            "error_frames": 0,
        }

    @property
    def serial_open(self) -> bool:
        return self._connected

    @property
    def sdk_started(self) -> bool:
        return self._running

    # -- test hooks ----------------------------------------------------------

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def running(self) -> bool:
        return self._running

    @property
    def fault_active(self) -> bool:
        return self.nan_feedback

    def inject_actual_position(self, q_actual: np.ndarray) -> None:
        """Force the underlying vendor-domain position (test setup)."""

        self._q_actual = np.asarray(q_actual, dtype=np.float64).copy()
        self._q_target_actual = self._q_actual.copy()
