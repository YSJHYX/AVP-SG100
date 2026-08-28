"""Thread-safe single-slot latest-target buffer (v3 §A3.4)."""

from __future__ import annotations

import threading
import time

from sg100_teleop.robot.arm_state import ArmTarget


class LatestTargetBuffer:
    """Hold only the newest ``ArmTarget``; older targets are dropped.

    The IK producer publishes at ~60 Hz while the CANFD sender consumes
    at 100-200 Hz, so the sender must never queue or replay targets.
    Stale publishes (non-increasing ``sequence``) are rejected so a
    late producer thread can not resurrect an old target after a
    disarm/clear.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._target: ArmTarget | None = None

    def publish(self, target: ArmTarget) -> bool:
        """Atomically replace the stored target.

        Returns ``True`` when accepted, ``False`` when rejected because
        ``sequence`` does not strictly increase.
        """
        target.validate()
        with self._lock:
            if self._target is not None and target.sequence <= self._target.sequence:
                return False
            self._target = target
            return True

    def latest(self) -> ArmTarget | None:
        with self._lock:
            return self._target

    def age_s(self, now_monotonic_ns: int | None = None) -> float | None:
        """Age of the stored target in seconds, ``None`` when empty."""
        with self._lock:
            if self._target is None:
                return None
        target = self._target
        if now_monotonic_ns is None:
            now_monotonic_ns = time.monotonic_ns()
        return (now_monotonic_ns - target.source_timestamp_ns) * 1e-9

    def clear(self) -> None:
        with self._lock:
            self._target = None
