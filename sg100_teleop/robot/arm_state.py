"""Dataclasses exchanged between the teleop producer and arm backends."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


ARM_DOF = 6


@dataclass(frozen=True)
class ArmTarget:
    """One joint-space target published by the 60 Hz teleop producer.

    ``source_timestamp_ns`` is the producer's ``time.monotonic_ns()`` at
    publication; the sender uses it to detect stale targets instead of
    re-sending an old one (v3 §A3.4).  ``sequence`` is monotonically
    increasing so the sender can tell "already sent" from "new".
    """

    q_rad: np.ndarray
    source_timestamp_ns: int
    sequence: int

    def validate(self) -> None:
        q = np.asarray(self.q_rad, dtype=np.float64)
        if q.shape != (ARM_DOF,):
            raise ValueError(
                f"ArmTarget.q_rad must have shape ({ARM_DOF},), "
                f"got {q.shape}"
            )
        if not np.all(np.isfinite(q)):
            raise ValueError("ArmTarget.q_rad contains non-finite values")


@dataclass(frozen=True)
class ArmState:
    """One feedback sample from the arm (v3 §A5.1).

    ``qd_rad_s`` is only available from the UDP realtime push; the TCP
    polling fallback leaves it ``None``.  ``controller_error`` is the
    parsed controller error code (``None`` when unavailable).
    """

    q_rad: np.ndarray
    qd_rad_s: np.ndarray | None
    received_monotonic_ns: int
    sequence: int
    controller_error: int | None
    arm_status: int | None
    valid: bool

    def validate(self) -> None:
        q = np.asarray(self.q_rad, dtype=np.float64)
        if q.shape != (ARM_DOF,):
            raise ValueError(
                f"ArmState.q_rad must have shape ({ARM_DOF},), "
                f"got {q.shape}"
            )
        if not np.all(np.isfinite(q)):
            raise ValueError("ArmState.q_rad contains non-finite values")
        if self.qd_rad_s is not None:
            qd = np.asarray(self.qd_rad_s, dtype=np.float64)
            if qd.shape != (ARM_DOF,):
                raise ValueError("ArmState.qd_rad_s must match q_rad shape")
            if not np.all(np.isfinite(qd)):
                raise ValueError("ArmState.qd_rad_s contains non-finite values")
