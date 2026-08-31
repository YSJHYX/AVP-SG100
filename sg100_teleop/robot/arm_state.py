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
    command_bundle_id: int | None = None

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
    controller_tcp: np.ndarray | None = None

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
        if self.controller_tcp is not None:
            tcp = np.asarray(self.controller_tcp, dtype=np.float64)
            if tcp.shape != (6,):
                raise ValueError("ArmState.controller_tcp must have shape (6,)")
            if not np.all(np.isfinite(tcp)):
                raise ValueError("ArmState.controller_tcp contains non-finite values")


@dataclass(frozen=True)
class ArmCommandSent:
    """One limiter-bounded command successfully accepted by RM65 CANFD.

    This is the canonical arm-action event.  It is emitted only after
    ``rm_movej_canfd`` returns success, and therefore must not be confused
    with a producer target, IK goal, or pre-limiter command.
    """

    q_rad: np.ndarray
    limiter_input_q_rad: np.ndarray
    sent_monotonic_ns: int
    source_timestamp_ns: int
    target_sequence: int
    command_bundle_id: int | None

    def validate(self) -> None:
        for name, value in (
            ("q_rad", self.q_rad),
            ("limiter_input_q_rad", self.limiter_input_q_rad),
        ):
            vector = np.asarray(value, dtype=np.float64)
            if vector.shape != (ARM_DOF,):
                raise ValueError(f"ArmCommandSent.{name} must have shape ({ARM_DOF},)")
            if not np.all(np.isfinite(vector)):
                raise ValueError(f"ArmCommandSent.{name} contains non-finite values")
        if self.sent_monotonic_ns <= 0 or self.source_timestamp_ns <= 0:
            raise ValueError("ArmCommandSent timestamps must be positive")
