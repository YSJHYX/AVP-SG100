"""Joint-space q / qdot / qddot trajectory limiter (v3 §A3.2).

CANFD passthrough has no planner and no speed parameter; the software
command trajectory itself is the only speed/acceleration limit on the
real arm.  This limiter turns any upstream target (including
discontinuous HOLD-resume targets) into a bounded, C1-smooth command
sequence using the *measured* sender period.
"""

from __future__ import annotations

import numpy as np

from sg100_teleop.robot.arm_state import ARM_DOF

# The measured dt is clamped into this window before use: a near-zero
# dt (duplicate tick) would inflate the velocity gain, and a huge dt
# (thread stall) would let one step travel far.
MIN_DT_S = 1e-3
MAX_DT_S = 5e-2


def _as_joint_vector(value: np.ndarray, label: str) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (ARM_DOF,):
        raise ValueError(
            f"{label} must have shape ({ARM_DOF},), got {vector.shape}"
        )
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{label} contains non-finite values")
    return vector


class JointTrajectorySafetyLimiter:
    """Bounded-speed integrator from targets to commands.

    Per update, with measured dt:

    1. desired velocity ``v_des = (target - q) / dt``, clamped to
       ``±qdot_max`` (position tracking can never slew faster);
    2. braking limit ``|v_des| <= sqrt(2 * qddot_max * |error|)`` —
       the fastest velocity that can still decelerate to zero before
       reaching the target.  Without it the accel clamp cannot slow a
       cruising command in time and the output overshoots/oscillates
       around the target;
    3. velocity change clamped to ``±qddot_max * dt`` (C1 smoothness);
    4. ``q += v * dt``, clamped into the soft joint limits
       ``[lower + margin, upper - margin]``.
    """

    def __init__(
        self,
        lower_rad: np.ndarray,
        upper_rad: np.ndarray,
        qdot_max_rad_s: float,
        qddot_max_rad_s2: float,
        soft_margin_rad: float,
    ) -> None:
        lower = _as_joint_vector(lower_rad, "lower_rad")
        upper = _as_joint_vector(upper_rad, "upper_rad")
        if np.any(upper <= lower):
            raise ValueError("upper_rad must exceed lower_rad on every joint")
        if qdot_max_rad_s <= 0.0:
            raise ValueError("qdot_max_rad_s must be positive")
        if qddot_max_rad_s2 <= 0.0:
            raise ValueError("qddot_max_rad_s2 must be positive")
        if soft_margin_rad < 0.0:
            raise ValueError("soft_margin_rad cannot be negative")
        if np.any(lower + soft_margin_rad >= upper - soft_margin_rad):
            raise ValueError("soft_margin_rad collapses the allowed range")

        self.qdot_max = float(qdot_max_rad_s)
        self.qddot_max = float(qddot_max_rad_s2)
        self.soft_lower = lower + soft_margin_rad
        self.soft_upper = upper - soft_margin_rad

        self._q: np.ndarray | None = None
        self._qd = np.zeros(ARM_DOF, dtype=np.float64)

    @property
    def seeded(self) -> bool:
        return self._q is not None

    @property
    def last_command(self) -> np.ndarray | None:
        return None if self._q is None else self._q.copy()

    @property
    def commanded_velocity_rad_s(self) -> np.ndarray | None:
        """The velocity the command trajectory is currently moving at."""
        return None if self._q is None else self._qd.copy()

    def reset(self, q_rad: np.ndarray) -> None:
        """Re-seed from actual feedback (startup, HOLD-reseed, disarm)."""
        q = _as_joint_vector(q_rad, "q_rad")
        q = np.clip(q, self.soft_lower, self.soft_upper)
        self._q = q
        self._qd = np.zeros(ARM_DOF, dtype=np.float64)

    def update(self, target_rad: np.ndarray, dt_s: float) -> np.ndarray:
        """Advance one sender tick toward ``target_rad``."""
        if self._q is None:
            raise RuntimeError("reset() must seed the limiter before update()")
        target = _as_joint_vector(target_rad, "target_rad")
        if not np.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("dt_s must be finite and positive")
        dt = float(np.clip(dt_s, MIN_DT_S, MAX_DT_S))

        assert self._q is not None
        error = target - self._q
        v_desired = np.clip(error / dt, -self.qdot_max, self.qdot_max)
        # Braking limit: never command a velocity that cannot be
        # decelerated to zero within the remaining distance.
        braking = np.sqrt(2.0 * self.qddot_max * np.abs(error))
        v_desired = np.clip(v_desired, -braking, braking)
        v_new = np.clip(
            v_desired,
            self._qd - self.qddot_max * dt,
            self._qd + self.qddot_max * dt,
        )
        q_new = np.clip(
            self._q + v_new * dt,
            self.soft_lower,
            self.soft_upper,
        )
        # Recompute the state velocity from the actual position step so
        # soft-limit clipping is reflected as a genuine boundary stop.
        self._qd = np.clip(
            (q_new - self._q) / dt, -self.qdot_max, self.qdot_max
        )
        self._q = q_new
        return q_new.copy()
