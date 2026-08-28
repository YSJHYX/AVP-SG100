"""Glitch-frame continuity gate for IK candidates (v3 §A3.3).

Failure policy is classification-based, not a blanket REJECT+HOLD:

- a single frame whose candidate jumps more than ``jump_threshold_rad``
  from the previous goal is a tracking glitch: drop that frame only,
  the chain keeps running;
- ``consecutive_for_hold`` such frames in a row mean the jump is real
  (anchor rebuild, tracking recovery): HOLD and re-seed the IK from
  actual feedback.

Normal position-DLS candidates seeded from the previous solution are
inherently continuous, so this gate is a cheap outlier guard, not the
primary defense (v3 §A3.3).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from sg100_teleop.robot.arm_state import ARM_DOF

DEFAULT_JUMP_THRESHOLD_RAD = 0.2
DEFAULT_CONSECUTIVE_FOR_HOLD = 3


class GateAction(Enum):
    ACCEPT = "accept"
    DROP_FRAME = "drop_frame"
    HOLD_AND_RESEED = "hold_and_reseed"


@dataclass(frozen=True)
class GateResult:
    action: GateAction
    q_rad: np.ndarray | None  # set only for ACCEPT


class IKContinuityGate:
    """Stateful outlier gate over per-frame IK candidates."""

    def __init__(
        self,
        jump_threshold_rad: float = DEFAULT_JUMP_THRESHOLD_RAD,
        consecutive_for_hold: int = DEFAULT_CONSECUTIVE_FOR_HOLD,
    ) -> None:
        if jump_threshold_rad <= 0.0:
            raise ValueError("jump_threshold_rad must be positive")
        if consecutive_for_hold < 1:
            raise ValueError("consecutive_for_hold must be >= 1")
        self.jump_threshold_rad = float(jump_threshold_rad)
        self.consecutive_for_hold = int(consecutive_for_hold)
        self._consecutive_rejects = 0
        self.total_dropped_frames = 0
        self.total_holds = 0

    def reset(self) -> None:
        """Clear reject bookkeeping (after reseed or recalibration)."""
        self._consecutive_rejects = 0

    def process(
        self, candidate_rad: np.ndarray, previous_goal_rad: np.ndarray
    ) -> GateResult:
        candidate = np.asarray(candidate_rad, dtype=np.float64)
        previous = np.asarray(previous_goal_rad, dtype=np.float64)
        if candidate.shape != (ARM_DOF,) or previous.shape != (ARM_DOF,):
            raise ValueError("gate inputs must be 6-vectors")
        if not np.all(np.isfinite(candidate)) or not np.all(
            np.isfinite(previous)
        ):
            return self._reject("non-finite candidate")

        jump = float(np.max(np.abs(candidate - previous)))
        if jump <= self.jump_threshold_rad:
            self._consecutive_rejects = 0
            return GateResult(GateAction.ACCEPT, candidate.copy())
        return self._reject(f"jump {jump:.3f} rad")

    def _reject(self, detail: str) -> GateResult:
        self._consecutive_rejects += 1
        self.total_dropped_frames += 1
        if self._consecutive_rejects >= self.consecutive_for_hold:
            self.total_holds += 1
            return GateResult(GateAction.HOLD_AND_RESEED, None)
        return GateResult(GateAction.DROP_FRAME, None)
