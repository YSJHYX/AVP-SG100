"""Runtime FREE / HAND / CLAW grasp morphology (post-retarget, pre-filter)."""

from __future__ import annotations

from enum import Enum

import numpy as np

from sg100_teleop.robot.sg100_driver import HAND_DOF, SG100HardwareMap


class GraspMode(str, Enum):
    FREE = "free"
    HAND = "hand"
    CLAW = "claw"


# thumb_j1 canonical domain is [0, ~125 deg]; claw parks at +90 deg.
CLAW_THUMB_J1_RAD = float(np.deg2rad(90.0))
CLAW_THUMB_J2_RAD = float(np.deg2rad(-149.0))
CLAW_LITTLE_J1_RAD = float(-np.pi)
CLAW_INDEX_J1_RAD = 0.0

HAND_INDEX_J1_RAD = 0.0
HAND_LITTLE_J1_RAD = 0.0

DEFAULT_HOLD_SLEW_RAD_S = 1.5


class GraspModeController:
    """Overwrite selected joints after retarget according to grasp mode."""

    def __init__(
        self,
        *,
        joint_order: list[str] | tuple[str, ...] | None = None,
        hold_slew_rad_s: float = DEFAULT_HOLD_SLEW_RAD_S,
        initial_mode: GraspMode = GraspMode.FREE,
    ) -> None:
        order = list(
            joint_order
            if joint_order is not None
            else SG100HardwareMap().software_joint_order
        )
        if len(order) != HAND_DOF:
            raise ValueError(f"joint_order must have length {HAND_DOF}")
        self.joint_order = tuple(order)
        self.index_of = {name: i for i, name in enumerate(self.joint_order)}
        for name in ("thumb_j1", "thumb_j2", "index_j1", "little_j1"):
            if name not in self.index_of:
                raise ValueError(f"missing joint {name}")
        if not np.isfinite(hold_slew_rad_s) or hold_slew_rad_s <= 0.0:
            raise ValueError("hold_slew_rad_s must be finite and > 0")
        self.hold_slew_rad_s = float(hold_slew_rad_s)
        self.mode = GraspMode(initial_mode)
        self._hold_state: np.ndarray | None = None
        self._init_thumb_j2 = False

    def set_mode(self, mode: GraspMode | str) -> GraspMode:
        self.mode = GraspMode(mode)
        self._init_thumb_j2 = self.mode == GraspMode.CLAW
        return self.mode

    def _hold_targets(self) -> dict[int, float]:
        i = self.index_of
        if self.mode == GraspMode.FREE:
            return {}
        if self.mode == GraspMode.HAND:
            return {
                i["index_j1"]: HAND_INDEX_J1_RAD,
                i["little_j1"]: HAND_LITTLE_J1_RAD,
            }
        return {
            i["index_j1"]: CLAW_INDEX_J1_RAD,
            i["little_j1"]: CLAW_LITTLE_J1_RAD,
            i["thumb_j1"]: CLAW_THUMB_J1_RAD,
        }

    def apply(
        self,
        q_hand_candidate: np.ndarray,
        *,
        dt_s: float,
    ) -> np.ndarray:
        q = np.asarray(q_hand_candidate, dtype=np.float64)
        if q.shape != (HAND_DOF,) or not np.all(np.isfinite(q)):
            raise ValueError("q_hand_candidate must be finite 11D")
        out = q.copy()
        if self._hold_state is None:
            self._hold_state = out.copy()
        holds = self._hold_targets()
        if not np.isfinite(dt_s) or dt_s <= 0.0:
            step = 0.0
        else:
            step = self.hold_slew_rad_s * float(dt_s)
        if self.mode == GraspMode.CLAW and self._init_thumb_j2:
            holds = dict(holds)
            holds[self.index_of["thumb_j2"]] = CLAW_THUMB_J2_RAD
        for index, target in holds.items():
            current = float(self._hold_state[index])
            delta = float(target) - current
            if step <= 0.0:
                next_v = float(target)
            else:
                next_v = current + float(np.clip(delta, -step, step))
            self._hold_state[index] = next_v
            out[index] = next_v
        if self.mode == GraspMode.CLAW and self._init_thumb_j2:
            j2 = self.index_of["thumb_j2"]
            if abs(float(self._hold_state[j2]) - CLAW_THUMB_J2_RAD) <= 1e-3:
                self._init_thumb_j2 = False
        for index in range(HAND_DOF):
            if index not in holds:
                self._hold_state[index] = float(out[index])
        return out
