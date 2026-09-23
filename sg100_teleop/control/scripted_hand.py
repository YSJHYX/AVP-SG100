"""Scripted hand source: enter claw, then J close / K hold / L open."""

from __future__ import annotations

from enum import Enum

import numpy as np

from sg100_teleop.control.grasp_mode import (
    CLAW_INDEX_J1_RAD,
    CLAW_LITTLE_J1_RAD,
    CLAW_THUMB_J1_RAD,
    CLAW_THUMB_J2_RAD,
)
from sg100_teleop.robot.sg100_driver import HAND_DOF, SG100HardwareMap


class ScriptMotion(str, Enum):
    HOLD = "hold"
    SHAPING = "shaping"
    CLOSING = "closing"
    OPENING = "opening"


DEFAULT_SLEW_RAD_S = float(np.deg2rad(30.0))
CLAW_LOCK_NAMES = ("thumb_j1", "index_j1", "middle_j1", "little_j1")
CLAW_SHAPE_NAMES = ("thumb_j1", "thumb_j2", "thumb_j3", "index_j1", "index_j2", "index_j3", "middle_j1", "middle_j2", "little_j1", "little_j2", "little_j3")
FLEXION_CLOSE_DEG = {
    "thumb_j2": 0.0,
    "thumb_j3": 45.0,
    "index_j2": 90.0,
    "index_j3": 45.0,
    "middle_j1": -90.0,
    "middle_j2": 45.0,
    "little_j2": 90.0,
    "little_j3": 45.0,
}


class ScriptHandController:
    def __init__(
        self,
        *,
        joint_order: list[str] | tuple[str, ...] | None = None,
        slew_rad_s: float = DEFAULT_SLEW_RAD_S,
    ) -> None:
        order = list(
            joint_order
            if joint_order is not None
            else SG100HardwareMap().software_joint_order
        )
        if len(order) != HAND_DOF:
            raise ValueError(f"joint_order must have length {HAND_DOF}")
        if not np.isfinite(slew_rad_s) or slew_rad_s <= 0.0:
            raise ValueError("slew_rad_s must be finite and > 0")
        self.joint_order = tuple(order)
        self.index_of = {name: i for i, name in enumerate(self.joint_order)}
        for name in (*CLAW_SHAPE_NAMES, *FLEXION_CLOSE_DEG):
            if name not in self.index_of:
                raise ValueError(f"missing joint {name}")
        self.slew_rad_s = float(slew_rad_s)
        self.q_claw = np.zeros(HAND_DOF, dtype=np.float64)
        self.q_claw[self.index_of["thumb_j1"]] = CLAW_THUMB_J1_RAD
        self.q_claw[self.index_of["thumb_j2"]] = CLAW_THUMB_J2_RAD
        self.q_claw[self.index_of["thumb_j3"]] = CLAW_THUMB_J3_RAD
        self.q_claw[self.index_of["index_j1"]] = CLAW_INDEX_J1_RAD
        self.q_claw[self.index_of["index_j2"]] = CLAW_INDEX_J2_RAD
        self.q_claw[self.index_of["index_j3"]] = CLAW_INDEX_J3_RAD
        self.q_claw[self.index_of["middle_j1"]] = CLAW_MIDDLE_J1_RAD
        self.q_claw[self.index_of["middle_j2"]] = CLAW_MIDDLE_J2_RAD
        self.q_claw[self.index_of["little_j1"]] = CLAW_LITTLE_J1_RAD
        self.q_claw[self.index_of["little_j2"]] = CLAW_LITTLE_J2_RAD
        self.q_claw[self.index_of["little_j3"]] = CLAW_LITTLE_J3_RAD
        self.q_open = self.q_claw.copy()
        self.q_close_max = self.q_claw.copy()
        for name, deg in FLEXION_CLOSE_DEG.items():
            self.q_close_max[self.index_of[name]] = float(np.deg2rad(deg))
        shape_idx = np.array(
            [self.index_of[n] for n in CLAW_SHAPE_NAMES], dtype=np.int32
        )
        lock_idx = np.array(
            [self.index_of[n] for n in CLAW_LOCK_NAMES], dtype=np.int32
        )
        flex_idx = np.array(
            [self.index_of[n] for n in FLEXION_CLOSE_DEG], dtype=np.int32
        )
        self._shape = np.zeros(HAND_DOF, dtype=bool)
        self._claw = np.zeros(HAND_DOF, dtype=bool)
        self._flex = np.zeros(HAND_DOF, dtype=bool)
        self._shape[shape_idx] = True
        self._claw[lock_idx] = True
        self._flex[flex_idx] = True
        self.enabled = False
        self.motion = ScriptMotion.HOLD
        self._q: np.ndarray | None = None

    @staticmethod
    def _as_hand(q: np.ndarray) -> np.ndarray:
        out = np.asarray(q, dtype=np.float64)
        if out.shape != (HAND_DOF,) or not np.all(np.isfinite(out)):
            raise ValueError("hand command must be finite 11D")
        return out.copy()

    def enter(self, q_current: np.ndarray) -> np.ndarray:
        self.enabled = True
        self.motion = ScriptMotion.SHAPING
        self._q = self._as_hand(q_current)
        return self._q.copy()

    def exit(self) -> None:
        self.enabled = False
        self.motion = ScriptMotion.HOLD

    def set_motion(self, motion: ScriptMotion | str) -> ScriptMotion:
        if not self.enabled or self._q is None:
            raise RuntimeError("script hand is not active")
        requested = ScriptMotion(motion)
        if requested == ScriptMotion.SHAPING:
            requested = ScriptMotion.HOLD
        self.motion = requested
        return self.motion

    def _slew(self, current: np.ndarray, target: np.ndarray, mask: np.ndarray, step: float) -> None:
        delta = np.clip(target[mask] - current[mask], -step, step)
        current[mask] = current[mask] + delta

    def step(self, dt_s: float) -> np.ndarray | None:
        if not self.enabled or self._q is None:
            return None
        if self.motion == ScriptMotion.HOLD or not np.isfinite(dt_s) or dt_s <= 0.0:
            return self._q.copy()
        step = self.slew_rad_s * float(dt_s)
        nxt = self._q.copy()
        if self.motion == ScriptMotion.SHAPING:
            self._slew(nxt, self.q_claw, self._shape, step)
        else:
            self._slew(nxt, self.q_claw, self._claw, step)
        if self.motion == ScriptMotion.CLOSING:
            self._slew(nxt, self.q_close_max, self._flex, step)
        elif self.motion == ScriptMotion.OPENING:
            self._slew(nxt, self.q_open, self._flex, step)
        self._q = nxt
        shape_done = np.allclose(self._q[self._shape], self.q_claw[self._shape], atol=1e-4)
        claw_done = np.allclose(self._q[self._claw], self.q_claw[self._claw], atol=1e-4)
        if self.motion == ScriptMotion.SHAPING and shape_done:
            self.motion = ScriptMotion.HOLD
        elif self.motion == ScriptMotion.CLOSING and claw_done:
            if np.allclose(self._q[self._flex], self.q_close_max[self._flex], atol=1e-4):
                self.motion = ScriptMotion.HOLD
        elif self.motion == ScriptMotion.OPENING and claw_done:
            if np.allclose(self._q[self._flex], self.q_open[self._flex], atol=1e-4):
                self.motion = ScriptMotion.HOLD
        return self._q.copy()
