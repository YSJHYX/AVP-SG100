"""Runtime FREE / HAND / CLAW / PINCH / TWO_FINGER morphology (pre-filter)."""

from __future__ import annotations

from enum import Enum

import numpy as np

from sg100_teleop.robot.sg100_driver import HAND_DOF, SG100HardwareMap


class GraspMode(str, Enum):
    FREE = "free"
    HAND = "hand"
    CLAW = "claw"
    PINCH = "pinch"
    TWO_FINGER = "two_finger"


# thumb_j1 canonical domain is [0, ~125 deg]; claw parks at +90 deg.
CLAW_THUMB_J1_RAD = float(np.deg2rad(90.0))
CLAW_THUMB_J2_RAD = float(np.deg2rad(-149.0))
CLAW_THUMB_J3_RAD = 0.0
CLAW_INDEX_J1_RAD = 0.0
CLAW_INDEX_J2_RAD = 0.0
CLAW_INDEX_J3_RAD = 0.0
CLAW_MIDDLE_J1_RAD = 0.0
CLAW_MIDDLE_J2_RAD = 0.0
CLAW_LITTLE_J1_RAD = float(-np.pi)
CLAW_LITTLE_J2_RAD = 0.0
CLAW_LITTLE_J3_RAD = 0.0

# Legacy SCRIPT shape constants remain independent of live CLAW control.
CLAW_THUMB_J2_LITTLE_OFFSET_RAD = float(np.deg2rad(149.0))

# Legacy claw shape definitions retained for SCRIPT/API compatibility.
CLAW_LOCK_RAD = (
    ("thumb_j1", CLAW_THUMB_J1_RAD),
    ("index_j1", CLAW_INDEX_J1_RAD),
    ("little_j1", CLAW_LITTLE_J1_RAD),
)

# The old shape values must not change mode 4 when live CLAW changes.
CLAW_INIT_RAD = (
    ("thumb_j2", CLAW_THUMB_J2_RAD),
    ("thumb_j3", CLAW_THUMB_J3_RAD),
    ("index_j2", CLAW_INDEX_J2_RAD),
    ("index_j3", CLAW_INDEX_J3_RAD),
    ("middle_j1", CLAW_MIDDLE_J1_RAD),
    ("middle_j2", CLAW_MIDDLE_J2_RAD),
    ("little_j2", CLAW_LITTLE_J2_RAD),
    ("little_j3", CLAW_LITTLE_J3_RAD),
)

# PINCH uses one calibrated index-flexion input for both proximal joints.
# Equal angular travel closes the opposed fingers together, within the
# existing conservative thumb teleop range. F_cmd remains downstream.
PINCH_ACTIVE_JOINTS = ("thumb_j2", "index_j2")
PINCH_THUMB_J2_OPEN_RAD = -2.0
PINCH_THUMB_J2_CLOSED_RAD = float(np.deg2rad(-30.0))
PINCH_J2_TRAVEL_RAD = PINCH_THUMB_J2_CLOSED_RAD - PINCH_THUMB_J2_OPEN_RAD
PINCH_DISTAL_RAD = float(np.deg2rad(50.0))
PINCH_LOCK_RAD = (
    ("thumb_j1", float(np.deg2rad(90.0))),
    ("thumb_j3", PINCH_DISTAL_RAD),
    ("index_j1", 0.0),
    ("index_j3", PINCH_DISTAL_RAD),
    ("middle_j1", 0.0),
    ("middle_j2", 0.0),
    ("little_j1", 0.0),
    ("little_j2", 0.0),
    ("little_j3", 0.0),
)

# CLAW forms two opposed two-finger jaws. One index-flexion progress drives
# all four proximal joints; middle flexion is negative in canonical radians.
CLAW_GRIPPER_ACTIVE_JOINTS = ("thumb_j2", "index_j2", "middle_j1", "little_j2")
CLAW_GRIPPER_TRAVEL_RAD = float(np.deg2rad(90.0))
CLAW_GRIPPER_LOCK_RAD = (
    ("thumb_j1", CLAW_THUMB_J1_RAD),
    ("thumb_j3", PINCH_DISTAL_RAD),
    ("index_j1", CLAW_INDEX_J1_RAD),
    ("index_j3", PINCH_DISTAL_RAD),
    ("middle_j2", PINCH_DISTAL_RAD),
    ("little_j1", CLAW_LITTLE_J1_RAD),
    ("little_j3", PINCH_DISTAL_RAD),
)

# Mode 6 uses only index J2/J3 to drive all four fingers. The middle finger
# has only J1/J2: proximal J1 flexes negatively; distal J2 follows index J3.
# Thumb J2 retains its claw offset, and the three base axes stay parked.
TWO_FINGER_THUMB_J2_OFFSET_RAD = float(np.deg2rad(-149.0))
TWO_FINGER_LOCK_RAD = (
    ("thumb_j1", float(np.deg2rad(90.0))),
    ("index_j1", float(np.deg2rad(90.0))),
    ("little_j1", float(np.deg2rad(-90.0))),
)
TWO_FINGER_INIT_RAD = (
    ("thumb_j2", TWO_FINGER_THUMB_J2_OFFSET_RAD),
    ("thumb_j3", 0.0),
    ("index_j2", 0.0),
    ("index_j3", 0.0),
    ("middle_j1", 0.0),
    ("middle_j2", 0.0),
    ("little_j2", 0.0),
    ("little_j3", 0.0),
)
TWO_FINGER_DISTAL_MAX_RAD = float(np.deg2rad(60.0))

HAND_INDEX_J1_RAD = 0.0
HAND_LITTLE_J1_RAD = 0.0

DEFAULT_HOLD_SLEW_RAD_S = 3.0
_INIT_DONE_TOL_RAD = 1e-3


class GraspModeController:
    """Overwrite selected joints after retarget according to grasp mode."""

    def __init__(
        self,
        *,
        joint_order: list[str] | tuple[str, ...] | None = None,
        hold_slew_rad_s: float = DEFAULT_HOLD_SLEW_RAD_S,
        initial_mode: GraspMode = GraspMode.FREE,
    ) -> None:
        hardware_map = SG100HardwareMap()
        order = list(
            joint_order
            if joint_order is not None
            else hardware_map.software_joint_order
        )
        if len(order) != HAND_DOF:
            raise ValueError(f"joint_order must have length {HAND_DOF}")
        self.joint_order = tuple(order)
        self.index_of = {name: i for i, name in enumerate(self.joint_order)}
        required = (
            *(name for name, _ in CLAW_LOCK_RAD),
            *(name for name, _ in CLAW_INIT_RAD),
            "index_j1",
            "little_j1",
        )
        for name in required:
            if name not in self.index_of:
                raise ValueError(f"missing joint {name}")
        if not np.isfinite(hold_slew_rad_s) or hold_slew_rad_s <= 0.0:
            raise ValueError("hold_slew_rad_s must be finite and > 0")
        self.hold_slew_rad_s = float(hold_slew_rad_s)
        self.mode = GraspMode(initial_mode)
        self._hold_state: np.ndarray | None = None
        self._two_finger_init_pending = self.mode == GraspMode.TWO_FINGER
        self._claw_holds = {}
        for name, target in CLAW_GRIPPER_LOCK_RAD:
            hardware_index = hardware_map.software_index_of[name]
            # In particular, -pi for little_j1 is slightly below the model's
            # -3.14 limit. Hold the legal pose, never expand the command range.
            self._claw_holds[self.index_of[name]] = float(np.clip(
                target, hardware_map.lower_rad[hardware_index],
                hardware_map.upper_rad[hardware_index],
            ))

    def set_mode(self, mode: GraspMode | str) -> GraspMode:
        self.mode = GraspMode(mode)
        self._two_finger_init_pending = self.mode == GraspMode.TWO_FINGER
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
        if self.mode == GraspMode.PINCH:
            return {i[name]: float(target) for name, target in PINCH_LOCK_RAD}
        if self.mode == GraspMode.TWO_FINGER:
            holds = {i[name]: float(target) for name, target in TWO_FINGER_LOCK_RAD}
            if self._two_finger_init_pending:
                holds.update((i[name], float(target)) for name, target in TWO_FINGER_INIT_RAD)
            return holds
        return self._claw_holds

    def apply(
        self,
        q_hand_candidate: np.ndarray,
        *,
        dt_s: float,
        pinch_closure: float | None = None,
    ) -> np.ndarray:
        q = np.asarray(q_hand_candidate, dtype=np.float64)
        if q.shape != (HAND_DOF,) or not np.all(np.isfinite(q)):
            raise ValueError("q_hand_candidate must be finite 11D")
        out = q.copy()
        if self.mode in (GraspMode.CLAW, GraspMode.PINCH):
            if pinch_closure is None or not np.isfinite(pinch_closure):
                raise ValueError(f"{self.mode.value.upper()} requires finite calibrated index flexion")
            # Both gripper modes use the same calibrated single input.
            closure = float(np.clip(pinch_closure, 0.0, 1.0))
        if self.mode == GraspMode.PINCH:
            # Use one shared progress value instead of two independently
            # mapped finger features, including when automatic pinch is on.
            travel = PINCH_J2_TRAVEL_RAD * closure
            out[self.index_of["thumb_j2"]] = float(np.clip(
                PINCH_THUMB_J2_OPEN_RAD + travel,
                PINCH_THUMB_J2_OPEN_RAD,
                PINCH_THUMB_J2_CLOSED_RAD,
            ))
            out[self.index_of["index_j2"]] = travel
        elif self.mode == GraspMode.CLAW:
            travel = CLAW_GRIPPER_TRAVEL_RAD * closure
            out[self.index_of["thumb_j2"]] = CLAW_THUMB_J2_RAD + travel
            out[self.index_of["index_j2"]] = travel
            out[self.index_of["middle_j1"]] = -travel
            out[self.index_of["little_j2"]] = travel
        if self._hold_state is None:
            self._hold_state = out.copy()
        holds = self._hold_targets()
        if not np.isfinite(dt_s) or dt_s <= 0.0:
            step = 0.0
        else:
            step = self.hold_slew_rad_s * float(dt_s)
        for index, target in holds.items():
            current = float(self._hold_state[index])
            delta = float(target) - current
            if step <= 0.0:
                next_v = float(target)
            else:
                next_v = current + float(np.clip(delta, -step, step))
            self._hold_state[index] = next_v
            out[index] = next_v
        if self.mode == GraspMode.TWO_FINGER:
            # Couple AFTER initialization/holds so the relationships also
            # apply during entry. Ignore all independent non-index inputs.
            j2 = float(out[self.index_of["index_j2"]])
            j3 = min(float(out[self.index_of["index_j3"]]), TWO_FINGER_DISTAL_MAX_RAD)
            out[self.index_of["thumb_j2"]] = j2 + TWO_FINGER_THUMB_J2_OFFSET_RAD
            out[self.index_of["middle_j1"]] = -j2
            out[self.index_of["little_j2"]] = j2
            for name in ("thumb_j3", "index_j3", "middle_j2", "little_j3"):
                out[self.index_of[name]] = j3
            for name, _ in TWO_FINGER_INIT_RAD:
                self._hold_state[self.index_of[name]] = float(out[self.index_of[name]])
        if self.mode == GraspMode.TWO_FINGER and self._two_finger_init_pending:
            # Return the complete initial pose for this frame before releasing
            # the eight coupled joints on the next frame. Base axes stay held.
            if all(
                abs(float(out[index]) - target) <= _INIT_DONE_TOL_RAD
                for index, target in holds.items()
            ):
                self._two_finger_init_pending = False
        for index in range(HAND_DOF):
            if index not in holds:
                self._hold_state[index] = float(out[index])
        return out
