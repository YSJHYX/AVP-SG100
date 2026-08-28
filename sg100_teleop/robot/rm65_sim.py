"""Sim-twin backend: writes MuJoCo ctrl exactly like the current path."""

from __future__ import annotations

import numpy as np

from sg100_teleop.robot.arm_backend import ArmBackend
from sg100_teleop.robot.arm_state import ARM_DOF, ArmState

# Default RM65-B joint limits, matching the MJCF assets
# (sg100_teleop/assets/rm65_sg100/mjcf/rm65_sg100_sim.xml joint_1..6
# ranges, radians).
RM65_JOINT_LOWER_RAD = np.array(
    [-3.106, -2.2689, -2.356, -3.106, -2.234, -6.28]
)
RM65_JOINT_UPPER_RAD = -RM65_JOINT_LOWER_RAD


def _as_six_vector(q_rad: np.ndarray) -> np.ndarray:
    q = np.asarray(q_rad, dtype=np.float64)
    if q.shape != (ARM_DOF,):
        raise ValueError(f"expected shape ({ARM_DOF},), got {q.shape}")
    if not np.all(np.isfinite(q)):
        raise ValueError("q contains non-finite values")
    return q


class RM65SimBackend(ArmBackend):
    """Passthrough backend over an existing MuJoCo model.

    Kept for interface parity during migration: the runner's sim mode
    keeps its current ``write_robot_controls`` call, and this backend
    writes only the arm actuator slice of ``data.ctrl`` so tests and
    tooling can drive the twin through the same abstraction.
    """

    def __init__(self, data, layout) -> None:
        # mujoco / layout imports stay local so this subpackage remains
        # importable without a MuJoCo runtime.
        self._data = data
        self._layout = layout
        self._enabled = True
        self._sequence = 0

    def connect(self) -> None:
        return None

    def disconnect(self) -> None:
        return None

    @property
    def enabled(self) -> bool:
        return self._enabled

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = bool(enabled)

    def send_joint_target(self, q_rad: np.ndarray) -> None:
        q = _as_six_vector(q_rad)
        self._data.ctrl[self._layout.arm_actuator_ids] = q

    def joint_state(self) -> ArmState | None:
        import time

        q = np.asarray(
            self._data.qpos[self._layout.arm_qpos_indices],
            dtype=np.float64,
        ).copy()
        self._sequence += 1
        return ArmState(
            q_rad=q,
            qd_rad_s=None,
            received_monotonic_ns=time.monotonic_ns(),
            sequence=self._sequence,
            controller_error=0,
            arm_status=None,
            valid=True,
        )

    def stop_smooth(self) -> int:
        return 0

    def stop_immediate(self) -> int:
        return 0

    def joint_limits_rad(self) -> tuple[np.ndarray, np.ndarray]:
        return RM65_JOINT_LOWER_RAD.copy(), RM65_JOINT_UPPER_RAD.copy()
