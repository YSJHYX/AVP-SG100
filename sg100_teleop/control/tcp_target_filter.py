"""Cartesian TCP target filtering for RM65 teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation


TCP_POSITION_FILTER_TAU = 0.06
TCP_ROTATION_FILTER_TAU = 0.06
TCP_MAX_LINEAR_VELOCITY = 0.25
TCP_MAX_ANGULAR_VELOCITY = 1.5

# A long gap must not turn the first returning sample into an effectively
# unfiltered jump. Rebase time and hold that one sample instead.
TCP_FILTER_MAX_DT = 0.25
ROTATION_ATOL = 1e-6


@dataclass(frozen=True)
class TcpTargetFilterConfig:
    position_tau: float = TCP_POSITION_FILTER_TAU
    rotation_tau: float = TCP_ROTATION_FILTER_TAU
    max_linear_velocity: float = TCP_MAX_LINEAR_VELOCITY
    max_angular_velocity: float = TCP_MAX_ANGULAR_VELOCITY

    def __post_init__(self) -> None:
        for name in (
            "position_tau",
            "rotation_tau",
            "max_linear_velocity",
            "max_angular_velocity",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
            object.__setattr__(self, name, value)


def _validate_pose(target_pose: np.ndarray) -> np.ndarray:
    pose = np.asarray(target_pose, dtype=np.float64)
    if pose.shape != (4, 4):
        raise ValueError(f"target_pose must have shape (4,4), got {pose.shape}")
    if not np.all(np.isfinite(pose)):
        raise ValueError("target_pose must contain only finite values")
    if not np.allclose(
        pose[3],
        np.array([0.0, 0.0, 0.0, 1.0]),
        atol=ROTATION_ATOL,
        rtol=0.0,
    ):
        raise ValueError("target_pose must have homogeneous last row [0,0,0,1]")

    rotation = pose[:3, :3]
    if not np.allclose(
        rotation.T @ rotation,
        np.eye(3),
        atol=ROTATION_ATOL,
        rtol=0.0,
    ):
        raise ValueError("target_pose rotation must be orthonormal")
    determinant = float(np.linalg.det(rotation))
    if not math.isclose(determinant, 1.0, abs_tol=ROTATION_ATOL):
        raise ValueError("target_pose rotation must have determinant +1")
    return pose.copy()


def _interpolate_so3(
    previous: np.ndarray,
    target: np.ndarray,
    fraction: float,
) -> np.ndarray:
    """Shortest-path SO(3) interpolation for active local-to-world matrices."""

    fraction = float(np.clip(fraction, 0.0, 1.0))
    delta_world = target @ previous.T
    delta_rotvec = Rotation.from_matrix(delta_world).as_rotvec()
    return Rotation.from_rotvec(fraction * delta_rotvec).as_matrix() @ previous


class TcpTargetFilter:
    """Low-pass and physically rate-limit an SE(3) target before IK."""

    def __init__(self, config: TcpTargetFilterConfig | None = None):
        self.config = config or TcpTargetFilterConfig()
        self._filtered_position = np.zeros(3, dtype=np.float64)
        self._filtered_rotation = np.eye(3, dtype=np.float64)
        self._previous_timestamp: float | None = None
        self._initialized = False

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def value(self) -> np.ndarray:
        if not self._initialized:
            raise RuntimeError("TCP target filter is not initialized")
        return self._make_pose()

    def clear(self) -> None:
        """Discard all state at a teleoperation session boundary."""

        self._previous_timestamp = None
        self._initialized = False

    def reset(
        self,
        target_pose: np.ndarray,
        timestamp: float | None = None,
    ) -> np.ndarray:
        pose = _validate_pose(target_pose)
        current_time = self._resolve_timestamp(timestamp)
        self._filtered_position[:] = pose[:3, 3]
        self._filtered_rotation[:] = pose[:3, :3]
        self._previous_timestamp = current_time
        self._initialized = True
        return self._make_pose()

    def update(
        self,
        raw_target_pose: np.ndarray,
        timestamp: float | None = None,
    ) -> np.ndarray:
        raw_pose = _validate_pose(raw_target_pose)
        current_time = self._resolve_timestamp(timestamp)

        if not self._initialized:
            return self.reset(raw_pose, timestamp=current_time)

        dt = current_time - float(self._previous_timestamp)
        if not math.isfinite(dt) or dt <= 0.0:
            return self._make_pose()
        if dt > TCP_FILTER_MAX_DT:
            self._previous_timestamp = current_time
            return self._make_pose()

        position_alpha = 1.0 - math.exp(-dt / self.config.position_tau)
        position_lp = self._filtered_position + position_alpha * (
            raw_pose[:3, 3] - self._filtered_position
        )
        position_delta = position_lp - self._filtered_position
        position_distance = float(np.linalg.norm(position_delta))
        max_position_step = self.config.max_linear_velocity * dt
        if position_distance > max_position_step:
            position_delta *= max_position_step / position_distance
        self._filtered_position += position_delta

        rotation_alpha = 1.0 - math.exp(-dt / self.config.rotation_tau)
        rotation_lp = _interpolate_so3(
            self._filtered_rotation,
            raw_pose[:3, :3],
            rotation_alpha,
        )
        rotation_delta = rotation_lp @ self._filtered_rotation.T
        rotation_angle = float(
            np.linalg.norm(Rotation.from_matrix(rotation_delta).as_rotvec())
        )
        max_rotation_step = self.config.max_angular_velocity * dt
        if rotation_angle > max_rotation_step:
            rotation_lp = _interpolate_so3(
                self._filtered_rotation,
                rotation_lp,
                max_rotation_step / rotation_angle,
            )
        self._filtered_rotation[:] = rotation_lp

        self._previous_timestamp = current_time
        return self._make_pose()

    @staticmethod
    def _resolve_timestamp(timestamp: float | None) -> float:
        current_time = time.monotonic() if timestamp is None else float(timestamp)
        if not math.isfinite(current_time):
            raise ValueError("timestamp must be finite")
        return current_time

    def _make_pose(self) -> np.ndarray:
        pose = np.eye(4, dtype=np.float64)
        pose[:3, :3] = self._filtered_rotation
        pose[:3, 3] = self._filtered_position
        return pose
