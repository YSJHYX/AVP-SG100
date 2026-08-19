from __future__ import annotations

import math
import numpy as np


def make_transform(
    position: np.ndarray,
    rotation: np.ndarray,
) -> np.ndarray:
    position = np.asarray(
        position,
        dtype=np.float64,
    )

    rotation = np.asarray(
        rotation,
        dtype=np.float64,
    )

    if position.shape != (3,):
        raise ValueError(
            f"position must have shape (3,), got {position.shape}"
        )

    if rotation.shape != (3, 3):
        raise ValueError(
            f"rotation must have shape (3,3), got {rotation.shape}"
        )

    T = np.eye(
        4,
        dtype=np.float64,
    )

    T[:3, :3] = rotation
    T[:3, 3] = position

    return T


def invert_transform(
    T: np.ndarray,
) -> np.ndarray:
    T = np.asarray(
        T,
        dtype=np.float64,
    )

    if T.shape != (4, 4):
        raise ValueError(
            f"T must have shape (4,4), got {T.shape}"
        )

    R = T[:3, :3]
    p = T[:3, 3]

    result = np.eye(
        4,
        dtype=np.float64,
    )

    result[:3, :3] = R.T
    result[:3, 3] = -(R.T @ p)

    return result


def rotation_angle(
    R: np.ndarray,
) -> float:
    value = (
        np.trace(R) - 1.0
    ) / 2.0

    value = float(
        np.clip(
            value,
            -1.0,
            1.0,
        )
    )

    return math.acos(value)


def rotation_from_two_unit_vectors(
    a: np.ndarray,
    b: np.ndarray,
) -> np.ndarray:
    """
    Minimal (shortest) rotation taking unit vector ``a`` onto unit
    vector ``b``.

    Robust Rodrigues construction:
      * parallel (a ~ +b)  -> identity
      * anti-parallel      -> 180 deg about a canonical axis chosen
                              as the least-parallel basis vector
    Never produces NaN for unit-norm inputs.
    """

    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)

    if a.shape != (3,) or b.shape != (3,):
        raise ValueError("direction vectors must have shape (3,)")
    if not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
        raise ValueError("direction vectors must be finite")

    a_norm = float(np.linalg.norm(a))
    b_norm = float(np.linalg.norm(b))
    if a_norm <= 1e-12 or b_norm <= 1e-12:
        raise ValueError("direction vectors must have non-zero norm")

    a = a / a_norm
    b = b / b_norm

    cross = np.cross(a, b)
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))

    cross_norm = float(np.linalg.norm(cross))

    if dot > 1.0 - 1e-12:
        return np.eye(3, dtype=np.float64)

    if cross_norm < 1e-12:
        # anti-parallel: pick the canonical basis least aligned with a
        basis = np.eye(3)
        projections = np.abs(a)
        axis = basis[int(np.argmin(projections))]
        axis = axis - np.dot(axis, a) * a
        axis /= np.linalg.norm(axis)

        K = np.array(
            [[0.0, -axis[2], axis[1]],
             [axis[2], 0.0, -axis[0]],
             [-axis[1], axis[0], 0.0]],
            dtype=np.float64,
        )
        return np.eye(3) + 2.0 * (K @ K)

    axis = cross / cross_norm
    angle = float(np.arctan2(cross_norm, dot))

    K = np.array(
        [[0.0, -axis[2], axis[1]],
         [axis[2], 0.0, -axis[0]],
         [-axis[1], axis[0], 0.0]],
        dtype=np.float64,
    )

    return (
        np.eye(3)
        + np.sin(angle) * K
        + (1.0 - np.cos(angle)) * (K @ K)
    )


class AVPWristRelativeMapper:
    """
    Maps relative AVP wrist SE(3) motion to a robot TCP target.

    This class is deliberately independent of VisionProStreamer.
    It accepts ordinary 4x4 homogeneous transforms.

    Definitions
    -----------
    T_avp_world_wrist:
        Current AVP wrist pose in AVP world coordinates.

    T_tcp_world:
        Robot teleop TCP pose in MuJoCo/world coordinates.

    R_tcp_from_avp_local:
        Static rotation mapping coordinates expressed in the
        initial AVP wrist frame into coordinates expressed in
        the initial robot TCP frame.

        For synthetic validation:
            identity

        For real AVP:
            determine once from actual AVP wrist-axis
            conventions / visual validation.

    translation_scale:
        Scale applied to AVP relative wrist translation.

        Example:
            0.5 means 10 cm human motion -> 5 cm robot motion.
    """

    def __init__(
        self,
        *,
        translation_scale: float = 1.0,
        R_tcp_from_avp_local: np.ndarray | None = None,
    ):
        self.translation_scale = float(
            translation_scale
        )

        if self.translation_scale <= 0:
            raise ValueError(
                "translation_scale must be > 0"
            )

        if R_tcp_from_avp_local is None:
            R_tcp_from_avp_local = np.eye(
                3,
                dtype=np.float64,
            )

        self.R_tcp_from_avp_local = np.asarray(
            R_tcp_from_avp_local,
            dtype=np.float64,
        )

        if self.R_tcp_from_avp_local.shape != (3, 3):
            raise ValueError(
                "R_tcp_from_avp_local must have shape (3,3)"
            )

        self._T_avp0 = None
        self._T_tcp0 = None
        self._n_forearm0_avp = None

    @property
    def calibrated(self) -> bool:
        return (
            self._T_avp0 is not None
            and self._T_tcp0 is not None
        )

    def calibrate(
        self,
        T_avp_world_wrist: np.ndarray,
        T_tcp_world: np.ndarray,
    ) -> None:
        """
        Bind the current AVP wrist pose to the current TCP pose.

        Immediately after calibration:
            target TCP == T_tcp_world
        """
        T_avp_world_wrist = np.asarray(
            T_avp_world_wrist,
            dtype=np.float64,
        )

        T_tcp_world = np.asarray(
            T_tcp_world,
            dtype=np.float64,
        )

        if T_avp_world_wrist.shape != (4, 4):
            raise ValueError(
                "T_avp_world_wrist must have shape (4,4)"
            )

        if T_tcp_world.shape != (4, 4):
            raise ValueError(
                "T_tcp_world must have shape (4,4)"
            )

        self._T_avp0 = (
            T_avp_world_wrist.copy()
        )

        self._T_tcp0 = (
            T_tcp_world.copy()
        )

        # A wrist recalibration starts a new session.  Never let a
        # forearm anchor from the previous session survive until a new,
        # valid 27-joint forearm sample is explicitly captured.
        self._n_forearm0_avp = None

    @property
    def forearm_axis_calibrated(self) -> bool:
        return self._n_forearm0_avp is not None

    @staticmethod
    def _normalized_direction(value: np.ndarray) -> np.ndarray | None:
        direction = np.asarray(value, dtype=np.float64)
        if direction.shape != (3,) or not np.all(np.isfinite(direction)):
            return None
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-12:
            return None
        return direction / norm

    def calibrate_forearm_axis(
        self,
        n_forearm_avp_world: np.ndarray,
    ) -> bool:
        """Bind the current human forearm direction to TCP local +Z.

        Invalid input clears the session anchor and returns ``False`` so
        callers can safely fall back to position-only IK.
        """

        self._n_forearm0_avp = None
        if not self.calibrated:
            return False

        direction = self._normalized_direction(n_forearm_avp_world)
        if direction is None:
            return False

        self._n_forearm0_avp = direction
        return True

    def map_forearm_direction(
        self,
        n_forearm_avp_world: np.ndarray,
        tcp_axis_local: np.ndarray | None = None,
    ) -> np.ndarray | None:
        """Map relative forearm direction change onto a robot TCP axis.

        Both human directions are expressed in the calibration wrist
        frame.  Their shortest relative rotation is basis-mapped into
        the calibration TCP frame, then applied to ``tcp_axis_local``.
        Consequently the calibration sample always maps exactly to the
        current TCP axis, independent of the human wrist/forearm angle.
        """

        if not self.calibrated or not self.forearm_axis_calibrated:
            return None

        current_world = self._normalized_direction(n_forearm_avp_world)
        if current_world is None:
            return None

        if tcp_axis_local is None:
            tcp_axis_local = np.array([0.0, 0.0, 1.0])
        tcp_axis = self._normalized_direction(tcp_axis_local)
        if tcp_axis is None:
            return None

        R_avp0 = self._T_avp0[:3, :3]
        n0_local = R_avp0.T @ self._n_forearm0_avp
        n_local = R_avp0.T @ current_world
        n0_local = self._normalized_direction(n0_local)
        n_local = self._normalized_direction(n_local)
        if n0_local is None or n_local is None:
            return None

        R_delta_avp = rotation_from_two_unit_vectors(
            n0_local,
            n_local,
        )
        A = self.R_tcp_from_avp_local
        R_delta_tcp = A @ R_delta_avp @ A.T

        target_world = (
            self._T_tcp0[:3, :3]
            @ R_delta_tcp
            @ tcp_axis
        )
        return self._normalized_direction(target_world)

    def relative_avp_motion(
        self,
        T_avp_world_wrist: np.ndarray,
    ) -> np.ndarray:
        if not self.calibrated:
            raise RuntimeError(
                "AVPWristRelativeMapper is not calibrated"
            )

        return (
            invert_transform(self._T_avp0)
            @ np.asarray(
                T_avp_world_wrist,
                dtype=np.float64,
            )
        )

    def map(
        self,
        T_avp_world_wrist: np.ndarray,
    ) -> np.ndarray:
        """
        Return target TCP pose in robot/MuJoCo world frame.
        """
        delta_avp = self.relative_avp_motion(
            T_avp_world_wrist
        )

        p_avp = delta_avp[:3, 3]
        R_avp = delta_avp[:3, :3]

        A = self.R_tcp_from_avp_local

        # Express relative AVP translation in initial TCP axes.
        p_tcp_local = (
            self.translation_scale
            * (A @ p_avp)
        )

        # Change basis for the relative wrist rotation.
        R_tcp_local = (
            A
            @ R_avp
            @ A.T
        )

        delta_tcp = make_transform(
            p_tcp_local,
            R_tcp_local,
        )

        return (
            self._T_tcp0
            @ delta_tcp
        )
