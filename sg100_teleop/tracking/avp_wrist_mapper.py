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
