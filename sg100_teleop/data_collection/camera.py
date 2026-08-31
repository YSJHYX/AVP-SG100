"""Camera-provider and synchronization placeholders for future sensors.

No provider in this module fabricates black frames.  The null provider returns
``None`` and declares the episode not ready for multimodal training.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class CameraCalibration:
    calibration_id: str
    parent_frame: str
    transform_parent_camera: np.ndarray
    intrinsics: np.ndarray
    distortion: np.ndarray
    depth_scale: float | None = None

    def validate(self) -> None:
        transform = np.asarray(self.transform_parent_camera, dtype=np.float64)
        intrinsics = np.asarray(self.intrinsics, dtype=np.float64)
        distortion = np.asarray(self.distortion, dtype=np.float64)
        if transform.shape != (4, 4) or intrinsics.shape != (3, 3):
            raise ValueError("camera transform/intrinsics shapes are invalid")
        if distortion.ndim != 1:
            raise ValueError("camera distortion must be one-dimensional")
        if not all(np.all(np.isfinite(x)) for x in (transform, intrinsics, distortion)):
            raise ValueError("camera calibration contains non-finite values")


@dataclass(frozen=True)
class CameraFrame:
    camera_name: str
    frame_id: int
    calibration_id: str
    sensor_timestamp_ns: int
    host_receive_timestamp_ns: int
    master_timestamp_ns: int
    rgb: np.ndarray | None
    depth: np.ndarray | None = None


class CameraProvider(ABC):
    @property
    @abstractmethod
    def present(self) -> bool:
        """Whether real camera hardware is connected and producing frames."""

    @abstractmethod
    def latest(self, camera_name: str) -> CameraFrame | None:
        """Return a real synchronized frame, or ``None`` when unavailable."""

    @abstractmethod
    def metadata(self) -> dict[str, object]:
        """Return episode-level camera availability metadata."""


class NullCameraProvider(CameraProvider):
    @property
    def present(self) -> bool:
        return False

    def latest(self, camera_name: str) -> CameraFrame | None:
        del camera_name
        return None

    def metadata(self) -> dict[str, object]:
        return {
            "head_camera_present": False,
            "wrist_camera_present": False,
            "multimodal_training_ready": False,
            "provider": "null",
            "master_clock": "time.monotonic_ns()",
        }


class RealSenseCameraProvider(CameraProvider):
    """Future RealSense seam; construction cannot masquerade as connection."""

    @property
    def present(self) -> bool:
        return False

    def latest(self, camera_name: str) -> CameraFrame | None:
        raise NotImplementedError(
            f"RealSense provider is not connected/configured for {camera_name!r}"
        )

    def metadata(self) -> dict[str, object]:
        return {
            "head_camera_present": False,
            "wrist_camera_present": False,
            "multimodal_training_ready": False,
            "provider": "realsense_future_interface",
            "master_clock": "time.monotonic_ns()",
        }


def calibration_to_json(calibration: CameraCalibration) -> dict[str, object]:
    calibration.validate()
    result = asdict(calibration)
    for key in ("transform_parent_camera", "intrinsics", "distortion"):
        result[key] = np.asarray(result[key]).tolist()
    return result
