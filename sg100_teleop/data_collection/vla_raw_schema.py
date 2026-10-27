"""30 Hz Arm-Hand + Dual RealSense raw VLA schema。"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
import re
from pathlib import Path

import numpy as np


SCHEMA_NAME = "rm65_sg100_dual_realsense_vla_raw"
SCHEMA_VERSION = 4
HAND_ACTION_DOMAIN = "sdk_user_rad"
DATASET_STATUS = "RAW_CAPTURE_QUARANTINED"
TRAINING_READY = False
ARM_JOINT_NAMES = tuple(f"rm65_joint{i}" for i in range(1, 7))
HAND_JOINT_NAMES = (
    "thumb_j1", "thumb_j2", "thumb_j3",
    "index_j1", "index_j2", "index_j3",
    "middle_j1", "middle_j2",
    "little_j1", "little_j2", "little_j3",
)
ROBOT_JOINT_NAMES = ARM_JOINT_NAMES + HAND_JOINT_NAMES
SUPPORTED_CAMERA_ROLES = ("head", "right_wrist")
DEFAULT_REQUIRED_CAMERAS = SUPPORTED_CAMERA_ROLES
SUPPORTED_CAMERA_MODALITIES = ("rgb", "depth")
DEFAULT_CAMERA_MODALITIES = ("rgb",)
LEGACY_CAMERA_MODALITIES = SUPPORTED_CAMERA_MODALITIES
EPISODE_PATTERN = re.compile(r"^episode_(\d{6})\.npz$")


@dataclass(frozen=True)
class FreshnessThresholds:
    max_arm_command_age_ns: int = 100_000_000
    max_arm_feedback_age_ns: int = 100_000_000
    max_hand_command_age_ns: int = 100_000_000
    max_hand_feedback_age_ns: int = 100_000_000
    max_camera_age_ns: int = 100_000_000
    max_telemetry_age_ns: int = 100_000_000

    def __post_init__(self) -> None:
        if any(int(value) <= 0 for value in self.__dict__.values()):
            raise ValueError("freshness thresholds 必须全部 > 0")

    def as_metadata(self) -> dict[str, np.ndarray]:
        return {
            key: np.asarray(int(value), dtype=np.int64)
            for key, value in self.__dict__.items()
        }


class FixedRateTimeline:
    """用整数绝对时间计算 tick，不累计 sleep 漂移。"""

    def __init__(self, hz: float, start_monotonic_ns: int) -> None:
        rate = Fraction(str(float(hz))).limit_denominator(1_000_000)
        if rate <= 0:
            raise ValueError("dataset hz 必须 > 0")
        self.hz = float(rate)
        self.start_monotonic_ns = int(start_monotonic_ns)
        self._numerator = rate.numerator
        self._denominator = rate.denominator

    def tick_ns(self, index: int) -> int:
        if int(index) < 0:
            raise ValueError("tick index 必须 >= 0")
        offset = (
            int(index) * 1_000_000_000 * self._denominator
        ) // self._numerator
        return self.start_monotonic_ns + offset


def scan_next_episode_index(output_dir: str | Path) -> int:
    root = Path(output_dir)
    indices: list[int] = []
    if root.exists():
        for path in root.iterdir():
            match = EPISODE_PATTERN.match(path.name)
            if match:
                indices.append(int(match.group(1)))
    return max(indices) + 1 if indices else 0


def parse_required_cameras(value: str | tuple[str, ...]) -> tuple[str, ...]:
    """解析正式支持的 recorder camera role 组合。"""
    if isinstance(value, str):
        roles = tuple(
            item.strip() for item in value.split(",") if item.strip()
        )
    else:
        roles = tuple(str(item).strip() for item in value if str(item).strip())
    roles = tuple(dict.fromkeys(roles))
    if roles not in (("head",), DEFAULT_REQUIRED_CAMERAS):
        raise ValueError(
            "--required-cameras 仅支持 head 或 head,right_wrist"
        )
    return roles


def parse_camera_modalities(value: str | tuple[str, ...]) -> tuple[str, ...]:
    """解析 recorder modality；RGB 是 VLA recorder 的强制基础模态。"""
    if isinstance(value, str):
        requested = tuple(
            item.strip().lower() for item in value.split(",") if item.strip()
        )
    else:
        requested = tuple(
            str(item).strip().lower() for item in value if str(item).strip()
        )
    requested = tuple(dict.fromkeys(requested))
    unknown = set(requested) - set(SUPPORTED_CAMERA_MODALITIES)
    if unknown:
        raise ValueError(f"未知 camera modality: {sorted(unknown)}")
    if "rgb" not in requested:
        raise ValueError("RGB is required for VLA recorder.")
    return tuple(
        modality for modality in SUPPORTED_CAMERA_MODALITIES
        if modality in requested
    )


def vector(value, size: int, *, fill: float = np.nan) -> np.ndarray:
    if value is None:
        return np.full(size, fill, dtype=np.float64)
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (size,) or not np.all(np.isfinite(array)):
        return np.full(size, fill, dtype=np.float64)
    return array.copy()
