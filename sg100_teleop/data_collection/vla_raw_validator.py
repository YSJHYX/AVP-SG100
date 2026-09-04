"""Off 后执行的 lightweight raw episode validation/summary。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import numpy as np


_EPISODE_METADATA_KEYS = {
    "schema_name", "schema_version", "episode_index", "language_instruction",
    "dataset_hz", "episode_start_monotonic_ns", "episode_start_wall_ns",
    "episode_stop_request_monotonic_ns", "robot_joint_names",
    "dataset_status", "training_ready", "dual_camera_synchronization",
    "camera_device_clock_domains_shared", "sg100_feedback_map_status",
    "media_writer_max_backlog", "media_writer_drop_count",
    "media_writer_error_count", "telemetry_received_packets",
    "telemetry_sequence_gap_count", "validation_quality",
    "max_arm_command_age_ns", "max_arm_feedback_age_ns",
    "max_hand_command_age_ns", "max_hand_feedback_age_ns",
    "max_camera_age_ns", "max_telemetry_age_ns",
}


@dataclass(frozen=True)
class ValidationReport:
    quality: str
    errors: tuple[str, ...]
    warnings: tuple[str, ...]
    metrics: dict[str, float | int | str]


def _valid_ratio(arrays, key: str) -> float:
    value = np.asarray(arrays[key], dtype=bool)
    return float(value.mean()) if value.size else 0.0


def _valid_ages(arrays, age_key: str, valid_key: str) -> tuple[float, float]:
    age = np.asarray(arrays[age_key], dtype=np.float64)
    valid = np.asarray(arrays[valid_key], dtype=bool)
    selected = age[valid]
    if selected.size == 0:
        return float("nan"), float("nan")
    return float(selected.mean()), float(selected.max())


def _duplicates(values: np.ndarray) -> int:
    values = np.asarray(values, dtype=np.int64)
    if values.size < 2:
        return 0
    return int(np.count_nonzero(values[1:] == values[:-1]))


def validate_episode(
    arrays: Mapping[str, np.ndarray],
    *,
    media_root: str | Path,
    writer_drop_count: int,
    writer_error_count: int,
    telemetry_sequence_gap_count: int,
) -> ValidationReport:
    timestamp = np.asarray(arrays["timestamp_ns"], dtype=np.int64)
    n = int(timestamp.size)
    errors: list[str] = []
    warnings: list[str] = []
    for key, value in arrays.items():
        array = np.asarray(value)
        camera_metadata = (
            (key.startswith("head_camera_") or key.startswith("wrist_camera_"))
            and not key.endswith((
                "host_timestamp_ns", "device_timestamp_ms", "frame_number",
                "age_ns", "valid",
            ))
        ) or key in ("head_depth_scale", "wrist_depth_scale")
        if (
            key not in _EPISODE_METADATA_KEYS
            and not camera_metadata
            and array.ndim >= 1
            and array.shape[0] != n
        ):
            errors.append(f"{key} frame_count={array.shape[0]} != {n}")
    if n == 0:
        errors.append("episode 没有 dataset tick")
    if n > 1 and np.any(np.diff(timestamp) <= 0):
        errors.append("timestamp_ns 不是严格递增")
    causal_pairs = (
        "arm_qcmd_source_timestamp_ns", "arm_qpos_source_timestamp_ns",
        "hand_qcmd_source_timestamp_ns", "hand_feedback_source_timestamp_ns",
        "head_camera_host_timestamp_ns", "wrist_camera_host_timestamp_ns",
    )
    for key in causal_pairs:
        source = np.asarray(arrays[key], dtype=np.int64)
        if np.any((source > 0) & (source > timestamp)):
            errors.append(f"{key} 存在 future sample")

    media = Path(media_root)
    expected = n
    for role in ("head", "right_wrist"):
        for modality, suffix in (("rgb", ".jpg"), ("depth", ".png")):
            count = len(tuple((media / role / modality).glob(f"*{suffix}")))
            if count != expected:
                errors.append(f"{role}/{modality} files={count} != {expected}")
    if writer_drop_count:
        errors.append(f"media writer drop={writer_drop_count}")
    if writer_error_count:
        errors.append(f"media writer error={writer_error_count}")
    if telemetry_sequence_gap_count:
        warnings.append(f"telemetry sequence gap={telemetry_sequence_gap_count}")

    jitter = np.asarray(arrays["sampler_jitter_ns"], dtype=np.float64)
    duration_s = float((timestamp[-1] - timestamp[0]) * 1e-9) if n > 1 else 0.0
    effective_hz = float((n - 1) / duration_s) if duration_s > 0 else 0.0
    metrics: dict[str, float | int | str] = {
        "frames": n,
        "duration_s": duration_s,
        "effective_hz": effective_hz,
        "jitter_mean_ms": float(jitter.mean() * 1e-6) if n else float("nan"),
        "jitter_p95_ms": float(np.percentile(jitter, 95) * 1e-6) if n else float("nan"),
        "jitter_p99_ms": float(np.percentile(jitter, 99) * 1e-6) if n else float("nan"),
        "jitter_max_ms": float(jitter.max() * 1e-6) if n else float("nan"),
        "arm_qcmd_valid_ratio": _valid_ratio(arrays, "arm_command_valid"),
        "arm_qpos_valid_ratio": _valid_ratio(arrays, "arm_feedback_valid"),
        "hand_qcmd_valid_ratio": _valid_ratio(arrays, "hand_command_valid"),
        "hand_feedback_valid_ratio": _valid_ratio(arrays, "hand_feedback_valid"),
        "hand_canonical_valid_ratio": _valid_ratio(arrays, "hand_feedback_canonical_valid"),
        "head_valid_ratio": _valid_ratio(arrays, "head_camera_valid"),
        "wrist_valid_ratio": _valid_ratio(arrays, "wrist_camera_valid"),
        "head_duplicates": _duplicates(arrays["head_camera_frame_number"]),
        "wrist_duplicates": _duplicates(arrays["wrist_camera_frame_number"]),
        "telemetry_packet_loss_sequence_gaps": int(telemetry_sequence_gap_count),
        "media_writer_drop_count": int(writer_drop_count),
        "media_writer_error_count": int(writer_error_count),
    }
    for prefix, age_key, valid_key in (
        ("arm_qcmd", "arm_qcmd_age_ns", "arm_command_valid"),
        ("arm_qpos", "arm_qpos_age_ns", "arm_feedback_valid"),
        ("hand_qcmd", "hand_qcmd_age_ns", "hand_command_valid"),
        ("hand_feedback", "hand_feedback_age_ns", "hand_feedback_valid"),
        ("head", "head_camera_age_ns", "head_camera_valid"),
        ("wrist", "wrist_camera_age_ns", "wrist_camera_valid"),
    ):
        mean_ns, max_ns = _valid_ages(arrays, age_key, valid_key)
        metrics[f"{prefix}_age_mean_ms"] = mean_ns * 1e-6
        metrics[f"{prefix}_age_max_ms"] = max_ns * 1e-6
    quality = "FAIL" if errors else ("WARN" if warnings else "PASS")
    return ValidationReport(quality, tuple(errors), tuple(warnings), metrics)


def format_report(report: ValidationReport) -> str:
    lines = [f"quality: {report.quality}"]
    lines.extend(f"{key}: {value}" for key, value in report.metrics.items())
    lines.extend(f"ERROR: {value}" for value in report.errors)
    lines.extend(f"WARN: {value}" for value in report.warnings)
    return "\n".join(lines)
