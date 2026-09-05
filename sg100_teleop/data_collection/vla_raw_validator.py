"""Off 后执行的 lightweight raw episode validation/summary。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import numpy as np

from sg100_teleop.data_collection.vla_raw_schema import (
    DEFAULT_CAMERA_MODALITIES,
    DEFAULT_REQUIRED_CAMERAS,
    ROBOT_JOINT_NAMES,
    parse_camera_modalities,
    parse_required_cameras,
)


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
    "required_cameras", "hardware_execution", "robot_telemetry_present",
    "camera_configuration_mode", "camera_modalities", "record_depth",
    "head_depth_present", "wrist_depth_present",
    "head_color_intrinsics", "head_depth_intrinsics",
    "wrist_color_intrinsics", "wrist_depth_intrinsics",
    "robot_desired_qcmd_semantics", "robot_effective_qcmd_semantics",
    "robot_qpos_semantics", "telemetry_publish_hz_requested",
    "telemetry_publish_hz_effective",
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
    values = values[values >= 0]
    if values.size < 2:
        return 0
    return int(np.count_nonzero(values[1:] == values[:-1]))


def telemetry_rate_diagnostics(
    arrays: Mapping[str, np.ndarray],
) -> dict[str, float]:
    """从 sampled publish sequence/timestamp 估计 publisher caller 实际频率。"""
    sequence = np.asarray(
        arrays.get("telemetry_publish_sequence", []), dtype=np.int64
    )
    if "telemetry_publish_timestamp_ns" in arrays:
        publish_ns = np.asarray(
            arrays["telemetry_publish_timestamp_ns"], dtype=np.int64
        )
    else:
        tick_ns = np.asarray(arrays.get("timestamp_ns", []), dtype=np.int64)
        age_ns = np.asarray(arrays.get("telemetry_age_ns", []), dtype=np.int64)
        publish_ns = np.where(age_ns >= 0, tick_ns - age_ns, 0)
    valid = (sequence >= 0) & (publish_ns > 0)
    sequence = sequence[valid]
    publish_ns = publish_ns[valid]
    if sequence.size:
        keep = np.r_[True, sequence[1:] != sequence[:-1]]
        sequence = sequence[keep]
        publish_ns = publish_ns[keep]
    delta_sequence = np.diff(sequence)
    delta_ns = np.diff(publish_ns)
    advancing = (delta_sequence > 0) & (delta_ns > 0)
    if np.any(advancing):
        effective_hz = float(
            delta_sequence[advancing].sum()
            / (delta_ns[advancing].sum() * 1e-9)
        )
    else:
        effective_hz = float("nan")
    ages = np.asarray(arrays.get("telemetry_age_ns", []), dtype=np.float64)
    ages = ages[ages >= 0]
    return {
        "telemetry_effective_hz": effective_hz,
        "telemetry_age_mean_ms": (
            float(ages.mean() * 1e-6) if ages.size else float("nan")
        ),
        "telemetry_age_p95_ms": (
            float(np.percentile(ages, 95) * 1e-6)
            if ages.size else float("nan")
        ),
        "telemetry_age_p99_ms": (
            float(np.percentile(ages, 99) * 1e-6)
            if ages.size else float("nan")
        ),
        "telemetry_age_max_ms": (
            float(ages.max() * 1e-6) if ages.size else float("nan")
        ),
    }


def validate_episode(
    arrays: Mapping[str, np.ndarray],
    *,
    media_root: str | Path,
    writer_drop_count: int,
    writer_error_count: int,
    telemetry_sequence_gap_count: int,
    enabled_camera_roles: tuple[str, ...] | None = None,
    enabled_camera_modalities: tuple[str, ...] | None = None,
) -> ValidationReport:
    timestamp = np.asarray(arrays["timestamp_ns"], dtype=np.int64)
    schema_version = int(np.asarray(arrays.get("schema_version", 1)).item())
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
        "telemetry_publish_timestamp_ns",
        "head_camera_host_timestamp_ns", "wrist_camera_host_timestamp_ns",
        "head_depth_host_timestamp_ns", "wrist_depth_host_timestamp_ns",
    )
    for key in causal_pairs:
        if key not in arrays:  # v1 H1 episodes share the camera timestamp.
            continue
        source = np.asarray(arrays[key], dtype=np.int64)
        if np.any((source > 0) & (source > timestamp)):
            errors.append(f"{key} 存在 future sample")

    expected_joint_names = np.asarray(ROBOT_JOINT_NAMES)
    actual_joint_names = np.asarray(arrays.get("robot_joint_names", []))
    if schema_version >= 3 and not np.array_equal(
        actual_joint_names, expected_joint_names
    ):
        errors.append("robot_joint_names 与 v3 canonical 17D ordering 不一致")
    if schema_version >= 3:
        required_v3 = {
            "robot_desired_qcmd_17d_rad",
            "robot_desired_qcmd_17d_valid",
            "telemetry_publish_timestamp_ns",
            "robot_desired_qcmd_semantics",
            "robot_effective_qcmd_semantics",
            "robot_qpos_semantics",
            "telemetry_publish_hz_requested",
            "telemetry_publish_hz_effective",
        }
        missing_v3 = required_v3 - set(arrays)
        if missing_v3:
            errors.append(f"v3 schema 缺少字段: {sorted(missing_v3)}")
        if {
            "robot_desired_qcmd_17d_rad",
            "robot_desired_qcmd_17d_valid",
        }.issubset(arrays):
            desired = np.asarray(
                arrays["robot_desired_qcmd_17d_rad"], dtype=np.float64
            )
            desired_valid = np.asarray(
                arrays["robot_desired_qcmd_17d_valid"], dtype=bool
            )
            if desired.shape != (n, 17):
                errors.append(
                    f"robot_desired_qcmd_17d_rad shape={desired.shape} != {(n, 17)}"
                )
            elif desired_valid.shape != (n,):
                errors.append(
                    "robot_desired_qcmd_17d_valid shape "
                    f"{desired_valid.shape} != {(n,)}"
                )
            else:
                if np.any(~np.all(np.isfinite(desired[desired_valid]), axis=1)):
                    errors.append("desired17 valid=True row 必须全部 finite")
                if np.any(np.isfinite(desired[~desired_valid])):
                    errors.append("desired17 valid=False row 必须全部为 NaN")
                arm_goal = np.asarray(arrays["arm_q_goal_rad"], dtype=np.float64)
                hand_goal = np.asarray(
                    arrays["hand_qcmd_retargeted_canonical_rad"],
                    dtype=np.float64,
                )
                if not np.allclose(
                    desired[desired_valid, :6], arm_goal[desired_valid],
                    equal_nan=True,
                ):
                    errors.append("desired17 前 6D 不等于 arm_q_goal_rad")
                if not np.allclose(
                    desired[desired_valid, 6:], hand_goal[desired_valid],
                    equal_nan=True,
                ):
                    errors.append(
                        "desired17 后 11D 不等于 hand desired canonical qcmd"
                    )

    for combined_key, first_key, second_key, first_valid_key, second_valid_key, label in (
        (
            "robot_qcmd_17d_rad", "arm_qcmd_sent_rad",
            "hand_qcmd_effective_canonical_rad", "arm_command_valid",
            "hand_command_valid", "effective17",
        ),
        (
            "robot_qpos_17d_rad", "arm_qpos_rad",
            "hand_qpos_canonical_rad", "arm_feedback_valid",
            "hand_feedback_canonical_valid", "measured17",
        ),
    ):
        combined = np.asarray(arrays[combined_key], dtype=np.float64)
        first = np.asarray(arrays[first_key], dtype=np.float64).copy()
        second = np.asarray(arrays[second_key], dtype=np.float64).copy()
        if schema_version >= 3:
            first[~np.asarray(arrays[first_valid_key], dtype=bool)] = np.nan
            second[~np.asarray(arrays[second_valid_key], dtype=bool)] = np.nan
        expected_combined = np.concatenate(
            (first, second),
            axis=1,
        )
        if combined.shape != (n, 17):
            errors.append(f"{combined_key} shape={combined.shape} != {(n, 17)}")
        elif not np.allclose(combined, expected_combined, equal_nan=True):
            errors.append(f"{label} 不等于其 authoritative component fields")

    if enabled_camera_roles is None:
        stored_roles = arrays.get("required_cameras")
        enabled_camera_roles = (
            parse_required_cameras(tuple(np.asarray(stored_roles).tolist()))
            if stored_roles is not None else DEFAULT_REQUIRED_CAMERAS
        )
    if enabled_camera_modalities is None:
        stored_modalities = arrays.get("camera_modalities")
        enabled_camera_modalities = (
            parse_camera_modalities(str(np.asarray(stored_modalities).item()))
            if stored_modalities is not None else DEFAULT_CAMERA_MODALITIES
        )
    if "camera_modalities" in arrays:
        modality_contract_keys = {
            "record_depth", "head_depth_present", "wrist_depth_present",
            "head_depth_host_timestamp_ns", "head_depth_device_timestamp_ms",
            "head_depth_frame_number", "head_depth_age_ns", "head_depth_valid",
            "wrist_depth_host_timestamp_ns", "wrist_depth_device_timestamp_ms",
            "wrist_depth_frame_number", "wrist_depth_age_ns",
            "wrist_depth_valid", "head_depth_frame_index",
            "wrist_depth_frame_index",
        }
        missing = modality_contract_keys - set(arrays)
        if missing:
            errors.append(f"camera modality schema 缺少字段: {sorted(missing)}")
    enabled_roles = set(enabled_camera_roles)
    enabled_modalities = set(enabled_camera_modalities)
    if "head" not in enabled_roles or enabled_roles - {"head", "right_wrist"}:
        errors.append(f"非法 enabled camera roles: {sorted(enabled_roles)}")
    if not bool(np.asarray(arrays["head_camera_present"]).item()):
        errors.append("required head camera metadata present=False")
    wrist_present = bool(np.asarray(arrays["wrist_camera_present"]).item())
    if wrist_present != ("right_wrist" in enabled_roles):
        errors.append("wrist_camera_present 与 enabled role 不一致")
    depth_expected = "depth" in enabled_modalities
    if "record_depth" in arrays and bool(
        np.asarray(arrays["record_depth"]).item()
    ) != depth_expected:
        errors.append("record_depth 与 camera_modalities 不一致")
    for prefix, role in (("head", "head"), ("wrist", "right_wrist")):
        key = f"{prefix}_depth_present"
        expected_present = role in enabled_roles and depth_expected
        if key in arrays and bool(np.asarray(arrays[key]).item()) != expected_present:
            errors.append(f"{key} 与 role/modality 配置不一致")
    media = Path(media_root)
    expected = n
    for role in ("head", "right_wrist"):
        for modality, suffix in (("rgb", ".jpg"), ("depth", ".png")):
            count = len(tuple((media / role / modality).glob(f"*{suffix}")))
            role_expected = (
                expected
                if role in enabled_roles and modality in enabled_modalities else 0
            )
            if count != role_expected:
                errors.append(
                    f"{role}/{modality} files={count} != {role_expected}"
                )
            if (
                modality not in enabled_modalities
                and (media / role / modality).exists()
            ):
                errors.append(f"disabled modality directory 不应存在: {role}/{modality}")
    if "right_wrist" not in enabled_roles:
        disabled_checks = {
            "wrist_camera_valid": False,
            "wrist_rgb_frame_index": -1,
            "wrist_depth_frame_index": -1,
            "wrist_camera_host_timestamp_ns": -1,
            "wrist_camera_frame_number": -1,
            "wrist_camera_age_ns": -1,
        }
        for key, expected_value in disabled_checks.items():
            if not np.all(np.asarray(arrays[key]) == expected_value):
                errors.append(f"disabled right_wrist sentinel 非法: {key}")
        if not np.all(np.isnan(np.asarray(
            arrays["wrist_camera_device_timestamp_ms"], dtype=np.float64
        ))):
            errors.append("disabled right_wrist device timestamp 必须全为 NaN")
    for prefix, role in (("head", "head"), ("wrist", "right_wrist")):
        if depth_expected and role in enabled_roles:
            continue
        disabled_depth_checks = {
            f"{prefix}_depth_valid": False,
            f"{prefix}_depth_frame_index": -1,
            f"{prefix}_depth_host_timestamp_ns": -1,
            f"{prefix}_depth_frame_number": -1,
            f"{prefix}_depth_age_ns": -1,
        }
        for key, expected_value in disabled_depth_checks.items():
            if key in arrays and not np.all(
                np.asarray(arrays[key]) == expected_value
            ):
                errors.append(f"disabled depth sentinel 非法: {key}")
        device_key = f"{prefix}_depth_device_timestamp_ms"
        if device_key in arrays and not np.all(np.isnan(np.asarray(
            arrays[device_key], dtype=np.float64
        ))):
            errors.append(f"disabled depth device timestamp 必须全为 NaN: {prefix}")
    if writer_drop_count:
        errors.append(f"media writer drop={writer_drop_count}")
    if writer_error_count:
        errors.append(f"media writer error={writer_error_count}")
    if telemetry_sequence_gap_count:
        warnings.append(f"telemetry sequence gap={telemetry_sequence_gap_count}")

    robot_telemetry_present = bool(
        np.asarray(arrays["robot_telemetry_present"]).item()
    )
    if not robot_telemetry_present:
        robot_value_keys = (
            "arm_q_goal_rad", "arm_qcmd_sent_rad", "arm_qpos_rad",
            "hand_qcmd_retargeted_canonical_rad",
            "hand_qcmd_effective_canonical_rad", "hand_qcmd_sdk_deg",
            "hand_feedback_sdk_rad", "hand_qpos_canonical_rad",
            "robot_qcmd_17d_rad", "robot_qpos_17d_rad",
        )
        if "robot_desired_qcmd_17d_rad" in arrays:
            robot_value_keys += ("robot_desired_qcmd_17d_rad",)
        for key in robot_value_keys:
            if not np.all(np.isnan(np.asarray(arrays[key], dtype=np.float64))):
                errors.append(f"telemetry absent 时 {key} 必须全为 NaN")
        robot_valid_keys = (
            "arm_command_valid", "arm_feedback_valid", "hand_command_valid",
            "hand_feedback_valid", "hand_feedback_canonical_valid",
            "robot_qcmd_17d_valid", "robot_qpos_17d_valid",
        )
        if "robot_desired_qcmd_17d_valid" in arrays:
            robot_valid_keys += ("robot_desired_qcmd_17d_valid",)
        for key in robot_valid_keys:
            if np.any(np.asarray(arrays[key], dtype=bool)):
                errors.append(f"telemetry absent 时 {key} 必须全为 False")

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
        "robot_desired_qcmd_17d_valid_ratio": (
            _valid_ratio(arrays, "robot_desired_qcmd_17d_valid")
            if "robot_desired_qcmd_17d_valid" in arrays else float("nan")
        ),
        "robot_qcmd_17d_valid_ratio": _valid_ratio(
            arrays, "robot_qcmd_17d_valid"
        ),
        "robot_qpos_17d_valid_ratio": _valid_ratio(
            arrays, "robot_qpos_17d_valid"
        ),
        "head_valid_ratio": _valid_ratio(arrays, "head_camera_valid"),
        "wrist_valid_ratio": _valid_ratio(arrays, "wrist_camera_valid"),
        "head_depth_valid_ratio": (
            _valid_ratio(arrays, "head_depth_valid")
            if "head_depth_valid" in arrays
            else _valid_ratio(arrays, "head_camera_valid")
        ),
        "wrist_depth_valid_ratio": (
            _valid_ratio(arrays, "wrist_depth_valid")
            if "wrist_depth_valid" in arrays
            else _valid_ratio(arrays, "wrist_camera_valid")
        ),
        "head_duplicates": _duplicates(arrays["head_camera_frame_number"]),
        "wrist_duplicates": _duplicates(arrays["wrist_camera_frame_number"]),
        "telemetry_packet_loss_sequence_gaps": int(telemetry_sequence_gap_count),
        "media_writer_drop_count": int(writer_drop_count),
        "media_writer_error_count": int(writer_error_count),
    }
    metrics.update(telemetry_rate_diagnostics(arrays))
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
