"""Curated episode 落盘前的只读契约校验；builder 与 inspector 共用。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import numpy as np

from sg100_teleop.data_collection.vla_curated_schema import (
    ARM_SLICE,
    CURATED_SCHEMA_NAME,
    FORBIDDEN_TRAJECTORY_KEYS,
    HAND_SLICE,
    METADATA_REQUIRED_KEYS,
    ROBOT_JOINT_NAMES,
    TRAJECTORY_CORE_KEYS,
    TRAJECTORY_OPTIONAL_KEYS,
    TRAJECTORY_VECTOR_KEYS,
)


@dataclass(frozen=True)
class CuratedValidationReport:
    quality: str
    errors: tuple[str, ...]
    warnings: tuple[str, ...]
    transition_count: int


def validate_curated_episode(
    metadata: Mapping[str, object],
    trajectory: Mapping[str, np.ndarray],
    *,
    media_root: str | Path | None = None,
    allow_empty: bool = False,
) -> CuratedValidationReport:
    errors: list[str] = []
    warnings: list[str] = []

    if metadata.get("schema_name") != CURATED_SCHEMA_NAME:
        errors.append(f"schema_name 非法: {metadata.get('schema_name')!r}")
    missing_metadata = [
        key for key in METADATA_REQUIRED_KEYS if key not in metadata
    ]
    if missing_metadata:
        errors.append(f"metadata 缺少字段: {sorted(missing_metadata)}")
    joint_names = list(metadata.get("joint_names", ()))
    if tuple(joint_names) != tuple(ROBOT_JOINT_NAMES):
        errors.append("joint_names 与 canonical 17D ordering 不一致")
    if list(metadata.get("arm_slice", ())) != list(ARM_SLICE):
        errors.append(f"arm_slice 必须是 {list(ARM_SLICE)}")
    if list(metadata.get("hand_slice", ())) != list(HAND_SLICE):
        errors.append(f"hand_slice 必须是 {list(HAND_SLICE)}")
    camera_roles = tuple(metadata.get("camera_roles", ()))
    if "head" not in camera_roles or set(camera_roles) - {
        "head", "right_wrist"
    }:
        errors.append(f"camera_roles 非法: {list(camera_roles)}")

    present = set(trajectory)
    missing = set(TRAJECTORY_CORE_KEYS) - present
    if missing:
        errors.append(f"trajectory 缺少字段: {sorted(missing)}")
    unknown = present - set(TRAJECTORY_CORE_KEYS) - set(
        TRAJECTORY_OPTIONAL_KEYS
    )
    if unknown:
        errors.append(f"trajectory 含未知字段: {sorted(unknown)}")
    forbidden = present & FORBIDDEN_TRAJECTORY_KEYS
    if forbidden:
        errors.append(f"trajectory 含 NO SAVE 字段: {sorted(forbidden)}")

    lengths: dict[str, int] = {}
    for key in sorted(present):
        array = np.asarray(trajectory[key])
        if key in TRAJECTORY_VECTOR_KEYS:
            width = TRAJECTORY_VECTOR_KEYS[key]
            if array.ndim != 2 or array.shape[1] != width:
                errors.append(f"{key} shape={array.shape} != (M, {width})")
                continue
            lengths[key] = int(array.shape[0])
        else:
            if array.ndim != 1:
                errors.append(f"{key} 必须是一维数组")
                continue
            lengths[key] = int(array.shape[0])
    m_values = set(lengths.values())
    if len(m_values) > 1:
        errors.append(f"trajectory 行数不一致: {sorted(m_values)}")
    m = m_values.pop() if len(m_values) == 1 else 0
    if m == 0 and not allow_empty:
        errors.append("trajectory 没有 transition")

    offsets = list(metadata.get("segment_offsets", ()))
    if m == 0:
        if offsets:
            errors.append("空 trajectory 的 segment_offsets 必须为空")
    else:
        if not offsets:
            errors.append("metadata 缺少有效 segment_offsets")
        elif (
            offsets[0] != 0
            or offsets[-1] != m
            or any(
                later <= earlier
                for earlier, later in zip(offsets, offsets[1:])
            )
        ):
            errors.append(
                f"segment_offsets 必须以 0 开头、{m} 结尾且严格递增: {offsets}"
            )
        boundaries = set(offsets[:-1])

        source_tick = np.asarray(
            trajectory["source_tick_index"], dtype=np.int64
        )
        next_tick = np.asarray(
            trajectory["next_state_tick_index"], dtype=np.int64
        )
        if not np.all(next_tick == source_tick + 1):
            errors.append("next_state_tick_index 必须等于 source_tick_index+1")
        if m > 1 and np.any(np.diff(source_tick) <= 0):
            errors.append("source_tick_index 不是严格递增")
        for index in range(1, m):
            if source_tick[index] != next_tick[index - 1]:
                if index not in boundaries:
                    errors.append(
                        f"tick 不连续处必须切 segment: row {index}"
                    )
            elif index in boundaries:
                pass  # 事件切分（estop/tracking/telemetry）允许在连续 tick 处

        arm_state_ts = np.asarray(
            trajectory["arm_qpos_source_timestamp_ns"], dtype=np.int64
        )
        hand_state_ts = np.asarray(
            trajectory["hand_qpos_source_timestamp_ns"], dtype=np.int64
        )
        arm_action_ts = np.asarray(
            trajectory["arm_qcmd_source_timestamp_ns"], dtype=np.int64
        )
        hand_action_ts = np.asarray(
            trajectory["hand_qcmd_source_timestamp_ns"], dtype=np.int64
        )
        for key in (arm_state_ts, hand_state_ts, arm_action_ts, hand_action_ts):
            if np.any(key <= 0):
                errors.append("native source timestamp 必须全部 > 0")
        # component-wise（行内）：state 组件 ts <= action 组件 ts；跨行 next 侧
        # 因果由 cleaner 依据 RAW 验证，此处验证可从 Curated 推导的部分。
        if np.any(arm_state_ts > arm_action_ts):
            errors.append("ARM component causality: state ts <= action ts 不成立")
        if np.any(hand_state_ts > hand_action_ts):
            errors.append("HAND component causality: state ts <= action ts 不成立")
        state_ts = np.asarray(trajectory["state_timestamp_ns"], dtype=np.int64)
        action_ts = np.asarray(
            trajectory["action_timestamp_ns"], dtype=np.int64
        )
        next_ts = np.asarray(
            trajectory["next_state_timestamp_ns"], dtype=np.int64
        )
        if not np.array_equal(state_ts, np.maximum(arm_state_ts, hand_state_ts)):
            errors.append("state_timestamp_ns 必须等于两侧 state source ts 的 max")
        if not np.array_equal(
            action_ts, np.maximum(arm_action_ts, hand_action_ts)
        ):
            errors.append("action_timestamp_ns 必须等于两侧 action source ts 的 max")
        if np.any(state_ts > action_ts) or np.any(action_ts >= next_ts):
            errors.append("unified causality: state <= action < next_state 不成立")

        for key in ("robot_qpos_17d_rad", "robot_qcmd_17d_rad"):
            values = np.asarray(trajectory[key], dtype=np.float64)
            if not np.all(np.isfinite(values)):
                errors.append(f"{key} 存在非有限值；Curated 只允许 valid 行")
        if "robot_desired_qcmd_17d_rad" in trajectory:
            desired = np.asarray(
                trajectory["robot_desired_qcmd_17d_rad"], dtype=np.float64
            )
            row_finite = np.all(np.isfinite(desired), axis=1)
            mixed = np.any(np.isfinite(desired[~row_finite]), axis=1)
            if np.any(mixed):
                errors.append("desired17 行必须整行 finite 或整行 NaN")

        head_index = np.asarray(
            trajectory["head_rgb_frame_index"], dtype=np.int64
        )
        if np.any(head_index < 0):
            errors.append("head_rgb_frame_index 必须 >= 0")
        wrist_index = np.asarray(
            trajectory["right_wrist_rgb_frame_index"], dtype=np.int64
        )
        if "right_wrist" in camera_roles:
            if np.any(wrist_index < 0):
                errors.append("wrist 启用时 right_wrist_rgb_frame_index 必须 >= 0")
        elif np.any(wrist_index != -1):
            errors.append("wrist 未启用时 right_wrist_rgb_frame_index 必须=-1")

    if media_root is not None and m > 0:
        media = Path(media_root)
        for role, index_array in (
            ("head", trajectory["head_rgb_frame_index"]),
            ("right_wrist", trajectory["right_wrist_rgb_frame_index"]),
        ):
            if role == "right_wrist" and role not in camera_roles:
                continue
            referenced = set(
                int(value)
                for value in np.asarray(index_array, dtype=np.int64).tolist()
                if value >= 0
            )
            rgb_dir = media / role / "rgb"
            existing = {
                int(path.stem)
                for path in rgb_dir.glob("*.jpg")
            } if rgb_dir.is_dir() else set()
            if referenced != existing:
                errors.append(
                    f"{role}/rgb media 与引用 index 不一致: "
                    f"referenced={sorted(referenced)} files={sorted(existing)}"
                )
            if (media / role / "depth").exists():
                errors.append(f"Curated 禁止 depth 目录: {role}/depth")
        for role in ("head", "right_wrist"):
            if role not in camera_roles and (media / role).exists():
                errors.append(f"未启用 role 不应有 media 目录: {role}")

    quality = "FAIL" if errors else ("WARN" if warnings else "PASS")
    return CuratedValidationReport(
        quality=quality,
        errors=tuple(errors),
        warnings=tuple(warnings),
        transition_count=m,
    )
