"""Curated VLA trajectory v1 contract：只保存不可推导的训练核心与 native timing。

Curated 是 RAW (`recording/vla_raw/`) 经过 validation / cleaning / causal
transition build 后的 30 Hz 训练轨迹层，输出到 `recording/vla_curated/`。
字段原则：

- 17D 是唯一动作/状态表示；禁止同时保存拆分的 arm 6D / hand 11D 副本，
  AHFE Arm/Hand factorization 由 slice metadata 确定性完成；
- skew / dt / desired-effective 距离 / native Hz 全部由保留 timestamp 在
  运行时推导（`vla_timing_metrics`），不落盘派生数组；
- Curated 默认只包含 training-valid transition；拒绝原因只进入
  `cleaning_report.json`，不进入主 trajectory；
- telemetry / writer / sampler / SDK-degree / depth 等 diagnostics 停留在
  RAW（或 RAW validation summary），不进入 Curated。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from sg100_teleop.data_collection.vla_raw_schema import (
    ARM_JOINT_NAMES,
    HAND_JOINT_NAMES,
    ROBOT_JOINT_NAMES,
)


CURATED_SCHEMA_NAME = "rm65_sg100_dual_realsense_vla_curated"
CURATED_SCHEMA_VERSION = 1

CURATED_DATASET_STATUS_READY = "CURATED_TRAINING_READY"
CURATED_DATASET_STATUS_NOT_READY = "CURATED_NOT_TRAINING_READY"

ACTION_REPRESENTATION = "FINAL_EFFECTIVE_BACKEND_COMMAND"
ACTION_UNIT = "rad"
STATE_REPRESENTATION = "REAL_MEASURED_CANONICAL_STATE"
STATE_UNIT = "rad"

# 三层数据目录（README「π0.5-AHFE Training Data Contract」的 authority）。
DEFAULT_RAW_DIR = Path("recording/vla_raw")
DEFAULT_CURATED_DIR = Path("recording/vla_curated")
DEFAULT_MODEL_READY_DIR = Path("datasets/pi05_ahfe")

ARM_SLICE = (0, len(ARM_JOINT_NAMES))
HAND_SLICE = (len(ARM_JOINT_NAMES), len(ROBOT_JOINT_NAMES))
if ROBOT_JOINT_NAMES != ARM_JOINT_NAMES + HAND_JOINT_NAMES:
    raise AssertionError("17D ordering 常量不一致")  # pragma: no cover

# trajectory.npz 的必需字段（全部 M 长度；17D 数组为 (M, 17)）。
TRAJECTORY_CORE_KEYS = (
    "source_tick_index",
    "next_state_tick_index",
    "state_timestamp_ns",
    "action_timestamp_ns",
    "next_state_timestamp_ns",
    "arm_qpos_source_timestamp_ns",
    "hand_qpos_source_timestamp_ns",
    "arm_qcmd_source_timestamp_ns",
    "hand_qcmd_source_timestamp_ns",
    "robot_qpos_17d_rad",
    "robot_qcmd_17d_rad",
    "head_rgb_frame_index",
    "right_wrist_rgb_frame_index",
)
# 可选 provenance：第一版默认不包含，显式 --include-desired 才写入。
TRAJECTORY_OPTIONAL_KEYS = ("robot_desired_qcmd_17d_rad",)
TRAJECTORY_VECTOR_KEYS = {
    "robot_qpos_17d_rad": 17,
    "robot_qcmd_17d_rad": 17,
    "robot_desired_qcmd_17d_rad": 17,
}

# metadata.json 必需字段。
METADATA_REQUIRED_KEYS = (
    "schema_name",
    "schema_version",
    "episode_id",
    "language_instruction",
    "dataset_hz",
    "joint_names",
    "arm_slice",
    "hand_slice",
    "camera_roles",
    "source_schema_name",
    "source_schema_version",
    "source_episode_path",
    "hardware_execution",
    "training_ready",
    "dataset_status",
    "action_representation",
    "action_unit",
    "state_representation",
    "state_unit",
    "segment_offsets",
)

# NO SAVE 列表：这些字段禁止出现在 Curated trajectory / metadata 之外的
# 任何重复表示里；出现即违反存储精简契约。
FORBIDDEN_TRAJECTORY_KEYS = frozenset({
    # 17D 的重复拆分表示（AHFE 由 slice 完成）
    "arm_qcmd_6d_rad", "hand_qcmd_11d_rad",
    "arm_qpos_6d_rad", "hand_qpos_11d_rad",
    "arm_replay", "hand_replay",
    # RAW component / SDK / wire 域字段（RAW 已保存）
    "arm_q_goal_rad", "arm_qcmd_sent_rad", "arm_qpos_rad",
    "hand_qcmd_retargeted_canonical_rad",
    "hand_qcmd_effective_canonical_rad",
    "hand_qcmd_sdk_deg", "hand_feedback_sdk_rad", "hand_feedback_modes",
    "hand_qpos_canonical_rad",
    # telemetry / writer / sampler diagnostics
    "telemetry_publish_sequence", "telemetry_age_ns", "telemetry_valid",
    "telemetry_publish_timestamp_ns",
    "writer_backlog", "writer_drop_count", "media_writer_drop_count",
    "media_writer_error_count",
    "sampler_wakeup_timestamp_ns", "sampler_jitter_ns",
    "arm_qcmd_sequence", "arm_feedback_sequence",
    "hand_command_sequence", "hand_feedback_sequence",
    # 可推导字段
    "timestamp_relative_ns",
    "arm_hand_state_skew_ns", "arm_hand_action_skew_ns",
    "state_action_dt_ns", "action_next_state_dt_ns",
    "desired_effective_l2", "desired_effective_max_abs",
    "arm_command_dt_mean_ns", "hand_feedback_dt_p95_ns",
    # per-row runtime camera 字段（RAW validation 已用）
    "head_camera_host_timestamp_ns", "head_camera_device_timestamp_ms",
    "head_camera_frame_number", "head_camera_age_ns", "head_camera_valid",
    "wrist_camera_host_timestamp_ns", "wrist_camera_device_timestamp_ms",
    "wrist_camera_frame_number", "wrist_camera_age_ns",
    "wrist_camera_valid",
    # Curated=valid-only，逐行有效性冗余
    "transition_valid", "invalid_reason_mask",
    # depth：π0.5-AHFE baseline 是 RGB-only
    "head_depth_frame_index", "wrist_depth_frame_index",
    "head_depth_valid", "wrist_depth_valid",
    # next-state component timestamp 可由 next_state_tick_index 从 RAW 取得，
    # 禁止为 component-wise causality 重复保存（v1 freeze）
    "next_arm_qpos_source_timestamp_ns",
    "next_hand_qpos_source_timestamp_ns",
    "next_state_source_timestamp_ns",
})

# cleaning gate 拒绝原因 bit（只进入 cleaning_report.json 统计，
# 绝不进入 accepted trajectory）。
INVALID_REASONS = (
    "STATE_QPOS_INVALID",
    "ACTION_QCMD_INVALID",
    "NEXT_STATE_QPOS_INVALID",
    "TICK_TIMESTAMP_ORDER_INVALID",
    "ARM_QPOS_SOURCE_TIMESTAMP_INVALID",
    "HAND_QPOS_SOURCE_TIMESTAMP_INVALID",
    "ARM_QCMD_SOURCE_TIMESTAMP_INVALID",
    "HAND_QCMD_SOURCE_TIMESTAMP_INVALID",
    "ARM_CAUSALITY_INVALID",
    "HAND_CAUSALITY_INVALID",
    "UNIFIED_CAUSALITY_INVALID",
    "HEAD_RGB_INVALID",
    "HEAD_RGB_MEDIA_MISSING",
    "WRIST_RGB_INVALID",
    "WRIST_RGB_MEDIA_MISSING",
)
INVALID_REASON_BITS = {
    name: (1 << index) for index, name in enumerate(INVALID_REASONS)
}


def invalid_reason_counts(reason_mask) -> dict[str, int]:
    """把 per-transition bitmask 汇总为 reason -> 计数（只保留非零项）。"""
    mask = np.asarray(reason_mask, dtype=np.int64)
    return {
        name: int(np.count_nonzero(mask & bit))
        for name, bit in INVALID_REASON_BITS.items()
        if np.any(mask & bit)
    }


def segments_from_mask(accepted) -> tuple[int, int, float, int]:
    """连续 accepted transition 的 segment 统计：count/min/mean/max length。"""
    values = np.asarray(accepted, dtype=bool)
    if values.size == 0 or not np.any(values):
        return 0, 0, 0.0, 0
    padded = np.r_[False, values, False]
    boundaries = np.flatnonzero(padded[1:] != padded[:-1])
    lengths = boundaries[1::2] - boundaries[0::2]
    return (
        int(lengths.size),
        int(lengths.min()),
        float(lengths.mean()),
        int(lengths.max()),
    )


def segment_offsets_from_rows(
    source_tick, next_state_tick, extra_split_before
) -> list[int]:
    """由 accepted rows 计算 metadata.segment_offsets。

    相邻 row 只有同时满足 ``source_tick[i] == next_state_tick[i-1]``（tick 连续）
    且 ``extra_split_before[i]`` 为 False 才允许同 segment；否则在 row i 前切分。
    extra_split_before 覆盖 RAW 可观察的 must-split 事件（estop、tracking /
    telemetry validity loss）。返回 [0, ..., M]：first==0、last==M、strictly
    increasing；空 trajectory 返回 []。
    """
    source = np.asarray(source_tick, dtype=np.int64)
    nxt = np.asarray(next_state_tick, dtype=np.int64)
    split = np.asarray(extra_split_before, dtype=bool)
    m = int(source.size)
    if nxt.size != m or split.size != m:
        raise ValueError("segment 输入数组长度不一致")
    if m == 0:
        return []
    offsets = [0]
    for index in range(1, m):
        if source[index] != nxt[index - 1] or bool(split[index]):
            offsets.append(index)
    offsets.append(m)
    return offsets
