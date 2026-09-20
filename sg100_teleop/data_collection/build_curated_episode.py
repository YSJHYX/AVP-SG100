"""RAW → Curated 清洗层：causal transition build + segment + cleaning_report。

输入是 `recording/vla_raw/episode_XXXXXX.npz`（及其 `_media` 目录），输出
`recording/vla_curated/episode_XXXXXX/`：

```text
episode_XXXXXX/
  metadata.json        # 训练契约元数据（含 17D ordering、AHFE slice、segment_offsets）
  trajectory.npz       # 仅 training-valid transition 的 30 Hz 核心数组
  media/<role>/rgb/    # accepted frame 的去重复制（RAW frame → curated index 映射）
  cleaning_report.json # 拒绝统计 / segment / readiness blockers
```

Curated = valid only。transition 接受条件（全部满足）：

- state/action/next-state 三段 17D valid（有效位 + finite + 无 desired fallback）；
- 4 个 native source timestamp 满足 ``0 < ts <= dataset tick``（RAW sampler 因果）；
- **component-wise causality**（v1 freeze 核心）：
  ``arm_qpos_ts[k] <= arm_qcmd_ts[k] < arm_qpos_ts[k+1]`` 且
  ``hand_qpos_ts[k] <= hand_qcmd_ts[k] < hand_qpos_ts[k+1]``——next 侧 timestamp
  通过 ``next_state_tick_index`` 从 RAW 读取，验证后不重复保存；
- unified causality：``state_ts <= action_ts < next_state_ts``（由 component-wise
  推导，仍显式检查）；
- state/action RGB valid 且 media 存在。

unified trajectory timestamp 定义：

```text
state_timestamp_ns      = max(arm_qpos_source_ts, hand_qpos_source_ts)
action_timestamp_ns     = max(arm_qcmd_source_ts, hand_qcmd_source_ts)
next_state_timestamp_ns = max(next arm qpos ts, next hand qpos ts)
```

segment 规则：tick 不连续或 RAW 可观察 must-split 事件（estop、tracking /
telemetry validity loss，观测自 transition 两个 tick）即切分；action horizon
禁止跨 segment。本模块是纯数据处理，不连接任何硬件。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import shutil
import uuid

import numpy as np

from sg100_teleop.data_collection.vla_curated_schema import (
    ACTION_REPRESENTATION,
    ACTION_UNIT,
    ARM_SLICE,
    CURATED_DATASET_STATUS_NOT_READY,
    CURATED_DATASET_STATUS_READY,
    CURATED_SCHEMA_NAME,
    CURATED_SCHEMA_VERSION,
    DEFAULT_CURATED_DIR,
    HAND_SLICE,
    INVALID_REASON_BITS,
    STATE_REPRESENTATION,
    STATE_UNIT,
    invalid_reason_counts,
    segment_offsets_from_rows,
)
from sg100_teleop.data_collection.vla_curated_validator import (
    CuratedValidationReport,
    validate_curated_episode,
)


@dataclass(frozen=True)
class CuratedBuildResult:
    raw_path: Path
    episode_dir: Path | None
    written: bool
    accepted_count: int
    rejected_count: int
    report: dict
    validation: CuratedValidationReport | None


def _scalar(arrays: dict, key: str, default=None):
    if key not in arrays:
        return default
    value = arrays[key]
    return value.item() if hasattr(value, "item") else value


def _tick_ranges(mask) -> list[list[int]]:
    """把 bool mask 压成 [first, last]（含端点）的连续区间列表。"""
    values = np.asarray(mask, dtype=bool)
    ranges: list[list[int]] = []
    if values.size == 0 or not np.any(values):
        return ranges
    padded = np.r_[False, values, False]
    boundaries = np.flatnonzero(padded[1:] != padded[:-1])
    return [
        [int(start), int(stop - 1)]
        for start, stop in zip(boundaries[0::2], boundaries[1::2])
    ]


def _reason_mask(
    arrays: dict[str, np.ndarray],
    *,
    wrist_enabled: bool,
    media_root: Path,
) -> np.ndarray:
    n = int(arrays["timestamp_ns"].size)
    if n < 2:
        return np.zeros(0, dtype=np.int64)
    reason = np.zeros(n - 1, dtype=np.int64)
    tick = np.arange(n - 1)
    timestamp = arrays["timestamp_ns"]
    qpos_valid = arrays["robot_qpos_17d_valid"]
    qcmd_valid = arrays["robot_qcmd_17d_valid"]

    reason[~qpos_valid[tick]] |= INVALID_REASON_BITS["STATE_QPOS_INVALID"]
    reason[~qcmd_valid[tick]] |= INVALID_REASON_BITS["ACTION_QCMD_INVALID"]
    reason[~qpos_valid[tick + 1]] |= INVALID_REASON_BITS[
        "NEXT_STATE_QPOS_INVALID"
    ]
    reason[timestamp[tick + 1] <= timestamp[tick]] |= INVALID_REASON_BITS[
        "TICK_TIMESTAMP_ORDER_INVALID"
    ]
    sources = (
        ("arm_qpos_source_timestamp_ns", "ARM_QPOS_SOURCE_TIMESTAMP_INVALID"),
        ("hand_feedback_source_timestamp_ns",
         "HAND_QPOS_SOURCE_TIMESTAMP_INVALID"),
        ("arm_qcmd_source_timestamp_ns", "ARM_QCMD_SOURCE_TIMESTAMP_INVALID"),
        ("hand_qcmd_source_timestamp_ns", "HAND_QCMD_SOURCE_TIMESTAMP_INVALID"),
    )
    for array_key, reason_key in sources:
        source = arrays[array_key]
        invalid = ~((source[tick] > 0) & (source[tick] <= timestamp[tick]))
        reason[invalid] |= INVALID_REASON_BITS[reason_key]

    # component-wise causality：state 组件 ts <= action 组件 ts < next state 组件 ts。
    # next 侧 timestamp 从 RAW 的 k+1 行读取（不保存到 Curated）。
    arm_state_ts = arrays["arm_qpos_source_timestamp_ns"]
    hand_state_ts = arrays["hand_feedback_source_timestamp_ns"]
    arm_action_ts = arrays["arm_qcmd_source_timestamp_ns"]
    hand_action_ts = arrays["hand_qcmd_source_timestamp_ns"]
    involved = (
        (arm_state_ts[tick] > 0)
        & (arm_action_ts[tick] > 0)
        & (arm_state_ts[tick + 1] > 0)
    )
    arm_causal = (
        (arm_state_ts[tick] <= arm_action_ts[tick])
        & (arm_action_ts[tick] < arm_state_ts[tick + 1])
    )
    reason[involved & ~arm_causal] |= INVALID_REASON_BITS[
        "ARM_CAUSALITY_INVALID"
    ]
    involved_hand = (
        (hand_state_ts[tick] > 0)
        & (hand_action_ts[tick] > 0)
        & (hand_state_ts[tick + 1] > 0)
    )
    hand_causal = (
        (hand_state_ts[tick] <= hand_action_ts[tick])
        & (hand_action_ts[tick] < hand_state_ts[tick + 1])
    )
    reason[involved_hand & ~hand_causal] |= INVALID_REASON_BITS[
        "HAND_CAUSALITY_INVALID"
    ]
    state_ts = np.maximum(arm_state_ts[tick], hand_state_ts[tick])
    action_ts = np.maximum(arm_action_ts[tick], hand_action_ts[tick])
    next_state_ts = np.maximum(
        arm_state_ts[tick + 1], hand_state_ts[tick + 1]
    )
    unified_involved = involved & involved_hand
    unified_causal = (state_ts <= action_ts) & (action_ts < next_state_ts)
    reason[unified_involved & ~unified_causal] |= INVALID_REASON_BITS[
        "UNIFIED_CAUSALITY_INVALID"
    ]

    head_valid = arrays["head_camera_valid"]
    head_index = arrays["head_rgb_frame_index"]
    reason[~head_valid[tick] | (head_index[tick] < 0)] |= INVALID_REASON_BITS[
        "HEAD_RGB_INVALID"
    ]
    if wrist_enabled:
        wrist_valid = arrays["wrist_camera_valid"]
        wrist_index = arrays["wrist_rgb_frame_index"]
        reason[
            ~wrist_valid[tick] | (wrist_index[tick] < 0)
        ] |= INVALID_REASON_BITS["WRIST_RGB_INVALID"]

    candidate = reason == 0
    for index in np.flatnonzero(candidate):
        source_index = int(head_index[index])
        if not (media_root / "head" / "rgb" / f"{source_index:06d}.jpg").is_file():
            reason[index] |= INVALID_REASON_BITS["HEAD_RGB_MEDIA_MISSING"]
    candidate = reason == 0
    if wrist_enabled:
        for index in np.flatnonzero(candidate):
            source_index = int(wrist_index[index])
            path = media_root / "right_wrist" / "rgb" / f"{source_index:06d}.jpg"
            if not path.is_file():
                reason[index] |= INVALID_REASON_BITS[
                    "WRIST_RGB_MEDIA_MISSING"
                ]
    return reason


def _split_before_events(arrays: dict[str, np.ndarray], accepted_tick: np.ndarray) -> np.ndarray:
    """RAW 可观察 must-split 事件：estop / tracking loss / telemetry loss。

    transition 覆盖 tick k 与 k+1；任一 tick 出现 adverse 事件则在 row 前
    切 segment（E-stop 结束当前 segment，不删除已有效数据）。C re-anchor
    generation 未在 RAW 记录 —— 该 limitation 见 README；正式 SOP 禁止在
    episode 中按 C。
    """
    adverse = (
        np.asarray(arrays["estop_latched"], dtype=bool)
        | ~np.asarray(arrays["tracking_valid"], dtype=bool)
        | ~np.asarray(arrays["telemetry_valid"], dtype=bool)
    )
    split = np.zeros(accepted_tick.size, dtype=bool)
    if accepted_tick.size:
        split = adverse[accepted_tick] | adverse[accepted_tick + 1]
    return split


def build_curated_episode(
    raw_episode: str | Path,
    output_dir: str | Path = DEFAULT_CURATED_DIR,
    *,
    include_desired: bool = False,
    allow_empty: bool = False,
) -> CuratedBuildResult:
    raw_path = Path(raw_episode)
    if not raw_path.is_file():
        raise FileNotFoundError(raw_path)
    media_root = raw_path.with_name(f"{raw_path.stem}_media")
    with np.load(raw_path, allow_pickle=False) as archive:
        arrays = {key: archive[key] for key in archive.files}

    timestamp = np.asarray(arrays["timestamp_ns"], dtype=np.int64)
    n = int(timestamp.size)
    required_cameras = tuple(
        str(role) for role in np.asarray(arrays["required_cameras"]).tolist()
    )
    wrist_enabled = "right_wrist" in required_cameras
    reason = _reason_mask(
        arrays, wrist_enabled=wrist_enabled, media_root=media_root
    )
    accepted_tick = np.flatnonzero(reason == 0)
    m = int(accepted_tick.size)

    episode_id = raw_path.stem
    raw_training_ready = bool(_scalar(arrays, "training_ready", False))
    raw_hardware_execution = bool(
        _scalar(arrays, "hardware_execution", False)
    )
    feedback_status = str(
        _scalar(arrays, "sg100_feedback_map_status", "UNKNOWN")
    )
    blockers: list[str] = []
    if not raw_training_ready:
        blockers.append("SOURCE_TRAINING_READY_FALSE")
    if not raw_hardware_execution:
        blockers.append("SOURCE_HARDWARE_EXECUTION_FALSE")
    if feedback_status == "F_FB_PENDING":
        blockers.append("SG100_F_FB_PENDING")
    if m == 0:
        blockers.append("NO_ACCEPTED_TRANSITIONS")

    split_before = _split_before_events(arrays, accepted_tick)
    next_tick = accepted_tick + 1
    offsets = segment_offsets_from_rows(accepted_tick, next_tick, split_before)

    arm_state_ts = np.asarray(
        arrays["arm_qpos_source_timestamp_ns"], dtype=np.int64
    )
    hand_state_ts = np.asarray(
        arrays["hand_feedback_source_timestamp_ns"], dtype=np.int64
    )
    arm_action_ts = np.asarray(
        arrays["arm_qcmd_source_timestamp_ns"], dtype=np.int64
    )
    hand_action_ts = np.asarray(
        arrays["hand_qcmd_source_timestamp_ns"], dtype=np.int64
    )
    report: dict = {
        "schema_name": CURATED_SCHEMA_NAME,
        "schema_version": CURATED_SCHEMA_VERSION,
        "source_episode_path": str(raw_path),
        "episode_id": episode_id,
        "raw_tick_count": n,
        "candidate_transition_count": max(0, n - 1),
        "accepted_transition_count": m,
        "rejected_transition_count": max(0, n - 1) - m,
        "invalid_reason_counts": invalid_reason_counts(reason),
        "rejected_tick_ranges": _tick_ranges(reason != 0),
        "segment_count": max(0, len(offsets) - 1),
        "segment_offsets": offsets,
        "segment_event_split_count": int(np.count_nonzero(split_before)),
        "time_range_start_ns": (
            int(timestamp[accepted_tick[0]]) if m else None
        ),
        "time_range_end_ns": (
            int(timestamp[accepted_tick[-1] + 1]) if m else None
        ),
        "include_desired": bool(include_desired and "robot_desired_qcmd_17d_rad" in arrays),
        "desired17_absent_in_source": "robot_desired_qcmd_17d_rad" not in arrays,
        "training_blockers": blockers,
        "quality_summary": {
            "accepted_ratio": (m / (n - 1)) if n > 1 else 0.0,
            "source_training_ready": raw_training_ready,
            "source_hardware_execution": raw_hardware_execution,
            "source_sg100_feedback_map_status": feedback_status,
            "source_validation_quality": str(
                _scalar(arrays, "validation_quality", "UNKNOWN")
            ),
        },
    }

    if m == 0 and not allow_empty:
        return CuratedBuildResult(
            raw_path=raw_path,
            episode_dir=None,
            written=False,
            accepted_count=0,
            rejected_count=max(0, n - 1),
            report=report,
            validation=None,
        )

    trajectory: dict[str, np.ndarray] = {
        "source_tick_index": accepted_tick.astype(np.int64),
        "next_state_tick_index": next_tick.astype(np.int64),
        "state_timestamp_ns": np.maximum(
            arm_state_ts[accepted_tick], hand_state_ts[accepted_tick]
        ),
        "action_timestamp_ns": np.maximum(
            arm_action_ts[accepted_tick], hand_action_ts[accepted_tick]
        ),
        "next_state_timestamp_ns": np.maximum(
            arm_state_ts[next_tick], hand_state_ts[next_tick]
        ),
        "arm_qpos_source_timestamp_ns": arm_state_ts[accepted_tick],
        "hand_qpos_source_timestamp_ns": hand_state_ts[accepted_tick],
        "arm_qcmd_source_timestamp_ns": arm_action_ts[accepted_tick],
        "hand_qcmd_source_timestamp_ns": hand_action_ts[accepted_tick],
        "robot_qpos_17d_rad": np.asarray(
            arrays["robot_qpos_17d_rad"], dtype=np.float64
        )[accepted_tick],
        "robot_qcmd_17d_rad": np.asarray(
            arrays["robot_qcmd_17d_rad"], dtype=np.float64
        )[accepted_tick],
        "head_rgb_frame_index": np.zeros(m, dtype=np.int64),
        "right_wrist_rgb_frame_index": (
            np.zeros(m, dtype=np.int64)
            if wrist_enabled
            else np.full(m, -1, dtype=np.int64)
        ),
    }
    if include_desired and "robot_desired_qcmd_17d_rad" in arrays:
        trajectory["robot_desired_qcmd_17d_rad"] = np.asarray(
            arrays["robot_desired_qcmd_17d_rad"], dtype=np.float64
        )[accepted_tick]

    training_ready = bool(
        raw_training_ready and raw_hardware_execution and m > 0
    )
    metadata: dict = {
        "schema_name": CURATED_SCHEMA_NAME,
        "schema_version": CURATED_SCHEMA_VERSION,
        "episode_id": episode_id,
        "language_instruction": str(
            _scalar(arrays, "language_instruction", "")
        ),
        "dataset_hz": float(_scalar(arrays, "dataset_hz", 30.0)),
        "joint_names": [
            str(name)
            for name in np.asarray(arrays["robot_joint_names"]).tolist()
        ],
        "arm_slice": [ARM_SLICE[0], ARM_SLICE[1]],
        "hand_slice": [HAND_SLICE[0], HAND_SLICE[1]],
        "camera_roles": (
            ["head", "right_wrist"] if wrist_enabled else ["head"]
        ),
        "source_schema_name": str(_scalar(arrays, "schema_name", "")),
        "source_schema_version": int(_scalar(arrays, "schema_version", 0)),
        "source_episode_path": str(raw_path),
        "hardware_execution": raw_hardware_execution,
        "training_ready": training_ready,
        "dataset_status": (
            CURATED_DATASET_STATUS_READY
            if training_ready else CURATED_DATASET_STATUS_NOT_READY
        ),
        "action_representation": ACTION_REPRESENTATION,
        "action_unit": ACTION_UNIT,
        "state_representation": STATE_REPRESENTATION,
        "state_unit": STATE_UNIT,
        "segment_offsets": offsets,
    }

    output_root = Path(output_dir)
    final_dir = output_root / episode_id
    if final_dir.exists():
        raise FileExistsError(f"拒绝覆盖已有 curated episode: {final_dir}")
    output_root.mkdir(parents=True, exist_ok=True)
    staging = output_root / f".pending_curated_{uuid.uuid4().hex}"
    try:
        staging.mkdir(parents=True)
        # RGB media 去重：RAW frame identity → curated media index 确定性映射；
        # 多个 transition 引用同一 RAW frame 时只保存一份、index 允许重复。
        head_index = np.asarray(arrays["head_rgb_frame_index"], dtype=np.int64)
        head_map: dict[int, int] = {}
        for curated_index, raw_tick in enumerate(accepted_tick):
            source_index = int(head_index[raw_tick])
            if source_index not in head_map:
                head_map[source_index] = len(head_map)
                src = media_root / "head" / "rgb" / f"{source_index:06d}.jpg"
                dst_dir = staging / "media" / "head" / "rgb"
                dst_dir.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dst_dir / f"{head_map[source_index]:06d}.jpg")
            trajectory["head_rgb_frame_index"][curated_index] = head_map[
                source_index
            ]
        if wrist_enabled:
            wrist_index = np.asarray(
                arrays["wrist_rgb_frame_index"], dtype=np.int64
            )
            wrist_map: dict[int, int] = {}
            for curated_index, raw_tick in enumerate(accepted_tick):
                source_index = int(wrist_index[raw_tick])
                if source_index not in wrist_map:
                    wrist_map[source_index] = len(wrist_map)
                    src = (
                        media_root / "right_wrist" / "rgb"
                        / f"{source_index:06d}.jpg"
                    )
                    dst_dir = staging / "media" / "right_wrist" / "rgb"
                    dst_dir.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(
                        src, dst_dir / f"{wrist_map[source_index]:06d}.jpg"
                    )
                trajectory["right_wrist_rgb_frame_index"][curated_index] = (
                    wrist_map[source_index]
                )

        validation = validate_curated_episode(
            metadata,
            trajectory,
            media_root=staging / "media",
            allow_empty=allow_empty,
        )
        if validation.quality == "FAIL":
            raise ValueError(
                "curated episode 校验失败: " + "; ".join(validation.errors)
            )
        with (staging / "metadata.json").open("w", encoding="utf-8") as handle:
            json.dump(metadata, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
        with (staging / "cleaning_report.json").open(
            "w", encoding="utf-8"
        ) as handle:
            json.dump(report, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
        with (staging / "trajectory.npz").open("wb") as handle:
            np.savez_compressed(handle, **trajectory)
        staging.rename(final_dir)
    except Exception:
        if staging.exists():
            shutil.rmtree(staging, ignore_errors=True)
        raise

    return CuratedBuildResult(
        raw_path=raw_path,
        episode_dir=final_dir,
        written=True,
        accepted_count=m,
        rejected_count=max(0, n - 1) - m,
        report=report,
        validation=validation,
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build curated 30 Hz causal-transition episode from raw VLA NPZ"
    )
    parser.add_argument("raw_episode", help="recording/vla_raw/episode_XXXXXX.npz")
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_CURATED_DIR),
        help=f"curated 输出根目录（默认 {DEFAULT_CURATED_DIR}）",
    )
    parser.add_argument(
        "--include-desired",
        action="store_true",
        help="附带可选 provenance 列 robot_desired_qcmd_17d_rad",
    )
    parser.add_argument(
        "--allow-empty",
        action="store_true",
        help="0 accepted transition 时仍写出空 curated episode（默认拒绝）",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    result = build_curated_episode(
        args.raw_episode,
        args.output_dir,
        include_desired=args.include_desired,
        allow_empty=args.allow_empty,
    )
    print(
        f"raw ticks: {result.report['raw_tick_count']} "
        f"accepted: {result.accepted_count} "
        f"rejected: {result.rejected_count}"
    )
    print(
        "invalid reasons: "
        + (
            json.dumps(result.report["invalid_reason_counts"], sort_keys=True)
            or "{}"
        )
    )
    print(
        f"segments: {result.report['segment_count']} "
        f"offsets: {result.report['segment_offsets']}"
    )
    print(f"training blockers: {result.report['training_blockers']}")
    if result.written:
        assert result.episode_dir is not None
        print(f"CURATED: {result.episode_dir}")
        print(f"validation: {result.validation.quality}")
    else:
        print("NOT WRITTEN: no training-valid transition; see report above")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
