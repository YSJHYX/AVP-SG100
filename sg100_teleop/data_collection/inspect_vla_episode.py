"""只读检查 RAW / Curated episode，并按需派生 timing 指标；不修改数据。

派生指标（skew / dt / native Hz / desired-effective 差异）全部在本工具
运行时由永久保留的 timestamp/sequence 计算，dataset 本身不保存这些数组。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from sg100_teleop.data_collection.vla_curated_schema import (
    ARM_SLICE,
    HAND_SLICE,
)
from sg100_teleop.data_collection.vla_timing_metrics import (
    action_next_state_dt_ns,
    arm_hand_action_skew_ns,
    arm_hand_state_skew_ns,
    desired_effective_summary,
    sequence_rate_hz,
    state_action_dt_ns,
    summary_stats_ns,
)
from sg100_teleop.data_collection.vla_raw_validator import validate_episode


def _ratio(archive, key: str) -> float:
    if key not in archive.files:
        return float("nan")
    values = np.asarray(archive[key], dtype=bool)
    return float(values.mean()) if values.size else 0.0


def _native_hz(archive) -> dict[str, object]:
    pairs = (
        ("arm_command", "arm_qcmd_sequence", "arm_qcmd_source_timestamp_ns"),
        ("arm_feedback", "arm_feedback_sequence", "arm_qpos_source_timestamp_ns"),
        ("hand_command", "hand_command_sequence", "hand_qcmd_source_timestamp_ns"),
        ("hand_feedback", "hand_feedback_sequence",
         "hand_feedback_source_timestamp_ns"),
    )
    result: dict[str, object] = {}
    observed: list[str] = []
    unobservable: list[str] = []
    for name, sequence_key, timestamp_key in pairs:
        if sequence_key in archive.files and timestamp_key in archive.files:
            hz = sequence_rate_hz(
                archive[sequence_key], archive[timestamp_key]
            )
            result[f"{name}_native_hz"] = hz
            if np.isfinite(hz):
                observed.append(name)
            else:
                unobservable.append(name)
    result["native_rate_evidence"] = (
        "sequence-delta estimate from sampled source timestamps ("
        + ",".join(observed) + ")"
        if observed else ""
    )
    if unobservable:
        result["native_rate_not_exactly_observable"] = (
            "NOT EXACTLY OBSERVABLE (insufficient sequence/timestamp "
            "evidence): " + ",".join(unobservable)
        )
    return result


def _desired_effective(archive_like, desired_key: str = "robot_desired_qcmd_17d_rad",
                       hand_domain: str | None = None) -> dict | None:
    files = (
        set(archive_like.files)
        if hasattr(archive_like, "files")
        else set(archive_like)
    )
    if not {"robot_qcmd_17d_rad", desired_key}.issubset(files):
        return None
    effective = np.asarray(archive_like["robot_qcmd_17d_rad"], dtype=np.float64).copy()
    if hand_domain is None:
        hand_domain = (
            str(np.asarray(archive_like["robot_qcmd_hand_domain"]).item())
            if "robot_qcmd_hand_domain" in files else "canonical_rad"
        )
    if hand_domain == "sdk_user_rad":
        from sg100_teleop.robot.sg100_driver import SG100HardwareMap

        hardware_map = SG100HardwareMap()
        effective[:, 6:] = (effective[:, 6:] - hardware_map.offset_rad) / hardware_map.gain
    return desired_effective_summary(archive_like[desired_key], effective)


def inspect_vla_episode(path: str | Path) -> dict[str, object]:
    npz_path = Path(path)
    if not npz_path.is_file():
        raise FileNotFoundError(npz_path)
    media_root = npz_path.with_name(f"{npz_path.stem}_media")
    with np.load(npz_path, allow_pickle=False) as archive:
        timestamp = np.asarray(archive["timestamp_ns"], dtype=np.int64)
        n = int(timestamp.size)
        duration_s = (
            float((timestamp[-1] - timestamp[0]) * 1e-9) if n > 1 else 0.0
        )
        report = validate_episode(
            archive,
            media_root=media_root,
            writer_drop_count=int(archive["media_writer_drop_count"]),
            writer_error_count=int(archive["media_writer_error_count"]),
            telemetry_sequence_gap_count=int(
                archive["telemetry_sequence_gap_count"]
            ),
        )
        modalities = (
            str(archive["camera_modalities"].item())
            if "camera_modalities" in archive.files else "rgb,depth (v1 legacy)"
        )
        summary: dict[str, object] = {
            "path": str(npz_path),
            "layer": "RAW",
            "schema": str(archive["schema_name"].item()),
            "schema_version": int(archive["schema_version"].item()),
            "frames": n,
            "duration_s": duration_s,
            "dataset_hz_effective": report.metrics["effective_hz"],
            "camera_modalities": modalities,
            "head_camera_present": bool(
                archive["head_camera_present"].item()
            ),
            "wrist_camera_present": bool(
                archive["wrist_camera_present"].item()
            ),
            "robot_telemetry_present": bool(
                archive["robot_telemetry_present"].item()
            ),
            "telemetry_effective_hz": report.metrics["telemetry_effective_hz"],
            "telemetry_age_mean_ms": report.metrics["telemetry_age_mean_ms"],
            "desired17_valid_ratio": _ratio(
                archive, "robot_desired_qcmd_17d_valid"
            ),
            "effective17_valid_ratio": _ratio(
                archive, "robot_qcmd_17d_valid"
            ),
            "qpos17_valid_ratio": _ratio(archive, "robot_qpos_17d_valid"),
            "training_ready": bool(archive["training_ready"].item()),
            "dataset_status": str(archive["dataset_status"].item()),
            "validation_quality": report.quality,
            "validation_errors": report.errors,
            "validation_warnings": report.warnings,
        }
        # 以下全部是运行时派生指标，dataset 中不保存。
        summary.update(_native_hz(archive))
        summary["arm_hand_state_skew"] = summary_stats_ns(
            arm_hand_state_skew_ns(
                archive["arm_qpos_source_timestamp_ns"],
                archive["hand_feedback_source_timestamp_ns"],
            )
        )
        summary["arm_hand_action_skew"] = summary_stats_ns(
            arm_hand_action_skew_ns(
                archive["arm_qcmd_source_timestamp_ns"],
                archive["hand_qcmd_source_timestamp_ns"],
            )
        )
        derived = _desired_effective(archive)
        if derived is not None:
            summary["desired_effective"] = derived
        return summary


def inspect_curated_episode(path: str | Path) -> dict[str, object]:
    import json

    episode_path = Path(path)
    if episode_path.is_dir():
        trajectory_path = episode_path / "trajectory.npz"
        metadata_path = episode_path / "metadata.json"
        report_path = episode_path / "cleaning_report.json"
        media_root = episode_path / "media"
    else:
        trajectory_path = episode_path
        metadata_path = episode_path.with_name("metadata.json")
        report_path = episode_path.with_name("cleaning_report.json")
        media_root = episode_path.with_name("media")
    if not trajectory_path.is_file():
        raise FileNotFoundError(trajectory_path)

    metadata: dict = {}
    if metadata_path.is_file():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    cleaning: dict = {}
    if report_path.is_file():
        cleaning = json.loads(report_path.read_text(encoding="utf-8"))

    with np.load(trajectory_path, allow_pickle=False) as trajectory:
        state_ts = np.asarray(
            trajectory["state_timestamp_ns"], dtype=np.int64
        )
        action_ts = np.asarray(
            trajectory["action_timestamp_ns"], dtype=np.int64
        )
        next_ts = np.asarray(
            trajectory["next_state_timestamp_ns"], dtype=np.int64
        )
        m = int(state_ts.size)
        duration_s = (
            float((next_ts[-1] - state_ts[0]) * 1e-9) if m else 0.0
        )
        offsets = list(metadata.get("segment_offsets", ()))
        summary: dict[str, object] = {
            "path": str(trajectory_path),
            "layer": "CURATED",
            "schema": metadata.get("schema_name", "unknown"),
            "schema_version": metadata.get("schema_version"),
            "episode_id": metadata.get("episode_id"),
            "transitions": m,
            "segment_count": max(0, len(offsets) - 1) if offsets else 0,
            "segment_offsets": offsets,
            "duration_s": duration_s,
            "transition_hz_observed": (
                float(m / duration_s) if duration_s > 0 else 0.0
            ),
            "native_rate_note": (
                "NATIVE EVENT RATE: NOT EXACTLY RECOVERABLE FROM CURATED "
                "DATA（30 Hz causal sampling 丢中间 native event；native "
                "rate 只能从 RAW sequence/timestamp 证据估计）"
            ),
            "source_episode_path": metadata.get("source_episode_path"),
            "training_ready": metadata.get("training_ready"),
            "dataset_status": metadata.get("dataset_status"),
            "action_representation": metadata.get("action_representation"),
            "language_instruction": metadata.get("language_instruction"),
            "cleaning_accepted": cleaning.get("accepted_transition_count"),
            "cleaning_rejected": cleaning.get("rejected_transition_count"),
            "cleaning_invalid_reason_counts": cleaning.get(
                "invalid_reason_counts", {}
            ),
            "training_blockers": cleaning.get("training_blockers", []),
        }
        # 全部为运行时派生指标；curated trajectory 不保存任何派生数组。
        summary["arm_hand_state_skew"] = summary_stats_ns(
            arm_hand_state_skew_ns(
                trajectory["arm_qpos_source_timestamp_ns"],
                trajectory["hand_qpos_source_timestamp_ns"],
            )
        )
        summary["arm_hand_action_skew"] = summary_stats_ns(
            arm_hand_action_skew_ns(
                trajectory["arm_qcmd_source_timestamp_ns"],
                trajectory["hand_qcmd_source_timestamp_ns"],
            )
        )
        summary["state_action_dt"] = summary_stats_ns(
            state_action_dt_ns(state_ts, action_ts)
        )
        summary["action_next_state_dt"] = summary_stats_ns(
            action_next_state_dt_ns(action_ts, next_ts)
        )
        derived = _desired_effective(
            trajectory, hand_domain=metadata.get("action_hand_domain", "canonical_rad")
        )
        if derived is not None:
            summary["desired_effective"] = derived
        summary["replay_slices"] = {
            "arm_qcmd_rad": [ARM_SLICE[0], ARM_SLICE[1]],
            "hand_qcmd_rad": [HAND_SLICE[0], HAND_SLICE[1]],
            "arm_hand_qcmd_rad": [0, 17],
        }
        if media_root.is_dir():
            camera_roles = metadata.get("camera_roles", ["head"])
            media_counts = {
                role: len(tuple((media_root / role / "rgb").glob("*.jpg")))
                for role in camera_roles
            }
            summary["media_rgb_counts"] = media_counts
        return summary


def inspect_auto(path: str | Path) -> dict[str, object]:
    """目录 → curated episode；npz → 按 schema 自动识别 RAW / curated。"""
    target = Path(path)
    if target.is_dir():
        return inspect_curated_episode(target)
    with np.load(target, allow_pickle=False) as archive:
        is_curated = "source_tick_index" in archive.files
    return (
        inspect_curated_episode(target) if is_curated
        else inspect_vla_episode(target)
    )


def _format(summary: dict[str, object]) -> str:
    lines = [f"{key}: {value}" for key, value in summary.items()]
    return "\n".join(lines)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only RAW/Curated VLA episode inspector"
    )
    parser.add_argument(
        "episode",
        help="RAW episode_XXXXXX.npz、curated episode 目录或 trajectory.npz",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    print(_format(inspect_auto(args.episode)))


if __name__ == "__main__":
    main()
