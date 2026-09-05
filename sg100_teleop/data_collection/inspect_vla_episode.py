"""只读检查 raw VLA NPZ 与对应 media，不修改 episode。"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from sg100_teleop.data_collection.vla_raw_validator import validate_episode


def _ratio(archive, key: str) -> float:
    if key not in archive.files:
        return float("nan")
    values = np.asarray(archive[key], dtype=bool)
    return float(values.mean()) if values.size else 0.0


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
        return {
            "path": str(npz_path),
            "schema": str(archive["schema_name"].item()),
            "schema_version": int(archive["schema_version"].item()),
            "frames": n,
            "duration_s": duration_s,
            "dataset_hz_effective": report.metrics["effective_hz"],
            "camera_modalities": modalities,
            "head_camera_present": bool(archive["head_camera_present"].item()),
            "wrist_camera_present": bool(archive["wrist_camera_present"].item()),
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


def _format(summary: dict[str, object]) -> str:
    lines = [f"{key}: {value}" for key, value in summary.items()]
    return "\n".join(lines)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read-only raw VLA episode inspector")
    parser.add_argument("episode", help="episode_XXXXXX.npz")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    print(_format(inspect_vla_episode(args.episode)))


if __name__ == "__main__":
    main()
