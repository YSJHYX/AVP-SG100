"""只读 replay 命令导出器：从 17D 唯一存储切片，绝不产生重复数据集文件。

Curated/RAW 只保存 `robot_qcmd_17d_rad`（effective）与可选
`robot_desired_qcmd_17d_rad`（desired provenance）。Arm / Hand / Arm+Hand
replay 轨迹全部由 slice 现场导出，effective 与 desired 必须在文件名与
sidecar metadata 中显式区分，防止软嵌把 desired 误认为 hardware executed
command。

用法示例：

```bash
.venv/bin/python -m sg100_teleop.data_collection.export_replay_commands \
  recording/vla_curated/episode_000001/trajectory.npz \
  --target hand \
  --output hand_qcmd_canonical_rad.npy
```
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from sg100_teleop.data_collection.vla_curated_schema import (
    ARM_SLICE,
    HAND_SLICE,
)


TARGET_CHOICES = ("arm", "hand", "arm_hand")
SOURCE_CHOICES = ("effective", "desired")
EFFECTIVE_COMMAND_SEMANTICS = "FINAL_EFFECTIVE_BACKEND_COMMAND"
DESIRED_COMMAND_SEMANTICS = "DESIRED_INTENT_NOT_HARDWARE_EXECUTED"


def detect_episode_kind(files) -> str:
    """区分 RAW episode npz 与 curated trajectory npz。"""
    names = set(files)
    if "source_tick_index" in names:
        return "curated"
    if "schema_name" in names and "timestamp_ns" in names:
        return "raw"
    raise ValueError("无法识别输入 npz：既不是 RAW episode 也不是 curated trajectory")


def target_slice(target: str) -> slice:
    if target == "arm":
        return slice(ARM_SLICE[0], ARM_SLICE[1])
    if target == "hand":
        return slice(HAND_SLICE[0], HAND_SLICE[1])
    return slice(0, ARM_SLICE[1] + (HAND_SLICE[1] - HAND_SLICE[0]))


def load_replay_commands(
    episode_npz: str | Path,
    *,
    target: str = "hand",
    source: str = "effective",
    drop_nan: bool = False,
) -> tuple[np.ndarray, dict]:
    """返回 (命令数组 (K, width) rad, provenance dict)；不写任何文件。"""
    if target not in TARGET_CHOICES:
        raise ValueError(f"target 必须是 {TARGET_CHOICES}")
    if source not in SOURCE_CHOICES:
        raise ValueError(f"source 必须是 {SOURCE_CHOICES}")
    path = Path(episode_npz)
    if not path.is_file():
        raise FileNotFoundError(path)
    with np.load(path, allow_pickle=False) as archive:
        kind = detect_episode_kind(archive.files)
        key = (
            "robot_qcmd_17d_rad"
            if source == "effective"
            else "robot_desired_qcmd_17d_rad"
        )
        if key not in archive.files:
            raise ValueError(
                f"{path} 不含 {key}"
                + ("（desired 需 RAW v3+ 或 curated --include-desired）"
                   if source == "desired" else "")
            )
        commands = np.asarray(archive[key], dtype=np.float64).copy()
        hand_domain = (
            str(archive["robot_qcmd_hand_domain"].item())
            if "robot_qcmd_hand_domain" in archive.files else "canonical_rad"
        )
    if kind == "curated":
        metadata_path = path.with_name("metadata.json")
        if metadata_path.exists():
            hand_domain = json.loads(metadata_path.read_text()).get(
                "action_hand_domain", "canonical_rad"
            )
    if source == "effective" and hand_domain == "sdk_user_rad":
        # Replay still feeds the canonical backend; undo only the data-domain map.
        from sg100_teleop.robot.sg100_driver import SG100HardwareMap

        hardware_map = SG100HardwareMap()
        commands[:, 6:] = (
            commands[:, 6:] - hardware_map.offset_rad
        ) / hardware_map.gain

    selected = np.ascontiguousarray(commands[:, target_slice(target)])
    total = int(selected.shape[0])
    finite_rows = np.all(np.isfinite(selected), axis=1)
    dropped = int(np.count_nonzero(~finite_rows))
    if dropped:
        if not drop_nan:
            raise ValueError(
                f"含 {dropped} 行 NaN（desired 未生效或 telemetry invalid）；"
                "显式 --nan drop 才会丢弃这些行"
            )
        selected = np.ascontiguousarray(selected[finite_rows])

    provenance = {
        "input": str(path),
        "episode_kind": kind,
        "source": source,
        "target": target,
        "command_semantics": (
            EFFECTIVE_COMMAND_SEMANTICS
            if source == "effective" else DESIRED_COMMAND_SEMANTICS
        ),
        "slice": [
            int(target_slice(target).start),
            int(target_slice(target).stop),
        ],
        "unit": "rad",
        "hand_command_domain": "canonical_rad",
        "recorded_hand_domain": hand_domain if source == "effective" else "canonical_rad",
        "rows_total": total,
        "rows_dropped_nan": dropped if drop_nan else 0,
        "shape": list(selected.shape),
        "note": (
            "effective = final backend command (hardware executed 语义)"
            if source == "effective"
            else "desired = operator/policy intent，未经 safety/allowlist，"
                 "不是 hardware executed command"
        ),
    }
    return selected, provenance


def export_replay_commands(
    episode_npz: str | Path,
    *,
    target: str = "hand",
    source: str = "effective",
    output: str | Path | None = None,
    drop_nan: bool = False,
) -> tuple[Path, Path]:
    """导出 .npy + sidecar .json；返回两个文件路径。"""
    commands, provenance = load_replay_commands(
        episode_npz, target=target, source=source, drop_nan=drop_nan
    )
    path = Path(episode_npz)
    if output is None:
        output = path.with_name(
            f"{path.stem}_{target}_qcmd_{source}_rad.npy"
        )
    output = Path(output)
    if output.exists():
        raise FileExistsError(f"拒绝覆盖已有导出文件: {output}")
    np.save(output, commands)
    sidecar = output.with_name(output.name + ".json")
    with sidecar.open("w", encoding="utf-8") as handle:
        json.dump(provenance, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    return output, sidecar


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Export arm/hand/arm_hand replay commands from 17D storage"
    )
    parser.add_argument(
        "episode",
        help="RAW episode npz 或 curated trajectory.npz",
    )
    parser.add_argument(
        "--target",
        choices=TARGET_CHOICES,
        default="hand",
        help="arm=(N,6) hand=(N,11) arm_hand=(N,17)",
    )
    parser.add_argument(
        "--source",
        choices=SOURCE_CHOICES,
        default="effective",
        help="effective=final backend command；desired=provenance intent",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="输出 .npy 路径；默认 <stem>_<target>_qcmd_<source>_rad.npy",
    )
    parser.add_argument(
        "--nan",
        choices=("fail", "drop"),
        default="fail",
        help="含 NaN 行时 fail（默认）或 drop",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    output, sidecar = export_replay_commands(
        args.episode,
        target=args.target,
        source=args.source,
        output=args.output,
        drop_nan=args.nan == "drop",
    )
    print(f"EXPORTED: {output}")
    print(f"SIDECAR: {sidecar}")
    print(f"source: {args.source} target: {args.target}")


if __name__ == "__main__":
    main()
