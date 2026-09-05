"""只读 RealSense raw/aligned depth quality probe；不连接任何机器人。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class DepthFrameStats:
    total_pixels: int
    valid_pixels: int
    zero_pixels: int
    valid_ratio: float
    zero_ratio: float
    min_nonzero_raw: float
    median_nonzero_raw: float
    p5_nonzero_raw: float
    p95_nonzero_raw: float
    max_nonzero_raw: float
    min_m: float
    median_m: float
    p5_m: float
    p95_m: float
    max_m: float
    center_valid_ratio: float
    center_median_m: float


def _percentile(values: np.ndarray, percentile: float) -> float:
    return float(np.percentile(values, percentile)) if values.size else float("nan")


def _finite_median(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    return float(np.median(finite)) if finite.size else float("nan")


def compute_depth_stats(
    depth_uint16: np.ndarray,
    depth_scale: float,
) -> DepthFrameStats:
    """统计一张未改值的 Z16；center ROI 为 x/y 各 25%~75%。"""
    depth = np.asarray(depth_uint16)
    if depth.dtype != np.uint16 or depth.ndim != 2:
        raise ValueError("depth 必须是 uint16 HxW raw Z16")
    scale = float(depth_scale)
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("depth_scale 必须是有限正数")
    valid_mask = depth > 0
    nonzero = depth[valid_mask].astype(np.float64, copy=False)
    total = int(depth.size)
    valid = int(np.count_nonzero(valid_mask))
    zero = total - valid
    height, width = depth.shape
    y0, y1 = int(height * 0.25), int(height * 0.75)
    x0, x1 = int(width * 0.25), int(width * 0.75)
    center = depth[y0:y1, x0:x1]
    center_nonzero = center[center > 0].astype(np.float64, copy=False)
    raw_values = (
        _percentile(nonzero, 0),
        _percentile(nonzero, 50),
        _percentile(nonzero, 5),
        _percentile(nonzero, 95),
        _percentile(nonzero, 100),
    )
    meter_values = tuple(value * scale for value in raw_values)
    return DepthFrameStats(
        total_pixels=total,
        valid_pixels=valid,
        zero_pixels=zero,
        valid_ratio=float(valid / total) if total else 0.0,
        zero_ratio=float(zero / total) if total else 0.0,
        min_nonzero_raw=raw_values[0],
        median_nonzero_raw=raw_values[1],
        p5_nonzero_raw=raw_values[2],
        p95_nonzero_raw=raw_values[3],
        max_nonzero_raw=raw_values[4],
        min_m=meter_values[0],
        median_m=meter_values[1],
        p5_m=meter_values[2],
        p95_m=meter_values[3],
        max_m=meter_values[4],
        center_valid_ratio=(
            float(center_nonzero.size / center.size) if center.size else 0.0
        ),
        center_median_m=_percentile(center_nonzero, 50) * scale,
    )


def aggregate_valid_ratios(stats: Iterable[DepthFrameStats]) -> dict[str, float]:
    ratios = np.asarray([item.valid_ratio for item in stats], dtype=np.float64)
    if ratios.size == 0:
        raise ValueError("至少需要一帧 depth stats")
    return {
        "mean": float(ratios.mean()),
        "p5": float(np.percentile(ratios, 5)),
        "p50": float(np.percentile(ratios, 50)),
        "p95": float(np.percentile(ratios, 95)),
        "min": float(ratios.min()),
        "max": float(ratios.max()),
    }


def compare_validity(
    raw: dict[str, float],
    aligned: dict[str, float],
    *,
    low_threshold: float = 0.5,
    align_drop_threshold: float = 0.1,
) -> tuple[float, str]:
    """返回 aligned-raw mean delta 和只读诊断分类。"""
    raw_mean = float(raw["mean"])
    aligned_mean = float(aligned["mean"])
    delta = aligned_mean - raw_mean
    if raw_mean < low_threshold and aligned_mean < low_threshold:
        diagnosis = "CAMERA / SCENE / DISTANCE / STEREO issue"
    elif raw_mean - aligned_mean >= align_drop_threshold:
        diagnosis = "ALIGN / PROCESSING issue"
    elif raw_mean >= low_threshold and aligned_mean >= low_threshold:
        diagnosis = "raw/aligned normal; inspect RECORDER WRITE / BUFFER / CONVERSION"
    else:
        diagnosis = "INCONCLUSIVE; inspect per-frame and center ROI metrics"
    return delta, diagnosis


def _frame_line(label: str, index: int, stats: DepthFrameStats) -> str:
    return (
        f"frame={index:03d} {label} total={stats.total_pixels} "
        f"valid={stats.valid_pixels} zero={stats.zero_pixels} "
        f"valid_ratio={stats.valid_ratio:.6f} zero_ratio={stats.zero_ratio:.6f} "
        f"raw[min/median/p5/p95/max]={stats.min_nonzero_raw:.1f}/"
        f"{stats.median_nonzero_raw:.1f}/{stats.p5_nonzero_raw:.1f}/"
        f"{stats.p95_nonzero_raw:.1f}/{stats.max_nonzero_raw:.1f} "
        f"m[min/median/p5/p95/max]={stats.min_m:.4f}/"
        f"{stats.median_m:.4f}/{stats.p5_m:.4f}/"
        f"{stats.p95_m:.4f}/{stats.max_m:.4f} "
        f"center_valid_ratio={stats.center_valid_ratio:.6f} "
        f"center_median_m={stats.center_median_m:.4f}"
    )


def _device_info(device, rs, name: str) -> str:
    key = getattr(rs.camera_info, name, None)
    if key is None:
        return "UNSUPPORTED"
    try:
        if hasattr(device, "supports") and not device.supports(key):
            return "UNSUPPORTED"
        return str(device.get_info(key))
    except Exception:
        return "UNSUPPORTED"


def _option_value(sensor, rs, name: str) -> str:
    option = getattr(rs.option, name, None)
    if option is None:
        return "UNSUPPORTED"
    try:
        if not sensor.supports(option):
            return "UNSUPPORTED"
        return str(sensor.get_option(option))
    except Exception:
        return "UNSUPPORTED"


def _write_previews(output_dir: Path, raw: np.ndarray, aligned: np.ndarray) -> None:
    import cv2

    output_dir.mkdir(parents=True, exist_ok=True)
    for label, depth in (("raw", raw), ("aligned", aligned)):
        valid = depth > 0
        nonzero = depth[valid]
        preview = np.zeros(depth.shape, dtype=np.uint8)
        if nonzero.size:
            low, high = np.percentile(nonzero, (5, 95))
            if high > low:
                scaled = np.clip((depth.astype(np.float64) - low) * 255 / (high - low), 0, 255)
                preview[valid] = scaled[valid].astype(np.uint8)
        colorized = cv2.applyColorMap(preview, cv2.COLORMAP_TURBO)
        colorized[~valid] = 0
        cv2.imwrite(str(output_dir / f"{label}_depth_preview.png"), colorized)
        cv2.imwrite(
            str(output_dir / f"{label}_valid_mask.png"),
            valid.astype(np.uint8) * 255,
        )
    print(f"Diagnostic previews (NOT RAW DATA): {output_dir}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only D455 raw vs aligned depth quality probe"
    )
    parser.add_argument("--serial", required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--preview-dir", type=Path, default=None)
    parser.add_argument("--quiet-per-frame", action="store_true")
    return parser


def run_probe(args, *, rs_module=None) -> None:
    if args.width <= 0 or args.height <= 0 or args.fps <= 0 or args.seconds <= 0:
        raise ValueError("width/height/fps/seconds 必须 > 0")
    if rs_module is None:
        try:
            import pyrealsense2 as rs_module
        except ImportError as exc:
            raise RuntimeError("缺少 pyrealsense2") from exc
    rs = rs_module
    target_frames = max(100, min(300, int(math.ceil(args.seconds * args.fps))))
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(args.serial)
    config.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.rgb8, args.fps
    )
    config.enable_stream(
        rs.stream.depth, args.width, args.height, rs.format.z16, args.fps
    )
    active = pipeline.start(config)
    raw_stats: list[DepthFrameStats] = []
    aligned_stats: list[DepthFrameStats] = []
    latest_raw = latest_aligned = None
    try:
        device = active.get_device()
        depth_sensor = device.first_depth_sensor()
        depth_scale = float(depth_sensor.get_depth_scale())
        print(f"model: {_device_info(device, rs, 'name')}")
        print(f"serial: {_device_info(device, rs, 'serial_number')}")
        print(f"firmware: {_device_info(device, rs, 'firmware_version')}")
        print(f"usb_type: {_device_info(device, rs, 'usb_type_descriptor')}")
        print(f"depth_scale: {depth_scale}")
        for name in (
            "visual_preset", "emitter_enabled", "laser_power",
            "min_distance", "max_distance",
        ):
            print(f"{name}: {_option_value(depth_sensor, rs, name)}")
        print(f"target_frames: {target_frames}")
        align = rs.align(rs.stream.color)
        for index in range(target_frames):
            frameset = pipeline.wait_for_frames(timeout_ms=2000)
            raw_frame = frameset.get_depth_frame()
            if not raw_frame:
                raise RuntimeError(f"frame {index}: raw depth missing")
            # Copy raw before align processing; neither path mutates/persists Z16.
            latest_raw = np.asanyarray(raw_frame.get_data()).astype(
                np.uint16, copy=True
            )
            aligned_frame = align.process(frameset).get_depth_frame()
            if not aligned_frame:
                raise RuntimeError(f"frame {index}: aligned depth missing")
            latest_aligned = np.asanyarray(aligned_frame.get_data()).astype(
                np.uint16, copy=True
            )
            raw_item = compute_depth_stats(latest_raw, depth_scale)
            aligned_item = compute_depth_stats(latest_aligned, depth_scale)
            raw_stats.append(raw_item)
            aligned_stats.append(aligned_item)
            if not args.quiet_per_frame:
                print(_frame_line("RAW", index, raw_item))
                print(_frame_line("ALIGNED", index, aligned_item))
        raw_summary = aggregate_valid_ratios(raw_stats)
        aligned_summary = aggregate_valid_ratios(aligned_stats)
        print("\nRAW DEPTH valid ratio")
        print(" ".join(f"{key}={value:.6f}" for key, value in raw_summary.items()))
        print("\nALIGNED DEPTH valid ratio")
        print(" ".join(f"{key}={value:.6f}" for key, value in aligned_summary.items()))
        raw_center = np.asarray([item.center_valid_ratio for item in raw_stats])
        aligned_center = np.asarray([item.center_valid_ratio for item in aligned_stats])
        raw_center_m = np.asarray([item.center_median_m for item in raw_stats])
        aligned_center_m = np.asarray([item.center_median_m for item in aligned_stats])
        print(
            "center ROI x/y=25%~75%: "
            f"raw_valid_mean={raw_center.mean():.6f} "
            f"aligned_valid_mean={aligned_center.mean():.6f} "
            f"raw_median_m={_finite_median(raw_center_m):.4f} "
            f"aligned_median_m={_finite_median(aligned_center_m):.4f}"
        )
        delta, diagnosis = compare_validity(raw_summary, aligned_summary)
        print(f"raw_vs_aligned_validity_delta(aligned-raw): {delta:.6f}")
        print(f"diagnosis: {diagnosis}")
        if args.preview_dir is not None:
            assert latest_raw is not None and latest_aligned is not None
            _write_previews(args.preview_dir, latest_raw, latest_aligned)
    finally:
        pipeline.stop()


def main() -> None:
    run_probe(build_arg_parser().parse_args())


if __name__ == "__main__":
    main()
