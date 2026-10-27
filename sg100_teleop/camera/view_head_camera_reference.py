"""D455 实时预览叠加新拍摄或已有的反色参考图。

运行：python -m sg100_teleop.camera.view_head_camera_reference
也支持：python sg100_teleop/camera/view_head_camera_reference.py
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sg100_teleop.camera.realsense_camera import RealSenseCamera


WINDOW_NAME = "D455 Head Camera | O overlay / L live / B background / Q quit"
FRAME_TIMEOUT_S = 5.0


def select_d455(rs, serial: str | None = None) -> tuple[str, str]:
    """自动选择唯一 D455；多台时必须指定 serial，避免拍到其他相机。"""
    cameras = [
        (
            str(device.get_info(rs.camera_info.serial_number)),
            str(device.get_info(rs.camera_info.name)),
        )
        for device in rs.context().query_devices()
    ]
    candidates = [item for item in cameras if item[1].endswith("RealSense D455")]
    if serial is not None:
        candidates = [item for item in candidates if item[0] == serial]
    if not candidates:
        detected = ", ".join(f"{name} ({number})" for number, name in cameras)
        raise RuntimeError(
            f"未找到 D455{f' serial={serial}' if serial else ''}；"
            f"检测到：{detected or '无 RealSense 设备'}"
        )
    if len(candidates) != 1:
        raise RuntimeError(
            "检测到多台 D455，请通过 --serial 指定头部相机："
            + ", ".join(number for number, _ in candidates)
        )
    return candidates[0]


def _validate_rgb(rgb: np.ndarray) -> None:
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("图像必须是 uint8 HxWx3 RGB")


def invert_rgb(rgb: np.ndarray) -> np.ndarray:
    _validate_rgb(rgb)
    return np.subtract(255, rgb, dtype=np.uint8)


def load_reference_image(path: Path, *, width: int, height: int, cv2) -> np.ndarray:
    """读取已经反色的参考图；保持像素和尺寸不变。"""
    path = path.expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"反色参考图不存在：{path}")
    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError(f"无法读取反色参考图：{path}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    _validate_rgb(rgb)
    expected_shape = (height, width, 3)
    if rgb.shape != expected_shape:
        raise ValueError(
            f"反色参考图尺寸为 {rgb.shape[1]}x{rgb.shape[0]}，"
            f"实时画面配置为 {width}x{height}；请使用相同分辨率"
        )
    return rgb


def compose_preview(
    live_rgb: np.ndarray, background_rgb: np.ndarray, alpha: float,
) -> np.ndarray:
    """alpha 是实时画面权重；反色参考图保持固定，不改动原始帧。"""
    _validate_rgb(live_rgb)
    _validate_rgb(background_rgb)
    if live_rgb.shape != background_rgb.shape:
        raise ValueError("实时画面和反色参考图尺寸不一致")
    if not math.isfinite(alpha) or not 0.0 <= alpha <= 1.0:
        raise ValueError("alpha 必须在 [0, 1] 内")
    blended = alpha * live_rgb.astype(np.float32)
    blended += (1.0 - alpha) * background_rgb.astype(np.float32)
    return np.rint(blended).astype(np.uint8)


def wait_for_reference(camera, warmup_seconds: float):
    """等待预热结束后新收到的帧，而不是使用启动时缓存的旧帧。"""
    reference_after_ns = time.monotonic_ns() + int(warmup_seconds * 1e9)
    deadline_ns = reference_after_ns + int(FRAME_TIMEOUT_S * 1e9)
    while time.monotonic_ns() < deadline_ns:
        frame = camera.latest_before(time.monotonic_ns())
        if frame is not None and frame.host_receive_monotonic_ns >= reference_after_ns:
            return frame
        time.sleep(0.01)
    raise RuntimeError(f"D455 拍照超时：{camera.last_error or '未收到新的 RGB 帧'}")


def save_reference(output_dir: Path, frame, background_rgb, camera_metadata, cv2):
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
    photo_path = output_dir / f"head_{stamp}.png"
    inverted_path = output_dir / f"head_{stamp}_inverted.png"
    metadata_path = output_dir / f"head_{stamp}.json"
    for path, rgb in (
        (photo_path, frame.color_rgb), (inverted_path, background_rgb),
    ):
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if not cv2.imwrite(str(path), bgr):
            raise RuntimeError(f"照片保存失败：{path}")
    metadata = {
        "model": camera_metadata["model"],
        "serial": camera_metadata["serial"],
        "color_stream_profile": camera_metadata["color_stream_profile"],
        "color_controls": camera_metadata["color_controls"],
        "frame_number": frame.frame_number,
        "host_receive_monotonic_ns": frame.host_receive_monotonic_ns,
        "device_timestamp_ms": frame.device_timestamp_ms,
        "photo": photo_path.name,
        "inverted_photo": inverted_path.name,
    }
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return photo_path, inverted_path, metadata_path


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", help="D455 序列号；省略时自动选择唯一的 D455")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-seconds", type=float, default=2.0)
    parser.add_argument("--alpha", type=float, default=0.5, help="实时画面权重，默认 0.5")
    parser.add_argument(
        "--output-dir", type=Path, default=Path("recording/head_camera_reference"),
        help="保存原图、反色图和相机信息的目录",
    )
    parser.add_argument(
        "--reference-image", type=Path,
        help="已有的反色 PNG/JPEG；提供后跳过重新拍照，直接作为实时叠加背景",
    )
    parser.add_argument("--capture-only", action="store_true", help="仅拍照保存，不打开窗口")
    return parser


def run_viewer(args, *, rs_module=None, cv2_module=None, camera_factory=RealSenseCamera):
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        raise ValueError("width/height/fps 必须 > 0")
    if not math.isfinite(args.warmup_seconds) or args.warmup_seconds < 0:
        raise ValueError("warmup-seconds 必须是有限非负数")
    if not math.isfinite(args.alpha) or not 0.0 <= args.alpha <= 1.0:
        raise ValueError("alpha 必须在 [0, 1] 内")
    if args.reference_image is not None and args.capture_only:
        raise ValueError("--reference-image 不能与 --capture-only 同时使用")
    if rs_module is None:
        import pyrealsense2 as rs_module
    if cv2_module is None:
        import cv2 as cv2_module
    cv2 = cv2_module
    reference_path = (
        args.reference_image.expanduser().resolve()
        if args.reference_image is not None else None
    )
    loaded_background = (
        load_reference_image(
            reference_path, width=args.width, height=args.height, cv2=cv2,
        )
        if reference_path is not None else None
    )
    serial, model = select_d455(rs_module, args.serial)
    camera = camera_factory(
        role="head", serial=serial, expected_model=model,
        width=args.width, height=args.height, fps=args.fps,
        enable_depth=False, ring_size=4, rs_module=rs_module,
    )
    window_created = False
    try:
        print(f"[HEAD] {model} serial={serial}", flush=True)
        camera.start()
        if loaded_background is None:
            reference = wait_for_reference(camera, args.warmup_seconds)
            background_rgb = invert_rgb(reference.color_rgb)
            paths = save_reference(
                args.output_dir, reference, background_rgb, camera.metadata(), cv2,
            )
            print(f"[HEAD] 原图：{paths[0].resolve()}", flush=True)
            print(f"[HEAD] 反色图：{paths[1].resolve()}", flush=True)
            if args.capture_only:
                return paths
        else:
            background_rgb = loaded_background
            paths = (reference_path,)
            print(f"[HEAD] 已加载反色图：{reference_path}", flush=True)

        if sys.platform.startswith("linux") and not (
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        ):
            if loaded_background is None:
                raise RuntimeError(
                    "原图和反色图已保存，但当前没有桌面显示；"
                    "请在桌面终端运行，或使用 --capture-only"
                )
            raise RuntimeError(
                "已有反色图已加载，但当前没有桌面显示；请在桌面终端运行"
            )
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        window_created = True
        cv2.resizeWindow(WINDOW_NAME, args.width, args.height)
        cv2.createTrackbar("Live weight %", WINDOW_NAME, round(args.alpha * 100), 100, lambda _: None)
        mode = "overlay"
        print("[HEAD] O 叠加 / L 实时 / B 反色背景 / Q 或 Esc 退出；滑块调整透明度", flush=True)
        while True:
            now_ns = time.monotonic_ns()
            live = camera.latest_before(now_ns)
            if live is None or now_ns - live.host_receive_monotonic_ns > FRAME_TIMEOUT_S * 1e9:
                raise RuntimeError(f"D455 实时画面中断：{camera.last_error or 'RGB 帧超时'}")
            alpha = cv2.getTrackbarPos("Live weight %", WINDOW_NAME) / 100.0
            if mode == "background":
                preview = background_rgb
            elif mode == "live":
                preview = live.color_rgb
            else:
                preview = compose_preview(live.color_rgb, background_rgb, alpha)
            cv2.imshow(WINDOW_NAME, cv2.cvtColor(preview, cv2.COLOR_RGB2BGR))
            key = cv2.waitKey(10) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (ord("b"), ord("B")):
                mode = "background"
            elif key in (ord("l"), ord("L")):
                mode = "live"
            elif key in (ord("o"), ord("O")):
                mode = "overlay"
        return paths
    finally:
        try:
            camera.stop()
        finally:
            if window_created:
                cv2.destroyWindow(WINDOW_NAME)


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    try:
        run_viewer(args)
    except KeyboardInterrupt:
        print("\n[HEAD] 已关闭相机预览")
    except (RuntimeError, ValueError, ImportError) as exc:
        parser.exit(1, f"[HEAD] {exc}\n")


if __name__ == "__main__":
    main()
