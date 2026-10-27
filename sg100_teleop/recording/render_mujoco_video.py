"""Render a recorded MuJoCo visual-state session to a silent H.264 MP4.

Example::

    python -m sg100_teleop.recording.render_mujoco_video \
        --input recording/<session>/mujoco_recording.npz

Optional overrides::

    --output PATH --camera workstation_front --fps 30 \
    --width 1280 --height 720

The renderer restores state and calls ``mj_forward``; it never re-runs
physics. ``--fps`` changes playback rate only (one recorded state remains one
video frame). The output has one H.264/yuv420p video stream and no audio.
"""

from __future__ import annotations

import argparse
from fractions import Fraction
import os
from pathlib import Path
import warnings

# Select a headless backend before importing MuJoCo. Respect an explicit user
# choice such as osmesa, but make Ubuntu/EGL the default for this CLI.
os.environ.setdefault("MUJOCO_GL", "egl")

import av
import mujoco
import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CAMERA = "workstation_front"
DEFAULT_FPS = 30.0
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
BLACK_MEAN_THRESHOLD = 2.0
CONSTANT_VARIANCE_THRESHOLD = 2.0


def _scalar_string(archive: np.lib.npyio.NpzFile, key: str) -> str | None:
    if key not in archive:
        return None
    value = np.asarray(archive[key])
    if value.size != 1:
        raise ValueError(f"Recording field {key!r} must be scalar")
    return str(value.item())


def _scalar_float(archive: np.lib.npyio.NpzFile, key: str) -> float | None:
    if key not in archive:
        return None
    value = np.asarray(archive[key], dtype=np.float64)
    if value.size != 1:
        raise ValueError(f"Recording field {key!r} must be scalar")
    result = float(value.item())
    return result if np.isfinite(result) else None


def _resolve_scene(scene_identity: str, repository_root: Path) -> Path:
    scene = Path(scene_identity)
    if not scene.is_absolute():
        scene = repository_root / scene
    scene = scene.resolve()
    if not scene.is_file():
        raise FileNotFoundError(
            f"Recorded MuJoCo scene does not exist: {scene}\n"
            f"Stored scene identity: {scene_identity}"
        )
    return scene


def _validate_recording_arrays(
    model: mujoco.MjModel,
    timestamps: np.ndarray,
    qpos: np.ndarray,
    mocap_pos: np.ndarray,
    mocap_quat: np.ndarray,
) -> None:
    if timestamps.ndim != 1 or timestamps.size == 0:
        raise ValueError("Recording timestamps must be a non-empty 1D array")
    frame_count = timestamps.size
    if qpos.shape != (frame_count, model.nq):
        raise ValueError(
            f"Recording qpos shape {qpos.shape} does not match "
            f"({frame_count}, {model.nq})"
        )
    if mocap_pos.shape != (frame_count, model.nmocap, 3):
        raise ValueError(
            f"Recording mocap_pos shape {mocap_pos.shape} does not match "
            f"({frame_count}, {model.nmocap}, 3)"
        )
    if mocap_quat.shape != (frame_count, model.nmocap, 4):
        raise ValueError(
            f"Recording mocap_quat shape {mocap_quat.shape} does not match "
            f"({frame_count}, {model.nmocap}, 4)"
        )
    for name, values in (
        ("timestamps", timestamps),
        ("qpos", qpos),
        ("mocap_pos", mocap_pos),
        ("mocap_quat", mocap_quat),
    ):
        if not np.all(np.isfinite(values)):
            raise ValueError(f"Recording {name} contains non-finite values")


def _restore_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos: np.ndarray,
    mocap_pos: np.ndarray,
    mocap_quat: np.ndarray,
    timestamp: float,
) -> None:
    data.qpos[:] = qpos
    data.mocap_pos[:] = mocap_pos
    data.mocap_quat[:] = mocap_quat
    data.time = float(timestamp)
    mujoco.mj_forward(model, data)


def _frame_sanity(frame: np.ndarray) -> tuple[float, float]:
    if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
        raise RuntimeError(
            f"Renderer returned unexpected frame: shape={frame.shape}, dtype={frame.dtype}"
        )
    if not np.all(np.isfinite(frame)):
        raise RuntimeError("Renderer returned non-finite pixels")
    mean = float(frame.mean())
    variance = float(frame.var())
    if mean < BLACK_MEAN_THRESHOLD or variance < CONSTANT_VARIANCE_THRESHOLD:
        warnings.warn(
            "Rendered frame appears nearly black/constant. Check camera and lighting.",
            RuntimeWarning,
            stacklevel=2,
        )
    return mean, variance


def render_recording(
    input_path: str | Path,
    *,
    output_path: str | Path | None = None,
    camera: str | None = None,
    fps: float | None = None,
    width: int = DEFAULT_WIDTH,
    height: int = DEFAULT_HEIGHT,
    repository_root: str | Path = REPOSITORY_ROOT,
) -> dict[str, object]:
    input_path = Path(input_path).resolve()
    if not input_path.is_file():
        raise FileNotFoundError(f"Recording file not found: {input_path}")
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")

    repository_root = Path(repository_root).resolve()
    with np.load(input_path, allow_pickle=False) as archive:
        scene_identity = _scalar_string(archive, "scene_path")
        if not scene_identity:
            raise ValueError("Recording has no scene_path metadata")
        recorded_camera = _scalar_string(archive, "camera_name")
        recorded_fps = _scalar_float(archive, "record_fps")
        timestamps = np.asarray(archive["timestamps"], dtype=np.float64)
        qpos = np.asarray(archive["qpos"], dtype=np.float64)
        mocap_pos = np.asarray(archive["mocap_pos"], dtype=np.float64)
        mocap_quat = np.asarray(archive["mocap_quat"], dtype=np.float64)

    scene_path = _resolve_scene(scene_identity, repository_root)
    camera = camera or recorded_camera or DEFAULT_CAMERA
    fps = float(fps if fps is not None else (recorded_fps or DEFAULT_FPS))
    if not np.isfinite(fps) or fps <= 0.0:
        raise ValueError("fps must be finite and > 0")

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    camera_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, camera
    )
    if camera_id < 0:
        raise ValueError(f"Camera {camera!r} not found")
    _validate_recording_arrays(model, timestamps, qpos, mocap_pos, mocap_quat)

    # Permit explicit higher-resolution overrides without requiring another
    # MJCF edit. This only changes the offscreen framebuffer allocation.
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), width)
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), height)

    output = (
        Path(output_path).resolve()
        if output_path is not None
        else input_path.with_name("mujoco_view.mp4")
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary_output = output.with_name(f".{output.stem}.tmp{output.suffix}")

    rate = Fraction(str(fps)).limit_denominator(100_000)
    container = av.open(str(temporary_output), mode="w", format="mp4")
    stream = container.add_stream("libx264", rate=rate)
    stream.width = width
    stream.height = height
    stream.pix_fmt = "yuv420p"
    stream.bit_rate = 8_000_000

    data = mujoco.MjData(model)
    first_mean = 0.0
    first_variance = 0.0
    try:
        with mujoco.Renderer(model, height=height, width=width) as renderer:
            for index in range(timestamps.size):
                _restore_state(
                    model,
                    data,
                    qpos[index],
                    mocap_pos[index],
                    mocap_quat[index],
                    timestamps[index],
                )
                renderer.update_scene(data, camera=camera)
                pixels = np.ascontiguousarray(renderer.render())
                if index == 0:
                    first_mean, first_variance = _frame_sanity(pixels)
                video_frame = av.VideoFrame.from_ndarray(pixels, format="rgb24")
                for packet in stream.encode(video_frame):
                    container.mux(packet)
            for packet in stream.encode():
                container.mux(packet)
    finally:
        container.close()

    if not temporary_output.is_file() or temporary_output.stat().st_size <= 0:
        raise RuntimeError(f"MP4 encoder produced no output: {temporary_output}")
    temporary_output.replace(output)

    summary: dict[str, object] = {
        "input": input_path,
        "scene": scene_path,
        "camera": camera,
        "frames": int(timestamps.size),
        "fps": fps,
        "width": width,
        "height": height,
        "duration": timestamps.size / fps,
        "output": output,
        "first_frame_mean": first_mean,
        "first_frame_variance": first_variance,
    }
    return summary


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Render MuJoCo visual-state recording to silent H.264 MP4."
    )
    parser.add_argument("--input", required=True, help="mujoco_recording.npz")
    parser.add_argument(
        "--output",
        default=None,
        help="Output MP4 (default: mujoco_view.mp4 beside the input)",
    )
    parser.add_argument(
        "--camera",
        default=None,
        help="MuJoCo camera (default: recording metadata or workstation_front)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=None,
        help="Playback FPS (default: recording metadata, fallback 30)",
    )
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    summary = render_recording(
        args.input,
        output_path=args.output,
        camera=args.camera,
        fps=args.fps,
        width=args.width,
        height=args.height,
    )
    print(f"Input: {summary['input']}")
    print(f"Scene: {summary['scene']}")
    print(f"Camera: {summary['camera']}")
    print(f"Frames: {summary['frames']}")
    print(f"FPS: {summary['fps']:g}")
    print(f"Resolution: {summary['width']}x{summary['height']}")
    print(f"Duration: {summary['duration']:.2f} s")
    print(f"Output: {summary['output']}")


if __name__ == "__main__":
    main()
