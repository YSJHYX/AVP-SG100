"""Render offline camera-preview stills from a recorded MuJoCo session.

Example::

    python -m sg100_teleop.recording.render_camera_preview \
        --input recording/<session>/mujoco_recording.npz \
        --camera workstation_front --camera wrist_camera

For each requested camera this renders representative recorded states to
PNG stills under ``recording/camera_preview/``.  The wrist view also gets a
full-hand grasp-closure pose and a strong-thumb-opposition pose, making the
inner-palm VLA view directly inspectable.  The head view gets a diagnostic
close-up of the SG100 at the interaction frame; that close-up moves only the
in-memory camera used by this script and never adds a production MJCF camera.

Frame selection and state restoration reuse the visual-state recording
contract of :mod:`sg100_teleop.recording.render_mujoco_video`; this tool
adds no second timeline — every camera renders the exact same qpos rows.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

# Select a headless backend before importing MuJoCo, mirroring
# render_mujoco_video.py.
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np

from sg100_teleop.recording.render_mujoco_video import (
    DEFAULT_HEIGHT,
    DEFAULT_WIDTH,
    _resolve_scene,
    _restore_state,
    _validate_recording_arrays,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
PALM_BODY_NAME = "sg100_base_link"
OBJECT_BODY_NAME = "water_bottle"
FRAME_TAGS = ("home", "mid", "interaction")
CAMERA_TAGS = {"workstation_front": "head", "wrist_camera": "wrist"}
WRIST_CAMERA = "wrist_camera"
HEAD_CAMERA = "workstation_front"
THUMB_JOINT_DIRECTIONS = {
    "thumb_j1": 1.0,
    "thumb_j2": -1.0,
    "thumb_j3": 1.0,
}
HAND_JOINT_NAMES = (
    "thumb_j1", "thumb_j2", "thumb_j3",
    "index_j1", "index_j2", "index_j3",
    "middle_j1", "middle_j2",
    "little_j1", "little_j2", "little_j3",
)
HAND_BODY_NAMES = (
    "sg100_base_link",
    "thumb_link1", "thumb_link2", "thumb_link3",
    "index_link1", "index_link2", "index_link3",
    "middle_link1", "middle_link2",
    "little_link1", "little_llink2", "little_linik3",
)
APPROACH_DISTANCE = 0.18
DISPLACED_EPSILON = 0.03
CLOSE_HAND_DISTANCE = 0.38


def _select_frames(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos: np.ndarray,
    mocap_pos: np.ndarray,
    mocap_quat: np.ndarray,
    timestamps: np.ndarray,
    stride: int = 10,
) -> dict[str, int]:
    """Pick early / middle / interaction frame indices by FK only."""
    frame_count = timestamps.size
    frames = {
        "home": 0,
        "mid": frame_count // 2,
        "interaction": frame_count - 1,
    }

    palm_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, PALM_BODY_NAME
    )
    # Prefer the scene's primary manipulated object by name; the scene has
    # several free-joint props, and "first free joint" would pick an
    # arbitrary bystander (e.g. a drinks can) instead of the grasp target.
    object_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, OBJECT_BODY_NAME
    )
    if object_id < 0:
        for body_id in range(model.nbody):
            joint_adr = int(model.body_jntadr[body_id])
            if (
                model.body_jntnum[body_id] == 1
                and joint_adr >= 0
                and model.jnt_type[joint_adr]
                == mujoco.mjtJoint.mjJNT_FREE
            ):
                object_id = body_id
                break
    if object_id >= 0:
        _restore_state(
            model, data, qpos[0], mocap_pos[0], mocap_quat[0], timestamps[0]
        )
        object_origin = data.xpos[object_id].copy()

    if palm_id >= 0 and object_id >= 0:
        # The "interaction" tag targets the grasp-APPROACH moment: the first
        # recorded state in which the palm comes within APPROACH_DISTANCE of
        # the object while the object has not yet been displaced — after
        # pickup the object sits under/behind the palm, which is a placement
        # pose, not a hand-object approach pose.  If the session never
        # approaches (idle recording), fall back to the closest palm-object
        # distance over the whole session.
        best_distance = float("inf")
        approached = False
        palm_object_distance = np.full(frame_count, np.inf, dtype=np.float64)
        palm_xy_radius = np.zeros(frame_count, dtype=np.float64)
        # Evaluate every state once because the close/opposition diagnostics
        # must describe an actual hand-object interaction, not a maximally
        # curled hand after the object has already moved elsewhere.
        for index in range(frame_count):
            _restore_state(
                model,
                data,
                qpos[index],
                mocap_pos[index],
                mocap_quat[index],
                timestamps[index],
            )
            palm_object_distance[index] = np.linalg.norm(
                data.xpos[palm_id] - data.xpos[object_id]
            )
            palm_xy_radius[index] = np.linalg.norm(data.xpos[palm_id, :2])
        frames["far_reach"] = int(np.argmax(palm_xy_radius))
        for index in range(0, frame_count, max(1, stride)):
            _restore_state(
                model,
                data,
                qpos[index],
                mocap_pos[index],
                mocap_quat[index],
                timestamps[index],
            )
            object_now = data.xpos[object_id]
            distance = float(np.linalg.norm(data.xpos[palm_id] - object_now))
            displaced = float(np.linalg.norm(object_now - object_origin))
            if distance < best_distance:
                best_distance = distance
                frames["interaction"] = index
            if not approached and (
                distance < APPROACH_DISTANCE and displaced < DISPLACED_EPSILON
            ):
                approached = True
                frames["interaction"] = index
                break

    # Select strong opposition from the recorded hand state itself.  Each
    # thumb joint is normalized within its MJCF range, with the direction of
    # opposition encoded explicitly (thumb_j2 closes toward its lower bound).
    opposition_score = np.zeros(frame_count, dtype=np.float64)
    for joint_name, direction in THUMB_JOINT_DIRECTIONS.items():
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
        )
        if joint_id < 0:
            raise ValueError(f"Required thumb joint {joint_name!r} missing")
        qpos_adr = int(model.jnt_qposadr[joint_id])
        lower, upper = np.asarray(model.jnt_range[joint_id], dtype=np.float64)
        span = upper - lower
        if not np.isfinite(span) or span <= 0.0:
            raise ValueError(f"Invalid range for thumb joint {joint_name!r}")
        value = qpos[:, qpos_adr]
        progress = (
            (value - lower) / span
            if direction > 0.0
            else (upper - value) / span
        )
        opposition_score += np.clip(progress, 0.0, 1.0)
    near_object = (
        palm_object_distance < 0.16
        if palm_id >= 0 and object_id >= 0
        else np.ones(frame_count, dtype=bool)
    )
    if not np.any(near_object):
        near_object = np.ones(frame_count, dtype=bool)
    frames["thumb_opposition"] = int(
        np.argmax(np.where(near_object, opposition_score, -np.inf))
    )

    # The strongest complete grasp is not necessarily the strongest thumb
    # pose.  Normalize every hand joint by its range and find the recorded
    # frame furthest from the open/home hand.  This gives camera validation a
    # deterministic closed-hand frame where the palm cavity and all opposing
    # digits must remain visible.
    normalized_hand_motion = np.zeros(
        (frame_count, len(HAND_JOINT_NAMES)), dtype=np.float64
    )
    for column, joint_name in enumerate(HAND_JOINT_NAMES):
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
        )
        if joint_id < 0:
            raise ValueError(f"Required hand joint {joint_name!r} missing")
        qpos_adr = int(model.jnt_qposadr[joint_id])
        lower, upper = np.asarray(model.jnt_range[joint_id], dtype=np.float64)
        span = upper - lower
        if not np.isfinite(span) or span <= 0.0:
            raise ValueError(f"Invalid range for hand joint {joint_name!r}")
        normalized_hand_motion[:, column] = (
            qpos[:, qpos_adr] - qpos[0, qpos_adr]
        ) / span
    closure_score = np.linalg.norm(normalized_hand_motion, axis=1)
    frames["grasp_closure"] = int(
        np.argmax(np.where(near_object, closure_score, -np.inf))
    )

    return frames


def _hand_center(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Return a visual diagnostic target near the SG100 link centroid."""
    points = []
    for name in HAND_BODY_NAMES:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id >= 0:
            points.append(np.asarray(data.xpos[body_id], dtype=np.float64))
    if not points:
        raise ValueError("No SG100 bodies found for close-hand diagnostic")
    return np.mean(points, axis=0)


def render_camera_previews(
    input_path: str | Path,
    *,
    cameras: list[str],
    output_dir: str | Path | None = None,
    suffix: str = "",
    width: int = DEFAULT_WIDTH,
    height: int = DEFAULT_HEIGHT,
    repository_root: str | Path = REPOSITORY_ROOT,
) -> dict[str, object]:
    input_path = Path(input_path).resolve()
    if not input_path.is_file():
        raise FileNotFoundError(f"Recording file not found: {input_path}")

    repository_root = Path(repository_root).resolve()
    with np.load(input_path, allow_pickle=False) as archive:
        scene_identity = str(np.asarray(archive["scene_path"]).item())
        timestamps = np.asarray(archive["timestamps"], dtype=np.float64)
        qpos = np.asarray(archive["qpos"], dtype=np.float64)
        mocap_pos = np.asarray(archive["mocap_pos"], dtype=np.float64)
        mocap_quat = np.asarray(archive["mocap_quat"], dtype=np.float64)

    scene_path = _resolve_scene(scene_identity, repository_root)
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    _validate_recording_arrays(
        model, timestamps, qpos, mocap_pos, mocap_quat
    )
    model.vis.global_.offwidth = max(
        int(model.vis.global_.offwidth), width
    )
    model.vis.global_.offheight = max(
        int(model.vis.global_.offheight), height
    )

    output_dir = (
        Path(output_dir).resolve()
        if output_dir is not None
        else REPOSITORY_ROOT / "recording" / "camera_preview"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    data = mujoco.MjData(model)
    frames = _select_frames(
        model, data, qpos, mocap_pos, mocap_quat, timestamps
    )

    rendered: dict[str, dict[str, dict[str, float]]] = {}
    with mujoco.Renderer(model, height=height, width=width) as renderer:
        for camera in cameras:
            camera_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_CAMERA, camera
            )
            if camera_id < 0:
                raise ValueError(f"Camera {camera!r} not found")
            camera_tag = CAMERA_TAGS.get(camera, camera)
            rendered[camera] = {}
            camera_frames = list(FRAME_TAGS)
            if camera == WRIST_CAMERA:
                camera_frames.extend(("grasp_closure", "thumb_opposition"))
            if camera == HEAD_CAMERA:
                camera_frames.extend(("far_reach", "close_hand"))
            for tag in camera_frames:
                index = frames[
                    "interaction" if tag == "close_hand" else tag
                ]
                _restore_state(
                    model,
                    data,
                    qpos[index],
                    mocap_pos[index],
                    mocap_quat[index],
                    timestamps[index],
                )
                saved_camera_pos = None
                if tag == "close_hand":
                    # Preserve the production camera's orientation and place
                    # a temporary world-fixed copy close to the hand.  The
                    # loaded model is private to this process, so this cannot
                    # mutate the MJCF or the recording timeline.
                    saved_camera_pos = model.cam_pos[camera_id].copy()
                    camera_rotation = data.cam_xmat[camera_id].reshape(3, 3)
                    optical_axis = -camera_rotation[:, 2]
                    model.cam_pos[camera_id] = (
                        _hand_center(model, data)
                        - CLOSE_HAND_DISTANCE * optical_axis
                    )
                    mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera=camera_id)
                pixels = np.ascontiguousarray(renderer.render())
                if saved_camera_pos is not None:
                    model.cam_pos[camera_id] = saved_camera_pos
                    mujoco.mj_forward(model, data)
                if pixels.dtype != np.uint8 or pixels.shape != (
                    height,
                    width,
                    3,
                ):
                    raise RuntimeError(
                        f"Unexpected frame for {camera}/{tag}: "
                        f"shape={pixels.shape}, dtype={pixels.dtype}"
                    )
                if not np.all(np.isfinite(pixels)):
                    raise RuntimeError(
                        f"Non-finite pixels for {camera}/{tag}"
                    )
                suffix_token = f"_{suffix.strip('_')}" if suffix else ""
                output_path = output_dir / (
                    f"{camera_tag}_{tag}{suffix_token}.png"
                )
                if not _write_png(output_path, pixels):
                    raise RuntimeError(f"Failed to write {output_path}")
                rendered[camera][tag] = {
                    "frame": float(index),
                    "time": float(timestamps[index]),
                    "mean_luminance": float(pixels.mean()),
                    "variance": float(pixels.var()),
                    "path": str(output_path),
                }

    return {"input": str(input_path), "frames": frames, "cameras": rendered}


def _write_png(path: Path, pixels: np.ndarray) -> bool:
    """Write an 8-bit RGB PNG without adding imaging dependencies."""
    try:
        import PIL.Image
    except ImportError:
        return _write_png_av(path, pixels)
    PIL.Image.fromarray(pixels, mode="RGB").save(path)
    return True


def _write_png_av(path: Path, pixels: np.ndarray) -> bool:
    """PNG fallback through PyAV (already a recording dependency)."""
    import av

    container = av.open(str(path), mode="w", format="png")
    stream = container.add_stream("png", rate=1)
    stream.width = pixels.shape[1]
    stream.height = pixels.shape[0]
    stream.pix_fmt = "rgb24"
    frame = av.VideoFrame.from_ndarray(pixels, format="rgb24")
    for packet in stream.encode(frame):
        container.mux(packet)
    for packet in stream.encode():
        container.mux(packet)
    container.close()
    return True


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Render camera preview stills from a recording."
    )
    parser.add_argument("--input", required=True, help="mujoco_recording.npz")
    parser.add_argument(
        "--camera",
        action="append",
        required=True,
        dest="cameras",
        help="MuJoCo camera name (repeatable)",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Output directory (default: recording/camera_preview)",
    )
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument(
        "--suffix",
        default="",
        help="Optional filename suffix, e.g. 'v2'",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    summary = render_camera_previews(
        args.input,
        cameras=args.cameras,
        output_dir=args.output_dir,
        suffix=args.suffix,
        width=args.width,
        height=args.height,
    )
    print(f"Input: {summary['input']}")
    print(f"Frames: {summary['frames']}")
    for camera, views in summary["cameras"].items():
        for tag, stats in views.items():
            print(
                f"{camera:>20}/{tag:<12} frame={int(stats['frame']):5d} "
                f"t={stats['time']:7.2f}s mean={stats['mean_luminance']:6.1f} "
                f"var={stats['variance']:8.1f} -> {stats['path']}"
            )


if __name__ == "__main__":
    main()
