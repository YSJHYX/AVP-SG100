"""Offline diagnostics for the Phase-1 v6 dual-camera refinement.

The tool never changes a recording or the runtime observation mapping.  It
loads the production MJCF, restores the same recorded MuJoCo states used by
the video renderer, evaluates candidate fixed camera poses in memory, and
writes labelled PNG diagnostics for human inspection.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from sg100_teleop.recording.render_camera_preview import _select_frames
from sg100_teleop.recording.render_mujoco_video import (
    _resolve_scene,
    _restore_state,
    _validate_recording_arrays,
)


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RECORDING = ROOT / "recording/20260820_181024/mujoco_recording.npz"
HEAD_CAMERA = "workstation_front"
WRIST_CAMERA = "wrist_camera"
PALM_BODY = "sg100_base_link"
OBJECT_BODY = "water_bottle"
TIP_BODIES = {
    "thumb": "thumb_link3",
    "index": "index_link3",
    "middle": "middle_link2",
    "little": "little_linik3",
}
TABLE_CORNERS = {
    "C1 near-right": np.array([-0.15, -0.35, 0.0]),
    "C2 far-right": np.array([0.75, -0.35, 0.0]),
    "C3 far-left": np.array([0.75, 0.65, 0.0]),
    "C4 near-left": np.array([-0.15, 0.65, 0.0]),
}


def _id(model: mujoco.MjModel, obj: mujoco.mjtObj, name: str) -> int:
    value = mujoco.mj_name2id(model, obj, name)
    if value < 0:
        raise ValueError(f"Required {obj.name} {name!r} is missing")
    return value


def look_at_xyaxes(origin: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return MuJoCo xyaxes and the normalized optical axis."""
    optical = np.asarray(target, dtype=np.float64) - np.asarray(origin, dtype=np.float64)
    optical /= np.linalg.norm(optical)
    z_axis = -optical
    x_axis = np.cross(np.array([0.0, 0.0, 1.0]), z_axis)
    if np.linalg.norm(x_axis) < 1e-9:
        x_axis = np.array([1.0, 0.0, 0.0])
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    return np.concatenate((x_axis, y_axis)), optical


def _xyaxes_to_quat(xyaxes: np.ndarray) -> np.ndarray:
    x_axis, y_axis = xyaxes[:3], xyaxes[3:]
    z_axis = np.cross(x_axis, y_axis)
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    quat = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, rotation.ravel())
    return quat


def set_camera(
    model: mujoco.MjModel,
    camera: int,
    *,
    pos: np.ndarray,
    target: np.ndarray,
    fovy: float,
) -> np.ndarray:
    xyaxes, optical = look_at_xyaxes(pos, target)
    model.cam_pos[camera] = pos
    model.cam_quat[camera] = _xyaxes_to_quat(xyaxes)
    model.cam_fovy[camera] = fovy
    return optical


def project(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera: int,
    point: np.ndarray,
    width: int,
    height: int,
) -> tuple[np.ndarray, float]:
    rotation = data.cam_xmat[camera].reshape(3, 3)
    local = rotation.T @ (point - data.cam_xpos[camera])
    depth = -float(local[2])
    if depth <= 0.0:
        return np.array([np.nan, np.nan]), depth
    scale = np.tan(np.deg2rad(model.cam_fovy[camera]) / 2.0)
    x_ndc = local[0] / (depth * scale * width / height)
    y_ndc = local[1] / (depth * scale)
    return np.array([(x_ndc + 1.0) * width / 2.0, (1.0 - y_ndc) * height / 2.0]), depth


def palm_local_point(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    world_point: np.ndarray,
) -> np.ndarray:
    palm = _id(model, mujoco.mjtObj.mjOBJ_BODY, PALM_BODY)
    rotation = data.xmat[palm].reshape(3, 3)
    return rotation.T @ (world_point - data.xpos[palm])


def interaction_target_from_fk(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    frames: dict[str, int],
    arrays: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
) -> np.ndarray:
    """Estimate grasp volume from palm, opposing digits, and object FK.

    At the recorded approach state, digit distal-link origins and the primary
    object centre are transformed into the palm frame.  Their two centroids
    define opposing sides of the open grasp; the midpoint is therefore the
    virtual grasp-volume centre.  Closure and opposition samples are retained
    for diagnostics but do not drag the target toward a displaced object.
    """
    timestamps, qpos, mocap_pos, mocap_quat = arrays
    object_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, OBJECT_BODY)
    digit_ids = [_id(model, mujoco.mjtObj.mjOBJ_BODY, body) for body in TIP_BODIES.values()]
    index = frames["interaction"]
    _restore_state(model, data, qpos[index], mocap_pos[index], mocap_quat[index], timestamps[index])
    digit_local = np.stack([palm_local_point(model, data, data.xpos[body]) for body in digit_ids])
    object_local = palm_local_point(model, data, data.xpos[object_id])
    open_grasp_center = 0.5 * (object_local + np.mean(digit_local, axis=0))
    # palm_collision is centred at (0, -0.03, 0.05) in this same frame.
    # Blending that physical palm-interior anchor with the open grasp centre
    # keeps both the cavity and the object volume in view.
    palm_interior = np.array([0.0, -0.03, 0.05], dtype=np.float64)
    return 0.60 * open_grasp_center + 0.40 * palm_interior


def print_fk_samples(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    frames: dict[str, int],
    arrays: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
) -> None:
    """Print the palm-local evidence used to define the grasp volume."""
    timestamps, qpos, mocap_pos, mocap_quat = arrays
    object_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, OBJECT_BODY)
    for tag in ("interaction", "grasp_closure", "thumb_opposition"):
        index = frames[tag]
        _restore_state(model, data, qpos[index], mocap_pos[index], mocap_quat[index], timestamps[index])
        print(f"{tag} frame={index}")
        print(f"  object {palm_local_point(model, data, data.xpos[object_id]).tolist()}")
        for label, body_name in TIP_BODIES.items():
            body = _id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            print(f"  {label:>6} {palm_local_point(model, data, data.xpos[body]).tolist()}")


def camera_points(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera: int,
    width: int,
    height: int,
) -> dict[str, tuple[np.ndarray, float]]:
    points: dict[str, np.ndarray] = {}
    palm = _id(model, mujoco.mjtObj.mjOBJ_BODY, PALM_BODY)
    points["palm"] = data.xpos[palm].copy()
    target_site = _id(model, mujoco.mjtObj.mjOBJ_SITE, "wrist_camera_interaction_target")
    points["grasp"] = data.site_xpos[target_site].copy()
    for label, body_name in TIP_BODIES.items():
        points[label] = data.xpos[_id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)].copy()
    points["object"] = data.xpos[_id(model, mujoco.mjtObj.mjOBJ_BODY, OBJECT_BODY)].copy()
    return {label: project(model, data, camera, point, width, height) for label, point in points.items()}


def annotate(path: Path, pixels: np.ndarray, values: dict[str, tuple[np.ndarray, float]]) -> None:
    colors = {
        "palm": "yellow", "grasp": "magenta", "thumb": "red",
        "index": "lime", "middle": "cyan", "little": "orange",
        "object": "white",
    }
    image = Image.fromarray(pixels)
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    width, height = image.size
    draw.line((width / 2 - 12, height / 2, width / 2 + 12, height / 2), fill="white", width=2)
    draw.line((width / 2, height / 2 - 12, width / 2, height / 2 + 12), fill="white", width=2)
    text_y = 8
    for label, (pixel, depth) in values.items():
        color = colors[label]
        if np.all(np.isfinite(pixel)):
            x, y = map(float, pixel)
            visible = 0.0 <= x < width and 0.0 <= y < height
            x = min(max(x, 8.0), width - 8.0)
            y = min(max(y, 8.0), height - 8.0)
            marker = label if visible else f"{label} (off-frame centre)"
            draw.ellipse(
                (x - 6, y - 6, x + 6, y + 6),
                outline=color,
                width=3,
            )
            draw.text((x + 8, y - 7), marker, fill=color, font=font)
        draw.text((8, text_y), f"{label:>6}: depth={depth:.4f} m pixel=({pixel[0]:.1f},{pixel[1]:.1f})", fill=color, font=font)
        text_y += 14
    image.save(path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=DEFAULT_RECORDING)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--candidate",
        choices=("initial", "W1", "W2", "W3", "production"),
        default="production",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--print-fk-samples", action="store_true")
    args = parser.parse_args()

    with np.load(args.input, allow_pickle=False) as archive:
        scene = str(np.asarray(archive["scene_path"]).item())
        timestamps = np.asarray(archive["timestamps"], dtype=np.float64)
        qpos = np.asarray(archive["qpos"], dtype=np.float64)
        mocap_pos = np.asarray(archive["mocap_pos"], dtype=np.float64)
        mocap_quat = np.asarray(archive["mocap_quat"], dtype=np.float64)
    model = mujoco.MjModel.from_xml_path(str(_resolve_scene(scene, ROOT)))
    _validate_recording_arrays(model, timestamps, qpos, mocap_pos, mocap_quat)
    data = mujoco.MjData(model)
    frames = _select_frames(model, data, qpos, mocap_pos, mocap_quat, timestamps)
    arrays = (timestamps, qpos, mocap_pos, mocap_quat)
    target = interaction_target_from_fk(model, data, frames, arrays)
    if args.print_fk_samples:
        print_fk_samples(model, data, frames, arrays)
    camera = _id(model, mujoco.mjtObj.mjOBJ_CAMERA, WRIST_CAMERA)
    # Preserve the user-approved initial v6 mount ray (0.020,-0.190,0.000)
    # and interaction target.  Only its radius changes: -2/-3/-4 cm.
    candidates = {
        "initial": (np.array([0.020000, -0.190000, 0.000]), 75.0),
        "W1": (np.array([0.017906, -0.170104, 0.000]), 75.0),
        "W2": (np.array([0.016859, -0.160165, 0.000]), 75.0),
        "W3": (np.array([0.015812, -0.150226, 0.000]), 75.0),
    }
    if args.candidate != "production":
        pos, fovy = candidates[args.candidate]
        xyaxes, optical = look_at_xyaxes(pos, target)
        quat = _xyaxes_to_quat(xyaxes)
        camera_head = _id(
            model, mujoco.mjtObj.mjOBJ_BODY, "wrist_camera_head"
        )
        model.body_pos[camera_head] = pos
        model.cam_pos[camera] = 0.0
        model.cam_quat[camera] = quat
        model.cam_fovy[camera] = fovy
        for geom_name in ("wrist_camera_visual", "wrist_camera_collision"):
            geom = _id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            model.geom_pos[geom] = -0.020 * optical
            model.geom_quat[geom] = quat
        # The production extension capsule still encodes the current MJCF
        # endpoint.  Hide it for optical candidate evaluation rather than
        # rendering a knowingly mismatched bracket through the test view.
        extension = _id(
            model, mujoco.mjtObj.mjOBJ_GEOM,
            "wrist_camera_extension_visual",
        )
        model.geom_rgba[extension, 3] = 0.0
        target_site = _id(model, mujoco.mjtObj.mjOBJ_SITE, "wrist_camera_interaction_target")
        model.site_pos[target_site] = target

    args.output_dir.mkdir(parents=True, exist_ok=True)
    with mujoco.Renderer(model, height=args.height, width=args.width) as renderer:
        for tag in ("home", "mid", "interaction", "grasp_closure", "thumb_opposition"):
            index = frames[tag]
            _restore_state(model, data, qpos[index], mocap_pos[index], mocap_quat[index], timestamps[index])
            renderer.update_scene(data, camera=camera)
            pixels = np.ascontiguousarray(renderer.render())
            path = args.output_dir / f"{args.candidate}_{tag}.png"
            Image.fromarray(pixels).save(path)
            if tag == "interaction":
                values = camera_points(model, data, camera, args.width, args.height)
                annotate(args.output_dir / f"{args.candidate}_interaction_projection.png", pixels, values)
                print(f"{args.candidate} target={target.tolist()} camera={model.cam_pos[camera].tolist()} fovy={model.cam_fovy[camera]:.1f}")
                for label, (pixel, depth) in values.items():
                    print(f"  {label:>6}: depth={depth:.6f} pixel=({pixel[0]:.2f},{pixel[1]:.2f})")


if __name__ == "__main__":
    main()
