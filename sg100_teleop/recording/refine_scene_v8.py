"""Render and audit the production-v8 MuJoCo workstation scene.

The committed reference recording predates scene simplification and has four
additional free objects.  Its first 17 qpos entries are nevertheless the
unchanged RM65+SG100 robot contract.  This tool intentionally replays only
those robot coordinates and keeps the three v8 task objects at their authored
poses.  It therefore validates the frozen camera trajectory without reviving
objects removed from the active scene.

Run from the repository root::

    python -m sg100_teleop.recording.refine_scene_v8
"""

from __future__ import annotations

import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SCENE = (
    ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/"
    "rm65_sg100_avp_grasp_scene.xml"
)
RECORDING = ROOT / "recording/20260820_181024/mujoco_recording.npz"
OUTPUT = ROOT / "recording/scene_preview_v8"
ROBOT_NQ = 17
EXPECTED_REFERENCE_FRAMES = 3129
WIDTH = 1280
HEIGHT = 720


def _restore_robot_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    legacy_qpos: np.ndarray,
    timestamp: float,
) -> None:
    """Restore the unchanged robot prefix and the authored v8 prop poses."""
    if legacy_qpos.ndim != 1 or legacy_qpos.size < ROBOT_NQ:
        raise ValueError(f"legacy qpos must contain at least {ROBOT_NQ} values")
    data.qpos[:] = model.qpos0
    data.qpos[:ROBOT_NQ] = legacy_qpos[:ROBOT_NQ]
    data.qvel[:] = 0.0
    data.time = float(timestamp)
    mujoco.mj_forward(model, data)


def _body_id(model: mujoco.MjModel, name: str) -> int:
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body < 0:
        raise ValueError(f"body {name!r} not found")
    return body


def _select_frames(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos: np.ndarray,
    timestamps: np.ndarray,
) -> dict[str, int]:
    palm = _body_id(model, "sg100_base_link")
    objects = [_body_id(model, name) for name in ("obj_coke", "obj_apple", "obj_mug")]
    crate = _body_id(model, "drop_bin")
    palm_positions = np.empty((qpos.shape[0], 3), dtype=np.float64)
    object_distances = np.empty(qpos.shape[0], dtype=np.float64)
    crate_distances = np.empty(qpos.shape[0], dtype=np.float64)
    for index in range(qpos.shape[0]):
        _restore_robot_state(model, data, qpos[index], timestamps[index])
        position = data.xpos[palm].copy()
        palm_positions[index] = position
        object_distances[index] = min(
            np.linalg.norm(position - data.xpos[body]) for body in objects
        )
        crate_distances[index] = np.linalg.norm(position - data.xpos[crate])
    return {
        "home": 0,
        "mid": qpos.shape[0] // 2,
        "interaction": int(np.argmin(object_distances)),
        "far_reach": int(np.argmax(np.linalg.norm(palm_positions[:, :2], axis=1))),
        "grasp": 2809 if qpos.shape[0] > 2809 else int(np.argmin(object_distances)),
        "container": int(np.argmin(crate_distances)),
    }


def _demo_coverage(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos: np.ndarray,
    timestamps: np.ndarray,
) -> dict[str, float | int]:
    camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "demo_camera")
    if camera < 0:
        raise ValueError("demo_camera not found")
    geom_ids = []
    for geom in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or ""
        if name != "workstation_floor_visual":
            geom_ids.append(geom)

    worst_extent = -np.inf
    worst_frame = -1
    worst_x = 0.0
    worst_y = 0.0
    covered_frames = 0
    tan_half = np.tan(np.deg2rad(model.cam_fovy[camera]) / 2.0)
    aspect = WIDTH / HEIGHT
    for index in range(qpos.shape[0]):
        _restore_robot_state(model, data, qpos[index], timestamps[index])
        rotation = data.cam_xmat[camera].reshape(3, 3)
        delta = data.geom_xpos[geom_ids] - data.cam_xpos[camera]
        points = delta @ rotation
        depth = -points[:, 2]
        if np.any(depth <= 0.0):
            raise RuntimeError(f"scene geometry behind demo_camera at frame {index}")
        radii = model.geom_rbound[geom_ids]
        x_extent = np.abs(points[:, 0]) / (depth * tan_half * aspect)
        x_extent += radii / (depth * tan_half * aspect)
        y_extent = np.abs(points[:, 1]) / (depth * tan_half)
        y_extent += radii / (depth * tan_half)
        extent = max(float(np.max(x_extent)), float(np.max(y_extent)))
        covered_frames += int(extent <= 1.0)
        if extent > worst_extent:
            worst_extent = extent
            worst_frame = index
            worst_x = float(np.max(x_extent))
            worst_y = float(np.max(y_extent))
    return {
        "scanned_frames": int(qpos.shape[0]),
        "covered_frames": covered_frames,
        "coverage_fraction": covered_frames / int(qpos.shape[0]),
        "worst_frame": worst_frame,
        "worst_x_extent_ndc": worst_x,
        "worst_y_extent_ndc": worst_y,
        "minimum_conservative_margin": 1.0 - worst_extent,
    }


def _cup_handle_audit(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, object]:
    data.qpos[:] = model.qpos0
    mujoco.mj_forward(model, data)
    mug = _body_id(model, "obj_mug")
    handle_xy = data.xmat[mug].reshape(3, 3)[:2, 1]
    handle_xy /= np.linalg.norm(handle_xy)
    palm_xy = np.array([0.25048434, 0.11727796]) - data.xpos[mug, :2]
    palm_xy /= np.linalg.norm(palm_xy)
    angle = np.rad2deg(np.arccos(np.clip(np.dot(handle_xy, palm_xy), -1.0, 1.0)))
    return {
        "mesh_handle_local_axis": "+Y",
        "handle_world_xy": handle_xy.tolist(),
        "cup_to_ready_palm_xy": palm_xy.tolist(),
        "alignment_error_degrees": float(angle),
        "upright_z_dot": float(data.xmat[mug].reshape(3, 3)[2, 2]),
    }


def _render(
    renderer: mujoco.Renderer,
    data: mujoco.MjData,
    camera: str,
    path: Path,
) -> dict[str, float]:
    renderer.update_scene(data, camera=camera)
    frame = np.ascontiguousarray(renderer.render()).copy()
    Image.fromarray(frame).save(path)
    return {
        "mean": float(frame.mean()),
        "variance": float(frame.var()),
        "near_white_fraction": float(np.mean(np.all(frame >= 250, axis=2))),
        "near_black_fraction": float(np.mean(np.all(frame <= 5, axis=2))),
    }


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    with np.load(RECORDING, allow_pickle=False) as archive:
        timestamps = np.asarray(archive["timestamps"], dtype=np.float64)
        qpos = np.asarray(archive["qpos"], dtype=np.float64)
    if timestamps.shape != (EXPECTED_REFERENCE_FRAMES,) or qpos.shape[0] != timestamps.size:
        raise ValueError(
            f"expected {EXPECTED_REFERENCE_FRAMES} reference frames, got {timestamps.size}"
        )
    if not np.all(np.isfinite(qpos)) or not np.all(np.isfinite(timestamps)):
        raise ValueError("reference trajectory contains non-finite values")

    OUTPUT.mkdir(parents=True, exist_ok=True)
    frames = _select_frames(model, data, qpos, timestamps)
    image_stats: dict[str, dict[str, float]] = {}
    jobs = (
        ("head_home_v8.png", "workstation_front", "home"),
        ("head_interaction_v8.png", "workstation_front", "interaction"),
        ("head_container_v8.png", "workstation_front", "container"),
        ("wrist_home_v8.png", "wrist_camera", "home"),
        ("wrist_interaction_v8.png", "wrist_camera", "interaction"),
        ("wrist_grasp_v8.png", "wrist_camera", "grasp"),
        ("demo_home_v8.png", "demo_camera", "home"),
        ("demo_mid_v8.png", "demo_camera", "mid"),
        ("demo_far_reach_v8.png", "demo_camera", "far_reach"),
        ("demo_container_v8.png", "demo_camera", "container"),
    )
    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        for filename, camera, tag in jobs:
            index = frames[tag]
            _restore_robot_state(model, data, qpos[index], timestamps[index])
            image_stats[filename] = _render(renderer, data, camera, OUTPUT / filename)

        _restore_robot_state(model, data, qpos[frames["home"]], timestamps[frames["home"]])
        image_stats["cup_handle_orientation_v8.png"] = _render(
            renderer, data, "workstation_front", OUTPUT / "cup_handle_orientation_v8.png"
        )

        apple_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "obj_apple_free")
        apple_qpos = int(model.jnt_qposadr[apple_joint])
        data.qpos[apple_qpos : apple_qpos + 7] = [0.55, -0.15, 0.016, 1, 0, 0, 0]
        mujoco.mj_forward(model, data)
        image_stats["crate_top_visibility_v8.png"] = _render(
            renderer, data, "demo_camera", OUTPUT / "crate_top_visibility_v8.png"
        )

    report = {
        "scene": str(SCENE.relative_to(ROOT)),
        "model_dimensions": {"nq": model.nq, "nv": model.nv, "nu": model.nu},
        "frames": frames,
        "demo_coverage": _demo_coverage(model, data, qpos, timestamps),
        "cup_handle": _cup_handle_audit(model, data),
        "image_statistics": image_stats,
    }
    (OUTPUT / "coverage_report_v8.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
