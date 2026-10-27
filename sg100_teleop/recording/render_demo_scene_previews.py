"""Render demo-scene preview PNGs from each scene's demo_ready
keyframe and from scripted task-representative states (V2).

Renders are pure mj_forward visualisation - no physics is re-run
for the keyframe views; the task-representative views teleport the
task objects to waypoint poses (documented in DEMO_SCENES.md).

Run::

    conda run -n mujoco-learn python -m \\
        sg100_teleop.recording.render_demo_scene_previews
"""

from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image

from sg100_teleop.sim.combined_robot_layout import (
    resolve_robot_layout,
)

ROOT = Path(__file__).resolve().parents[2]
MJCF = ROOT / "sg100_teleop/assets/rm65_sg100/mjcf"
OUT_ROOT = ROOT / "recording/demo_scene_previews"

WIDTH, HEIGHT = 1280, 720
CAMERAS = ("workstation_front", "wrist_camera", "demo_camera")

READY_ARM = np.array(
    [
        0.34818753987135775,
        -1.3614850477620644,
        -2.286025521790937,
        -0.10246395780782178,
        2.0744881064029075,
        -0.049588596290682636,
    ]
)
HAND_CLOSED = np.array(
    [0.55, -0.75, 0.45, 0.9, 1.35, 0.5, -0.7, 0.9, 1.0, -0.35, 0.8]
)
# V2 settled hammer pose on the cradle (== keyframe == body pos)
HAMMER_REST = [
    0.339998, -0.119996, 0.038549,
    0.999958, -0.000257, -0.009136, -0.0000176,
]


def _qadr(model, joint):
    jid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, joint
    )
    return int(model.jnt_qposadr[jid])


def _render_set(model, data, renderer, out_dir: Path, tag: str,
                cameras=CAMERAS):
    out_dir.mkdir(parents=True, exist_ok=True)
    for cam in cameras:
        renderer.update_scene(data, camera=cam)
        img = renderer.render()
        if cam == "workstation_front":
            name = f"workstation_front_{tag}.png"
        elif cam == "wrist_camera":
            name = f"wrist_{tag}.png"
        else:
            name = f"demo_{tag}.png"
        Image.fromarray(img).save(out_dir / name)
    print(f"  wrote {tag}")


def _load(xml):
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    layout = resolve_robot_layout(model)
    key = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_KEY, "demo_ready"
    )

    def load():
        mujoco.mj_resetDataKeyframe(model, data, key)
        data.ctrl[layout.arm_actuator_ids] = READY_ARM
        data.ctrl[layout.hand_actuator_ids] = 0.0
        return data

    return model, data, layout, load


def _demo01(renderer):
    model, data, layout, load = _load(
        MJCF / "rm65_sg100_demo_hammer_peg_assembly_scene.xml"
    )
    out = OUT_ROOT / "hammer_peg_assembly"
    hammer_j = _qadr(model, "demo_hammer_free")
    peg_q = _qadr(model, "demo_peg_slide")

    load(); mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "ready")

    # raised power grasp on the cradle handle
    load()
    data.qpos[layout.hand_qpos_indices] = HAND_CLOSED
    data.qpos[hammer_j:hammer_j + 7] = HAMMER_REST
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "hammer_grasp")

    # pre-strike above the peg, then strike contact
    load()
    data.qpos[hammer_j:hammer_j + 7] = [
        0.415, 0.209, 0.205, 1, 0, 0, 0,
    ]
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "hammer_prestrike")
    data.qpos[hammer_j + 2] = 0.152
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "hammer_strike")

    # peg before / after (full insertion)
    load(); mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "peg_before")
    data.qpos[peg_q] = 0.040
    data.qpos[hammer_j:hammer_j + 7] = HAMMER_REST
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "peg_after")


def _demo02(renderer):
    model, data, layout, load = _load(
        MJCF
        / "rm65_sg100_demo_drawer_loading_and_closing_scene.xml"
    )
    out = OUT_ROOT / "drawer_loading_and_closing"
    drawer_q = _qadr(model, "demo_drawer_slide")
    item_j = _qadr(model, "demo_drawer_item_free")

    # closed + ready, then hook grasp on the handle
    load(); mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "drawer_closed_ready")
    data.qpos[layout.hand_qpos_indices] = HAND_CLOSED
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "drawer_handle_grasp")

    # fully open
    data.qpos[drawer_q] = 0.15
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "drawer_open")

    # tape grasped on the table (hand closed over it)
    data.qpos[layout.hand_qpos_indices] = HAND_CLOSED
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "table_object_grasp")

    # tape lowered into the open drawer
    data.qpos[item_j:item_j + 7] = [
        0.47, 0.18, 0.060, 1, 0, 0, 0,
    ]
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "drawer_loading")
    data.qpos[item_j:item_j + 3] = [0.47, 0.18, 0.0405]
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "drawer_loaded")

    # closed with the tape inside
    data.qpos[drawer_q] = 0.0
    data.qpos[item_j:item_j + 3] = [0.65, 0.18, 0.0405]
    data.qpos[layout.hand_qpos_indices] = 0
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "drawer_closed_final")


def _demo03(renderer):
    model, data, layout, load = _load(
        MJCF / "rm65_sg100_demo_maintenance_kit_packing_scene.xml"
    )
    out = OUT_ROOT / "maintenance_kit_packing"
    qa = _qadr(model, "kit_object_a_free")
    qb = _qadr(model, "kit_object_b_free")

    load(); mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "ready")
    _render_set(model, data, renderer, out, "maintenance_initial")

    # box grasped mid-air
    load()
    data.qpos[qa:qa + 7] = [0.44, 0.34, 0.16, 1, 0, 0, 0]
    data.qpos[layout.hand_qpos_indices] = HAND_CLOSED
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "box_grasp")

    # box packed, screwdriver grasped
    load()
    data.qpos[qa:qa + 7] = [0.32, -0.12, 0.0105, 1, 0, 0, 0]
    data.qpos[qb:qb + 7] = [0.54, 0.20, 0.16, 1, 0, 0, 0]
    data.qpos[layout.hand_qpos_indices] = HAND_CLOSED
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "screwdriver_grasp")

    # box packed, screwdriver mid-carry rotated toward the slot
    load()
    data.qpos[qa:qa + 7] = [0.32, -0.12, 0.0105, 1, 0, 0, 0]
    data.qpos[qb:qb + 7] = [0.44, 0.05, 0.20, 1, 0, 0, 0]
    data.qpos[layout.hand_qpos_indices] = HAND_CLOSED
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "screwdriver_reorient")

    # complete kit
    load()
    data.qpos[qa:qa + 7] = [0.32, -0.12, 0.0105, 1, 0, 0, 0]
    data.qpos[qb:qb + 7] = [0.42, -0.295, 0.0071, 1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "maintenance_complete")


def main() -> int:
    xml = MJCF / "rm65_sg100_demo_hammer_peg_assembly_scene.xml"
    model = mujoco.MjModel.from_xml_path(str(xml))
    renderer = mujoco.Renderer(model, HEIGHT, WIDTH)

    print("DEMO 01 V2 - hammer_peg_assembly")
    _demo01(renderer)
    print("DEMO 02 V2 - drawer_loading_and_closing")
    _demo02(renderer)
    print("DEMO 03 V2 - maintenance_kit_packing")
    _demo03(renderer)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
