"""Render demo-scene preview PNGs from each scene's demo_ready
keyframe and from scripted task-representative states.

Part of the demo scene suite deliverables (preview requirements).
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


def _qadr(model, joint):
    jid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, joint
    )
    return int(model.jnt_qposadr[jid])


def _render_set(model, data, renderer, out_dir: Path, tag: str):
    out_dir.mkdir(parents=True, exist_ok=True)
    for cam in CAMERAS:
        renderer.update_scene(data, camera=cam)
        img = renderer.render()
        if cam == "workstation_front":
            name = f"workstation_front_{tag}.png"
        elif cam == "wrist_camera":
            name = f"wrist_{tag}.png"
        else:
            name = f"demo_{tag}.png"
        Image.fromarray(img).save(out_dir / name)
    print(f"  wrote {tag}: 3 cameras")


def _demo01(renderer):
    xml = MJCF / "rm65_sg100_demo_hammer_peg_assembly_scene.xml"
    out = OUT_ROOT / "hammer_peg_assembly"
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    layout = resolve_robot_layout(model)
    key = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_KEY, "demo_ready"
    )
    hammer_j = _qadr(model, "demo_hammer_free")
    peg_q = _qadr(model, "demo_peg_slide")

    def load(arm=None):
        mujoco.mj_resetDataKeyframe(model, data, key)
        data.ctrl[layout.arm_actuator_ids] = (
            arm if arm is not None else READY_ARM
        )
        data.ctrl[layout.hand_actuator_ids] = 0.0

    # READY
    load()
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "ready")

    # hammer lifted into a power grasp (hand closed onto the handle,
    # hammer raised mid-air) - hand joints closed for the visual
    load()
    # place hand over the handle grasp point via arm q from the IK
    # audit (approximate representative posture), fingers flexed
    data.qpos[layout.hand_qpos_indices] = np.array(
        [0.55, -0.75, 0.45, 0.9, 1.35, 0.5,
         -0.7, 0.9, 1.0, -0.35, 0.8]
    )
    data.qpos[hammer_j:hammer_j + 7] = [
        0.34, -0.129, 0.20, 0.999809, 0.0, -0.019533, 0.0,
    ]
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "hammer_grasp")

    # pre-strike: mallet face aligned above the peg
    load()
    data.qpos[hammer_j:hammer_j + 7] = [
        0.415, 0.209, 0.205, 1, 0, 0, 0,
    ]
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "hammer_prestrike")

    # strike contact
    load()
    data.qpos[hammer_j:hammer_j + 7] = [
        0.415, 0.209, 0.152, 1, 0, 0, 0,
    ]
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "hammer_strike")

    # peg before / after (full insertion)
    load()
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "peg_before")
    data.qpos[peg_q] = 0.040
    data.qpos[hammer_j:hammer_j + 7] = [
        0.34, -0.129, 0.05, 0.999809, 0.0, -0.019533, 0.0,
    ]
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "peg_after")


def _demo02(renderer):
    xml = (
        MJCF
        / "rm65_sg100_demo_drawer_retrieval_and_loading_scene.xml"
    )
    out = OUT_ROOT / "drawer_retrieval_and_loading"
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    layout = resolve_robot_layout(model)
    key = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_KEY, "demo_ready"
    )
    drawer_q = _qadr(model, "demo_drawer_slide")
    item_j = _qadr(model, "demo_drawer_item_free")

    def load():
        mujoco.mj_resetDataKeyframe(model, data, key)
        data.ctrl[layout.arm_actuator_ids] = READY_ARM
        data.ctrl[layout.hand_actuator_ids] = 0.0

    load()
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "ready")
    _render_set(model, data, renderer, out, "drawer_closed")

    # open
    data.qpos[drawer_q] = 0.15
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "drawer_open")

    # object grasped out of the open drawer (lifted mid-air)
    data.qpos[item_j:item_j + 7] = [
        0.45, 0.18, 0.16, 1, 0, 0, 0,
    ]
    data.qpos[layout.hand_qpos_indices] = np.array(
        [0.55, -0.75, 0.45, 0.9, 1.35, 0.5,
         -0.7, 0.9, 1.0, -0.35, 0.8]
    )
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "drawer_object_grasp")

    # object in tray
    data.qpos[item_j:item_j + 7] = [
        0.33, -0.22, 0.0345, 1, 0, 0, 0,
    ]
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "tray_goal")


def _demo03(renderer):
    xml = (
        MJCF / "rm65_sg100_demo_maintenance_kit_packing_scene.xml"
    )
    out = OUT_ROOT / "maintenance_kit_packing"
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    layout = resolve_robot_layout(model)
    key = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_KEY, "demo_ready"
    )
    qa = _qadr(model, "kit_object_a_free")
    qb = _qadr(model, "kit_object_b_free")
    qc = _qadr(model, "kit_object_c_free")

    closed = np.array(
        [0.55, -0.75, 0.45, 0.9, 1.35, 0.5,
         -0.7, 0.9, 1.0, -0.35, 0.8]
    )

    def load():
        mujoco.mj_resetDataKeyframe(model, data, key)
        data.ctrl[layout.arm_actuator_ids] = READY_ARM
        data.ctrl[layout.hand_actuator_ids] = 0.0

    load()
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "ready")
    _render_set(model, data, renderer, out, "kit_initial")

    # A grasped (lifted), B/C still on table
    load()
    data.qpos[qa:qa + 7] = [0.31, 0.30, 0.16, 1, 0, 0, 0]
    data.qpos[layout.hand_qpos_indices] = closed
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "object_a_grasp")

    # A packed, B grasped
    load()
    data.qpos[qa:qa + 7] = [0.32, -0.12, 0.0105, 1, 0, 0, 0]
    data.qpos[qb:qb + 7] = [0.41, 0.32, 0.16, 1, 0, 0, 0]
    data.qpos[layout.hand_qpos_indices] = closed
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "object_b_grasp")

    # A + B packed, C grasped (rotated toward slot axis)
    load()
    data.qpos[qa:qa + 7] = [0.32, -0.12, 0.0105, 1, 0, 0, 0]
    data.qpos[qb:qb + 7] = [
        0.515, -0.115, 0.0105, 1, 0, 0, 0,
    ]
    data.qpos[qc:qc + 7] = [
        0.42, -0.295, 0.13, 1, 0, 0, 0,
    ]
    data.qpos[layout.hand_qpos_indices] = closed
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "object_c_grasp")

    # complete kit
    load()
    data.qpos[qa:qa + 7] = [0.32, -0.12, 0.0105, 1, 0, 0, 0]
    data.qpos[qb:qb + 7] = [
        0.515, -0.115, 0.0105, 1, 0, 0, 0,
    ]
    data.qpos[qc:qc + 7] = [
        0.42, -0.295, 0.0071, 1, 0, 0, 0,
    ]
    mujoco.mj_forward(model, data)
    _render_set(model, data, renderer, out, "kit_complete")


def main() -> int:
    # any scene compiles the renderer target; use DEMO 01
    xml = MJCF / "rm65_sg100_demo_hammer_peg_assembly_scene.xml"
    model = mujoco.MjModel.from_xml_path(str(xml))
    renderer = mujoco.Renderer(model, HEIGHT, WIDTH)

    print("DEMO 01 - hammer_peg_assembly")
    _demo01(renderer)
    print("DEMO 02 - drawer_retrieval_and_loading")
    _demo02(renderer)
    print("DEMO 03 - maintenance_kit_packing")
    _demo03(renderer)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
