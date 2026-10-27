"""Generate per-demo initial-pose manifests (DEMO_INITIAL_STATE).

For each demo scene, loads the XML, applies the demo_ready
keyframe (the single source of task initial state), runs
mj_forward, and records the world pose of every streamed body -
robot base, SG100, dynamic task objects, drawer moving body, peg
body, fixtures - into::

    recording/demo_scene_previews/<task>/initial_pose_manifest.json

The USDZ export consumes the SAME keyframe, so this manifest is
also the pre-export reference table for the MJCF <-> USDZ
transform comparison.

Run::

    conda run -n mujoco-learn python -m \\
        sg100_teleop.recording.generate_demo_initial_pose_manifest
"""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
MJCF = ROOT / "sg100_teleop/assets/rm65_sg100/mjcf"
OUT_ROOT = ROOT / "recording/demo_scene_previews"

DEMO_TASKS = {
    "01": {
        "scene": "rm65_sg100_demo_hammer_peg_assembly_scene.xml",
        "folder": "hammer_peg_assembly",
        "bodies": [
            "base_link", "link_6", "sg100_base_link",
            "demo_hammer", "demo_peg", "peg_fixture",
            "hammer_tool_rest", "grasp_table",
        ],
    },
    "02": {
        "scene": (
            "rm65_sg100_demo_drawer_loading_and_closing_scene.xml"
        ),
        "folder": "drawer_loading_and_closing",
        "bodies": [
            "base_link", "link_6", "sg100_base_link",
            "drawer_cabinet", "demo_drawer",
            "demo_drawer_item", "grasp_table",
        ],
    },
    "03": {
        "scene": (
            "rm65_sg100_demo_maintenance_kit_packing_scene.xml"
        ),
        "folder": "maintenance_kit_packing",
        "bodies": [
            "base_link", "link_6", "sg100_base_link",
            "kit_object_a", "kit_object_b",
            "kit_tray", "grasp_table",
        ],
    },
}


def build_manifest(demo: str) -> dict:
    task = DEMO_TASKS[demo]
    xml = MJCF / task["scene"]
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_KEY, "demo_ready"
    )
    assert key >= 0
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)

    bodies = {}
    for name in task["bodies"]:
        bid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, name
        )
        assert bid >= 0, f"{demo}: body missing: {name}"
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(
            quat, data.xmat[bid].reshape(9)
        )
        bodies[name] = {
            "position_m": [
                round(float(v), 6) for v in data.xpos[bid]
            ],
            "quaternion_wxyz": [
                round(float(v), 6) for v in quat
            ],
        }

    return {
        "demo": demo,
        "scene": task["scene"],
        "state": "demo_ready keyframe (== XML-default task state)",
        "bodies": bodies,
    }


def main() -> int:
    for demo, task in sorted(DEMO_TASKS.items()):
        manifest = build_manifest(demo)
        out_dir = OUT_ROOT / task["folder"]
        out_dir.mkdir(parents=True, exist_ok=True)
        out = out_dir / "initial_pose_manifest.json"
        out.write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"demo {demo}: {out} ({len(manifest['bodies'])} bodies)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
