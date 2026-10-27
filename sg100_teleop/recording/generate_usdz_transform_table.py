"""MJCF <-> USDZ initial-transform table (V2 PART 68).

For every streamed body of each demo scene, prints and writes a
markdown table comparing:

  * MJCF side  - the ``demo_ready`` keyframe world pose (the same
    state the exporter consumes and the same state recorded in
    ``initial_pose_manifest.json``);
  * USDZ side  - the packaged Xform's local-to-world transform read
    back from the USD stage.

Acceptance mirrors the V2 contract test: translation <= 1 mm,
rotation <= 0.5 deg.

Run (needs ``pxr``, so use the USDZ venv)::

    .venv_usdz/bin/python -m \\
        sg100_teleop.recording.generate_usdz_transform_table
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ASSET_ROOT = ROOT / "sg100_teleop/assets/rm65_sg100"
MJCF = ASSET_ROOT / "mjcf"
OUT_ROOT = ROOT / "recording/demo_scene_previews"

RM65_BODIES = (
    "base_link", "link_1", "link_2", "link_3",
    "link_4", "link_5", "link_6",
)
SG100_BODIES = (
    "sg100_base_link", "thumb_link1", "thumb_link2", "thumb_link3",
    "index_link1", "index_link2", "index_link3", "middle_link1",
    "middle_link2", "little_link1", "little_llink2",
    "little_linik3",
)

DEMO_SUITES = {
    "01": {
        "xml": MJCF
        / "rm65_sg100_demo_hammer_peg_assembly_scene.xml",
        "usdz": ASSET_ROOT
        / "usdz/rm65_sg100_demo_hammer_peg_assembly_compat.usdz",
        "folder": "hammer_peg_assembly",
        "task_bodies": (
            "hammer_tool_rest", "demo_hammer",
            "peg_fixture", "demo_peg", "grasp_table",
        ),
    },
    "02": {
        "xml": (
            MJCF
            / "rm65_sg100_demo_drawer_loading_and_closing_scene.xml"
        ),
        "usdz": (
            ASSET_ROOT
            / "usdz/"
            "rm65_sg100_demo_drawer_loading_and_closing_compat.usdz"
        ),
        "folder": "drawer_loading_and_closing",
        "task_bodies": (
            "drawer_cabinet", "demo_drawer",
            "demo_drawer_item", "grasp_table",
        ),
    },
    "03": {
        "xml": (
            MJCF
            / "rm65_sg100_demo_maintenance_kit_packing_scene.xml"
        ),
        "usdz": (
            ASSET_ROOT
            / "usdz/"
            "rm65_sg100_demo_maintenance_kit_packing_compat.usdz"
        ),
        "folder": "maintenance_kit_packing",
        "task_bodies": (
            "kit_object_a", "kit_object_b",
            "kit_tray", "grasp_table",
        ),
    },
}


def _quat_error_deg(q_a, q_b) -> float:
    """Geodesic angle between two wxyz quaternions, in degrees."""
    q_a = np.asarray(q_a, dtype=float)
    q_b = np.asarray(q_b, dtype=float)
    q_a = q_a / np.linalg.norm(q_a)
    q_b = q_b / np.linalg.norm(q_b)
    return float(
        np.degrees(2.0 * np.arccos(min(1.0, abs(float(q_a @ q_b)))))
    )


def build_table(demo: str) -> tuple[str, list[str]]:
    from pxr import Usd, UsdGeom

    suite = DEMO_SUITES[demo]
    model = mujoco.MjModel.from_xml_path(str(suite["xml"]))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_KEY, "demo_ready"
    )
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)

    stage = Usd.Stage.Open(str(suite["usdz"]))
    geometry = stage.GetDefaultPrim().GetChild("Geometry")
    cache = UsdGeom.XformCache()

    lines = [
        f"### DEMO {demo} - {suite['folder']}",
        "",
        "| body | MJCF pos (m) | USDZ pos (m) | dpos (mm) |"
        " MJCF quat (wxyz) | USDZ quat (wxyz) | drot (deg) |",
        "|---|---|---|---|---|---|---|",
    ]
    worst_pos = worst_rot = 0.0
    for name in RM65_BODIES + SG100_BODIES + tuple(
        suite["task_bodies"]
    ):
        bid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, name
        )
        assert bid >= 0, name
        mj_pos = np.asarray(data.xpos[bid], dtype=float)
        mj_quat = np.zeros(4)
        mujoco.mju_mat2Quat(mj_quat, data.xmat[bid].reshape(9))

        prim = geometry.GetChild(name)
        assert prim, f"{demo}: missing prim {name}"
        world = cache.GetLocalToWorldTransform(prim)
        usd_pos = np.asarray(
            world.ExtractTranslation(), dtype=float
        )
        usd_rot = np.asarray(
            world.ExtractRotationMatrix(), dtype=float
        ).T  # USD row-vector convention -> MuJoCo column-vector
        usd_quat = np.zeros(4)
        mujoco.mju_mat2Quat(usd_quat, usd_rot.reshape(9))

        dpos_mm = float(
            np.max(np.abs(usd_pos - mj_pos)) * 1000.0
        )
        drot_deg = _quat_error_deg(usd_quat, mj_quat)
        worst_pos = max(worst_pos, dpos_mm)
        worst_rot = max(worst_rot, drot_deg)
        lines.append(
            f"| {name} | "
            f"({mj_pos[0]:.4f}, {mj_pos[1]:.4f}, {mj_pos[2]:.4f}) | "
            f"({usd_pos[0]:.4f}, {usd_pos[1]:.4f}, {usd_pos[2]:.4f}) |"
            f" {dpos_mm:.3f} | "
            f"({mj_quat[0]:.4f}, {mj_quat[1]:.4f},"
            f" {mj_quat[2]:.4f}, {mj_quat[3]:.4f}) | "
            f"({usd_quat[0]:.4f}, {usd_quat[1]:.4f},"
            f" {usd_quat[2]:.4f}, {usd_quat[3]:.4f}) |"
            f" {drot_deg:.3f} |"
        )
    lines += [
        "",
        f"Worst translation error: **{worst_pos:.3f} mm**"
        f" (tolerance 1.000 mm); worst rotation error:"
        f" **{worst_rot:.3f} deg** (tolerance 0.500 deg).",
        "",
    ]
    header = (
        f"DEMO {demo} ({len(RM65_BODIES) + len(SG100_BODIES)}"
        f" + {len(suite['task_bodies'])} bodies): "
        f"worst dpos {worst_pos:.3f} mm, worst drot"
        f" {worst_rot:.3f} deg"
    )
    return header, lines


def main() -> int:
    doc = [
        "# Demo suite V2 - MJCF <-> USDZ initial transform table",
        "",
        "MJCF side: ``demo_ready`` keyframe world pose (the state",
        "the exporter consumes; identical to each scene's",
        "initial_pose_manifest.json). USDZ side: packaged Xform"
        " local-to-world",
        "transform. V2 acceptance: <= 1 mm / <= 0.5 deg.",
        "",
    ]
    for demo in sorted(DEMO_SUITES):
        header, lines = build_table(demo)
        print(header)
        doc.extend(lines)
    out = OUT_ROOT / "usdz_transform_table.md"
    out.write_text("\n".join(doc) + "\n")
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
