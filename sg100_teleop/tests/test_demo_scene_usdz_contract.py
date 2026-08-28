"""USDZ contract regression for the demo scene suite.

Run with ``.venv_usdz/bin/python -m pytest`` (mirrors
test_avp_mujoco_usdz_contract.py, which is the V8 reference).

Covers the demo-suite USDZ acceptance items:
  * package integrity and self-contained asset references
    (no absolute /home /data /tmp references - PART 86);
  * Z up-axis and 1.0 meters-per-unit (scale base - PART 87);
  * robot prim naming and hierarchy compatible with the V8 USDZ
    (first 19 Geometry children identical - PART 84);
  * every streamed body prim exists as a flat Geometry child
    (no nested runtime bodies, no ownership collisions);
  * USDZ initial world transforms equal the MuJoCo demo_ready
    keyframe state for robot and task bodies, which simultaneously
    certifies the V8 grasp-ready initial pose (PART 83) and the
    task-object orientations (PART 88);
  * world AABB agreement between MuJoCo and USDZ (PART 87);
  * no Camera prims, no Light prims, no camera hardware
    (PART 85 - the V8 USDZ carries no Camera prims either).
"""

from __future__ import annotations

import zipfile
from pathlib import Path

import mujoco
import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[2]
ASSET_ROOT = ROOT / "sg100_teleop/assets/rm65_sg100"
V8_COMPAT_USDZ = ASSET_ROOT / "usdz/rm65_sg100_avp_grasp_v8_compat.usdz"

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
ROBOT_BODIES = RM65_BODIES + SG100_BODIES

DEMO_SUITES = {
    "01": {
        "xml": ASSET_ROOT
        / "mjcf/rm65_sg100_demo_hammer_peg_assembly_scene.xml",
        "usdz": ASSET_ROOT
        / "usdz/rm65_sg100_demo_hammer_peg_assembly_compat.usdz",
        "task_bodies": (
            "hammer_tool_rest", "demo_hammer",
            "peg_fixture", "demo_peg", "grasp_table",
        ),
        "task_meshes": ("hammer_visual",),
    },
    "02": {
        "xml": (
            ASSET_ROOT
            / "mjcf/"
            "rm65_sg100_demo_drawer_retrieval_and_loading_scene.xml"
        ),
        "usdz": (
            ASSET_ROOT
            / "usdz/"
            "rm65_sg100_demo_drawer_retrieval_and_loading_compat.usdz"
        ),
        "task_bodies": (
            "drawer_cabinet", "demo_drawer",
            "demo_drawer_item", "goal_tray", "grasp_table",
        ),
        "task_meshes": ("drawer_item_visual",),
    },
    "03": {
        "xml": (
            ASSET_ROOT
            / "mjcf/"
            "rm65_sg100_demo_maintenance_kit_packing_scene.xml"
        ),
        "usdz": (
            ASSET_ROOT
            / "usdz/"
            "rm65_sg100_demo_maintenance_kit_packing_compat.usdz"
        ),
        "task_bodies": (
            "kit_object_a", "kit_object_b",
            "kit_object_c", "kit_tray", "grasp_table",
        ),
        "task_meshes": (
            "kit_object_a_visual", "kit_object_b_visual",
            "kit_object_c_visual",
        ),
    },
}

TRANSFORM_TOLERANCE = 2e-6
AABB_TOLERANCE = 1e-6


def _stage_geometry(stage):
    geometry = stage.GetDefaultPrim().GetChild("Geometry")
    assert geometry
    return geometry


def _keyframe_state(xml: Path):
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_KEY, "demo_ready"
    )
    assert key >= 0
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)
    return model, data


def _mujoco_mesh_world_points(model, data, geom_name):
    geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, geom_name
    )
    assert geom_id >= 0, geom_name
    mesh_id = int(model.geom_dataid[geom_id])
    start = int(model.mesh_vertadr[mesh_id])
    count = int(model.mesh_vertnum[mesh_id])
    points = np.asarray(
        model.mesh_vert[start:start + count], dtype=np.float64
    )
    rotation = np.asarray(data.geom_xmat[geom_id]).reshape(3, 3)
    return points @ rotation.T + data.geom_xpos[geom_id]


def _usd_mesh_world_points(stage, mesh_prim) -> np.ndarray:
    from pxr import UsdGeom

    points = np.asarray(
        UsdGeom.Mesh(mesh_prim).GetPointsAttr().Get(),
        dtype=np.float64,
    )
    matrix = np.asarray(
        UsdGeom.XformCache().GetLocalToWorldTransform(mesh_prim),
        dtype=np.float64,
    )
    return (np.column_stack((points, np.ones(points.shape[0])))
            @ matrix)[:, :3]


@pytest.mark.parametrize("demo", sorted(DEMO_SUITES))
def test_demo_usdz_package_and_robot_naming(demo):
    from pxr import Usd, UsdGeom
    from sg100_teleop.tools.export_avp_mujoco_usdz import (
        audit_compat_stage,
        validate_usdz_package,
    )

    suite = DEMO_SUITES[demo]
    usdz = suite["usdz"]
    assert usdz.is_file(), f"missing USDZ: {usdz}"
    assert zipfile.is_zipfile(usdz)

    # PART 86: every packaged reference resolves; no absolute
    # host paths anywhere in the archive
    package = validate_usdz_package(usdz)
    assert not package["unresolved_assets"], package
    with zipfile.ZipFile(usdz) as archive:
        names = archive.namelist()
        for member in names:
            assert not member.startswith(
                ("/", "home/", "data/", "tmp/")
            ), member

    stage = Usd.Stage.Open(str(usdz))
    assert stage is not None and stage.GetDefaultPrim()
    assert UsdGeom.GetStageUpAxis(stage) == "Z"
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0

    geometry = _stage_geometry(stage)
    children = tuple(
        child.GetName() for child in geometry.GetChildren()
    )
    expected = ROBOT_BODIES + suite["task_bodies"]
    assert children == expected, (children, expected)

    # PART 84: robot prim block identical to the V8 USDZ layout
    v8_stage = Usd.Stage.Open(str(V8_COMPAT_USDZ))
    v8_children = tuple(
        child.GetName()
        for child in _stage_geometry(v8_stage).GetChildren()
    )
    assert children[:19] == v8_children[:19] == ROBOT_BODIES

    # flat runtime bodies only
    paths, collisions, nested, physics = audit_compat_stage(
        stage, list(expected)
    )
    assert set(paths) == set(expected)
    assert not collisions
    assert not nested
    assert not physics

    # PART 85: no camera/light hardware in the USDZ
    assert not any(
        prim.IsA(UsdGeom.Camera) for prim in stage.Traverse()
    )
    assert not any(
        "Light" in prim.GetTypeName() for prim in stage.Traverse()
    )


@pytest.mark.parametrize("demo", sorted(DEMO_SUITES))
def test_demo_usdz_initial_transforms_match_keyframe(demo):
    """PART 83 + 88: USDZ opens in the V8 grasp-ready pose."""
    from pxr import Usd, UsdGeom

    suite = DEMO_SUITES[demo]
    model, data = _keyframe_state(suite["xml"])
    stage = Usd.Stage.Open(str(suite["usdz"]))
    geometry = _stage_geometry(stage)
    cache = UsdGeom.XformCache()

    for body_name in ROBOT_BODIES + tuple(suite["task_bodies"]):
        body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, body_name
        )
        assert body_id >= 0, body_name
        prim = geometry.GetChild(body_name)
        assert prim, f"missing prim: {body_name}"
        matrix = cache.GetLocalToWorldTransform(prim)
        pos_err = float(
            np.max(
                np.abs(
                    np.asarray(matrix.ExtractTranslation())
                    - data.xpos[body_id]
                )
            )
        )
        rot_err = float(
            np.max(
                np.abs(
                    np.asarray(
                        matrix.ExtractRotationMatrix()
                    ).T
                    - data.xmat[body_id].reshape(3, 3)
                )
            )
        )
        assert pos_err <= TRANSFORM_TOLERANCE, (
            f"{demo}/{body_name}: position error {pos_err}"
        )
        assert rot_err <= TRANSFORM_TOLERANCE, (
            f"{demo}/{body_name}: rotation error {rot_err}"
        )


@pytest.mark.parametrize("demo", sorted(DEMO_SUITES))
def test_demo_usdz_world_aabb_matches_mujoco(demo):
    """PART 87: no x1000 / x100 / /100 scale drift on scanned meshes."""
    from pxr import Usd

    suite = DEMO_SUITES[demo]
    model, data = _keyframe_state(suite["xml"])
    stage = Usd.Stage.Open(str(suite["usdz"]))
    geometry = _stage_geometry(stage)

    renderables = {
        prim.GetName(): prim
        for prim in stage.Traverse()
        if prim.GetTypeName() == "Mesh"
    }

    for geom_name in suite["task_meshes"]:
        prim = renderables[geom_name]
        assert prim, f"missing mesh prim: {geom_name}"
        mj = _mujoco_mesh_world_points(model, data, geom_name)
        usd = _usd_mesh_world_points(stage, prim)
        mj_extent = mj.max(0) - mj.min(0)
        usd_extent = usd.max(0) - usd.min(0)
        assert np.allclose(
            mj_extent, usd_extent, atol=AABB_TOLERANCE
        ), (
            f"{demo}/{geom_name}: AABB {usd_extent.tolist()} vs "
            f"MuJoCo {mj_extent.tolist()}"
        )
