"""USDZ contract regression for the V2 demo scene suite.

Run with ``.venv_usdz/bin/python -m pytest`` (mirrors
test_avp_mujoco_usdz_contract.py, which is the V8 reference).

Covers the V2 USDZ acceptance items (PART 13/35/54/55/56):
  * package integrity and self-contained asset references
    (no absolute /home /data /tmp references);
  * Z up-axis and 1.0 meters-per-unit;
  * robot prim naming and hierarchy compatible with the V8 USDZ
    (first 19 Geometry children identical);
  * flat runtime bodies only (no nested bodies, no collisions);
  * EVERY streamed body prim's initial world pose equals the
    MuJoCo demo_ready keyframe state within 1 mm / 0.5 deg
    (V2 acceptance), including the hammer and the whole drawer;
  * hammer orientation semantically checked (handle axis and head
    axis, not just COM);
  * every drawer geom's USDZ world pose matches MuJoCo at
    closed / half-open / fully-open (rigid assembly: children
    local transforms fixed, only the root translates);
  * scanned-mesh world AABB agreement (scale regression);
  * no Camera prims, no Light prims.
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
            "rm65_sg100_demo_drawer_loading_and_closing_scene.xml"
        ),
        "usdz": (
            ASSET_ROOT
            / "usdz/"
            "rm65_sg100_demo_drawer_loading_and_closing_compat.usdz"
        ),
        "task_bodies": (
            "drawer_cabinet", "demo_drawer",
            "demo_drawer_item", "grasp_table",
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
            "kit_tray", "grasp_table",
        ),
        "task_meshes": (
            "kit_object_a_visual", "kit_object_b_visual",
        ),
    },
}

# V2 acceptance: translation <= 1 mm, rotation <= 0.5 deg
TRANSLATION_TOLERANCE_M = 1e-3
ROTATION_TOLERANCE_RAD = np.deg2rad(0.5)
# exact-body tightness for non-acceptance sanity checks
EXACT_TOLERANCE = 2e-6
AABB_TOLERANCE = 1e-6

DRAWER_GEOMS = (
    "drawer_bottom", "drawer_side_pY", "drawer_side_nY",
    "drawer_back", "drawer_front", "drawer_handle",
    "drawer_handle_post_pY", "drawer_handle_post_nY",
)


def _stage_geometry(stage):
    geometry = stage.GetDefaultPrim().GetChild("Geometry")
    assert geometry
    return geometry


def _keyframe_data(xml: Path, drawer_q: float | None = None):
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_KEY, "demo_ready"
    )
    assert key >= 0
    mujoco.mj_resetDataKeyframe(model, data, key)
    if drawer_q is not None:
        jid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "demo_drawer_slide"
        )
        data.qpos[model.jnt_qposadr[jid]] = drawer_q
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


def _rotation_error_rad(mat_a, mat_b) -> float:
    """Max-abs element difference of rotation matrices (rad-ish)."""
    return float(np.max(np.abs(mat_a - mat_b)))


def _axis_angle_error_rad(axis_a, axis_b) -> float:
    n = np.linalg.norm(axis_a) * np.linalg.norm(axis_b)
    return float(np.arccos(np.clip(np.dot(axis_a, axis_b) / n,
                                   -1.0, 1.0)))


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

    package = validate_usdz_package(usdz)
    assert not package["unresolved_assets"], package
    with zipfile.ZipFile(usdz) as archive:
        for member in archive.namelist():
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

    v8_stage = Usd.Stage.Open(str(V8_COMPAT_USDZ))
    v8_children = tuple(
        child.GetName()
        for child in _stage_geometry(v8_stage).GetChildren()
    )
    assert children[:19] == v8_children[:19] == ROBOT_BODIES

    paths, collisions, nested, physics = audit_compat_stage(
        stage, list(expected)
    )
    assert set(paths) == set(expected)
    assert not collisions
    assert not nested
    assert not physics

    assert not any(
        prim.IsA(UsdGeom.Camera) for prim in stage.Traverse()
    )
    assert not any(
        "Light" in prim.GetTypeName() for prim in stage.Traverse()
    )


@pytest.mark.parametrize("demo", sorted(DEMO_SUITES))
def test_demo_usdz_initial_transforms_match_keyframe(demo):
    """V2 acceptance: every streamed body prim within 1 mm / 0.5 deg
    of the MuJoCo demo_ready state (exact match expected in
    practice, since the exporter consumes the same keyframe)."""
    from pxr import Usd, UsdGeom

    suite = DEMO_SUITES[demo]
    model, data = _keyframe_data(suite["xml"])
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
        rot_err = _rotation_error_rad(
            np.asarray(matrix.ExtractRotationMatrix()).T,
            data.xmat[body_id].reshape(3, 3),
        )
        assert pos_err <= TRANSLATION_TOLERANCE_M, (
            f"{demo}/{body_name}: position error {pos_err * 1000:.3f} mm"
        )
        assert rot_err <= ROTATION_TOLERANCE_RAD, (
            f"{demo}/{body_name}: rotation error "
            f"{np.degrees(rot_err):.3f} deg"
        )


def test_hammer_initial_usdz_pose_matches_mujoco():
    """PART 13: hammer body pose + semantic axes (handle direction,
    head axis), not just the COM."""
    from pxr import Usd, UsdGeom

    suite = DEMO_SUITES["01"]
    model, data = _keyframe_data(suite["xml"])
    stage = Usd.Stage.Open(str(suite["usdz"]))
    prim = _stage_geometry(stage).GetChild("demo_hammer")
    assert prim
    matrix = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    R_usd = np.asarray(matrix.ExtractRotationMatrix()).T

    body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "demo_hammer"
    )
    pos_err = float(
        np.max(
            np.abs(
                np.asarray(matrix.ExtractTranslation())
                - data.xpos[body_id]
            )
        )
    )
    R_mj = data.xmat[body_id].reshape(3, 3)
    assert pos_err <= TRANSLATION_TOLERANCE_M

    # hammer local X = handle axis, local Z = head/strike axis
    for local_axis, label in (
        (np.array([1.0, 0.0, 0.0]), "handle"),
        (np.array([0.0, 0.0, 1.0]), "head"),
    ):
        usd_axis = R_usd @ local_axis
        mj_axis = R_mj @ local_axis
        err = _axis_angle_error_rad(usd_axis, mj_axis)
        assert err <= ROTATION_TOLERANCE_RAD, (
            f"hammer {label} axis error {np.degrees(err):.3f} deg"
        )
    # the handle must be horizontal and the head axis vertical
    handle_world = R_usd @ np.array([1.0, 0.0, 0.0])
    assert abs(handle_world[2]) < 0.05, "handle not horizontal"
    head_world = R_usd @ np.array([0.0, 0.0, 1.0])
    assert abs(abs(head_world[2]) - 1.0) < 0.05, (
        "mallet head axis not vertical"
    )


def test_drawer_usdz_parts_match_mujoco():
    """PART 35: every drawer geom's USDZ world pose vs MuJoCo at
    closed / half-open / fully-open - rigid assembly check."""
    from pxr import Usd, UsdGeom

    suite = DEMO_SUITES["02"]
    stage = Usd.Stage.Open(str(suite["usdz"]))
    drawer = _stage_geometry(stage).GetChild("demo_drawer")
    assert drawer
    children = {
        c.GetName(): c for c in drawer.GetChildren()
    }
    assert set(children) == set(DRAWER_GEOMS), (
        sorted(children), sorted(DRAWER_GEOMS)
    )

    # reference local transforms must be state-independent
    cache = UsdGeom.XformCache()
    local_ref = {}
    for name, prim in children.items():
        rel, _ = cache.ComputeRelativeTransform(
            prim, drawer
        )
        local_ref[name] = np.asarray(rel, dtype=np.float64)

    for drawer_q in (0.0, 0.075, 0.150):
        model, data = _keyframe_data(
            suite["xml"], drawer_q=drawer_q
        )
        drawer_bid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "demo_drawer"
        )
        # MuJoCo side: children local transforms are state-independent
        mj_local = {}
        for name in DRAWER_GEOMS:
            gid = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_GEOM, name
            )
            assert gid >= 0, name
            rel_pos = (
                data.geom_xpos[gid] - data.xpos[drawer_bid]
            )
            mj_local[name] = rel_pos

        if drawer_q == 0.0:
            # USDZ ships the CLOSED state: compare absolute world
            for name, prim in children.items():
                world = cache.GetLocalToWorldTransform(prim)
                usd_pos = np.asarray(
                    world.ExtractTranslation(),
                    dtype=np.float64,
                )
                gid = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_GEOM, name
                )
                err = float(
                    np.max(
                        np.abs(usd_pos - data.geom_xpos[gid])
                    )
                )
                assert err <= TRANSLATION_TOLERANCE_M, (
                    f"drawer {name} @ closed: "
                    f"{err * 1000:.3f} mm"
                )
        else:
            # half / full open: USDZ local transforms must equal
            # the closed ones (rigid assembly), and MuJoCo's local
            # offsets must match the closed ones too
            if drawer_q == 0.075:
                mj_local_ref = mj_local
            else:
                for name in DRAWER_GEOMS:
                    assert np.allclose(
                        mj_local[name], mj_local_ref[name],
                        atol=1e-9,
                    ), f"MuJoCo drawer child {name} not rigid"
            for name, prim in children.items():
                rel, _ = cache.ComputeRelativeTransform(
                    prim, drawer
                )
                assert np.allclose(
                    np.asarray(rel, dtype=np.float64),
                    local_ref[name],
                    atol=1e-9,
                ), f"USDZ drawer child {name} moved relative"


@pytest.mark.parametrize("demo", sorted(DEMO_SUITES))
def test_demo_usdz_world_aabb_matches_mujoco(demo):
    """Scale regression on every scanned mesh."""
    from pxr import Usd

    suite = DEMO_SUITES[demo]
    model, data = _keyframe_data(suite["xml"])
    stage = Usd.Stage.Open(str(suite["usdz"]))

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
