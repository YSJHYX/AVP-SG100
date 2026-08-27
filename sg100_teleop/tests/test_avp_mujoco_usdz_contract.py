"""Production V8 regression checks for the MuJoCo -> AVP USDZ pipeline.

Run this file with ``.venv_usdz/bin/python``.  The application environment is
intentionally not coupled to Pixar USD or mujoco-usd-converter.
"""

from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path
import zipfile

import mujoco
import numpy as np

from sg100_teleop.sim.combined_robot_layout import (
    ROBOT_JOINT_ORDER,
    initialize_robot_state,
    resolve_robot_layout,
)


ROOT = Path(__file__).resolve().parents[2]
ASSET_ROOT = ROOT / "sg100_teleop/assets/rm65_sg100"
SCENE_XML = ASSET_ROOT / "mjcf/rm65_sg100_avp_grasp_scene.xml"
OLD_ROBOT_USDZ = ASSET_ROOT / "usdz/rm65_sg100_sim_compat.usdz"
V8_COMPAT_USDZ = ASSET_ROOT / "usdz/rm65_sg100_avp_grasp_v8_compat.usdz"
REALWORLD_XML = ASSET_ROOT / "mjcf/rm65_sg100_realworld_teleop_scene.xml"
REALWORLD_COMPAT_USDZ = (
    ASSET_ROOT / "usdz/rm65_sg100_realworld_teleop_compat.usdz"
)
REALWORLD_READY_KEYFRAME = "realworld_ready"

Q_ARM_READY = np.array(
    [0.32488, -0.76769, 0.97621, 0.22463, -1.80649, -0.12430],
    dtype=np.float64,
)
Q_HAND_READY = np.zeros(11, dtype=np.float64)

RM65_BODIES = (
    "base_link",
    "link_1",
    "link_2",
    "link_3",
    "link_4",
    "link_5",
    "link_6",
)
SG100_BODIES = (
    "sg100_base_link",
    "thumb_link1",
    "thumb_link2",
    "thumb_link3",
    "index_link1",
    "index_link2",
    "index_link3",
    "middle_link1",
    "middle_link2",
    "little_link1",
    "little_llink2",
    "little_linik3",
)
ROBOT_BODIES = RM65_BODIES + SG100_BODIES
SCENE_BODIES = ("obj_coke", "obj_apple", "obj_mug", "drop_bin", "grasp_table")
V8_BODIES = ROBOT_BODIES + SCENE_BODIES

PRE_BASE_READY_POSES = {
    "link_1": (
        [0.0, 0.0, 0.2405],
        [
            0.9476890366459414, -0.31919506547108084, 0.0,
            0.31919506547108084, 0.9476890366459414, 0.0,
            0.0, 0.0, 1.0,
        ],
    ),
    "link_6": (
        [0.250484342635546, 0.11727796077348264, 0.6255079942914179],
        [
            -0.06051401019629293, 0.5141679282809396, 0.8555521001652975,
            0.16161763582527386, -0.8407658983264679, 0.5167131157630297,
            0.9849963423080381, 0.16954069050990536, -0.032220488841921135,
        ],
    ),
    "sg100_base_link": (
        [0.250484342635546, 0.11727796077348264, 0.6255079942914179],
        [
            -0.06051401019629293, 0.5141679282809396, 0.8555521001652975,
            0.16161763582527386, -0.8407658983264679, 0.5167131157630297,
            0.9849963423080381, 0.16954069050990536, -0.032220488841921135,
        ],
    ),
}

SG100_VISUAL_FACE_COUNTS = {
    "sg100_base_link_visual_part1": 187_999,
    "sg100_base_link_visual_part2": 181_028,
    "sg100_base_link_visual_part3": 112_678,
    "thumb_link1_visual": 16_970,
    "thumb_link2_visual": 193_892,
    "thumb_link3_visual": 19_130,
    "index_link1_visual": 19_560,
    "index_link2_visual": 193_892,
    "index_link3_visual": 19_130,
    "middle_link1_visual": 193_892,
    "middle_link2_visual": 19_130,
    "little_link1_visual": 19_502,
    "little_llink2_visual": 193_892,
    "little_linik3_visual": 19_130,
}

EXPECTED_VISUALS = {
    *(f"rm65_visual_{index}" for index in range(7)),
    *SG100_VISUAL_FACE_COUNTS,
    "obj_coke_visual",
    "obj_apple_visual",
    "mug_visual",
    "wood_crate_bottom",
    "wood_crate_wall_pX",
    "wood_crate_wall_nX",
    "wood_crate_wall_pY",
    "wood_crate_wall_nY",
    "grasp_table_collision",
}


def _body_id(model: mujoco.MjModel, name: str) -> int:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    assert body_id >= 0, f"body not found: {name}"
    return body_id


def _stage_geometry(stage):
    geometry = stage.GetDefaultPrim().GetChild("Geometry")
    assert geometry
    return geometry


def _world_points(stage, mesh_prim) -> np.ndarray:
    from pxr import UsdGeom

    points = np.asarray(UsdGeom.Mesh(mesh_prim).GetPointsAttr().Get(), dtype=np.float64)
    matrix = np.asarray(
        UsdGeom.XformCache().GetLocalToWorldTransform(mesh_prim), dtype=np.float64
    )
    return (np.column_stack((points, np.ones(points.shape[0]))) @ matrix)[:, :3]


def _mujoco_geom_world_points(
    model: mujoco.MjModel, data: mujoco.MjData, geom_name: str
) -> np.ndarray:
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    assert geom_id >= 0
    mesh_id = int(model.geom_dataid[geom_id])
    start = int(model.mesh_vertadr[mesh_id])
    count = int(model.mesh_vertnum[mesh_id])
    points = np.asarray(model.mesh_vert[start : start + count], dtype=np.float64)
    rotation = np.asarray(data.geom_xmat[geom_id]).reshape(3, 3)
    return points @ rotation.T + data.geom_xpos[geom_id]


def _handle_direction(points: np.ndarray, body_origin: np.ndarray) -> np.ndarray:
    radial = np.linalg.norm(points[:, :2] - body_origin[:2], axis=1)
    handle = points[radial >= np.percentile(radial, 95), :2].mean(axis=0)
    direction = handle - body_origin[:2]
    return direction / np.linalg.norm(direction)


def _assert_body_world_transforms(
    stage, model, data, body_names=V8_BODIES
) -> float:
    from pxr import UsdGeom

    geometry = _stage_geometry(stage)
    cache = UsdGeom.XformCache()
    max_error = 0.0
    for body_name in body_names:
        body_id = _body_id(model, body_name)
        matrix = cache.GetLocalToWorldTransform(geometry.GetChild(body_name))
        position_error = np.max(
            np.abs(np.asarray(matrix.ExtractTranslation()) - data.xpos[body_id])
        )
        rotation_error = np.max(
            np.abs(
                np.asarray(matrix.ExtractRotationMatrix()).T
                - data.xmat[body_id].reshape(3, 3)
            )
        )
        max_error = max(max_error, float(position_error), float(rotation_error))
    assert max_error <= 2e-6
    return max_error


def test_v8_source_contract() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    assert (model.nq, model.nv, model.nu) == (38, 35, 17)
    assert model.nbody == 25
    assert model.ncam == 3
    assert model.nlight == 1
    assert tuple(model.body(index).name for index in range(1, model.nbody)) == V8_BODIES
    assert tuple(model.camera(index).name for index in range(model.ncam)) == (
        "workstation_front",
        "demo_camera",
        "wrist_camera",
    )

    layout = resolve_robot_layout(model)
    assert tuple(model.joint(int(index)).name for index in layout.joint_ids) == (
        ROBOT_JOINT_ORDER
    )
    assert len(layout.actuator_ids) == 17
    qpos_addresses = tuple(
        int(model.jnt_qposadr[model.joint(name).id]) for name in ROBOT_JOINT_ORDER
    )
    assert qpos_addresses == tuple(range(17))

    data = mujoco.MjData(model)
    initialize_robot_state(model, data, layout, Q_ARM_READY, Q_HAND_READY)
    for name, (position, rotation) in PRE_BASE_READY_POSES.items():
        body_id = _body_id(model, name)
        np.testing.assert_allclose(data.xpos[body_id], position, atol=1e-10)
        np.testing.assert_allclose(data.xmat[body_id], rotation, atol=1e-10)


def test_fresh_v8_export_contract() -> None:
    from sg100_teleop.tools.export_avp_mujoco_usdz import export_compat_usdz

    with tempfile.TemporaryDirectory(prefix="test_avp_v8_export_") as directory:
        output = Path(directory) / "v8.usdz"
        audit = export_compat_usdz(SCENE_XML, output)
        assert audit.runtime_body_names == list(V8_BODIES)
        assert len(audit.body_paths) == 24
        assert len(audit.kept_renderables) == 30
        assert not audit.ownership_collisions
        assert not audit.nested_runtime_bodies
        assert not audit.physics_prims
        assert not audit.world_static_renderables
        assert len(audit.excluded_world_renderables) == 1
        assert audit.excluded_world_renderables[0].endswith("/workstation_floor_visual")

        corrections = [
            Path(record.destination_path).name
            for record in audit.kept_renderables
            if record.compiled_mesh_correction
        ]
        assert corrections == ["mug_visual"]
        mesh_records = [
            record
            for record in audit.kept_renderables
            if record.compiled_mesh_fit_error > 0.0
        ]
        assert len(mesh_records) == 24
        assert max(record.compiled_mesh_fit_error for record in mesh_records) <= 1e-7
        assert max(record.transform_error for record in audit.kept_renderables) <= 1e-8
        for record in audit.kept_renderables:
            if not record.compiled_mesh_correction:
                np.testing.assert_allclose(record.bbox_extent_ratio, np.ones(3), atol=1e-10)


def test_prebuilt_v8_package_and_scene_contract() -> None:
    from pxr import Usd, UsdGeom
    from sg100_teleop.tools.export_avp_mujoco_usdz import (
        audit_compat_stage,
        validate_usdz_package,
    )

    assert OLD_ROBOT_USDZ.is_file()
    assert OLD_ROBOT_USDZ.stat().st_size == 11_971_227
    assert V8_COMPAT_USDZ.is_file()
    assert zipfile.is_zipfile(V8_COMPAT_USDZ)
    with zipfile.ZipFile(V8_COMPAT_USDZ) as archive:
        assert archive.testzip() is None
        assert set(archive.namelist()) == {
            "avp_compat.usdc",
            "0/coke_red.png",
            "0/ycb-apple.png",
            "0/image0.png",
        }

    package = validate_usdz_package(V8_COMPAT_USDZ)
    assert not package["unresolved_assets"]
    assert len(package["resolved_assets"]) == 3
    stage = Usd.Stage.Open(str(V8_COMPAT_USDZ))
    assert stage is not None and stage.GetDefaultPrim()
    assert UsdGeom.GetStageUpAxis(stage) == "Z"
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0

    geometry = _stage_geometry(stage)
    assert tuple(child.GetName() for child in geometry.GetChildren()) == V8_BODIES
    paths, collisions, nested, physics = audit_compat_stage(stage, list(V8_BODIES))
    assert set(paths) == set(V8_BODIES)
    assert not collisions and not nested and not physics
    assert all(stage.GetPrimAtPath(path).GetParent() == geometry for path in paths.values())

    old_stage = Usd.Stage.Open(str(OLD_ROBOT_USDZ))
    old_names = tuple(child.GetName() for child in _stage_geometry(old_stage).GetChildren())
    assert old_names == ROBOT_BODIES
    assert tuple(child.GetName() for child in geometry.GetChildren())[:19] == old_names

    renderables = {
        prim.GetName(): prim
        for prim in stage.Traverse()
        if prim.GetTypeName() in {"Mesh", "Cube", "Capsule", "Cylinder"}
    }
    assert set(renderables) == EXPECTED_VISUALS
    assert not any(prim.IsA(UsdGeom.Camera) for prim in stage.Traverse())
    assert not any("Light" in prim.GetTypeName() for prim in stage.Traverse())
    forbidden = (
        "wrist_camera",
        "camera_mount",
        "camera_head",
        "camera_collision",
        "workstation_front",
        "demo_camera",
        "water_bottle",
        "redbull",
        "pringles",
        "orange",
        "diagnostic",
        "debug",
        "palm_collision",
        "mug_coll_",
    )
    all_names = "\n".join(prim.GetName().lower() for prim in stage.Traverse())
    assert not any(name in all_names for name in forbidden)

    for visual_name, expected_faces in SG100_VISUAL_FACE_COUNTS.items():
        mesh = UsdGeom.Mesh(renderables[visual_name])
        assert len(mesh.GetFaceVertexCountsAttr().Get()) == expected_faces

    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    _assert_body_world_transforms(stage, model, data)

    bbox = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], useExtentsHint=False
    )
    table_range = bbox.ComputeWorldBound(geometry.GetChild("grasp_table")).ComputeAlignedRange()
    np.testing.assert_allclose(table_range.GetMin(), [-0.15, -0.35, -0.04], atol=2e-8)
    np.testing.assert_allclose(table_range.GetMax(), [0.75, 0.65, 0.0], atol=2e-8)

    crate = geometry.GetChild("drop_bin")
    assert {child.GetName() for child in crate.GetChildren()} == {
        "wood_crate_bottom",
        "wood_crate_wall_pX",
        "wood_crate_wall_nX",
        "wood_crate_wall_pY",
        "wood_crate_wall_nY",
    }
    crate_range = bbox.ComputeWorldBound(crate).ComputeAlignedRange()
    np.testing.assert_allclose(crate_range.GetMin(), [0.445, -0.255, 0.0], atol=2e-8)
    np.testing.assert_allclose(crate_range.GetMax(), [0.655, -0.045, 0.08], atol=2e-8)
    top_center = np.array([0.55, -0.15, 0.079])
    for child in crate.GetChildren():
        child_range = bbox.ComputeWorldBound(child).ComputeAlignedRange()
        inside = np.all(top_center >= child_range.GetMin()) and np.all(
            top_center <= child_range.GetMax()
        )
        assert not inside, f"crate is closed at the top by {child.GetName()}"

    mug_mesh = renderables["mug_visual"]
    mujoco_points = _mujoco_geom_world_points(model, data, "mug_visual")
    usd_points = _world_points(stage, mug_mesh)
    assert mujoco_points.shape == usd_points.shape
    np.testing.assert_allclose(usd_points, mujoco_points, atol=5e-9, rtol=0.0)
    body_origin = data.xpos[_body_id(model, "obj_mug")]
    usd_body_matrix = UsdGeom.XformCache().GetLocalToWorldTransform(
        geometry.GetChild("obj_mug")
    )
    usd_body_origin = np.asarray(usd_body_matrix.ExtractTranslation())
    source_direction = _handle_direction(mujoco_points, body_origin)
    package_direction = _handle_direction(usd_points, usd_body_origin)
    angle = np.degrees(
        np.arccos(np.clip(np.dot(source_direction, package_direction), -1.0, 1.0))
    )
    assert angle <= 2.0
    assert np.degrees(
        np.arccos(np.clip(np.dot(source_direction, [-1.0, 0.0]), -1.0, 1.0))
    ) <= 10.0


def test_runtime_accepts_the_prebuilt_v8_package() -> None:
    runtime_python = ROOT / ".venv/bin/python"
    script = (
        "from avp_stream.streamer import VisionProStreamer; "
        "import sys; print(VisionProStreamer._validate_prebuilt_usdz(sys.argv[1]))"
    )
    result = subprocess.run(
        [str(runtime_python), "-c", script, str(V8_COMPAT_USDZ)],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    assert result.stdout.strip() == str(V8_COMPAT_USDZ.resolve())


def test_realworld_ready_usdz_contract() -> None:
    from pxr import Usd, UsdGeom
    from sg100_teleop.tools.export_avp_mujoco_usdz import (
        audit_compat_stage,
        export_compat_usdz,
        validate_usdz_package,
    )

    with tempfile.TemporaryDirectory(prefix="test_avp_realworld_export_") as directory:
        output = Path(directory) / "realworld.usdz"
        audit = export_compat_usdz(
            REALWORLD_XML,
            output,
            initial_keyframe=REALWORLD_READY_KEYFRAME,
        )
        assert audit.initial_keyframe == REALWORLD_READY_KEYFRAME
        assert audit.runtime_body_names == list(ROBOT_BODIES)
        assert len(audit.body_paths) == 19
        assert len(audit.kept_renderables) == 21
        assert not audit.ownership_collisions
        assert not audit.nested_runtime_bodies
        assert not audit.physics_prims
        assert not audit.world_static_renderables
        assert not audit.excluded_world_renderables
        assert max(record.transform_error for record in audit.kept_renderables) <= 1e-8

    package = validate_usdz_package(REALWORLD_COMPAT_USDZ)
    assert not package["resolved_assets"]
    assert not package["unresolved_assets"]
    with zipfile.ZipFile(REALWORLD_COMPAT_USDZ) as archive:
        assert archive.testzip() is None
        assert archive.namelist() == ["avp_compat.usdc"]

    stage = Usd.Stage.Open(str(REALWORLD_COMPAT_USDZ))
    assert stage is not None and stage.GetDefaultPrim()
    assert UsdGeom.GetStageUpAxis(stage) == "Z"
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0
    geometry = _stage_geometry(stage)
    assert tuple(child.GetName() for child in geometry.GetChildren()) == ROBOT_BODIES
    paths, collisions, nested, physics = audit_compat_stage(stage, list(ROBOT_BODIES))
    assert set(paths) == set(ROBOT_BODIES)
    assert not collisions and not nested and not physics

    model = mujoco.MjModel.from_xml_path(str(REALWORLD_XML))
    data = mujoco.MjData(model)
    keyframe_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_KEY, REALWORLD_READY_KEYFRAME
    )
    assert keyframe_id >= 0
    mujoco.mj_resetDataKeyframe(model, data, keyframe_id)
    mujoco.mj_forward(model, data)
    _assert_body_world_transforms(stage, model, data, ROBOT_BODIES)

    zero_data = mujoco.MjData(model)
    mujoco.mj_forward(model, zero_data)
    usd_link6 = np.asarray(
        UsdGeom.XformCache()
        .GetLocalToWorldTransform(geometry.GetChild("link_6"))
        .ExtractTranslation()
    )
    assert np.linalg.norm(usd_link6 - zero_data.xpos[model.body("link_6").id]) > 0.5

    renderable_names = {
        prim.GetName()
        for prim in stage.Traverse()
        if prim.GetTypeName() in {"Mesh", "Cube", "Capsule", "Cylinder"}
    }
    expected_robot_visuals = {
        *(f"rm65_visual_{index}" for index in range(7)),
        *SG100_VISUAL_FACE_COUNTS,
    }
    assert renderable_names == expected_robot_visuals
    assert not any(prim.IsA(UsdGeom.Camera) for prim in stage.Traverse())
    assert not any("Light" in prim.GetTypeName() for prim in stage.Traverse())


def main() -> None:
    test_v8_source_contract()
    print("V8 MJCF compile/physics/camera contract: PASS")
    test_fresh_v8_export_contract()
    print("fresh V8 USDZ conversion contract: PASS")
    test_prebuilt_v8_package_and_scene_contract()
    print("prebuilt V8 package/visual/transform/handle contract: PASS")
    test_runtime_accepts_the_prebuilt_v8_package()
    print("AVP prebuilt-package runtime validation: PASS")
    test_realworld_ready_usdz_contract()
    print("realworld grasp-ready USDZ contract: PASS")


if __name__ == "__main__":
    main()
