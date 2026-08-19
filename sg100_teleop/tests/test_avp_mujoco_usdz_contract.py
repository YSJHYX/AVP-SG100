"""Regression tests for the RM65+SG100 AVP compatibility USDZ pipeline.

The source-model checks run in the normal MuJoCo environment.  The USD tests
must be run with ``.venv_usdz/bin/python`` because the application runtime is
deliberately not coupled to pxr or mujoco-usd-converter.
"""

from __future__ import annotations

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
SCENE_XML = (
    ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/"
    "rm65_sg100_avp_grasp_scene.xml"
)
ROBOT_XML = (
    ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/"
    "rm65_sg100_avp_grasp_robot.xml"
)
SIM_XML = (
    ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/"
    "rm65_sg100_sim.xml"
)
SIM_NBODY = 20
SIM_RUNTIME_BODIES = 19
SIM_DIMS = (17, 17, 17)

Q_ARM_READY = np.array(
    [0.32488, -0.76769, 0.97621, 0.22463, -1.80649, -0.12430],
    dtype=np.float64,
)
Q_HAND_READY = np.zeros(11, dtype=np.float64)

# Recorded before introducing the fixed base_link body; dims/nbody updated
# for the 6 grasp objects (coke / redbull / pringles / apple / orange / mug
# freejoints).  Rotations are xmat row-major values from MuJoCo after
# mj_forward at the teleop ready pose.
PRE_BASE_DIMS = (66, 61, 17)
PRE_BASE_NBODY = 28
PRE_BASE_QPOS_ADDRESSES = tuple(range(17))
PRE_BASE_READY_POSES = {
    "link_1": (
        [0.0, 0.0, 0.2405],
        [
            0.9476890366459414, -0.31919506547108084, 0.0,
            0.31919506547108084, 0.9476890366459414, 0.0,
            0.0, 0.0, 1.0,
        ],
    ),
    "link_3": (
        [0.16848454097051674, 0.05674872573942969, 0.42469701838223456],
        [
            -0.9271576755823023, 0.1961905573509667, 0.31919885622467387,
            -0.3122839421623087, 0.06608062713615731, -0.9476877598580984,
            -0.20702025040171984, -0.9783366577633743, 7.99993404854149e-12,
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
    "index_link2": (
        [0.318505478498823, 0.2177804499487535, 0.6294402495169099],
        [
            0.5141715580779054, 0.8555499187269574, -0.060514010196292836,
            -0.8407637060899408, 0.5167166828274061, 0.16161763582527378,
            0.16954055380836813, -0.03222120814214886, 0.9849963423080379,
        ],
    ),
}

SCALE_VISUALS = {
    "rm65_visual_0",
    "rm65_visual_3",
    "rm65_visual_6",
    "sg100_base_link_visual",
    "index_link2_visual",
    "middle_link1_visual",
}


def _body_id(model, name: str) -> int:
    result = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    assert result >= 0, f"body not found: {name}"
    return result


def test_base_link_source_and_kinematic_regression() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    layout = resolve_robot_layout(model)
    initialize_robot_state(model, data, layout, Q_ARM_READY, Q_HAND_READY)

    assert (model.nq, model.nv, model.nu) == PRE_BASE_DIMS
    assert model.nbody == PRE_BASE_NBODY + 1

    base_id = _body_id(model, "base_link")
    assert int(model.body_jntnum[base_id]) == 0
    assert int(model.body_dofnum[base_id]) == 0
    np.testing.assert_allclose(data.xpos[base_id], np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(data.xmat[base_id], np.eye(3).ravel(), atol=1e-12)

    qpos_addresses = tuple(
        int(
            model.jnt_qposadr[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ]
        )
        for name in ROBOT_JOINT_ORDER
    )
    assert qpos_addresses == PRE_BASE_QPOS_ADDRESSES

    for name, (expected_position, expected_rotation) in PRE_BASE_READY_POSES.items():
        body_id = _body_id(model, name)
        np.testing.assert_allclose(data.xpos[body_id], expected_position, atol=1e-10)
        np.testing.assert_allclose(data.xmat[body_id], expected_rotation, atol=1e-10)

    # sg100_base_link is the authoritative teleoperation TCP body.
    tcp_id = _body_id(model, "sg100_base_link")
    np.testing.assert_allclose(
        data.xpos[tcp_id], PRE_BASE_READY_POSES["sg100_base_link"][0], atol=1e-10
    )


def test_source_table_bottle_contact() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    table_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "grasp_table_collision")
    bottle_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "water_bottle_collision")
    table_top = float(data.geom_xpos[table_id, 2] + model.geom_size[table_id, 2])
    bottle_bottom = float(data.geom_xpos[bottle_id, 2] - model.geom_size[bottle_id, 1])
    assert abs(bottle_bottom - table_top) < 1e-6


def test_sim_source_regression() -> None:
    model = mujoco.MjModel.from_xml_path(str(SIM_XML))
    assert (model.nq, model.nv, model.nu) == SIM_DIMS
    assert model.nbody == SIM_NBODY

    base_id = _body_id(model, "base_link")
    assert int(model.body_jntnum[base_id]) == 0
    assert int(model.body_dofnum[base_id]) == 0
    link1_id = _body_id(model, "link_1")
    assert int(model.body_parentid[link1_id]) == base_id
    assert int(model.body_parentid[base_id]) == 0

    qpos_addresses = tuple(
        int(
            model.jnt_qposadr[
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ]
        )
        for name in ROBOT_JOINT_ORDER
    )
    assert qpos_addresses == tuple(range(17))


def _run_usd_contract_checks(
    temp_dir: Path,
    *,
    xml_path: Path = SCENE_XML,
    expected_bodies: int = 28,
    expect_guides: bool = True,
    expect_table_bottle: bool = True,
) -> None:
    try:
        from pxr import Usd, UsdGeom
        from sg100_teleop.tools.export_avp_mujoco_usdz import (
            audit_compat_stage,
            export_compat_usdz,
            runtime_body_names,
        )
    except ImportError as exc:
        raise RuntimeError(
            "USD contract tests require .venv_usdz/bin/python"
        ) from exc

    output = temp_dir / "scene_compat.usdz"
    intermediate = temp_dir / "intermediate"
    audit = export_compat_usdz(
        xml_path,
        output,
        intermediate_dir=intermediate,
    )
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    body_names = runtime_body_names(model)

    assert len(body_names) == expected_bodies
    assert len(audit.body_paths) == expected_bodies
    assert len(audit.kept_renderables) == expected_bodies
    assert not audit.ownership_collisions
    assert not audit.nested_runtime_bodies
    assert not audit.physics_prims
    assert not audit.world_static_renderables
    if expect_guides:
        assert any(path.endswith("/workspace_ready") for path in audit.skipped_guides)
    assert any(path.endswith("/PhysicsScene") for path in audit.skipped_physics_only)

    stage = Usd.Stage.Open(str(output))
    assert stage is not None
    assert stage.GetMetadata("upAxis") == "Z"
    assert stage.GetMetadata("metersPerUnit") == 1.0
    body_paths, collisions, nested, physics = audit_compat_stage(stage, body_names)
    assert not collisions
    assert not nested
    assert not physics

    geometry = stage.GetDefaultPrim().GetChild("Geometry")
    assert geometry
    assert all(stage.GetPrimAtPath(path).GetParent() == geometry for path in body_paths.values())

    base = stage.GetPrimAtPath(body_paths["base_link"])
    rm65_base = base.GetChild("rm65_visual_0")
    assert rm65_base
    assert rm65_base.GetParent() == base
    assert not geometry.GetChild("rm65_visual_0")

    visual_by_body = {record.body: record for record in audit.kept_renderables}
    assert set(visual_by_body) == set(body_names)
    for record in audit.kept_renderables:
        assert record.transform_error <= 1e-8
        np.testing.assert_allclose(record.bbox_extent_ratio, np.ones(3), atol=1e-10)

    scale_records = {
        Path(record.destination_path).name: record
        for record in audit.kept_renderables
        if Path(record.destination_path).name in SCALE_VISUALS
    }
    assert set(scale_records) == {
        name for name in SCALE_VISUALS
        if name in {Path(r.destination_path).name for r in audit.kept_renderables}
    }

    # Main robot visuals retain either authored display color or a material binding.
    for visual_name in ("rm65_visual_0", "rm65_visual_3", "sg100_base_link_visual"):
        record = next(
            item for item in audit.kept_renderables if Path(item.destination_path).name == visual_name
        )
        prim = stage.GetPrimAtPath(record.destination_path)
        color = UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()
        material_targets = prim.GetRelationship("material:binding").GetTargets()
        assert color or material_targets

    if not expect_table_bottle:
        assert zipfile.is_zipfile(output)
        with zipfile.ZipFile(output) as archive:
            assert archive.testzip() is None
            assert archive.namelist()
        return

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], useExtentsHint=False
    )
    table_prim = stage.GetPrimAtPath(
        visual_by_body["grasp_table"].destination_path
    )
    bottle_prim = stage.GetPrimAtPath(
        visual_by_body["water_bottle"].destination_path
    )
    table_range = bbox_cache.ComputeWorldBound(table_prim).ComputeAlignedRange()
    bottle_range = bbox_cache.ComputeWorldBound(bottle_prim).ComputeAlignedRange()
    gap = float(bottle_range.GetMin()[2] - table_range.GetMax()[2])
    assert abs(gap) < 1e-6

    assert zipfile.is_zipfile(output)
    with zipfile.ZipFile(output) as archive:
        assert archive.testzip() is None
        assert archive.namelist()


def test_usd_contract() -> None:
    with tempfile.TemporaryDirectory(prefix="test_avp_compat_") as directory:
        _run_usd_contract_checks(Path(directory))


def test_sim_usd_contract() -> None:
    with tempfile.TemporaryDirectory(prefix="test_avp_sim_compat_") as directory:
        _run_usd_contract_checks(
            Path(directory),
            xml_path=SIM_XML,
            expected_bodies=SIM_RUNTIME_BODIES,
            expect_guides=False,
            expect_table_bottle=False,
        )


def main() -> None:
    test_base_link_source_and_kinematic_regression()
    print("base_link source + kinematic regression: PASS")
    test_source_table_bottle_contact()
    print("source table/bottle contact: PASS")
    test_sim_source_regression()
    print("sim source regression: PASS")
    with tempfile.TemporaryDirectory(prefix="test_avp_compat_") as directory:
        _run_usd_contract_checks(Path(directory))
    print("AVP USDZ contract A-P: PASS")
    with tempfile.TemporaryDirectory(prefix="test_avp_sim_compat_") as directory:
        _run_usd_contract_checks(
            Path(directory),
            xml_path=SIM_XML,
            expected_bodies=SIM_RUNTIME_BODIES,
            expect_guides=False,
            expect_table_bottle=False,
        )
    print("AVP sim USDZ contract: PASS")


if __name__ == "__main__":
    main()
