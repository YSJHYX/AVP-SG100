"""Demo-suite robot friction contract (V2 PART A / PART 9).

The demo scenes include ``rm65_sg100_demo_robot.xml`` instead of
the production robot.  That file must differ from production in
EXACTLY ONE way: the friction of the SG100 hand contact geoms.
Everything else - joint contract, mass/inertia, actuator contract,
body hierarchy, geom shapes/sizes/placements - must be identical.
Production robot friction itself must be unchanged.
"""

from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
import pytest

from sg100_teleop.sim.combined_robot_layout import (
    ROBOT_JOINT_ORDER,
    resolve_robot_layout,
)


ROOT = Path(__file__).resolve().parents[2]
MJCF = ROOT / "sg100_teleop/assets/rm65_sg100/mjcf"

PRODUCTION_ROBOT = (
    MJCF / "rm65_sg100_avp_grasp_robot.xml"
)
DEMO_ROBOT = MJCF / "rm65_sg100_demo_robot.xml"
V8_SCENE = MJCF / "rm65_sg100_avp_grasp_scene.xml"
DEMO_SCENES = {
    "01": MJCF / "rm65_sg100_demo_hammer_peg_assembly_scene.xml",
    "02": (
        MJCF / "rm65_sg100_demo_drawer_loading_and_closing_scene.xml"
    ),
    "03": (
        MJCF / "rm65_sg100_demo_maintenance_kit_packing_scene.xml"
    ),
}

# Hand contact geoms that actually touch task objects, by finger
# (audit output demanded by the V2 spec):
HAND_CONTACT_GEOMS = {
    "thumb": (
        "thumb_link2_collision",
        "thumb_link3_collision",
    ),
    "index": (
        "index_link2_collision",
        "index_link3_collision",
    ),
    "middle": ("middle_link2_collision",),
    "little": (
        "little_llink2_collision",
        "little_linik3_collision",
    ),
    "palm": ("palm_collision",),
}

DEMO_FRICTION = (1.4, 0.01, 0.001)
PRODUCTION_PAD_FRICTION = (1.2, 0.01, 0.001)
PRODUCTION_DEFAULT_FRICTION = (1.0, 0.005, 0.0001)


def _geom_friction(model, name):
    gid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, name
    )
    assert gid >= 0, name
    return tuple(model.geom_friction[gid])


def _geom_record(model):
    return {
        mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_GEOM, gid
        ): (
            model.geom_friction[gid].copy(),
            int(model.geom_priority[gid]),
            model.geom_size[gid].copy(),
            model.geom_pos[gid].copy(),
            model.geom_quat[gid].copy(),
            int(model.geom_contype[gid]),
            int(model.geom_conaffinity[gid]),
            int(model.geom_group[gid]),
            int(model.geom_type[gid]),
        )
        for gid in range(model.ngeom)
        if model.geom_bodyid[gid] > 0
    }


@pytest.fixture(scope="module")
def robots():
    return (
        mujoco.MjModel.from_xml_path(str(PRODUCTION_ROBOT)),
        mujoco.MjModel.from_xml_path(str(DEMO_ROBOT)),
    )


def test_production_robot_friction_unchanged(robots):
    prod, _ = robots
    # the five explicit pads keep 1.2; the three distal capsules
    # keep MuJoCo defaults (no explicit friction attribute)
    for geom in (
        "palm_collision",
        "thumb_link2_collision",
        "index_link2_collision",
        "middle_link2_collision",
        "little_llink2_collision",
    ):
        assert _geom_friction(prod, geom) == pytest.approx(
            PRODUCTION_PAD_FRICTION
        ), geom
    for geom in (
        "thumb_link3_collision",
        "index_link3_collision",
        "little_linik3_collision",
    ):
        # default friction -> MuJoCo's compiled default tuple
        assert _geom_friction(prod, geom) == pytest.approx(
            PRODUCTION_DEFAULT_FRICTION
        ), geom
        assert int(
            prod.geom_priority[
                mujoco.mj_name2id(
                    prod, mujoco.mjtObj.mjOBJ_GEOM, geom
                )
            ]
        ) == 0, geom


def test_demo_robot_only_hand_friction_differs(robots):
    """Every hand contact geom carries the raised pad friction with
    priority 1; every other robot property is identical."""
    prod, demo = robots
    prod_geoms = _geom_record(prod)
    demo_geoms = _geom_record(demo)
    assert set(prod_geoms) == set(demo_geoms)

    hand_names = {
        g for group in HAND_CONTACT_GEOMS.values() for g in group
    }
    for name, prod_rec in prod_geoms.items():
        demo_rec = demo_geoms[name]
        if name in hand_names:
            assert demo_rec[0] == pytest.approx(DEMO_FRICTION), name
            assert demo_rec[1] == 1, name
        # everything except friction/priority identical everywhere
        assert int(demo_rec[1]) == int(prod_rec[1]) or (
            name in hand_names
        ), name
        for field in range(2, len(prod_rec)):
            a, b = demo_rec[field], prod_rec[field]
            assert np.array_equal(a, b), (
                f"non-friction difference on {name} field {field}"
            )


def test_demo_robot_full_contract_identical(robots):
    """Joint / actuator / mass / hierarchy contracts identical."""
    prod, demo = robots
    assert demo.nq == prod.nq
    assert demo.nv == prod.nv
    assert demo.nu == prod.nu == 17
    assert demo.nbody == prod.nbody

    for name in ROBOT_JOINT_ORDER:
        jid_p = mujoco.mj_name2id(
            prod, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        jid_d = mujoco.mj_name2id(
            demo, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        assert jid_p == jid_d >= 0, name
        assert np.array_equal(
            prod.jnt_axis[jid_p], demo.jnt_axis[jid_d]
        ), name
        assert np.array_equal(
            prod.jnt_range[jid_p], demo.jnt_range[jid_d]
        ), name

    # body tree: names, parents, inertias
    for field in ("body_parentid",):
        assert np.array_equal(
            getattr(prod, field), getattr(demo, field)
        )
    assert np.allclose(
        prod.body_mass, demo.body_mass, atol=0.0
    )
    assert np.allclose(
        prod.body_inertia, demo.body_inertia, atol=0.0
    )
    assert np.allclose(
        prod.body_ipos, demo.body_ipos, atol=0.0
    )
    assert np.allclose(
        prod.body_iquat, demo.body_iquat, atol=0.0
    )

    # actuators: names, bindings, gains, ctrlrange
    for name in (
        f"{j}_pos" for j in ROBOT_JOINT_ORDER
    ):
        aid_p = mujoco.mj_name2id(
            prod, mujoco.mjtObj.mjOBJ_ACTUATOR, name
        )
        aid_d = mujoco.mj_name2id(
            demo, mujoco.mjtObj.mjOBJ_ACTUATOR, name
        )
        assert aid_p == aid_d >= 0, name
        assert np.array_equal(
            prod.actuator_ctrlrange[aid_p],
            demo.actuator_ctrlrange[aid_d],
        ), name
        assert np.array_equal(
            prod.actuator_gaintype[aid_p],
            demo.actuator_gaintype[aid_d],
        )
        assert np.array_equal(
            prod.actuator_gainprm[aid_p],
            demo.actuator_gainprm[aid_d],
        )

    # cameras (wrist_camera) identical
    assert demo.ncam == prod.ncam == 1
    assert np.array_equal(prod.cam_pos, demo.cam_pos)
    assert np.array_equal(prod.cam_mat0, demo.cam_mat0)
    assert np.array_equal(prod.cam_fovy, demo.cam_fovy)


@pytest.mark.parametrize("demo_name", sorted(DEMO_SCENES))
def test_demo_scenes_include_demo_robot(demo_name):
    scene = DEMO_SCENES[demo_name].read_text()
    assert '<include file="rm65_sg100_demo_robot.xml"/>' in scene
    assert (
        '<include file="rm65_sg100_avp_grasp_robot.xml"/>'
        not in scene
    )


def test_production_scene_uses_production_robot():
    scene = V8_SCENE.read_text()
    assert (
        '<include file="rm65_sg100_avp_grasp_robot.xml"/>'
        in scene
    )
