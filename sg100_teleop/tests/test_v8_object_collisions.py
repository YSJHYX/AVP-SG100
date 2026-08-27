"""Actual-contact regression tests for the three Production V8 task objects."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from sg100_teleop.kinematics.rm65_ik import RM65IKSolver
from sg100_teleop.sim.combined_robot_layout import (
    ARM_DOF,
    HAND_DOF,
    initialize_robot_state,
    resolve_robot_layout,
)
from sg100_teleop.sim.run_avp_arm_hand_sim import (
    TCP_BODY,
    compute_startup_arm_q,
)


ROOT = Path(__file__).resolve().parents[2]
SCENE = (
    ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/"
    "rm65_sg100_avp_grasp_scene.xml"
)
OBJECTS = ("obj_coke", "obj_apple", "obj_mug")
BODY_CENTER_Z_OFFSET = {
    "obj_coke": 0.0611,
    "obj_apple": 0.0356,
    "obj_mug": 0.0,
}
COLLISION_RADIUS = {
    "obj_coke": 0.033,
    "obj_apple": 0.036,
    "obj_mug": 0.059,
}


@dataclass(frozen=True)
class ContactResult:
    step: int
    count: int
    geom_pairs: tuple[tuple[str, str], ...]


def make_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(SCENE))


def _set_free_body(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_name: str,
    position,
    velocity=(0.0, 0.0, 0.0),
) -> None:
    joint = model.joint(f"{body_name}_free").id
    qpos_address = int(model.jnt_qposadr[joint])
    dof_address = int(model.jnt_dofadr[joint])
    data.qpos[qpos_address : qpos_address + 7] = model.qpos0[
        qpos_address : qpos_address + 7
    ]
    data.qpos[qpos_address : qpos_address + 3] = position
    data.qvel[dof_address : dof_address + 6] = 0.0
    data.qvel[dof_address : dof_address + 3] = velocity


def _contact_body_names(model, contact) -> frozenset[str]:
    return frozenset(
        (
            model.body(int(model.geom_bodyid[contact.geom1])).name,
            model.body(int(model.geom_bodyid[contact.geom2])).name,
        )
    )


def moving_pair_contact(first: str, second: str) -> ContactResult:
    """Move ``first`` into ``second`` in free space and return real contacts."""

    model = make_model()
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    collision_center = np.array([0.35, 0.20, 0.40], dtype=np.float64)

    for index, name in enumerate(OBJECTS):
        _set_free_body(
            model,
            data,
            name,
            np.array([0.10 + 0.25 * index, -0.25, 0.50])
            - np.array([0.0, 0.0, BODY_CENTER_Z_OFFSET[name]]),
        )
    _set_free_body(
        model,
        data,
        second,
        collision_center
        - np.array([0.0, 0.0, BODY_CENTER_Z_OFFSET[second]]),
    )
    _set_free_body(
        model,
        data,
        first,
        collision_center
        - np.array(
            [COLLISION_RADIUS[first] + COLLISION_RADIUS[second] + 0.03, 0.0, 0.0]
        )
        - np.array([0.0, 0.0, BODY_CENTER_Z_OFFSET[first]]),
        velocity=(0.30, 0.0, 0.0),
    )
    mujoco.mj_forward(model, data)

    wanted = frozenset((first, second))
    for step in range(500):
        mujoco.mj_step(model, data)
        assert all(
            int(model.geom_bodyid[contact.geom1])
            != int(model.geom_bodyid[contact.geom2])
            for contact in data.contact
        ), "same-body collision appeared"
        matching = [
            contact
            for contact in data.contact
            if _contact_body_names(model, contact) == wanted
        ]
        if matching:
            return ContactResult(
                step=step,
                count=len(matching),
                geom_pairs=tuple(
                    (
                        model.geom(contact.geom1).name,
                        model.geom(contact.geom2).name,
                    )
                    for contact in matching
                ),
            )
    raise AssertionError(f"No MuJoCo contact generated for {first} -> {second}")


def _ready_state(model: mujoco.MjModel) -> tuple[mujoco.MjData, object]:
    data = mujoco.MjData(model)
    layout = resolve_robot_layout(model)
    arm_q = compute_startup_arm_q(
        model,
        data,
        RM65IKSolver(model, tcp_body_name=TCP_BODY),
    )
    assert arm_q is not None
    initialize_robot_state(
        model,
        data,
        layout,
        arm_q,
        np.zeros(HAND_DOF, dtype=np.float64),
    )
    return data, layout


def _compiled_visual_local_points(model: mujoco.MjModel) -> np.ndarray:
    geom = model.geom("mug_visual").id
    mesh = int(model.geom_dataid[geom])
    start = int(model.mesh_vertadr[mesh])
    count = int(model.mesh_vertnum[mesh])
    points = np.asarray(model.mesh_vert[start : start + count], dtype=np.float64)
    rotation_flat = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(rotation_flat, model.geom_quat[geom])
    return (
        points @ rotation_flat.reshape(3, 3).T
        + np.asarray(model.geom_pos[geom])
    )


def _local_handle_direction(points: np.ndarray) -> np.ndarray:
    handle = points[points[:, 1] <= np.percentile(points[:, 1], 5), :2].mean(axis=0)
    direction = handle
    return direction / np.linalg.norm(direction)


def test_three_object_pairs_generate_actual_contacts() -> None:
    for pair in (
        ("obj_coke", "obj_apple"),
        ("obj_coke", "obj_mug"),
        ("obj_apple", "obj_mug"),
    ):
        result = moving_pair_contact(*pair)
        assert result.count >= 1
        assert result.geom_pairs


def test_objects_remain_stable_on_table_for_three_seconds() -> None:
    model = make_model()
    data, layout = _ready_state(model)
    table_contacts: set[str] = set()
    table_body = model.body("grasp_table").id
    for _ in range(1500):
        arm_dofs = layout.dof_indices[:ARM_DOF]
        data.qfrc_applied[arm_dofs] = data.qfrc_bias[arm_dofs]
        mujoco.mj_step(model, data)
        for contact in data.contact:
            body_ids = {
                int(model.geom_bodyid[contact.geom1]),
                int(model.geom_bodyid[contact.geom2]),
            }
            if table_body not in body_ids:
                continue
            for name in OBJECTS:
                if model.body(name).id in body_ids:
                    table_contacts.add(name)

    assert table_contacts == set(OBJECTS)
    assert np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))
    for name in OBJECTS:
        body = model.body(name).id
        assert float(data.xpos[body, 2]) > -0.005
        joint = model.joint(f"{name}_free").id
        dof = int(model.jnt_dofadr[joint])
        assert float(np.linalg.norm(data.qvel[dof : dof + 6])) < 0.02


def _crate_wall_contact(
    object_name: str,
    body_z: float,
    start_xy=(0.56, -0.15),
    velocity=(0.20, 0.0, 0.0),
) -> ContactResult:
    model = make_model()
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    for index, name in enumerate(OBJECTS):
        _set_free_body(model, data, name, [0.10 + index * 0.20, 0.40, 0.50])
    _set_free_body(
        model,
        data,
        object_name,
        [*start_xy, body_z],
        velocity=velocity,
    )
    mujoco.mj_forward(model, data)
    for step in range(500):
        mujoco.mj_step(model, data)
        matching = []
        for contact in data.contact:
            geom_names = (
                model.geom(contact.geom1).name,
                model.geom(contact.geom2).name,
            )
            if any(name.startswith("wood_crate_wall") for name in geom_names) and (
                "obj_coke_collision" in geom_names
                or any(name.startswith("mug_coll_") for name in geom_names)
            ):
                matching.append(contact)
        if matching:
            return ContactResult(
                step=step,
                count=len(matching),
                geom_pairs=tuple(
                    (model.geom(c.geom1).name, model.geom(c.geom2).name)
                    for c in matching
                ),
            )
    raise AssertionError(f"No crate wall contact for {object_name}")


def test_crate_bottom_and_walls_remain_collidable() -> None:
    model = make_model()
    data, layout = _ready_state(model)
    _set_free_body(model, data, "obj_apple", [0.55, -0.15, 0.12])
    mujoco.mj_forward(model, data)
    apple = model.body("obj_apple").id
    crate = model.body("drop_bin").id
    apple_crate_contact = False
    for _ in range(750):
        arm_dofs = layout.dof_indices[:ARM_DOF]
        data.qfrc_applied[arm_dofs] = data.qfrc_bias[arm_dofs]
        mujoco.mj_step(model, data)
        apple_crate_contact |= any(
            _contact_body_names(model, contact) == frozenset(("obj_apple", "drop_bin"))
            for contact in data.contact
        )
    assert apple_crate_contact
    assert np.linalg.norm(data.xpos[apple, :2] - [0.55, -0.15]) < 0.03
    assert 0.01 < float(data.xpos[apple, 2]) < 0.03
    assert crate >= 0

    assert _crate_wall_contact("obj_coke", 0.0155).count >= 1
    assert _crate_wall_contact(
        "obj_mug",
        0.075,
        start_xy=(0.55, -0.15),
        velocity=(0.0, -0.20, 0.0),
    ).count >= 1


def test_collision_masks_cover_objects_robot_table_and_crate() -> None:
    model = make_model()
    collision_geoms = (
        "obj_coke_collision",
        "obj_apple_collision",
        *(f"mug_coll_{index}" for index in range(32)),
    )
    counterparts = (
        "obj_coke_collision",
        "obj_apple_collision",
        "mug_coll_0",
        "index_link2_collision",
        "index_link3_collision",
        "palm_collision",
        "rm65_visual_3",
        "grasp_table_collision",
        "wood_crate_bottom",
    )
    for geom_name in collision_geoms:
        geom = model.geom(geom_name).id
        assert (
            int(model.geom_contype[geom]),
            int(model.geom_conaffinity[geom]),
        ) == (2, 11)
        for counterpart_name in counterparts:
            counterpart = model.geom(counterpart_name).id
            if int(model.geom_bodyid[geom]) == int(model.geom_bodyid[counterpart]):
                continue
            enabled = bool(
                (int(model.geom_contype[geom]) & int(model.geom_conaffinity[counterpart]))
                or (
                    int(model.geom_contype[counterpart])
                    & int(model.geom_conaffinity[geom])
                )
            )
            assert enabled, f"masked collision: {geom_name} / {counterpart_name}"


def _moving_apple_into_robot(target_geom_name: str) -> ContactResult:
    model = make_model()
    model.opt.gravity[:] = 0.0
    data, _layout = _ready_state(model)
    target_geom = model.geom(target_geom_name).id
    target_body = int(model.geom_bodyid[target_geom])
    target_position = data.geom_xpos[target_geom].copy()
    for index, name in enumerate(OBJECTS):
        _set_free_body(model, data, name, [1.5 + 0.2 * index, 1.5, 0.2])
    _set_free_body(
        model,
        data,
        "obj_apple",
        target_position + [0.12, 0.0, 0.0],
        [-0.2, 0.0, 0.0],
    )
    wanted = {model.body("obj_apple").id, target_body}
    for step in range(800):
        mujoco.mj_step(model, data)
        matching = [
            contact
            for contact in data.contact
            if {
                int(model.geom_bodyid[contact.geom1]),
                int(model.geom_bodyid[contact.geom2]),
            }
            == wanted
        ]
        if matching:
            return ContactResult(
                step=step,
                count=len(matching),
                geom_pairs=tuple(
                    (model.geom(contact.geom1).name, model.geom(contact.geom2).name)
                    for contact in matching
                ),
            )
    raise AssertionError(f"No Apple contact generated with {target_geom_name}")


def test_actual_object_contacts_with_sg100_and_rm65() -> None:
    assert _moving_apple_into_robot("palm_collision").count >= 1
    assert _moving_apple_into_robot("rm65_visual_3").count >= 1


def test_mug_yaw_rotates_handle_90_degrees_toward_crate() -> None:
    model = make_model()
    data, _layout = _ready_state(model)
    mug = model.body("obj_mug").id
    mug_joint = model.joint("obj_mug_free").id
    qpos_address = int(model.jnt_qposadr[mug_joint])
    np.testing.assert_allclose(
        model.qpos0[qpos_address : qpos_address + 3],
        [0.55, 0.15, 0.04281],
        atol=1e-12,
        rtol=0.0,
    )
    rotation = data.xmat[mug].reshape(3, 3)
    np.testing.assert_allclose(rotation[:, 2], [0.0, 0.0, 1.0], atol=1e-12)

    local_points = _compiled_visual_local_points(model)
    local_handle = _local_handle_direction(local_points)
    np.testing.assert_allclose(local_handle, [0.0, -1.0], atol=1e-7)
    world_handle = rotation[:2, :2] @ local_handle
    world_handle /= np.linalg.norm(world_handle)
    previous_yaw = 1.706530094896234
    current_yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    np.testing.assert_allclose(current_yaw, previous_yaw - np.pi / 2, atol=1e-12)
    crate = data.xpos[model.body("drop_bin").id]
    toward_crate = crate[:2] - data.xpos[mug, :2]
    toward_crate /= np.linalg.norm(toward_crate)
    angle = float(
        np.degrees(np.arccos(np.clip(np.dot(world_handle, toward_crate), -1.0, 1.0)))
    )
    assert angle <= 10.0


def test_scene_dimensions_and_free_body_contract() -> None:
    model = make_model()
    assert (model.nq, model.nv, model.nu, model.ncam, model.nlight) == (
        38,
        35,
        17,
        3,
        1,
    )
    free_bodies = {
        model.body(int(model.jnt_bodyid[joint])).name
        for joint in range(model.njnt)
        if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE
    }
    assert free_bodies == set(OBJECTS)


def main() -> None:
    test_three_object_pairs_generate_actual_contacts()
    for pair in (
        ("obj_coke", "obj_apple"),
        ("obj_coke", "obj_mug"),
        ("obj_apple", "obj_mug"),
    ):
        print(f"{pair[0]} -> {pair[1]}: {moving_pair_contact(*pair)}")
    test_objects_remain_stable_on_table_for_three_seconds()
    print("three objects -> table + 3 s stability: PASS")
    test_crate_bottom_and_walls_remain_collidable()
    print("apple -> crate bottom; Coke/mug -> crate walls: PASS")
    test_collision_masks_cover_objects_robot_table_and_crate()
    print("object/SG100/RM65/table/crate masks: PASS")
    test_actual_object_contacts_with_sg100_and_rm65()
    print(f"Apple -> SG100: {_moving_apple_into_robot('palm_collision')}")
    print(f"Apple -> RM65: {_moving_apple_into_robot('rm65_visual_3')}")
    test_mug_yaw_rotates_handle_90_degrees_toward_crate()
    print("mug translation/upright/90-degrees-toward-crate: PASS")
    test_scene_dimensions_and_free_body_contract()
    print("V8 dimensions + three free objects: PASS")


if __name__ == "__main__":
    main()
