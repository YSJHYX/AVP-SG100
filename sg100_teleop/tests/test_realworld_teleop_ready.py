"""Regression checks for the robot-only realworld teleop ready posture."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from sg100_teleop.kinematics.rm65_ik import RM65IKSolver
from sg100_teleop.sim.combined_robot_layout import (
    HAND_DOF,
    ROBOT_JOINT_ORDER,
    initialize_robot_state,
    resolve_robot_layout,
)
from sg100_teleop.sim.camera_observations import (
    CAMERA_TO_OBSERVATION_KEY,
    validate_camera_observation_contract,
)
from sg100_teleop.sim.run_avp_arm_hand_sim import (
    TCP_BODY,
    compute_startup_arm_q,
)
from sg100_teleop.sim.view_realworld_teleop_scene import (
    READY_KEYFRAME,
    SCENE_PATH as REALWORLD_XML,
    load_ready_scene,
)


ROOT = Path(__file__).resolve().parents[2]
GRASP_XML = (
    ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/"
    "rm65_sg100_avp_grasp_scene.xml"
)
FORBIDDEN_NAMES = (
    "grasp_table",
    "drop_bin",
    "obj_coke",
    "obj_apple",
    "obj_mug",
    "workstation_floor_visual",
    "demo_camera",
)


def _joint_value(model, data, name: str) -> float:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    assert joint_id >= 0
    return float(data.qpos[int(model.jnt_qposadr[joint_id])])


def _startup_grasp_state():
    model = mujoco.MjModel.from_xml_path(str(GRASP_XML))
    data = mujoco.MjData(model)
    layout = resolve_robot_layout(model)
    solver = RM65IKSolver(model, tcp_body_name=TCP_BODY)
    arm_q = compute_startup_arm_q(model, data, solver)
    assert arm_q is not None
    initialize_robot_state(
        model,
        data,
        layout,
        arm_q,
        np.zeros(HAND_DOF, dtype=np.float64),
    )
    return model, data


def test_ready_keyframe_matches_actual_grasp_startup() -> None:
    grasp_model, grasp_data = _startup_grasp_state()
    real_model, real_data = load_ready_scene()

    assert (real_model.nq, real_model.nv, real_model.nu, real_model.ncam) == (
        17,
        17,
        17,
        2,
    )
    assert real_model.nkey == 1
    assert mujoco.mj_name2id(
        real_model, mujoco.mjtObj.mjOBJ_KEY, READY_KEYFRAME
    ) == 0

    grasp_q = np.array(
        [_joint_value(grasp_model, grasp_data, name) for name in ROBOT_JOINT_ORDER]
    )
    real_q = np.array(
        [_joint_value(real_model, real_data, name) for name in ROBOT_JOINT_ORDER]
    )
    np.testing.assert_allclose(real_q, grasp_q, atol=1e-12, rtol=0.0)

    for body_name in ("base_link", "link_6", "sg100_base_link"):
        grasp_id = grasp_model.body(body_name).id
        real_id = real_model.body(body_name).id
        np.testing.assert_allclose(
            real_data.xpos[real_id], grasp_data.xpos[grasp_id], atol=1e-12, rtol=0.0
        )
        np.testing.assert_allclose(
            real_data.xmat[real_id], grasp_data.xmat[grasp_id], atol=1e-12, rtol=0.0
        )


def test_joint_semantics_and_robot_contract_are_unchanged() -> None:
    grasp_model = mujoco.MjModel.from_xml_path(str(GRASP_XML))
    real_model = mujoco.MjModel.from_xml_path(str(REALWORLD_XML))
    grasp_layout = resolve_robot_layout(grasp_model)
    real_layout = resolve_robot_layout(real_model)

    assert tuple(grasp_model.body(index).name for index in range(1, 20)) == tuple(
        real_model.body(index).name for index in range(1, real_model.nbody)
    )
    for name in ROBOT_JOINT_ORDER:
        grasp_joint = grasp_model.joint(name).id
        real_joint = real_model.joint(name).id
        np.testing.assert_array_equal(
            real_model.jnt_axis[real_joint], grasp_model.jnt_axis[grasp_joint]
        )
        np.testing.assert_array_equal(
            real_model.jnt_range[real_joint], grasp_model.jnt_range[grasp_joint]
        )
        grasp_qpos = int(grasp_model.jnt_qposadr[grasp_joint])
        real_qpos = int(real_model.jnt_qposadr[real_joint])
        assert grasp_model.qpos0[grasp_qpos] == real_model.qpos0[real_qpos] == 0.0

        actuator_name = f"{name}_pos"
        grasp_actuator = grasp_model.actuator(actuator_name).id
        real_actuator = real_model.actuator(actuator_name).id
        assert int(grasp_model.actuator_trnid[grasp_actuator, 0]) == grasp_joint
        assert int(real_model.actuator_trnid[real_actuator, 0]) == real_joint

    np.testing.assert_array_equal(grasp_layout.qpos_indices, real_layout.qpos_indices)
    np.testing.assert_array_equal(grasp_layout.dof_indices, real_layout.dof_indices)
    np.testing.assert_array_equal(grasp_layout.actuator_ids, real_layout.actuator_ids)
    np.testing.assert_array_equal(
        grasp_model.body_pos[grasp_model.body("base_link").id],
        real_model.body_pos[real_model.body("base_link").id],
    )
    np.testing.assert_array_equal(
        grasp_model.body_quat[grasp_model.body("base_link").id],
        real_model.body_quat[real_model.body("base_link").id],
    )


def test_realworld_scene_remains_robot_only() -> None:
    model = mujoco.MjModel.from_xml_path(str(REALWORLD_XML))
    assert model.ncam == 2
    assert model.nbody == 20
    assert tuple(model.camera(index).name for index in range(model.ncam)) == (
        "workstation_front",
        "wrist_camera",
    )
    assert set(validate_camera_observation_contract(model)) == set(
        CAMERA_TO_OBSERVATION_KEY
    )
    for name in FORBIDDEN_NAMES:
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) < 0
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) < 0
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name) < 0
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name) < 0


def main() -> None:
    test_ready_keyframe_matches_actual_grasp_startup()
    print("realworld_ready qpos + FK match grasp startup: PASS")
    test_joint_semantics_and_robot_contract_are_unchanged()
    print("joint zero/axis/range/actuator/body contract: PASS")
    test_realworld_scene_remains_robot_only()
    print("robot-only scene contents: PASS")


if __name__ == "__main__":
    main()
