from pathlib import Path

import mujoco
import numpy as np

from sg100_teleop.sim.combined_robot_layout import (
    ARM_DOF,
    HAND_DOF,
    ROBOT_JOINT_ORDER,
    initialize_robot_state,
    resolve_robot_layout,
    write_robot_controls,
)


ROOT = Path(__file__).resolve().parents[2]

ROBOT_ONLY_MODEL = (
    ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/rm65_sg100_sim.xml"
)

GRASP_SCENE_MODEL = (
    ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/rm65_sg100_grasp_scene.xml"
)

Q_ARM_READY = np.array(
    [
        0.32488,
        -0.76769,
        0.97621,
        0.22463,
        -1.80649,
        -0.12430,
    ],
    dtype=np.float64,
)

Q_HAND_STARTUP = np.zeros(HAND_DOF, dtype=np.float64)


def load_and_initialize(path: Path):
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    layout = resolve_robot_layout(model)
    qpos0 = model.qpos0.copy()

    initialize_robot_state(
        model,
        data,
        layout,
        Q_ARM_READY,
        Q_HAND_STARTUP,
    )

    if len(layout.joint_ids) != len(ROBOT_JOINT_ORDER):
        raise RuntimeError("Expected 17 resolved robot joints")
    if len(layout.actuator_ids) != len(ROBOT_JOINT_ORDER):
        raise RuntimeError("Expected 17 resolved robot actuators")

    np.testing.assert_allclose(
        data.qpos[layout.arm_qpos_indices],
        Q_ARM_READY,
    )
    np.testing.assert_allclose(
        data.qpos[layout.hand_qpos_indices],
        Q_HAND_STARTUP,
    )

    robot_qpos = set(layout.qpos_indices.tolist())
    scene_qpos = [
        index
        for index in range(model.nq)
        if index not in robot_qpos
    ]
    np.testing.assert_allclose(
        data.qpos[scene_qpos],
        qpos0[scene_qpos],
    )

    return model, data, layout


def bottle_freejoint_qpos(model, data):
    joint_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "water_bottle_free",
    )
    if joint_id < 0:
        raise RuntimeError("water_bottle_free not found")
    if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE:
        raise RuntimeError("water_bottle_free is not a free joint")

    qpos_address = int(model.jnt_qposadr[joint_id])
    free_qpos = data.qpos[qpos_address:qpos_address + 7].copy()
    return qpos_address, free_qpos


def main():
    robot_model, _, robot_layout = load_and_initialize(
        ROBOT_ONLY_MODEL
    )
    print(
        "ROBOT-ONLY: "
        f"nq={robot_model.nq}, nv={robot_model.nv}, "
        f"nu={robot_model.nu}, "
        f"joints={len(robot_layout.joint_ids)}, "
        f"actuators={len(robot_layout.actuator_ids)}: PASS"
    )

    scene_model, scene_data, scene_layout = load_and_initialize(
        GRASP_SCENE_MODEL
    )
    bottle_qpos_address, bottle_qpos_initial = bottle_freejoint_qpos(
        scene_model,
        scene_data,
    )
    np.testing.assert_allclose(
        bottle_qpos_initial,
        scene_model.qpos0[
            bottle_qpos_address:bottle_qpos_address + 7
        ],
    )
    np.testing.assert_allclose(
        np.linalg.norm(bottle_qpos_initial[3:7]),
        1.0,
        atol=1e-12,
    )
    print(
        "SCENE INITIALIZATION: "
        f"nq={scene_model.nq}, nv={scene_model.nv}, "
        f"nu={scene_model.nu}, bottle_quat_norm="
        f"{np.linalg.norm(bottle_qpos_initial[3:7]):.12f}: PASS"
    )

    q_arm_target = Q_ARM_READY.copy()
    q_arm_target[0] += 0.01

    q_hand_target = Q_HAND_STARTUP.copy()
    q_hand_target[0] = 0.05
    q_hand_target[4] = 0.05

    robot_qpos_before = scene_data.qpos[
        scene_layout.qpos_indices
    ].copy()

    write_robot_controls(
        scene_data,
        scene_layout,
        q_arm_target,
        q_hand_target,
    )

    for _ in range(50):
        mujoco.mj_step(scene_model, scene_data)

    robot_qpos_after = scene_data.qpos[
        scene_layout.qpos_indices
    ]
    arm_response = float(
        np.max(
            np.abs(
                robot_qpos_after[:ARM_DOF]
                - robot_qpos_before[:ARM_DOF]
            )
        )
    )
    hand_response = float(
        np.max(
            np.abs(
                robot_qpos_after[ARM_DOF:]
                - robot_qpos_before[ARM_DOF:]
            )
        )
    )
    if arm_response <= 0.0:
        raise RuntimeError("RM65 did not respond to the smoke target")
    if hand_response <= 0.0:
        raise RuntimeError("SG100 did not respond to the smoke target")

    _, bottle_qpos_after = bottle_freejoint_qpos(
        scene_model,
        scene_data,
    )
    if not np.all(np.isfinite(bottle_qpos_after)):
        raise RuntimeError("Bottle freejoint state is non-finite")
    np.testing.assert_allclose(
        np.linalg.norm(bottle_qpos_after[3:7]),
        1.0,
        atol=1e-10,
    )

    print(
        "SCENE Q17 CONTROL SMOKE: "
        f"arm_response={arm_response:.9f}, "
        f"hand_response={hand_response:.9f}, "
        f"bottle_quat_norm="
        f"{np.linalg.norm(bottle_qpos_after[3:7]):.12f}: PASS"
    )


if __name__ == "__main__":
    main()
