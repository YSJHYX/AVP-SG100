from pathlib import Path
import math

import mujoco
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]

MODEL_PATH = (
    REPO_ROOT
    / "sg100_teleop"
    / "assets"
    / "rm65_sg100"
    / "mjcf"
    / "rm65_sg100_sim.xml"
)


ARM_DOF = 6
HAND_DOF = 11


def body_pose(model, data, name):
    body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        name,
    )

    if body_id < 0:
        raise RuntimeError(f"Body not found: {name}")

    p = data.xpos[body_id].copy()

    R = (
        data.xmat[body_id]
        .reshape(3, 3)
        .copy()
    )

    return body_id, p, R


def relative_pose(p_a, R_a, p_b, R_b):
    """
    Pose of B expressed in frame A.
    """
    p_rel = R_a.T @ (p_b - p_a)
    R_rel = R_a.T @ R_b

    return p_rel, R_rel


def rotation_angle(R):
    value = (
        np.trace(R) - 1.0
    ) / 2.0

    value = np.clip(
        value,
        -1.0,
        1.0,
    )

    return math.acos(value)


def set_configuration(data, q):
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0

    data.qpos[:] = q


def main():
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )

    data = mujoco.MjData(model)

    assert model.nq == 17
    assert model.nv == 17

    link6_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "link_6",
    )

    sg100_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "sg100_base_link",
    )

    if link6_id < 0 or sg100_id < 0:
        raise RuntimeError(
            "Required body not found"
        )

    print("===== MODEL STRUCTURE =====")
    print("link_6 body id       =", link6_id)
    print("sg100_base body id   =", sg100_id)
    print(
        "sg100 parent body id =",
        model.body_parentid[sg100_id],
    )

    if model.body_parentid[sg100_id] != link6_id:
        raise RuntimeError(
            "sg100_base_link is not a direct child "
            "of link_6"
        )

    print(
        "Hierarchy:",
        "link_6 -> sg100_base_link",
    )
    print("Hierarchy check: PASS")

    # --------------------------------------------------------
    # Several representative RM65 configurations.
    # --------------------------------------------------------
    configurations = {
        "ZERO": np.array([
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]),

        "POSE_A": np.array([
             0.30,
            -0.40,
             0.35,
            -0.25,
             0.30,
             0.20,
        ]),

        "POSE_B": np.array([
            -0.45,
             0.30,
            -0.25,
             0.40,
            -0.35,
             0.50,
        ]),

        "WRIST_ROTATED": np.array([
             0.20,
            -0.30,
             0.20,
             0.60,
            -0.45,
             1.20,
        ]),
    }

    position_errors = []
    rotation_errors = []

    print("\n===== FK / TCP TEST =====")

    reference_tcp_by_arm_pose = {}

    for name, arm_q in configurations.items():

        q = np.zeros(17)
        q[:ARM_DOF] = arm_q

        set_configuration(
            data,
            q,
        )

        mujoco.mj_forward(
            model,
            data,
        )

        _, p_link6, R_link6 = body_pose(
            model,
            data,
            "link_6",
        )

        _, p_tcp, R_tcp = body_pose(
            model,
            data,
            "sg100_base_link",
        )

        p_rel, R_rel = relative_pose(
            p_link6,
            R_link6,
            p_tcp,
            R_tcp,
        )

        pos_error = np.linalg.norm(
            p_rel
        )

        rot_error = rotation_angle(
            R_rel
        )

        position_errors.append(
            pos_error
        )

        rotation_errors.append(
            rot_error
        )

        reference_tcp_by_arm_pose[name] = (
            p_tcp.copy(),
            R_tcp.copy(),
        )

        print(
            "\n----------------------------------------"
        )
        print(name)
        print("----------------------------------------")

        print("arm q:")
        print(arm_q)

        print("\nTCP world position [m]:")
        print(p_tcp)

        print("\nTCP world rotation:")
        print(R_tcp)

        print("\nTCP +X axis in world:")
        print(R_tcp[:, 0])

        print("TCP +Y axis in world:")
        print(R_tcp[:, 1])

        print("TCP +Z axis in world:")
        print(R_tcp[:, 2])

        print(
            "\nlink_6 -> sg100_base translation [m]:"
        )
        print(p_rel)

        print(
            "link_6 -> sg100_base rotation error [deg]:",
            np.degrees(rot_error),
        )

    # --------------------------------------------------------
    # Important semantic test:
    #
    # Hand articulation must NOT change teleop_tcp.
    # --------------------------------------------------------
    print(
        "\n===== HAND-INDEPENDENCE TEST ====="
    )

    arm_q = configurations["POSE_A"]

    q_zero_hand = np.zeros(17)
    q_zero_hand[:6] = arm_q

    set_configuration(
        data,
        q_zero_hand,
    )

    mujoco.mj_forward(
        model,
        data,
    )

    _, p_before, R_before = body_pose(
        model,
        data,
        "sg100_base_link",
    )

    q_hand = q_zero_hand.copy()

    q_hand[6:] = np.array([
         0.50,
        -0.50,
         0.40,
         0.25,
         0.60,
         0.40,
        -0.60,
         0.40,
        -0.50,
         0.60,
         0.40,
    ])

    set_configuration(
        data,
        q_hand,
    )

    mujoco.mj_forward(
        model,
        data,
    )

    _, p_after, R_after = body_pose(
        model,
        data,
        "sg100_base_link",
    )

    hand_position_effect = np.linalg.norm(
        p_after - p_before
    )

    R_delta = R_before.T @ R_after

    hand_rotation_effect = rotation_angle(
        R_delta
    )

    print(
        "TCP position change from hand motion [m] =",
        hand_position_effect,
    )

    print(
        "TCP rotation change from hand motion [deg] =",
        np.degrees(hand_rotation_effect),
    )

    # --------------------------------------------------------
    # Final assertions.
    # --------------------------------------------------------
    max_pos_error = max(
        position_errors
    )

    max_rot_error = max(
        rotation_errors
    )

    print("\n===== SUMMARY =====")

    print(
        "max link6->TCP translation error [m] =",
        max_pos_error,
    )

    print(
        "max link6->TCP rotation error [deg] =",
        np.degrees(max_rot_error),
    )

    print(
        "hand-induced TCP position change [m] =",
        hand_position_effect,
    )

    print(
        "hand-induced TCP rotation change [deg] =",
        np.degrees(hand_rotation_effect),
    )

    POSITION_TOL = 1e-10
    ROTATION_TOL = 1e-7

    if max_pos_error > POSITION_TOL:
        raise RuntimeError(
            "link_6 and teleop_tcp origins are not coincident"
        )

    if max_rot_error > ROTATION_TOL:
        raise RuntimeError(
            "link_6 and teleop_tcp orientations are not identical"
        )

    if hand_position_effect > POSITION_TOL:
        raise RuntimeError(
            "Hand articulation incorrectly changes teleop_tcp position"
        )

    if hand_rotation_effect > ROTATION_TOL:
        raise RuntimeError(
            "Hand articulation incorrectly changes teleop_tcp orientation"
        )

    print("\nFK/TCP VALIDATION: PASS")


if __name__ == "__main__":
    main()
