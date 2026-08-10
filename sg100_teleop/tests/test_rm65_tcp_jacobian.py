from pathlib import Path
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

TCP_BODY = "sg100_base_link"

ARM_DOF = 6
TOTAL_DOF = 17

EPS = 1e-6


def get_tcp_pose(model, data, body_id):
    p = data.xpos[body_id].copy()
    R = data.xmat[body_id].reshape(3, 3).copy()
    return p, R


def so3_log(R):
    """
    Rotation matrix -> rotation vector.

    Output is expressed in the same/world coordinate frame as R.
    Suitable here for small finite-difference rotations.
    """
    cos_theta = (
        np.trace(R) - 1.0
    ) / 2.0

    cos_theta = np.clip(
        cos_theta,
        -1.0,
        1.0,
    )

    theta = np.arccos(cos_theta)

    if theta < 1e-10:
        return 0.5 * np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ])

    return (
        theta
        / (2.0 * np.sin(theta))
        * np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ])
    )


def set_q(model, data, q):
    data.qpos[:] = q
    data.qvel[:] = 0.0

    mujoco.mj_forward(
        model,
        data,
    )


def analytic_jacobian(model, data, body_id):
    """
    Jacobian of TCP body-frame origin in world coordinates.
    """
    jacp = np.zeros(
        (3, model.nv),
        dtype=np.float64,
    )

    jacr = np.zeros(
        (3, model.nv),
        dtype=np.float64,
    )

    # Exact TCP point = sg100_base_link frame origin.
    point = data.xpos[body_id].copy()

    mujoco.mj_jac(
        model,
        data,
        jacp,
        jacr,
        point,
        body_id,
    )

    return jacp, jacr


def finite_difference_column(
    model,
    data,
    body_id,
    q0,
    joint_index,
):
    """
    Central finite difference for one joint.

    Position:
        dp/dq

    Orientation:
        world-frame angular change / dq
    """

    q_plus = q0.copy()
    q_minus = q0.copy()

    q_plus[joint_index] += EPS
    q_minus[joint_index] -= EPS

    set_q(
        model,
        data,
        q_plus,
    )

    p_plus, R_plus = get_tcp_pose(
        model,
        data,
        body_id,
    )

    set_q(
        model,
        data,
        q_minus,
    )

    p_minus, R_minus = get_tcp_pose(
        model,
        data,
        body_id,
    )

    # --------------------------------------------------------
    # Translational finite difference
    # --------------------------------------------------------
    dp = (
        p_plus - p_minus
    ) / (2.0 * EPS)

    # --------------------------------------------------------
    # Rotational finite difference
    #
    # R_plus * R_minus^T is the relative rotation represented
    # in the world frame.
    # --------------------------------------------------------
    R_delta = (
        R_plus
        @ R_minus.T
    )

    drot = (
        so3_log(R_delta)
        / (2.0 * EPS)
    )

    return dp, drot


def test_pose(
    model,
    data,
    body_id,
    name,
    arm_q,
):
    q0 = np.zeros(
        TOTAL_DOF,
        dtype=np.float64,
    )

    q0[:ARM_DOF] = arm_q

    set_q(
        model,
        data,
        q0,
    )

    jacp, jacr = analytic_jacobian(
        model,
        data,
        body_id,
    )

    J_analytic = np.vstack([
        jacp[:, :ARM_DOF],
        jacr[:, :ARM_DOF],
    ])

    J_fd = np.zeros(
        (6, ARM_DOF),
        dtype=np.float64,
    )

    for j in range(ARM_DOF):
        dp, drot = finite_difference_column(
            model,
            data,
            body_id,
            q0,
            j,
        )

        J_fd[:3, j] = dp
        J_fd[3:, j] = drot

    # Restore reference configuration.
    set_q(
        model,
        data,
        q0,
    )

    error = (
        J_analytic - J_fd
    )

    max_abs_error = np.max(
        np.abs(error)
    )

    position_error = np.max(
        np.abs(error[:3])
    )

    rotation_error = np.max(
        np.abs(error[3:])
    )

    # --------------------------------------------------------
    # Check SG100 joint influence.
    # TCP is above all hand joints in the tree, so columns
    # corresponding to q[6:17] must be exactly zero.
    # --------------------------------------------------------
    hand_jac = np.vstack([
        jacp[:, ARM_DOF:],
        jacr[:, ARM_DOF:],
    ])

    hand_max = np.max(
        np.abs(hand_jac)
    )

    # Singular values are useful for the next IK stage.
    singular_values = np.linalg.svd(
        J_analytic,
        compute_uv=False,
    )

    condition_number = (
        singular_values[0]
        / singular_values[-1]
        if singular_values[-1] > 1e-12
        else np.inf
    )

    print(
        "\n============================================================"
    )
    print(name)
    print(
        "============================================================"
    )

    print("arm q:")
    print(arm_q)

    print("\nAnalytic J:")
    print(
        np.array2string(
            J_analytic,
            precision=6,
            suppress_small=True,
        )
    )

    print("\nFinite-difference J:")
    print(
        np.array2string(
            J_fd,
            precision=6,
            suppress_small=True,
        )
    )

    print("\nError:")
    print(
        np.array2string(
            error,
            precision=3,
            suppress_small=False,
        )
    )

    print("\nmax position error =", position_error)
    print("max rotation error =", rotation_error)
    print("max total error    =", max_abs_error)

    print(
        "hand Jacobian max  =",
        hand_max,
    )

    print(
        "singular values    =",
        singular_values,
    )

    print(
        "condition number   =",
        condition_number,
    )

    return {
        "max_error": max_abs_error,
        "position_error": position_error,
        "rotation_error": rotation_error,
        "hand_max": hand_max,
        "condition": condition_number,
    }


def main():
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )

    data = mujoco.MjData(model)

    if model.nq != TOTAL_DOF:
        raise RuntimeError(
            f"Expected nq={TOTAL_DOF}, got {model.nq}"
        )

    if model.nv != TOTAL_DOF:
        raise RuntimeError(
            f"Expected nv={TOTAL_DOF}, got {model.nv}"
        )

    body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        TCP_BODY,
    )

    if body_id < 0:
        raise RuntimeError(
            f"TCP body not found: {TCP_BODY}"
        )

    print("===== RM65 TCP JACOBIAN VALIDATION =====")
    print("model:", MODEL_PATH)
    print("TCP:", TCP_BODY)
    print("TCP body id:", body_id)

    poses = {
        # Avoid using only q=0 because a serial arm can be
        # singular or near-singular at special configurations.
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

    results = []

    for name, q in poses.items():
        results.append(
            test_pose(
                model,
                data,
                body_id,
                name,
                q,
            )
        )

    max_jac_error = max(
        r["max_error"]
        for r in results
    )

    max_hand_jac = max(
        r["hand_max"]
        for r in results
    )

    print(
        "\n============================================================"
    )
    print("SUMMARY")
    print(
        "============================================================"
    )

    print(
        "maximum analytic-vs-FD error =",
        max_jac_error,
    )

    print(
        "maximum hand Jacobian entry  =",
        max_hand_jac,
    )

    # A tolerance of 1e-5 is deliberately much looser than
    # the expected numerical error, while still strong enough
    # to expose wrong frame/order/Jacobian conventions.
    JAC_TOL = 1e-5
    HAND_TOL = 1e-12

    if max_jac_error > JAC_TOL:
        raise RuntimeError(
            "TCP analytic Jacobian does not match "
            "finite-difference Jacobian"
        )

    if max_hand_jac > HAND_TOL:
        raise RuntimeError(
            "SG100 joints incorrectly affect teleop TCP Jacobian"
        )

    print("\nTCP JACOBIAN VALIDATION: PASS")


if __name__ == "__main__":
    main()
