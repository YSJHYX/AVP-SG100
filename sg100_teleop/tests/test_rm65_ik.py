from pathlib import Path
import math

import mujoco
import numpy as np

from sg100_teleop.kinematics.rm65_ik import (
    RM65IKSolver,
    so3_log,
)


REPO_ROOT = Path(__file__).resolve().parents[2]

MODEL_PATH = (
    REPO_ROOT
    / "sg100_teleop"
    / "assets"
    / "rm65_sg100"
    / "mjcf"
    / "rm65_sg100_sim.xml"
)


def skew(axis):
    x, y, z = axis

    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=np.float64,
    )


def axis_angle_rotation(
    axis,
    angle,
):
    axis = np.asarray(
        axis,
        dtype=np.float64,
    )

    axis /= np.linalg.norm(axis)

    K = skew(axis)

    return (
        np.eye(3)
        + math.sin(angle) * K
        + (
            1.0 - math.cos(angle)
        ) * (K @ K)
    )


def evaluate_pose_error(
    solver,
    q,
    p_target,
    R_target,
):
    p_actual, R_actual = solver.fk(q)

    position_error = np.linalg.norm(
        p_target - p_actual
    )

    R_error = (
        R_target
        @ R_actual.T
    )

    rotation_error = np.linalg.norm(
        so3_log(R_error)
    )

    return (
        position_error,
        rotation_error,
        p_actual,
        R_actual,
    )


def main():
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )

    solver = RM65IKSolver(
        model,
        tcp_body_name="sg100_base_link",
    )

    # Already validated FK/Jacobian pose.
    q_seed = np.array(
        [
             0.30,
            -0.40,
             0.35,
            -0.25,
             0.30,
             0.20,
        ],
        dtype=np.float64,
    )

    p0, R0 = solver.fk(
        q_seed
    )

    print("===== RM65 6D IK VALIDATION =====")

    print("\nSeed q:")
    print(q_seed)

    print("\nSeed TCP position:")
    print(p0)

    print("\nSeed TCP rotation:")
    print(R0)

    TRANSLATION = 0.020  # 20 mm
    ROTATION = math.radians(5.0)

    tests = []

    # ---------------------------------------------------------
    # World-frame translations
    # ---------------------------------------------------------
    for name, direction in [
        (
            "TRANSLATE_WORLD_X",
            np.array([1.0, 0.0, 0.0]),
        ),
        (
            "TRANSLATE_WORLD_Y",
            np.array([0.0, 1.0, 0.0]),
        ),
        (
            "TRANSLATE_WORLD_Z",
            np.array([0.0, 0.0, 1.0]),
        ),
    ]:
        tests.append(
            (
                name,
                p0 + TRANSLATION * direction,
                R0.copy(),
            )
        )

    # ---------------------------------------------------------
    # World-frame rotations
    #
    # Left multiplication means rotation about a world axis.
    # ---------------------------------------------------------
    for name, axis in [
        (
            "ROTATE_WORLD_X",
            np.array([1.0, 0.0, 0.0]),
        ),
        (
            "ROTATE_WORLD_Y",
            np.array([0.0, 1.0, 0.0]),
        ),
        (
            "ROTATE_WORLD_Z",
            np.array([0.0, 0.0, 1.0]),
        ),
    ]:
        R_delta = axis_angle_rotation(
            axis,
            ROTATION,
        )

        tests.append(
            (
                name,
                p0.copy(),
                R_delta @ R0,
            )
        )

    failures = []

    for (
        name,
        p_target,
        R_target,
    ) in tests:

        print(
            "\n"
            + "=" * 70
        )

        print(name)

        print(
            "=" * 70
        )

        q_solution, result = solver.solve(
            q_seed=q_seed,
            target_position=p_target,
            target_rotation=R_target,
        )

        (
            pos_error,
            rot_error,
            p_actual,
            R_actual,
        ) = evaluate_pose_error(
            solver,
            q_solution,
            p_target,
            R_target,
        )

        print("\nconverged:")
        print(result.converged)

        print("\niterations:")
        print(result.iterations)

        print("\nq solution:")
        print(q_solution)

        print("\ndelta q:")
        print(
            q_solution - q_seed
        )

        print("\ntarget position:")
        print(p_target)

        print("\nachieved position:")
        print(p_actual)

        print(
            "\nposition error [m] =",
            pos_error,
        )

        print(
            "position error [mm] =",
            pos_error * 1000.0,
        )

        print(
            "rotation error [rad] =",
            rot_error,
        )

        print(
            "rotation error [deg] =",
            math.degrees(rot_error),
        )

        print(
            "\nsigma_min =",
            result.sigma_min,
        )

        print(
            "condition =",
            result.condition_number,
        )

        print(
            "damping   =",
            result.damping,
        )

        within_lower = np.all(
            q_solution
            >= solver.lower
        )

        within_upper = np.all(
            q_solution
            <= solver.upper
        )

        print(
            "within joint limits =",
            within_lower
            and within_upper,
        )

        # Independent test tolerances, slightly looser than
        # the internal solver stopping tolerance.
        position_pass = (
            pos_error < 2e-4
        )

        rotation_pass = (
            rot_error < 2e-3
        )

        test_pass = (
            result.converged
            and position_pass
            and rotation_pass
            and within_lower
            and within_upper
        )

        print(
            "\nTEST RESULT:",
            "PASS" if test_pass else "FAIL",
        )

        if not test_pass:
            failures.append(name)

    print(
        "\n"
        + "=" * 70
    )

    print("SUMMARY")

    print(
        "=" * 70
    )

    print(
        "tests:",
        len(tests),
    )

    print(
        "failures:",
        failures,
    )

    if failures:
        raise RuntimeError(
            "IK validation failed: "
            + ", ".join(failures)
        )

    print(
        "\nRM65 6D IK VALIDATION: PASS"
    )


if __name__ == "__main__":
    main()
