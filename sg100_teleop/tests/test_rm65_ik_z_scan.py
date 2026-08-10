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


def evaluate(
    solver,
    q,
    p_target,
    R_target,
):
    p, R = solver.fk(q)

    pos_err = float(
        np.linalg.norm(
            p_target - p
        )
    )

    rot_err = float(
        np.linalg.norm(
            so3_log(
                R_target @ R.T
            )
        )
    )

    return pos_err, rot_err, p


def run_case(
    solver,
    q_seed,
    p_target,
    R_target,
):
    q, result = solver.solve(
        q_seed=q_seed,
        target_position=p_target,
        target_rotation=R_target,
    )

    pos_err, rot_err, p = evaluate(
        solver,
        q,
        p_target,
        R_target,
    )

    return (
        q,
        result,
        pos_err,
        rot_err,
        p,
    )


def print_result(
    dz_mm,
    q,
    result,
    pos_err,
    rot_err,
    p,
):
    print(
        f"{dz_mm:+6.1f} mm | "
        f"conv={str(result.converged):5s} | "
        f"pos={pos_err*1000:8.3f} mm | "
        f"rot={math.degrees(rot_err):8.3f} deg | "
        f"sigma={result.sigma_min:10.3e} | "
        f"cond={result.condition_number:12.3e} | "
        f"q3={q[2]:+9.5f} | "
        f"z={p[2]:.6f}"
    )


def main():
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )

    solver = RM65IKSolver(
        model,
        tcp_body_name="sg100_base_link",
    )

    q0 = np.array(
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

    p0, R0 = solver.fk(q0)

    print("===== RM65 WORLD-Z IK SCAN =====")
    print("seed q =", q0)
    print("seed p =", p0)

    # ---------------------------------------------------------
    # DIRECT:
    # every target starts from the same original configuration.
    # ---------------------------------------------------------
    print("\n===== DIRECT FROM ORIGINAL SEED =====")

    dz_values_mm = [
        -20,
        -10,
        -5,
        5,
        10,
        15,
        20,
    ]

    for dz_mm in dz_values_mm:
        p_target = p0.copy()
        p_target[2] += dz_mm / 1000.0

        (
            q,
            result,
            pos_err,
            rot_err,
            p,
        ) = run_case(
            solver,
            q0,
            p_target,
            R0,
        )

        print_result(
            dz_mm,
            q,
            result,
            pos_err,
            rot_err,
            p,
        )

    # ---------------------------------------------------------
    # POSITIVE-Z CONTINUATION:
    #
    # use the previous successful solution as the next seed.
    # This tests whether failure is merely due to one large
    # Cartesian step.
    # ---------------------------------------------------------
    print(
        "\n===== +Z CONTINUATION ====="
    )

    q_seed = q0.copy()

    for dz_mm in [
        2.5,
        5.0,
        7.5,
        10.0,
        12.5,
        15.0,
        17.5,
        20.0,
    ]:
        p_target = p0.copy()
        p_target[2] += dz_mm / 1000.0

        (
            q,
            result,
            pos_err,
            rot_err,
            p,
        ) = run_case(
            solver,
            q_seed,
            p_target,
            R0,
        )

        print_result(
            dz_mm,
            q,
            result,
            pos_err,
            rot_err,
            p,
        )

        if result.converged:
            q_seed = q.copy()
        else:
            print(
                "Continuation stopped at",
                dz_mm,
                "mm",
            )
            break


if __name__ == "__main__":
    main()
