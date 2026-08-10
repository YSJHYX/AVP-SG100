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

TCP_BODY = "sg100_base_link"

ARM_DOF = 6
HAND_DOF = 11
TOTAL_DOF = 17

PHYSICS_DT = 0.002       # 500 Hz
PHYSICS_PER_TARGET = 8   # 62.5 Hz target update
TARGET_DT = PHYSICS_DT * PHYSICS_PER_TARGET

DURATION = 8.0
TRAJECTORY_HZ = 0.125

X_AMPLITUDE = 0.010  # 10 mm
Y_AMPLITUDE = 0.008  # 8 mm


def pose_from_sim(
    model,
    data,
    body_id,
):
    p = data.xpos[
        body_id
    ].copy()

    R = (
        data.xmat[
            body_id
        ]
        .reshape(3, 3)
        .copy()
    )

    return p, R


def pose_error(
    p_target,
    R_target,
    p_actual,
    R_actual,
):
    pos_error = float(
        np.linalg.norm(
            p_target - p_actual
        )
    )

    R_error = (
        R_target
        @ R_actual.T
    )

    rot_error = float(
        np.linalg.norm(
            so3_log(R_error)
        )
    )

    return (
        pos_error,
        rot_error,
    )


def percentile(values, q):
    return float(
        np.percentile(
            np.asarray(values),
            q,
        )
    )


def main():
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )

    if model.nq != TOTAL_DOF:
        raise RuntimeError(
            f"Expected nq={TOTAL_DOF}, got {model.nq}"
        )

    if model.nu != TOTAL_DOF:
        raise RuntimeError(
            f"Expected nu={TOTAL_DOF}, got {model.nu}"
        )

    if abs(
        model.opt.timestep
        - PHYSICS_DT
    ) > 1e-12:
        raise RuntimeError(
            f"Expected timestep={PHYSICS_DT}, "
            f"got {model.opt.timestep}"
        )

    data = mujoco.MjData(model)

    solver = RM65IKSolver(
        model,
        tcp_body_name=TCP_BODY,
    )

    tcp_body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        TCP_BODY,
    )

    if tcp_body_id < 0:
        raise RuntimeError(
            f"TCP body not found: {TCP_BODY}"
        )

    # ---------------------------------------------------------
    # Initial validated RM65 configuration.
    # ---------------------------------------------------------
    q_initial = np.array(
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

    p_center, R_target = solver.fk(
        q_initial
    )

    # ---------------------------------------------------------
    # Initialize MuJoCo at exactly the same configuration.
    # ---------------------------------------------------------
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0

    data.qpos[:ARM_DOF] = q_initial
    data.ctrl[:ARM_DOF] = q_initial

    # Hand stays at zero for this test.
    data.qpos[ARM_DOF:] = 0.0
    data.ctrl[ARM_DOF:] = 0.0

    mujoco.mj_forward(
        model,
        data,
    )

    # Previous successful IK solution becomes next seed.
    q_ik = q_initial.copy()
    q_previous = q_initial.copy()

    target_frames = int(
        round(
            DURATION / TARGET_DT
        )
    )

    ik_failures = 0

    ik_position_errors = []
    ik_rotation_errors = []

    sim_position_errors = []
    sim_rotation_errors = []

    joint_steps = []
    sigma_mins = []
    conditions = []
    dampings = []

    max_hand_q = 0.0

    print(
        "===== CONTINUOUS TCP TELEOP REGRESSION ====="
    )

    print(
        "physics rate       =",
        1.0 / PHYSICS_DT,
        "Hz",
    )

    print(
        "target update rate =",
        1.0 / TARGET_DT,
        "Hz",
    )

    print(
        "duration           =",
        DURATION,
        "s",
    )

    print(
        "target frames      =",
        target_frames,
    )

    print(
        "trajectory freq    =",
        TRAJECTORY_HZ,
        "Hz",
    )

    print(
        "X amplitude        =",
        X_AMPLITUDE * 1000.0,
        "mm",
    )

    print(
        "Y amplitude        =",
        Y_AMPLITUDE * 1000.0,
        "mm",
    )

    print("\ninitial q:")
    print(q_initial)

    print("\nTCP center:")
    print(p_center)

    # ---------------------------------------------------------
    # Continuous target loop.
    # ---------------------------------------------------------
    for frame in range(
        target_frames
    ):
        t = (
            frame
            * TARGET_DT
        )

        phase = (
            2.0
            * math.pi
            * TRAJECTORY_HZ
            * t
        )

        # Start exactly at p_center.
        #
        # Smooth closed Lissajous-like trajectory:
        #
        # dx(0) = 0
        # dy(0) = 0
        p_target = p_center.copy()

        p_target[0] += (
            X_AMPLITUDE
            * math.sin(phase)
        )

        p_target[1] += (
            Y_AMPLITUDE
            * math.sin(
                2.0 * phase
            )
        )

        # -----------------------------------------------------
        # IK update, analogous to receiving a new AVP wrist
        # tracking frame.
        # -----------------------------------------------------
        q_candidate, result = solver.solve(
            q_seed=q_ik,
            target_position=p_target,
            target_rotation=R_target,
        )

        if result.converged:
            q_ik = q_candidate

            # -----------------------------------------------
            # Independent FK verification of IK output.
            # -----------------------------------------------
            p_ik, R_ik = solver.fk(
                q_ik
            )

            (
                ik_pos_error,
                ik_rot_error,
            ) = pose_error(
                p_target,
                R_target,
                p_ik,
                R_ik,
            )

            ik_position_errors.append(
                ik_pos_error
            )

            ik_rotation_errors.append(
                ik_rot_error
            )

            joint_step = float(
                np.max(
                    np.abs(
                        q_ik
                        - q_previous
                    )
                )
            )

            joint_steps.append(
                joint_step
            )

            q_previous = q_ik.copy()

            sigma_mins.append(
                result.sigma_min
            )

            conditions.append(
                result.condition_number
            )

            dampings.append(
                result.damping
            )

        else:
            # Simulation-only fallback:
            #
            # hold last valid arm target.
            ik_failures += 1

        # -----------------------------------------------------
        # Send current valid IK solution to MuJoCo.
        # -----------------------------------------------------
        data.ctrl[:ARM_DOF] = q_ik
        data.ctrl[ARM_DOF:] = 0.0

        # -----------------------------------------------------
        # 500 Hz physics between tracking updates.
        # -----------------------------------------------------
        for _ in range(
            PHYSICS_PER_TARGET
        ):
            mujoco.mj_step(
                model,
                data,
            )

        if not (
            np.all(
                np.isfinite(
                    data.qpos
                )
            )
            and
            np.all(
                np.isfinite(
                    data.qvel
                )
            )
        ):
            raise RuntimeError(
                f"Non-finite simulation state "
                f"at frame {frame}"
            )

        # -----------------------------------------------------
        # Measure actual simulated TCP tracking.
        # -----------------------------------------------------
        p_sim, R_sim = pose_from_sim(
            model,
            data,
            tcp_body_id,
        )

        (
            sim_pos_error,
            sim_rot_error,
        ) = pose_error(
            p_target,
            R_target,
            p_sim,
            R_sim,
        )

        sim_position_errors.append(
            sim_pos_error
        )

        sim_rotation_errors.append(
            sim_rot_error
        )

        max_hand_q = max(
            max_hand_q,
            float(
                np.max(
                    np.abs(
                        data.qpos[
                            ARM_DOF:
                        ]
                    )
                )
            ),
        )

    # ---------------------------------------------------------
    # Statistics
    # ---------------------------------------------------------
    max_ik_pos = max(
        ik_position_errors
    )

    max_ik_rot = max(
        ik_rotation_errors
    )

    max_sim_pos = max(
        sim_position_errors
    )

    max_sim_rot = max(
        sim_rotation_errors
    )

    max_joint_step = max(
        joint_steps
    )

    min_sigma = min(
        sigma_mins
    )

    max_condition = max(
        conditions
    )

    max_damping = max(
        dampings
    )

    print(
        "\n=============================================="
    )

    print("SUMMARY")

    print(
        "=============================================="
    )

    print(
        "IK failures =",
        ik_failures,
        "/",
        target_frames,
    )

    print(
        "\nIK position error:"
    )

    print(
        "  max  =",
        max_ik_pos * 1000.0,
        "mm",
    )

    print(
        "  p95  =",
        percentile(
            ik_position_errors,
            95,
        ) * 1000.0,
        "mm",
    )

    print(
        "\nIK rotation error:"
    )

    print(
        "  max  =",
        math.degrees(
            max_ik_rot
        ),
        "deg",
    )

    print(
        "  p95  =",
        math.degrees(
            percentile(
                ik_rotation_errors,
                95,
            )
        ),
        "deg",
    )

    print(
        "\nSIM TCP position tracking:"
    )

    print(
        "  max  =",
        max_sim_pos * 1000.0,
        "mm",
    )

    print(
        "  p95  =",
        percentile(
            sim_position_errors,
            95,
        ) * 1000.0,
        "mm",
    )

    print(
        "\nSIM TCP rotation tracking:"
    )

    print(
        "  max  =",
        math.degrees(
            max_sim_rot
        ),
        "deg",
    )

    print(
        "  p95  =",
        math.degrees(
            percentile(
                sim_rotation_errors,
                95,
            )
        ),
        "deg",
    )

    print(
        "\nmax q-goal step/frame =",
        max_joint_step,
        "rad",
    )

    print(
        "min sigma_min       =",
        min_sigma,
    )

    print(
        "max condition       =",
        max_condition,
    )

    print(
        "max damping         =",
        max_damping,
    )

    print(
        "max hand |qpos|     =",
        max_hand_q,
    )

    # ---------------------------------------------------------
    # Regression criteria
    # ---------------------------------------------------------
    if ik_failures != 0:
        raise RuntimeError(
            f"Continuous IK had "
            f"{ik_failures} failed frames"
        )

    if max_ik_pos > 2e-4:
        raise RuntimeError(
            "Continuous IK position error "
            "exceeded 0.2 mm"
        )

    if max_ik_rot > 2e-3:
        raise RuntimeError(
            "Continuous IK rotation error "
            "exceeded 0.002 rad"
        )

    # This is not an industrial servo benchmark.
    # It only verifies that the current validation actuators
    # can follow a slowly varying teleop target without
    # pathological lag.
    if max_sim_pos > 5e-3:
        raise RuntimeError(
            "Simulated TCP lag exceeded 5 mm"
        )

    if max_sim_rot > math.radians(1.0):
        raise RuntimeError(
            "Simulated TCP orientation lag "
            "exceeded 1 degree"
        )

    if max_joint_step > 0.05:
        raise RuntimeError(
            "IK joint target has a discontinuous "
            "frame-to-frame jump > 0.05 rad"
        )

    HAND_DYNAMIC_TOL = 1e-3

    if max_hand_q > HAND_DYNAMIC_TOL:
        raise RuntimeError(
            "SG100 dynamic deflection exceeded "
            "1e-3 rad during arm-only teleop"
        )

    print(
        "\nCONTINUOUS TCP TELEOP: PASS"
    )


if __name__ == "__main__":
    main()
