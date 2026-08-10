from __future__ import annotations

import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from sg100_teleop.kinematics.rm65_ik import (
    RM65IKSolver,
    so3_log,
)
from sg100_teleop.tracking.avp_wrist_mapper import (
    AVPWristRelativeMapper,
    make_transform,
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
TOTAL_DOF = 17

PHYSICS_DT = 0.002
PHYSICS_PER_TRACKING_FRAME = 8

TRACKING_DT = (
    PHYSICS_DT
    * PHYSICS_PER_TRACKING_FRAME
)

TRACKING_HZ = 1.0 / TRACKING_DT


def rot_z(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)

    return np.array(
        [
            [c, -s, 0.0],
            [s,  c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def tcp_pose(
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


def fake_avp_delta(
    t: float,
):
    """
    Synthetic relative AVP wrist motion.

    Timeline:
        0-1 s   neutral
        1-5 s   local X oscillation
        5-9 s   local Y oscillation
        9-13 s  local Z wrist rotation
        13-19 s combined X/Y/rotation

    These amplitudes are deliberately conservative because
    this project only needs to validate the AVP->MuJoCo
    teleoperation chain.
    """

    p = np.zeros(
        3,
        dtype=np.float64,
    )

    R = np.eye(
        3,
        dtype=np.float64,
    )

    phase_name = "NEUTRAL"

    if 1.0 <= t < 5.0:
        phase_name = "LOCAL_X"

        u = (
            2.0
            * math.pi
            * (t - 1.0)
            / 4.0
        )

        # Human-side amplitude = 20 mm.
        p[0] = (
            0.020
            * math.sin(u)
        )

    elif 5.0 <= t < 9.0:
        phase_name = "LOCAL_Y"

        u = (
            2.0
            * math.pi
            * (t - 5.0)
            / 4.0
        )

        # Human-side amplitude = 16 mm.
        p[1] = (
            0.016
            * math.sin(u)
        )

    elif 9.0 <= t < 13.0:
        phase_name = "LOCAL_RZ"

        u = (
            2.0
            * math.pi
            * (t - 9.0)
            / 4.0
        )

        angle = (
            math.radians(5.0)
            * math.sin(u)
        )

        R = rot_z(angle)

    elif 13.0 <= t < 19.0:
        phase_name = "COMBINED"

        u = (
            2.0
            * math.pi
            * (t - 13.0)
            / 6.0
        )

        p[0] = (
            0.016
            * math.sin(u)
        )

        p[1] = (
            0.012
            * math.sin(2.0 * u)
        )

        angle = (
            math.radians(4.0)
            * math.sin(u)
        )

        R = rot_z(angle)

    return (
        make_transform(
            p,
            R,
        ),
        phase_name,
    )


def main():
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )

    data = mujoco.MjData(model)

    if model.nq != TOTAL_DOF:
        raise RuntimeError(
            f"Expected nq={TOTAL_DOF}, "
            f"got {model.nq}"
        )

    if model.nu != TOTAL_DOF:
        raise RuntimeError(
            f"Expected nu={TOTAL_DOF}, "
            f"got {model.nu}"
        )

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
    # Initial RM65 configuration.
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

    p_tcp0, R_tcp0 = solver.fk(
        q_initial
    )

    T_tcp0 = make_transform(
        p_tcp0,
        R_tcp0,
    )

    # ---------------------------------------------------------
    # Fake initial AVP wrist pose.
    #
    # Deliberately non-zero to prove that calibration removes
    # dependence on the absolute AVP world position.
    # ---------------------------------------------------------
    T_avp0 = make_transform(
        np.array(
            [
                1.10,
                0.25,
                1.20,
            ],
            dtype=np.float64,
        ),
        np.eye(
            3,
            dtype=np.float64,
        ),
    )

    # 0.5:
    # 20 mm fake human motion -> 10 mm robot motion.
    mapper = AVPWristRelativeMapper(
        translation_scale=0.5,
        R_tcp_from_avp_local=np.eye(3),
    )

    mapper.calibrate(
        T_avp0,
        T_tcp0,
    )

    # ---------------------------------------------------------
    # Initialize simulation.
    # ---------------------------------------------------------
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0

    data.qpos[:ARM_DOF] = q_initial
    data.ctrl[:ARM_DOF] = q_initial

    # Hand stays neutral in E5-A.
    data.qpos[ARM_DOF:] = 0.0
    data.ctrl[ARM_DOF:] = 0.0

    mujoco.mj_forward(
        model,
        data,
    )

    q_arm_goal = q_initial.copy()

    DURATION = 19.0

    frame_count = int(
        DURATION / TRACKING_DT
    )

    ik_failures = 0
    max_joint_step = 0.0

    max_sim_position_error = 0.0
    max_sim_rotation_error = 0.0

    min_sigma = float("inf")
    max_condition = 0.0

    max_hand_qpos = 0.0

    last_phase = None

    print(
        "===== FAKE AVP ARM TELEOP ====="
    )

    print(
        "physics rate  =",
        1.0 / PHYSICS_DT,
        "Hz",
    )

    print(
        "tracking rate =",
        TRACKING_HZ,
        "Hz",
    )

    print(
        "translation scale =",
        mapper.translation_scale,
    )

    print(
        "duration =",
        DURATION,
        "s",
    )

    print(
        "\nSequence:"
    )

    print(
        "  NEUTRAL"
    )

    print(
        "  LOCAL_X"
    )

    print(
        "  LOCAL_Y"
    )

    print(
        "  LOCAL_RZ"
    )

    print(
        "  COMBINED"
    )

    with mujoco.viewer.launch_passive(
        model,
        data,
    ) as viewer:

        viewer.cam.distance = 1.4
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20

        viewer.cam.lookat[:] = [
            0.0,
            0.0,
            0.45,
        ]

        for frame in range(
            frame_count
        ):
            if not viewer.is_running():
                break

            frame_start = (
                time.perf_counter()
            )

            t = (
                frame
                * TRACKING_DT
            )

            (
                delta_avp,
                phase_name,
            ) = fake_avp_delta(t)

            if phase_name != last_phase:
                print(
                    f"\n[{t:5.2f}s] "
                    f"{phase_name}"
                )

                last_phase = (
                    phase_name
                )

            # -----------------------------------------------
            # Fake AVP absolute wrist pose.
            #
            # This matches the semantics expected from the
            # future VisionProStreamer wrist transform.
            # -----------------------------------------------
            T_avp = (
                T_avp0
                @ delta_avp
            )

            # -----------------------------------------------
            # AVP relative pose -> TCP target.
            # -----------------------------------------------
            T_tcp_target = mapper.map(
                T_avp
            )

            p_target = (
                T_tcp_target[
                    :3,
                    3,
                ]
            )

            R_target = (
                T_tcp_target[
                    :3,
                    :3,
                ]
            )

            q_previous = (
                q_arm_goal.copy()
            )

            # -----------------------------------------------
            # TCP target -> RM65 IK.
            # -----------------------------------------------
            (
                q_candidate,
                result,
            ) = solver.solve(
                q_seed=q_arm_goal,
                target_position=p_target,
                target_rotation=R_target,
            )

            if result.converged:
                q_arm_goal = (
                    q_candidate
                )

                joint_step = float(
                    np.max(
                        np.abs(
                            q_arm_goal
                            - q_previous
                        )
                    )
                )

                max_joint_step = max(
                    max_joint_step,
                    joint_step,
                )

                if np.isfinite(
                    result.sigma_min
                ):
                    min_sigma = min(
                        min_sigma,
                        result.sigma_min,
                    )

                if np.isfinite(
                    result.condition_number
                ):
                    max_condition = max(
                        max_condition,
                        result.condition_number,
                    )

            else:
                # Simulation-only fallback:
                # keep the last valid joint target.
                ik_failures += 1

            # -----------------------------------------------
            # Position-control target.
            # -----------------------------------------------
            data.ctrl[:ARM_DOF] = (
                q_arm_goal
            )

            # E5-A:
            # SG100 finger commands stay zero.
            data.ctrl[ARM_DOF:] = 0.0

            # -----------------------------------------------
            # 500 Hz physics.
            # -----------------------------------------------
            for _ in range(
                PHYSICS_PER_TRACKING_FRAME
            ):
                mujoco.mj_step(
                    model,
                    data,
                )

            # -----------------------------------------------
            # Actual simulated TCP.
            # -----------------------------------------------
            (
                p_actual,
                R_actual,
            ) = tcp_pose(
                model,
                data,
                tcp_body_id,
            )

            (
                pos_error,
                rot_error,
            ) = pose_error(
                p_target,
                R_target,
                p_actual,
                R_actual,
            )

            max_sim_position_error = max(
                max_sim_position_error,
                pos_error,
            )

            max_sim_rotation_error = max(
                max_sim_rotation_error,
                rot_error,
            )

            max_hand_qpos = max(
                max_hand_qpos,
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

            # Render once per fake tracking frame:
            # 62.5 Hz, not 500 Hz.
            viewer.sync()

            # -----------------------------------------------
            # Real-time pacing.
            # -----------------------------------------------
            elapsed = (
                time.perf_counter()
                - frame_start
            )

            remaining = (
                TRACKING_DT
                - elapsed
            )

            if remaining > 0:
                time.sleep(
                    remaining
                )

        print(
            "\n========================================"
        )

        print(
            "SUMMARY"
        )

        print(
            "========================================"
        )

        print(
            "IK failures =",
            ik_failures,
        )

        print(
            "max q_goal step/frame =",
            max_joint_step,
            "rad",
        )

        print(
            "min sigma_min =",
            min_sigma,
        )

        print(
            "max condition =",
            max_condition,
        )

        print(
            "max simulated TCP error =",
            max_sim_position_error
            * 1000.0,
            "mm",
        )

        print(
            "max simulated orientation error =",
            math.degrees(
                max_sim_rotation_error
            ),
            "deg",
        )

        print(
            "max hand |qpos| =",
            max_hand_qpos,
            "rad",
        )

        if ik_failures == 0:
            print(
                "\nFAKE AVP ARM TELEOP: PASS"
            )
        else:
            print(
                "\nFAKE AVP ARM TELEOP: "
                "COMPLETED WITH IK HOLDS"
            )

        print(
            "\nClose viewer to exit."
        )

        while viewer.is_running():
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    main()
