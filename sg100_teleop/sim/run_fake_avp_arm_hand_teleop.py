from __future__ import annotations

import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from sg100_teleop.hand.canonical_hand import (
    CanonicalHandState,
)
from sg100_teleop.kinematics.rm65_ik import (
    RM65IKSolver,
    so3_log,
)
from sg100_teleop.retargeting.sg100_retargeter import (
    SG100Retargeter,
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
HAND_DOF = 11
TOTAL_DOF = 17

PHYSICS_DT = 0.002
PHYSICS_PER_TRACKING_FRAME = 8
TRACKING_DT = PHYSICS_DT * PHYSICS_PER_TRACKING_FRAME
TRACKING_HZ = 1.0 / TRACKING_DT

DURATION = 20.0


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

    return pos_error, rot_error


def smooth_cycle(u: float) -> float:
    """
    0 -> 1 -> 0 smoothly over u in [0,1].
    """
    u = float(
        np.clip(u, 0.0, 1.0)
    )

    return 0.5 * (
        1.0
        - math.cos(
            2.0 * math.pi * u
        )
    )


def fake_avp_wrist_delta(
    t: float,
):
    """
    Synthetic AVP wrist motion.

    0-2 s:
        neutral

    2-6 s:
        arm only

    6-10 s:
        hand only; wrist neutral

    10-16 s:
        arm + hand

    16-20 s:
        wrist rotation + open/close
    """

    p = np.zeros(
        3,
        dtype=np.float64,
    )

    R = np.eye(
        3,
        dtype=np.float64,
    )

    phase = "NEUTRAL"

    if 2.0 <= t < 6.0:
        phase = "ARM_ONLY"

        u = (
            t - 2.0
        ) / 4.0

        angle = (
            2.0
            * math.pi
            * u
        )

        # Fake human motion.
        # translation_scale=0.5 later turns:
        #
        # 20 mm -> 10 mm robot motion
        # 12 mm ->  6 mm robot motion
        p[0] = (
            0.020
            * math.sin(angle)
        )

        p[1] = (
            0.012
            * math.sin(
                2.0 * angle
            )
        )

    elif 10.0 <= t < 16.0:
        phase = "ARM_AND_HAND"

        u = (
            t - 10.0
        ) / 6.0

        angle = (
            2.0
            * math.pi
            * u
        )

        p[0] = (
            0.016
            * math.sin(angle)
        )

        p[1] = (
            0.012
            * math.sin(
                2.0 * angle
            )
        )

        R = rot_z(
            math.radians(4.0)
            * math.sin(angle)
        )

    elif 16.0 <= t < 20.0:
        phase = "WRIST_AND_HAND"

        u = (
            t - 16.0
        ) / 4.0

        angle = (
            2.0
            * math.pi
            * u
        )

        R = rot_z(
            math.radians(5.0)
            * math.sin(angle)
        )

    elif 6.0 <= t < 10.0:
        phase = "HAND_ONLY"

    return (
        make_transform(
            p,
            R,
        ),
        phase,
    )


def fake_hand_state(
    t: float,
) -> CanonicalHandState:
    """
    Fake device-independent semantic hand source.

    IMPORTANT:
        This does NOT directly generate SG100 q targets.

        It generates exactly the same CanonicalHandState that
        future AVP tracking/calibration will generate.
    """

    grasp = 0.0

    if 6.0 <= t < 10.0:
        u = (
            t - 6.0
        ) / 4.0

        grasp = smooth_cycle(u)

    elif 10.0 <= t < 16.0:
        u = (
            t - 10.0
        ) / 6.0

        # One smooth close/open cycle while arm moves.
        grasp = smooth_cycle(u)

    elif 16.0 <= t < 20.0:
        u = (
            t - 16.0
        ) / 4.0

        # Two quicker open/close cycles.
        grasp = 0.5 * (
            1.0
            - math.cos(
                4.0 * math.pi * u
            )
        )

    return CanonicalHandState(
        thumb_opposition=0.75 * grasp,
        thumb_abduction=0.25 * grasp,
        thumb_distal=0.60 * grasp,

        index_adduction=0.15 * grasp,
        index_flexion=0.80 * grasp,
        index_distal=0.75 * grasp,

        middle_flexion=0.85 * grasp,
        middle_distal=0.75 * grasp,

        little_outward=0.15 * grasp,
        little_flexion=0.80 * grasp,
        little_distal=0.75 * grasp,
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

    retargeter = SG100Retargeter()

    expected_hand_order = [
        "thumb_j1",
        "thumb_j2",
        "thumb_j3",
        "index_j1",
        "index_j2",
        "index_j3",
        "middle_j1",
        "middle_j2",
        "little_j1",
        "little_j2",
        "little_j3",
    ]

    if retargeter.joint_order != expected_hand_order:
        raise RuntimeError(
            "Unexpected SG100 retargeter joint order:\n"
            f"{retargeter.joint_order}"
        )

    # ---------------------------------------------------------
    # Verify MuJoCo q ordering matches the retargeter exactly.
    # ---------------------------------------------------------
    mujoco_hand_order = []

    for jid in range(
        ARM_DOF,
        TOTAL_DOF,
    ):
        name = mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            jid,
        )

        mujoco_hand_order.append(
            name
        )

    if (
        mujoco_hand_order
        != retargeter.joint_order
    ):
        raise RuntimeError(
            "SG100 retargeter/MuJoCo joint-order mismatch.\n"
            f"Retargeter: {retargeter.joint_order}\n"
            f"MuJoCo:     {mujoco_hand_order}"
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
    # Validated teleop-ready arm configuration.
    # ---------------------------------------------------------
    q_arm_initial = np.array(
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

    q_hand_initial = (
        retargeter.retarget(
            CanonicalHandState.neutral()
        )
    )

    p_tcp0, R_tcp0 = solver.fk(
        q_arm_initial
    )

    T_tcp0 = make_transform(
        p_tcp0,
        R_tcp0,
    )

    # ---------------------------------------------------------
    # Synthetic AVP absolute wrist pose.
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

    mapper = AVPWristRelativeMapper(
        translation_scale=0.5,
        R_tcp_from_avp_local=np.eye(
            3,
            dtype=np.float64,
        ),
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

    data.qpos[:ARM_DOF] = (
        q_arm_initial
    )

    data.ctrl[:ARM_DOF] = (
        q_arm_initial
    )

    data.qpos[ARM_DOF:] = (
        q_hand_initial
    )

    data.ctrl[ARM_DOF:] = (
        q_hand_initial
    )

    mujoco.mj_forward(
        model,
        data,
    )

    q_arm_goal = (
        q_arm_initial.copy()
    )

    q_hand_goal = (
        q_hand_initial.copy()
    )

    previous_arm_goal = (
        q_arm_goal.copy()
    )

    previous_hand_goal = (
        q_hand_goal.copy()
    )

    frame_count = int(
        DURATION / TRACKING_DT
    )

    ik_failures = 0

    max_arm_goal_step = 0.0
    max_hand_goal_step = 0.0

    max_sim_tcp_position_error = 0.0
    max_sim_tcp_rotation_error = 0.0

    max_hand_tracking_error = 0.0

    min_sigma = float("inf")
    max_condition = 0.0

    last_phase = None

    print(
        "===== FAKE AVP 17DOF TELEOP ====="
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
        "\nJoint layout:"
    )

    print(
        "  q[0:6]   = RM65"
    )

    print(
        "  q[6:17]  = SG100"
    )

    print(
        "\nHand joint order:"
    )

    for i, name in enumerate(
        retargeter.joint_order
    ):
        print(
            f"  [{i:02d}] {name}"
        )

    print(
        "\nSequence:"
    )

    print(
        "  NEUTRAL"
    )

    print(
        "  ARM_ONLY"
    )

    print(
        "  HAND_ONLY"
    )

    print(
        "  ARM_AND_HAND"
    )

    print(
        "  WRIST_AND_HAND"
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

            # =================================================
            # ARM SOURCE
            # =================================================
            (
                delta_avp,
                phase_name,
            ) = fake_avp_wrist_delta(
                t
            )

            if phase_name != last_phase:
                print(
                    f"\n[{t:5.2f}s] "
                    f"{phase_name}"
                )

                last_phase = (
                    phase_name
                )

            T_avp = (
                T_avp0
                @ delta_avp
            )

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

                arm_step = float(
                    np.max(
                        np.abs(
                            q_arm_goal
                            - previous_arm_goal
                        )
                    )
                )

                max_arm_goal_step = max(
                    max_arm_goal_step,
                    arm_step,
                )

                previous_arm_goal = (
                    q_arm_goal.copy()
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
                # Validation-only fallback.
                # Hold last valid RM65 target.
                ik_failures += 1

            # =================================================
            # HAND SOURCE
            # =================================================
            hand_state = fake_hand_state(
                t
            )

            q_hand_goal = (
                retargeter.retarget(
                    hand_state
                )
            )

            if q_hand_goal.shape != (
                HAND_DOF,
            ):
                raise RuntimeError(
                    "Retargeter output shape "
                    f"is {q_hand_goal.shape}"
                )

            hand_step = float(
                np.max(
                    np.abs(
                        q_hand_goal
                        - previous_hand_goal
                    )
                )
            )

            max_hand_goal_step = max(
                max_hand_goal_step,
                hand_step,
            )

            previous_hand_goal = (
                q_hand_goal.copy()
            )

            # =================================================
            # MERGED 17DOF TARGET
            # =================================================
            data.ctrl[:ARM_DOF] = (
                q_arm_goal
            )

            data.ctrl[ARM_DOF:] = (
                q_hand_goal
            )

            # =================================================
            # 500 Hz physics
            # =================================================
            for _ in range(
                PHYSICS_PER_TRACKING_FRAME
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
                    f"Non-finite simulation "
                    f"state at frame {frame}"
                )

            # =================================================
            # METRICS
            # =================================================
            (
                p_actual,
                R_actual,
            ) = tcp_pose(
                model,
                data,
                tcp_body_id,
            )

            (
                tcp_pos_error,
                tcp_rot_error,
            ) = pose_error(
                p_target,
                R_target,
                p_actual,
                R_actual,
            )

            max_sim_tcp_position_error = max(
                max_sim_tcp_position_error,
                tcp_pos_error,
            )

            max_sim_tcp_rotation_error = max(
                max_sim_tcp_rotation_error,
                tcp_rot_error,
            )

            hand_tracking_error = float(
                np.max(
                    np.abs(
                        data.qpos[
                            ARM_DOF:
                        ]
                        - q_hand_goal
                    )
                )
            )

            max_hand_tracking_error = max(
                max_hand_tracking_error,
                hand_tracking_error,
            )

            # Viewer follows tracking cadence,
            # not 500 Hz physics cadence.
            viewer.sync()

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
            "max arm q_goal step/frame =",
            max_arm_goal_step,
            "rad",
        )

        print(
            "max hand q_goal step/frame =",
            max_hand_goal_step,
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
            "max simulated TCP position error =",
            max_sim_tcp_position_error
            * 1000.0,
            "mm",
        )

        print(
            "max simulated TCP rotation error =",
            math.degrees(
                max_sim_tcp_rotation_error
            ),
            "deg",
        )

        print(
            "max SG100 tracking error =",
            max_hand_tracking_error,
            "rad",
        )

        # -----------------------------------------------------
        # Validation criteria
        # -----------------------------------------------------
        if ik_failures != 0:
            raise RuntimeError(
                f"IK failures occurred: "
                f"{ik_failures}"
            )

        if max_arm_goal_step > 0.05:
            raise RuntimeError(
                "Arm target discontinuity > 0.05 rad/frame"
            )

        if max_hand_goal_step > 0.10:
            raise RuntimeError(
                "Hand target discontinuity > 0.10 rad/frame"
            )

        if (
            max_sim_tcp_position_error
            > 5e-3
        ):
            raise RuntimeError(
                "TCP tracking error exceeded 5 mm"
            )

        if (
            max_sim_tcp_rotation_error
            > math.radians(1.0)
        ):
            raise RuntimeError(
                "TCP rotation error exceeded 1 degree"
            )

        if (
            max_hand_tracking_error
            > 0.15
        ):
            raise RuntimeError(
                "SG100 tracking error exceeded "
                "0.15 rad"
            )

        print(
            "\nFAKE AVP 17DOF TELEOP: PASS"
        )

        print(
            "\nClose viewer to exit."
        )

        while viewer.is_running():
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    main()
