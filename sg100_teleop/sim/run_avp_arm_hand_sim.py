from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from avp_stream import VisionProStreamer

from sg100_teleop.calibration.io import load_calibration

from sg100_teleop.hand.canonical_hand import CanonicalHandState

from sg100_teleop.control.joint_command_filter import JointCommandFilter

from sg100_teleop.kinematics.rm65_ik import RM65IKSolver

from sg100_teleop.retargeting.sg100_retargeter import SG100Retargeter

from sg100_teleop.tracking.avp_adapter import AVPHandAdapter

from sg100_teleop.tracking.avp_source import get_hand_from_tracking

from sg100_teleop.tracking.avp_wrist_mapper import AVPWristRelativeMapper, make_transform

# ============================================================
# Default files
# ============================================================

DEFAULT_MODEL = (
    "sg100_teleop/assets/rm65_sg100/"
    "mjcf/rm65_sg100_sim.xml"
)

DEFAULT_CALIBRATION = (
    "sg100_teleop/config/"
    "avp_right_calibration.yaml"
)

# ============================================================
# Combined model layout
# ============================================================
ARM_DOF = 6
HAND_DOF = 11
TOTAL_DOF = 17

TCP_BODY = "sg100_base_link"

RM65_JOINT_ORDER = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]


SG100_JOINT_ORDER = [
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

# ============================================================
# RM65 teleoperation start configuration
# ============================================================

RM65_TELEOP_READY_Q = np.array(
    [
        0.32488,
        -0.76769,
        0.97621,
        0.22463,
        -1.80649,
        -0.12430,
    ],
    dtype=np.float64
)

# ============================================================
# Initial AVP -> robot axis mapping
# ============================================================

AVP_TO_TCP_LOCAL_ROTATION = np.eye(3, dtype=np.float64)

def main():
    # ========================================================
    # Command-line arguments
    # ========================================================

    parser = argparse.ArgumentParser(
        description=(
            "Apple Vision Pro -> "
            "RM65 + SG100 MuJoCo teleoperation"
        )
    )

    parser.add_argument(
        "--ip",
        required=True,
        help="Apple Vision Pro IP address",
    )

    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL,
        help="Combined RM65 + SG100 MuJoCo XML",
    )

    parser.add_argument(
        "--calibration",
        default=DEFAULT_CALIBRATION,
        help="AVP right-hand calibration YAML",
    )

    parser.add_argument(
        "--port",
        type=int,
        default=50051,
        help="gRPC port used by AVP MuJoCo streaming",
    )

    parser.add_argument(
        "--tracking-hz",
        type=float,
        default=60.0,
        help="AVP tracking query frequency",
    )

    parser.add_argument(
        "--stream-hz",
        type=float,
        default=60.0,
        help="MuJoCo state streaming frequency back to AVP",
    )

    parser.add_argument(
        "--translation-scale",
        type=float,
        default=0.5,
        help="Scale from relative AVP wrist translation "
        "to robot TCP translation",
    )

    parser.add_argument(
        "--filter-tau",
        type=float,
        default=0.06,
        help="SG100 hand command low-pass time constant",
    )

    parser.add_argument(
        "--max-hand-velocity",
        type=float,
        default=4.0,
        help="Maximum SG100 hand joint velocity (rad/s)",
    )

    parser.add_argument(
        "--tracking-timeout",
        type=float,
        default=0.5,
        help=(
            "Seconds without valid right-hand trackiing "
            "before reporting tracking loss"
        ),
    )

    parser.add_argument(
        "--attach",
        nargs=4,
        type=float,
        default=[0.0, 0.7, 0.8, 0.0],
        metavar=("X", "Y", "Z", "YAW_DEG"),
        help=(
            "Placement of the MuJoCo robot "
            "inside the AVP scene: x y z yaw_deg"
        ),
    )

    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print low-rate runtime diagnostics",
    )

    args = parser.parse_args()

    # ========================================================
    # Argument validation
    # ========================================================

    if args.tracking_hz <= 0.0:
        raise ValueError(
            "--tracking-hz must be > 0"
        )
    
    if args.stream_hz <= 0.0:
        raise ValueError(
            "--stream-hz must be > 0"
        )
    
    if args.translation_scale <= 0.0:
        raise ValueError(
            "--translation-scale must be > 0"
        )

    if args.filter_tau <= 0.0:
        raise ValueError(
            "--filter-tau must be > 0"
        )

    if args.max_hand_velocity <= 0.0:
        raise ValueError(
            "--max-hand-velocity must be > 0"
        )
    
    if args.tracking_timeout <= 0.0:
        raise ValueError(
            "--tracking-timeout must be > 0"
        )

    # ========================================================
    # Resolve input files
    # ========================================================

    model_path = Path(args.model)

    calibration_path = Path(args.calibration)

    if not model_path.exists():
        raise FileNotFoundError(
            "Combined MuJoCo model not found: " 
            f"{model_path}"
        )

    if not calibration_path.exists():
        raise FileNotFoundError(
            "AVP calibration file not found: "
            f"{calibration_path}\n"
            "Run calibrate_avp_hand first."
        )

    # ========================================================
    # Startup information
    # ========================================================

    print(
        "===== AVP -> RM65 + SG100 SIM ====="
    )

    print(
        "model             :",
        model_path,
    )

    print(
        "hand calibration  :",
        calibration_path,
    )

    print(
        "Vision Pro        :",
        args.ip,
    )

    print(
        "tracking rate     :",
        args.tracking_hz,
        "Hz",
    )

    print(
        "AVP stream rate   :",
        args.stream_hz,
        "Hz",
    )

    print(
        "translation scale :",
        args.translation_scale,
    )

        # ========================================================
    # MuJoCo combined model
    # ========================================================

    model = mujoco.MjModel.from_xml_path(
        str(model_path)
    )

    data = mujoco.MjData(
        model
    )


    # ========================================================
    # Basic model-dimension validation
    # ========================================================

    if model.nq != TOTAL_DOF:
        raise RuntimeError(
            "Unexpected combined-model nq: "
            f"expected {TOTAL_DOF}, "
            f"got {model.nq}"
        )

    if model.nv != TOTAL_DOF:
        raise RuntimeError(
            "Unexpected combined-model nv: "
            f"expected {TOTAL_DOF}, "
            f"got {model.nv}"
        )

    if model.nu != TOTAL_DOF:
        raise RuntimeError(
            "Unexpected combined-model nu: "
            f"expected {TOTAL_DOF}, "
            f"got {model.nu}"
        )


    # ========================================================
    # Expected joint layout
    # ========================================================

    EXPECTED_JOINT_ORDER = (
        RM65_JOINT_ORDER
        + SG100_JOINT_ORDER
    )

    if len(EXPECTED_JOINT_ORDER) != TOTAL_DOF:
        raise RuntimeError(
            "Internal joint-order definition "
            "does not contain 17 joints."
        )


    # ========================================================
    # Validate qpos / DoF ordering
    # ========================================================

    for expected_index, joint_name in enumerate(
        EXPECTED_JOINT_ORDER
    ):
        joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            joint_name,
        )

        if joint_id < 0:
            raise RuntimeError(
                "Required joint not found in MuJoCo model: "
                f"{joint_name}"
            )

        qpos_index = int(
            model.jnt_qposadr[
                joint_id
            ]
        )

        dof_index = int(
            model.jnt_dofadr[
                joint_id
            ]
        )

        if qpos_index != expected_index:
            raise RuntimeError(
                "MuJoCo qpos joint-order mismatch.\n"
                f"Joint: {joint_name}\n"
                f"Expected qpos index: "
                f"{expected_index}\n"
                f"Actual qpos index: "
                f"{qpos_index}"
            )

        if dof_index != expected_index:
            raise RuntimeError(
                "MuJoCo DoF joint-order mismatch.\n"
                f"Joint: {joint_name}\n"
                f"Expected DoF index: "
                f"{expected_index}\n"
                f"Actual DoF index: "
                f"{dof_index}"
            )


    # ========================================================
    # Validate actuator / ctrl ordering
    # ========================================================

    for expected_index, joint_name in enumerate(
        EXPECTED_JOINT_ORDER
    ):
        joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            joint_name,
        )

        actuator_joint_id = int(
            model.actuator_trnid[
                expected_index,
                0,
            ]
        )

        if actuator_joint_id != joint_id:
            actual_joint_name = (
                mujoco.mj_id2name(
                    model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    actuator_joint_id,
                )
            )

            raise RuntimeError(
                "MuJoCo actuator-order mismatch.\n"
                f"ctrl[{expected_index}] "
                f"should drive {joint_name}, "
                f"but drives {actual_joint_name}"
            )


    # ========================================================
    # Validate teleoperation TCP
    # ========================================================

    tcp_body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        TCP_BODY,
    )

    if tcp_body_id < 0:
        raise RuntimeError(
            "Teleoperation TCP body not found: "
            f"{TCP_BODY}"
        )

    tcp_parent_id = int(
        model.body_parentid[
            tcp_body_id
        ]
    )

    tcp_parent_name = (
        mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            tcp_parent_id,
        )
    )

    if tcp_parent_name != "link_6":
        raise RuntimeError(
            "Unexpected TCP attachment.\n"
            f"{TCP_BODY} parent should be "
            f"link_6, got {tcp_parent_name}"
        )


    # ========================================================
    # Physics timing
    # ========================================================

    physics_dt = float(
        model.opt.timestep
    )

    if physics_dt <= 0.0:
        raise RuntimeError(
            "MuJoCo timestep must be > 0"
        )

    physics_hz = (
        1.0
        / physics_dt
    )


    # ========================================================
    # Initialize simulation state
    # ========================================================

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0

    data.qpos[
        :ARM_DOF
    ] = RM65_TELEOP_READY_Q

    data.ctrl[
        :ARM_DOF
    ] = RM65_TELEOP_READY_Q

    mujoco.mj_forward(
        model,
        data,
    )


    # ========================================================
    # Model diagnostics
    # ========================================================

    print()
    print(
        "Combined MuJoCo model validated."
    )

    print(
        "nq / nv / nu      :",
        model.nq,
        "/",
        model.nv,
        "/",
        model.nu,
    )

    print(
        "physics rate      :",
        physics_hz,
        "Hz",
    )

    print(
        "teleop TCP        :",
        TCP_BODY,
    )

    print(
        "TCP parent        :",
        tcp_parent_name,
    )

    print(
        "arm qpos          :",
        "[0:6]",
    )

    print(
        "hand qpos         :",
        "[6:17]",
    )
        # ========================================================
    # SG100 hand retargeting pipeline
    # ========================================================

    calibration = load_calibration(
        calibration_path
    )

    avp_adapter = AVPHandAdapter(
        calibration
    )

    retargeter = SG100Retargeter()


    # ========================================================
    # Validate SG100 retargeter joint ordering
    # ========================================================

    if (
        retargeter.joint_order
        != SG100_JOINT_ORDER
    ):
        raise RuntimeError(
            "SG100 retargeter joint-order mismatch.\n"
            f"Expected: {SG100_JOINT_ORDER}\n"
            f"Actual:   {retargeter.joint_order}"
        )


    # ========================================================
    # Initial semantic hand state
    # ========================================================

    neutral_hand = (
        CanonicalHandState.neutral()
    )

    q_hand_goal = (
        retargeter.retarget(
            neutral_hand
        )
    )


    # ========================================================
    # Validate initial hand target
    # ========================================================

    if q_hand_goal.shape != (
        HAND_DOF,
    ):
        raise RuntimeError(
            "Unexpected SG100 retarget output shape: "
            f"expected ({HAND_DOF},), "
            f"got {q_hand_goal.shape}"
        )

    if not np.all(
        np.isfinite(
            q_hand_goal
        )
    ):
        raise RuntimeError(
            "Initial SG100 target contains "
            "non-finite values."
        )

    if not np.allclose(
        q_hand_goal,
        np.zeros(
            HAND_DOF,
            dtype=np.float64,
        ),
        atol=1e-12,
    ):
        raise RuntimeError(
            "Canonical neutral hand does not "
            "retarget to SG100 q=0.\n"
            f"q_hand_goal={q_hand_goal}"
        )


    # ========================================================
    # SG100 command filter
    # ========================================================

    hand_command_filter = (
        JointCommandFilter(
            dof=HAND_DOF,
            tau=args.filter_tau,
            max_velocity=(
                args.max_hand_velocity
            ),
        )
    )

    hand_command_filter.reset(
        q_hand_goal
    )

    q_hand_cmd = (
        q_hand_goal.copy()
    )


    # ========================================================
    # Synchronize MuJoCo hand state
    # ========================================================

    data.qpos[
        ARM_DOF:
    ] = q_hand_goal

    data.ctrl[
        ARM_DOF:
    ] = q_hand_goal

    mujoco.mj_forward(
        model,
        data,
    )


    # ========================================================
    # Hand-pipeline diagnostics
    # ========================================================

    print()
    print(
        "SG100 hand pipeline initialized."
    )

    print(
        "hand DoF           :",
        HAND_DOF,
    )

    print(
        "initial q_goal     :",
        np.round(
            q_hand_goal,
            4,
        ),
    )

    print(
        "filter tau         :",
        args.filter_tau,
        "s",
    )

    print(
        "max hand velocity  :",
        args.max_hand_velocity,
        "rad/s",
    )

    # ========================================================
    # RM65 Cartesian teleoperation pipeline
    # ========================================================

    ik_solver = RM65IKSolver(
        model,
        tcp_body_name=TCP_BODY,
    )

    # ========================================================
    # Initial RM65 joint target
    # ========================================================

    q_arm_goal = (
        RM65_TELEOP_READY_Q.copy()
    )

    # ========================================================
    # Robot TCP pose from the current MuJoCo state
    # ========================================================

    p_tcp0 = (
        data.xpos[tcp_body_id].copy()
    )

    R_tcp0 = (
        data.xmat[tcp_body_id].reshape(3,3).copy()
    )

    T_tcp0 = make_transform(
        p_tcp0,
        R_tcp0,
    )

    # ========================================================
    # Cross-check TCP pose against RM65IKSolver FK
    # ========================================================

    (
        p_tcp_fk,
        R_tcp_fk,
    ) = ik_solver.fk(q_arm_goal)

    tcp_position_consistency_error = float(
        np.linalg.norm(
            p_tcp_fk - p_tcp0
        )
    )

    tcp_rotation_consistency_error = float(
        np.linalg.norm(
            R_tcp_fk - R_tcp0
        )
    )

    if (
        tcp_position_consistency_error
        > 1e-10
    ):
        raise RuntimeError(
            "MuJoCo TCP position and IK FK "
            "are inconsistent.\n"
            f"Error: "
            f"{tcp_position_consistency_error}"
        )

    if (
        tcp_rotation_consistency_error
        > 1e-10
    ):
        raise RuntimeError(
            "MuJoCo TCP rotation and IK FK "
            "are inconsistent.\n"
            f"Error: "
            f"{tcp_rotation_consistency_error}"
        )


    # ========================================================
    # AVP wrist -> robot TCP mapper
    # ========================================================
    wrist_mapper = (
        AVPWristRelativeMapper(
            translation_scale=(
                args.translation_scale
            ),
            R_tcp_from_avp_local=(
                AVP_TO_TCP_LOCAL_ROTATION
            ),
        )
    )

    # ========================================================
    # Wrist calibration state
    # ========================================================

    wrist_calibrated = False

    # ========================================================
    # IK diagnostics
    # ========================================================

    last_ik_result = None

    ik_failure_count = 0

    # ========================================================
    # Arm-pipeline diagonostics
    # ========================================================

    print()
    print(
        "RM65 arm pipeline initialized."
    )

    print(
        "teleop ready q     :",
        np.round(
            q_arm_goal,
            4,
        ),
    )

    print(
        "initial TCP pos    :",
        np.round(
            p_tcp0,
            6,
        ),
    )

    print(
        "translation scale  :",
        args.translation_scale,
    )

    print(
        "wrist calibrated   :",
        wrist_calibrated,
    )

    print(
        "TCP/FK pos error   :",
        tcp_position_consistency_error,
    )

    print(
        "TCP/FK rot error   :",
        tcp_rotation_consistency_error,
    )

    # ========================================================
    # Apple Vision Pro streamer
    # ========================================================

    streamer = VisionProStreamer(
        ip=args.ip,
        record=False,
        verbose=args.verbose,
    )


    # ========================================================
    # Configure MuJoCo state streaming back to AVP
    # ========================================================

    streamer.configure_mujoco(
        xml_path=str(
            model_path
        ),
        model=model,
        data=data,
        relative_to=list(
            args.attach
        ),
        grpc_port=args.port,
    )


    # ========================================================
    # Start AVP WebRTC / simulation streaming
    # ========================================================

    streamer.start_webrtc()


    # ========================================================
    # Runtime periods
    # ========================================================

    tracking_period = (
        1.0
        / args.tracking_hz
    )

    stream_period = (
        1.0
        / args.stream_hz
    )


    # ========================================================
    # Runtime clocks
    # ========================================================

    last_wall = (
        time.perf_counter()
    )

    physics_accumulator = 0.0

    last_tracking_query = (
        -float("inf")
    )

    last_valid_tracking = (
        -float("inf")
    )

    last_stream = (
        -float("inf")
    )

    last_print = (
        -float("inf")
    )


    # ========================================================
    # Tracking state
    # ========================================================

    tracking_lost_printed = False


    # ========================================================
    # Runtime diagnostics
    # ========================================================

    tracking_frame_count = 0

    valid_tracking_count = 0

    rejected_tracking_count = 0


    # ========================================================
    # Startup status
    # ========================================================

    print()
    print(
        "Vision Pro streamer initialized."
    )

    print(
        "tracking period    :",
        tracking_period,
        "s",
    )

    print(
        "stream period      :",
        stream_period,
        "s",
    )

    print(
        "physics period     :",
        physics_dt,
        "s",
    )

    print()
    print(
        "Waiting for valid right-hand tracking..."
    )

        # ========================================================
    # Main runtime loop
    # ========================================================

    try:
        with (
            mujoco.viewer.launch_passive(
                model,
                data,
            )
            as viewer
        ):

            while viewer.is_running():

                # =================================================
                # Wall-clock timing
                # =================================================

                now = (
                    time.perf_counter()
                )

                wall_dt = (
                    now
                    - last_wall
                )

                last_wall = now

                # Avoid a huge physics catch-up after:
                #
                # - debugger pause
                # - window drag
                # - OS scheduling stall
                #
                # The simulation validation runner should
                # recover gracefully instead of trying to
                # integrate hundreds of missed physics steps.
                wall_dt = min(
                    wall_dt,
                    0.05,
                )

                physics_accumulator += (
                    wall_dt
                )


                # =================================================
                # AVP tracking update
                # =================================================

                if (
                    now
                    - last_tracking_query
                    >= tracking_period
                ):
                    last_tracking_query = (
                        now
                    )

                    tracking_frame_count += 1

                    tracking = (
                        streamer.get_latest()
                    )

                    hand = (
                        get_hand_from_tracking(
                            tracking,
                            side="right",
                        )
                    )


                    # ---------------------------------------------
                    # Only process a frame when the right hand
                    # exists.
                    # ---------------------------------------------

                    if hand is not None:

                        try:
                            # =====================================
                            # HAND BRANCH
                            #
                            # AVP hand skeleton
                            #       ↓
                            # CanonicalHandState
                            #       ↓
                            # SG100 target
                            # =====================================

                            canonical = (
                                avp_adapter.convert(
                                    hand
                                )
                            )

                            q_hand_candidate = (
                                retargeter.retarget(
                                    canonical
                                )
                            )


                            # -------------------------------------
                            # Validate SG100 candidate
                            # -------------------------------------

                            if (
                                q_hand_candidate.shape
                                != (HAND_DOF,)
                            ):
                                raise ValueError(
                                    "Unexpected SG100 "
                                    "candidate shape: "
                                    f"{q_hand_candidate.shape}"
                                )

                            if not np.all(
                                np.isfinite(
                                    q_hand_candidate
                                )
                            ):
                                raise ValueError(
                                    "SG100 candidate contains "
                                    "non-finite values."
                                )


                            # =====================================
                            # ARM BRANCH
                            #
                            # The same HandData is used here.
                            #
                            # hand[0] is the AVP wrist SE(3).
                            # =====================================

                            T_avp_wrist = np.asarray(
                                hand[0],
                                dtype=np.float64,
                            ).copy()


                            # -------------------------------------
                            # Validate wrist transform
                            # -------------------------------------

                            if (
                                T_avp_wrist.shape
                                != (4, 4)
                            ):
                                raise ValueError(
                                    "AVP wrist transform must "
                                    "have shape (4,4), got "
                                    f"{T_avp_wrist.shape}"
                                )

                            if not np.all(
                                np.isfinite(
                                    T_avp_wrist
                                )
                            ):
                                raise ValueError(
                                    "AVP wrist transform "
                                    "contains non-finite values."
                                )


                            # =====================================
                            # First valid wrist frame:
                            #
                            # establish relative teleoperation
                            # reference.
                            #
                            # IMPORTANT:
                            # no arm IK movement is generated from
                            # this first frame.
                            # =====================================

                            if not wrist_calibrated:

                                wrist_mapper.calibrate(
                                    T_avp_wrist,
                                    T_tcp0,
                                )

                                wrist_calibrated = True

                                print()
                                print(
                                    "AVP wrist reference "
                                    "calibrated."
                                )

                                print(
                                    "AVP wrist initial pos :",
                                    np.round(
                                        T_avp_wrist[
                                            :3,
                                            3,
                                        ],
                                        6,
                                    ),
                                )

                                print(
                                    "Robot TCP anchor pos  :",
                                    np.round(
                                        p_tcp0,
                                        6,
                                    ),
                                )

                                print(
                                    "RM65 remains at "
                                    "teleop-ready pose."
                                )


                            # =====================================
                            # Subsequent frames:
                            #
                            # AVP wrist
                            #   ↓
                            # relative mapper
                            #   ↓
                            # TCP target
                            #   ↓
                            # RM65 IK
                            # =====================================

                            else:

                                T_tcp_target = (
                                    wrist_mapper.map(
                                        T_avp_wrist
                                    )
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


                                # ---------------------------------
                                # Solve RM65 IK using the previous
                                # valid solution as the next seed.
                                # ---------------------------------

                                (
                                    q_arm_candidate,
                                    ik_result,
                                ) = (
                                    ik_solver.solve(
                                        q_seed=(
                                            q_arm_goal
                                        ),
                                        target_position=(
                                            p_target
                                        ),
                                        target_rotation=(
                                            R_target
                                        ),
                                    )
                                )

                                last_ik_result = (
                                    ik_result
                                )


                                # ---------------------------------
                                # Accept only a valid converged IK
                                # solution.
                                # ---------------------------------

                                if ik_result.converged:

                                    if (
                                        q_arm_candidate.shape
                                        != (ARM_DOF,)
                                    ):
                                        raise ValueError(
                                            "Unexpected RM65 IK "
                                            "output shape: "
                                            f"{q_arm_candidate.shape}"
                                        )

                                    if not np.all(
                                        np.isfinite(
                                            q_arm_candidate
                                        )
                                    ):
                                        raise ValueError(
                                            "RM65 IK output "
                                            "contains non-finite "
                                            "values."
                                        )

                                    q_arm_goal = (
                                        q_arm_candidate
                                        .copy()
                                    )


                                # ---------------------------------
                                # IK failure:
                                #
                                # hold the last valid q_arm_goal.
                                #
                                # This is intentionally lightweight
                                # because this project is simulation
                                # teleoperation validation only.
                                # ---------------------------------

                                else:

                                    ik_failure_count += 1

                                    if args.verbose:
                                        print(
                                            "IK hold: "
                                            f"pos_err="
                                            f"{ik_result.position_error * 1000.0:.3f} mm, "
                                            f"rot_err="
                                            f"{np.degrees(ik_result.rotation_error):.3f} deg, "
                                            f"sigma_min="
                                            f"{ik_result.sigma_min:.5f}, "
                                            f"condition="
                                            f"{ik_result.condition_number:.2f}"
                                        )


                            # =====================================
                            # Accept SG100 target
                            #
                            # This happens only after this AVP frame
                            # has passed both hand and wrist input
                            # validation.
                            # =====================================

                            q_hand_goal = (
                                q_hand_candidate.copy()
                            )


                            # =====================================
                            # Mark the sensory frame as valid
                            # =====================================

                            last_valid_tracking = (
                                now
                            )

                            valid_tracking_count += 1


                            # -------------------------------------
                            # Tracking recovery notification
                            # -------------------------------------

                            if (
                                tracking_lost_printed
                            ):
                                print()
                                print(
                                    "Right-hand tracking "
                                    "recovered."
                                )

                                tracking_lost_printed = (
                                    False
                                )


                        # =========================================
                        # Reject malformed tracking frames
                        # =========================================

                        except (
                            ValueError,
                            np.linalg.LinAlgError,
                        ) as exc:

                            rejected_tracking_count += 1

                            if args.verbose:
                                print(
                                    "Tracking frame "
                                    "rejected:",
                                    exc,
                                )


                # =================================================
                # Tracking-loss behavior
                #
                # Tracking loss and IK failure are intentionally
                # different concepts.
                #
                # Tracking loss:
                #     no usable AVP frame
                #
                # IK failure:
                #     valid AVP frame, but requested TCP target
                #     cannot be solved reliably
                #
                # In both cases:
                #     HOLD the last valid robot targets.
                # =================================================

                if valid_tracking_count > 0:

                    tracking_age = (
                        now
                        - last_valid_tracking
                    )

                    if (
                        tracking_age
                        > args.tracking_timeout
                        and not tracking_lost_printed
                    ):
                        print()
                        print(
                            "WARNING: right-hand tracking "
                            "lost."
                        )

                        print(
                            "Holding last RM65 and "
                            "SG100 targets."
                        )

                        tracking_lost_printed = (
                            True
                        )


                # =================================================
                # MuJoCo physics
                #
                # The arm target is updated by AVP/IK at the
                # tracking rate.
                #
                # The SG100 goal is also updated at the tracking
                # rate, but its command filter runs at the MuJoCo
                # physics rate.
                # =================================================

                while (
                    physics_accumulator
                    >= physics_dt
                ):

                    # ---------------------------------------------
                    # RM65 command
                    #
                    # q_arm_goal is already a continuous IK output.
                    # We intentionally do not add another arm filter
                    # in this first integrated runner.
                    # ---------------------------------------------

                    data.ctrl[
                        :ARM_DOF
                    ] = q_arm_goal


                    # ---------------------------------------------
                    # SG100 command
                    #
                    # low-rate q_hand_goal
                    #         ↓
                    # JointCommandFilter @ physics rate
                    #         ↓
                    # high-rate q_hand_cmd
                    # ---------------------------------------------

                    q_hand_cmd = (
                        hand_command_filter.update(
                            q_hand_goal,
                            physics_dt,
                        )
                    )

                    data.ctrl[
                        ARM_DOF:
                    ] = q_hand_cmd


                    # ---------------------------------------------
                    # Advance MuJoCo by one physics timestep.
                    # ---------------------------------------------

                    mujoco.mj_step(
                        model,
                        data,
                    )

                    physics_accumulator -= (
                        physics_dt
                    )


                # =================================================
                # Local MuJoCo viewer
                #
                # Important:
                # viewer.sync() is outside the 500 Hz physics loop.
                # =================================================

                viewer.sync()


                # =================================================
                # Stream current MuJoCo state back to AVP
                # =================================================

                if (
                    now
                    - last_stream
                    >= stream_period
                ):
                    streamer.update_sim()

                    last_stream = now


                # =================================================
                # Low-rate diagnostics
                # =================================================

                if (
                    args.verbose
                    and (
                        now
                        - last_print
                        >= 0.5
                    )
                ):
                    print()

                    print(
                        "tracking frames :",
                        tracking_frame_count,
                    )

                    print(
                        "valid frames    :",
                        valid_tracking_count,
                    )

                    print(
                        "rejected frames :",
                        rejected_tracking_count,
                    )

                    print(
                        "wrist calibrated:",
                        wrist_calibrated,
                    )

                    print(
                        "IK failures     :",
                        ik_failure_count,
                    )

                    print(
                        "q_arm_goal      :",
                        np.round(
                            q_arm_goal,
                            3,
                        ),
                    )

                    print(
                        "arm qpos        :",
                        np.round(
                            data.qpos[
                                :ARM_DOF
                            ],
                            3,
                        ),
                    )

                    print(
                        "q_hand_goal     :",
                        np.round(
                            q_hand_goal,
                            3,
                        ),
                    )

                    print(
                        "q_hand_cmd      :",
                        np.round(
                            q_hand_cmd,
                            3,
                        ),
                    )

                    print(
                        "hand qpos       :",
                        np.round(
                            data.qpos[
                                ARM_DOF:
                            ],
                            3,
                        ),
                    )


                    # ---------------------------------------------
                    # Print the latest IK health metrics only when
                    # at least one IK solve has occurred.
                    # ---------------------------------------------

                    if (
                        last_ik_result
                        is not None
                    ):
                        print(
                            "IK pos error    :",
                            round(
                                last_ik_result.position_error
                                * 1000.0,
                                3,
                            ),
                            "mm",
                        )

                        print(
                            "IK rot error    :",
                            round(
                                np.degrees(
                                    last_ik_result.rotation_error
                                ),
                                3,
                            ),
                            "deg",
                        )

                        print(
                            "IK sigma_min    :",
                            round(
                                last_ik_result.sigma_min,
                                5,
                            ),
                        )

                        print(
                            "IK condition    :",
                            round(
                                last_ik_result.condition_number,
                                2,
                            ),
                        )

                    last_print = now


                # =================================================
                # Small cooperative sleep
                #
                # This is not the physics clock.
                # Physics timing is controlled by
                # physics_accumulator above.
                # =================================================

                time.sleep(
                    0.001
                )


    except KeyboardInterrupt:
        pass

        print()
    print(
        "========================================"
    )

    print(
        "AVP -> RM65 + SG100 simulation stopped."
    )

    print(
        "tracking queries  :",
        tracking_frame_count,
    )

    print(
        "valid frames      :",
        valid_tracking_count,
    )

    print(
        "rejected frames   :",
        rejected_tracking_count,
    )

    print(
        "IK failures       :",
        ik_failure_count,
    )

    print(
        "wrist calibrated  :",
        wrist_calibrated,
    )

    print(
        "========================================"
    )

if __name__ == "__main__":
    main()