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

from sg100_teleop.control.tcp_target_filter import TcpTargetFilter

from sg100_teleop.control.teleop_arming import (
    TeleopArmingController,
)

from sg100_teleop.kinematics.rm65_ik import RM65IKSolver

from sg100_teleop.retargeting.sg100_retargeter import SG100Retargeter

from sg100_teleop.sim.combined_robot_layout import (
    ARM_DOF,
    HAND_DOF,
    SG100_JOINT_ORDER,
    initialize_robot_state,
    resolve_robot_layout,
    write_robot_controls,
)

from sg100_teleop.tracking.avp_adapter import AVPHandAdapter

from sg100_teleop.tracking.avp_source import get_hand_from_tracking

from sg100_teleop.tracking.avp_to_tcp_config import (
    load_avp_to_tcp_config,
)

from sg100_teleop.tracking.frame_gate import (
    TrackingFrameGate,
    TrackingFrameStatus,
)

from sg100_teleop.tracking.avp_wrist_mapper import AVPWristRelativeMapper, make_transform

# ============================================================
# Default files
# ============================================================

DEFAULT_MODEL = (
    "sg100_teleop/assets/rm65_sg100/"
    "mjcf/rm65_sg100_avp_grasp_scene.xml"
)

DEFAULT_CALIBRATION = (
    "sg100_teleop/config/"
    "avp_right_calibration.yaml"
)

DEFAULT_AVP_TO_TCP_CONFIG = (
    "sg100_teleop/config/"
    "avp_to_tcp_identity.yaml"
)

TCP_BODY = "sg100_base_link"
TCP_FOREARM_AXIS_LOCAL = np.array(
    [0.0, 0.0, 1.0],
    dtype=np.float64,
)
FOREARM_DIRECTION_EPSILON_M = 1e-6


def extract_forearm_axis(
    hand: np.ndarray,
    *,
    epsilon_m: float = FOREARM_DIRECTION_EPSILON_M,
) -> np.ndarray | None:
    """Return normalized AVP forearmArm -> forearmWrist direction.

    Legacy 25-joint frames and malformed/degenerate 27-joint samples are
    deliberately reported as unavailable so forearm-axis mode can fall back
    to position-only IK without indexing invalid data.
    """

    joints = np.asarray(hand, dtype=np.float64)
    if joints.ndim != 3 or joints.shape[0] < 27:
        return None
    if joints.shape[1:] != (4, 4):
        return None
    if not np.isfinite(epsilon_m) or epsilon_m <= 0.0:
        raise ValueError("epsilon_m must be finite and > 0")

    p_forearm_wrist = joints[25, :3, 3]
    p_forearm_arm = joints[26, :3, 3]
    if (
        not np.all(np.isfinite(p_forearm_wrist))
        or not np.all(np.isfinite(p_forearm_arm))
    ):
        return None

    direction = p_forearm_wrist - p_forearm_arm
    norm = float(np.linalg.norm(direction))
    if not np.isfinite(norm) or norm <= epsilon_m:
        return None
    return direction / norm


def solve_forearm_axis_or_position(
    ik_solver,
    wrist_mapper: AVPWristRelativeMapper,
    q_seed: np.ndarray,
    target_position: np.ndarray,
    n_forearm_avp: np.ndarray | None,
    forearm_axis_calibrated: bool,
):
    """Run the 5-DoF task, or its required position-only fallback."""

    a_target = (
        wrist_mapper.map_forearm_direction(
            n_forearm_avp,
            TCP_FOREARM_AXIS_LOCAL,
        )
        if forearm_axis_calibrated and n_forearm_avp is not None
        else None
    )
    if a_target is None:
        return ik_solver.solve(
            q_seed=q_seed,
            target_position=target_position,
            target_rotation=None,
            position_only=True,
        )

    return ik_solver.solve(
        q_seed=q_seed,
        target_position=target_position,
        target_rotation=None,
        target_axis=a_target,
        tcp_axis_local=TCP_FOREARM_AXIS_LOCAL,
    )


def select_full_pose_target_rotation(
    T_tcp_target: np.ndarray,
    *,
    lock_tcp_rotation: bool,
    R_tcp_rotation_lock: np.ndarray | None,
) -> np.ndarray:
    """Select mapped relative rotation unless the A/B lock is requested."""

    target = np.asarray(T_tcp_target, dtype=np.float64)
    if target.shape != (4, 4):
        raise ValueError("T_tcp_target must have shape (4,4)")

    if not lock_tcp_rotation:
        return target[:3, :3].copy()

    if R_tcp_rotation_lock is None:
        raise RuntimeError(
            "--lock-tcp-rotation requires a calibrated TCP rotation anchor"
        )
    rotation_lock = np.asarray(R_tcp_rotation_lock, dtype=np.float64)
    if rotation_lock.shape != (3, 3):
        raise ValueError("R_tcp_rotation_lock must have shape (3,3)")
    return rotation_lock.copy()

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
# Startup pose: pre-grasp stance near the bottle
# ============================================================
#
# run_grasp_demo.py starts with the open hand wrapped around the
# bottle.  That exact pose presses the link-2 capsules a few mm into
# the bottle, which would nudge it on the first physics step, so the
# teleop startup instead holds the palm a set distance from the
# bottle.  Solved by IK at runtime so it tracks the bottle placement
# in the scene XML; if IK fails the teleop-ready pose above is used
# unchanged.

GRASP_PALM_OFFSET = np.array(
    [-0.08, -0.13, 0.0]
)  # grasp palm target from bottle center (world); mirrors run_grasp_demo.py

# Startup stance: same lateral offset and approach orientation as the
# grasp pose, but the palm center (TCP) held this far from the bottle
# center along the approach direction.  The grasp pose itself sits at
# 0.128 m; 0.25 m leaves a clear pre-grasp standoff.
STARTUP_PALM_TO_BOTTLE_M = 0.25

_IK_SEED_OFFSETS = (
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 0.0, 0.0, np.pi),
    (0.0, 0.0, 0.0, 0.0, np.pi / 2, 0.0),
    (0.0, 0.0, 0.0, 0.0, -np.pi / 2, 0.0),
)


def _approach_rotation(approach):
    """Rotation whose z-axis points along a horizontal `approach`.

    Compact copy of run_grasp_demo._approach_R with roll = 0.
    """

    z = np.asarray(approach, dtype=np.float64)
    z = z / np.linalg.norm(z)

    ref = np.array([0.0, 0.0, 1.0])
    x = ref - np.dot(ref, z) * z
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)

    return np.column_stack([x, y, z])


def compute_startup_arm_q(model, data, ik_solver):
    """Pre-grasp startup pose near the scene's water bottle.

    Returns the best-margin converged IK solution, or None when the
    bottle body is missing or no seed converges.
    """

    bottle_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "water_bottle",
    )

    if bottle_id < 0:
        return None

    # A fresh MjData still holds the XML initial state, so this reads
    # the bottle's authored placement.
    mujoco.mj_forward(model, data)
    bottle = data.xpos[bottle_id].copy()

    # Approach direction: base -> teleop-ready TCP (horizontal), the
    # same convention as run_grasp_demo.py.
    p_ready, _ = ik_solver.fk(RM65_TELEOP_READY_Q)
    radial = np.array([p_ready[0], p_ready[1], 0.0])
    radial /= np.linalg.norm(radial)

    # Slide the grasp palm target out along the approach direction so
    # the palm sits STARTUP_PALM_TO_BOTTLE_M from the bottle center
    # (the grasp pose itself is 0.128 m = -GRASP_PALM_OFFSET . radial).
    grasp_radial_pull = float(
        np.dot(GRASP_PALM_OFFSET, radial)
    )
    palm_target = (
        bottle
        + GRASP_PALM_OFFSET
        - (STARTUP_PALM_TO_BOTTLE_M + grasp_radial_pull) * radial
    )
    R_grasp = _approach_rotation(radial)

    best = None
    for off in _IK_SEED_OFFSETS:
        seed = np.clip(
            RM65_TELEOP_READY_Q + np.asarray(off),
            ik_solver.lower,
            ik_solver.upper,
        )
        q, res = ik_solver.solve(
            q_seed=seed,
            target_position=palm_target,
            target_rotation=R_grasp,
        )
        if res.converged and np.all(np.isfinite(q)):
            margin = float(
                np.min(
                    np.minimum(
                        q - ik_solver.lower,
                        ik_solver.upper - q,
                    )
                )
            )
            if best is None or margin > best[1]:
                best = (q, margin)

    return best[0] if best is not None else None

def build_arg_parser():
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
        "--usdz",
        default=None,
        metavar="PATH",
        help=(
            "Prebuilt MuJoCo USDZ scene. When provided, "
            "cache and conversion are bypassed."
        ),
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
        "--avp-to-tcp-config",
        default=DEFAULT_AVP_TO_TCP_CONFIG,
        help=(
            "YAML containing translation_scale and the proper "
            "fixed AVP-world to robot-world controller-axis basis"
        ),
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
        default=2.0,
        help=(
            "Seconds without valid right-hand trackiing "
            "before reporting tracking loss"
        ),
    )

    # Teleop arm-IK mode: exactly one of these may be selected.
    mode_group = parser.add_mutually_exclusive_group()

    mode_group.add_argument(
        "--lock-tcp-rotation",
        action="store_true",
        help=(
            "Lock the TCP target orientation to the rotation "
            "captured at neutral calibration; only wrist "
            "translation (xyz) drives the arm. IK still solves "
            "the full 6D pose against the fixed orientation."
        ),
    )

    mode_group.add_argument(
        "--position-only-ik",
        action="store_true",
        help=(
            "True 3D position-only IK: the wrist controls TCP "
            "xyz only and orientation is NOT an IK task "
            "constraint at all, freeing arm posture and "
            "enlarging the reachable position range."
        ),
    )

    mode_group.add_argument(
        "--forearm-axis-ik",
        action="store_true",
        help=(
            "5-DoF IK: wrist translation controls TCP xyz and the "
            "AVP forearm axis controls TCP local +Z; roll about +Z "
            "remains unconstrained. Invalid/legacy forearm tracking "
            "falls back to position-only IK."
        ),
    )

    parser.add_argument(
        "--attach",
        nargs=4,
        type=float,
        default=[0.0, 0.0, 0.8, 90],
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

    return parser


def main():
    # ========================================================
    # Command-line arguments
    # ========================================================

    parser = build_arg_parser()

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

    avp_to_tcp_config_path = Path(
        args.avp_to_tcp_config
    )

    prebuilt_usdz_path = None

    if args.usdz is not None:
        prebuilt_usdz_path = (
            VisionProStreamer._validate_prebuilt_usdz(
                args.usdz
            )
        )

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

    if not avp_to_tcp_config_path.exists():
        raise FileNotFoundError(
            "AVP-to-TCP config file not found: "
            f"{avp_to_tcp_config_path}"
        )

    avp_to_tcp_config = load_avp_to_tcp_config(
        avp_to_tcp_config_path
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
        "AVP/TCP config    :",
        avp_to_tcp_config_path,
    )

    print(
        "arm IK mode       :",
        (
            "forearm-axis (5-DoF, TCP +Z / roll free)"
            if args.forearm_axis_ik
            else (
                "position-only (orientation free)"
                if args.position_only_ik
                else (
                    "fixed-orientation (calibration anchor)"
                    if args.lock_tcp_rotation
                    else "6-DoF pose (follows wrist)"
                )
            )
        ),
    )

    print(
        "USDZ source       :",
        (
            "prebuilt"
            if prebuilt_usdz_path is not None
            else "cache/converter"
        ),
    )

    if prebuilt_usdz_path is not None:
        print(
            "USDZ path         :",
            prebuilt_usdz_path,
        )
        print(
            "USDZ size         :",
            f"{Path(prebuilt_usdz_path).stat().st_size / (1024 * 1024):.2f} MB",
        )

    print(
        "basis source      :",
        avp_to_tcp_config.source,
    )

    print(
        "translation scale :",
        avp_to_tcp_config.translation_scale,
    )

    print(
        "AVP -> TCP fixed-axis basis:\n",
        avp_to_tcp_config.avp_to_tcp_local_rotation,
    )

    print(
        "TCP filter         :",
        avp_to_tcp_config.filter,
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


    # Resolve the named robot layout instead of assuming that the
    # complete scene contains exactly 17 qpos / DoFs. Extra passive
    # scene state (for example a bottle freejoint) is allowed.
    robot_layout = resolve_robot_layout(model)


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
    # Startup arm pose: pre-grasp near the bottle
    # ========================================================

    ik_solver = RM65IKSolver(
        model,
        tcp_body_name=TCP_BODY,
    )

    startup_arm_q = compute_startup_arm_q(
        model,
        data,
        ik_solver,
    )

    if startup_arm_q is None:
        startup_arm_q = (
            RM65_TELEOP_READY_Q.copy()
        )
        print(
            "startup pose      : pre-grasp IK "
            "unreachable, using teleop-ready"
        )
    else:
        print(
            "startup pose      : pre-grasp, "
            f"q = {np.round(startup_arm_q, 5).tolist()}"
        )

    # ========================================================
    # Initialize simulation state
    # ========================================================

    initialize_robot_state(
        model,
        data,
        robot_layout,
        startup_arm_q,
        np.zeros(HAND_DOF, dtype=np.float64),
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
        robot_layout.arm_qpos_indices.tolist(),
    )

    print(
        "hand qpos         :",
        robot_layout.hand_qpos_indices.tolist(),
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
        tuple(retargeter.joint_order)
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
        robot_layout.hand_qpos_indices
    ] = q_hand_goal

    data.ctrl[
        robot_layout.hand_actuator_ids
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

    # ik_solver was created together with the startup pose above.

    # ========================================================
    # Initial RM65 joint target
    # ========================================================

    q_arm_goal = (
        startup_arm_q.copy()
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
                avp_to_tcp_config.translation_scale
            ),
            R_tcp_from_avp_local=(
                avp_to_tcp_config
                .avp_to_tcp_local_rotation
            ),
        )
    )

    # RM65 Cartesian filtering belongs after wrist mapping and before IK.
    # Its timestamp advances only when a new valid tracking frame is used.
    tcp_target_filter = TcpTargetFilter(
        avp_to_tcp_config.filter
    )

    # ========================================================
    # Wrist calibration state
    # ========================================================

    wrist_calibrated = False

    teleop_arming = TeleopArmingController()

    T_avp_wrist_0 = None
    T_tcp_runtime_0 = None
    R_tcp_rotation_lock = None
    n_forearm0_avp = None
    forearm_axis_calibrated = False
    q_hand0_runtime = None

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
        avp_to_tcp_config.translation_scale,
    )

    print(
        "wrist calibrated   :",
        wrist_calibrated,
    )

    print(
        "teleop state       :",
        teleop_arming.state.value,
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
        record=True,
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
        usdz_path=prebuilt_usdz_path,
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

    tracking_gate = TrackingFrameGate(
        max_frame_age_seconds=args.tracking_timeout,
        tracking_timeout_seconds=args.tracking_timeout,
    )


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

    def print_teleop_state(message: str) -> None:
        print()
        print(
            "TELEOP STATE:",
            teleop_arming.state.value,
        )
        print(message)

    def request_neutral_calibration() -> bool:
        requested = (
            teleop_arming.request_neutral_calibration()
        )
        if requested:
            print_teleop_state(
                "Neutral calibration requested; waiting for "
                "the next NEW + VALID right-hand frame."
            )
        else:
            print()
            print(
                "Neutral calibration request ignored in state:",
                teleop_arming.state.value,
            )
        return requested

    def on_viewer_key(keycode: int) -> None:
        if keycode in (ord("C"), ord("c")):
            request_neutral_calibration()

        # ========================================================
    # Main runtime loop
    # ========================================================

    try:
        with (
            mujoco.viewer.launch_passive(
                model,
                data,
                key_callback=on_viewer_key,
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

                now_monotonic_ns = (
                    time.monotonic_ns()
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

                    frame_status = tracking_gate.inspect(
                        tracking,
                        now_monotonic_ns,
                    )

                    hand = (
                        get_hand_from_tracking(
                            tracking,
                            side="right",
                        )
                        if frame_status
                        == TrackingFrameStatus.CANDIDATE
                        else None
                    )

                    if (
                        frame_status
                        == TrackingFrameStatus.CANDIDATE
                        and hand is None
                    ):
                        tracking_gate.reject_malformed()
                        rejected_tracking_count += 1


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
                                    # Hand articulation is defined by the
                                    # legacy-compatible first 25 joints.  Keep
                                    # optional forearm[25:27] validity isolated
                                    # so a bad cue can fall back to position-only
                                    # without disrupting SG100 retargeting.
                                    hand[:25]
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

                            n_forearm_avp = (
                                extract_forearm_axis(hand)
                                if args.forearm_axis_ik
                                else None
                            )


                            frame_sequence = int(
                                tracking.frame_sequence
                            )

                            received_monotonic_ns = int(
                                tracking.received_monotonic_ns
                            )

                            if teleop_arming.observe_valid_frame(
                                frame_sequence
                            ):
                                print_teleop_state(
                                    "Valid right-hand tracking available. "
                                    "Hold a neutral pose and press C to arm."
                                )

                            # A parseable NEW + VALID frame refreshes tracking
                            # independently of whether teleop is armed.
                            tracking_gate.accept(
                                now_monotonic_ns
                            )
                            valid_tracking_count += 1

                            calibration_frame = (
                                teleop_arming.should_calibrate(
                                    frame_sequence,
                                    received_monotonic_ns,
                                )
                            )

                            # =====================================
                            # Explicit neutral calibration frame
                            # =====================================

                            if calibration_frame:
                                # Every calibration anchors both the mapper
                                # and filter to the current actual TCP pose.
                                T_tcp_anchor = make_transform(
                                    data.xpos[tcp_body_id].copy(),
                                    data.xmat[tcp_body_id]
                                    .reshape(3, 3)
                                    .copy(),
                                )

                                wrist_mapper.calibrate(
                                    T_avp_wrist,
                                    T_tcp_anchor,
                                )
                                tcp_target_filter.reset(
                                    T_tcp_anchor,
                                    timestamp=(
                                        received_monotonic_ns
                                        * 1e-9
                                    ),
                                )

                                # A valid 27-joint sample establishes the
                                # human forearm direction at this same neutral
                                # frame.  Invalid/legacy data clears the old
                                # anchor and keeps this session position-only.
                                if args.forearm_axis_ik:
                                    forearm_axis_calibrated = (
                                        n_forearm_avp is not None
                                        and wrist_mapper
                                        .calibrate_forearm_axis(
                                            n_forearm_avp
                                        )
                                    )
                                    n_forearm0_avp = (
                                        n_forearm_avp.copy()
                                        if forearm_axis_calibrated
                                        else None
                                    )
                                else:
                                    forearm_axis_calibrated = False
                                    n_forearm0_avp = None

                                T_avp_wrist_0 = (
                                    T_avp_wrist.copy()
                                )
                                T_tcp_runtime_0 = (
                                    T_tcp_anchor.copy()
                                )
                                # Orientation anchor for
                                # --lock-tcp-rotation: the TCP rotation
                                # at the moment the neutral anchor is
                                # captured.
                                R_tcp_rotation_lock = (
                                    T_tcp_anchor[
                                        :3,
                                        :3,
                                    ].copy()
                                )
                                q_hand0_runtime = (
                                    q_hand_candidate.copy()
                                )

                                q_hand_goal = (
                                    q_hand0_runtime.copy()
                                )
                                q_hand_cmd = (
                                    q_hand0_runtime.copy()
                                )
                                hand_command_filter.reset(
                                    q_hand0_runtime
                                )

                                wrist_calibrated = True
                                teleop_arming.complete_calibration()

                                print_teleop_state(
                                    "Neutral wrist and runtime hand anchors "
                                    "captured from the same frame."
                                )
                                print(
                                    "frame sequence       :",
                                    frame_sequence,
                                )
                                print(
                                    "AVP wrist anchor pos :",
                                    np.round(
                                        T_avp_wrist_0[:3, 3],
                                        6,
                                    ),
                                )
                                print(
                                    "Robot TCP anchor pos :",
                                    np.round(
                                        T_tcp_runtime_0[:3, 3],
                                        6,
                                    ),
                                )
                                print(
                                    "q_hand0_runtime      :",
                                    np.round(
                                        q_hand0_runtime,
                                        4,
                                    ),
                                )

                            # =====================================
                            # Armed wrist + hand teleoperation
                            # =====================================

                            elif teleop_arming.active:
                                T_tcp_raw = wrist_mapper.map(
                                    T_avp_wrist
                                )
                                T_tcp_target = (
                                    tcp_target_filter.update(
                                        T_tcp_raw,
                                        timestamp=(
                                            received_monotonic_ns
                                            * 1e-9
                                        ),
                                    )
                                )

                                p_target = (
                                    T_tcp_target[
                                        :3,
                                        3,
                                    ]
                                )


                                # ---------------------------------
                                # Solve RM65 IK using the previous
                                # valid solution as the next seed.
                                #
                                # forearm-axis: 3D translation plus TCP +Z
                                #   direction; local-Z roll is free. Missing
                                #   forearm data falls back to position-only.
                                # position-only: true 3D translation
                                #   task, orientation not constrained.
                                # lock-rotation: full 6D pose IK
                                #   against the calibration anchor.
                                # default: full 6D pose IK against
                                #   the mapped wrist orientation.
                                # ---------------------------------

                                if args.forearm_axis_ik:
                                    (
                                        q_arm_candidate,
                                        ik_result,
                                    ) = solve_forearm_axis_or_position(
                                        ik_solver,
                                        wrist_mapper,
                                        q_arm_goal,
                                        p_target,
                                        n_forearm_avp,
                                        forearm_axis_calibrated,
                                    )

                                elif args.position_only_ik:
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
                                                None
                                            ),
                                            position_only=(
                                                True
                                            ),
                                        )
                                    )

                                else:
                                    R_target = (
                                        select_full_pose_target_rotation(
                                            T_tcp_target,
                                            lock_tcp_rotation=(
                                                args.lock_tcp_rotation
                                            ),
                                            R_tcp_rotation_lock=(
                                                R_tcp_rotation_lock
                                            ),
                                        )
                                    )

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


                                # The hand goal comes from the same sensory
                                # frame used by the wrist/arm branch above.
                                q_hand_goal = (
                                    q_hand_candidate.copy()
                                )


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

                            tracking_gate.reject_malformed()
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

                if (
                    tracking_gate.tracking_timed_out(
                        now_monotonic_ns
                    )
                    and not tracking_lost_printed
                ):
                    if teleop_arming.disarm_for_tracking_timeout():
                        # Drop the mapper object so the old wrist anchor cannot
                        # be used after tracking reacquisition.
                        wrist_mapper = AVPWristRelativeMapper(
                            translation_scale=(
                                avp_to_tcp_config
                                .translation_scale
                            ),
                            R_tcp_from_avp_local=(
                                avp_to_tcp_config
                                .avp_to_tcp_local_rotation
                            ),
                        )
                        tcp_target_filter.clear()
                        wrist_calibrated = False
                        T_avp_wrist_0 = None
                        T_tcp_runtime_0 = None
                        R_tcp_rotation_lock = None
                        n_forearm0_avp = None
                        forearm_axis_calibrated = False
                        q_hand0_runtime = None
                        print_teleop_state(
                            "Tracking timeout invalidated the wrist anchor. "
                            "Reacquisition requires another C confirmation."
                        )

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

                    write_robot_controls(
                        data,
                        robot_layout,
                        q_arm_goal,
                        q_hand_cmd,
                    )


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
                        "duplicate frames:",
                        tracking_gate.duplicate_frames,
                    )

                    print(
                        "invalid frames  :",
                        tracking_gate.invalid_frames,
                    )

                    print(
                        "wrist calibrated:",
                        wrist_calibrated,
                    )

                    print(
                        "teleop state     :",
                        teleop_arming.state.value,
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
                                robot_layout.arm_qpos_indices
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
                                robot_layout.hand_qpos_indices
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
        "duplicate frames  :",
        tracking_gate.duplicate_frames,
    )

    print(
        "invalid frames    :",
        tracking_gate.invalid_frames,
    )

    print(
        "malformed frames  :",
        tracking_gate.malformed_frames,
    )

    print(
        "stale frames      :",
        tracking_gate.stale_frames,
    )

    print(
        "missing metadata  :",
        tracking_gate.missing_metadata_frames,
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
        "teleop state      :",
        teleop_arming.state.value,
    )

    print(
        "q_hand0_runtime   :",
        (
            "unset"
            if q_hand0_runtime is None
            else np.round(q_hand0_runtime, 4)
        ),
    )

    print(
        "========================================"
    )

if __name__ == "__main__":
    main()
