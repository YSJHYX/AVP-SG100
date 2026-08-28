from __future__ import annotations

import argparse
import math
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

from sg100_teleop.recording.mujoco_state_recorder import (
    MujocoStateRecorder,
)

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
    # "mjcf/rm65_sg100_avp_grasp_scene.xml"
    # "mjcf/rm65_sg100_demo_hammer_peg_assembly_scenes.xml"
    # "mjcf/rm65_sg100_demo_drawer_retrieval_and_loading_scene.xml"
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

# Verbose-only: an accepted IK solution whose largest single-frame joint
# step exceeds this is logged as a suspected discontinuity.  Large dq
# with a small pos_err means the same pose was reached from very
# different joints (branch flip); large dq with a large pos_err means a
# HOLD-release catch-up.  Normal 60 Hz teleop stays far below it.
IK_ACCEPT_JUMP_LOG_RAD = 0.2

# Per-frame joint-step clamp applied to accepted best-effort fallback
# solutions so a single solve can never produce a visible jump.  0.05
# rad per tracking frame aligns the fallback ceiling with the physics-
# rate arm command filter (~3 rad/s) so boundary following cannot
# out-slew the resume catch-up.
IK_FALLBACK_MAX_STEP_RAD = 0.05

# ============================================================
# Wrist-roll (position + forearm roll) teleoperation mode
#
# Task decomposition ported from the Quest teleop
# (teleoperate_rm65_sg100.py): TCP translation drives a
# best-effort position task, and the forearm roll extracted from
# the AVP wrist rotation drives joint 6 directly.  Wrist pitch and
# yaw deliberately do not feed the arm.  joint 6's position
# Jacobian column is exactly zero because sg100_base_link sits on
# the joint-6 axis, so the roll overwrite cannot disturb the
# position solution.
# ============================================================

WRIST_ROLL_MAX_RAD = np.radians(90.0)

# Roll gain and direction.  Flip the sign if joint 6 turns against
# the operator's wrist during the one-time visual validation.
WRIST_ROLL_SCALE = 1.0
WRIST_ROLL_SIGN = 1.0

# One-pole low-pass on the roll scalar (seconds).
WRIST_ROLL_FILTER_TAU = 0.06

# Fallback roll axis (wrist-local) when the calibration frame has
# no valid 27-joint forearm direction.
WRIST_ROLL_FALLBACK_AXIS_LOCAL = np.array(
    [0.0, 0.0, 1.0],
    dtype=np.float64,
)


def _wrap_to_pi(angle: float) -> float:
    """Wrap one angle to [-pi, pi]."""
    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def extract_wrist_roll(
    rotation: np.ndarray,
    axis: np.ndarray,
) -> float:
    """Extract the signed twist (roll) of ``rotation`` about ``axis``.

    Swing-twist decomposition ported from the Quest teleop: both
    arguments are expressed in the same frame, and the returned
    angle is the rotation's twist component about ``axis``.  q and
    -q encode the same rotation; keeping w non-negative selects the
    shortest signed twist and avoids a discontinuity at ordinary
    poses.
    """
    axis = np.asarray(axis, dtype=np.float64)
    axis_norm = float(np.linalg.norm(axis))
    if axis.shape != (3,) or axis_norm < 1e-12:
        raise ValueError("roll axis must be a non-zero 3-vector")

    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(
        quaternion,
        np.asarray(rotation, dtype=np.float64).reshape(9),
    )
    if quaternion[0] < 0.0:
        quaternion *= -1.0

    unit_axis = axis / axis_norm
    projected = float(np.dot(quaternion[1:], unit_axis))
    twist_norm = math.hypot(float(quaternion[0]), projected)
    if twist_norm < 1e-12:
        # A 180-degree pure swing has no uniquely defined twist.
        # Holding the calibrated wrist angle is the safest behaviour
        # at that singularity.
        return 0.0

    angle = 2.0 * math.atan2(projected, float(quaternion[0]))
    return _wrap_to_pi(angle)


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
    posture_reference: np.ndarray | None = None,
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
            posture_reference=posture_reference,
        )

    return ik_solver.solve(
        q_seed=q_seed,
        target_position=target_position,
        target_rotation=None,
        target_axis=a_target,
        tcp_axis_local=TCP_FOREARM_AXIS_LOCAL,
        posture_reference=posture_reference,
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

# The v8 multi-object scene removed the water_bottle body.  When it is
# absent, anchor the pre-grasp startup pose to this historical bottle
# placement (authored in the pre-refactor scene, commit fc4991e) so the
# startup configuration matches the pre-refactor behaviour exactly.
STARTUP_ANCHOR_FALLBACK_M = np.array(
    [0.46101150, 0.29368833, 0.11000000],
    dtype=np.float64,
)

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

    Returns the best-margin converged IK solution, or None when no
    seed converges.  Without a water_bottle body (v8 multi-object
    scene) the anchor falls back to the historical bottle placement
    so the startup pose stays aligned with the pre-refactor scene.
    """

    bottle_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "water_bottle",
    )

    if bottle_id < 0:
        bottle = STARTUP_ANCHOR_FALLBACK_M.copy()
    else:
        # A fresh MjData still holds the XML initial state, so this
        # reads the bottle's authored placement.
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
        "--arm-filter-tau",
        type=float,
        default=0.03,
        help="RM65 arm command low-pass time constant (s)",
    )

    parser.add_argument(
        "--max-arm-velocity",
        type=float,
        default=3.0,
        help="Maximum RM65 joint velocity (rad/s)",
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

    parser.add_argument(
        "--recording-dir",
        default="recording",
        help=(
            "Session root for lightweight MuJoCo visual-state recordings "
            "(default: recording)"
        ),
    )

    parser.add_argument(
        "--video-fps",
        type=float,
        default=30.0,
        help="Simulation-time visual-state sampling frequency (default: 30 Hz)",
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

    mode_group.add_argument(
        "--wrist-roll-ik",
        action="store_true",
        help=(
            "4-DoF IK: wrist translation controls TCP xyz and the "
            "AVP forearm roll drives joint 6 directly; wrist "
            "pitch/yaw are ignored. Position solving is best-effort "
            "damped least squares, so targets beyond the reachable "
            "set are followed along the workspace boundary instead "
            "of failing."
        ),
    )

    # ========================================================
    # Real RM65 backend (v3 RM65_真机迁移方案_v3.md §B2.3).
    # The sim default keeps the current behaviour unchanged.
    # ========================================================

    parser.add_argument(
        "--arm-backend",
        choices=("sim", "real"),
        default="sim",
        help=(
            "Arm execution backend: 'sim' drives MuJoCo actuators "
            "(current behaviour); 'real' streams joint targets to the "
            "RM65 controller over CANFD and drives MuJoCo as a "
            "feedback-followed kinematic twin (default: sim)"
        ),
    )

    parser.add_argument(
        "--arm-ip",
        default=None,
        help="RM65 controller IP for --arm-backend real (e.g. 192.168.1.18)",
    )

    parser.add_argument(
        "--arm-port",
        type=int,
        default=8080,
        help="RM65 controller TCP port (default: 8080)",
    )

    parser.add_argument(
        "--arm-follow",
        choices=("low", "filter", "high"),
        default="low",
        help=(
            "rm_movej_canfd follow mode: low (follow=False), filter "
            "(follow=True + trajectory_mode=2), high (follow=True raw); "
            "see v3 §A2.2 (default: low)"
        ),
    )

    parser.add_argument(
        "--arm-radio",
        type=int,
        default=0,
        help="rm_movej_canfd smoothing coefficient for filter mode (default: 0)",
    )

    parser.add_argument(
        "--arm-send-hz",
        type=float,
        default=100.0,
        help="Real backend CANFD sender rate in Hz (default: 100)",
    )

    parser.add_argument(
        "--arm-safety-profile",
        type=int,
        choices=(10, 25, 50),
        default=10,
        help=(
            "Software safety profile scaling the real-arm limiter: "
            "10 -> qdot 0.5 / qddot 2.0, 25 -> 1.25 / 5.0, "
            "50 -> 2.5 / 10.0 (rad, rad/s, rad/s^2) (default: 10)"
        ),
    )

    parser.add_argument(
        "--arm-tool-profile",
        choices=("flange_only", "sg100_attached"),
        default="flange_only",
        help="Tool profile for the real arm (default: flange_only)",
    )

    parser.add_argument(
        "--arm-target-timeout-ms",
        type=int,
        default=150,
        help="Real backend stale-target watchdog in ms (default: 150)",
    )

    parser.add_argument(
        "--arm-feedback-timeout-ms",
        type=int,
        default=200,
        help="Real backend stale-feedback watchdog in ms (default: 200)",
    )

    parser.add_argument(
        "--arm-max-follow-error-deg",
        type=float,
        default=20.0,
        help=(
            "Gross cmd-vs-actual position mismatch bound in deg for "
            "the real backend. Primary liveness check is velocity-based "
            "dead-arm detection; ordinary low-follow lag never reaches "
            "this bound (default: 20)"
        ),
    )

    parser.add_argument(
        "--arm-verbose",
        action="store_true",
        help="Print real backend events and timing statistics",
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

    if args.arm_filter_tau <= 0.0:
        raise ValueError(
            "--arm-filter-tau must be > 0"
        )

    if args.max_arm_velocity <= 0.0:
        raise ValueError(
            "--max-arm-velocity must be > 0"
        )

    if args.max_hand_velocity <= 0.0:
        raise ValueError(
            "--max-hand-velocity must be > 0"
        )
    
    if args.tracking_timeout <= 0.0:
        raise ValueError(
            "--tracking-timeout must be > 0"
        )

    if args.arm_backend == "real" and not args.arm_ip:
        raise ValueError(
            "--arm-backend real requires --arm-ip"
        )

    if args.arm_send_hz <= 0.0:
        raise ValueError(
            "--arm-send-hz must be > 0"
        )

    if args.arm_target_timeout_ms <= 0:
        raise ValueError(
            "--arm-target-timeout-ms must be > 0"
        )

    if args.arm_feedback_timeout_ms <= 0:
        raise ValueError(
            "--arm-feedback-timeout-ms must be > 0"
        )

    if args.arm_max_follow_error_deg <= 0.0:
        raise ValueError(
            "--arm-max-follow-error-deg must be > 0"
        )

    if not np.isfinite(args.video_fps) or args.video_fps <= 0.0:
        raise ValueError(
            "--video-fps must be finite and > 0"
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
            "wrist-roll (position + forearm roll, pitch/yaw free)"
            if args.wrist_roll_ik
            else (
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

    # ========================================================
    # Real RM65 backend (v3 §B2.3 / §A4).
    #
    # Created before the startup pose so real mode anchors every
    # downstream initial value (q_arm_goal, posture anchor, command
    # filters, twin state) to the actual arm configuration.  The
    # bottle pre-grasp startup IK is skipped: on the real arm the
    # robot starts wherever it physically is.
    # ========================================================

    real_arm_backend = None

    if args.arm_backend == "real":
        from sg100_teleop.robot.rm65_real import RM65RealBackend

        safety_profiles = {
            10: (0.5, 2.0),
            25: (1.25, 5.0),
            50: (2.5, 10.0),
        }
        qdot_max, qddot_max = safety_profiles[args.arm_safety_profile]

        def real_arm_event(name: str, detail: str) -> None:
            if args.arm_verbose or name in (
                "connected",
                "disarm",
                "send_failure",
            ):
                print(f"[REAL-ARM] {name}: {detail}")

        real_arm_backend = RM65RealBackend(
            args.arm_ip,
            args.arm_port,
            send_hz=args.arm_send_hz,
            follow=args.arm_follow,
            radio=args.arm_radio,
            qdot_max_rad_s=qdot_max,
            qddot_max_rad_s2=qddot_max,
            target_timeout_s=args.arm_target_timeout_ms * 1e-3,
            feedback_timeout_s=args.arm_feedback_timeout_ms * 1e-3,
            gross_follow_error_rad=np.deg2rad(
                args.arm_max_follow_error_deg
            ),
            on_event=real_arm_event,
        )
        real_arm_backend.connect()
        real_arm_backend.start()

        print(
            "[REAL-ARM] backend started "
            f"(follow={args.arm_follow}, {args.arm_send_hz} Hz, "
            f"PROFILE_{args.arm_safety_profile}, "
            f"tool={args.arm_tool_profile})"
        )
        print(
            "[REAL-ARM] keys: C = calibrate anchor | "
            "A = ARM / DISARM streaming | Ctrl-C = stop+disconnect"
        )
        print(
            "[REAL-ARM] streaming is DISABLED until you press A"
        )

    if real_arm_backend is not None:
        startup_arm_q = (
            real_arm_backend.joint_state().q_rad.copy()
        )
        print(
            "startup pose      : real arm actual q = "
            f"{np.round(startup_arm_q, 5).tolist()}"
        )
    else:
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

    # Posture anchor for the IK null-space term: the arm configuration
    # captured at neutral calibration.  Redundancy (position-only and
    # fallback solves) resolves toward this instead of random-walking.
    q_arm_reference = (
        startup_arm_q.copy()
    )

    # ========================================================
    # RM65 arm command filter
    #
    # q_arm_goal may still step discontinuously (solution branch
    # switch, fallback resume).  The physics-rate low-pass + joint
    # velocity limit converts any such step into a bounded-speed
    # slew, mirroring the SG100 hand path below.
    # ========================================================

    arm_command_filter = (
        JointCommandFilter(
            dof=ARM_DOF,
            tau=args.arm_filter_tau,
            max_velocity=(
                args.max_arm_velocity
            ),
        )
    )

    arm_command_filter.reset(
        startup_arm_q.copy()
    )

    q_arm_cmd = (
        q_arm_goal.copy()
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

    # Wrist-roll mode anchors: joint-6 zero, the roll axis captured
    # at the neutral frame, and the filtered roll state.
    q6_roll_anchor = None
    roll_axis_anchor = None
    filtered_roll = 0.0
    last_roll_time_s = None

    # ========================================================
    # IK diagnostics
    # ========================================================

    last_ik_result = None

    ik_failure_count = 0

    ik_fallback_count = 0

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
        "arm filter tau    :",
        args.arm_filter_tau,
        "s",
    )

    print(
        "max arm velocity  :",
        args.max_arm_velocity,
        "rad/s",
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
    # Lightweight MuJoCo visual-state recording
    #
    # This performs no rendering or video encoding. Sampling is
    # driven by data.time and finalized once when the session ends.
    # ========================================================

    state_recorder = MujocoStateRecorder(
        model,
        scene_path=model_path,
        recording_dir=args.recording_dir,
        record_fps=args.video_fps,
        camera_name="workstation_front",
    )
    recording_session_dir = state_recorder.start(data)

    print(
        "recording session :",
        recording_session_dir,
    )
    print(
        "recording rate    :",
        args.video_fps,
        "FPS (simulation time)",
    )


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

        elif keycode in (ord("A"), ord("a")) and (
            real_arm_backend is not None
        ):
            # v3 §A4.2: calibration (C) and arm-enable (A) are separate
            # actions.  Disarm order follows §A3.5: quiesce the sender
            # first, then command the decelerated stop.
            if real_arm_backend.enabled:
                real_arm_backend.set_enabled(False)
                real_arm_backend.stop_smooth()
                print(
                    "[REAL-ARM] DISARMED (A); "
                    "sender disabled, slow stop commanded"
                )
            else:
                real_arm_backend.set_enabled(True)
                print(
                    "[REAL-ARM] ARMED (A); streaming enabled"
                )

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
                                if (
                                    args.forearm_axis_ik
                                    or args.wrist_roll_ik
                                )
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
                                # Posture anchor for the IK null-space
                                # term, captured with the same neutral
                                # frame.
                                q_arm_reference = (
                                    q_arm_goal.copy()
                                )

                                # Wrist-roll anchors: the joint-6 zero
                                # and the roll axis.  The axis is the
                                # forearm direction at this neutral
                                # frame expressed in the anchor wrist
                                # frame; without valid 27-joint
                                # forearm data the wrist-local
                                # fallback axis is used.
                                if args.wrist_roll_ik:
                                    q6_roll_anchor = float(
                                        q_arm_reference[5]
                                    )

                                    roll_axis_anchor = None
                                    if (
                                        n_forearm_avp is not None
                                    ):
                                        axis = (
                                            T_avp_wrist_0[:3, :3].T
                                            @ n_forearm_avp
                                        )
                                        axis_norm = float(
                                            np.linalg.norm(axis)
                                        )
                                        if axis_norm > 1e-12:
                                            roll_axis_anchor = (
                                                axis / axis_norm
                                            )
                                    if roll_axis_anchor is None:
                                        roll_axis_anchor = (
                                            WRIST_ROLL_FALLBACK_AXIS_LOCAL
                                            .copy()
                                        )

                                    filtered_roll = 0.0
                                    last_roll_time_s = None
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

                                if args.wrist_roll_ik:
                                    # Position task: best-effort DLS
                                    # with the posture anchor.  The
                                    # roll overwrite below cannot
                                    # disturb the position solution
                                    # because joint 6's position
                                    # Jacobian column is zero.
                                    (
                                        q_arm_candidate,
                                        ik_result,
                                    ) = ik_solver.solve(
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
                                        best_effort=(
                                            True
                                        ),
                                        posture_reference=(
                                            q_arm_reference
                                        ),
                                    )

                                    # Roll channel: twist of the
                                    # anchor-relative wrist rotation
                                    # about the calibrated forearm
                                    # direction, low-passed on the
                                    # frame timestamps.
                                    R_rel_anchor = (
                                        T_avp_wrist_0[:3, :3].T
                                        @ T_avp_wrist[:3, :3]
                                    )
                                    raw_roll = float(
                                        np.clip(
                                            WRIST_ROLL_SIGN
                                            * WRIST_ROLL_SCALE
                                            * extract_wrist_roll(
                                                R_rel_anchor,
                                                roll_axis_anchor,
                                            ),
                                            -WRIST_ROLL_MAX_RAD,
                                            WRIST_ROLL_MAX_RAD,
                                        )
                                    )

                                    roll_time_s = (
                                        received_monotonic_ns * 1e-9
                                    )
                                    if (
                                        last_roll_time_s is None
                                        or roll_time_s
                                        <= last_roll_time_s
                                    ):
                                        filtered_roll = raw_roll
                                    else:
                                        roll_dt = (
                                            roll_time_s
                                            - last_roll_time_s
                                        )
                                        roll_alpha = (
                                            1.0
                                            - math.exp(
                                                -roll_dt
                                                / WRIST_ROLL_FILTER_TAU
                                            )
                                        )
                                        filtered_roll += (
                                            roll_alpha
                                            * (
                                                raw_roll
                                                - filtered_roll
                                            )
                                        )
                                    last_roll_time_s = roll_time_s

                                    q_arm_candidate[5] = (
                                        np.clip(
                                            q6_roll_anchor
                                            + filtered_roll,
                                            ik_solver.lower[5],
                                            ik_solver.upper[5],
                                        )
                                    )

                                elif args.forearm_axis_ik:
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
                                        posture_reference=(
                                            q_arm_reference
                                        ),
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
                                            posture_reference=(
                                                q_arm_reference
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
                                # solution.  Wrist-roll mode always
                                # accepts its best-effort position
                                # result plus the direct roll channel
                                # (Quest-style always-track
                                # semantics); the generic fallback
                                # branch is intentionally bypassed
                                # there.
                                # ---------------------------------

                                if (
                                    ik_result.converged
                                    or args.wrist_roll_ik
                                ):

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

                                    accept_dq = np.abs(
                                        q_arm_candidate
                                        - q_arm_goal
                                    )

                                    accept_dq_max = float(
                                        np.max(
                                            accept_dq,
                                        )
                                    )

                                    if (
                                        args.verbose
                                        and accept_dq_max
                                        > IK_ACCEPT_JUMP_LOG_RAD
                                    ):
                                        print(
                                            "IK accept jump: "
                                            f"dq_max="
                                            f"{accept_dq_max:.3f} rad "
                                            f"({np.degrees(accept_dq_max):.1f} deg), "
                                            f"dq="
                                            f"{np.round(accept_dq, 3).tolist()}, "
                                            f"reason="
                                            f"{ik_result.reason}, "
                                            f"pos_err="
                                            f"{ik_result.position_error * 1000.0:.3f} mm, "
                                            f"rot_err="
                                            f"{np.degrees(ik_result.rotation_error):.3f} deg"
                                        )

                                    q_arm_goal = (
                                        q_arm_candidate
                                        .copy()
                                    )


                                # ---------------------------------
                                # IK failure:
                                #
                                # A plain hold would freeze the arm
                                # while the target keeps drifting and
                                # then snap back in a single frame
                                # when the target re-enters the
                                # feasible set.  Instead, fall back to
                                # the damped position task in
                                # best-effort mode: the arm keeps
                                # tracking from the closest reachable
                                # configuration, orientation waits.
                                # The per-frame clamp bounds any
                                # single solve to a continuous step.
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
                                            f"{ik_result.condition_number:.2f}, "
                                            f"reason="
                                            f"{ik_result.reason}"
                                        )

                                    (
                                        q_arm_fallback,
                                        fallback_result,
                                    ) = ik_solver.solve(
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
                                        best_effort=(
                                            True
                                        ),
                                        posture_reference=(
                                            q_arm_reference
                                        ),
                                    )

                                    if (
                                        q_arm_fallback.shape
                                        == (ARM_DOF,)
                                        and np.all(
                                            np.isfinite(
                                                q_arm_fallback
                                            )
                                        )
                                    ):
                                        q_arm_goal = np.clip(
                                            q_arm_fallback,
                                            q_arm_goal
                                            - IK_FALLBACK_MAX_STEP_RAD,
                                            q_arm_goal
                                            + IK_FALLBACK_MAX_STEP_RAD,
                                        )

                                        ik_fallback_count += 1

                                        last_ik_result = (
                                            fallback_result
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
                        # Real arm: quiesce the sender first, then
                        # command the decelerated stop (v3 §A3.5).
                        if (
                            real_arm_backend is not None
                            and real_arm_backend.enabled
                        ):
                            real_arm_backend.set_enabled(False)
                            real_arm_backend.stop_smooth()
                            print(
                                "[REAL-ARM] DISARMED (tracking "
                                "timeout); slow stop commanded"
                            )
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
                        q6_roll_anchor = None
                        roll_axis_anchor = None
                        filtered_roll = 0.0
                        last_roll_time_s = None
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
                    # low-rate q_arm_goal
                    #         ↓
                    # JointCommandFilter @ physics rate
                    #         ↓
                    # high-rate q_arm_cmd
                    #
                    # Any upstream discontinuity (branch switch,
                    # fallback resume) degrades into a bounded-speed
                    # slew instead of a snap.
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

                    if real_arm_backend is not None:
                        # ==========================================
                        # Real backend path (v3 §A5.3 kinematic twin)
                        #
                        # - publish the 60 Hz IK goal; the backend
                        #   limiter owns real-arm rate limiting with
                        #   its measured sender dt (the sim
                        #   ArmCommandFilter is bypassed to avoid
                        #   double filtering),
                        # - arm joints follow real feedback,
                        # - hand joints follow the filtered retarget
                        #   targets,
                        # - no mj_step: the twin is kinematic, so the
                        #   calibration anchor read from the twin
                        #   equals actual-q + software FK (§A6.2).
                        # ==========================================

                        real_arm_backend.send_joint_target(
                            q_arm_goal
                        )

                        q_hand_cmd = (
                            hand_command_filter.update(
                                q_hand_goal,
                                physics_dt,
                            )
                        )

                        feedback = (
                            real_arm_backend.joint_state()
                        )
                        if feedback is not None:
                            data.qpos[
                                robot_layout.arm_qpos_indices
                            ] = feedback.q_rad

                        data.qpos[
                            robot_layout.hand_qpos_indices
                        ] = q_hand_cmd

                        mujoco.mj_forward(
                            model,
                            data,
                        )

                        state_recorder.maybe_record(
                            data
                        )

                    else:
                        q_arm_cmd = (
                            arm_command_filter.update(
                                q_arm_goal,
                                physics_dt,
                            )
                        )

                        q_hand_cmd = (
                            hand_command_filter.update(
                                q_hand_goal,
                                physics_dt,
                            )
                        )

                        write_robot_controls(
                            data,
                            robot_layout,
                            q_arm_cmd,
                            q_hand_cmd,
                        )


                        # ---------------------------------------------
                        # Advance MuJoCo by one physics timestep.
                        # ---------------------------------------------

                        mujoco.mj_step(
                            model,
                            data,
                        )

                        state_recorder.maybe_record(
                            data
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
                        "IK fallbacks    :",
                        ik_fallback_count,
                    )

                    print(
                        "q_arm_goal      :",
                        np.round(
                            q_arm_goal,
                            3,
                        ),
                    )

                    print(
                        "arm cmd         :",
                        np.round(
                            q_arm_cmd,
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

                    if real_arm_backend is not None:
                        stats = real_arm_backend.sender_stats
                        if stats.get("count", 0) > 0:
                            print(
                                "real sender      :",
                                f"n={int(stats['count'])} "
                                f"mean={stats['mean_s'] * 1e3:.2f}ms "
                                f"p95={stats['p95_s'] * 1e3:.2f}ms "
                                f"p99={stats['p99_s'] * 1e3:.2f}ms "
                                f"max={stats['max_s'] * 1e3:.2f}ms "
                                f"misses={int(stats['deadline_misses'])}",
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
        print()

    finally:
        if real_arm_backend is not None:
            # v3 §A3.5 order on exit: quiesce, slow stop, disconnect.
            try:
                real_arm_backend.set_enabled(False)
                real_arm_backend.stop_smooth()
            finally:
                real_arm_backend.disconnect()
                print("[REAL-ARM] backend disconnected")
        recording_path = state_recorder.stop()
        print(
            "MuJoCo recording :",
            recording_path,
        )
        print(
            "recorded frames  :",
            state_recorder.frame_count,
        )

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
        "IK fallbacks      :",
        ik_fallback_count,
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
