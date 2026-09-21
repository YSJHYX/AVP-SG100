from __future__ import annotations

import argparse
import dataclasses
import math
import os
import sys
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

# Keep the documented ``python sg100_teleop/sim/...py`` invocation working
# even when this checkout has not been installed in editable mode.
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import mujoco
import mujoco.viewer
import numpy as np
import yaml

from avp_stream import VisionProStreamer

from sg100_teleop.calibration.io import load_calibration

from sg100_teleop.hand.canonical_hand import CanonicalHandState

from sg100_teleop.control.joint_command_filter import JointCommandFilter

from sg100_teleop.control.grasp_mode import (
    CLAW_THUMB_J1_RAD,
    GraspMode,
    GraspModeController,
)

from sg100_teleop.control.scripted_hand import (
    ScriptHandController,
    ScriptMotion,
)

from sg100_teleop.control.tcp_target_filter import TcpTargetFilter

from sg100_teleop.control.teleop_arming import (
    TeleopArmingController,
)

from sg100_teleop.data_collection.collector import (
    ArmHandDataCollector,
    CollectorConfig,
)

from sg100_teleop.kinematics.rm65_ik import RM65IKSolver

from sg100_teleop.retargeting.sg100_retargeter import SG100Retargeter

from sg100_teleop.retargeting.thumb_debug import (
    build_lock_disabled_retargeter,
    format_thumb_retarget_debug,
    hand_tip_geometry_mm,
)

from sg100_teleop.robot.sg100_driver import (
    MODE_JOINT_POSITION,
    SG100CommandLimitContract,
    SG100HardwareMap,
)

from sg100_teleop.robot.sg100_mock_driver import MockSG100Driver

from sg100_teleop.robot.rm65_sim import (
    RM65_JOINT_LOWER_RAD,
    RM65_JOINT_UPPER_RAD,
)

from sg100_teleop.recording.arm_joint_comparison import (
    ArmJointComparisonRecorder,
)

from sg100_teleop.recording.mujoco_state_recorder import (
    MujocoStateRecorder,
)
from sg100_teleop.recording.real_hardware_error_recorder import (
    RealHardwareErrorRecorder,
)

from sg100_teleop.sim.combined_robot_layout import (
    ARM_DOF,
    HAND_DOF,
    SG100_JOINT_ORDER,
    initialize_robot_state,
    resolve_robot_layout,
    write_robot_controls,
)

from sg100_teleop.sim.hammer_impact_diagnostic import (
    HammerImpactDiagnostic,
    enable_collision_debug_visualization,
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
    # "mjcf/rm65_sg100_demo_hammer_peg_assembly_scene.xml"
    # "mjcf/rm65_sg100_demo_drawer_loading_and_closing_scene.xml"
    # "mjcf/rm65_sg100_demo_maintenance_kit_packing_scene.xml"
    'mjcf/rm65_sg100_realworld_teleop_scene.xml' 
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

# When both physical backends are selected, startup is deterministic: the
# SG100 receives thumb_j2=-31 deg and 0 deg on the other ten joints through
# its vendor SDK, then the RM65 executes one planned, blocking move to
# 15 degrees on J1-J6.
# The conservative RealMan speed matches the existing PROFILE_10 bring-up
# convention; teleoperation streaming remains disabled until the operator's
# normal C/A gate.
REAL_STARTUP_HAND_Q_RAD = np.zeros(HAND_DOF, dtype=np.float64)
REAL_STARTUP_HAND_Q_RAD[
    SG100_JOINT_ORDER.index("thumb_j2")
] = np.deg2rad(-31.0)
REAL_STARTUP_ARM_Q_RAD = np.deg2rad(
    np.full(ARM_DOF, 15.0, dtype=np.float64)
)
REAL_STARTUP_ARM_SPEED_PERCENT = 10

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

# ============================================================
# Optional operator-defined real-hardware startup pose
# ============================================================

INITIAL_ARM_JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))
INITIAL_HAND_JOINT_NAMES = tuple(SG100_JOINT_ORDER)

# Same vendor-evidenced low-gain group used by the restricted MODE7 path.
INITIAL_HAND_KP = 0.1
INITIAL_HAND_KI = 0.0
INITIAL_HAND_KD = 0.0
INITIAL_HAND_OUTPUT_LIMIT = 0.5

_RM_INITIAL_STATUS_TEXT = {
    0: "success",
    1: "controller rejected the request",
    -1: "send failed",
    -2: "receive failed or timed out",
    -3: "reply parsing failed",
    -4: "in-position device check failed",
    -5: "single-thread receive timed out",
}


class TeleopInitialPoseError(RuntimeError):
    """Fail-closed startup-pose configuration or hardware error."""


@dataclasses.dataclass(frozen=True)
class InitialArmPoseConfig:
    angles_deg: np.ndarray
    speed_percent: int
    timeout_s: int
    tolerance_deg: float
    max_start_delta_deg: float


@dataclasses.dataclass(frozen=True)
class InitialHandPoseConfig:
    angles_deg: np.ndarray
    command_duration_s: float


@dataclasses.dataclass(frozen=True)
class TeleopInitialPoseConfig:
    arm: InitialArmPoseConfig
    hand: InitialHandPoseConfig


@dataclasses.dataclass(frozen=True)
class TeleopInitialPosePlan:
    config: TeleopInitialPoseConfig
    hand_sdk_user_deg: np.ndarray


def _initial_pose_mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise TeleopInitialPoseError(f"{field} must be a YAML mapping")
    return value


def _initial_pose_float(value: Any, field: str) -> float:
    if isinstance(value, bool) or value is None:
        raise TeleopInitialPoseError(f"{field} must be a finite number")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise TeleopInitialPoseError(
            f"{field} must be a finite number"
        ) from exc
    if not np.isfinite(parsed):
        raise TeleopInitialPoseError(f"{field} must be a finite number")
    return parsed


def _initial_pose_int(
    value: Any,
    field: str,
    low: int,
    high: int,
) -> int:
    if isinstance(value, bool):
        raise TeleopInitialPoseError(
            f"{field} must be an integer in [{low}, {high}]"
        )
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise TeleopInitialPoseError(
            f"{field} must be an integer in [{low}, {high}]"
        ) from exc
    if parsed != value or not low <= parsed <= high:
        raise TeleopInitialPoseError(
            f"{field} must be an integer in [{low}, {high}]"
        )
    return parsed


def _initial_pose_joint_vector(
    value: Any,
    expected_names: Sequence[str],
    field: str,
) -> np.ndarray:
    mapping = _initial_pose_mapping(value, field)
    expected = set(expected_names)
    provided = set(mapping)
    missing = [name for name in expected_names if name not in provided]
    extra = sorted(str(name) for name in provided - expected)
    if missing or extra:
        details = []
        if missing:
            details.append(f"missing {missing}")
        if extra:
            details.append(f"unknown {extra}")
        raise TeleopInitialPoseError(f"{field}: " + "; ".join(details))
    return np.asarray(
        [
            _initial_pose_float(mapping[name], f"{field}.{name}")
            for name in expected_names
        ],
        dtype=np.float64,
    )


def load_teleop_initial_pose_config(
    path: Path | str,
) -> TeleopInitialPoseConfig:
    """Load a strict named-joint startup pose; all angles are degrees."""

    config_path = Path(path).expanduser().resolve()
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise TeleopInitialPoseError(
            f"cannot read initial-pose config {config_path}: {exc}"
        ) from exc
    except yaml.YAMLError as exc:
        raise TeleopInitialPoseError(
            f"invalid YAML in initial-pose config {config_path}: {exc}"
        ) from exc

    root = _initial_pose_mapping(raw, "root")
    if root.get("version") != 1:
        raise TeleopInitialPoseError("initial-pose config version must be 1")
    if root.get("angle_unit") != "deg":
        raise TeleopInitialPoseError("angle_unit must be exactly 'deg'")

    arm = _initial_pose_mapping(root.get("arm"), "arm")
    hand = _initial_pose_mapping(root.get("hand"), "hand")
    arm_tolerance = _initial_pose_float(
        arm.get("tolerance_deg", 1.0), "arm.tolerance_deg"
    )
    max_start_delta = _initial_pose_float(
        arm.get("max_start_delta_deg", 120.0),
        "arm.max_start_delta_deg",
    )
    hand_duration = _initial_pose_float(
        hand.get("command_duration_s", 3.0),
        "hand.command_duration_s",
    )
    if not 0.0 < arm_tolerance <= 5.0:
        raise TeleopInitialPoseError("arm.tolerance_deg must be in (0, 5]")
    if not 0.0 < max_start_delta <= 360.0:
        raise TeleopInitialPoseError(
            "arm.max_start_delta_deg must be in (0, 360]"
        )
    if not 0.1 <= hand_duration <= 30.0:
        raise TeleopInitialPoseError(
            "hand.command_duration_s must be in [0.1, 30]"
        )

    return TeleopInitialPoseConfig(
        arm=InitialArmPoseConfig(
            angles_deg=_initial_pose_joint_vector(
                arm.get("angles_deg"),
                INITIAL_ARM_JOINT_NAMES,
                "arm.angles_deg",
            ),
            speed_percent=_initial_pose_int(
                arm.get("speed_percent", 5),
                "arm.speed_percent",
                1,
                20,
            ),
            timeout_s=_initial_pose_int(
                arm.get("timeout_s", 60),
                "arm.timeout_s",
                1,
                180,
            ),
            tolerance_deg=arm_tolerance,
            max_start_delta_deg=max_start_delta,
        ),
        hand=InitialHandPoseConfig(
            angles_deg=_initial_pose_joint_vector(
                hand.get("angles_deg"),
                INITIAL_HAND_JOINT_NAMES,
                "hand.angles_deg",
            ),
            command_duration_s=hand_duration,
        ),
    )


def build_teleop_initial_pose_plan(
    config: TeleopInitialPoseConfig,
    *,
    sdk_dir: Path | str | None = None,
) -> TeleopInitialPosePlan:
    """Validate arm, canonical hand, and runtime SDK limits offline."""

    arm_low = np.rad2deg(RM65_JOINT_LOWER_RAD)
    arm_high = np.rad2deg(RM65_JOINT_UPPER_RAD)
    outside_arm = np.flatnonzero(
        (config.arm.angles_deg < arm_low)
        | (config.arm.angles_deg > arm_high)
    )
    if outside_arm.size:
        messages = [
            (
                f"{INITIAL_ARM_JOINT_NAMES[index]}="
                f"{config.arm.angles_deg[index]:.3f} deg outside "
                f"[{arm_low[index]:.3f}, {arm_high[index]:.3f}]"
            )
            for index in outside_arm
        ]
        raise TeleopInitialPoseError(
            "invalid RM65 initial target: " + "; ".join(messages)
        )

    hardware_map = SG100HardwareMap()
    if tuple(hardware_map.software_joint_order) != INITIAL_HAND_JOINT_NAMES:
        raise TeleopInitialPoseError(
            "SG100 joint order changed; update initial-pose integration"
        )
    hand_rad = np.deg2rad(config.hand.angles_deg)
    outside_hand = np.flatnonzero(
        (hand_rad < hardware_map.lower_rad)
        | (hand_rad > hardware_map.upper_rad)
    )
    if outside_hand.size:
        messages = [
            (
                f"{INITIAL_HAND_JOINT_NAMES[index]}="
                f"{config.hand.angles_deg[index]:.3f} deg outside "
                "canonical range "
                f"[{np.rad2deg(hardware_map.lower_rad[index]):.3f}, "
                f"{np.rad2deg(hardware_map.upper_rad[index]):.3f}]"
            )
            for index in outside_hand
        ]
        raise TeleopInitialPoseError(
            "invalid SG100 initial target: " + "; ".join(messages)
        )

    try:
        from sg100_teleop.robot.sg100_sdk_driver import (
            DEFAULT_SDK_DIR,
            load_vendor_user_ranges,
        )

        resolved_sdk_dir = DEFAULT_SDK_DIR if sdk_dir is None else Path(sdk_dir)
        vendor_ranges = load_vendor_user_ranges(resolved_sdk_dir)
    except Exception as exc:
        raise TeleopInitialPoseError(
            f"cannot load SG100 runtime angle limits: {exc}"
        ) from exc
    if vendor_ranges is None:
        raise TeleopInitialPoseError(
            f"SG100 runtime angle map is missing under {resolved_sdk_dir}"
        )
    application = SG100CommandLimitContract(
        hardware_map,
        vendor_ranges,
    ).apply(hand_rad)
    if (
        application.canonical_clamped_indices
        or application.sdk_clamped_indices
        or not np.allclose(
            application.effective_canonical_rad,
            hand_rad,
            rtol=0.0,
            atol=1e-7,
        )
    ):
        raise TeleopInitialPoseError(
            "SG100 initial target would be clamped by runtime SDK limits; "
            "edit the YAML instead of relying on implicit clamping"
        )
    return TeleopInitialPosePlan(
        config=config,
        hand_sdk_user_deg=application.sdk_sent_deg.copy(),
    )


def _print_initial_pose_vector(
    title: str,
    names: Sequence[str],
    values: np.ndarray,
) -> None:
    print(title)
    for name, value in zip(names, values, strict=True):
        print(f"  {name:>10s}: {value:9.3f} deg")


def print_teleop_initial_pose_plan(
    plan: TeleopInitialPosePlan,
    *,
    arm_ip: str,
    arm_port: int,
    hand_port: str,
    hand_baudrate: int,
) -> None:
    config = plan.config
    print("\n=== REAL TELEOP INITIAL POSE PLAN (NO MOTION YET) ===")
    print(f"RM65 endpoint : {arm_ip}:{arm_port}")
    print(f"SG100 endpoint: {hand_port} @ {hand_baudrate}")
    _print_initial_pose_vector(
        "RM65 target (controller degrees):",
        INITIAL_ARM_JOINT_NAMES,
        config.arm.angles_deg,
    )
    _print_initial_pose_vector(
        "SG100 target (canonical degrees):",
        INITIAL_HAND_JOINT_NAMES,
        config.hand.angles_deg,
    )
    _print_initial_pose_vector(
        "SG100 mapped SDK user degrees:",
        INITIAL_HAND_JOINT_NAMES,
        plan.hand_sdk_user_deg,
    )
    print(
        f"RM65 motion   : planned movej, {config.arm.speed_percent}% speed, "
        f"timeout {config.arm.timeout_s}s"
    )
    print(
        f"SG100 motion  : MODE7 low gain "
        f"{INITIAL_HAND_KP}/{INITIAL_HAND_KI}/{INITIAL_HAND_KD}, "
        f"output_limit={INITIAL_HAND_OUTPUT_LIMIT}, command for "
        f"{config.hand.command_duration_s:g}s"
    )
    print("Movement order: RM65 first, then SG100, then WAIT-C teleop.")


class RM65InitialPoseSession:
    """Own one low-speed planned RM65 startup move before CANFD starts."""

    def __init__(
        self,
        ip: str,
        port: int,
        config: InitialArmPoseConfig,
    ) -> None:
        self.ip = str(ip)
        self.port = int(port)
        self.config = config
        self._arm: Any | None = None

    def connect(self) -> None:
        try:
            from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
            from Robotic_Arm.rm_robot_interface import RoboticArm
        except ImportError as exc:
            raise TeleopInitialPoseError(
                "Robotic_Arm SDK is unavailable in this Python environment"
            ) from exc
        arm = RoboticArm(rm_thread_mode_e.RM_SINGLE_MODE_E)
        handle = arm.rm_create_robot_arm(self.ip, self.port)
        if handle is None or getattr(handle, "id", -1) == -1:
            arm.rm_destroy()
            raise TeleopInitialPoseError(
                f"cannot connect to RM65 at {self.ip}:{self.port}"
            )
        if int(getattr(arm, "arm_dof", -1)) != ARM_DOF:
            try:
                arm.rm_delete_robot_arm()
            finally:
                arm.rm_destroy()
            raise TeleopInitialPoseError(
                f"expected {ARM_DOF}-DoF RM65, controller reports "
                f"{getattr(arm, 'arm_dof', None)}"
            )
        self._arm = arm

    def read_joint_deg(self) -> np.ndarray:
        if self._arm is None:
            raise TeleopInitialPoseError("RM65 is not connected")
        tag, joints = self._arm.rm_get_joint_degree()
        if int(tag) != 0:
            raise TeleopInitialPoseError(
                f"rm_get_joint_degree -> {tag} "
                f"({_RM_INITIAL_STATUS_TEXT.get(tag, 'unknown')})"
            )
        values = np.asarray(joints, dtype=np.float64)[:ARM_DOF]
        if values.shape != (ARM_DOF,) or not np.all(np.isfinite(values)):
            raise TeleopInitialPoseError("RM65 returned invalid joint feedback")
        return values

    def require_controller_ready(self) -> None:
        if self._arm is None:
            raise TeleopInitialPoseError("RM65 is not connected")
        tag, state = self._arm.rm_get_current_arm_state()
        if int(tag) != 0:
            raise TeleopInitialPoseError(
                f"rm_get_current_arm_state -> {tag} "
                f"({_RM_INITIAL_STATUS_TEXT.get(tag, 'unknown')})"
            )
        err = state.get("err") if isinstance(state, Mapping) else None
        try:
            codes = err.get("err", []) if isinstance(err, Mapping) else []
            controller_error = int(codes[0]) if codes else 0
        except (TypeError, ValueError, IndexError) as exc:
            raise TeleopInitialPoseError(
                "cannot parse RM65 controller error state"
            ) from exc
        if controller_error != 0:
            raise TeleopInitialPoseError(
                f"RM65 controller error is active: {controller_error}"
            )

    def validate_start_delta(self, current_deg: np.ndarray) -> np.ndarray:
        current = np.asarray(current_deg, dtype=np.float64)
        if current.shape != (ARM_DOF,) or not np.all(np.isfinite(current)):
            raise TeleopInitialPoseError("RM65 current pose must be finite 6D")
        delta = self.config.angles_deg - current
        too_large = np.flatnonzero(
            np.abs(delta) > self.config.max_start_delta_deg
        )
        if too_large.size:
            details = "; ".join(
                f"{INITIAL_ARM_JOINT_NAMES[index]} "
                f"delta={delta[index]:+.3f} deg"
                for index in too_large
            )
            raise TeleopInitialPoseError(
                "RM65 startup motion exceeds max_start_delta_deg="
                f"{self.config.max_start_delta_deg:g}: {details}"
            )
        return delta

    def move_and_verify(self) -> tuple[np.ndarray, np.ndarray]:
        if self._arm is None:
            raise TeleopInitialPoseError("RM65 is not connected")
        tag = int(
            self._arm.rm_movej(
                self.config.angles_deg.tolist(),
                self.config.speed_percent,
                0,
                0,
                self.config.timeout_s,
            )
        )
        if tag != 0:
            raise TeleopInitialPoseError(
                f"rm_movej -> {tag} "
                f"({_RM_INITIAL_STATUS_TEXT.get(tag, 'unknown')})"
            )
        reached = self.read_joint_deg()
        error = reached - self.config.angles_deg
        if np.max(np.abs(error)) > self.config.tolerance_deg:
            raise TeleopInitialPoseError(
                "RM65 did not reach the initial pose: max error "
                f"{np.max(np.abs(error)):.3f} deg > "
                f"{self.config.tolerance_deg:.3f} deg"
            )
        return reached, error

    def slow_stop(self) -> None:
        if self._arm is None:
            return
        try:
            self._arm.rm_set_arm_slow_stop()
        except Exception:
            pass

    def close(self) -> None:
        arm, self._arm = self._arm, None
        if arm is None:
            return
        try:
            arm.rm_delete_robot_arm()
        finally:
            arm.rm_destroy()

    def __enter__(self) -> "RM65InitialPoseSession":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        if exc_type is not None:
            self.slow_stop()
        self.close()


def initialize_rm65_initial_pose(
    ip: str,
    port: int,
    config: InitialArmPoseConfig,
    *,
    auto_confirm: bool,
) -> tuple[np.ndarray, np.ndarray]:
    """Confirm, execute, and verify the planned RM65 startup move."""

    print(
        "\n[WARNING] The real RM65 and all 11 SG100 joints will move. "
        "Clear the workspace and keep an operator at the physical "
        "emergency stop."
    )
    with RM65InitialPoseSession(ip, port, config) as arm:
        arm.require_controller_ready()
        current = arm.read_joint_deg()
        delta = arm.validate_start_delta(current)
        _print_initial_pose_vector(
            "RM65 current feedback:", INITIAL_ARM_JOINT_NAMES, current
        )
        _print_initial_pose_vector(
            "RM65 requested delta:", INITIAL_ARM_JOINT_NAMES, delta
        )
        if not auto_confirm:
            response = input(
                "Type INITIALIZE to run the RM65 + SG100 startup pose: "
            ).strip()
            if response != "INITIALIZE":
                raise TeleopInitialPoseError(
                    "operator confirmation did not match; no motion sent"
                )
        print("\n[INITIAL POSE 1/2] Moving RM65 with planned movej...")
        reached, error = arm.move_and_verify()
        _print_initial_pose_vector(
            "[RM65] reached feedback:", INITIAL_ARM_JOINT_NAMES, reached
        )
        _print_initial_pose_vector(
            "[RM65] final error:", INITIAL_ARM_JOINT_NAMES, error
        )
        return reached, error


def initialize_sg100_initial_pose(
    driver,
    config: InitialHandPoseConfig,
) -> np.ndarray:
    """Send one bounded low-gain target, observe fresh MODE7, then passivate."""

    application = driver.command_limit_contract.apply(
        np.deg2rad(config.angles_deg)
    )
    if (
        application.canonical_clamped_indices
        or application.sdk_clamped_indices
        or not np.allclose(
            application.effective_canonical_rad,
            np.deg2rad(config.angles_deg),
            rtol=0.0,
            atol=1e-7,
        )
    ):
        raise TeleopInitialPoseError(
            "SG100 initial target would be clamped by runtime limits"
        )

    print("\n[INITIAL POSE 2/2] Moving SG100 with low-gain MODE7...")
    try:
        before = driver.read_feedback()
        before_sequence = 0 if before is None else int(before.sequence)
        event = driver.send_positions_rad(np.deg2rad(config.angles_deg))
        if event is None:
            raise TeleopInitialPoseError(
                "SG100 SetHandControl rejected the initial target"
            )

        deadline = time.monotonic() + config.command_duration_s
        saw_fresh_mode7 = False
        while time.monotonic() < deadline:
            if driver.fault_active:
                raise TeleopInitialPoseError(
                    "SG100 reported an error frame during initialization"
                )
            feedback = driver.read_feedback()
            if (
                feedback is not None
                and feedback.valid
                and int(feedback.sequence) > before_sequence
                and int(feedback.received_monotonic_ns)
                >= event.sent_monotonic_ns
                and feedback.joint_modes
                == (MODE_JOINT_POSITION,) * HAND_DOF
            ):
                saw_fresh_mode7 = True
            time.sleep(0.01)
        if not saw_fresh_mode7:
            raise TeleopInitialPoseError(
                "SG100 target was accepted but no fresh all-MODE7 "
                "feedback arrived"
            )
        effective_deg = np.rad2deg(
            application.effective_canonical_rad
        )
    except BaseException:
        try:
            driver.hold_passive()
        except Exception:
            pass
        raise

    if not driver.hold_passive():
        raise TeleopInitialPoseError(
            "SG100 reached the end of initialization but MODE_NULL "
            "passive hold was rejected"
        )
    print(
        "[SG100] target accepted and fresh MODE7 feedback observed; "
        "MODE_NULL restored before WAIT-C."
    )
    _print_initial_pose_vector(
        "[SG100] effective commanded target:",
        INITIAL_HAND_JOINT_NAMES,
        effective_deg,
    )
    print(
        "[SG100] canonical reached-pose verification remains unavailable "
        "without validated F_fb; visually confirm the hand pose."
    )
    return effective_deg


def _wrap_to_pi(angle: float) -> float:
    """Wrap one angle to [-pi, pi]."""
    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def _print_sg100_command_debug(
    q_hand_cmd: np.ndarray,
    hand_result,
    restricted_backend,
    *,
    _state: dict = {"last_ns": 0},
) -> None:
    """~5 Hz telemetry of the authoritative hand command (opt-in).

    Prints the pre-backend canonical command, the allowlisted real
    joints, the post-F_cmd SDK target, the rejection reason (if any),
    and the C/enable gate state.  No hardware interaction.
    """

    enabled = getattr(_print_sg100_command_debug, "enabled", False)
    if not enabled:
        return
    now_ns = time.monotonic_ns()
    if now_ns - _state["last_ns"] < 200_000_000:
        return
    _state["last_ns"] = now_ns

    hardware_map = restricted_backend.hardware_map
    sdk_deg = np.degrees(
        hardware_map.gain * np.asarray(q_hand_cmd, dtype=np.float64)
        + hardware_map.offset_rad
    )
    allowed = tuple(
        hardware_map.software_joint_order[i]
        for i in restricted_backend.active_joint_indices
    )
    print()
    print("=" * 52)
    print("SG100 COMMAND DEBUG (canonical -> backend)")
    print("=" * 52)
    print(f"timestamp_ns     : {now_ns}")
    print(f"canonical rad    : {np.round(q_hand_cmd, 4).tolist()}")
    print(f"allowed joints   : {allowed}")
    print(f"C anchored       : {restricted_backend.anchored}")
    print(f"enabled          : {restricted_backend.enabled}")
    print(f"estop latched    : {restricted_backend.estopped}")
    print(f"post-F_cmd sdk deg: {np.round(sdk_deg, 2).tolist()}")
    print(
        f"command result   : accepted={hand_result.accepted} "
        f"reason={hand_result.reason}"
        + (" (slew-limited)" if hand_result.slew_limited else "")
    )


def _driver_diagnostics_snapshot(driver) -> dict | None:
    if driver is None:
        return None
    try:
        return driver.diagnostics
    except Exception as exc:  # logging must not alter the control path
        return {"snapshot_error": repr(exc)}


def _record_sg100_safe_stop(
    error_recorder,
    *,
    hand_result,
    q_hand_cmd: np.ndarray,
    restricted_backend,
    hand_driver,
    latest_control_audit,
    physics_dt: float,
) -> None:
    """Persist one rejected real-hand command with its causal context."""

    if error_recorder is None:
        return
    now_ns = time.monotonic_ns()
    feedback = getattr(hand_driver, "latest_feedback", None)
    tracking_received_ns = None
    if latest_control_audit is not None:
        tracking_received_ns = latest_control_audit.get(
            "tracking_received_monotonic_ns"
        )
    driver_diagnostics = _driver_diagnostics_snapshot(hand_driver)
    try:
        serial_open = hand_driver.serial_open
    except Exception as exc:
        serial_open = f"snapshot_error:{exc!r}"
    error_recorder.record(
        severity="ERROR",
        component="sg100_real_hand",
        event="SAFE_STOP",
        message=str(hand_result.reason),
        details={
            "command_result": hand_result,
            "canonical_qcmd_rad": q_hand_cmd,
            "physics_dt_s": physics_dt,
            "backend_state_after_rejection": {
                "anchored": restricted_backend.anchored,
                "enabled": restricted_backend.enabled,
                "estopped": restricted_backend.estopped,
                "active_joint_indices": restricted_backend.active_joint_indices,
                "active_joint_names": [
                    restricted_backend.hardware_map.software_joint_order[index]
                    for index in restricted_backend.active_joint_indices
                ],
                "last_canonical_rad": restricted_backend._last_canonical,
                "last_sdk_target_deg": restricted_backend._last_sdk_target,
                "max_cycle_delta_rad": restricted_backend.max_cycle_delta_rad,
                "max_velocity_rad_s": restricted_backend.max_velocity_rad_s,
                "tracking_spike_rad": restricted_backend.tracking_spike_rad,
                "stale_tracking_s": restricted_backend.stale_tracking_ns * 1e-9,
                "feedback_stall_s": restricted_backend.feedback_stall_ns * 1e-9,
            },
            "transport": {
                "port": getattr(hand_driver, "port", None),
                "serial_open": serial_open,
                "diagnostics": driver_diagnostics,
            },
            "latest_feedback": feedback,
            "feedback_age_ms": (
                None
                if feedback is None
                else (now_ns - feedback.received_monotonic_ns) * 1e-6
            ),
            "latest_control_audit": latest_control_audit,
            "tracking_age_ms": (
                None
                if tracking_received_ns is None
                else (now_ns - int(tracking_received_ns)) * 1e-6
            ),
        },
        include_stack=True,
    )


def _emit_sg100_safety_events(restricted_backend, error_recorder) -> None:
    """Print and persist rate-limited command-limit events."""

    if restricted_backend is None:
        return
    for event in restricted_backend.drain_safety_events():
        details = dict(event.details or {})
        details.update({
            "joint": event.joint_name,
            "joint_index": event.joint_index,
            "domain": event.domain,
            "requested": event.requested,
            "effective": event.effective,
            "unit": event.unit,
            "timestamp_ns": event.timestamp_ns,
        })
        if event.event == "SDK_LIMIT_MAPPING_MISMATCH":
            print(
                "ERROR SDK_LIMIT_MAPPING_MISMATCH",
                f"joint={event.joint_name}",
                f"requested={event.requested} deg",
                f"effective={event.effective} deg",
            )
            severity = "ERROR"
        elif event.event == "JOINT_LIMIT_CLAMP":
            print(
                "JOINT_LIMIT_CLAMP",
                f"joint={event.joint_name}",
                f"requested={event.requested:.6f} deg",
                f"effective={event.effective:.6f} deg",
                f"domain={event.domain}",
            )
            severity = "WARNING"
        else:
            print(
                "JOINT_LIMIT_CLAMP_CLEARED",
                f"joint={event.joint_name}",
            )
            severity = "INFO"
        if error_recorder is not None:
            error_recorder.record(
                severity=severity,
                component="sg100_real_hand",
                event=event.event,
                message=event.event,
                details=details,
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

def disarm_real_command_authority(
    real_arm_backend=None,
    restricted_hand_backend=None,
) -> bool:
    """显式撤销 real command authority；disabled backend 不产生额外动作。"""
    changed = False
    if restricted_hand_backend is not None and restricted_hand_backend.enabled:
        restricted_hand_backend.set_enabled(False)
        changed = True
    if real_arm_backend is not None and real_arm_backend.enabled:
        real_arm_backend.set_enabled(False)
        real_arm_backend.stop_smooth()
        changed = True
    return changed


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
        "--initial-pose-config",
        default=None,
        metavar="YAML",
        help=(
            "Optional operator-defined 6+11 joint startup pose in degrees. "
            "Requires both real backends. The pose is executed before "
            "normal WAIT-C teleoperation."
        ),
    )

    parser.add_argument(
        "--initial-pose-dry-run",
        action="store_true",
        help=(
            "Validate and print --initial-pose-config, then exit without "
            "opening either robot connection"
        ),
    )

    parser.add_argument(
        "--initial-pose-auto-confirm",
        action="store_true",
        help=(
            "Skip typing INITIALIZE before real startup motion. Requires "
            "--initial-pose-config; intended only for supervised automation."
        ),
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

    parser.add_argument(
        "--collect-arm-hand-data",
        action="store_true",
        help=(
            "Record causal rm65_sg100_joint17_v1 transitions. Requires "
            "--arm-backend real because arm state must be measured feedback."
        ),
    )

    parser.add_argument(
        "--telemetry-publish",
        default="",
        metavar="HOST:PORT",
        help=(
            "Non-blocking read-only robot telemetry UDP destination. "
            "Empty by default, so the existing teleop path is unchanged."
        ),
    )

    parser.add_argument(
        "--telemetry-publish-hz",
        type=float,
        default=100.0,
        help=(
            "Requested maximum external telemetry snapshot rate / throttle "
            "target (default: 100 Hz; actual rate is caller-loop limited)"
        ),
    )

    parser.add_argument(
        "--data-stale-pre-ms",
        type=float,
        default=100.0,
        help="Maximum age of pre-action RM65 feedback (default: 100 ms)",
    )

    parser.add_argument(
        "--data-stale-post-ms",
        type=float,
        default=250.0,
        help="Maximum wait for post-action RM65 feedback (default: 250 ms)",
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

    # ========================================================
    # Restricted SG100 hand backend. The sim default stays unchanged;
    # real accepts the same final post-filter canonical qcmd but only
    # activates the real-hardware-validated joint allowlist. F_fb/data
    # remain blocked independently.
    # ========================================================

    parser.add_argument(
        "--hand-backend",
        choices=("sim", "real", "mock"),
        default="sim",
        help=(
            "Hand execution backend: 'sim' keeps the software-only "
            "command proxy (current behaviour); 'real' sends only the "
            "allowlisted final qcmd through the restricted SG100 envelope "
            "and keeps F_fb/data quarantined; 'mock' exercises the data path "
            "against MockSG100Driver (no hardware)"
        ),
    )

    parser.add_argument(
        "--hand-port",
        default="/dev/ttyUSB0",
        help="UART port of the SG100 palm board for --hand-backend real",
    )

    parser.add_argument(
        "--hand-baudrate",
        type=int,
        default=1_000_000,
        help="SG100 UART baudrate (vendor default 1000000)",
    )

    parser.add_argument(
        "--hand-sdk-dir",
        default=None,
        help="SG100 SDK root (default: $SG100_SDK_DIR, else "
        "<repo>/../SG100_SDK_Publish, else ~/SG100_SDK_Publish)",
    )

    parser.add_argument(
        "--hand-feedback-poll-hz",
        type=float,
        default=100.0,
        help="SG100 measured-feedback polling rate",
    )

    parser.add_argument(
        "--sg100-real-joints",
        nargs="+",
        default=("thumb_j1", "thumb_j2", "thumb_j3", "index_j1", "index_j2", "index_j3", "middle_j1", "middle_j2", "little_j1", "little_j2", "little_j3",),
        help=(
            "Restricted real-hand active allowlist. 当前真实硬件只解锁 "
            "index_j3(J6)；J7 与 full-11D 均保持阻断。"
        ),
    )

    parser.add_argument(
        "--sg100-command-debug",
        action="store_true",
        help=(
            "低频(~5Hz)打印权威 canonical hand command 的 telemetry："
            "时间戳、11D canonical rad、allowlist、post-F_cmd SDK deg、"
            "拒绝原因与 C/enable 状态"
        ),
    )

    parser.add_argument(
        "--thumb-lock-hardware-cap-deg",
        type=float,
        default=None,
        help=(
            "HARDWARE VALIDATION CAP ONLY: clamps the configured "
            "power-grasp thumb_j1 lock bias to at most this many "
            "degrees for this run (staged first-hardware testing: "
            "10 -> 20 -> 30 -> 40). Default None keeps the production "
            "config unchanged; the cap never alters the blend "
            "semantics, only the bias amplitude for this session."
        ),
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
        "--arm-not-moving-stop",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Stop the real arm on sustained low measured velocity relative "
            "to commanded velocity (arm_not_moving). Default: off, warning "
            "only. Target/feedback timeouts, controller faults and gross "
            "position-error stops remain active."
        ),
    )

    parser.add_argument(
        "--arm-max-follow-error-deg",
        type=float,
        default=20.0,
        help=(
            "Gross cmd-vs-actual position mismatch bound in deg for "
            "the real backend; remains active even when "
            "--arm-not-moving-stop is off (default: 20)"
        ),
    )

    parser.add_argument(
        "--arm-verbose",
        action="store_true",
        help="Print real backend events and timing statistics",
    )

    parser.add_argument(
        "--arm-joint-compare",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Read-only MuJoCo/feedback/sent joint comparison JSONL log; "
            "requires --arm-backend real (default: off)"
        ),
    )

    parser.add_argument(
        "--arm-joint-compare-hz",
        type=float,
        default=2.0,
        help="Joint comparison sampling rate in (0, 10] Hz (default: 2)",
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

    parser.add_argument(
        "--thumb-retarget-debug",
        action="store_true",
        help=(
            "Print a low-rate (~5 Hz) thumb retargeting diagnostic: "
            "grasp/lock weights, human thumb features, raw/bias/final "
            "SG100 thumb q, and live thumb tip geometry"
        ),
    )

    parser.add_argument(
        "--hammer-impact-diagnostic",
        action="store_true",
        help=(
            "Record bounded Hammer-head/Peg impact traces and print one "
            "summary per impact (default: disabled)"
        ),
    )

    parser.add_argument(
        "--show-hammer-collision-debug",
        action="store_true",
        help=(
            "Show Hammer/Peg collision proxies and MuJoCo contact points "
            "without changing production physics (default: disabled)"
        ),
    )

    return parser


def main():
    # ========================================================
    # Command-line arguments
    # ========================================================

    parser = build_arg_parser()
    args = parser.parse_args()

    # Assigned after the MuJoCo recording session directory exists.  The
    # real-arm callback closes over this reference, so background errors are
    # persisted for the rest of the run without changing its control path.
    real_error_recorder = None
    previous_excepthook = None

    # Opt-in low-rate command telemetry (see _print_sg100_command_debug).
    _print_sg100_command_debug.enabled = bool(args.sg100_command_debug)

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

    if args.collect_arm_hand_data and args.arm_backend != "real":
        raise ValueError(
            "--collect-arm-hand-data requires --arm-backend real; "
            "simulated qpos is not RM65 measured feedback"
        )

    if args.collect_arm_hand_data and args.hand_backend == "real":
        raise ValueError(
            "SG100 F_fb 尚未验证：real-hand dataset 必须保持 QUARANTINED"
        )

    if args.arm_joint_compare and args.arm_backend != "real":
        raise ValueError("--arm-joint-compare requires --arm-backend real")

    if (
        not math.isfinite(args.arm_joint_compare_hz)
        or not 0.0 < args.arm_joint_compare_hz <= 10.0
        or not math.isfinite(1e9 / args.arm_joint_compare_hz)
    ):
        raise ValueError("--arm-joint-compare-hz must be finite and in (0, 10]")

    if args.hammer_impact_diagnostic and args.arm_backend != "sim":
        raise ValueError(
            "--hammer-impact-diagnostic requires --arm-backend sim; the "
            "real backend uses a kinematic MuJoCo twin without mj_step"
        )

    if args.data_stale_pre_ms <= 0.0 or args.data_stale_post_ms <= 0.0:
        raise ValueError("data stale-state thresholds must be > 0")

    if args.telemetry_publish_hz <= 0.0:
        raise ValueError("--telemetry-publish-hz must be > 0")

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

    if args.initial_pose_dry_run and args.initial_pose_config is None:
        raise ValueError(
            "--initial-pose-dry-run requires --initial-pose-config"
        )

    if args.initial_pose_auto_confirm and args.initial_pose_config is None:
        raise ValueError(
            "--initial-pose-auto-confirm requires --initial-pose-config"
        )

    if (
        args.initial_pose_config is not None
        and not args.initial_pose_dry_run
        and (
            args.arm_backend != "real"
            or args.hand_backend != "real"
        )
    ):
        raise ValueError(
            "--initial-pose-config execution requires "
            "--arm-backend real and --hand-backend real"
        )

    initial_pose_plan = None
    if args.initial_pose_config is not None:
        initial_pose_config = load_teleop_initial_pose_config(
            args.initial_pose_config
        )
        initial_pose_plan = build_teleop_initial_pose_plan(
            initial_pose_config,
            sdk_dir=args.hand_sdk_dir,
        )
        print_teleop_initial_pose_plan(
            initial_pose_plan,
            arm_ip=args.arm_ip or "<not configured>",
            arm_port=args.arm_port,
            hand_port=args.hand_port,
            hand_baudrate=args.hand_baudrate,
        )
        if args.initial_pose_dry_run:
            print(
                "\n[INITIAL POSE DRY RUN] Configuration is valid. "
                "No device was opened and no motion was sent."
            )
            return

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

    # Prepare the per-run directory before connecting either real backend so
    # startup/connection exceptions also have a durable destination.  Actual
    # MuJoCo sampling starts only after the robot state is initialized.
    state_recorder = MujocoStateRecorder(
        model,
        scene_path=model_path,
        recording_dir=args.recording_dir,
        record_fps=args.video_fps,
        camera_name="workstation_front",
        hand_qpos_indices=robot_layout.hand_qpos_indices,
    )
    recording_session_dir = state_recorder.prepare_session()

    if args.arm_backend == "real" or args.hand_backend == "real":
        real_error_recorder = RealHardwareErrorRecorder(
            recording_session_dir,
            context={
                "pid": os.getpid(),
                "cwd": Path.cwd(),
                "python_executable": sys.executable,
                "argv": sys.argv,
                "arguments": vars(args),
                "model_path": model_path,
                "recording_session_dir": recording_session_dir,
                "real_components": {
                    "arm": args.arm_backend == "real",
                    "hand": args.hand_backend == "real",
                },
            },
        )
        previous_excepthook = sys.excepthook

        def real_hardware_excepthook(exception_type, exception, tb):
            real_error_recorder.record_exception(
                component="runtime",
                event="UNHANDLED_EXCEPTION",
                exception=exception,
                details={"exception_type_from_hook": exception_type.__name__},
                traceback_object=tb,
            )
            previous_excepthook(exception_type, exception, tb)

        sys.excepthook = real_hardware_excepthook
        print(
            "real error log    :",
            real_error_recorder.path,
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
    # filters, twin state) to measured arm feedback.  When both physical
    # backends are selected, the SG100 and RM65 startup targets are applied
    # before that state is captured.
    # ========================================================

    real_arm_backend = None
    hand_driver = None
    restricted_hand_backend = None
    hand_hardware_map = None

    if args.arm_backend == "real":
        from sg100_teleop.robot.rm65_real import RM65RealBackend

        if initial_pose_plan is not None:
            reached_deg, error_deg = initialize_rm65_initial_pose(
                args.arm_ip,
                args.arm_port,
                initial_pose_plan.config.arm,
                auto_confirm=args.initial_pose_auto_confirm,
            )
            if real_error_recorder is not None:
                real_error_recorder.record(
                    severity="INFO",
                    component="rm65_initial_pose",
                    event="INITIAL_POSE_REACHED",
                    message=(
                        "RM65 planned startup pose completed before "
                        "CANFD teleoperation backend startup"
                    ),
                    details={
                        "target_deg": (
                            initial_pose_plan.config.arm.angles_deg
                        ),
                        "reached_deg": reached_deg,
                        "error_deg": error_deg,
                        "speed_percent": (
                            initial_pose_plan.config.arm.speed_percent
                        ),
                    },
                )

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
                "arm_not_moving_warning",
            ):
                print(f"[REAL-ARM] {name}: {detail}")
            if real_error_recorder is not None:
                severity = (
                    "ERROR"
                    if "error" in name or "failure" in name
                    else "WARNING"
                    if name in ("disarm", "arm_not_moving_warning")
                    else "INFO"
                )
                real_error_recorder.record(
                    severity=severity,
                    component="rm65_real_arm",
                    event=name.upper(),
                    message=detail,
                    details={"sender_thread_event": True},
                    include_stack=severity == "ERROR",
                )

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
            arm_not_moving_stop=args.arm_not_moving_stop,
            on_event=real_arm_event,
        )
        real_arm_backend.connect()

    # Connect the real SG100 before starting the RM65 sender threads.  This
    # gives the dual-hardware startup sequence one unambiguous point at which
    # both SDK connections have succeeded and no teleoperation sender is
    # active yet.
    try:
        if args.hand_backend == "real":
            from sg100_teleop.robot.sg100_sdk_driver import SG100SDKDriver
            from sg100_teleop.robot.sg100_restricted_avp_backend import (
                SG100RestrictedAVPBackend,
            )

            hand_hardware_map = SG100HardwareMap()
            hand_driver = SG100SDKDriver(
                args.hand_port,
                baudrate=args.hand_baudrate,
                hardware_map=hand_hardware_map,
                sdk_dir=(
                    Path(args.hand_sdk_dir)
                    if args.hand_sdk_dir is not None
                    else None
                ),
            )
            restricted_hand_backend = SG100RestrictedAVPBackend(
                hand_driver,
                joint_allowlist=args.sg100_real_joints,
                stale_tracking_s=0.50,
                # AVP targets are already smoothed by hand_command_filter.
                # Let this independent limiter ramp from the C anchor.
                max_velocity_rad_s=4.0,
                max_cycle_delta_rad=0.20,
                tracking_spike_rad=1.00,
            )
            hand_driver.connect()
            hand_driver.start()
            # Seed MODE_NULL before switching, so connection and realtime
            # entry themselves cannot move the hand.
            hand_driver.enter_realtime()
            print(
                "hand backend      :",
                f"{type(hand_driver).__name__} @ "
                f"{getattr(hand_driver, 'port', 'mock')}",
                "(realtime, MODE_NULL hold)",
            )
            print(
                "hand allowlist    :",
                tuple(args.sg100_real_joints),
                "(C anchor + A enable required after startup)",
            )
            print(
                "hand cycle-delta  :",
                (
                    "disabled"
                    if restricted_hand_backend.max_cycle_delta_rad is None
                    else (
                        f"{restricted_hand_backend.max_cycle_delta_rad} rad"
                    )
                ),
                "(tracking-spike + velocity limits retained)",
            )
            if real_error_recorder is not None:
                real_error_recorder.record(
                    severity="INFO",
                    component="sg100_real_hand",
                    event="BACKEND_READY",
                    message=(
                        "SG100 real backend connected and entered realtime"
                    ),
                    details={
                        "port": getattr(hand_driver, "port", None),
                        "baudrate": args.hand_baudrate,
                        "allowlist": tuple(args.sg100_real_joints),
                        "backend_limits": {
                            "max_cycle_delta_rad": (
                                restricted_hand_backend.max_cycle_delta_rad
                            ),
                            "max_velocity_rad_s": (
                                restricted_hand_backend.max_velocity_rad_s
                            ),
                            "tracking_spike_rad": (
                                restricted_hand_backend.tracking_spike_rad
                            ),
                            "stale_tracking_s": (
                                restricted_hand_backend.stale_tracking_ns
                                * 1e-9
                            ),
                            "feedback_stall_s": (
                                restricted_hand_backend.feedback_stall_ns
                                * 1e-9
                            ),
                        },
                        "driver_diagnostics": hand_driver.diagnostics,
                    },
                )

        if real_arm_backend is not None and args.hand_backend == "real":
            assert hand_driver is not None
            print(
                "[STARTUP] both real SDK connections are ready; "
                "commanding SG100 thumb_j2 to -31 deg and all other "
                "hand joints to 0 deg"
            )
            hand_startup_command = hand_driver.send_positions_rad(
                REAL_STARTUP_HAND_Q_RAD.copy()
            )
            if hand_startup_command is None:
                raise RuntimeError(
                    "SG100 SDK rejected the startup hand target "
                    "(thumb_j2=-31 deg, other joints=0 deg)"
                )
            print(
                "[STARTUP] SG100 startup target accepted; "
                "commanding RM65 J1-J6 to 15 deg at "
                f"{REAL_STARTUP_ARM_SPEED_PERCENT}%"
            )
            startup_arm_measured_q = real_arm_backend.movej_blocking(
                REAL_STARTUP_ARM_Q_RAD.copy(),
                speed_percent=REAL_STARTUP_ARM_SPEED_PERCENT,
            )
            print(
                "[STARTUP] RM65 planned move complete; actual q(deg) =",
                np.round(np.degrees(startup_arm_measured_q), 3).tolist(),
            )
            if real_error_recorder is not None:
                real_error_recorder.record(
                    severity="INFO",
                    component="real_hardware_startup",
                    event="INITIAL_POSE_COMPLETE",
                    message=(
                        "SG100 startup target (thumb_j2=-31 degrees, all "
                        "other joints=0 degrees) accepted and RM65 J1-J6 "
                        "planned move to 15 degrees completed"
                    ),
                    details={
                        "hand_target_rad": REAL_STARTUP_HAND_Q_RAD,
                        "hand_command": hand_startup_command,
                        "arm_target_rad": REAL_STARTUP_ARM_Q_RAD,
                        "arm_measured_rad": startup_arm_measured_q,
                        "arm_speed_percent": (
                            REAL_STARTUP_ARM_SPEED_PERCENT
                        ),
                    },
                )
    except BaseException:
        # Startup happens before the main runtime try/finally.  If either SDK
        # setup or initial motion fails, explicitly leave both devices in a
        # stopped/disconnected state before propagating the error.
        try:
            if hand_driver is not None:
                try:
                    hand_driver.stop()
                finally:
                    hand_driver.disconnect()
        finally:
            if real_arm_backend is not None:
                try:
                    real_arm_backend.stop_smooth()
                finally:
                    real_arm_backend.disconnect()
        raise

    if real_arm_backend is not None:
        real_arm_backend.start()

        print(
            "[REAL-ARM] backend started "
            f"(follow={args.arm_follow}, {args.arm_send_hz} Hz, "
            f"PROFILE_{args.arm_safety_profile}, "
            f"tool={args.arm_tool_profile})"
        )
        print(
            "[REAL-ARM] arm_not_moving stop: "
            + ("ON" if args.arm_not_moving_stop else "OFF (warning only)")
            + "; other stop protections remain active"
        )
        print(
            "[REAL-ARM] keys: C = calibrate anchor | "
            "A = ARM / DISARM streaming | D = DISARM only | "
            "Ctrl-C = stop+disconnect"
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

    # Optional FIRST-HARDWARE validation cap on the power-grasp lock
    # bias (staged 10 -> 20 -> 30 -> 40 deg).  Implemented as a session
    # bias override so the blend semantics stay untouched; without the
    # flag the production config (+40 deg) is used unchanged.
    if args.thumb_lock_hardware_cap_deg is not None:
        cap = float(args.thumb_lock_hardware_cap_deg)
        if not (0.0 < cap <= 40.0 + 1e-9):
            raise SystemExit(
                "--thumb-lock-hardware-cap-deg must lie in (0, 40]"
            )
        import tempfile as _tempfile

        import yaml as _yaml

        _semantics_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "sg100_joint_semantics.yaml"
        )
        with _semantics_path.open("r") as f:
            _semantics = _yaml.safe_load(f)
        _biases = _semantics["power_grasp_thumb_lock"]["biases_deg"]
        _biases["thumb_j1"] = min(float(_biases["thumb_j1"]), cap)
        with _tempfile.NamedTemporaryFile(
            "w", suffix=".yaml", delete=False
        ) as f:
            _yaml.safe_dump(_semantics, f)
            _semantics_temp_path = f.name
        retargeter = SG100Retargeter(_semantics_temp_path)
        print(
            "thumb lock hardware cap :",
            f"thumb_j1 bias limited to {_biases['thumb_j1']:.1f} deg "
            "(validation session)",
        )

    # RAW (pre-lock) thumb commands for the debug overlay only; the
    # production retarget path is untouched.
    retargeter_no_lock = (
        build_lock_disabled_retargeter()
        if args.thumb_retarget_debug
        else None
    )

    last_thumb_debug = -float("inf")


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

    expected_neutral_q = np.asarray(
        [
            retargeter.joint_zero[name]
            for name in retargeter.joint_order
        ],
        dtype=np.float64,
    )

    if not np.allclose(
        q_hand_goal,
        expected_neutral_q,
        atol=1e-12,
    ):
        raise RuntimeError(
            "Canonical neutral hand does not "
            "retarget to the configured SG100 semantic zero pose.\n"
            f"expected={expected_neutral_q}\n"
            f"q_hand_goal={q_hand_goal}"
        )

    # Keep the retargeter-neutral invariant above as a regression check, then
    # seed the visual twin and command filter from the requested real startup
    # pose. The physical SG100 command is still deferred until its driver has
    # connected and entered realtime below.
    if initial_pose_plan is not None:
        q_hand_goal = np.deg2rad(
            initial_pose_plan.config.hand.angles_deg
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

    grasp_mode_controller = GraspModeController(
        joint_order=tuple(retargeter.joint_order),
    )
    script_hand_controller = ScriptHandController(
        joint_order=tuple(retargeter.joint_order),
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
    print(
        "grasp mode         :",
        grasp_mode_controller.mode.value,
        "(1=FREE 2=HAND 3=CLAW 4=SCRIPT J/K/L)",
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

    # Latest complete AVP -> TCP -> IK audit record.  It is attached to
    # high-rate command decisions but never used as canonical action data.
    latest_control_audit = None
    latest_ik_accepted = False

    telemetry_period_ns = int(1e9 / args.telemetry_publish_hz)
    last_telemetry_publish_ns = 0

    def publish_external_telemetry(publish_due_ns: int) -> None:
        """Publish cached backend truth only; perform no SDK or robot I/O."""
        if telemetry_publisher is None:
            return
        snapshot = build_teleop_snapshot(
            now_ns=publish_due_ns,
            tracking_received_ns=(
                tracking_gate.last_valid_tracking_monotonic_ns
            ),
            tracking_timeout_ns=int(args.tracking_timeout * 1e9),
            avp_anchored=wrist_calibrated,
            arm_backend=real_arm_backend,
            arm_q_goal_rad=q_arm_goal,
            arm_sender_hz=args.arm_send_hz,
            hand_driver=hand_driver,
            fallback_hand_feedback=hand_feedback,
            restricted_hand_backend=restricted_hand_backend,
            hand_qcmd_retargeted_rad=q_hand_cmd,
            latest_hand_command=latest_hand_telemetry,
            telemetry_publish_hz_requested=args.telemetry_publish_hz,
        )
        telemetry_publisher.publish(snapshot)


    # ========================================================
    # Lightweight MuJoCo visual-state recording
    #
    # This performs no rendering or video encoding. Sampling is
    # driven by data.time and finalized once when the session ends.
    # ========================================================

    assert state_recorder.start(data) == recording_session_dir
    if real_error_recorder is not None and real_arm_backend is not None:
        real_error_recorder.record(
            severity="INFO",
            component="rm65_real_arm",
            event="BACKEND_READY",
            message="RM65 real backend connected and started",
            details={
                "joint_state": real_arm_backend.joint_state(),
                "sender_stats": real_arm_backend.sender_stats,
                "enabled": real_arm_backend.enabled,
            },
        )

    hammer_impact_diagnostic = None
    if args.hammer_impact_diagnostic:
        hammer_impact_diagnostic = HammerImpactDiagnostic(
            model,
            recording_session_dir / "hammer_diagnostic",
        )
        print(
            "hammer diagnostic :",
            recording_session_dir / "hammer_diagnostic",
        )

    # ========================================================
    # SG100 hand backend. 'real' is wrapped by the restricted command
    # envelope; 'mock' retains the software data-path regression.
    # ========================================================

    if args.hand_backend == "mock":
        hand_hardware_map = SG100HardwareMap()
        hand_driver = MockSG100Driver(hand_hardware_map)

    hand_feedback_poll_period = 1.0 / args.hand_feedback_poll_hz
    last_hand_feedback_poll = -float("inf")
    hand_feedback = None
    latest_hand_telemetry = None
    hand_telemetry_sequence = 0

    if args.hand_backend == "mock":
        assert hand_driver is not None
        hand_driver.connect()
        hand_driver.start()
        hand_driver.enter_realtime()
        print(
            "hand backend      :",
            f"{type(hand_driver).__name__} @ "
            f"{getattr(hand_driver, 'port', 'mock')}",
            "(realtime, MODE_NULL hold)",
        )
        if restricted_hand_backend is not None:
            print(
                "hand allowlist    :",
                tuple(args.sg100_real_joints),
                "(C anchor + A enable required)",
            )
            print(
                "hand cycle-delta  :",
                (
                    "disabled"
                    if restricted_hand_backend.max_cycle_delta_rad is None
                    else (
                        f"{restricted_hand_backend.max_cycle_delta_rad} rad"
                    )
                ),
                "(tracking-spike + velocity limits retained)",
            )
            if real_error_recorder is not None:
                real_error_recorder.record(
                    severity="INFO",
                    component="sg100_real_hand",
                    event="BACKEND_READY",
                    message="SG100 real backend connected and entered realtime",
                    details={
                        "port": getattr(hand_driver, "port", None),
                        "baudrate": args.hand_baudrate,
                        "allowlist": tuple(args.sg100_real_joints),
                        "backend_limits": {
                            "max_cycle_delta_rad": (
                                restricted_hand_backend.max_cycle_delta_rad
                            ),
                            "max_velocity_rad_s": (
                                restricted_hand_backend.max_velocity_rad_s
                            ),
                            "tracking_spike_rad": (
                                restricted_hand_backend.tracking_spike_rad
                            ),
                            "stale_tracking_s": (
                                restricted_hand_backend.stale_tracking_ns * 1e-9
                            ),
                            "feedback_stall_s": (
                                restricted_hand_backend.feedback_stall_ns * 1e-9
                            ),
                        },
                        "driver_diagnostics": hand_driver.diagnostics,
                    },
                )

    arm_hand_collector = None
    if args.collect_arm_hand_data:
        arm_lower_rad, arm_upper_rad = real_arm_backend.joint_limits_rad()
        hand_joint_ids = robot_layout.joint_ids[ARM_DOF:]
        hand_lower_rad = model.jnt_range[hand_joint_ids, 0].copy()
        hand_upper_rad = model.jnt_range[hand_joint_ids, 1].copy()
        hand_measured = hand_driver is not None
        arm_hand_collector = ArmHandDataCollector(
            recording_session_dir / "arm_hand_data",
            # Proxy mode: the calibrated runtime qcmd does not exist
            # yet.  Never invent a zero/home baseline; calibration
            # establishes the real software command proxy before the
            # first eligible action.  (Measured mode builds the replay
            # seed from the first measured SG100 feedback instead.)
            initial_hand_qcmd=None,
            arm_lower_rad=arm_lower_rad,
            arm_upper_rad=arm_upper_rad,
            hand_lower_rad=hand_lower_rad,
            hand_upper_rad=hand_upper_rad,
            hand_measured_feedback=hand_measured,
            config=CollectorConfig(
                stale_pre_state_ns=int(args.data_stale_pre_ms * 1e6),
                stale_post_state_ns=int(args.data_stale_post_ms * 1e6),
            ),
        )
        # RECORDER_ARMED pre-roll begins before the first motion command.
        arm_hand_collector.observe_arm_feedback(real_arm_backend.joint_state())
        if hand_measured:
            arm_hand_collector.observe_hand_feedback(
                hand_driver.read_feedback()
            )

    print(
        "recording session :",
        recording_session_dir,
    )
    print(
        "recording rate    :",
        args.video_fps,
        "FPS (simulation time)",
    )
    if arm_hand_collector is not None:
        print(
            "joint17 data     :",
            recording_session_dir / "arm_hand_data",
        )
        print(
            "joint17 schema   : rm65_sg100_joint17_v1 "
            "(real feedback / actual commands, rad)",
        )
        print(
            "hand baseline   :",
            "SERIALIZED COMMAND PROXY"
            if arm_hand_collector.hand_proxy_baseline_valid
            else "WAITING FOR VALID FINAL TELEOP QCMD",
        )

    telemetry_publisher = None
    if args.telemetry_publish:
        from sg100_teleop.telemetry.teleop_telemetry import (
            TeleopTelemetryPublisher,
            build_teleop_snapshot,
        )

        telemetry_publisher = TeleopTelemetryPublisher(
            args.telemetry_publish
        )
        print(
            "external telemetry:",
            f"UDP {args.telemetry_publish} @ "
            f"requested max {args.telemetry_publish_hz:g} Hz "
            "(caller-loop limited, non-blocking, read-only)",
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
        nonlocal q_hand_goal
        if keycode in (ord("C"), ord("c")):
            request_neutral_calibration()

        elif keycode in (ord("A"), ord("a")) and (
            real_arm_backend is not None or restricted_hand_backend is not None
        ):
            # v3 §A4.2: calibration (C) and arm-enable (A) are separate
            # actions.  Disarm order follows §A3.5: quiesce the sender
            # first, then command the decelerated stop.
            currently_enabled = bool(
                (real_arm_backend is not None and real_arm_backend.enabled)
                or (
                    restricted_hand_backend is not None
                    and restricted_hand_backend.enabled
                )
            )
            if currently_enabled:
                disarm_real_command_authority(
                    real_arm_backend,
                    restricted_hand_backend,
                )
                print(
                    "[REAL] DISARMED (A); active senders disabled"
                )
                if real_error_recorder is not None:
                    real_error_recorder.record(
                        severity="INFO",
                        component="operator_gate",
                        event="DISARMED_A",
                        message="Operator toggled real command authority off",
                        details={
                            "teleop_state": teleop_arming.state.value,
                            "hand_anchored": (
                                restricted_hand_backend.anchored
                                if restricted_hand_backend is not None
                                else None
                            ),
                        },
                    )
            else:
                if restricted_hand_backend is not None:
                    restricted_hand_backend.set_enabled(True)
                if real_arm_backend is not None:
                    real_arm_backend.set_enabled(True)
                print(
                    "[REAL] ARMED (A); restricted streaming enabled"
                )
                if real_error_recorder is not None:
                    real_error_recorder.record(
                        severity="INFO",
                        component="operator_gate",
                        event="ARMED_A",
                        message="Operator enabled real command authority",
                        details={
                            "teleop_state": teleop_arming.state.value,
                            "hand_anchored": (
                                restricted_hand_backend.anchored
                                if restricted_hand_backend is not None
                                else None
                            ),
                            "hand_enabled": (
                                restricted_hand_backend.enabled
                                if restricted_hand_backend is not None
                                else None
                            ),
                            "arm_enabled": (
                                real_arm_backend.enabled
                                if real_arm_backend is not None
                                else None
                            ),
                        },
                    )

        elif keycode in (ord("D"), ord("d")) and (
            real_arm_backend is not None or restricted_hand_backend is not None
        ):
            changed = disarm_real_command_authority(
                real_arm_backend,
                restricted_hand_backend,
            )
            print(
                "[REAL] DISARMED (D); active senders disabled"
                if changed else "[REAL] already DISARMED (D)"
            )
            if real_error_recorder is not None:
                real_error_recorder.record(
                    severity="INFO",
                    component="operator_gate",
                    event="DISARMED_D",
                    message="Operator requested real-command disarm",
                    details={"state_changed": changed},
                )

        elif keycode in (ord("E"), ord("e")) and (
            restricted_hand_backend is not None
        ):
            result = restricted_hand_backend.emergency_stop()
            print("[REAL-HAND]", result.reason, "MODE_NULL requested")
            if real_error_recorder is not None:
                real_error_recorder.record(
                    severity="ERROR",
                    component="sg100_real_hand",
                    event="EMERGENCY_STOP",
                    message=result.reason,
                    details={"command_result": result},
                    include_stack=True,
                )

        elif keycode == ord("1"):
            script_hand_controller.exit()
            mode = grasp_mode_controller.set_mode(GraspMode.FREE)
            print("[HAND] grasp mode:", mode.value, "| FREE: no joint holds")

        elif keycode == ord("2"):
            script_hand_controller.exit()
            mode = grasp_mode_controller.set_mode(GraspMode.HAND)
            print(
                "[HAND] grasp mode:",
                mode.value,
                "| HAND: lock index_j1/little_j1=0",
            )

        elif keycode == ord("3"):
            script_hand_controller.exit()
            mode = grasp_mode_controller.set_mode(GraspMode.CLAW)
            print(
                "[HAND] grasp mode:",
                mode.value,
                "| CLAW: little_j1=-180deg, thumb_j2 init -149deg then follow, "
                f"thumb_j1=+{np.rad2deg(CLAW_THUMB_J1_RAD):.0f}deg hold",
            )

        elif keycode == ord("4"):
            q_seed = script_hand_controller.enter(hand_command_filter.value)
            q_hand_goal = q_seed
            print(
                "[HAND] source: script SHAPING claw "
                "(same pose as 3, not AVP) | J close  K hold  L open"
            )

        elif keycode in (ord("J"), ord("j")):
            if not script_hand_controller.enabled:
                print("[HAND] J ignored; press 4 for script source")
            else:
                motion = script_hand_controller.set_motion(ScriptMotion.CLOSING)
                print("[HAND] script:", motion.value)

        elif keycode in (ord("K"), ord("k")):
            if not script_hand_controller.enabled:
                print("[HAND] K ignored; press 4 for script source")
            else:
                motion = script_hand_controller.set_motion(ScriptMotion.HOLD)
                print("[HAND] script:", motion.value)

        elif keycode in (ord("L"), ord("l")):
            if not script_hand_controller.enabled:
                print("[HAND] L ignored; press 4 for script source")
            else:
                motion = script_hand_controller.set_motion(ScriptMotion.OPENING)
                print("[HAND] script:", motion.value)

        # ========================================================
    # Main runtime loop
    # ========================================================

    arm_joint_comparison_recorder = None
    run_outcome = "completed"
    try:
        if args.arm_joint_compare:
            arm_joint_comparison_recorder = ArmJointComparisonRecorder(
                recording_session_dir,
                rate_hz=args.arm_joint_compare_hz,
                feedback_timeout_ms=args.arm_feedback_timeout_ms,
            )
            print(
                "[ARM-JOINT-COMPARE] read-only log:",
                arm_joint_comparison_recorder.path,
                f"@ {args.arm_joint_compare_hz:g} Hz (background writer)",
            )
        with (
            mujoco.viewer.launch_passive(
                model,
                data,
                key_callback=on_viewer_key,
            )
            as viewer
        ):

            if args.show_hammer_collision_debug:
                enable_collision_debug_visualization(model, viewer)

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
                        if real_error_recorder is not None:
                            real_error_recorder.record(
                                severity="WARNING",
                                component="avp_tracking",
                                event="MALFORMED_RIGHT_HAND_PAYLOAD",
                                message=(
                                    "Candidate tracking frame contained no "
                                    "usable right-hand payload"
                                ),
                                details={
                                    "frame_status": frame_status,
                                    "tracking_query_count": tracking_frame_count,
                                    "rejected_tracking_count": (
                                        rejected_tracking_count
                                    ),
                                    "teleop_state": teleop_arming.state.value,
                                },
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
                                    # Hand articulation is defined by the
                                    # legacy-compatible first 25 joints.  Keep
                                    # optional forearm[25:27] validity isolated
                                    # so a bad cue can fall back to position-only
                                    # without disrupting SG100 retargeting.
                                    hand[:25]
                                )
                            )

                            q_hand_candidate, hand_stages = (
                                retargeter.retarget_with_stages(
                                    canonical
                                )
                            )
                            q_hand_candidate = grasp_mode_controller.apply(
                                q_hand_candidate,
                                dt_s=tracking_period,
                            )

                            # Full hand command provenance (audit only):
                            # raw retarget -> pinch-coordinated ->
                            # power-grasp-locked final.  The remaining
                            # two levels (post-filter, hardware-sent)
                            # are recorded at the send site.
                            if arm_hand_collector is not None:
                                arm_hand_collector.record_hand_pipeline_stages(
                                    hand_stages
                                )

                            if (
                                args.thumb_retarget_debug
                                and (
                                    now
                                    - last_thumb_debug
                                    >= 0.2
                                )
                            ):
                                last_thumb_debug = now

                                raw_q = (
                                    retargeter_no_lock
                                    .retarget(canonical)
                                )

                                thumb_info = (
                                    retargeter
                                    .thumb_lock_diagnostics(
                                        raw_q,
                                        canonical.as_dict(),
                                    )
                                )

                                print()
                                print(
                                    format_thumb_retarget_debug(
                                        thumb_info,
                                        canonical.as_dict(),
                                        hand_tip_geometry_mm(
                                            model,
                                            data,
                                        ),
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
                                if script_hand_controller.enabled:
                                    q_hand_goal = script_hand_controller.enter(
                                        q_hand0_runtime
                                    )

                                if restricted_hand_backend is not None:
                                    anchor_result = restricted_hand_backend.anchor(
                                        q_hand0_runtime
                                    )
                                    _emit_sg100_safety_events(
                                        restricted_hand_backend,
                                        real_error_recorder,
                                    )
                                    if not anchor_result.accepted:
                                        raise RuntimeError(
                                            "SG100 restricted C anchor failed: "
                                            f"{anchor_result.reason}"
                                        )
                                    print(
                                        "[REAL-HAND] C anchor cached with "
                                        "MODE7 zero output; A still required"
                                    )
                                    if real_error_recorder is not None:
                                        real_error_recorder.record(
                                            severity="INFO",
                                            component="sg100_real_hand",
                                            event="C_ANCHOR_CACHED",
                                            message=(
                                                "MODE7 zero-output anchor cached; "
                                                "A enable still required"
                                            ),
                                            details={
                                                "frame_sequence": frame_sequence,
                                                "q_hand0_runtime_rad": (
                                                    q_hand0_runtime
                                                ),
                                                "command_result": anchor_result,
                                                "driver_diagnostics": (
                                                    hand_driver.diagnostics
                                                ),
                                            },
                                        )

                                if (
                                    arm_hand_collector is not None
                                    and not arm_hand_collector.hand_measured_feedback
                                    and not arm_hand_collector.hand_proxy_baseline_valid
                                ):
                                    arm_hand_collector.establish_hand_proxy_baseline(
                                        q_hand_cmd,
                                        timestamp_ns=time.monotonic_ns(),
                                    )
                                    print(
                                        "[DATA] hand baseline: SERIALIZED "
                                        "COMMAND PROXY"
                                    )
                                elif (
                                    arm_hand_collector is not None
                                    and arm_hand_collector.hand_measured_feedback
                                ):
                                    # Measured mode: the replay seed is the
                                    # first MEASURED SG100 feedback; the
                                    # software qcmd needs no proxy baseline.
                                    print(
                                        "[DATA] hand baseline: MEASURED "
                                        "SG100 FEEDBACK (replay seed)"
                                    )

                                wrist_calibrated = True
                                teleop_arming.complete_calibration()
                                latest_control_audit = None
                                latest_ik_accepted = False

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

                                latest_ik_accepted = bool(
                                    ik_result.converged
                                    or args.wrist_roll_ik
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
                                if not script_hand_controller.enabled:
                                    q_hand_goal = (
                                        q_hand_candidate.copy()
                                    )

                                latest_control_audit = {
                                    "raw_avp_wrist_pose": T_avp_wrist.copy(),
                                    "mapped_relative_wrist_target": T_tcp_raw.copy(),
                                    "filtered_tcp_target": T_tcp_target.copy(),
                                    "ik_q_arm_goal": q_arm_goal.copy(),
                                    "ik_primary_candidate": q_arm_candidate.copy(),
                                    "ik_reason": ik_result.reason,
                                    "ik_converged": bool(ik_result.converged),
                                    "continuity_gate_status": (
                                        "not_applied_in_current_runtime"
                                    ),
                                    "latest_target_q": q_arm_goal.copy(),
                                    "tracking_status": "valid",
                                    "safety_status": "ok",
                                    "tracking_frame_sequence": frame_sequence,
                                    "tracking_received_monotonic_ns": (
                                        received_monotonic_ns
                                    ),
                                }


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

                            if real_error_recorder is not None:
                                real_error_recorder.record_exception(
                                    component="avp_tracking",
                                    event="TRACKING_FRAME_REJECTED",
                                    exception=exc,
                                    details={
                                        "tracking_query_count": (
                                            tracking_frame_count
                                        ),
                                        "rejected_tracking_count": (
                                            rejected_tracking_count
                                        ),
                                        "teleop_state": (
                                            teleop_arming.state.value
                                        ),
                                    },
                                )

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
                    if real_error_recorder is not None:
                        last_tracking_ns = (
                            tracking_gate.last_valid_tracking_monotonic_ns
                        )
                        real_error_recorder.record(
                            severity="ERROR",
                            component="avp_tracking",
                            event="TRACKING_TIMEOUT",
                            message=(
                                "Right-hand tracking timed out; real targets "
                                "are held and command authority is invalidated"
                            ),
                            details={
                                "timeout_s": args.tracking_timeout,
                                "last_valid_tracking_monotonic_ns": (
                                    last_tracking_ns
                                ),
                                "tracking_age_ms": (
                                    None
                                    if last_tracking_ns is None
                                    else (
                                        now_monotonic_ns - last_tracking_ns
                                    ) * 1e-6
                                ),
                                "tracking_counts": {
                                    "queries": tracking_frame_count,
                                    "valid": valid_tracking_count,
                                    "rejected": rejected_tracking_count,
                                    "duplicate": tracking_gate.duplicate_frames,
                                    "invalid": tracking_gate.invalid_frames,
                                    "malformed": tracking_gate.malformed_frames,
                                    "stale": tracking_gate.stale_frames,
                                    "missing_metadata": (
                                        tracking_gate.missing_metadata_frames
                                    ),
                                },
                                "teleop_state_before_timeout": (
                                    teleop_arming.state.value
                                ),
                                "real_arm_enabled": (
                                    real_arm_backend.enabled
                                    if real_arm_backend is not None
                                    else None
                                ),
                                "real_hand_state": (
                                    None
                                    if restricted_hand_backend is None
                                    else {
                                        "anchored": (
                                            restricted_hand_backend.anchored
                                        ),
                                        "enabled": (
                                            restricted_hand_backend.enabled
                                        ),
                                        "estopped": (
                                            restricted_hand_backend.estopped
                                        ),
                                    }
                                ),
                                "hand_driver_diagnostics": (
                                    _driver_diagnostics_snapshot(hand_driver)
                                ),
                            },
                            include_stack=True,
                        )
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
                        latest_control_audit = None
                        latest_ik_accepted = False
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
                    if script_hand_controller.enabled:
                        q_script = script_hand_controller.step(physics_dt)
                        if q_script is not None:
                            q_hand_goal = q_script

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

                        feedback = real_arm_backend.joint_state()
                        if arm_hand_collector is not None:
                            # Continuous pre-roll: feedback is observed even
                            # before the first eligible motion command.
                            arm_hand_collector.observe_arm_feedback(feedback)

                        command_bundle_id = None
                        decision_timestamp_ns = None
                        if (
                            arm_hand_collector is not None
                            and real_arm_backend.enabled
                            and teleop_arming.active
                            and latest_control_audit is not None
                        ):
                            command_bundle_id = (
                                arm_hand_collector.next_command_bundle_id()
                            )
                            decision_timestamp_ns = time.monotonic_ns()

                        real_arm_backend.send_joint_target(
                            q_arm_goal,
                            command_bundle_id=command_bundle_id,
                        )

                        q_hand_cmd = (
                            hand_command_filter.update(
                                q_hand_goal,
                                physics_dt,
                            )
                        )

                        # Authoritative canonical hand command: latch for
                        # the recording contract BEFORE any backend
                        # consumes it (identical vector goes to MuJoCo
                        # twin qpos below and/or the SG100 backend).
                        state_recorder.set_hand_command(q_hand_cmd)

                        if (
                            restricted_hand_backend is not None
                            and restricted_hand_backend.enabled
                            and latest_control_audit is not None
                        ):
                            hand_result = restricted_hand_backend.submit(
                                q_hand_cmd,
                                tracking_received_ns=int(
                                    latest_control_audit[
                                        "tracking_received_monotonic_ns"
                                    ]
                                ),
                                dt_s=physics_dt,
                            )
                            _emit_sg100_safety_events(
                                restricted_hand_backend,
                                real_error_recorder,
                            )
                            if hand_result.accepted:
                                hand_telemetry_sequence += 1
                                latest_hand_telemetry = {
                                    "desired": q_hand_cmd.copy(),
                                    "effective": (
                                        hand_result.effective_canonical_qcmd_rad.copy()
                                    ),
                                    "sdk_deg": hand_result.sdk_target_deg.copy(),
                                    "timestamp_ns": hand_result.sent_monotonic_ns,
                                    "sequence": hand_telemetry_sequence,
                                }
                            _print_sg100_command_debug(
                                q_hand_cmd,
                                hand_result,
                                restricted_hand_backend,
                            )
                            if not hand_result.accepted:
                                print(
                                    "[REAL-HAND] safe stop:",
                                    hand_result.reason,
                                )
                                _record_sg100_safe_stop(
                                    real_error_recorder,
                                    hand_result=hand_result,
                                    q_hand_cmd=q_hand_cmd,
                                    restricted_backend=(
                                        restricted_hand_backend
                                    ),
                                    hand_driver=hand_driver,
                                    latest_control_audit=(
                                        latest_control_audit
                                    ),
                                    physics_dt=physics_dt,
                                )
                        elif args.hand_backend == "mock" and hand_driver is not None:
                            hand_driver.send_positions_rad(
                                q_hand_cmd,
                                command_bundle_id=command_bundle_id,
                            )

                        if (
                            hand_driver is not None
                            and now - last_hand_feedback_poll
                            >= hand_feedback_poll_period
                        ):
                            last_hand_feedback_poll = now
                            hand_feedback = (
                                hand_driver.read_feedback()
                            )
                            if (
                                arm_hand_collector is not None
                                and hand_feedback is not None
                            ):
                                arm_hand_collector.observe_hand_feedback(
                                    hand_feedback
                                )
                                if hand_feedback.canonical_valid:
                                    data.qpos[
                                        robot_layout.hand_qpos_indices
                                    ] = hand_feedback.q_rad

                        if feedback is not None:
                            data.qpos[
                                robot_layout.arm_qpos_indices
                            ] = feedback.q_rad

                        if hand_driver is None:
                            data.qpos[
                                robot_layout.hand_qpos_indices
                            ] = q_hand_cmd

                        hand_applied_timestamp_ns = time.monotonic_ns()
                        if command_bundle_id is not None:
                            tracking_age_ns = (
                                decision_timestamp_ns
                                - int(
                                    latest_control_audit[
                                        "tracking_received_monotonic_ns"
                                    ]
                                )
                            )
                            arm_hand_collector.record_control_decision(
                                command_bundle_id=command_bundle_id,
                                decision_timestamp_ns=decision_timestamp_ns,
                                hand_command_applied_timestamp_ns=(
                                    hand_applied_timestamp_ns
                                ),
                                hand_qcmd_rad=q_hand_cmd,
                                audit=latest_control_audit,
                                tracking_valid=(
                                    tracking_age_ns
                                    <= int(args.tracking_timeout * 1e9)
                                ),
                                safety_ok=real_arm_backend.enabled,
                                ik_accepted=latest_ik_accepted,
                                stale_target=(
                                    tracking_age_ns
                                    > int(args.arm_target_timeout_ms * 1e6)
                                ),
                                hand_command_valid=bool(
                                    np.all(np.isfinite(q_hand_cmd))
                                ),
                            )

                        if arm_hand_collector is not None:
                            arm_hand_collector.ingest_arm_command_events(
                                real_arm_backend.drain_command_events()
                            )
                            if hand_driver is not None:
                                arm_hand_collector.ingest_hand_command_events(
                                    hand_driver.drain_command_events()
                                )

                        mujoco.mj_forward(
                            model,
                            data,
                        )

                        if arm_joint_comparison_recorder is not None:
                            arm_joint_comparison_recorder.maybe_record(
                                mujoco_qpos=data.qpos,
                                arm_qpos_indices=robot_layout.arm_qpos_indices,
                                feedback=feedback,
                                arm_backend=real_arm_backend,
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

                        state_recorder.set_hand_command(q_hand_cmd)

                        if (
                            restricted_hand_backend is not None
                            and restricted_hand_backend.enabled
                            and latest_control_audit is not None
                        ):
                            hand_result = restricted_hand_backend.submit(
                                q_hand_cmd,
                                tracking_received_ns=int(
                                    latest_control_audit[
                                        "tracking_received_monotonic_ns"
                                    ]
                                ),
                                dt_s=physics_dt,
                            )
                            _emit_sg100_safety_events(
                                restricted_hand_backend,
                                real_error_recorder,
                            )
                            if hand_result.accepted:
                                hand_telemetry_sequence += 1
                                latest_hand_telemetry = {
                                    "desired": q_hand_cmd.copy(),
                                    "effective": (
                                        hand_result.effective_canonical_qcmd_rad.copy()
                                    ),
                                    "sdk_deg": hand_result.sdk_target_deg.copy(),
                                    "timestamp_ns": hand_result.sent_monotonic_ns,
                                    "sequence": hand_telemetry_sequence,
                                }
                            _print_sg100_command_debug(
                                q_hand_cmd,
                                hand_result,
                                restricted_hand_backend,
                            )
                            if not hand_result.accepted:
                                print(
                                    "[REAL-HAND] safe stop:",
                                    hand_result.reason,
                                )
                                _record_sg100_safe_stop(
                                    real_error_recorder,
                                    hand_result=hand_result,
                                    q_hand_cmd=q_hand_cmd,
                                    restricted_backend=(
                                        restricted_hand_backend
                                    ),
                                    hand_driver=hand_driver,
                                    latest_control_audit=(
                                        latest_control_audit
                                    ),
                                    physics_dt=physics_dt,
                                )
                        elif args.hand_backend == "mock" and hand_driver is not None:
                            hand_driver.send_positions_rad(
                                q_hand_cmd,
                                command_bundle_id=None,
                            )
                        if hand_driver is not None:
                            if (
                                now - last_hand_feedback_poll
                                >= hand_feedback_poll_period
                            ):
                                last_hand_feedback_poll = now
                                hand_feedback = (
                                    hand_driver.read_feedback()
                                )
                                if (
                                    arm_hand_collector is not None
                                    and hand_feedback is not None
                                ):
                                    arm_hand_collector.observe_hand_feedback(
                                        hand_feedback
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

                        if hammer_impact_diagnostic is not None:
                            hammer_impact_diagnostic.sample(
                                data,
                                q_arm_goal,
                                target_tcp=(
                                    tcp_target_filter.value
                                    if tcp_target_filter.initialized
                                    else None
                                ),
                                raw_target_tcp=(
                                    latest_control_audit[
                                        "mapped_relative_wrist_target"
                                    ]
                                    if latest_control_audit is not None
                                    else None
                                ),
                            )

                        state_recorder.maybe_record(
                            data
                        )

                    # External telemetry is a best-effort data-plane tap.
                    # Its configured data-plane rate does not change AVP, RM65,
                    # SG100, physics, or feedback control rates above.
                    telemetry_now_ns = time.monotonic_ns()
                    if (
                        telemetry_publisher is not None
                        and telemetry_now_ns - last_telemetry_publish_ns
                        >= telemetry_period_ns
                    ):
                        publish_external_telemetry(telemetry_now_ns)
                        last_telemetry_publish_ns = telemetry_now_ns

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
        run_outcome = "operator_interrupted"
        if real_error_recorder is not None:
            real_error_recorder.record(
                severity="INFO",
                component="runtime",
                event="KEYBOARD_INTERRUPT",
                message="Operator requested shutdown with Ctrl-C",
            )
        print()

    except Exception as exc:
        run_outcome = "runtime_exception"
        if real_error_recorder is not None:
            real_error_recorder.record_exception(
                component="runtime",
                event="RUNTIME_EXCEPTION",
                exception=exc,
                details={
                    "teleop_state": teleop_arming.state.value,
                    "tracking_query_count": tracking_frame_count,
                    "valid_tracking_count": valid_tracking_count,
                    "rejected_tracking_count": rejected_tracking_count,
                    "hand_driver_diagnostics": (
                        _driver_diagnostics_snapshot(hand_driver)
                    ),
                    "real_arm_sender_stats": (
                        real_arm_backend.sender_stats
                        if real_arm_backend is not None
                        else None
                    ),
                },
            )
        raise

    finally:
        if arm_joint_comparison_recorder is not None:
            arm_joint_comparison_recorder.request_close()
        if telemetry_publisher is not None:
            telemetry_publisher.close()
        if hammer_impact_diagnostic is not None:
            hammer_impact_diagnostic.close()
        if hand_driver is not None:
            # Final hand feedback + sent events, then stop sending.
            try:
                if restricted_hand_backend is not None:
                    restricted_hand_backend.set_enabled(False)
                if arm_hand_collector is not None:
                    arm_hand_collector.observe_hand_feedback(
                        hand_driver.read_feedback()
                    )
                    arm_hand_collector.ingest_hand_command_events(
                        hand_driver.drain_command_events()
                    )
            finally:
                hand_driver.stop()
                hand_driver.disconnect()
                print("[HAND] SG100 driver disconnected")
        if real_arm_backend is not None:
            # v3 §A3.5 order on exit: quiesce, slow stop, disconnect.
            try:
                if arm_hand_collector is not None:
                    arm_hand_collector.observe_arm_feedback(
                        real_arm_backend.joint_state()
                    )
                    arm_hand_collector.ingest_arm_command_events(
                        real_arm_backend.drain_command_events()
                    )
                real_arm_backend.set_enabled(False)
                real_arm_backend.stop_smooth()
            finally:
                real_arm_backend.disconnect()
                print("[REAL-ARM] backend disconnected")
        if arm_joint_comparison_recorder is not None:
            log_closed = arm_joint_comparison_recorder.close()
            print(
                "[ARM-JOINT-COMPARE]",
                f"written={arm_joint_comparison_recorder.written_samples}",
                f"dropped={arm_joint_comparison_recorder.dropped_samples}",
                f"closed={log_closed}",
                f"error={arm_joint_comparison_recorder.last_error}",
            )
        if arm_hand_collector is not None:
            arm_hand_recording_path = arm_hand_collector.finalize()
            print(
                "Arm-hand data    :",
                arm_hand_recording_path,
            )
            print(
                "causal transitions:",
                len(arm_hand_collector.transitions),
            )
            print(
                "hard rejections  :",
                len(arm_hand_collector.rejections),
            )
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

    if real_error_recorder is not None:
        real_error_recorder.finish(
            outcome=run_outcome,
            details={
                "recording_path": recording_path,
                "recorded_frames": state_recorder.frame_count,
                "tracking_counts": {
                    "queries": tracking_frame_count,
                    "valid": valid_tracking_count,
                    "rejected": rejected_tracking_count,
                    "duplicate": tracking_gate.duplicate_frames,
                    "invalid": tracking_gate.invalid_frames,
                    "malformed": tracking_gate.malformed_frames,
                    "stale": tracking_gate.stale_frames,
                    "missing_metadata": (
                        tracking_gate.missing_metadata_frames
                    ),
                },
                "ik_failures": ik_failure_count,
                "ik_fallbacks": ik_fallback_count,
                "final_teleop_state": teleop_arming.state.value,
                "hand_driver_diagnostics": (
                    _driver_diagnostics_snapshot(hand_driver)
                ),
                "real_arm_sender_stats": (
                    real_arm_backend.sender_stats
                    if real_arm_backend is not None
                    else None
                ),
            },
        )
        print(
            "real error log    :",
            real_error_recorder.path,
        )
        if previous_excepthook is not None:
            sys.excepthook = previous_excepthook

    print(
        "========================================"
    )

if __name__ == "__main__":
    main()
