"""Real RM65 backend: CANFD sender + feedback polling (v3 §B2/§B4).

Every RealMan SDK interaction lives here.  The rad<->deg bridge exists
only in this module (v3 §A6.4).  The watchdog runs *inside* the sender
thread so the thread that stops streaming is the thread that streams
(v3 §A3.5).
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Callable

import numpy as np

from sg100_teleop.robot.arm_backend import ArmBackend
from sg100_teleop.robot.arm_state import ARM_DOF, ArmState, ArmTarget
from sg100_teleop.robot.latest_target_buffer import LatestTargetBuffer
from sg100_teleop.robot.rm65_sim import (
    RM65_JOINT_LOWER_RAD,
    RM65_JOINT_UPPER_RAD,
)
from sg100_teleop.robot.safety_monitor import SafetyMonitor, SafetyTrigger, StopSequence

FOLLOW_MODES = ("low", "filter", "high")

# v3 §B3.1 PROFILE_10 initial values.
DEFAULT_SEND_HZ = 100.0
DEFAULT_FEEDBACK_POLL_HZ = 50.0
DEFAULT_QDOT_MAX_RAD_S = 0.5
DEFAULT_QDDOT_MAX_RAD_S2 = 2.0
DEFAULT_SOFT_MARGIN_RAD = np.deg2rad(2.0)
DEFAULT_TARGET_TIMEOUT_S = 0.15
DEFAULT_FEEDBACK_TIMEOUT_S = 0.20
# Gross position-mismatch bound; the primary liveness check is the
# velocity-based dead-arm detector (see SafetyMonitor).
DEFAULT_GROSS_FOLLOW_ERROR_RAD = 0.35
DEFAULT_MAX_SEND_FAILURES = 3
PERIOD_HISTORY = 20000
DEADLINE_MISS_FACTOR = 1.5


def _emit_print(name: str, detail: str) -> None:
    print(f"[REAL-ARM] {name}: {detail}")


class RM65RealBackend(ArmBackend):
    """Stream limiter-bounded joint targets to RM65 over CANFD.

    Lifecycle: ``connect()`` validates the arm and seeds the limiter
    from actual feedback; ``start()`` spawns sender + feedback threads;
    ``send_joint_target()`` publishes from the 60 Hz producer loop;
    any ``SafetyMonitor`` trigger runs the ordered ``StopSequence``.
    """

    def __init__(
        self,
        ip: str,
        port: int = 8080,
        *,
        send_hz: float = DEFAULT_SEND_HZ,
        follow: str = "low",
        radio: int = 0,
        feedback_poll_hz: float = DEFAULT_FEEDBACK_POLL_HZ,
        qdot_max_rad_s: float = DEFAULT_QDOT_MAX_RAD_S,
        qddot_max_rad_s2: float = DEFAULT_QDDOT_MAX_RAD_S2,
        soft_margin_rad: float = DEFAULT_SOFT_MARGIN_RAD,
        target_timeout_s: float = DEFAULT_TARGET_TIMEOUT_S,
        feedback_timeout_s: float = DEFAULT_FEEDBACK_TIMEOUT_S,
        gross_follow_error_rad: float = DEFAULT_GROSS_FOLLOW_ERROR_RAD,
        max_send_failures: int = DEFAULT_MAX_SEND_FAILURES,
        lower_rad: np.ndarray = RM65_JOINT_LOWER_RAD,
        upper_rad: np.ndarray = RM65_JOINT_UPPER_RAD,
        on_event: Callable[[str, str], None] = _emit_print,
        sdk: object | None = None,
    ) -> None:
        if follow not in FOLLOW_MODES:
            raise ValueError(f"follow must be one of {FOLLOW_MODES}")
        if send_hz <= 0.0 or feedback_poll_hz <= 0.0:
            raise ValueError("rates must be positive")
        if max_send_failures < 1:
            raise ValueError("max_send_failures must be >= 1")

        # Lazy import so unit tests run without the RealMan SDK.
        from sg100_teleop.robot.joint_trajectory_safety import (
            JointTrajectorySafetyLimiter,
        )

        self._ip = ip
        self._port = port
        self._send_hz = float(send_hz)
        self._follow = follow
        self._radio = int(radio)
        self._feedback_poll_hz = float(feedback_poll_hz)
        self._on_event = on_event
        self._owns_sdk = sdk is None
        self._sdk = sdk

        self._buffer = LatestTargetBuffer()
        self._limiter = JointTrajectorySafetyLimiter(
            lower_rad=lower_rad,
            upper_rad=upper_rad,
            qdot_max_rad_s=qdot_max_rad_s,
            qddot_max_rad_s2=qddot_max_rad_s2,
            soft_margin_rad=soft_margin_rad,
        )
        self._monitor = SafetyMonitor(
            target_timeout_s=target_timeout_s,
            feedback_timeout_s=feedback_timeout_s,
            gross_follow_error_rad=gross_follow_error_rad,
        )
        self._max_send_failures = int(max_send_failures)

        self._enabled = False
        self._connected = False
        self._shutdown = threading.Event()
        # The SDK is used in RM_SINGLE_MODE_E: its request/response
        # matching on one TCP handle is not safe under concurrent
        # calls.  Every SDK call must hold this lock, and the sender
        # thread additionally performs the feedback polling itself so
        # the hot path (canfd + get_state) is single-threaded.
        self._sdk_lock = threading.Lock()
        self._feedback_lock = threading.Lock()
        self._feedback: ArmState | None = None
        self._feedback_sequence = 0
        # Previous sample for finite-difference feedback velocity
        # (TCP polling has no qd; the UDP push path will fill it
        # directly — see v3 §A5.2).
        self._feedback_prev_q: np.ndarray | None = None
        self._feedback_prev_ns: int = 0
        self._publish_sequence = 0
        self._send_failures = 0
        self._send_count = 0
        self._periods_s: deque[float] = deque(maxlen=PERIOD_HISTORY)
        self._deadline_misses = 0
        self._sender_thread: threading.Thread | None = None
        self._feedback_thread: threading.Thread | None = None

        # v3 §A3.5: disable/drain happen in the sender thread itself;
        # drain is a no-op there, but the StopSequence keeps the order
        # explicit and unit-tested.
        self._stop_sequence = StopSequence(
            disable_sender=lambda: setattr(self, "_enabled", False),
            drain_sender=self._drain_sender,
            slow_stop=self.stop_smooth,
            immediate_stop=self.stop_immediate,
            clear_buffer=self._buffer.clear,
            on_disarm=self._on_disarm,
        )

    # ---------------------------------------------------------
    # Connection
    # ---------------------------------------------------------

    def _create_sdk(self) -> object:
        from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
        from Robotic_Arm.rm_robot_interface import RoboticArm

        # DUAL mode: the SDK runs its own receive thread, so the sender
        # thread (canfd) and the feedback thread (state polling) can
        # call concurrently without corrupting request/response
        # matching.  SINGLE mode forced polling into the sender tick,
        # whose added TCP round-trip stretched the send period and
        # doubled the low-follow tracking lag (measured tau 0.42 s vs
        # 0.2 s with a clean 10 ms stream).
        arm = RoboticArm(rm_thread_mode_e.RM_DUAL_MODE_E)
        handle = arm.rm_create_robot_arm(self._ip, self._port)
        if handle is None or getattr(handle, "id", -1) == -1:
            arm.rm_destroy()
            raise ConnectionError(
                f"could not create arm handle at {self._ip}:{self._port}"
            )
        return arm

    def connect(self) -> None:
        if self._connected:
            return
        if self._sdk is None:
            self._sdk = self._create_sdk()
        dof = int(getattr(self._sdk, "arm_dof", ARM_DOF))
        if dof != ARM_DOF:
            raise ConnectionError(f"expected 6-DoF RM65, controller reports {dof}")
        q_rad = self._read_joints_rad()
        self._limiter.reset(q_rad)
        # Expose the connect-time reading as the first feedback sample
        # so joint_state() is valid before start().
        with self._feedback_lock:
            self._feedback_prev_q = q_rad.copy()
            self._feedback_prev_ns = time.monotonic_ns()
            self._feedback_sequence += 1
            self._feedback = ArmState(
                q_rad=q_rad.copy(),
                qd_rad_s=None,
                received_monotonic_ns=self._feedback_prev_ns,
                sequence=self._feedback_sequence,
                controller_error=0,
                arm_status=None,
                valid=True,
            )
        self._connected = True
        self._on_event("connected", f"{self._ip}:{self._port} q0={np.round(q_rad, 3).tolist()}")

    def start(self) -> None:
        """Spawn sender and feedback threads (idempotent).

        DUAL thread mode + ``_sdk_lock`` make concurrent SDK calls
        safe; keeping the poll off the sender tick preserves the clean
        ~10 ms stream period the low-follow mode needs.
        """
        if not self._connected:
            raise RuntimeError("connect() before start()")
        if self._sender_thread is not None:
            return
        self._shutdown.clear()
        self._sender_thread = threading.Thread(
            target=self._sender_loop, name="rm65-canfd-sender", daemon=True
        )
        self._feedback_thread = threading.Thread(
            target=self._feedback_loop, name="rm65-feedback-poll", daemon=True
        )
        self._feedback_thread.start()
        self._sender_thread.start()

    def disconnect(self) -> None:
        """Join threads; release the SDK handle only if self-created.

        An SDK object injected via ``sdk=`` stays owned by the caller.
        """
        self._shutdown.set()
        for thread in (self._sender_thread, self._feedback_thread):
            if thread is not None:
                thread.join(timeout=2.0)
        self._sender_thread = None
        self._feedback_thread = None
        if self._sdk is not None and self._owns_sdk:
            delete = getattr(self._sdk, "rm_delete_robot_arm", None)
            if callable(delete):
                delete()
        self._connected = False
        self._enabled = False

    # ---------------------------------------------------------
    # ArmBackend interface
    # ---------------------------------------------------------

    @property
    def enabled(self) -> bool:
        return self._enabled

    def set_enabled(self, enabled: bool) -> None:
        """ARM/DISARM; re-enabling re-seeds the limiter from feedback."""
        self._enabled = bool(enabled)
        if enabled:
            self._send_failures = 0
            self._monitor.reset_follow_error_state()
            feedback = self.joint_state()
            if feedback is not None:
                self._limiter.reset(feedback.q_rad)

    def send_joint_target(self, q_rad: np.ndarray) -> None:
        q = np.asarray(q_rad, dtype=np.float64)
        self._publish_sequence += 1
        self._buffer.publish(
            ArmTarget(
                q_rad=q,
                source_timestamp_ns=time.monotonic_ns(),
                sequence=self._publish_sequence,
            )
        )

    def joint_state(self) -> ArmState | None:
        with self._feedback_lock:
            return self._feedback

    def stop_smooth(self) -> int:
        call = getattr(self._sdk, "rm_set_arm_slow_stop", None)
        if not callable(call):
            return 0
        with self._sdk_lock:
            return int(call())

    def stop_immediate(self) -> int:
        call = getattr(self._sdk, "rm_set_arm_stop", None)
        if not callable(call):
            return 0
        with self._sdk_lock:
            return int(call())

    def joint_limits_rad(self) -> tuple[np.ndarray, np.ndarray]:
        return RM65_JOINT_LOWER_RAD.copy(), RM65_JOINT_UPPER_RAD.copy()

    # ---------------------------------------------------------
    # Sender timing statistics (v3 §B4.4)
    # ---------------------------------------------------------

    @property
    def sender_stats(self) -> dict[str, float]:
        periods = np.asarray(self._periods_s, dtype=np.float64)
        if periods.size == 0:
            return {"count": 0}
        return {
            "count": float(periods.size),
            "mean_s": float(periods.mean()),
            "p95_s": float(np.percentile(periods, 95)),
            "p99_s": float(np.percentile(periods, 99)),
            "max_s": float(periods.max()),
            "deadline_misses": float(self._deadline_misses),
        }

    # ---------------------------------------------------------
    # Internals
    # ---------------------------------------------------------

    def _drain_sender(self) -> None:
        """No-op: the stop sequence runs inside the sender thread."""

    def _on_disarm(self, reason: str) -> None:
        self._on_event("disarm", reason)

    def _read_joints_rad(self) -> np.ndarray:
        with self._sdk_lock:
            tag, joints = self._sdk.rm_get_joint_degree()
        if tag != 0:
            raise ConnectionError(f"rm_get_joint_degree -> {tag}")
        joints = np.asarray(joints, dtype=np.float64)[:ARM_DOF]
        if joints.shape != (ARM_DOF,) or not np.all(np.isfinite(joints)):
            raise ConnectionError("invalid joint feedback at connect")
        return np.deg2rad(joints)  # deg -> rad bridge (feedback side)

    def _send_canfd(self, q_cmd_rad: np.ndarray) -> int:
        """rad -> deg bridge (command side); the only conversion site."""
        joint_deg = np.degrees(q_cmd_rad).tolist()
        if self._follow == "low":
            follow_flag, trajectory_mode = False, 0
        elif self._follow == "filter":
            follow_flag, trajectory_mode = True, 2
        else:  # high
            follow_flag, trajectory_mode = True, 0
        with self._sdk_lock:
            return int(
                self._sdk.rm_movej_canfd(
                    joint_deg,
                    follow_flag,
                    0.0,
                    trajectory_mode,
                    self._radio,
                )
            )

    def _poll_feedback_once(self) -> None:
        with self._sdk_lock:
            tag, state = self._sdk.rm_get_current_arm_state()
        now_ns = time.monotonic_ns()
        if tag != 0:
            # Keep the last sample; the feedback-age watchdog decides.
            return
        joints = np.asarray(state.get("joint", [])[:ARM_DOF], dtype=np.float64)
        if joints.shape != (ARM_DOF,) or not np.all(np.isfinite(joints)):
            return
        controller_error = self._parse_controller_error(state.get("err"))
        q_rad = np.deg2rad(joints)  # deg -> rad bridge
        with self._feedback_lock:
            qd_rad_s: np.ndarray | None = None
            if self._feedback_prev_q is not None:
                dt_fb = (now_ns - self._feedback_prev_ns) * 1e-9
                if 1e-3 < dt_fb < 1.0:
                    qd_rad_s = (q_rad - self._feedback_prev_q) / dt_fb
            self._feedback_prev_q = q_rad.copy()
            self._feedback_prev_ns = now_ns
            self._feedback_sequence += 1
            self._feedback = ArmState(
                q_rad=q_rad,
                qd_rad_s=qd_rad_s,  # finite-diff (UDP push fills directly)
                received_monotonic_ns=now_ns,
                sequence=self._feedback_sequence,
                controller_error=controller_error,
                arm_status=None,
                valid=True,
            )

    @staticmethod
    def _parse_controller_error(err: object) -> int | None:
        # SDK shape: {"err_len": 1, "err": ["0"]}
        try:
            codes = err.get("err", []) if isinstance(err, dict) else []
            return int(codes[0]) if codes else 0
        except (TypeError, ValueError, IndexError):
            return None

    def _feedback_loop(self) -> None:
        period = 1.0 / self._feedback_poll_hz
        while not self._shutdown.is_set():
            try:
                self._poll_feedback_once()
            except Exception as exc:  # noqa: BLE001 - keep polling
                self._on_event("feedback_error", repr(exc))
            self._shutdown.wait(period)

    def _sender_loop(self) -> None:
        period = 1.0 / self._send_hz
        next_tick = time.monotonic()
        last_tick_ns = time.monotonic_ns()
        last_cmd: np.ndarray | None = None

        while not self._shutdown.is_set():
            now_ns = time.monotonic_ns()
            dt_s = (now_ns - last_tick_ns) * 1e-9
            last_tick_ns = now_ns
            self._periods_s.append(dt_s)
            if dt_s > period * DEADLINE_MISS_FACTOR:
                self._deadline_misses += 1

            if self._enabled:
                target = self._buffer.latest()
                feedback = self.joint_state()
                trigger = self._monitor.check(
                    now_monotonic_ns=now_ns,
                    latest_target=target,
                    feedback=feedback,
                    last_cmd_rad=last_cmd,
                    commanded_velocity_rad_s=(
                        self._limiter.commanded_velocity_rad_s
                    ),
                    actual_velocity_rad_s=feedback.qd_rad_s,
                    dt_s=dt_s,
                )
                if trigger is not None:
                    self._stop_sequence.execute(trigger.name + "/" + trigger.detail)
                    last_cmd = None
                elif target is not None:
                    # Advance the limiter EVERY tick toward the latest
                    # target and stream the result: the sender performs
                    # the 60 Hz -> send-rate interpolation.  Stale data
                    # is guarded by the target-age watchdog, not by
                    # starving the command stream.
                    if last_cmd is None and feedback is not None:
                        # Fresh session / re-arm: seed from actual state.
                        self._limiter.reset(feedback.q_rad)
                    try:
                        cmd = self._limiter.update(target.q_rad, dt_s)
                    except ValueError as exc:
                        self._on_event("limiter_error", str(exc))
                    else:
                        tag = self._send_canfd(cmd)
                        if tag == 0:
                            self._send_failures = 0
                            self._send_count += 1
                            last_cmd = cmd
                        else:
                            self._send_failures += 1
                            self._on_event("send_failure", f"rm_movej_canfd -> {tag}")
                            if self._send_failures >= self._max_send_failures:
                                self._stop_sequence.execute(
                                    f"send_failures/{self._send_failures}"
                                )
                                last_cmd = None

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                self._shutdown.wait(delay)
            else:
                next_tick = time.monotonic()
