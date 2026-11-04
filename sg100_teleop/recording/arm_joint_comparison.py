"""Opt-in, best-effort joint snapshots; never performs robot I/O."""

from __future__ import annotations

from datetime import datetime
import json
import math
from pathlib import Path
import queue
import threading
import time

import numpy as np

from sg100_teleop.robot.arm_state import ARM_DOF, ArmCommandSent, ArmState


ARM_JOINT_COMPARISON_LOG = "arm_joint_comparison.jsonl"


def _joint_vector(value) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (ARM_DOF,) or not np.all(np.isfinite(vector)):
        raise ValueError("comparison requires six finite joint angles in radians")
    return vector.copy()


def build_arm_joint_comparison(
    *,
    snapshot_monotonic_ns: int,
    mujoco_qpos_rad,
    feedback: ArmState | None,
    command: ArmCommandSent | None,
    arm_enabled: bool,
    feedback_timeout_ms: float,
) -> dict:
    """Copy actual MuJoCo qpos and the SAME feedback used to update that twin.

    The latest sent command is a separate, asynchronous post-limiter sample.
    Its difference from feedback is not a time-aligned response measurement.
    Differences are signed arithmetic differences, not wrapped angles.
    """

    now_ns = int(snapshot_monotonic_ns)
    mujoco_q = _joint_vector(mujoco_qpos_rad)
    feedback_q = None if feedback is None else _joint_vector(feedback.q_rad)
    sent_q = None if command is None else _joint_vector(command.q_rad)
    feedback_ns = None if feedback is None else int(feedback.received_monotonic_ns)
    sent_ns = None if command is None else int(command.sent_monotonic_ns)
    feedback_age_ms = None if feedback_ns is None else (now_ns - feedback_ns) / 1e6
    twin_delta = None if feedback_q is None else np.degrees(mujoco_q - feedback_q)
    sent_delta = (
        None if feedback_q is None or sent_q is None
        else np.degrees(sent_q - feedback_q)
    )
    feedback_valid = bool(feedback is not None and feedback.valid)
    feedback_fresh = bool(
        feedback_age_ms is not None
        and 0.0 <= feedback_age_ms <= feedback_timeout_ms
    )
    return {
        "schema_version": 1,
        "wall_time": datetime.now().astimezone().isoformat(),
        "snapshot_monotonic_ns": now_ns,
        "joint_names": [f"joint_{index + 1}" for index in range(ARM_DOF)],
        "mujoco_source": "runtime data.qpos after feedback synchronization and mj_forward",
        "feedback_source": "same loop-local ArmState used to synchronize the twin",
        "sent_source": "latest successful post-limiter CANFD command; asynchronous to feedback",
        "arm_enabled": bool(arm_enabled),
        "mujoco_qpos_rad": mujoco_q.tolist(),
        "mujoco_qpos_deg": np.degrees(mujoco_q).tolist(),
        "feedback_q_rad": None if feedback_q is None else feedback_q.tolist(),
        "feedback_q_deg": None if feedback_q is None else np.degrees(feedback_q).tolist(),
        "feedback_received_monotonic_ns": feedback_ns,
        "feedback_age_ms": feedback_age_ms,
        "feedback_timeout_ms": float(feedback_timeout_ms),
        "feedback_sequence": None if feedback is None else int(feedback.sequence),
        "feedback_valid": feedback_valid,
        "feedback_fresh": feedback_fresh,
        "comparison_valid": feedback_valid and feedback_fresh,
        "controller_error": None if feedback is None else feedback.controller_error,
        "sent_q_rad": None if sent_q is None else sent_q.tolist(),
        "sent_q_deg": None if sent_q is None else np.degrees(sent_q).tolist(),
        "sent_monotonic_ns": sent_ns,
        "sent_age_ms": None if sent_ns is None else (now_ns - sent_ns) / 1e6,
        "sent_command_sequence": None if command is None else int(command.command_sequence),
        "sent_target_sequence": None if command is None else int(command.target_sequence),
        "sent_source_monotonic_ns": None if command is None else int(command.source_timestamp_ns),
        "feedback_minus_sent_time_ms": (
            None if feedback_ns is None or sent_ns is None
            else (feedback_ns - sent_ns) / 1e6
        ),
        "mujoco_minus_feedback_deg": None if twin_delta is None else twin_delta.tolist(),
        "sent_minus_feedback_deg": None if sent_delta is None else sent_delta.tolist(),
        "max_abs_mujoco_minus_feedback_deg": (
            None if twin_delta is None else float(np.max(np.abs(twin_delta)))
        ),
        "max_abs_sent_minus_feedback_deg": (
            None if sent_delta is None else float(np.max(np.abs(sent_delta)))
        ),
    }


class ArmJointComparisonRecorder:
    """Low-rate cache snapshots with bounded queuing and off-loop disk I/O.

    Disabled runners do not instantiate this class. The producer reads only
    ``latest_command_sent`` and ``enabled``; it never polls an SDK, drains
    command events, writes qpos, or changes robot state. Log errors and queue
    overflow only affect diagnostics, never the control/safety path.
    """

    def __init__(
        self,
        session_dir: str | Path,
        *,
        rate_hz: float = 2.0,
        feedback_timeout_ms: float = 200.0,
        queue_size: int = 16,
    ) -> None:
        if (
            not math.isfinite(rate_hz)
            or not 0.0 < rate_hz <= 10.0
            or not math.isfinite(1e9 / rate_hz)
        ):
            raise ValueError("comparison rate must be finite and in (0, 10] Hz")
        if not math.isfinite(feedback_timeout_ms) or feedback_timeout_ms <= 0.0:
            raise ValueError("feedback_timeout_ms must be finite and > 0")
        if queue_size <= 0:
            raise ValueError("queue_size must be > 0")
        self.path = Path(session_dir) / ARM_JOINT_COMPARISON_LOG
        self.period_ns = int(1e9 / rate_hz)
        self.feedback_timeout_ms = float(feedback_timeout_ms)
        self._next_sample_ns = 0
        self._sequence = 0
        self._queue: queue.Queue = queue.Queue(maxsize=queue_size)
        self._closing = threading.Event()
        self._writer_failed = threading.Event()
        self.dropped_samples = 0
        self.written_samples = 0
        self.last_error: str | None = None
        self._writer = threading.Thread(
            target=self._write_loop,
            name="arm-joint-comparison-log",
            daemon=True,
        )
        try:
            self._writer.start()
        except Exception as exc:
            self.last_error = repr(exc)
            self._writer_failed.set()

    def maybe_record(
        self,
        *,
        mujoco_qpos,
        arm_qpos_indices,
        feedback: ArmState | None,
        arm_backend,
    ) -> bool:
        """Called just after real-arm qpos synchronization and mj_forward.

        ``feedback`` is the loop-local snapshot, not a second backend read.
        The throttle uses wall monotonic time, never MuJoCo ``data.time``.
        """

        started_ns = time.monotonic_ns()
        if (
            self._closing.is_set()
            or self._writer_failed.is_set()
            or started_ns < self._next_sample_ns
        ):
            return False
        self._next_sample_ns = started_ns + self.period_ns
        self._sequence += 1
        try:
            qpos = np.asarray(mujoco_qpos)[arm_qpos_indices].copy()
            command = arm_backend.latest_command_sent()
            enabled = arm_backend.enabled
            payload = build_arm_joint_comparison(
                snapshot_monotonic_ns=time.monotonic_ns(),
                mujoco_qpos_rad=qpos,
                feedback=feedback,
                command=command,
                arm_enabled=enabled,
                feedback_timeout_ms=self.feedback_timeout_ms,
            )
            payload.update(
                sequence=self._sequence,
                snapshot_started_monotonic_ns=started_ns,
                dropped_samples_before=self.dropped_samples,
            )
            self._queue.put_nowait(payload)
            return True
        except queue.Full:
            self.dropped_samples += 1
        except Exception as exc:
            self.dropped_samples += 1
            self.last_error = repr(exc)
        return False

    def _write_loop(self) -> None:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as handle:
                while not self._closing.is_set() or not self._queue.empty():
                    try:
                        payload = self._queue.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    try:
                        handle.write(json.dumps(payload, ensure_ascii=False, allow_nan=False) + "\n")
                        handle.flush()
                        self.written_samples += 1
                    finally:
                        self._queue.task_done()
        except Exception as exc:
            self.last_error = repr(exc)
            self._writer_failed.set()

    def request_close(self) -> None:
        """Stop accepting samples without waiting, before hardware cleanup."""
        self._closing.set()

    def close(self, timeout_s: float = 0.5) -> bool:
        """Bounded flush/join; call after hardware has safely disconnected."""
        self.request_close()
        if self._writer.ident is not None:
            self._writer.join(timeout=max(0.0, timeout_s))
        return not self._writer.is_alive()
