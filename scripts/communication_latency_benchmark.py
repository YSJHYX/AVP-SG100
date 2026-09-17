#!/usr/bin/env python3
"""RM65 + SG100 communication-latency acceptance benchmark.

The acceptance criteria come from the ``通讯延迟测试`` section of the
project acceptance workbook.  The default invocation is a dry run: no device
is opened and no command is sent.  Use ``--simulate`` for an offline report or
``--execute`` for supervised real-hardware measurements.

The real test records:

* RM65 state-query call latency, including SDK-lock contention;
* SG100 explicit status-request enqueue to the first fresh valid callback;
* SG100 callback update intervals and host-observed sequence gaps;
* RM65/SG100 command-acceptance timestamp skew for one bundle id;
* CRC errors, request timeouts, callback gaps, and device errors;
* 100 ms interruption detection and the following 20 ms command gate;
* independent results for stationary, motion, and motion+logging/visualization
  profiles.

Important measurement boundary: the public SG100 SDK queues 0x26 requests and
does not expose the scheduler's wire-send timestamp or a response transaction
id.  SG100 RTT is therefore an uncorrelated enqueue-to-next-fresh-callback
proxy: a periodic callback can satisfy the wait, so it is not equivalent to
wire RTT. SG100 command time is SetHandControl cache-acceptance time. The JSON
report preserves these qualifications and never labels them as physical motion
synchronization.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import sys
import threading
import time
import traceback
from collections import deque
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from sg100_teleop.robot.rm65_real import RM65RealBackend  # noqa: E402
from sg100_teleop.robot.arm_state import ARM_DOF, ArmState  # noqa: E402
from sg100_teleop.robot.sg100_driver import (  # noqa: E402
    HAND_DOF,
    MODE_JOINT_POSITION,
)
from sg100_teleop.robot.sg100_restricted_avp_backend import (  # noqa: E402
    SG100RestrictedAVPBackend,
)
from sg100_teleop.robot.sg100_sdk_driver import (  # noqa: E402
    DEFAULT_BAUDRATE,
    DEFAULT_SDK_DIR,
    SG100SDKDriver,
)


PROFILE_STATIONARY = "stationary"
PROFILE_MOTION = "motion"
PROFILE_MOTION_LOG_VIS = "motion-logging-visualization"
REQUIRED_PROFILES = (
    PROFILE_STATIONARY,
    PROFILE_MOTION,
    PROFILE_MOTION_LOG_VIS,
)

HAND_JOINT_NAMES = (
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
)
HARDWARE_VALIDATED_MOTION_JOINTS = ("index_j3",)


@dataclasses.dataclass(frozen=True)
class AcceptanceCriteria:
    """Provisional thresholds copied from the workbook."""

    rtt_p95_ms: float = 50.0
    rtt_p99_ms: float = 70.0
    rtt_max_ms: float = 100.0
    feedback_min_hz: float = 20.0
    feedback_interval_p99_ms: float = 50.0
    feedback_interval_max_ms: float = 100.0
    command_sync_p95_ms: float = 50.0
    command_sync_max_ms: float = 100.0
    max_error_rate: float = 0.001
    interruption_timeout_ms: float = 100.0
    command_block_deadline_ms: float = 20.0


CRITERIA = AcceptanceCriteria()


@dataclasses.dataclass(frozen=True)
class RM65QuerySample:
    request_queued_ns: int
    request_started_ns: int
    response_received_ns: int | None
    valid: bool
    timeout: bool
    error: str | None = None

    @property
    def latency_ms(self) -> float | None:
        if not self.valid or self.response_received_ns is None:
            return None
        return (self.response_received_ns - self.request_started_ns) * 1e-6

    @property
    def sdk_lock_wait_ms(self) -> float:
        return (self.request_started_ns - self.request_queued_ns) * 1e-6


class TimedRM65Backend(RM65RealBackend):
    """RM65 backend with SDK-call RTT and lock-wait observations.

    The RTT clock starts only after acquiring the production backend's SDK
    lock, immediately before ``rm_get_current_arm_state``. Lock wait is stored
    separately, so the communication RTT is not inflated by local sender
    contention. The feedback update below intentionally mirrors the production
    method while retaining its limiter, sender, and watchdog architecture.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._latency_query_lock = threading.Lock()
        self._latency_query_samples: deque[RM65QuerySample] = deque()

    def _poll_feedback_once(self) -> None:
        request_queued_ns = time.monotonic_ns()
        request_started_ns = request_queued_ns
        response_received_ns: int | None = None
        valid = False
        timeout = False
        error: str | None = None
        raised: BaseException | None = None
        try:
            with self._sdk_lock:
                request_started_ns = time.monotonic_ns()
                tag, state = self._sdk.rm_get_current_arm_state()
            response_received_ns = time.monotonic_ns()
            if tag != 0:
                timeout = int(tag) in {-2, -5}
                error = f"rm_get_current_arm_state -> {int(tag)}"
            else:
                joints = np.asarray(
                    state.get("joint", [])[:ARM_DOF], dtype=np.float64
                )
                if joints.shape != (ARM_DOF,) or not np.all(np.isfinite(joints)):
                    error = "invalid_joint_state_response"
                else:
                    controller_error = self._parse_controller_error(state.get("err"))
                    q_rad = np.deg2rad(joints)
                    with self._feedback_lock:
                        qd_rad_s: np.ndarray | None = None
                        if self._feedback_prev_q is not None:
                            dt_fb = (
                                response_received_ns - self._feedback_prev_ns
                            ) * 1e-9
                            if 1e-3 < dt_fb < 1.0:
                                qd_rad_s = (
                                    q_rad - self._feedback_prev_q
                                ) / dt_fb
                        self._feedback_prev_q = q_rad.copy()
                        self._feedback_prev_ns = response_received_ns
                        self._feedback_sequence += 1
                        self._feedback = ArmState(
                            q_rad=q_rad,
                            qd_rad_s=qd_rad_s,
                            received_monotonic_ns=response_received_ns,
                            sequence=self._feedback_sequence,
                            controller_error=controller_error,
                            arm_status=None,
                            valid=True,
                            controller_tcp=self._parse_controller_tcp(
                                state.get("pose")
                            ),
                        )
                    valid = True
        except BaseException as exc:  # preserve production loop behaviour
            raised = exc
            error = repr(exc)
        sample = RM65QuerySample(
            request_queued_ns=request_queued_ns,
            request_started_ns=request_started_ns,
            response_received_ns=response_received_ns,
            valid=valid,
            timeout=timeout,
            error=error,
        )
        with self._latency_query_lock:
            self._latency_query_samples.append(sample)
        if raised is not None:
            raise raised

    def _feedback_loop(self) -> None:
        """Poll on an absolute cadence instead of RTT plus one period."""

        period_s = 1.0 / self._feedback_poll_hz
        next_tick = time.monotonic()
        while not self._shutdown.is_set():
            try:
                self._poll_feedback_once()
            except Exception as exc:  # noqa: BLE001 - keep polling
                self._on_event("feedback_error", repr(exc))
            next_tick += period_s
            delay_s = next_tick - time.monotonic()
            if delay_s > 0.0:
                self._shutdown.wait(delay_s)
            else:
                next_tick = time.monotonic()

    def drain_latency_query_samples(self) -> tuple[RM65QuerySample, ...]:
        with self._latency_query_lock:
            samples = tuple(self._latency_query_samples)
            self._latency_query_samples.clear()
        return samples

    def latest_target_timestamp_ns(self) -> int | None:
        """Timestamp used by the production stale-target watchdog."""

        target = self._buffer.latest()
        return None if target is None else int(target.source_timestamp_ns)


class ThreadSafeSG100SDKDriver(SG100SDKDriver):
    """Serialize Python-side callback-cache reads across benchmark threads."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._benchmark_feedback_read_lock = threading.Lock()

    def read_feedback(self):
        with self._benchmark_feedback_read_lock:
            return super().read_feedback()


@dataclasses.dataclass(frozen=True)
class ObservedHandFeedback:
    received_ns: int
    sequence: int
    valid: bool
    joint_modes: tuple[int, ...] | None


class SG100FeedbackObserver:
    """Continuously deduplicate the SDK's latest callback cache."""

    def __init__(self, driver: Any, *, poll_period_s: float = 0.001) -> None:
        self.driver = driver
        self.poll_period_s = float(poll_period_s)
        self._samples: list[ObservedHandFeedback] = []
        self._errors: list[dict[str, Any]] = []
        self._condition = threading.Condition()
        self._shutdown = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_sequence = -1

    def start(self) -> None:
        if self._thread is not None:
            return
        self._shutdown.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="sg100-latency-feedback-observer",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._shutdown.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
        self._thread = None

    def _run(self) -> None:
        while not self._shutdown.is_set():
            try:
                feedback = self.driver.read_feedback()
                if feedback is not None:
                    sequence = int(feedback.sequence)
                    with self._condition:
                        if sequence != self._last_sequence:
                            sample = ObservedHandFeedback(
                                received_ns=int(feedback.received_monotonic_ns),
                                sequence=sequence,
                                valid=bool(feedback.valid),
                                joint_modes=(
                                    None
                                    if feedback.joint_modes is None
                                    else tuple(int(value) for value in feedback.joint_modes)
                                ),
                            )
                            self._samples.append(sample)
                            self._last_sequence = sequence
                            self._condition.notify_all()
            except Exception as exc:  # noqa: BLE001 - diagnostic worker
                with self._condition:
                    self._errors.append(
                        {"timestamp_ns": time.monotonic_ns(), "error": repr(exc)}
                    )
                    self._condition.notify_all()
            self._shutdown.wait(self.poll_period_s)

    @property
    def latest_sequence(self) -> int:
        with self._condition:
            return self._last_sequence

    def latest_valid(self) -> ObservedHandFeedback | None:
        with self._condition:
            for sample in reversed(self._samples):
                if sample.valid:
                    return sample
        return None

    def wait_fresh(
        self,
        *,
        after_sequence: int,
        not_before_ns: int,
        timeout_s: float,
    ) -> ObservedHandFeedback | None:
        deadline = time.monotonic() + float(timeout_s)
        with self._condition:
            while True:
                for sample in reversed(self._samples):
                    if sample.sequence <= after_sequence:
                        break
                    if sample.received_ns >= not_before_ns:
                        return sample
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(timeout=remaining)

    def samples_between(
        self, start_ns: int, end_ns: int
    ) -> tuple[ObservedHandFeedback, ...]:
        with self._condition:
            return tuple(
                sample
                for sample in self._samples
                if start_ns <= sample.received_ns <= end_ns
            )

    def errors_between(self, start_ns: int, end_ns: int) -> tuple[dict, ...]:
        with self._condition:
            return tuple(
                row
                for row in self._errors
                if start_ns <= int(row["timestamp_ns"]) <= end_ns
            )

    def discard_through(self, timestamp_ns: int) -> None:
        """Bound observer memory after a run while retaining newer samples."""

        cutoff = int(timestamp_ns)
        with self._condition:
            self._samples = [
                sample
                for sample in self._samples
                if sample.received_ns > cutoff
            ]
            self._errors = [
                row
                for row in self._errors
                if int(row["timestamp_ns"]) > cutoff
            ]


@dataclasses.dataclass(frozen=True)
class SG100QuerySample:
    request_id: int
    request_started_ns: int
    request_accepted_ns: int | None
    response_received_ns: int | None
    response_sequence: int | None
    valid: bool
    timeout: bool
    error: str | None = None

    @property
    def latency_ms(self) -> float | None:
        if not self.valid or self.response_received_ns is None:
            return None
        return (self.response_received_ns - self.request_started_ns) * 1e-6


class SG100QueryWorker:
    """Issue one outstanding 0x26 status request at a time."""

    def __init__(
        self,
        driver: Any,
        observer: SG100FeedbackObserver,
        *,
        query_hz: float,
        timeout_s: float,
        duration_s: float,
    ) -> None:
        self.driver = driver
        self.observer = observer
        self.query_hz = float(query_hz)
        self.timeout_s = float(timeout_s)
        self.duration_s = float(duration_s)
        self.samples: list[SG100QuerySample] = []
        self._thread: threading.Thread | None = None
        self._shutdown = threading.Event()

    def start(self) -> None:
        self._shutdown.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="sg100-explicit-status-query",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._shutdown.set()

    def join(self) -> None:
        if self._thread is not None:
            self._thread.join(timeout=self.duration_s + self.timeout_s + 2.0)
            if self._thread.is_alive():
                raise RuntimeError("SG100 query worker did not stop")

    def _run(self) -> None:
        period_s = 1.0 / self.query_hz
        deadline = time.monotonic() + self.duration_s
        next_tick = time.monotonic()
        request_id = 0
        while time.monotonic() < deadline and not self._shutdown.is_set():
            request_id += 1
            baseline_sequence = self.observer.latest_sequence
            request_started_ns = time.monotonic_ns()
            try:
                accepted = bool(self.driver.request_status())
            except Exception as exc:  # noqa: BLE001 - keep raw failure
                self.samples.append(
                    SG100QuerySample(
                        request_id,
                        request_started_ns,
                        None,
                        None,
                        None,
                        False,
                        False,
                        repr(exc),
                    )
                )
            else:
                request_accepted_ns = time.monotonic_ns()
                if not accepted:
                    self.samples.append(
                        SG100QuerySample(
                            request_id,
                            request_started_ns,
                            request_accepted_ns,
                            None,
                            None,
                            False,
                            False,
                            "request_rejected",
                        )
                    )
                else:
                    response = self.observer.wait_fresh(
                        after_sequence=baseline_sequence,
                        not_before_ns=request_started_ns,
                        timeout_s=self.timeout_s,
                    )
                    if response is None:
                        self.samples.append(
                            SG100QuerySample(
                                request_id,
                                request_started_ns,
                                request_accepted_ns,
                                None,
                                None,
                                False,
                                True,
                                "timeout_waiting_for_fresh_callback",
                            )
                        )
                    else:
                        self.samples.append(
                            SG100QuerySample(
                                request_id,
                                request_started_ns,
                                request_accepted_ns,
                                response.received_ns,
                                response.sequence,
                                bool(response.valid),
                                False,
                                None if response.valid else "invalid_callback",
                            )
                        )
            next_tick += period_s
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                self._shutdown.wait(delay)
            else:
                next_tick = time.monotonic()


class HeadlessVisualizationLoad:
    """Exercise a normal plotting/render path without opening a GUI."""

    def __init__(self, *, refresh_hz: float = 10.0) -> None:
        import matplotlib

        matplotlib.use("Agg")
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg

        self._figure = Figure(figsize=(6.4, 3.6), dpi=100)
        self._canvas = FigureCanvasAgg(self._figure)
        self._axis = self._figure.subplots()
        self._refresh_period_ns = int(1e9 / float(refresh_hz))
        self._last_render_ns = 0
        self._times: deque[float] = deque(maxlen=300)
        self._values: deque[float] = deque(maxlen=300)

    def update(self, elapsed_s: float, value: float) -> None:
        self._times.append(float(elapsed_s))
        self._values.append(float(value))
        now_ns = time.monotonic_ns()
        if now_ns - self._last_render_ns < self._refresh_period_ns:
            return
        self._last_render_ns = now_ns
        self._axis.clear()
        self._axis.plot(tuple(self._times), tuple(self._values))
        self._axis.set_xlabel("time (s)")
        self._axis.set_ylabel("arm target offset (rad)")
        self._canvas.draw()


def _metric_le(value: float | None, limit: float) -> bool:
    return value is not None and math.isfinite(value) and value <= limit


def summarize_values(values: Sequence[float]) -> dict[str, float | int | None]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {
            "count": 0,
            "mean": None,
            "p95": None,
            "p99": None,
            "max": None,
        }
    array = np.asarray(finite, dtype=np.float64)
    return {
        "count": int(array.size),
        "mean": float(array.mean()),
        "p95": float(np.percentile(array, 95)),
        "p99": float(np.percentile(array, 99)),
        "max": float(array.max()),
    }


def max_consecutive(flags: Iterable[bool]) -> int:
    longest = 0
    current = 0
    for flag in flags:
        if flag:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest


def evaluate_rtt(
    samples: Sequence[dict[str, Any]],
    *,
    criteria: AcceptanceCriteria = CRITERIA,
) -> dict[str, Any]:
    latencies = [
        float(row["latency_ms"])
        for row in samples
        if bool(row.get("valid")) and row.get("latency_ms") is not None
    ]
    timeout_count = sum(bool(row.get("timeout")) for row in samples)
    error_count = sum(not bool(row.get("valid")) for row in samples)
    total = len(samples)
    stats = summarize_values(latencies)
    lock_waits = [
        float(row["sdk_lock_wait_ms"])
        for row in samples
        if row.get("sdk_lock_wait_ms") is not None
    ]
    result = {
        "total_requests": total,
        "valid_responses": len(latencies),
        "timeout_count": timeout_count,
        "error_count": error_count,
        "max_consecutive_failures": max_consecutive(
            not bool(row.get("valid")) for row in samples
        ),
        "timeout_rate": (timeout_count / total if total else None),
        "error_rate": (error_count / total if total else None),
        "latency_ms": stats,
    }
    if lock_waits:
        result["sdk_lock_wait_ms"] = summarize_values(lock_waits)
    result["acceptance"] = {
        "p95_le_50_ms": _metric_le(stats["p95"], criteria.rtt_p95_ms),
        "p99_le_70_ms": _metric_le(stats["p99"], criteria.rtt_p99_ms),
        "max_le_100_ms": _metric_le(stats["max"], criteria.rtt_max_ms),
        "timeouts_retained": total == len(samples),
    }
    result["pass"] = all(result["acceptance"].values())
    return result


def evaluate_feedback(
    samples: Sequence[dict[str, Any]],
    *,
    criteria: AcceptanceCriteria = CRITERIA,
) -> dict[str, Any]:
    # Preserve callback observation order. Sorting by timestamp would hide
    # clock reversals and out-of-order callbacks, which are test failures.
    ordered = list(samples)
    valid = [row for row in ordered if bool(row.get("valid"))]
    intervals_ms: list[float] = []
    non_monotonic = 0
    estimated_gaps = 0
    gap_events: list[bool] = []
    for previous, current in zip(valid, valid[1:]):
        dt_ns = int(current["received_ns"]) - int(previous["received_ns"])
        if dt_ns <= 0:
            non_monotonic += 1
            continue
        intervals_ms.append(dt_ns * 1e-6)
        sequence_delta = int(current["sequence"]) - int(previous["sequence"])
        if sequence_delta > 1:
            estimated_gaps += sequence_delta - 1
            gap_events.append(True)
        elif sequence_delta <= 0:
            non_monotonic += 1
            gap_events.append(True)
        else:
            gap_events.append(False)
    mean_hz: float | None = None
    if len(valid) >= 2:
        elapsed_ns = int(valid[-1]["received_ns"]) - int(valid[0]["received_ns"])
        if elapsed_ns > 0:
            mean_hz = (len(valid) - 1) * 1e9 / elapsed_ns
    stats = summarize_values(intervals_ms)
    result = {
        "observed_frames": len(ordered),
        "valid_new_frames": len(valid),
        "invalid_frames": len(ordered) - len(valid),
        "mean_frequency_hz": mean_hz,
        "interval_ms": stats,
        "non_monotonic_count": non_monotonic,
        "estimated_host_callback_sequence_gaps": estimated_gaps,
        "max_consecutive_invalid_frames": max_consecutive(
            not bool(row.get("valid")) for row in ordered
        ),
        "max_consecutive_gap_events": max_consecutive(gap_events),
        "sequence_semantics": (
            "host callback sequence; gaps are estimated missed callbacks, "
            "not a device wire sequence"
        ),
    }
    result["acceptance"] = {
        "mean_frequency_at_least_20_hz": bool(
            mean_hz is not None and mean_hz >= criteria.feedback_min_hz
        ),
        "interval_p99_le_50_ms": _metric_le(
            stats["p99"], criteria.feedback_interval_p99_ms
        ),
        "interval_max_le_100_ms": _metric_le(
            stats["max"], criteria.feedback_interval_max_ms
        ),
        "timestamps_monotonic": non_monotonic == 0,
    }
    result["pass"] = all(result["acceptance"].values())
    return result


def evaluate_command_sync(
    samples: Sequence[dict[str, Any]],
    *,
    criteria: AcceptanceCriteria = CRITERIA,
) -> dict[str, Any]:
    skews = [
        float(row["skew_ms"])
        for row in samples
        if bool(row.get("valid")) and row.get("skew_ms") is not None
    ]
    total = len(samples)
    missing = total - len(skews)
    stats = summarize_values(skews)
    result = {
        "total_bundles": total,
        "matched_bundles": len(skews),
        "unmatched_bundles": missing,
        "unmatched_rate": (missing / total if total else None),
        "absolute_send_skew_ms": stats,
        "semantics": (
            "RM65 rm_movej_canfd return vs SG100 SetHandControl cache "
            "acceptance; this is command-submission synchrony, not motion synchrony"
        ),
    }
    result["acceptance"] = {
        "p95_le_50_ms": _metric_le(stats["p95"], criteria.command_sync_p95_ms),
        "max_le_100_ms": _metric_le(stats["max"], criteria.command_sync_max_ms),
        "unmatched_rate_le_0_1_percent": bool(
            total > 0 and missing / total <= criteria.max_error_rate
        ),
    }
    result["pass"] = all(result["acceptance"].values())
    return result


def evaluate_integrity(
    *,
    valid_frames: int,
    invalid_frames: int,
    status_frames: int,
    crc_errors: int,
    request_count: int,
    request_timeouts: int,
    request_errors: int,
    estimated_sequence_gaps: int,
    device_error_frames: int,
    control_failure_events: int,
    criteria: AcceptanceCriteria = CRITERIA,
) -> dict[str, Any]:
    frame_denominator = max(
        int(status_frames), int(valid_frames) + int(invalid_frames), 0
    )
    gap_denominator = max(valid_frames + estimated_sequence_gaps, 0)
    crc_rate = crc_errors / frame_denominator if frame_denominator else None
    invalid_frame_rate = (
        invalid_frames / frame_denominator if frame_denominator else None
    )
    timeout_rate = request_timeouts / request_count if request_count else None
    request_failure_count = request_timeouts + request_errors
    request_failure_rate = (
        request_failure_count / request_count if request_count else None
    )
    gap_rate = (
        estimated_sequence_gaps / gap_denominator if gap_denominator else None
    )
    result = {
        "valid_frame_count": int(valid_frames),
        "invalid_frame_count": int(invalid_frames),
        "status_frame_denominator": int(frame_denominator),
        "invalid_frame_rate": invalid_frame_rate,
        "crc_error_count": int(crc_errors),
        "crc_error_rate": crc_rate,
        "request_count": int(request_count),
        "request_timeout_count": int(request_timeouts),
        "request_timeout_rate": timeout_rate,
        "request_non_timeout_error_count": int(request_errors),
        "request_failure_count": int(request_failure_count),
        "request_failure_rate": request_failure_rate,
        "estimated_callback_gap_count": int(estimated_sequence_gaps),
        "estimated_callback_gap_denominator": int(gap_denominator),
        "estimated_callback_gap_rate": gap_rate,
        "device_error_frames": int(device_error_frames),
        "control_failure_events": int(control_failure_events),
        "device_sequence_available": False,
        "loss_rate_label": "estimated callback gap rate",
    }
    result["acceptance"] = {
        "crc_error_rate_le_0_1_percent": bool(
            crc_rate is not None and crc_rate <= criteria.max_error_rate
        ),
        "invalid_frame_rate_le_0_1_percent": bool(
            invalid_frame_rate is not None
            and invalid_frame_rate <= criteria.max_error_rate
        ),
        "request_failure_rate_le_0_1_percent": bool(
            request_failure_rate is not None
            and request_failure_rate <= criteria.max_error_rate
        ),
        "estimated_gap_rate_le_0_1_percent": bool(
            gap_rate is not None and gap_rate <= criteria.max_error_rate
        ),
        "no_device_error_frames": device_error_frames == 0,
        "no_control_failure_events": control_failure_events == 0,
    }
    result["pass"] = all(result["acceptance"].values())
    return result


def evaluate_interruption(
    row: dict[str, Any], *, criteria: AcceptanceCriteria = CRITERIA
) -> dict[str, Any]:
    detection_ms = float(row["timeout_trigger_ns"] - row["last_valid_ns"]) * 1e-6
    block_ms = float(
        row["commands_forbidden_ns"] - row["timeout_trigger_ns"]
    ) * 1e-6
    evaluated = dict(row)
    evaluated["detection_ms"] = detection_ms
    evaluated["command_block_after_trigger_ms"] = block_ms
    evaluated["acceptance"] = {
        "fault_detected": bool(row.get("fault_detected", True)),
        "timeout_triggered_after_100_ms_loss": detection_ms >= criteria.interruption_timeout_ms,
        "trigger_lateness_within_20_ms": detection_ms <= (
            criteria.interruption_timeout_ms + criteria.command_block_deadline_ms
        ),
        "new_commands_forbidden_within_20_ms": (
            0.0 <= block_ms <= criteria.command_block_deadline_ms
        ),
        "no_command_after_gate": int(row.get("commands_after_gate", 0)) == 0,
        "explicit_post_gate_attempts_rejected": (
            "post_gate_attempts" not in row
            or (
                int(row.get("post_gate_attempts", 0)) > 0
                and int(row.get("post_gate_attempts", 0))
                == int(row.get("post_gate_attempt_rejections", 0))
            )
        ),
        "no_historical_target_replay": int(
            row.get("historical_targets_replayed", 0)
        ) == 0,
    }
    evaluated["pass"] = all(evaluated["acceptance"].values())
    return evaluated


def evaluate_sample_coverage(
    *,
    configured_duration_s: float,
    actual_duration_s: float,
    query_hz: float,
    command_hz: float,
    rm65_query_count: int,
    sg100_query_count: int,
    feedback_count: int,
    command_bundle_count: int,
    minimum_fraction: float = 0.95,
    criteria: AcceptanceCriteria = CRITERIA,
) -> dict[str, Any]:
    expected_queries = configured_duration_s * query_hz
    expected_commands = configured_duration_s * command_hz
    expected_feedback = configured_duration_s * criteria.feedback_min_hz

    def fraction(actual: int, expected: float) -> float:
        return actual / expected if expected > 0.0 else 0.0

    result = {
        "minimum_required_fraction": minimum_fraction,
        "configured_duration_s": configured_duration_s,
        "actual_duration_s": actual_duration_s,
        "rm65_query_fraction": fraction(rm65_query_count, expected_queries),
        "sg100_query_fraction": fraction(sg100_query_count, expected_queries),
        "feedback_fraction_at_20_hz_minimum": fraction(
            feedback_count, expected_feedback
        ),
        "command_bundle_fraction": fraction(
            command_bundle_count, expected_commands
        ),
    }
    result["acceptance"] = {
        "duration_covered": actual_duration_s
        >= configured_duration_s * minimum_fraction,
        "rm65_queries_covered": result["rm65_query_fraction"]
        >= minimum_fraction,
        "sg100_queries_covered": result["sg100_query_fraction"]
        >= minimum_fraction,
        "feedback_covered": result["feedback_fraction_at_20_hz_minimum"]
        >= minimum_fraction,
        "command_bundles_covered": result["command_bundle_fraction"]
        >= minimum_fraction,
    }
    result["pass"] = all(result["acceptance"].values())
    return result


def counter_delta(after: int, before: int, *, bits: int = 64) -> int:
    after_i = int(after)
    before_i = int(before)
    if after_i >= before_i:
        return after_i - before_i
    return (1 << bits) - before_i + after_i


def _named_angles_deg(path: Path, section: str, names: Sequence[str]) -> np.ndarray:
    with path.open("r", encoding="utf-8") as stream:
        root = yaml.safe_load(stream)
    try:
        mapping = root[section]["angles_deg"]
    except (TypeError, KeyError) as exc:
        raise ValueError(f"{path}: missing {section}.angles_deg") from exc
    if not isinstance(mapping, dict):
        raise ValueError(f"{path}: {section}.angles_deg must be a mapping")
    missing = [name for name in names if name not in mapping]
    unknown = sorted(set(mapping) - set(names))
    if missing or unknown:
        raise ValueError(
            f"{path}: {section}.angles_deg missing={missing}, unknown={unknown}"
        )
    values: list[float] = []
    for name in names:
        value = mapping[name]
        if value is None:
            raise ValueError(f"{path}: {section}.angles_deg.{name} is null")
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"{path}: {section}.angles_deg.{name} is not numeric"
            ) from exc
        if not math.isfinite(number):
            raise ValueError(
                f"{path}: {section}.angles_deg.{name} is not finite"
            )
        values.append(number)
    return np.asarray(values, dtype=np.float64)


def load_hand_center_deg(path: str | None) -> np.ndarray | None:
    if path is None:
        return None
    return _named_angles_deg(Path(path).expanduser().resolve(), "hand", HAND_JOINT_NAMES)


def _counter_snapshot(driver: Any) -> dict[str, Any]:
    diagnostics = driver.diagnostics
    return {
        "status_recv_count": int(diagnostics.get("status_recv_count", 0)),
        "hand_control_calls": int(diagnostics.get("hand_control_calls", 0)),
        "crc_error_count": int(diagnostics.get("crc_error_count", 0)),
        "error_frames": int(diagnostics.get("error_frames", 0)),
        "crc_error_timestamps_ns": [
            int(value)
            for value in diagnostics.get("crc_error_timestamps_ns", [])
        ],
    }


class RealCommunicationRig:
    """One connection shared by independent load-profile repetitions."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.arm_events: list[dict[str, Any]] = []
        self._bundle_id = 0
        self._command_gate_lock = threading.Lock()
        self._command_gate_open = False
        self._command_gate_closed_ns: int | None = None
        self.command_gate_rejections = 0
        self.arm = TimedRM65Backend(
            args.rm65_ip,
            args.rm65_port,
            send_hz=args.arm_send_hz,
            feedback_poll_hz=args.query_hz,
            qdot_max_rad_s=args.arm_max_velocity_rad_s,
            qddot_max_rad_s2=args.arm_max_acceleration_rad_s2,
            target_timeout_s=CRITERIA.interruption_timeout_ms * 1e-3,
            feedback_timeout_s=CRITERIA.interruption_timeout_ms * 1e-3,
            arm_not_moving_stop=False,
            on_event=self._on_arm_event,
        )
        self.hand = ThreadSafeSG100SDKDriver(
            args.sg100_port,
            baudrate=args.sg100_baudrate,
            sdk_dir=args.sg100_sdk_dir,
        )
        self.observer = SG100FeedbackObserver(self.hand)
        self.arm_center_rad: np.ndarray | None = None
        self.hand_center_rad = (
            np.zeros(HAND_DOF, dtype=np.float64)
            if args.hand_center_deg is None
            else np.deg2rad(args.hand_center_deg)
        )
        self.restricted_hand: SG100RestrictedAVPBackend | None = None
        self._active_profile: str | None = None

    def _on_arm_event(self, name: str, detail: str) -> None:
        row = {
            "timestamp_ns": time.monotonic_ns(),
            "name": str(name),
            "detail": str(detail),
        }
        self.arm_events.append(row)
        print(f"[RM65] {name}: {detail}")

    def next_bundle_id(self) -> int:
        self._bundle_id += 1
        return self._bundle_id

    @property
    def command_gate_open(self) -> bool:
        with self._command_gate_lock:
            return self._command_gate_open

    def _open_command_gate(self) -> None:
        with self._command_gate_lock:
            self._command_gate_open = True
            self._command_gate_closed_ns = None

    def _close_command_gate(self) -> int:
        now_ns = time.monotonic_ns()
        with self._command_gate_lock:
            if self._command_gate_open or self._command_gate_closed_ns is None:
                self._command_gate_open = False
                self._command_gate_closed_ns = now_ns
            closed_ns = self._command_gate_closed_ns
        assert closed_ns is not None
        return closed_ns

    def connect(self) -> None:
        self.arm.connect()
        self.arm.start()
        try:
            self.hand.connect()
            self.hand.start()
            self.hand.enter_realtime()
            self.observer.start()
            feedback = self.observer.wait_fresh(
                after_sequence=-1,
                not_before_ns=0,
                timeout_s=self.args.query_timeout_ms * 1e-3,
            )
            if feedback is None or not feedback.valid:
                raise RuntimeError("SG100 produced no valid startup feedback")
            arm_state = self.arm.joint_state()
            if arm_state is None or not arm_state.valid:
                raise RuntimeError("RM65 produced no valid startup feedback")
            self.arm_center_rad = arm_state.q_rad.copy()
            self._validate_motion_envelope()
        except BaseException:
            self.close()
            raise

    def _validate_motion_envelope(self) -> None:
        assert self.arm_center_rad is not None
        lower, upper = self.arm.joint_limits_rad()
        arm_delta = math.radians(self.args.arm_amplitude_deg)
        arm_soft_margin = math.radians(2.0)
        index = self.args.arm_joint - 1
        if not (
            lower[index] + arm_soft_margin
            <= self.arm_center_rad[index] - arm_delta
            and self.arm_center_rad[index] + arm_delta
            <= upper[index] - arm_soft_margin
        ):
            raise ValueError(
                "RM65 motion envelope exceeds the selected joint soft limit"
            )
        application = self.hand.command_limit_contract.apply(self.hand_center_rad)
        if application.canonical_clamped_indices or application.sdk_clamped_indices:
            raise ValueError("SG100 hand center is outside runtime command limits")
        hand_delta = math.radians(self.args.hand_amplitude_deg)
        hand_index = HAND_JOINT_NAMES.index(self.args.hand_joint)
        for sign in (-1.0, 1.0):
            candidate = self.hand_center_rad.copy()
            candidate[hand_index] += sign * hand_delta
            limited = self.hand.command_limit_contract.apply(candidate)
            if limited.canonical_clamped_indices or limited.sdk_clamped_indices:
                raise ValueError("SG100 motion envelope exceeds runtime command limits")

    def _wait_all_mode7(self, *, timeout_s: float = 2.0) -> None:
        deadline = time.monotonic() + timeout_s
        sequence = self.observer.latest_sequence
        while time.monotonic() < deadline:
            sample = self.observer.wait_fresh(
                after_sequence=sequence,
                not_before_ns=0,
                timeout_s=min(0.1, max(0.0, deadline - time.monotonic())),
            )
            if sample is None:
                continue
            sequence = sample.sequence
            if (
                sample.valid
                and sample.joint_modes == (MODE_JOINT_POSITION,) * HAND_DOF
            ):
                return
        raise RuntimeError("SG100 did not report fresh all-MODE7 feedback")

    def prepare_profile(self, profile: str) -> int:
        assert self.arm_center_rad is not None
        self._close_command_gate()
        moving = profile != PROFILE_STATIONARY
        self._active_profile = profile

        if moving:
            self.restricted_hand = SG100RestrictedAVPBackend(
                self.hand,
                joint_allowlist=(self.args.hand_joint,),
                hardware_validated_joints=HARDWARE_VALIDATED_MOTION_JOINTS,
                stale_tracking_s=CRITERIA.interruption_timeout_ms * 1e-3,
                feedback_stall_s=CRITERIA.interruption_timeout_ms * 1e-3,
            )
            anchored = self.restricted_hand.anchor(self.hand_center_rad)
            if not anchored.accepted:
                raise RuntimeError(f"SG100 anchor rejected: {anchored.reason}")
            self._wait_all_mode7()
            self.restricted_hand.set_enabled(True)
        else:
            self.restricted_hand = None
            sdk_target = self.hand.command_limit_contract.apply(
                self.hand_center_rad
            ).sdk_sent_deg
            submission = self.hand.send_mode7_zero_output_sdk_targets(sdk_target)
            if submission is None:
                raise RuntimeError("SG100 zero-output stationary command rejected")
            self._wait_all_mode7()

        # Arm last: its 100 ms stale-target watchdog must not expire while the
        # hand is still waiting for fresh all-MODE7 feedback.
        preparation_bundle = self.next_bundle_id()
        self.arm.send_joint_target(
            self.arm_center_rad,
            command_bundle_id=preparation_bundle,
        )
        self.arm.set_enabled(True)
        self._open_command_gate()
        return preparation_bundle

    def target_pair(self, elapsed_s: float) -> tuple[np.ndarray, np.ndarray, float]:
        assert self.arm_center_rad is not None
        phase = math.sin(2.0 * math.pi * self.args.motion_frequency_hz * elapsed_s)
        arm = self.arm_center_rad.copy()
        hand = self.hand_center_rad.copy()
        if self._active_profile != PROFILE_STATIONARY:
            arm[self.args.arm_joint - 1] += math.radians(
                self.args.arm_amplitude_deg
            ) * phase
            hand[HAND_JOINT_NAMES.index(self.args.hand_joint)] += math.radians(
                self.args.hand_amplitude_deg
            ) * phase
        return arm, hand, phase

    def send_bundle(
        self, *, elapsed_s: float, bundle_id: int
    ) -> tuple[int | None, str | None, float]:
        arm_target, hand_target, phase = self.target_pair(elapsed_s)
        if not self.command_gate_open:
            self.command_gate_rejections += 1
            return None, "command_gate_closed", phase
        if not self.arm.enabled:
            self._close_command_gate()
            self.command_gate_rejections += 1
            if self.restricted_hand is not None:
                self.restricted_hand.set_enabled(False)
            else:
                self.hand.hold_passive()
            return None, "rm65_disarmed_command_gate_closed", phase
        self.arm.send_joint_target(arm_target, command_bundle_id=bundle_id)
        if self._active_profile == PROFILE_STATIONARY:
            sdk_target = self.hand.command_limit_contract.apply(
                hand_target
            ).sdk_sent_deg
            submission = self.hand.send_mode7_zero_output_sdk_targets(sdk_target)
            if submission is None:
                self.safe_stop()
                return None, "SG100 stationary submission rejected", phase
            return int(submission.sent_monotonic_ns), None, phase
        assert self.restricted_hand is not None
        now_ns = time.monotonic_ns()
        result = self.restricted_hand.submit(
            hand_target,
            tracking_received_ns=now_ns,
            now_ns=now_ns,
            dt_s=1.0 / self.args.command_hz,
        )
        if not result.accepted:
            self.safe_stop()
            return None, result.reason, phase
        return int(result.sent_monotonic_ns), None, phase

    def settle_and_stop(self) -> None:
        if self.arm_center_rad is None:
            return
        deadline = time.monotonic() + self.args.settle_s
        next_tick = time.monotonic()
        while time.monotonic() < deadline and self.arm.enabled:
            bundle = self.next_bundle_id()
            self.arm.send_joint_target(self.arm_center_rad, command_bundle_id=bundle)
            if self.restricted_hand is not None and self.restricted_hand.enabled:
                now_ns = time.monotonic_ns()
                self.restricted_hand.submit(
                    self.hand_center_rad,
                    tracking_received_ns=now_ns,
                    now_ns=now_ns,
                    dt_s=1.0 / self.args.command_hz,
                )
            next_tick += 1.0 / self.args.command_hz
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
        self.safe_stop()

    def safe_stop(self) -> tuple[int, int]:
        commands_forbidden_ns = self._close_command_gate()
        self.arm.set_enabled(False)
        if self.restricted_hand is not None:
            self.restricted_hand.set_enabled(False)
        else:
            self.hand.hold_passive()
        completed_ns = time.monotonic_ns()
        self.arm.stop_smooth()
        return commands_forbidden_ns, completed_ns

    def close(self) -> None:
        self.observer.stop()
        try:
            self.arm.set_enabled(False)
            self.arm.stop_smooth()
        except Exception:  # noqa: BLE001 - best-effort cleanup
            pass
        try:
            self.hand.stop()
        except Exception:  # noqa: BLE001 - best-effort cleanup
            pass
        try:
            self.hand.disconnect()
        except Exception:  # noqa: BLE001 - best-effort cleanup
            pass
        try:
            self.arm.disconnect()
        except Exception:  # noqa: BLE001 - best-effort cleanup
            pass


class CommandPairCollector:
    def __init__(self, run_id: str) -> None:
        self.run_id = run_id
        self.pending: dict[int, dict[str, Any]] = {}
        self.rows: list[dict[str, Any]] = []
        self.raw_rows: list[dict[str, Any]] = []
        self.max_arm_source_age_ms = 0.0
        self.stale_arm_command_count = 0

    def observe_hand(
        self,
        bundle_id: int,
        hand_sent_ns: int | None,
        error: str | None,
    ) -> None:
        row = {
            "run_id": self.run_id,
            "sample_type": "command_sync",
            "bundle_id": bundle_id,
            "hand_sent_ns": hand_sent_ns,
            "arm_sent_ns": None,
            "skew_ms": None,
            "valid": False,
            "timeout": False,
            "error": error,
        }
        self.pending[bundle_id] = row
        self.raw_rows.append(
            {
                "run_id": self.run_id,
                "sample_type": "hand_command",
                "bundle_id": bundle_id,
                "hand_sent_ns": hand_sent_ns,
                "valid": hand_sent_ns is not None,
                "error": error,
            }
        )

    def drain_arm(self, rig: RealCommunicationRig) -> None:
        for event in rig.arm.drain_command_events():
            source_age_ms = (
                int(event.sent_monotonic_ns) - int(event.source_timestamp_ns)
            ) * 1e-6
            self.max_arm_source_age_ms = max(
                self.max_arm_source_age_ms, source_age_ms
            )
            if source_age_ms > CRITERIA.interruption_timeout_ms:
                self.stale_arm_command_count += 1
            bundle_id = event.command_bundle_id
            self.raw_rows.append(
                {
                    "run_id": self.run_id,
                    "sample_type": "arm_command",
                    "bundle_id": bundle_id,
                    "arm_sent_ns": int(event.sent_monotonic_ns),
                    "source_age_ms": source_age_ms,
                    "valid": True,
                }
            )
            if bundle_id is None or bundle_id not in self.pending:
                continue
            row = self.pending[bundle_id]
            if row["arm_sent_ns"] is not None:
                continue
            row["arm_sent_ns"] = int(event.sent_monotonic_ns)
            hand_ns = row["hand_sent_ns"]
            if hand_ns is not None:
                row["skew_ms"] = abs(int(event.sent_monotonic_ns) - int(hand_ns)) * 1e-6
                row["valid"] = True
                row["error"] = None

    def finish(self) -> None:
        for bundle_id in sorted(self.pending):
            row = self.pending[bundle_id]
            if not row["valid"] and row["error"] is None:
                row["timeout"] = True
                row["error"] = "unmatched_arm_or_hand_submission"
            self.rows.append(row)
            self.raw_rows.append(dict(row))


def run_command_phase(
    rig: RealCommunicationRig,
    *,
    duration_s: float,
    collector: CommandPairCollector | None,
    live_log: Any | None = None,
    visualizer: HeadlessVisualizationLoad | None = None,
) -> None:
    started = time.monotonic()
    deadline = started + float(duration_s)
    next_tick = started
    while time.monotonic() < deadline:
        elapsed_s = time.monotonic() - started
        bundle_id = rig.next_bundle_id()
        hand_sent_ns, error, phase = rig.send_bundle(
            elapsed_s=elapsed_s,
            bundle_id=bundle_id,
        )
        if collector is not None:
            collector.observe_hand(bundle_id, hand_sent_ns, error)
            collector.drain_arm(rig)
        if live_log is not None:
            live_log.write(json.dumps(
                {
                    "timestamp_ns": time.monotonic_ns(),
                    "bundle_id": bundle_id,
                    "phase": phase,
                    "hand_sent_ns": hand_sent_ns,
                    "error": error,
                },
                ensure_ascii=False,
            ) + "\n")
            live_log.flush()
        if visualizer is not None:
            visualizer.update(elapsed_s, phase * math.radians(rig.args.arm_amplitude_deg))
        next_tick += 1.0 / rig.args.command_hz
        delay = next_tick - time.monotonic()
        if delay > 0.0:
            time.sleep(delay)
        else:
            next_tick = time.monotonic()
    if collector is not None:
        flush_deadline = time.monotonic() + min(0.05, 1.0 / rig.args.command_hz)
        while time.monotonic() < flush_deadline:
            collector.drain_arm(rig)
            time.sleep(0.001)
        collector.finish()


def _query_rows_rm65(
    run_id: str, samples: Iterable[RM65QuerySample]
) -> list[dict[str, Any]]:
    rows = []
    for index, sample in enumerate(samples, 1):
        rows.append(
            {
                "run_id": run_id,
                "sample_type": "rm65_rtt",
                "sequence": index,
                "request_queued_ns": sample.request_queued_ns,
                "request_started_ns": sample.request_started_ns,
                "response_received_ns": sample.response_received_ns,
                "latency_ms": sample.latency_ms,
                "sdk_lock_wait_ms": sample.sdk_lock_wait_ms,
                "valid": sample.valid,
                "timeout": sample.timeout,
                "error": sample.error,
            }
        )
    return rows


def _query_rows_sg100(
    run_id: str, samples: Iterable[SG100QuerySample]
) -> list[dict[str, Any]]:
    return [
        {
            "run_id": run_id,
            "sample_type": "sg100_rtt_proxy",
            "sequence": sample.request_id,
            "request_started_ns": sample.request_started_ns,
            "request_accepted_ns": sample.request_accepted_ns,
            "response_received_ns": sample.response_received_ns,
            "response_sequence": sample.response_sequence,
            "latency_ms": sample.latency_ms,
            "valid": sample.valid,
            "timeout": sample.timeout,
            "error": sample.error,
        }
        for sample in samples
    ]


def _feedback_rows(
    run_id: str, samples: Iterable[ObservedHandFeedback]
) -> list[dict[str, Any]]:
    return [
        {
            "run_id": run_id,
            "sample_type": "sg100_feedback",
            "sequence": sample.sequence,
            "received_ns": sample.received_ns,
            "valid": sample.valid,
            "joint_modes": (
                None if sample.joint_modes is None else list(sample.joint_modes)
            ),
            "timeout": False,
            "error": None if sample.valid else "invalid_feedback",
        }
        for sample in samples
    ]


def feedback_anomaly_rows(
    run_id: str,
    samples: Sequence[ObservedHandFeedback],
    *,
    criteria: AcceptanceCriteria = CRITERIA,
) -> list[dict[str, Any]]:
    anomalies: list[dict[str, Any]] = []
    ordered = list(samples)
    for sample in ordered:
        if not sample.valid:
            anomalies.append({
                "run_id": run_id,
                "sample_type": "feedback_anomaly",
                "sequence": sample.sequence,
                "received_ns": sample.received_ns,
                "valid": False,
                "timeout": False,
                "error": "invalid_feedback",
            })
    for previous, current in zip(ordered, ordered[1:]):
        interval_ms = (current.received_ns - previous.received_ns) * 1e-6
        sequence_delta = current.sequence - previous.sequence
        reasons: list[str] = []
        if interval_ms <= 0.0:
            reasons.append("non_monotonic_feedback_timestamp")
        elif interval_ms > criteria.feedback_interval_p99_ms:
            reasons.append("feedback_interval_over_50_ms")
        if sequence_delta > 1:
            reasons.append("host_callback_sequence_gap")
        elif sequence_delta <= 0:
            reasons.append("non_increasing_callback_sequence")
        if reasons:
            anomalies.append({
                "run_id": run_id,
                "sample_type": "feedback_interval_anomaly",
                "sequence": current.sequence,
                "received_ns": current.received_ns,
                "interval_ms": interval_ms,
                "valid": False,
                "timeout": False,
                "error": ",".join(reasons),
                "previous_sequence": previous.sequence,
                "sequence_delta": sequence_delta,
            })
    return anomalies


ARM_FAILURE_EVENT_NAMES = frozenset({
    "disarm",
    "feedback_error",
    "limiter_error",
    "send_failure",
})


def run_real_load(
    rig: RealCommunicationRig,
    args: argparse.Namespace,
    *,
    profile: str,
    repeat: int,
    run_dir: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    run_id = f"{profile}-repeat-{repeat}"
    print(f"\n[RUN] {run_id}: warmup {args.warmup_s:.1f}s")
    rig.prepare_profile(profile)
    run_command_phase(rig, duration_s=args.warmup_s, collector=None)

    rig.arm.drain_latency_query_samples()
    rig.arm.drain_command_events()
    arm_event_index = len(rig.arm_events)
    diag_before = _counter_snapshot(rig.hand)
    started_ns = time.monotonic_ns()

    query_worker = SG100QueryWorker(
        rig.hand,
        rig.observer,
        query_hz=args.query_hz,
        timeout_s=args.query_timeout_ms * 1e-3,
        duration_s=args.duration_s,
    )
    collector = CommandPairCollector(run_id)
    visualizer = None
    live_log = None
    if profile == PROFILE_MOTION_LOG_VIS:
        visualizer = HeadlessVisualizationLoad(refresh_hz=args.visualization_hz)
        live_log = (run_dir / f"{run_id}-live-events.jsonl").open(
            "w", encoding="utf-8"
        )

    print(f"[RUN] {run_id}: collect {args.duration_s:.1f}s")
    query_worker.start()
    try:
        run_command_phase(
            rig,
            duration_s=args.duration_s,
            collector=collector,
            live_log=live_log,
            visualizer=visualizer,
        )
    finally:
        if live_log is not None:
            live_log.close()
        query_worker.stop()
        query_worker.join()
    ended_ns = time.monotonic_ns()
    diag_after = _counter_snapshot(rig.hand)
    rig.settle_and_stop()

    rm65_rows = _query_rows_rm65(
        run_id,
        (
            sample
            for sample in rig.arm.drain_latency_query_samples()
            if started_ns <= sample.request_started_ns <= ended_ns
        ),
    )
    sg100_rows = _query_rows_sg100(run_id, query_worker.samples)
    observed_feedback = rig.observer.samples_between(started_ns, ended_ns)
    feedback_rows = _feedback_rows(run_id, observed_feedback)
    feedback_anomalies = feedback_anomaly_rows(run_id, observed_feedback)
    crc_anomalies = [
        {
            "run_id": run_id,
            "sample_type": "crc_error",
            "received_ns": timestamp_ns,
            "valid": False,
            "timeout": False,
            "error": "sg100_crc_error",
        }
        for timestamp_ns in diag_after["crc_error_timestamps_ns"]
        if started_ns <= timestamp_ns <= ended_ns
    ]
    feedback_summary = evaluate_feedback(feedback_rows)
    rm65_summary = evaluate_rtt(rm65_rows)
    sg100_summary = evaluate_rtt(sg100_rows)
    sync_summary = evaluate_command_sync(collector.rows)
    actual_duration_s = (ended_ns - started_ns) * 1e-9
    coverage = evaluate_sample_coverage(
        configured_duration_s=args.duration_s,
        actual_duration_s=actual_duration_s,
        query_hz=args.query_hz,
        command_hz=args.command_hz,
        rm65_query_count=int(rm65_summary["total_requests"]),
        sg100_query_count=int(sg100_summary["total_requests"]),
        feedback_count=int(feedback_summary["valid_new_frames"]),
        command_bundle_count=int(sync_summary["total_bundles"]),
    )

    status_frames = counter_delta(
        diag_after["status_recv_count"], diag_before["status_recv_count"], bits=32
    )
    crc_errors = counter_delta(
        diag_after["crc_error_count"], diag_before["crc_error_count"]
    )
    device_errors = counter_delta(
        diag_after["error_frames"], diag_before["error_frames"]
    )
    observer_errors = rig.observer.errors_between(started_ns, ended_ns)
    observer_error_count = len(observer_errors)
    observer_anomalies = [
        {
            "run_id": run_id,
            "sample_type": "sg100_observer_error",
            "received_ns": int(row["timestamp_ns"]),
            "valid": False,
            "timeout": False,
            "error": str(row["error"]),
        }
        for row in observer_errors
    ]
    arm_failure_events = [
        event
        for event in rig.arm_events[arm_event_index:]
        if str(event.get("name")) in ARM_FAILURE_EVENT_NAMES
    ]
    arm_failure_rows = [
        {
            "run_id": run_id,
            "sample_type": "rm65_backend_error",
            "received_ns": int(event["timestamp_ns"]),
            "valid": False,
            "timeout": False,
            "error": f"{event['name']}:{event['detail']}",
            "event_name": str(event["name"]),
        }
        for event in arm_failure_events
    ]
    integrity = evaluate_integrity(
        valid_frames=int(feedback_summary["valid_new_frames"]),
        invalid_frames=int(feedback_summary["invalid_frames"]),
        status_frames=status_frames,
        crc_errors=crc_errors,
        request_count=(
            int(rm65_summary["total_requests"])
            + int(sg100_summary["total_requests"])
        ),
        request_timeouts=(
            int(rm65_summary["timeout_count"])
            + int(sg100_summary["timeout_count"])
        ),
        request_errors=(
            int(rm65_summary["error_count"])
            - int(rm65_summary["timeout_count"])
            + int(sg100_summary["error_count"])
            - int(sg100_summary["timeout_count"])
        ),
        estimated_sequence_gaps=int(
            feedback_summary["estimated_host_callback_sequence_gaps"]
        ),
        device_error_frames=device_errors,
        control_failure_events=(
            len(arm_failure_events) + observer_error_count
        ),
    )
    for device, summary in (
        ("rm65", rm65_summary),
        ("sg100", sg100_summary),
    ):
        total_requests = int(summary["total_requests"])
        failure_count = int(summary["error_count"])
        failure_rate = (
            failure_count / total_requests if total_requests else None
        )
        integrity[f"{device}_request_failure_rate"] = failure_rate
        key = f"{device}_request_failure_rate_le_0_1_percent"
        integrity["acceptance"][key] = bool(
            failure_rate is not None
            and failure_rate <= CRITERIA.max_error_rate
        )
    integrity["pass"] = all(integrity["acceptance"].values())

    raw_rows = (
        rm65_rows
        + sg100_rows
        + feedback_rows
        + collector.raw_rows
        + feedback_anomalies
        + crc_anomalies
        + observer_anomalies
        + arm_failure_rows
    )
    report = {
        "run_id": run_id,
        "profile": profile,
        "repeat": repeat,
        "started_monotonic_ns": started_ns,
        "ended_monotonic_ns": ended_ns,
        "warmup_s": args.warmup_s,
        "collection_s": actual_duration_s,
        "sample_coverage": coverage,
        "rm65_rtt": rm65_summary,
        "sg100_rtt_proxy": sg100_summary,
        "sg100_feedback_update": feedback_summary,
        "arm_hand_command_sync": sync_summary,
        "communication_integrity": integrity,
        "anomaly_records": {
            "feedback_interval_or_sequence": len(feedback_anomalies),
            "crc": len(crc_anomalies),
            "observer_exceptions": observer_error_count,
            "rm65_backend_failures": len(arm_failure_events),
        },
        "backlog": {
            "max_rm65_target_source_age_ms": collector.max_arm_source_age_ms,
            "stale_rm65_command_count": collector.stale_arm_command_count,
            "continuous_command_backlog": (
                collector.stale_arm_command_count > 0
            ),
        },
    }
    report["pass"] = bool(
        rm65_summary["pass"]
        and sg100_summary["pass"]
        and feedback_summary["pass"]
        and sync_summary["pass"]
        and integrity["pass"]
        and coverage["pass"]
        and not report["backlog"]["continuous_command_backlog"]
    )
    print(
        f"[RUN] {run_id}: {'PASS' if report['pass'] else 'FAIL'} "
        f"RM65 p95={rm65_summary['latency_ms']['p95']} ms, "
        f"SG100 proxy p95={sg100_summary['latency_ms']['p95']} ms, "
        f"sync p95={sync_summary['absolute_send_skew_ms']['p95']} ms"
    )
    rig.observer.discard_through(ended_ns)
    return report, raw_rows


def run_real_interruptions(
    rig: RealCommunicationRig,
    args: argparse.Namespace,
) -> list[dict[str, Any]]:
    """Exercise the consumer-side fail-closed timing with stationary targets.

    ``device_feedback`` deliberately suppresses feedback at the benchmark's
    consumer boundary.  It verifies detection/gating code and safe-stop calls,
    but not USB unplug detection; the report marks that qualification.
    """

    rows: list[dict[str, Any]] = []
    for scenario in ("upstream_target", "device_feedback"):
        for repeat in range(1, args.interruption_repeats + 1):
            rig.prepare_profile(PROFILE_STATIONARY)
            # Establish fresh traffic without moving either device.
            run_command_phase(rig, duration_s=0.25, collector=None)
            if not rig.command_gate_open or not rig.arm.enabled:
                raise RuntimeError(
                    f"{scenario} interruption precondition lost command enable"
                )
            if scenario == "upstream_target":
                last_valid_ns = rig.arm.latest_target_timestamp_ns()
                detector = "RM65 production stale-target watchdog"
                last_valid_kind = "last upstream target publication"
            else:
                feedback = rig.observer.latest_valid()
                last_valid_ns = (
                    None if feedback is None else int(feedback.received_ns)
                )
                detector = "benchmark SG100 feedback-consumer watchdog"
                last_valid_kind = "last consumed valid SG100 callback"
            if last_valid_ns is None:
                raise RuntimeError(
                    f"{scenario} interruption has no valid pre-fault timestamp"
                )

            fault_started_ns = time.monotonic_ns()
            timeout_ns = int(CRITERIA.interruption_timeout_ms * 1e6)
            hard_deadline_ns = last_valid_ns + timeout_ns + 50_000_000
            next_command_ns = fault_started_ns
            fault_detected = False
            while time.monotonic_ns() <= hard_deadline_ns:
                now_ns = time.monotonic_ns()
                if scenario == "upstream_target":
                    # No more targets are published. The real RM65 backend
                    # watchdog must disarm; this loop only observes its gate.
                    fault_detected = not rig.arm.enabled
                else:
                    # Freeze the consumer's last-valid timestamp while physical
                    # feedback continues. This safely exercises fail-closed
                    # timing without asking the operator to pull a cable.
                    fault_detected = now_ns - last_valid_ns >= timeout_ns
                if fault_detected:
                    break
                if scenario == "device_feedback" and now_ns >= next_command_ns:
                    bundle = rig.next_bundle_id()
                    rig.send_bundle(
                        elapsed_s=(now_ns - fault_started_ns) * 1e-9,
                        bundle_id=bundle,
                    )
                    next_command_ns += int(1e9 / args.command_hz)
                time.sleep(0.001)

            timeout_trigger_ns = time.monotonic_ns()
            commands_forbidden_ns, safe_stop_completed_ns = rig.safe_stop()

            # Report any SDK return already in flight when the gate closed,
            # separately from a fresh post-gate target submission.
            time.sleep(CRITERIA.command_block_deadline_ms * 1e-3)
            in_flight_events = rig.arm.drain_command_events()
            in_flight_after_gate = sum(
                int(event.sent_monotonic_ns) > commands_forbidden_ns
                for event in in_flight_events
            )

            # Attempt fresh publications after the gate. They must be rejected
            # before either hardware API is reached.
            gate_attempts = 3
            rejection_before = rig.command_gate_rejections
            hand_calls_before = _counter_snapshot(rig.hand)["hand_control_calls"]
            for _ in range(gate_attempts):
                bundle = rig.next_bundle_id()
                rig.send_bundle(elapsed_s=0.0, bundle_id=bundle)
            time.sleep(max(0.02, 2.0 / args.arm_send_hz))
            post_gate_events = rig.arm.drain_command_events()
            hand_calls_after = _counter_snapshot(rig.hand)["hand_control_calls"]
            post_gate_hand_calls = counter_delta(
                hand_calls_after, hand_calls_before
            )
            post_gate_arm_commands = len(post_gate_events)
            post_gate_rejections = (
                rig.command_gate_rejections - rejection_before
            )
            command_count_after_gate = (
                post_gate_arm_commands + post_gate_hand_calls
            )

            recovery_bundle = rig.prepare_profile(PROFILE_STATIONARY)
            recovery_deadline = time.monotonic() + 0.05
            recovery_events = []
            while time.monotonic() < recovery_deadline:
                recovery_events.extend(rig.arm.drain_command_events())
                if recovery_events:
                    break
                time.sleep(0.001)
            historical_replays = sum(
                event.command_bundle_id != recovery_bundle
                for event in recovery_events
            )
            rig.safe_stop()
            row = evaluate_interruption(
                {
                    "scenario": scenario,
                    "repeat": repeat,
                    "fault_detected": fault_detected,
                    "fault_started_ns": fault_started_ns,
                    "detector": detector,
                    "last_valid_kind": last_valid_kind,
                    "last_valid_ns": last_valid_ns,
                    "timeout_trigger_ns": timeout_trigger_ns,
                    "commands_forbidden_ns": commands_forbidden_ns,
                    "safe_stop_completed_ns": safe_stop_completed_ns,
                    "commands_after_gate": command_count_after_gate,
                    "post_gate_attempts": gate_attempts,
                    "post_gate_attempt_rejections": post_gate_rejections,
                    "post_gate_arm_commands": post_gate_arm_commands,
                    "post_gate_hand_control_calls": post_gate_hand_calls,
                    "in_flight_arm_returns_after_gate": in_flight_after_gate,
                    "historical_targets_replayed": historical_replays,
                    "recovery_bundle_id": recovery_bundle,
                    "recovery_kind": (
                        "fresh-target rearm; physical transport reconnect not exercised"
                    ),
                    "fault_injection": (
                        "producer publication stopped"
                        if scenario == "upstream_target"
                        else "software suppression at feedback-consumer boundary"
                    ),
                }
            )
            rows.append(row)
            print(
                f"[INTERRUPTION] {scenario} repeat {repeat}: "
                f"{'PASS' if row['pass'] else 'FAIL'} "
                f"detect={row['detection_ms']:.2f}ms "
                f"gate={row['command_block_after_trigger_ms']:.2f}ms"
            )
    return rows


def build_mock_run(profile: str, repeat: int, *, sample_count: int = 200) -> tuple[
    dict[str, Any], list[dict[str, Any]]
]:
    run_id = f"{profile}-repeat-{repeat}"
    base_ns = 10_000_000_000 * repeat
    rm65_rows: list[dict[str, Any]] = []
    sg100_rows: list[dict[str, Any]] = []
    feedback_rows: list[dict[str, Any]] = []
    sync_rows: list[dict[str, Any]] = []
    for index in range(sample_count):
        tick_ns = base_ns + index * 50_000_000
        rm_latency = 8.0 + (index % 5) * 0.2
        sg_latency = 11.0 + (index % 7) * 0.2
        skew = 3.0 + (index % 3) * 0.1
        rm65_rows.append({
            "run_id": run_id,
            "sample_type": "rm65_rtt",
            "sequence": index + 1,
            "request_started_ns": tick_ns,
            "response_received_ns": tick_ns + int(rm_latency * 1e6),
            "latency_ms": rm_latency,
            "valid": True,
            "timeout": False,
            "error": None,
        })
        sg100_rows.append({
            "run_id": run_id,
            "sample_type": "sg100_rtt_proxy",
            "sequence": index + 1,
            "request_started_ns": tick_ns,
            "request_accepted_ns": tick_ns + 100_000,
            "response_received_ns": tick_ns + int(sg_latency * 1e6),
            "latency_ms": sg_latency,
            "valid": True,
            "timeout": False,
            "error": None,
        })
        feedback_rows.append({
            "run_id": run_id,
            "sample_type": "sg100_feedback",
            "sequence": index + 1,
            "received_ns": tick_ns,
            "valid": True,
            "timeout": False,
            "error": None,
        })
        sync_rows.append({
            "run_id": run_id,
            "sample_type": "command_sync",
            "bundle_id": index + 1,
            "arm_sent_ns": tick_ns,
            "hand_sent_ns": tick_ns + int(skew * 1e6),
            "skew_ms": skew,
            "valid": True,
            "timeout": False,
            "error": None,
        })
    rm65 = evaluate_rtt(rm65_rows)
    sg100 = evaluate_rtt(sg100_rows)
    feedback = evaluate_feedback(feedback_rows)
    sync = evaluate_command_sync(sync_rows)
    integrity = evaluate_integrity(
        valid_frames=sample_count,
        invalid_frames=0,
        status_frames=sample_count,
        crc_errors=0,
        request_count=sample_count,
        request_timeouts=0,
        request_errors=0,
        estimated_sequence_gaps=0,
        device_error_frames=0,
        control_failure_events=0,
    )
    report = {
        "run_id": run_id,
        "profile": profile,
        "repeat": repeat,
        "rm65_rtt": rm65,
        "sg100_rtt_proxy": sg100,
        "sg100_feedback_update": feedback,
        "arm_hand_command_sync": sync,
        "communication_integrity": integrity,
        "backlog": {
            "max_rm65_target_source_age_ms": 4.0,
            "stale_rm65_command_count": 0,
            "continuous_command_backlog": False,
        },
        "pass": bool(
            rm65["pass"]
            and sg100["pass"]
            and feedback["pass"]
            and sync["pass"]
            and integrity["pass"]
        ),
    }
    return report, rm65_rows + sg100_rows + feedback_rows + sync_rows


def build_mock_interruptions(repeats: int) -> list[dict[str, Any]]:
    rows = []
    for scenario in ("upstream_target", "device_feedback"):
        for repeat in range(1, repeats + 1):
            last = repeat * 1_000_000_000
            rows.append(evaluate_interruption({
                "scenario": scenario,
                "repeat": repeat,
                "last_valid_ns": last,
                "timeout_trigger_ns": last + 100_500_000,
                "commands_forbidden_ns": last + 101_000_000,
                "safe_stop_completed_ns": last + 102_000_000,
                "commands_after_gate": 0,
                "historical_targets_replayed": 0,
                "fault_injection": "simulated",
            }))
    return rows


def interruption_raw_rows(
    rows: Sequence[dict[str, Any]],
) -> list[dict[str, Any]]:
    raw_rows = []
    for row in rows:
        failed_checks = [
            name
            for name, passed in row.get("acceptance", {}).items()
            if not bool(passed)
        ]
        raw_rows.append({
            "run_id": (
                f"interruption-{row.get('scenario', 'unknown')}-"
                f"repeat-{row.get('repeat', 'unknown')}"
            ),
            "sample_type": "interruption",
            "sequence": row.get("repeat"),
            "received_ns": row.get("timeout_trigger_ns"),
            "latency_ms": row.get("detection_ms"),
            "valid": bool(row.get("pass")),
            "timeout": not bool(row.get("fault_detected", True)),
            "error": None if not failed_checks else ",".join(failed_checks),
            "interruption_details": dict(row),
        })
    return raw_rows


RAW_CSV_FIELDS = (
    "run_id",
    "sample_type",
    "sequence",
    "bundle_id",
    "request_queued_ns",
    "request_started_ns",
    "request_accepted_ns",
    "response_received_ns",
    "response_sequence",
    "received_ns",
    "arm_sent_ns",
    "hand_sent_ns",
    "latency_ms",
    "sdk_lock_wait_ms",
    "skew_ms",
    "source_age_ms",
    "valid",
    "timeout",
    "error",
    "details_json",
)


def _serialize_raw_row(row: dict[str, Any]) -> dict[str, Any]:
    extras = {
        key: value for key, value in row.items() if key not in RAW_CSV_FIELDS
    }
    serialized = {key: row.get(key) for key in RAW_CSV_FIELDS}
    serialized["details_json"] = (
        json.dumps(extras, ensure_ascii=False, sort_keys=True)
        if extras
        else ""
    )
    return serialized


def _write_raw_csv(
    path: Path,
    rows: Sequence[dict[str, Any]],
    *,
    append: bool,
) -> None:
    needs_header = not append or not path.exists() or path.stat().st_size == 0
    mode = "a" if append else "w"
    with path.open(mode, encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=RAW_CSV_FIELDS)
        if needs_header:
            writer.writeheader()
        writer.writerows(_serialize_raw_row(row) for row in rows)


def write_raw_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    _write_raw_csv(path, rows, append=False)


def append_raw_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    _write_raw_csv(path, rows, append=True)


def _write_anomalies_jsonl(
    path: Path,
    rows: Sequence[dict[str, Any]],
    *,
    append: bool,
) -> None:
    with path.open("a" if append else "w", encoding="utf-8") as stream:
        for row in rows:
            if (
                not bool(row.get("valid", True))
                or row.get("sample_type") in {
                    "crc_error",
                    "feedback_anomaly",
                    "feedback_interval_anomaly",
                }
            ):
                stream.write(
                    json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n"
                )


def write_anomalies_jsonl(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    _write_anomalies_jsonl(path, rows, append=False)


def append_anomalies_jsonl(
    path: Path, rows: Sequence[dict[str, Any]]
) -> None:
    _write_anomalies_jsonl(path, rows, append=True)


def _profiles_from_args(values: Sequence[str]) -> tuple[str, ...]:
    if "all" in values:
        if len(values) != 1:
            raise ValueError("--profiles all cannot be combined with other profiles")
        return REQUIRED_PROFILES
    return tuple(dict.fromkeys(values))


def protocol_completeness(args: argparse.Namespace) -> dict[str, Any]:
    profiles_complete = set(args.profiles) == set(REQUIRED_PROFILES)
    return {
        "all_three_load_profiles": profiles_complete,
        "at_least_three_repeats_per_profile": args.repeats >= 3,
        "warmup_at_least_30_s": args.warmup_s >= 30.0,
        "collection_at_least_300_s": args.duration_s >= 300.0,
        "five_interruption_repeats_each": (
            not args.skip_interruptions and args.interruption_repeats >= 5
        ),
        "real_hardware": bool(args.execute),
    }


def assemble_report(
    args: argparse.Namespace,
    run_reports: Sequence[dict[str, Any]],
    interruptions: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    completeness = protocol_completeness(args)
    every_run_passed = bool(run_reports) and all(
        bool(run["pass"]) for run in run_reports
    )
    interruption_failure = bool(interruptions) and not all(
        bool(row["pass"]) for row in interruptions
    )
    interruption_missing = not args.skip_interruptions and not interruptions
    if not every_run_passed or interruption_failure or interruption_missing:
        overall = "FAIL"
    elif not all(completeness.values()):
        overall = "SCREENING_PASS_NOT_FULL_ACCEPTANCE"
    else:
        overall = "PASS_WITH_MEASUREMENT_QUALIFICATIONS"
    return {
        "schema_name": "rm65_sg100_communication_latency",
        "schema_version": "1.0",
        "created_local_time": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "source_requirement": (
            "臂手联合测试和通讯延迟输出及验收标准.xlsx / 通讯延迟测试"
        ),
        "criteria_status": "provisional; workbook cells marked 待确认 remain provisional",
        "acceptance_criteria": dataclasses.asdict(CRITERIA),
        "measurement_semantics": {
            "clock": "time.monotonic_ns on one host",
            "rm65_rtt": (
                "SDK-call boundary after production backend lock to immediately "
                "after rm_get_current_arm_state returns; lock wait is separate"
            ),
            "sg100_rtt": (
                "RequestHandStatus enqueue start to first fresh valid callback; "
                "public SDK exposes neither wire-send time nor transaction id, "
                "so a periodic callback may satisfy the wait and this proxy can "
                "differ in either direction from wire RTT"
            ),
            "command_sync": (
                "RM65 rm_movej_canfd successful return vs SG100 SetHandControl "
                "cache acceptance; not physical action synchronization"
            ),
            "device_loss": (
                "SG100 has host callback sequence only; reported loss is an estimate"
            ),
            "interruption": (
                "automated feedback loss is injected at the consumer boundary; "
                "physical cable/power interruption still needs a supervised test"
            ),
            "visualization_load": (
                "Matplotlib Agg figure rendering plus per-command flushed JSONL; "
                "this reproduces logging/render load without a production GUI"
            ),
        },
        "config": {
            "mode": "real" if args.execute else "simulation",
            "profiles": list(args.profiles),
            "repeats": args.repeats,
            "warmup_s": args.warmup_s,
            "duration_s": args.duration_s,
            "query_hz": args.query_hz,
            "command_hz": args.command_hz,
            "interruption_repeats": args.interruption_repeats,
        },
        "protocol_completeness": completeness,
        "runs": list(run_reports),
        "interruption_tests": list(interruptions),
        "summary": {
            "run_count": len(run_reports),
            "passed_run_count": sum(bool(run["pass"]) for run in run_reports),
            "failed_run_ids": [
                run["run_id"] for run in run_reports if not bool(run["pass"])
            ],
            "interruption_count": len(interruptions),
            "passed_interruption_count": sum(
                bool(row["pass"]) for row in interruptions
            ),
            "each_run_judged_independently": True,
            "combined_average_used_for_acceptance": False,
        },
        "overall_result": overall,
    }


def create_run_dir(output_dir: str) -> Path:
    root = Path(output_dir).expanduser().resolve()
    run_dir = root / time.strftime("%Y%m%d_%H%M%S")
    suffix = 1
    candidate = run_dir
    while candidate.exists():
        candidate = Path(f"{run_dir}_{suffix:02d}")
        suffix += 1
    candidate.mkdir(parents=True)
    return candidate


def print_plan(args: argparse.Namespace) -> None:
    print("RM65 + SG100 communication latency benchmark — DRY RUN")
    print("profiles                 :", ", ".join(args.profiles))
    print("repeats/profile          :", args.repeats)
    print("warmup / collection      :", f"{args.warmup_s:g}s / {args.duration_s:g}s")
    print("query / command rate     :", f"{args.query_hz:g}Hz / {args.command_hz:g}Hz")
    print("interruption repetitions :", (
        "SKIPPED" if args.skip_interruptions else args.interruption_repeats
    ))
    print("RM65                    :", f"{args.rm65_ip}:{args.rm65_port}")
    print("SG100                   :", f"{args.sg100_port} @ {args.sg100_baudrate}")
    print("output root             :", args.output_dir)
    print("hardware opened         : NO")
    print("commands sent           : NO")
    if any(profile != PROFILE_STATIONARY for profile in args.profiles):
        print("motion authorization     : --allow-motion + typed confirmation required")
        print("hand center config       :", args.initial_pose_config or "MISSING")
    print("Use --simulate for an offline pipeline check or --execute on hardware.")


def _confirm_motion(args: argparse.Namespace) -> None:
    moving = any(profile != PROFILE_STATIONARY for profile in args.profiles)
    if not moving:
        return
    if not args.allow_motion:
        raise ValueError("motion profiles require --allow-motion")
    if args.hand_center_deg is None:
        raise ValueError(
            "motion profiles require --initial-pose-config with all 11 hand angles"
        )
    print("\nMOTION TEST SAFETY CHECK")
    print("  RM65 joint             :", args.arm_joint)
    print("  RM65 amplitude         :", f"±{args.arm_amplitude_deg:g} deg")
    print("  SG100 active joint     :", args.hand_joint)
    print("  SG100 amplitude        :", f"±{args.hand_amplitude_deg:g} deg")
    print("  Physical e-stop must be staffed and the workspace must be clear.")
    if args.auto_confirm_motion:
        return
    answer = input("Type RUN MOTION TEST to continue: ").strip()
    if answer != "RUN MOTION TEST":
        raise RuntimeError("motion test cancelled")


def run(args: argparse.Namespace) -> tuple[int, Path]:
    if args.execute:
        _confirm_motion(args)
    run_dir = create_run_dir(args.output_dir)
    raw_path = run_dir / "raw_samples.csv"
    anomalies_path = run_dir / "anomalies.jsonl"
    report_path = run_dir / "report.json"
    write_raw_csv(raw_path, [])
    write_anomalies_jsonl(anomalies_path, [])

    def persist_rows(rows: Sequence[dict[str, Any]]) -> None:
        append_raw_csv(raw_path, rows)
        append_anomalies_jsonl(anomalies_path, rows)

    run_reports: list[dict[str, Any]] = []
    interruptions: list[dict[str, Any]] = []
    fatal_error: dict[str, Any] | None = None
    fatal_code = 0
    aborted_after_failed_run: str | None = None
    rig: RealCommunicationRig | None = None
    try:
        if args.simulate:
            for profile in args.profiles:
                for repeat in range(1, args.repeats + 1):
                    report, rows = build_mock_run(profile, repeat)
                    run_reports.append(report)
                    persist_rows(rows)
            if not args.skip_interruptions:
                interruptions = build_mock_interruptions(
                    args.interruption_repeats
                )
        else:
            rig = RealCommunicationRig(args)
            print("[CONNECT] opening RM65 and SG100 links")
            rig.connect()
            for profile in args.profiles:
                for repeat in range(1, args.repeats + 1):
                    report, rows = run_real_load(
                        rig,
                        args,
                        profile=profile,
                        repeat=repeat,
                        run_dir=run_dir,
                    )
                    run_reports.append(report)
                    persist_rows(rows)
                    if not report["pass"]:
                        aborted_after_failed_run = str(report["run_id"])
                        print(
                            "[STOP] failed run; remaining hardware profiles "
                            "will not be re-armed"
                        )
                        break
                if aborted_after_failed_run is not None:
                    break
            if (
                aborted_after_failed_run is None
                and not args.skip_interruptions
            ):
                interruptions = run_real_interruptions(rig, args)
    except KeyboardInterrupt as exc:
        fatal_code = 130
        fatal_error = {
            "type": type(exc).__name__,
            "message": "operator interrupted the benchmark",
            "traceback": traceback.format_exc(),
        }
    except Exception as exc:  # noqa: BLE001 - persist hardware failures
        fatal_code = 1
        fatal_error = {
            "type": type(exc).__name__,
            "message": str(exc),
            "traceback": traceback.format_exc(),
        }
    finally:
        if rig is not None:
            rig.close()
            print("[DISCONNECT] both device links closed; SG100 is MODE_NULL")

    persist_rows(interruption_raw_rows(interruptions))
    report = assemble_report(args, run_reports, interruptions)
    report["aborted_after_failed_run"] = aborted_after_failed_run
    if fatal_error is not None:
        report["fatal_error"] = fatal_error
        report["overall_result"] = "FAIL"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print("\n[RESULT]", report["overall_result"])
    print("[OUTPUT]", report_path)
    print("[OUTPUT]", raw_path)
    print("[OUTPUT]", anomalies_path)
    if fatal_code:
        return fatal_code, run_dir
    return (0 if not str(report["overall_result"]).startswith("FAIL") else 1), run_dir


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--simulate",
        action="store_true",
        help="generate a deterministic offline report; opens no hardware",
    )
    mode.add_argument(
        "--execute",
        action="store_true",
        help="connect to real RM65 and SG100 hardware",
    )
    parser.add_argument(
        "--profiles",
        nargs="+",
        default=["all"],
        choices=("all",) + REQUIRED_PROFILES,
    )
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--warmup-s", type=float, default=30.0)
    parser.add_argument("--duration-s", type=float, default=300.0)
    parser.add_argument("--query-hz", type=float, default=20.0)
    parser.add_argument("--command-hz", type=float, default=20.0)
    parser.add_argument("--query-timeout-ms", type=float, default=100.0)
    parser.add_argument("--rm65-ip", default="192.168.1.18")
    parser.add_argument("--rm65-port", type=int, default=8080)
    parser.add_argument("--arm-send-hz", type=float, default=100.0)
    parser.add_argument("--arm-joint", type=int, default=1)
    parser.add_argument("--arm-amplitude-deg", type=float, default=1.0)
    parser.add_argument("--arm-max-velocity-rad-s", type=float, default=0.2)
    parser.add_argument("--arm-max-acceleration-rad-s2", type=float, default=0.5)
    parser.add_argument("--sg100-port", default="/dev/ttyUSB0")
    parser.add_argument("--sg100-baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--sg100-sdk-dir", default=str(DEFAULT_SDK_DIR))
    parser.add_argument(
        "--hand-joint",
        choices=HARDWARE_VALIDATED_MOTION_JOINTS,
        default="index_j3",
        help="real motion is restricted to the currently validated index_j3/J6",
    )
    parser.add_argument("--hand-amplitude-deg", type=float, default=0.5)
    parser.add_argument(
        "--initial-pose-config",
        help="existing initial-pose YAML; hand.angles_deg defines motion center",
    )
    parser.add_argument("--motion-frequency-hz", type=float, default=0.1)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--visualization-hz", type=float, default=10.0)
    parser.add_argument(
        "--allow-motion",
        action="store_true",
        help="required for either motion load profile",
    )
    parser.add_argument(
        "--auto-confirm-motion",
        action="store_true",
        help="skip typed motion confirmation; supervised automation only",
    )
    parser.add_argument("--interruption-repeats", type=int, default=5)
    parser.add_argument("--skip-interruptions", action="store_true")
    parser.add_argument(
        "--output-dir",
        default="recording/communication_latency",
    )
    args = parser.parse_args(argv)
    try:
        args.profiles = _profiles_from_args(args.profiles)
    except ValueError as exc:
        parser.error(str(exc))
    positive = {
        "--repeats": args.repeats,
        "--warmup-s": args.warmup_s,
        "--duration-s": args.duration_s,
        "--query-hz": args.query_hz,
        "--command-hz": args.command_hz,
        "--query-timeout-ms": args.query_timeout_ms,
        "--arm-send-hz": args.arm_send_hz,
        "--arm-amplitude-deg": args.arm_amplitude_deg,
        "--arm-max-velocity-rad-s": args.arm_max_velocity_rad_s,
        "--arm-max-acceleration-rad-s2": args.arm_max_acceleration_rad_s2,
        "--sg100-baudrate": args.sg100_baudrate,
        "--hand-amplitude-deg": args.hand_amplitude_deg,
        "--motion-frequency-hz": args.motion_frequency_hz,
        "--settle-s": args.settle_s,
        "--visualization-hz": args.visualization_hz,
        "--interruption-repeats": args.interruption_repeats,
    }
    for name, value in positive.items():
        if not math.isfinite(float(value)) or float(value) <= 0.0:
            parser.error(f"{name} must be finite and > 0")
    if not 1 <= args.arm_joint <= 6:
        parser.error("--arm-joint must be in 1..6")
    if args.arm_amplitude_deg > 3.0:
        parser.error("--arm-amplitude-deg must be <= 3 for this benchmark")
    if args.hand_amplitude_deg > 0.5:
        parser.error("--hand-amplitude-deg must be <= 0.5 for this benchmark")
    if args.motion_frequency_hz > 0.5:
        parser.error("--motion-frequency-hz must be <= 0.5")
    if args.arm_max_velocity_rad_s > 0.5:
        parser.error("--arm-max-velocity-rad-s must be <= 0.5")
    if args.arm_max_acceleration_rad_s2 > 2.0:
        parser.error("--arm-max-acceleration-rad-s2 must be <= 2.0")
    if args.query_hz > 50.0:
        parser.error("--query-hz must be <= 50")
    if args.command_hz > 100.0:
        parser.error("--command-hz must be <= 100")
    if args.arm_send_hz > 200.0:
        parser.error("--arm-send-hz must be <= 200")
    if args.visualization_hz > 30.0:
        parser.error("--visualization-hz must be <= 30")
    if not 1 <= args.rm65_port <= 65535:
        parser.error("--rm65-port must be in 1..65535")
    if not str(args.rm65_ip).strip():
        parser.error("--rm65-ip must not be empty")
    if not str(args.sg100_port).strip():
        parser.error("--sg100-port must not be empty")
    if not str(args.output_dir).strip():
        parser.error("--output-dir must not be empty")
    if args.auto_confirm_motion and not args.execute:
        parser.error("--auto-confirm-motion requires --execute")
    if args.auto_confirm_motion and not args.allow_motion:
        parser.error("--auto-confirm-motion requires --allow-motion")
    if args.query_timeout_ms < CRITERIA.rtt_max_ms:
        parser.error(
            f"--query-timeout-ms must be >= {CRITERIA.rtt_max_ms:g} so the "
            "acceptance maximum can be observed"
        )
    try:
        args.hand_center_deg = load_hand_center_deg(args.initial_pose_config)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.execute and not args.simulate:
        print_plan(args)
        return 0
    try:
        code, _ = run(args)
        return code
    except KeyboardInterrupt:
        print("\n[ABORT] interrupted; cleanup requested")
        return 130
    except Exception as exc:  # noqa: BLE001 - CLI diagnostic surface
        print(f"[FAIL] {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
