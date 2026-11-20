"""Durable structured error log for real-hardware teleoperation runs."""

from __future__ import annotations

import atexit
from collections import OrderedDict
from dataclasses import asdict, is_dataclass
from datetime import datetime
from enum import Enum
import json
import math
import os
import queue
from pathlib import Path
import threading
import time
import traceback
from typing import Any

import numpy as np


REAL_HARDWARE_ERROR_LOG = "real_hardware_errors.jsonl"


def _json_safe(value: Any) -> Any:
    """Convert diagnostic values to strict, human-readable JSON values."""

    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Enum):
        return _json_safe(value.value)
    if isinstance(value, BaseException):
        return {
            "type": type(value).__name__,
            "message": str(value),
            "repr": repr(value),
        }
    if is_dataclass(value) and not isinstance(value, type):
        return _json_safe(asdict(value))
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json_safe(item) for item in value]
    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError):
        return repr(value)[:8000]
    return value


class RealHardwareErrorRecorder:
    """Append-only JSONL recorder whose error entries are flushed immediately."""

    schema_version = 1

    def __init__(
        self,
        session_dir: str | Path,
        *,
        context: dict[str, Any] | None = None,
        background: bool = False,
        queue_size: int = 1024,
    ) -> None:
        self.session_dir = Path(session_dir)
        self.path = self.session_dir / REAL_HARDWARE_ERROR_LOG
        self._start_monotonic_ns = time.monotonic_ns()
        self._sequence = 0
        self._severity_counts: dict[str, int] = {}
        self._lock = threading.Lock()
        # Keep a bounded identity cache; retaining every traceback leaks runs.
        self._logged_exceptions = OrderedDict()
        self.dropped_records = 0
        self.last_error = ""
        if queue_size < 1:
            raise ValueError("queue_size must be positive")
        self._queue = queue.Queue(maxsize=queue_size) if background else None
        self._closing = threading.Event()
        self._writer = None
        if background:
            self._writer = threading.Thread(target=self._write_loop, daemon=True,
                                            name="hardware-error-log")
            self._writer.start()
            atexit.register(self.close)
        self.record(
            severity="INFO",
            component="runtime",
            event="REAL_HARDWARE_SESSION_STARTED",
            message="Real-hardware diagnostic logging started",
            details={"context": context or {}},
        )

    @property
    def severity_counts(self) -> dict[str, int]:
        with self._lock:
            return dict(self._severity_counts)

    def record(
        self,
        *,
        severity: str,
        component: str,
        event: str,
        message: str,
        details: dict[str, Any] | None = None,
        include_stack: bool = False,
    ) -> None:
        severity_name = str(severity).upper()
        now_ns = time.monotonic_ns()
        with self._lock:
            self._sequence += 1
            self._severity_counts[severity_name] = (
                self._severity_counts.get(severity_name, 0) + 1
            )
            payload = {
                "schema_version": self.schema_version,
                "sequence": self._sequence,
                "wall_time": datetime.now().astimezone().isoformat(),
                "monotonic_ns": now_ns,
                "elapsed_s": (now_ns - self._start_monotonic_ns) * 1e-9,
                "severity": severity_name,
                "component": str(component),
                "event": str(event),
                "message": str(message),
                "details": _json_safe(details or {}),
            }
            if include_stack:
                payload["call_stack"] = traceback.format_stack(limit=16)[:-1]
            if self._queue is not None:
                if self._closing.is_set():
                    self.dropped_records += 1
                    return
                try:
                    self._queue.put_nowait(payload)
                except queue.Full:
                    self.dropped_records += 1
                return
            self._write_payload(payload)

    def _write_payload(self, payload):
        line = json.dumps(payload, ensure_ascii=False, sort_keys=True, allow_nan=False)
        self.session_dir.mkdir(parents=True, exist_ok=True)
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(line + "\n")
            handle.flush()
            if payload["severity"] == "ERROR":
                os.fsync(handle.fileno())

    def _write_loop(self):
        while not self._closing.is_set() or not self._queue.empty():
            try:
                payload = self._queue.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                self._write_payload(payload)
            except Exception as exc:
                with self._lock:
                    self.last_error = repr(exc)
                    self.dropped_records += 1
            finally:
                self._queue.task_done()
        if self.dropped_records or self.last_error:
            try:
                self._write_payload({
                    "severity": "WARNING", "component": "diagnostic_logger",
                    "event": "DIAGNOSTIC_LOG_INCOMPLETE",
                    "dropped_records": self.dropped_records, "last_error": self.last_error,
                })
            except Exception as exc:
                self.last_error = repr(exc)

    def record_exception(
        self,
        *,
        component: str,
        event: str,
        exception: BaseException,
        details: dict[str, Any] | None = None,
        traceback_object=None,
    ) -> None:
        exception_id = id(exception)
        with self._lock:
            if self._logged_exceptions.get(exception_id) is exception:
                return
            self._logged_exceptions[exception_id] = exception
            if len(self._logged_exceptions) > 64:
                self._logged_exceptions.popitem(last=False)
        tb = exception.__traceback__ if traceback_object is None else traceback_object
        merged = dict(details or {})
        merged.update(
            {
                "exception_type": type(exception).__name__,
                "exception_message": str(exception),
                "exception_repr": repr(exception),
                "traceback": "".join(
                    traceback.format_exception(type(exception), exception, tb)
                ),
            }
        )
        self.record(
            severity="ERROR",
            component=component,
            event=event,
            message=f"{type(exception).__name__}: {exception}",
            details=merged,
        )

    def finish(
        self,
        *,
        outcome: str,
        details: dict[str, Any] | None = None,
        timeout_s: float = 2.0,
    ) -> bool:
        self.record(
            severity="INFO",
            component="runtime",
            event="REAL_HARDWARE_SESSION_FINISHED",
            message=f"Real-hardware session finished: {outcome}",
            details={
                "outcome": outcome,
                "severity_counts_before_finish": self.severity_counts,
                **(details or {}),
            },
        )

        return self.close(timeout_s=timeout_s)

    def close(self, timeout_s: float = 2.0) -> bool:
        """Bounded drain, also used at process exit after startup exceptions."""
        if self._writer is not None:
            self._closing.set()
            self._writer.join(timeout=max(0.0, timeout_s))
            if self._writer.is_alive():
                return False
            atexit.unregister(self.close)
        return not self.last_error and not self.dropped_records
