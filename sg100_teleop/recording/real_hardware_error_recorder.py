"""Durable structured error log for real-hardware teleoperation runs."""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
from datetime import datetime
from enum import Enum
import json
import math
import os
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
    ) -> None:
        self.session_dir = Path(session_dir)
        self.path = self.session_dir / REAL_HARDWARE_ERROR_LOG
        self._start_monotonic_ns = time.monotonic_ns()
        self._sequence = 0
        self._severity_counts: dict[str, int] = {}
        self._lock = threading.Lock()
        # Keep the exception objects alive for this short-lived run so Python
        # cannot recycle an id and accidentally suppress a later exception.
        self._logged_exceptions: dict[int, BaseException] = {}
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
            line = json.dumps(
                payload,
                ensure_ascii=False,
                sort_keys=True,
                allow_nan=False,
            )
            self.session_dir.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(line + "\n")
                handle.flush()
                if severity_name == "ERROR":
                    os.fsync(handle.fileno())

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
    ) -> None:
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
