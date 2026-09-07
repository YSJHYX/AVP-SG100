"""Structured real-hardware error-log regressions."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from sg100_teleop.recording.real_hardware_error_recorder import (
    REAL_HARDWARE_ERROR_LOG,
    RealHardwareErrorRecorder,
)


def _records(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_error_log_is_created_and_serializes_diagnostics(tmp_path):
    recorder = RealHardwareErrorRecorder(
        tmp_path,
        context={"hand_backend": "real"},
    )
    recorder.record(
        severity="ERROR",
        component="sg100",
        event="COMMAND_REJECTED",
        message="STALE_AVP_TRACKING",
        details={
            "q": np.array([0.0, 0.25]),
            "path": tmp_path,
            "nonfinite": float("nan"),
        },
    )
    recorder.finish(outcome="interrupted", details={"frames": 12})

    records = _records(tmp_path / REAL_HARDWARE_ERROR_LOG)
    assert [record["event"] for record in records] == [
        "REAL_HARDWARE_SESSION_STARTED",
        "COMMAND_REJECTED",
        "REAL_HARDWARE_SESSION_FINISHED",
    ]
    assert records[1]["details"]["q"] == [0.0, 0.25]
    assert records[1]["details"]["nonfinite"] == "nan"
    assert records[2]["details"]["severity_counts_before_finish"]["ERROR"] == 1


def test_exception_includes_traceback_and_is_deduplicated(tmp_path):
    recorder = RealHardwareErrorRecorder(tmp_path)
    try:
        raise RuntimeError("transport failed")
    except RuntimeError as exc:
        recorder.record_exception(
            component="sg100",
            event="UNHANDLED_EXCEPTION",
            exception=exc,
        )
        recorder.record_exception(
            component="sg100",
            event="UNHANDLED_EXCEPTION",
            exception=exc,
        )

    records = _records(tmp_path / REAL_HARDWARE_ERROR_LOG)
    errors = [record for record in records if record["severity"] == "ERROR"]
    assert len(errors) == 1
    assert errors[0]["details"]["exception_type"] == "RuntimeError"
    assert "raise RuntimeError" in errors[0]["details"]["traceback"]
