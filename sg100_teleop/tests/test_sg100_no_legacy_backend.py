"""Sentinel: the old-SDK raw backend must not come back.

The 2026-09-03 migration to the V1.5_NO_TACTILE SDK removed the entire
CANFD-passive + custom RS485 raw-protocol workaround stack.  This test
fails if any of its production identifiers are reintroduced in the
backend modules, or if any wrapper export revives the legacy surface.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
ROBOT_DIR = ROOT / "sg100_teleop" / "robot"
WRAPPER_CPP = ROBOT_DIR / "sg100_c_wrapper.cpp"
DRIVER_PY = ROBOT_DIR / "sg100_sdk_driver.py"

# Identifiers that existed only in the removed raw-protocol backend.
LEGACY_IDENTIFIERS = [
    "RS485StreamParser",
    "AuditedRS485Transport",
    "crc32_bzip2",
    "append_crc32_le",
    "WireBootstrapAudit",
    "seed_realtime_hold",
    "safe_switch_physical_palm_to_realtime",
    "start_safe_realtime_hold",
    "stop_safe_realtime_hold",
    "update_realtime_target",
    "wire_bootstrap_snapshot",
    "snapshot_first_realtime_tx",
    "crc_diagnostics",
    "request_palm_runtime_mode",
    "verify_palm_realtime",
    "palm_mode_query_diagnostics",
    "SAFE_SWITCH_RESULT_MESSAGES",
    "SG100FirstRealtimeTxSnapshot",
    "SG100RealtimeBootstrapAudit",
    "InitForCANFD",
    "PassiveInjectRx",
    "PassiveGetTxBytes",
    "CANFDProcessRxFrame",
    "CANFDGetTxFrame",
    "VPT_H0_PASSIVE",
    "motion_commands_during_seed",
]

# Legacy wrapper exports (extern "C") that must not be rebuilt.
LEGACY_EXPORTS = [
    "sg100_c_connect",
    "sg100_c_connect_mode",
    "sg100_c_disconnect",
    "sg100_c_send_positions",
    "sg100_c_request_status",
    "sg100_request_palm_runtime_mode",
    "sg100_verify_palm_realtime",
    "sg100_safe_switch_physical_palm_to_realtime",
    "sg100_seed_realtime_cache",
    "sg100_start_safe_realtime_hold",
    "sg100_stop_safe_realtime_hold",
    "sg100_update_realtime_target",
    "sg100_get_bootstrap_status_wire_frame",
    "sg100_get_first_crc_failure_raw",
]


def test_backend_sources_free_of_legacy_identifiers() -> None:
    for path in (WRAPPER_CPP, DRIVER_PY):
        source = path.read_text()
        for identifier in LEGACY_IDENTIFIERS:
            assert identifier not in source, (
                f"legacy identifier {identifier!r} reappeared in {path.name}"
            )


def test_wrapper_source_exports_only_new_surface() -> None:
    source = WRAPPER_CPP.read_text()
    for export in LEGACY_EXPORTS:
        assert f" {export}(" not in source, (
            f"legacy export {export!r} reappeared in the C wrapper"
        )


def test_no_python_module_imports_legacy_driver_symbols() -> None:
    # Production code only: tests may reference legacy names in negative
    # assertions (that is the sentinel's own job).
    production_dirs = [
        ROOT / "sg100_teleop" / "robot",
        ROOT / "sg100_teleop" / "sim",
        ROOT / "sg100_teleop" / "tools",
        ROOT / "sg100_teleop" / "data_collection",
    ]
    result = subprocess.run(
        [
            "grep",
            "-rn",
            "-E",
            "seed_realtime_hold|safe_switch_physical_palm|"
            "start_safe_realtime_hold|wire_bootstrap_snapshot|"
            "snapshot_first_realtime_tx|crc_diagnostics",
            "--include=*.py",
            *[str(path) for path in production_dirs],
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 1, (
        "legacy backend references remain:\n" + result.stdout
    )


def test_compiled_wrapper_has_no_legacy_exports(tmp_path) -> None:
    """When the wrapper is built, its dynamic surface must be legacy-free."""

    sys.path.insert(0, str(ROOT))
    try:
        from sg100_teleop.robot.sg100_sdk_driver import (
            DEFAULT_SDK_DIR,
            WRAPPER_LIBRARY,
            _load_wrapper_library,
        )

        lib = _load_wrapper_library(sdk_dir=DEFAULT_SDK_DIR)
        for export in LEGACY_EXPORTS:
            assert not hasattr(lib, export), (
                f"built wrapper exports legacy symbol {export!r}"
            )
        assert WRAPPER_LIBRARY.exists()
    finally:
        sys.path.pop(0)
