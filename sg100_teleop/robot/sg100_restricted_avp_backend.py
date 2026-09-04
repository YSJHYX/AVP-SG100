"""Restricted AVP -> SG100 command backend (command path only).

The existing retargeter remains the only producer of canonical 11D targets.
This layer applies F_cmd once, activates only a hardware-validated allowlist,
and deliberately makes no claim about F_fb or dataset readiness.
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Iterable

import numpy as np

from sg100_teleop.robot.sg100_driver import (
    HAND_DOF, MODE_JOINT_POSITION, SG100HardwareMap,
)


DEFAULT_HARDWARE_VALIDATED_JOINTS = ("index_j3",)


@dataclass(frozen=True)
class RestrictedCommandResult:
    accepted: bool
    reason: str
    canonical_qcmd_rad: np.ndarray | None = None
    sdk_target_deg: np.ndarray | None = None
    active_joint_indices: tuple[int, ...] = ()
    slew_limited: bool = False
    effective_canonical_qcmd_rad: np.ndarray | None = None
    sent_monotonic_ns: int = 0


class SG100RestrictedAVPBackend:
    """Fail-closed command envelope around the native SG100 SDK driver."""

    def __init__(
        self, driver, *, joint_allowlist: Iterable[str],
        hardware_validated_joints: Iterable[str] = DEFAULT_HARDWARE_VALIDATED_JOINTS,
        hardware_map: SG100HardwareMap | None = None,
        max_cycle_delta_rad: float = 0.05,
        max_velocity_rad_s: float = 0.5,
        tracking_spike_rad: float = 0.20,
        stale_tracking_s: float = 0.10,
        feedback_stall_s: float = 0.10,
        crc_storm_delta: int = 3,
    ) -> None:
        self.driver = driver
        self.hardware_map = hardware_map or SG100HardwareMap()
        requested = tuple(dict.fromkeys(str(value) for value in joint_allowlist))
        validated = set(str(value) for value in hardware_validated_joints)
        unknown = set(requested) - set(self.hardware_map.software_joint_order)
        pending = set(requested) - validated
        if not requested:
            raise ValueError("joint_allowlist must not be empty")
        if unknown:
            raise ValueError(f"unknown SG100 joints: {sorted(unknown)}")
        if pending:
            raise ValueError(
                "allowlist contains joints without real active-command evidence: "
                f"{sorted(pending)}"
            )
        self.active_joint_indices = tuple(sorted(
            self.hardware_map.software_index_of[name] for name in requested
        ))
        self.vendor_ranges = driver.vendor_user_ranges
        if self.vendor_ranges is None or len(self.vendor_ranges) != HAND_DOF:
            raise ValueError("11 SDK command ranges are required")
        values = (
            max_cycle_delta_rad, max_velocity_rad_s, tracking_spike_rad,
            stale_tracking_s, feedback_stall_s,
        )
        if not all(np.isfinite(value) and value > 0 for value in values):
            raise ValueError("safety limits must be finite and > 0")
        if int(crc_storm_delta) <= 0:
            raise ValueError("crc_storm_delta must be > 0")
        self.max_cycle_delta_rad = float(max_cycle_delta_rad)
        self.max_velocity_rad_s = float(max_velocity_rad_s)
        self.tracking_spike_rad = float(tracking_spike_rad)
        self.stale_tracking_ns = int(stale_tracking_s * 1e9)
        self.feedback_stall_ns = int(feedback_stall_s * 1e9)
        self.crc_storm_delta = int(crc_storm_delta)
        self.anchored = False
        self.enabled = False
        self.estopped = False
        self._last_canonical: np.ndarray | None = None
        self._last_sdk_target: np.ndarray | None = None
        self._crc_baseline = 0

    def _validate_and_map(self, canonical_qcmd_rad: np.ndarray) -> np.ndarray:
        q = np.asarray(canonical_qcmd_rad, dtype=np.float64)
        if q.shape != (HAND_DOF,) or not np.all(np.isfinite(q)):
            raise ValueError("canonical qcmd must be finite 11D")
        if np.any(q < self.hardware_map.lower_rad) or np.any(
            q > self.hardware_map.upper_rad
        ):
            raise ValueError("canonical qcmd outside mechanical range; no clamp")
        # F_cmd exactly once, written explicitly because canonical_to_actual
        # clips and would hide a gross upstream mapping error.
        sdk_rad = self.hardware_map.gain * q + self.hardware_map.offset_rad
        sdk_deg = np.rad2deg(sdk_rad)
        for index, (low, high) in enumerate(self.vendor_ranges):
            if not float(low) <= float(sdk_deg[index]) <= float(high):
                raise ValueError(
                    f"SDK J{index + 1} target outside range; no clamp"
                )
        return sdk_deg

    def anchor(self, canonical_qcmd_rad: np.ndarray) -> RestrictedCommandResult:
        """C gate: cache mapped targets with zero gains/output, never raw F_fb."""

        sdk_target = self._validate_and_map(canonical_qcmd_rad)
        submission = self.driver.send_mode7_zero_output_sdk_targets(sdk_target)
        if submission is None:
            return self._safe_stop("ANCHOR_ZERO_OUTPUT_REJECTED")
        self._last_canonical = np.asarray(canonical_qcmd_rad, dtype=np.float64).copy()
        self._last_sdk_target = sdk_target.copy()
        self._crc_baseline = int(self.driver.diagnostics.get("crc_error_count", 0))
        self.anchored = True
        self.enabled = False
        return RestrictedCommandResult(
            True, "ANCHORED_ZERO_OUTPUT", self._last_canonical.copy(),
            sdk_target.copy(), self.active_joint_indices,
            effective_canonical_qcmd_rad=self._last_canonical.copy(),
            sent_monotonic_ns=int(submission.sent_monotonic_ns),
        )

    def set_enabled(self, enabled: bool) -> None:
        if enabled and (not self.anchored or self.estopped):
            raise RuntimeError("C anchor is required before enable")
        self.enabled = bool(enabled)
        if not self.enabled:
            self.driver.hold_passive()

    def emergency_stop(self) -> RestrictedCommandResult:
        self.estopped = True
        return self._safe_stop("EMERGENCY_STOP")

    def _safe_stop(self, reason: str) -> RestrictedCommandResult:
        try:
            self.driver.hold_passive()
        finally:
            self.enabled = False
        return RestrictedCommandResult(False, reason)

    def submit(
        self, canonical_qcmd_rad: np.ndarray, *, tracking_received_ns: int,
        now_ns: int | None = None, dt_s: float,
    ) -> RestrictedCommandResult:
        """Validate, rate-limit, map, and submit one existing retargeter qcmd."""

        if self.estopped:
            return RestrictedCommandResult(False, "ESTOP_LATCHED")
        if not self.anchored:
            return RestrictedCommandResult(False, "C_ANCHOR_REQUIRED")
        if not self.enabled:
            return RestrictedCommandResult(False, "NOT_ENABLED_NO_ACTIVE_COMMAND")
        now = time.monotonic_ns() if now_ns is None else int(now_ns)
        if now - int(tracking_received_ns) > self.stale_tracking_ns:
            return self._safe_stop("STALE_AVP_TRACKING")
        if not np.isfinite(dt_s) or dt_s <= 0.0:
            return self._safe_stop("INVALID_COMMAND_DT")
        if self.driver.fault_active:
            return self._safe_stop("SG100_FAULT")
        if hasattr(self.driver, "serial_open") and not self.driver.serial_open:
            return self._safe_stop("TRANSPORT_CLOSED")
        feedback = self.driver.read_feedback()
        if feedback is None or not feedback.valid:
            return self._safe_stop("TRANSPORT_FEEDBACK_MISSING")
        if now - int(feedback.received_monotonic_ns) > self.feedback_stall_ns:
            return self._safe_stop("TRANSPORT_STALL")
        if feedback.joint_modes != (MODE_JOINT_POSITION,) * HAND_DOF:
            return self._safe_stop("UNEXPECTED_RX_MODE")
        diagnostics = self.driver.diagnostics
        if (
            "vendor_work_mode" in diagnostics
            and int(diagnostics["vendor_work_mode"]) != 0
        ):
            return self._safe_stop("REALTIME_MODE_LOSS")
        crc_delta = int(diagnostics.get("crc_error_count", 0)) - self._crc_baseline
        if crc_delta >= self.crc_storm_delta:
            return self._safe_stop("CRC_STORM")
        try:
            candidate = np.asarray(canonical_qcmd_rad, dtype=np.float64)
            self._validate_and_map(candidate)
        except ValueError as exc:
            return self._safe_stop(f"COMMAND_REJECTED:{exc}")
        assert self._last_canonical is not None
        delta = candidate - self._last_canonical
        active_delta = np.abs(delta[list(self.active_joint_indices)])
        if np.any(active_delta > self.tracking_spike_rad):
            return self._safe_stop("TRACKING_SPIKE")
        if np.any(active_delta > self.max_cycle_delta_rad):
            return self._safe_stop("PER_CYCLE_DELTA_EXCEEDED")
        velocity_step = self.max_velocity_rad_s * float(dt_s)
        limited = self._last_canonical.copy()
        for index in self.active_joint_indices:
            limited[index] += float(np.clip(delta[index], -velocity_step, velocity_step))
        slew_limited = bool(np.any(np.abs(limited - candidate) > 1e-12))
        try:
            sdk_target = self._validate_and_map(limited)
        except ValueError as exc:
            return self._safe_stop(f"LIMITED_COMMAND_REJECTED:{exc}")
        # Unopened joints retain the anchor/last target and carry zero output.
        assert self._last_sdk_target is not None
        transmitted = self._last_sdk_target.copy()
        for index in self.active_joint_indices:
            transmitted[index] = sdk_target[index]
        submission = self.driver.send_mode7_active_allowlist(
            self.active_joint_indices, transmitted
        )
        if submission is None:
            return self._safe_stop("SET_HAND_CONTROL_REJECTED")
        self._last_canonical = limited
        self._last_sdk_target = transmitted
        return RestrictedCommandResult(
            True, "ACCEPTED", candidate.copy(), transmitted.copy(),
            self.active_joint_indices, slew_limited,
            effective_canonical_qcmd_rad=limited.copy(),
            sent_monotonic_ns=int(submission.sent_monotonic_ns),
        )
