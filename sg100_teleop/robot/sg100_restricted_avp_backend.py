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
    HAND_DOF, MODE_JOINT_POSITION, SG100CommandLimitContract,
    SG100HardwareMap,
)


DEFAULT_HARDWARE_VALIDATED_JOINTS = ("thumb_j1", "thumb_j2", "thumb_j3", "index_j1", "index_j2", "index_j3", "middle_j1", "middle_j2", "little_j1", "little_j2", "little_j3",)


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


@dataclass(frozen=True)
class SG100SafetyEvent:
    event: str
    timestamp_ns: int
    joint_name: str | None = None
    joint_index: int | None = None
    domain: str | None = None
    requested: float | None = None
    effective: float | None = None
    unit: str | None = None
    details: dict | None = None


class SG100RestrictedAVPBackend:
    """Fail-closed command envelope around the native SG100 SDK driver."""

    def __init__(
        self, driver, *, joint_allowlist: Iterable[str],
        hardware_validated_joints: Iterable[str] = DEFAULT_HARDWARE_VALIDATED_JOINTS,
        hardware_map: SG100HardwareMap | None = None,
        max_cycle_delta_rad: float | None = 0.05,
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
        self.command_limit_contract = SG100CommandLimitContract(
            self.hardware_map, self.vendor_ranges
        )
        values = (
            max_velocity_rad_s, tracking_spike_rad, stale_tracking_s,
            feedback_stall_s,
        )
        if not all(np.isfinite(value) and value > 0 for value in values):
            raise ValueError("safety limits must be finite and > 0")
        if max_cycle_delta_rad is not None and (
            not np.isfinite(max_cycle_delta_rad) or max_cycle_delta_rad <= 0
        ):
            raise ValueError(
                "max_cycle_delta_rad must be None or finite and > 0"
            )
        if int(crc_storm_delta) <= 0:
            raise ValueError("crc_storm_delta must be > 0")
        self.max_cycle_delta_rad = (
            None
            if max_cycle_delta_rad is None
            else float(max_cycle_delta_rad)
        )
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
        self._safety_events: list[SG100SafetyEvent] = []
        self._clamp_event_state: dict[int, tuple[float, float]] = {}

    def _validate_and_map(self, canonical_qcmd_rad: np.ndarray) -> np.ndarray:
        """Compatibility helper: validate shape, clamp, map and SDK-guard."""

        return self.command_limit_contract.apply(
            canonical_qcmd_rad
        ).sdk_sent_deg

    def _record_limit_events(self, application) -> None:
        changed = set(application.canonical_clamped_indices)
        for index in range(HAND_DOF):
            if index not in changed:
                if index in self._clamp_event_state:
                    del self._clamp_event_state[index]
                    self._safety_events.append(SG100SafetyEvent(
                        event="JOINT_LIMIT_CLAMP_CLEARED",
                        timestamp_ns=time.monotonic_ns(),
                        joint_name=self.hardware_map.software_joint_order[index],
                        joint_index=index,
                        domain="SDK",
                    ))
                continue
            requested_sdk_deg = float(np.rad2deg(
                self.hardware_map.gain[index]
                * application.desired_canonical_rad[index]
                + self.hardware_map.offset_rad[index]
            ))
            effective_sdk_deg = float(application.sdk_sent_deg[index])
            previous = self._clamp_event_state.get(index)
            state = (requested_sdk_deg, effective_sdk_deg)
            # Entry and meaningful requested/effective changes only.  This
            # avoids a 500 Hz stream of identical clamp diagnostics.
            if previous is None or max(
                abs(state[0] - previous[0]), abs(state[1] - previous[1])
            ) >= 0.1:
                self._safety_events.append(SG100SafetyEvent(
                    event="JOINT_LIMIT_CLAMP",
                    timestamp_ns=time.monotonic_ns(),
                    joint_name=self.hardware_map.software_joint_order[index],
                    joint_index=index,
                    domain="SDK",
                    requested=requested_sdk_deg,
                    effective=effective_sdk_deg,
                    unit="deg",
                    details={
                        "desired_canonical_rad": float(
                            application.desired_canonical_rad[index]
                        ),
                        "effective_canonical_rad": float(
                            application.effective_canonical_rad[index]
                        ),
                    },
                ))
                self._clamp_event_state[index] = state
        for index in application.sdk_mapping_mismatch_indices:
            self._safety_events.append(SG100SafetyEvent(
                event="SDK_LIMIT_MAPPING_MISMATCH",
                timestamp_ns=time.monotonic_ns(),
                joint_name=self.hardware_map.software_joint_order[index],
                joint_index=index,
                domain="SDK",
                requested=float(application.sdk_before_guard_deg[index]),
                effective=float(application.sdk_sent_deg[index]),
                unit="deg",
            ))

    def drain_safety_events(self) -> tuple[SG100SafetyEvent, ...]:
        events = tuple(self._safety_events)
        self._safety_events.clear()
        return events

    def anchor(self, canonical_qcmd_rad: np.ndarray) -> RestrictedCommandResult:
        """C gate: cache mapped targets with zero gains/output, never raw F_fb."""

        application = self.command_limit_contract.apply(canonical_qcmd_rad)
        self._record_limit_events(application)
        sdk_target = application.sdk_sent_deg
        submission = self.driver.send_mode7_zero_output_sdk_targets(sdk_target)
        if submission is None:
            return self._safe_stop("ANCHOR_ZERO_OUTPUT_REJECTED")
        self._last_canonical = application.effective_canonical_rad.copy()
        self._last_sdk_target = sdk_target.copy()
        self._crc_baseline = int(self.driver.diagnostics.get("crc_error_count", 0))
        self.anchored = True
        self.enabled = False
        return RestrictedCommandResult(
            True, "ANCHORED_ZERO_OUTPUT",
            application.desired_canonical_rad.copy(),
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
            application = self.command_limit_contract.apply(canonical_qcmd_rad)
        except ValueError as exc:
            return self._safe_stop(f"COMMAND_REJECTED:{exc}")
        self._record_limit_events(application)
        candidate = application.effective_canonical_rad
        assert self._last_canonical is not None
        delta = candidate - self._last_canonical
        active_delta = np.abs(delta[list(self.active_joint_indices)])
        if np.any(active_delta > self.tracking_spike_rad):
            return self._safe_stop("TRACKING_SPIKE")
        if (
            self.max_cycle_delta_rad is not None
            and np.any(active_delta > self.max_cycle_delta_rad)
        ):
            return self._safe_stop("PER_CYCLE_DELTA_EXCEEDED")
        velocity_step = self.max_velocity_rad_s * float(dt_s)
        limited = self._last_canonical.copy()
        for index in self.active_joint_indices:
            limited[index] += float(np.clip(delta[index], -velocity_step, velocity_step))
        slew_limited = bool(np.any(np.abs(limited - candidate) > 1e-12))
        limited_application = self.command_limit_contract.apply(limited)
        sdk_target = limited_application.sdk_sent_deg
        if limited_application.sdk_mapping_mismatch_indices:
            self._record_limit_events(limited_application)
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
        final_effective = (
            np.deg2rad(transmitted) - self.hardware_map.offset_rad
        ) / self.hardware_map.gain
        self._last_canonical = final_effective.copy()
        self._last_sdk_target = transmitted
        return RestrictedCommandResult(
            True, "ACCEPTED", application.desired_canonical_rad.copy(),
            transmitted.copy(),
            self.active_joint_indices, slew_limited,
            effective_canonical_qcmd_rad=final_effective.copy(),
            sent_monotonic_ns=int(submission.sent_monotonic_ns),
        )
