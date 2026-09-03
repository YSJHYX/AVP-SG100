"""Causal hardware recorder for ``rm65_sg100_joint17_v1``.

The collector joins asynchronous hand-application, successful RM65 send, and
measured RM65 feedback events.  It never constructs transitions by shifting
sample arrays and never normalizes the stored physical joint values.
"""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import time
from typing import Any, Iterable, Mapping

import numpy as np

from sg100_teleop.data_collection.camera import CameraProvider, NullCameraProvider
from sg100_teleop.data_collection.schema import (
    ARM_DIM,
    HAND_DIM,
    PHYSICAL_DIM,
    SCHEMA_NAME,
    CanonicalTransition,
    HandCommandApplied,
    MarkovStatus,
    RejectedTransition,
    RejectionReason,
    as_joint_vector,
    combine_joint17,
)
from sg100_teleop.robot.arm_state import ArmCommandSent, ArmState
from sg100_teleop.robot.sg100_driver import SG100CommandSent, SG100HandFeedback


@dataclass(frozen=True)
class CollectorConfig:
    stale_pre_state_ns: int = 100_000_000
    stale_post_state_ns: int = 250_000_000
    feedback_buffer_size: int = 4096
    hand_history_size: int = 8192
    stale_pre_hand_state_ns: int = 150_000_000
    stale_post_hand_state_ns: int = 300_000_000


@dataclass(frozen=True)
class _HandSample:
    timestamp_ns: int
    q_rad: np.ndarray
    command_bundle_id: int | None


@dataclass(frozen=True)
class _HandFeedbackSample:
    timestamp_ns: int
    q_rad: np.ndarray
    sequence: int
    valid: bool


@dataclass(frozen=True)
class _HandSentSample:
    timestamp_ns: int
    q_rad: np.ndarray
    command_bundle_id: int | None


@dataclass(frozen=True)
class _PendingTransition:
    decision: HandCommandApplied
    arm_command: ArmCommandSent
    pre_feedback: ArmState
    effective_timestamp_ns: int
    hand_command: _HandSentSample | None = None


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (RejectionReason, MarkovStatus)):
        return value.value
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


class ArmHandDataCollector:
    """Build strictly causal 17D transitions from real hardware events."""

    def __init__(
        self,
        output_dir: str | Path,
        *,
        initial_hand_qcmd: np.ndarray | None = None,
        arm_lower_rad: np.ndarray | None = None,
        arm_upper_rad: np.ndarray | None = None,
        hand_lower_rad: np.ndarray | None = None,
        hand_upper_rad: np.ndarray | None = None,
        camera_provider: CameraProvider | None = None,
        config: CollectorConfig = CollectorConfig(),
        started_timestamp_ns: int | None = None,
        hand_measured_feedback: bool = False,
    ) -> None:
        self.output_dir = Path(output_dir)
        self.config = config
        if config.stale_pre_state_ns <= 0 or config.stale_post_state_ns <= 0:
            raise ValueError("stale-state thresholds must be positive")
        if (
            config.stale_pre_hand_state_ns <= 0
            or config.stale_post_hand_state_ns <= 0
        ):
            raise ValueError("stale-hand-state thresholds must be positive")
        self._arm_lower = self._optional_bounds(arm_lower_rad, ARM_DIM, "arm_lower_rad")
        self._arm_upper = self._optional_bounds(arm_upper_rad, ARM_DIM, "arm_upper_rad")
        self._hand_lower = self._optional_bounds(hand_lower_rad, HAND_DIM, "hand_lower_rad")
        self._hand_upper = self._optional_bounds(hand_upper_rad, HAND_DIM, "hand_upper_rad")
        self.camera_provider = camera_provider or NullCameraProvider()
        self.started_timestamp_ns = int(
            time.monotonic_ns() if started_timestamp_ns is None else started_timestamp_ns
        )
        if self.started_timestamp_ns <= 0:
            raise ValueError("started_timestamp_ns must be positive")

        self._feedback: deque[ArmState] = deque(maxlen=config.feedback_buffer_size)
        self._hand_history: deque[_HandSample] = deque(maxlen=config.hand_history_size)
        self._decisions: dict[int, HandCommandApplied] = {}
        self._pending: list[_PendingTransition] = []
        self._seen_feedback_sequence: set[int] = set()
        self._seen_arm_target_sequence: set[int] = set()
        # SG100 measured-feedback mode (real hardware): parallel to the
        # RM65 feedback/command deques.  In proxy mode these stay
        # empty and every hand canonical value keeps the V1
        # command-proxy semantics, bit-exact.
        self.hand_measured_feedback = bool(hand_measured_feedback)
        self._hand_feedback: deque[_HandFeedbackSample] = deque(
            maxlen=config.feedback_buffer_size
        )
        self._hand_sent: deque[_HandSentSample] = deque(
            maxlen=config.hand_history_size
        )
        self._seen_hand_feedback_sequence: set[int] = set()
        self._seen_hand_sent: set[tuple[int, int]] = set()
        self._hand_measured_baseline: _HandFeedbackSample | None = None
        self._last_hand_feedback_ns: int | None = None
        self._hand_sent_for_bundle: dict[int, _HandSentSample] = {}
        self._raw_events: list[dict[str, Any]] = []
        self._raw_event_sequence = 0
        self._hand_proxy_baseline_timestamp_ns: int | None = None
        self._hand_proxy_baseline_valid = False
        self.transitions: list[CanonicalTransition] = []
        self.rejections: list[RejectedTransition] = []
        self._bundle_sequence = 0
        self._last_feedback_timestamp_ns: int | None = None
        self._last_canonical_action: np.ndarray | None = None
        self._closed = False
        if initial_hand_qcmd is not None:
            self.establish_hand_proxy_baseline(
                initial_hand_qcmd,
                timestamp_ns=self.started_timestamp_ns,
            )

    def establish_hand_proxy_baseline(
        self,
        hand_qcmd_rad: np.ndarray,
        *,
        timestamp_ns: int | None = None,
        source: str = "final_teleop_hand_qcmd",
        hardware_applied: bool = False,
    ) -> None:
        """Serialize the current final teleop hand command as the proxy baseline.

        The caller must supply the command state actually held by the teleop
        runtime.  This method deliberately has no zero/home fallback.
        """
        if self._hand_proxy_baseline_valid:
            raise ValueError("hand proxy baseline is already established")
        q_hand = as_joint_vector(hand_qcmd_rad, HAND_DIM, "hand_qcmd_rad")
        if not np.all(np.isfinite(q_hand)):
            raise ValueError("hand_qcmd_rad contains non-finite values")
        if not self._inside(q_hand, self._hand_lower, self._hand_upper):
            raise ValueError("hand_qcmd_rad is outside joint bounds")
        baseline_timestamp_ns = int(
            time.monotonic_ns() if timestamp_ns is None else timestamp_ns
        )
        if baseline_timestamp_ns <= 0:
            raise ValueError("hand proxy baseline timestamp must be positive")
        if not source:
            raise ValueError("hand proxy baseline source must be non-empty")
        if hardware_applied:
            raise ValueError(
                "SG100 hardware is not present; baseline cannot be hardware-applied"
            )
        self._hand_history.append(
            _HandSample(baseline_timestamp_ns, q_hand, None)
        )
        self._hand_proxy_baseline_timestamp_ns = baseline_timestamp_ns
        self._hand_proxy_baseline_valid = True
        self._raw(
            "hand_proxy_baseline",
            baseline_timestamp_ns,
            {
                "hand_qcmd": q_hand,
                "source": source,
                "measured": False,
                "hardware_feedback": False,
                "hardware_applied": False,
                "representation": "absolute_joint_position",
                "unit": "rad",
            },
        )

    @staticmethod
    def _optional_bounds(value: np.ndarray | None, size: int, name: str) -> np.ndarray | None:
        if value is None:
            return None
        result = as_joint_vector(value, size, name)
        if not np.all(np.isfinite(result)):
            raise ValueError(f"{name} contains non-finite values")
        return result

    def next_command_bundle_id(self) -> int:
        self._bundle_sequence += 1
        return self._bundle_sequence

    def record_hand_pipeline_stages(
        self,
        stages: Mapping[str, Any],
        *,
        timestamp_ns: int | None = None,
    ) -> None:
        """Record the hand command provenance chain (audit only).

        Expected keys from SG100Retargeter.retarget_with_stages:
        raw_retarget_qcmd / coordinated_qcmd / final_qcmd.  The two
        remaining provenance levels (final_filtered_qcmd after the
        JointCommandFilter, hardware_sent_qcmd from the driver) are
        recorded by hand_final_command / hand_hardware_sent events.
        """

        timestamp = int(
            time.monotonic_ns() if timestamp_ns is None else timestamp_ns
        )
        payload = {
            "raw_retarget_qcmd": stages.get("raw_retarget_qcmd"),
            "coordinated_qcmd": stages.get("coordinated_qcmd"),
            "final_qcmd": stages.get("final_qcmd"),
            "power_grasp_lock": stages.get("power_grasp_lock"),
            "representation": "absolute_joint_position",
            "unit": "rad",
        }
        if payload["raw_retarget_qcmd"] is not None:
            self._raw("hand_raw_retarget", timestamp, payload)
            self._raw(
                "hand_coordinated_command",
                timestamp,
                payload,
            )
            self._raw("hand_final_command", timestamp, payload)

    @property
    def hand_proxy_baseline_valid(self) -> bool:
        return self._hand_proxy_baseline_valid

    @property
    def hand_proxy_baseline_timestamp_ns(self) -> int | None:
        return self._hand_proxy_baseline_timestamp_ns

    def observe_arm_feedback(self, feedback: ArmState | None) -> None:
        if feedback is None or feedback.sequence in self._seen_feedback_sequence:
            return
        self._seen_feedback_sequence.add(feedback.sequence)
        timestamp = int(feedback.received_monotonic_ns)
        self._raw("arm_feedback", timestamp, {
            "sequence": feedback.sequence,
            "q_rad": feedback.q_rad,
            "qd_rad_s": feedback.qd_rad_s,
            "valid": feedback.valid,
            "controller_error": feedback.controller_error,
            "arm_status": feedback.arm_status,
            "controller_tcp": feedback.controller_tcp,
            "source": "rm65_measured_joint_feedback",
        })
        if timestamp <= 0:
            self._reject(None, RejectionReason.CLOCK_DISCONTINUITY, timestamp, "feedback timestamp <= 0")
            return
        if self._last_feedback_timestamp_ns is not None and timestamp <= self._last_feedback_timestamp_ns:
            self._reject(None, RejectionReason.NON_MONOTONIC_TIMESTAMP, timestamp, "feedback master clock did not increase")
            return
        self._last_feedback_timestamp_ns = timestamp
        try:
            feedback.validate()
        except ValueError as exc:
            self._reject(None, RejectionReason.ARM_FEEDBACK_INVALID, timestamp, str(exc))
            self._reject_pending_after_invalid_feedback(feedback, str(exc))
            return
        if not feedback.valid or feedback.controller_error not in (None, 0):
            detail = f"valid={feedback.valid}, controller_error={feedback.controller_error}"
            self._reject(None, RejectionReason.ARM_FEEDBACK_INVALID, timestamp, detail)
            self._reject_pending_after_invalid_feedback(feedback, detail)
            return
        self._feedback.append(feedback)
        if self.hand_measured_feedback:
            self._try_complete_pending()
        else:
            self._complete_pending(feedback)

    # ------------------------------------------------------------------
    # SG100 measured-feedback mode (real hardware)
    # ------------------------------------------------------------------

    def observe_hand_feedback(
        self, feedback: SG100HandFeedback | None
    ) -> None:
        """Ingest one measured SG100 feedback sample (measured mode).

        Deduplicates by the driver's sample sequence so a frozen
        status cache can never masquerade as a new measurement, and
        stamps raw audit provenance (measured=true).
        """

        if not self.hand_measured_feedback or feedback is None:
            return
        if feedback.sequence in self._seen_hand_feedback_sequence:
            return
        self._seen_hand_feedback_sequence.add(feedback.sequence)

        timestamp = int(feedback.received_monotonic_ns)
        self._raw("hand_feedback", timestamp, {
            "sequence": feedback.sequence,
            "q_rad": feedback.q_rad,
            "raw_positions": feedback.raw_positions,
            # Numeric callback value converted by pi/180. Its physical
            # meaning is returned-mode-dependent: MODE_NULL is
            # real-hardware-proven raw/accumulated-like, mode 7 is
            # vendor-confirmed intended joint angle but unverified here.
            "raw_positions_domain": (
                "sdk_callback_rad_equivalent_mode_dependent_unverified"
            ),
            # Hardware Joint_Status.mode bytes and the mode-aware
            # classification, preserved end to end by the SDK driver.
            "joint_modes": (
                list(feedback.joint_modes)
                if feedback.joint_modes is not None
                else None
            ),
            "feedback_domain": feedback.feedback_domain,
            "canonical_valid": feedback.canonical_valid,
            "physical_scale_characterized": False,
            "valid": feedback.valid,
            "measured": True,
            "hardware_feedback": True,
            "hardware_applied": False,
            "source": "sg100_measured_joint_feedback",
            "timestamp_source": "host_monotonic_receive",
            "representation": "absolute_joint_position",
            "unit": "rad",
        })

        if timestamp <= 0:
            self._reject(
                None,
                RejectionReason.CLOCK_DISCONTINUITY,
                timestamp,
                "hand feedback timestamp <= 0",
            )
            return
        if (
            self._last_hand_feedback_ns is not None
            and timestamp <= self._last_hand_feedback_ns
        ):
            self._reject(
                None,
                RejectionReason.NON_MONOTONIC_TIMESTAMP,
                timestamp,
                "hand feedback master clock did not increase",
            )
            return
        self._last_hand_feedback_ns = timestamp

        if not feedback.valid or not np.all(
            np.isfinite(feedback.q_rad)
        ):
            self._reject(
                None,
                RejectionReason.HAND_FEEDBACK_INVALID,
                timestamp,
                f"valid={feedback.valid}",
            )
            return

        if feedback.joint_modes is not None and not feedback.canonical_valid:
            # Mode gate (ARM_HAND_DATA_CONTRACT_V1 / MODE7 audit): only
            # mode-7 feedback whose physical-angle semantics are
            # hardware-validated may become measured_hand_q.  Anything
            # else with mode information — MODE_NULL accumulated values
            # (e.g. the real N1 ~1854 deg trace), mode-1 samples, mixed
            # modes, or unvalidated mode-7 candidates — is diagnostic
            # only and is rejected as a measurement.  Samples without
            # mode information (mock/legacy drivers) keep the previous
            # manifest-level quarantine path.
            self._reject(
                None,
                RejectionReason.HAND_FEEDBACK_NON_CANONICAL_MODE,
                timestamp,
                (
                    f"feedback_domain={feedback.feedback_domain}, "
                    f"joint_modes={list(feedback.joint_modes)}, "
                    "mode7 canonical validation pending"
                ),
            )
            return

        sample = _HandFeedbackSample(
            timestamp_ns=timestamp,
            q_rad=feedback.q_rad.copy(),
            sequence=feedback.sequence,
            valid=True,
        )
        self._hand_feedback.append(sample)

        if self._hand_measured_baseline is None:
            # Replay seed: the FIRST valid measured sample.  It is a
            # measurement-baseline for replay/audit only; canonical
            # state_hand comes from measured feedback strictly before
            # each action, NOT from this baseline (the frozen V1 bug
            # required baseline == first accepted state; that
            # requirement is gone).
            self._hand_measured_baseline = sample
            self._raw("hand_measured_baseline", timestamp, {
                "q_rad": sample.q_rad,
                "sequence": sample.sequence,
                "measured": True,
                "hardware_feedback": True,
                "hardware_applied": False,
                "source": "sg100_measured_joint_feedback",
                "role": "replay_seed_audit_only",
                "representation": "absolute_joint_position",
                "unit": "rad",
            })

        self._try_complete_pending()

    def ingest_hand_command_events(
        self, events: Iterable[SG100CommandSent]
    ) -> None:
        """Ingest SG100 commands actually accepted by the transport.

        Only these events may become canonical hand actions
        (DATA CONTRACT: action_hand = final hardware-sent qcmd).
        """

        if not self.hand_measured_feedback:
            return
        for event in events:
            timestamp = int(event.sent_monotonic_ns)
            key = (timestamp, int(event.command_bundle_id or 0))
            if key in self._seen_hand_sent:
                continue
            self._seen_hand_sent.add(key)

            self._raw("hand_hardware_sent", timestamp, {
                "q_rad": event.q_rad,
                "command_bundle_id": event.command_bundle_id,
                "transmission_confirmed": event.transmission_confirmed,
                "timestamp_kind": "host_send",
                "hardware_applied": False,
                "representation": "absolute_joint_position",
                "unit": "rad",
            })

            if timestamp <= 0:
                self._reject(
                    event.command_bundle_id,
                    RejectionReason.CLOCK_DISCONTINUITY,
                    timestamp,
                    "hand sent timestamp <= 0",
                )
                continue
            if not np.all(np.isfinite(event.q_rad)):
                self._reject(
                    event.command_bundle_id,
                    RejectionReason.NONFINITE_ACTION,
                    timestamp,
                    "hand hardware-sent q is non-finite",
                )
                continue

            sent = _HandSentSample(
                timestamp_ns=timestamp,
                q_rad=event.q_rad.copy(),
                command_bundle_id=event.command_bundle_id,
            )
            self._hand_sent.append(sent)

            if (
                sent.command_bundle_id is not None
                and sent.command_bundle_id
                not in self._decisions
            ):
                self._reject(
                    sent.command_bundle_id,
                    RejectionReason.HAND_SENT_WITHOUT_DECISION,
                    timestamp,
                    "successful hand send has no matching command bundle",
                )
                continue
            if sent.command_bundle_id is not None:
                self._hand_sent_for_bundle[
                    sent.command_bundle_id
                ] = sent

            self._assemble_pending()

    def _assemble_pending(self) -> None:
        """Create pending transitions once decision+arm+hand are all in."""

        for pending in list(self._pending):
            if pending.hand_command is None:
                bundle = pending.decision.command_bundle_id
                hand_sent = self._hand_sent_for_bundle.get(bundle)
                if hand_sent is not None:
                    self._pending.remove(pending)
                    self._pending.append(
                        _PendingTransition(
                            decision=pending.decision,
                            arm_command=pending.arm_command,
                            pre_feedback=pending.pre_feedback,
                            effective_timestamp_ns=max(
                                pending.effective_timestamp_ns,
                                hand_sent.timestamp_ns,
                            ),
                            hand_command=hand_sent,
                        )
                    )
        self._try_complete_pending()

    def _try_complete_pending(self) -> None:
        """Complete measured-mode pending transitions.

        A transition needs, on BOTH arm and hand sides: measured
        feedback strictly before the action-effective time and the
        first NEW measured feedback strictly after it; the canonical
        hand action is the hardware-sent qcmd.  Any other successfully
        sent hand command between the action and the post-action hand
        feedback makes the transition INTERVENING_HAND_COMMAND (the
        post state would not be attributable to this action alone).
        """

        if not self.hand_measured_feedback:
            return
        remaining: list[_PendingTransition] = []
        for pending in self._pending:
            effective = pending.effective_timestamp_ns
            bundle_id = pending.decision.command_bundle_id

            if pending.hand_command is None:
                # Arm side assembled but the hand command for this
                # bundle never reached the transport: keep waiting
                # (finalize() rejects leftovers).
                remaining.append(pending)
                continue

            post_arm = next(
                (
                    item
                    for item in self._feedback
                    if item.received_monotonic_ns > effective
                ),
                None,
            )
            if post_arm is None:
                remaining.append(pending)
                continue

            post_hand = next(
                (
                    item
                    for item in self._hand_feedback
                    if item.timestamp_ns > effective
                ),
                None,
            )
            if post_hand is None:
                remaining.append(pending)
                continue

            intervening = [
                item
                for item in self._hand_sent
                if effective < item.timestamp_ns
                <= post_hand.timestamp_ns
                and item.command_bundle_id != bundle_id
            ]
            if intervening:
                self._reject(
                    bundle_id,
                    RejectionReason.INTERVENING_HAND_COMMAND,
                    post_hand.timestamp_ns,
                    f"{len(intervening)} other hand command(s) sent "
                    "between action and post-action feedback",
                )
                continue

            state_hand = self._latest_hand_feedback_before(effective)
            if state_hand is None:
                self._reject(
                    bundle_id,
                    RejectionReason.NO_PRE_ACTION_HAND_FEEDBACK,
                    effective,
                    "no measured SG100 feedback before action",
                )
                continue
            if effective - state_hand.timestamp_ns > self.config.stale_pre_hand_state_ns:
                self._reject(
                    bundle_id,
                    RejectionReason.STALE_PRE_HAND_STATE,
                    effective,
                    "pre-action hand feedback is stale",
                )
                continue
            if (
                post_hand.timestamp_ns - effective
                > self.config.stale_post_hand_state_ns
            ):
                self._reject(
                    bundle_id,
                    RejectionReason.STALE_POST_HAND_STATE,
                    post_hand.timestamp_ns,
                    "first post-action hand feedback is stale",
                )
                continue

            self._finalize_transition(
                pending, post_arm, post_hand
            )
        self._pending = remaining

    def _finalize_transition(
        self,
        pending: _PendingTransition,
        post_arm: ArmState,
        post_hand: _HandFeedbackSample,
    ) -> None:
        bundle_id = pending.decision.command_bundle_id
        state_hand = self._latest_hand_feedback_before(
            pending.effective_timestamp_ns
        )
        assert state_hand is not None  # checked by caller

        state = combine_joint17(
            pending.pre_feedback.q_rad, state_hand.q_rad
        )
        action = combine_joint17(
            pending.arm_command.q_rad,
            pending.hand_command.q_rad,
        )
        next_state = combine_joint17(
            post_arm.q_rad, post_hand.q_rad
        )
        if not np.all(np.isfinite(state)) or not np.all(
            np.isfinite(next_state)
        ):
            self._reject(
                bundle_id,
                RejectionReason.NONFINITE_STATE,
                post_hand.timestamp_ns,
                "state/next_state is non-finite",
            )
            return
        if not np.all(np.isfinite(action)):
            self._reject(
                bundle_id,
                RejectionReason.NONFINITE_ACTION,
                post_hand.timestamp_ns,
                "action is non-finite",
            )
            return
        if (
            not self._joint17_inside(state)
            or not self._joint17_inside(action)
            or not self._joint17_inside(next_state)
        ):
            self._reject(
                bundle_id,
                RejectionReason.OUT_OF_RANGE,
                post_hand.timestamp_ns,
                "canonical vector outside joint bounds",
            )
            return

        feedback_dt_s = (
            post_arm.received_monotonic_ns
            - pending.pre_feedback.received_monotonic_ns
        ) * 1e-9
        hand_feedback_dt_s = (
            post_hand.timestamp_ns - state_hand.timestamp_ns
        ) * 1e-9
        hand_velocity = (
            post_hand.q_rad - state_hand.q_rad
        ) / hand_feedback_dt_s
        previous_action = (
            np.full(PHYSICAL_DIM, np.nan)
            if self._last_canonical_action is None
            else self._last_canonical_action.copy()
        )
        transition = CanonicalTransition(
            command_bundle_id=bundle_id,
            state=state,
            action=action,
            next_state=next_state,
            state_timestamp_ns=(
                min(
                    pending.pre_feedback.received_monotonic_ns,
                    state_hand.timestamp_ns,
                )
            ),
            decision_timestamp_ns=(
                pending.decision.decision_timestamp_ns
            ),
            arm_command_sent_timestamp_ns=(
                pending.arm_command.sent_monotonic_ns
            ),
            # Field name kept for schema compatibility; in measured
            # mode the value is the HARDWARE SEND time of the hand
            # command (host send), not a software "applied" time.
            hand_command_applied_timestamp_ns=(
                pending.hand_command.timestamp_ns
            ),
            action_effective_timestamp_ns=(
                pending.effective_timestamp_ns
            ),
            next_state_timestamp_ns=max(
                post_arm.received_monotonic_ns,
                post_hand.timestamp_ns,
            ),
            auxiliary={
                "arm_qvel_pre_rad_s": (
                    pending.pre_feedback.qd_rad_s
                ),
                "arm_qvel_post_rad_s": post_arm.qd_rad_s,
                "feedback_dt_s": feedback_dt_s,
                "hand_feedback_dt_s": hand_feedback_dt_s,
                "previous_action": previous_action,
                "tracking_status": pending.decision.audit.get(
                    "tracking_status", "valid"
                ),
                "safety_status": pending.decision.audit.get(
                    "safety_status", "ok"
                ),
                "arm_command_latency_s": (
                    pending.arm_command.sent_monotonic_ns
                    - pending.decision.decision_timestamp_ns
                )
                * 1e-9,
                "hand_qcmd_velocity_proxy_rad_s": hand_velocity,
                "hand_velocity_measured_rad_s": hand_velocity,
                "controller_tcp_pre": (
                    pending.pre_feedback.controller_tcp
                ),
                "controller_tcp_post": post_arm.controller_tcp,
            },
        )
        transition.validate()
        self.transitions.append(transition)
        self._last_canonical_action = action.copy()

    def _latest_hand_feedback_before(
        self, timestamp_ns: int
    ) -> _HandFeedbackSample | None:
        candidates = [
            sample
            for sample in self._hand_feedback
            if sample.timestamp_ns < timestamp_ns
        ]
        return max(
            candidates,
            key=lambda sample: sample.timestamp_ns,
            default=None,
        )

    def record_control_decision(
        self,
        *,
        command_bundle_id: int,
        decision_timestamp_ns: int,
        hand_command_applied_timestamp_ns: int,
        hand_qcmd_rad: np.ndarray,
        audit: Mapping[str, Any] | None = None,
        tracking_valid: bool = True,
        safety_ok: bool = True,
        ik_accepted: bool = True,
        stale_target: bool = False,
        hand_command_valid: bool = True,
    ) -> None:
        if command_bundle_id in self._decisions:
            self._reject(command_bundle_id, RejectionReason.NON_MONOTONIC_TIMESTAMP, decision_timestamp_ns, "duplicate command_bundle_id")
            return
        try:
            q_hand = as_joint_vector(hand_qcmd_rad, HAND_DIM, "hand_qcmd_rad")
        except ValueError as exc:
            self._raw("control_decision", int(decision_timestamp_ns), {
                "command_bundle_id": command_bundle_id,
                "hand_qcmd_rad": hand_qcmd_rad,
                "validation_error": str(exc),
            })
            self._reject(
                command_bundle_id,
                RejectionReason.HAND_COMMAND_INVALID,
                int(decision_timestamp_ns),
                str(exc),
            )
            return
        decision = HandCommandApplied(
            command_bundle_id=int(command_bundle_id),
            decision_timestamp_ns=int(decision_timestamp_ns),
            applied_timestamp_ns=int(hand_command_applied_timestamp_ns),
            q_rad=q_hand,
            audit=dict(audit or {}),
            tracking_valid=bool(tracking_valid),
            safety_ok=bool(safety_ok),
            ik_accepted=bool(ik_accepted),
            stale_target=bool(stale_target),
            hand_command_valid=bool(hand_command_valid),
        )
        self._decisions[decision.command_bundle_id] = decision
        if self.hand_measured_feedback:
            # Measured mode: the recorded qcmd is a CANDIDATE only.
            # The canonical action becomes the hardware-sent qcmd
            # (ingest_hand_command_events); this raw event is the
            # command-side audit trail.
            self._raw("control_decision", decision.decision_timestamp_ns, {
                **asdict(decision),
                "hand_action_source": "hand_candidate_qcmd",
                "canonical_action_source": "final_hardware_sent_qcmd",
                "hand_command_timestamp_kind": "candidate",
                "hardware_applied": False,
            })
        else:
            self._raw("control_decision", decision.decision_timestamp_ns, {
                **asdict(decision),
                "hand_action_source": "final_teleop_qcmd",
                "hand_command_timestamp_kind": "applied",
                "hardware_applied": False,
            })
        if self.hand_measured_feedback:
            if decision.decision_timestamp_ns <= 0 or decision.applied_timestamp_ns <= 0:
                self._reject(decision.command_bundle_id, RejectionReason.CLOCK_DISCONTINUITY, decision.decision_timestamp_ns, "decision/hand timestamp <= 0")
                return
            if decision.applied_timestamp_ns < decision.decision_timestamp_ns:
                self._reject(decision.command_bundle_id, RejectionReason.CLOCK_DISCONTINUITY, decision.applied_timestamp_ns, "hand applied before decision")
                return
            if not decision.hand_command_valid or not np.all(np.isfinite(q_hand)):
                self._reject(decision.command_bundle_id, RejectionReason.HAND_COMMAND_INVALID, decision.applied_timestamp_ns, "invalid/non-finite final hand qcmd")
                return
            if not self._inside(q_hand, self._hand_lower, self._hand_upper):
                self._reject(decision.command_bundle_id, RejectionReason.OUT_OF_RANGE, decision.applied_timestamp_ns, "hand qcmd outside joint bounds")
                return
            if self._hand_measured_baseline is None:
                self._reject(
                    decision.command_bundle_id,
                    RejectionReason.NO_PRE_ACTION_HAND_FEEDBACK,
                    decision.decision_timestamp_ns,
                    "no measured SG100 feedback observed before first decision",
                )
                return
            return
        if not self._hand_proxy_baseline_valid:
            self._reject(
                decision.command_bundle_id,
                RejectionReason.MISSING_HAND_PROXY_BASELINE,
                decision.decision_timestamp_ns,
                "no serialized current final hand qcmd baseline",
            )
            return
        if decision.decision_timestamp_ns <= 0 or decision.applied_timestamp_ns <= 0:
            self._reject(decision.command_bundle_id, RejectionReason.CLOCK_DISCONTINUITY, decision.decision_timestamp_ns, "decision/hand timestamp <= 0")
            return
        if decision.applied_timestamp_ns < decision.decision_timestamp_ns:
            self._reject(decision.command_bundle_id, RejectionReason.CLOCK_DISCONTINUITY, decision.applied_timestamp_ns, "hand applied before decision")
            return
        if (
            self._hand_proxy_baseline_timestamp_ns is None
            or self._hand_proxy_baseline_timestamp_ns >= decision.applied_timestamp_ns
        ):
            self._reject(
                decision.command_bundle_id,
                RejectionReason.HAND_BASELINE_AFTER_ACTION,
                decision.applied_timestamp_ns,
                "hand proxy baseline must precede the first hand action",
            )
            return
        if self._hand_history and decision.applied_timestamp_ns <= self._hand_history[-1].timestamp_ns:
            self._reject(decision.command_bundle_id, RejectionReason.NON_MONOTONIC_TIMESTAMP, decision.applied_timestamp_ns, "hand timestamps did not increase")
            return
        if not decision.hand_command_valid or not np.all(np.isfinite(q_hand)):
            self._reject(decision.command_bundle_id, RejectionReason.HAND_COMMAND_INVALID, decision.applied_timestamp_ns, "invalid/non-finite final hand qcmd")
            return
        if not self._inside(q_hand, self._hand_lower, self._hand_upper):
            self._reject(decision.command_bundle_id, RejectionReason.OUT_OF_RANGE, decision.applied_timestamp_ns, "hand qcmd outside joint bounds")
            return
        self._hand_history.append(
            _HandSample(decision.applied_timestamp_ns, q_hand, decision.command_bundle_id)
        )

    def ingest_arm_command_events(self, events: Iterable[ArmCommandSent]) -> None:
        for event in events:
            self._raw("arm_command_sent", int(event.sent_monotonic_ns), {
                **asdict(event),
                "arm_action_source": "real_joint_command_sent",
                "representation": "absolute_joint_position",
                "unit": "rad",
            })
            try:
                event.validate()
            except ValueError as exc:
                self._reject(event.command_bundle_id, RejectionReason.NONFINITE_ACTION, int(event.sent_monotonic_ns), str(exc))
                continue
            if event.target_sequence in self._seen_arm_target_sequence:
                # Re-streaming one target is retained in the raw audit stream,
                # but one control decision creates at most one transition.
                continue
            self._seen_arm_target_sequence.add(event.target_sequence)
            bundle_id = event.command_bundle_id
            if bundle_id is None or bundle_id not in self._decisions:
                self._reject(bundle_id, RejectionReason.ARM_COMMAND_NOT_SENT, int(event.sent_monotonic_ns), "successful send has no matching command bundle")
                continue
            decision = self._decisions[bundle_id]
            reason = self._decision_rejection(decision)
            if reason is not None:
                self._reject(bundle_id, reason, int(event.sent_monotonic_ns), "command bundle failed hard validation")
                continue
            effective = max(int(event.sent_monotonic_ns), decision.applied_timestamp_ns)
            if decision.decision_timestamp_ns >= effective:
                self._reject(bundle_id, RejectionReason.CLOCK_DISCONTINUITY, effective, "action effective time is not after decision")
                continue
            if not np.all(np.isfinite(event.q_rad)):
                self._reject(bundle_id, RejectionReason.NONFINITE_ACTION, effective, "arm q_cmd_sent is non-finite")
                continue
            if not self._inside(event.q_rad, self._arm_lower, self._arm_upper):
                self._reject(bundle_id, RejectionReason.OUT_OF_RANGE, effective, "arm q_cmd_sent outside joint bounds")
                continue
            pre = self._latest_valid_feedback_before(effective)
            if pre is None:
                self._reject(bundle_id, RejectionReason.NO_PRE_ACTION_FEEDBACK, effective, "no measured RM65 feedback before command bundle")
                continue
            if pre.received_monotonic_ns >= effective:
                self._reject(bundle_id, RejectionReason.STATE_AFTER_ACTION, effective, "pre-state is not before action")
                continue
            if effective - pre.received_monotonic_ns > self.config.stale_pre_state_ns:
                self._reject(bundle_id, RejectionReason.STALE_PRE_STATE, effective, "latest pre-action feedback is stale")
                continue
            self._pending.append(_PendingTransition(decision, event, pre, effective))
            if self.hand_measured_feedback:
                # Wait for the hand hardware-sent event of the same
                # bundle; effective time is finalized there
                # (max(arm_sent, hand_sent)).
                self._assemble_pending()
                continue
            # A post-feedback sample may already be buffered because the caller
            # drained command events after polling feedback.
            for feedback in self._feedback:
                if feedback.received_monotonic_ns > effective:
                    self._complete_pending(feedback)
                    break

    def finalize(self) -> Path:
        if self._closed:
            return self.output_dir / "arm_hand_joint17_v1.npz"
        for pending in list(self._pending):
            if self.hand_measured_feedback and pending.hand_command is None:
                reason = RejectionReason.HAND_COMMAND_NOT_SENT
                detail = "episode ended before the hand command reached the transport"
            else:
                reason = RejectionReason.NO_POST_ACTION_FEEDBACK
                detail = "episode ended before post-action measured feedback"
            self._reject(
                pending.decision.command_bundle_id,
                reason,
                pending.effective_timestamp_ns,
                detail,
            )
        self._pending.clear()
        used = {transition.command_bundle_id for transition in self.transitions}
        rejected = {item.command_bundle_id for item in self.rejections}
        for bundle_id, decision in self._decisions.items():
            if bundle_id not in used and bundle_id not in rejected:
                if self.hand_measured_feedback and bundle_id not in self._hand_sent_for_bundle:
                    self._reject(bundle_id, RejectionReason.HAND_COMMAND_NOT_SENT, decision.applied_timestamp_ns, "no successful SG100 send for command bundle")
                else:
                    self._reject(bundle_id, RejectionReason.ARM_COMMAND_NOT_SENT, decision.applied_timestamp_ns, "no successful RM65 send for command bundle")

        self.output_dir.mkdir(parents=True, exist_ok=True)
        episode_path = self.output_dir / "arm_hand_joint17_v1.npz"
        arrays = self._transition_arrays()
        np.savez_compressed(episode_path, schema_name=np.asarray(SCHEMA_NAME), **arrays)
        raw_path = self.output_dir / "arm_hand_raw_events.jsonl"
        raw_path.write_text(
            "".join(json.dumps(_jsonable(event), sort_keys=True) + "\n" for event in self._raw_events),
            encoding="utf-8",
        )
        markov = self.markov_diagnostic()
        counts = Counter(item.reason.value for item in self.rejections)
        if self.hand_measured_feedback:
            hand_manifest = {
                "writer_version": 2,
                "state": "RM65 measured joint feedback | SG100 measured joint feedback",
                "action": "RM65 successful post-limiter command | SG100 final hardware-sent qcmd",
                "next_state": "RM65 post-action measured feedback | SG100 first post-action measured feedback",
                "arm_state_source": "rm65_measured_joint_feedback",
                "arm_action_source": "real_joint_command_sent",
                "hand_state_source": "sg100_measured_joint_feedback",
                "hand_initial_state_source": "first_measured_feedback_replay_seed",
                "hand_action_source": "final_hardware_sent_qcmd",
                "hand_measured_baseline_serialized": self._hand_measured_baseline is not None,
                "hand_measured_baseline_timestamp_ns": (
                    None
                    if self._hand_measured_baseline is None
                    else self._hand_measured_baseline.timestamp_ns
                ),
                "arm_backend": "real",
                "rm65_real_hardware": True,
                "hand_hardware_present": True,
                "hand_feedback_available": True,
                "hand_feedback_valid": True,
                "hand_feedback_semantics": "sg100_driver_reported_joint_feedback",
                "hand_feedback_physical_scale": "UNCHARACTERIZED",
                "hand_canonical_mapping_verdict": "BLOCKED",
                "hand_feedback_timestamp_source": "host_monotonic_receive",
                "hand_action_hardware_applied": False,
                "hand_action_hardware_sent": True,
                "hand_intervening_command_check": True,
            }
        else:
            hand_manifest = {
                "writer_version": 1,
                "state": "RM65 measured joint feedback | SG100 previous-command proxy",
                "action": "RM65 successful post-limiter command | SG100 final teleop software qcmd",
                "next_state": "RM65 post-action measured feedback | latest post-action hand qcmd proxy",
                "arm_state_source": "rm65_measured_joint_feedback",
                "arm_action_source": "real_joint_command_sent",
                "hand_state_source": "previous_command_proxy",
                "hand_initial_state_source": "serialized_command_proxy_baseline",
                "hand_action_source": "final_teleop_qcmd",
                "hand_proxy_baseline_serialized": self._hand_proxy_baseline_valid,
                "hand_proxy_baseline_timestamp_ns": self._hand_proxy_baseline_timestamp_ns,
                "arm_backend": "real",
                "rm65_real_hardware": True,
                "hand_hardware_present": False,
                "hand_action_hardware_applied": False,
                "hand_feedback_available": False,
                "hand_feedback_valid": False,
                "hand_feedback_physical_scale": "NOT_APPLICABLE_PROXY",
                "hand_canonical_mapping_verdict": "BLOCKED",
            }
        manifest = {
            "schema": SCHEMA_NAME,
            "physical_dimension": PHYSICAL_DIM,
            "layout": {"arm": "0:6", "hand": "6:17"},
            **hand_manifest,
            "representation": "absolute_joint_position",
            "unit": "rad",
            "normalization": "none_in_hardware_recorder",
            "master_clock": "time.monotonic_ns()",
            "transition_count": len(self.transitions),
            "rejection_count": len(self.rejections),
            "rejection_counts": dict(sorted(counts.items())),
            "markov_diagnostic": markov.value,
            "quarantined": (
                markov is MarkovStatus.MARKOV_SUSPECT
                or self.hand_measured_feedback
            ),
            "quarantine_reasons": [
                reason
                for reason, active in (
                    ("MARKOV_SUSPECT", markov is MarkovStatus.MARKOV_SUSPECT),
                    (
                        "SG100_PHYSICAL_SCALE_UNCHARACTERIZED",
                        self.hand_measured_feedback,
                    ),
                )
                if active
            ],
            "training_eligible_indices": (
                []
                if (
                    markov is MarkovStatus.MARKOV_SUSPECT
                    or self.hand_measured_feedback
                )
                else list(range(len(self.transitions)))
            ),
            "camera": self.camera_provider.metadata(),
            "raw_events_file": raw_path.name,
            "canonical_file": episode_path.name,
        }
        (self.output_dir / "arm_hand_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        self._closed = True
        return episode_path

    def markov_diagnostic(self) -> MarkovStatus:
        # This is deliberately a diagnostic, not a proof.  Position-only state
        # omits controller/motor/contact hidden variables.
        if len(self.transitions) < 5:
            return MarkovStatus.INSUFFICIENT_DATA
        for transition in self.transitions:
            feedback_dt = float(transition.auxiliary["feedback_dt_s"])
            latency = float(transition.auxiliary["arm_command_latency_s"])
            if feedback_dt <= 0.0 or feedback_dt > 0.5 or latency < 0.0 or latency > 0.5:
                return MarkovStatus.MARKOV_SUSPECT
        return MarkovStatus.MARKOV_OK

    def _complete_pending(self, post: ArmState) -> None:
        remaining: list[_PendingTransition] = []
        for pending in self._pending:
            if post.received_monotonic_ns <= pending.effective_timestamp_ns:
                remaining.append(pending)
                continue
            bundle_id = pending.decision.command_bundle_id
            if post.received_monotonic_ns - pending.effective_timestamp_ns > self.config.stale_post_state_ns:
                self._reject(bundle_id, RejectionReason.STALE_POST_STATE, post.received_monotonic_ns, "first post-action feedback is stale")
                continue
            state_hand = self._latest_hand_at(pending.pre_feedback.received_monotonic_ns)
            next_hand = self._latest_hand_at(post.received_monotonic_ns)
            if state_hand is None or next_hand is None:
                self._reject(bundle_id, RejectionReason.HAND_COMMAND_INVALID, post.received_monotonic_ns, "missing hand command proxy at state boundary")
                continue
            state = combine_joint17(pending.pre_feedback.q_rad, state_hand.q_rad)
            action = combine_joint17(pending.arm_command.q_rad, pending.decision.q_rad)
            next_state = combine_joint17(post.q_rad, next_hand.q_rad)
            if not np.all(np.isfinite(state)) or not np.all(np.isfinite(next_state)):
                self._reject(bundle_id, RejectionReason.NONFINITE_STATE, post.received_monotonic_ns, "state/next_state is non-finite")
                continue
            if not np.all(np.isfinite(action)):
                self._reject(bundle_id, RejectionReason.NONFINITE_ACTION, post.received_monotonic_ns, "action is non-finite")
                continue
            if not self._joint17_inside(state) or not self._joint17_inside(action) or not self._joint17_inside(next_state):
                self._reject(bundle_id, RejectionReason.OUT_OF_RANGE, post.received_monotonic_ns, "canonical vector outside joint bounds")
                continue
            feedback_dt_s = (post.received_monotonic_ns - pending.pre_feedback.received_monotonic_ns) * 1e-9
            hand_velocity = (next_hand.q_rad - state_hand.q_rad) / feedback_dt_s
            previous_action = (
                np.full(PHYSICAL_DIM, np.nan)
                if self._last_canonical_action is None
                else self._last_canonical_action.copy()
            )
            transition = CanonicalTransition(
                command_bundle_id=bundle_id,
                state=state,
                action=action,
                next_state=next_state,
                state_timestamp_ns=pending.pre_feedback.received_monotonic_ns,
                decision_timestamp_ns=pending.decision.decision_timestamp_ns,
                arm_command_sent_timestamp_ns=pending.arm_command.sent_monotonic_ns,
                hand_command_applied_timestamp_ns=pending.decision.applied_timestamp_ns,
                action_effective_timestamp_ns=pending.effective_timestamp_ns,
                next_state_timestamp_ns=post.received_monotonic_ns,
                auxiliary={
                    "arm_qvel_pre_rad_s": pending.pre_feedback.qd_rad_s,
                    "arm_qvel_post_rad_s": post.qd_rad_s,
                    "feedback_dt_s": feedback_dt_s,
                    "previous_action": previous_action,
                    "tracking_status": pending.decision.audit.get("tracking_status", "valid"),
                    "safety_status": pending.decision.audit.get("safety_status", "ok"),
                    "arm_command_latency_s": (
                        pending.arm_command.sent_monotonic_ns
                        - pending.decision.decision_timestamp_ns
                    ) * 1e-9,
                    "hand_qcmd_velocity_proxy_rad_s": hand_velocity,
                    "controller_tcp_pre": pending.pre_feedback.controller_tcp,
                    "controller_tcp_post": post.controller_tcp,
                },
            )
            transition.validate()
            self.transitions.append(transition)
            self._last_canonical_action = action.copy()
        self._pending = remaining

    def _reject_pending_after_invalid_feedback(self, feedback: ArmState, detail: str) -> None:
        remaining = []
        for pending in self._pending:
            if feedback.received_monotonic_ns > pending.effective_timestamp_ns:
                self._reject(pending.decision.command_bundle_id, RejectionReason.ARM_FEEDBACK_INVALID, feedback.received_monotonic_ns, detail)
            else:
                remaining.append(pending)
        self._pending = remaining

    def _decision_rejection(self, decision: HandCommandApplied) -> RejectionReason | None:
        if not decision.tracking_valid:
            return RejectionReason.TRACKING_LOST
        if not decision.safety_ok:
            return RejectionReason.SAFETY_FAULT
        if not decision.ik_accepted:
            return RejectionReason.IK_REJECTED
        if decision.stale_target:
            return RejectionReason.STALE_TARGET
        if not decision.hand_command_valid:
            return RejectionReason.HAND_COMMAND_INVALID
        return None

    def _latest_valid_feedback_before(self, timestamp_ns: int) -> ArmState | None:
        candidates = [
            feedback
            for feedback in self._feedback
            if feedback.valid and feedback.received_monotonic_ns < timestamp_ns
        ]
        return max(candidates, key=lambda feedback: feedback.received_monotonic_ns, default=None)

    def _latest_hand_at(self, timestamp_ns: int) -> _HandSample | None:
        candidates = [sample for sample in self._hand_history if sample.timestamp_ns <= timestamp_ns]
        return max(candidates, key=lambda sample: sample.timestamp_ns, default=None)

    @staticmethod
    def _inside(value: np.ndarray, lower: np.ndarray | None, upper: np.ndarray | None) -> bool:
        if lower is not None and np.any(value < lower):
            return False
        if upper is not None and np.any(value > upper):
            return False
        return True

    def _joint17_inside(self, value: np.ndarray) -> bool:
        return self._inside(value[:ARM_DIM], self._arm_lower, self._arm_upper) and self._inside(
            value[ARM_DIM:], self._hand_lower, self._hand_upper
        )

    def _reject(
        self,
        bundle_id: int | None,
        reason: RejectionReason,
        timestamp_ns: int,
        detail: str,
    ) -> None:
        rejection = RejectedTransition(bundle_id, reason, int(timestamp_ns), detail)
        self.rejections.append(rejection)
        self._raw("hard_rejection", int(timestamp_ns), asdict(rejection))

    def _raw(self, event_type: str, timestamp_ns: int, payload: Mapping[str, Any]) -> None:
        self._raw_event_sequence += 1
        payload_with_sequence = dict(payload)
        payload_with_sequence.setdefault("sequence_id", self._raw_event_sequence)
        self._raw_events.append({
            "event_type": event_type,
            "sequence_id": self._raw_event_sequence,
            "master_timestamp_ns": int(timestamp_ns),
            "payload": _jsonable(payload_with_sequence),
        })

    def _transition_arrays(self) -> dict[str, np.ndarray]:
        count = len(self.transitions)
        def vectors(name: str) -> np.ndarray:
            if not count:
                return np.empty((0, PHYSICAL_DIM), dtype=np.float64)
            return np.stack([getattr(item, name) for item in self.transitions])

        def integers(name: str) -> np.ndarray:
            return np.asarray([getattr(item, name) for item in self.transitions], dtype=np.int64)

        def auxiliary_vector(name: str, size: int) -> np.ndarray:
            if not count:
                return np.empty((0, size), dtype=np.float64)
            rows = []
            for item in self.transitions:
                value = item.auxiliary.get(name)
                rows.append(
                    np.full(size, np.nan, dtype=np.float64)
                    if value is None
                    else np.asarray(value, dtype=np.float64)
                )
            return np.stack(rows)

        def auxiliary_scalar(name: str) -> np.ndarray:
            return np.asarray(
                [float(item.auxiliary[name]) for item in self.transitions],
                dtype=np.float64,
            )

        return {
            "state": vectors("state"),
            "action": vectors("action"),
            "next_state": vectors("next_state"),
            "command_bundle_id": integers("command_bundle_id"),
            "state_timestamp_ns": integers("state_timestamp_ns"),
            "decision_timestamp_ns": integers("decision_timestamp_ns"),
            "arm_command_sent_timestamp_ns": integers("arm_command_sent_timestamp_ns"),
            "hand_command_applied_timestamp_ns": integers("hand_command_applied_timestamp_ns"),
            "action_effective_timestamp_ns": integers("action_effective_timestamp_ns"),
            "next_state_timestamp_ns": integers("next_state_timestamp_ns"),
            "arm_qvel_pre_rad_s": auxiliary_vector("arm_qvel_pre_rad_s", ARM_DIM),
            "arm_qvel_post_rad_s": auxiliary_vector("arm_qvel_post_rad_s", ARM_DIM),
            "feedback_dt_s": auxiliary_scalar("feedback_dt_s"),
            "previous_action": auxiliary_vector("previous_action", PHYSICAL_DIM),
            "arm_command_latency_s": auxiliary_scalar("arm_command_latency_s"),
            "hand_qcmd_velocity_proxy_rad_s": auxiliary_vector(
                "hand_qcmd_velocity_proxy_rad_s", HAND_DIM
            ),
            "controller_tcp_pre": auxiliary_vector("controller_tcp_pre", ARM_DIM),
            "controller_tcp_post": auxiliary_vector("controller_tcp_post", ARM_DIM),
            "tracking_status": np.asarray(
                [str(item.auxiliary["tracking_status"]) for item in self.transitions]
            ),
            "safety_status": np.asarray(
                [str(item.auxiliary["safety_status"]) for item in self.transitions]
            ),
            "arm_state_measured": np.ones(count, dtype=bool),
            "hand_state_measured": (
                np.ones(count, dtype=bool)
                if self.hand_measured_feedback
                else np.zeros(count, dtype=bool)
            ),
            "hand_action_hardware_applied": np.zeros(count, dtype=bool),
            "hand_action_hardware_sent": (
                np.ones(count, dtype=bool)
                if self.hand_measured_feedback
                else np.zeros(count, dtype=bool)
            ),
        }
