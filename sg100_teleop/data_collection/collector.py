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


@dataclass(frozen=True)
class CollectorConfig:
    stale_pre_state_ns: int = 100_000_000
    stale_post_state_ns: int = 250_000_000
    feedback_buffer_size: int = 4096
    hand_history_size: int = 8192


@dataclass(frozen=True)
class _HandSample:
    timestamp_ns: int
    q_rad: np.ndarray
    command_bundle_id: int | None


@dataclass(frozen=True)
class _PendingTransition:
    decision: HandCommandApplied
    arm_command: ArmCommandSent
    pre_feedback: ArmState
    effective_timestamp_ns: int


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
    ) -> None:
        self.output_dir = Path(output_dir)
        self.config = config
        if config.stale_pre_state_ns <= 0 or config.stale_post_state_ns <= 0:
            raise ValueError("stale-state thresholds must be positive")
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
        self._complete_pending(feedback)

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
        self._raw("control_decision", decision.decision_timestamp_ns, {
            **asdict(decision),
            "hand_action_source": "final_teleop_qcmd",
            "hand_command_timestamp_kind": "applied",
            "hardware_applied": False,
        })
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
            self._reject(
                pending.decision.command_bundle_id,
                RejectionReason.NO_POST_ACTION_FEEDBACK,
                pending.effective_timestamp_ns,
                "episode ended before post-action measured feedback",
            )
        self._pending.clear()
        used = {transition.command_bundle_id for transition in self.transitions}
        rejected = {item.command_bundle_id for item in self.rejections}
        for bundle_id, decision in self._decisions.items():
            if bundle_id not in used and bundle_id not in rejected:
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
        manifest = {
            "schema": SCHEMA_NAME,
            "physical_dimension": PHYSICAL_DIM,
            "layout": {"arm": "0:6", "hand": "6:17"},
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
            "representation": "absolute_joint_position",
            "unit": "rad",
            "normalization": "none_in_hardware_recorder",
            "master_clock": "time.monotonic_ns()",
            "transition_count": len(self.transitions),
            "rejection_count": len(self.rejections),
            "rejection_counts": dict(sorted(counts.items())),
            "markov_diagnostic": markov.value,
            "quarantined": markov is MarkovStatus.MARKOV_SUSPECT,
            "training_eligible_indices": (
                [] if markov is MarkovStatus.MARKOV_SUSPECT else list(range(len(self.transitions)))
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
            "hand_state_measured": np.zeros(count, dtype=bool),
            "hand_action_hardware_applied": np.zeros(count, dtype=bool),
        }
