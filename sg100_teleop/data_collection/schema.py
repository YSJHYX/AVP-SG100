"""Stable physical schema and event records for DATA CONTRACT V1."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping

import numpy as np


SCHEMA_NAME = "rm65_sg100_joint17_v1"
ARM_DIM = 6
HAND_DIM = 11
PHYSICAL_DIM = ARM_DIM + HAND_DIM
ARM_SLICE = slice(0, ARM_DIM)
HAND_SLICE = slice(ARM_DIM, PHYSICAL_DIM)


class RejectionReason(str, Enum):
    NON_MONOTONIC_TIMESTAMP = "NON_MONOTONIC_TIMESTAMP"
    STATE_AFTER_ACTION = "STATE_AFTER_ACTION"
    NEXT_STATE_BEFORE_ACTION = "NEXT_STATE_BEFORE_ACTION"
    NO_PRE_ACTION_FEEDBACK = "NO_PRE_ACTION_FEEDBACK"
    NO_POST_ACTION_FEEDBACK = "NO_POST_ACTION_FEEDBACK"
    STALE_PRE_STATE = "STALE_PRE_STATE"
    STALE_POST_STATE = "STALE_POST_STATE"
    ARM_FEEDBACK_INVALID = "ARM_FEEDBACK_INVALID"
    ARM_COMMAND_NOT_SENT = "ARM_COMMAND_NOT_SENT"
    HAND_COMMAND_INVALID = "HAND_COMMAND_INVALID"
    IK_REJECTED = "IK_REJECTED"
    SAFETY_FAULT = "SAFETY_FAULT"
    TRACKING_LOST = "TRACKING_LOST"
    STALE_TARGET = "STALE_TARGET"
    NONFINITE_STATE = "NONFINITE_STATE"
    NONFINITE_ACTION = "NONFINITE_ACTION"
    OUT_OF_RANGE = "OUT_OF_RANGE"
    CLOCK_DISCONTINUITY = "CLOCK_DISCONTINUITY"
    MISSING_HAND_PROXY_BASELINE = "MISSING_HAND_PROXY_BASELINE"
    INVALID_HAND_PROXY_BASELINE = "INVALID_HAND_PROXY_BASELINE"
    HAND_BASELINE_AFTER_ACTION = "HAND_BASELINE_AFTER_ACTION"
    HAND_BASELINE_STATE_MISMATCH = "HAND_BASELINE_STATE_MISMATCH"


class MarkovStatus(str, Enum):
    MARKOV_OK = "MARKOV_OK"
    MARKOV_SUSPECT = "MARKOV_SUSPECT"
    INSUFFICIENT_DATA = "INSUFFICIENT_DATA"


@dataclass(frozen=True)
class HandCommandApplied:
    command_bundle_id: int
    decision_timestamp_ns: int
    applied_timestamp_ns: int
    q_rad: np.ndarray
    audit: Mapping[str, Any] = field(default_factory=dict)
    tracking_valid: bool = True
    safety_ok: bool = True
    ik_accepted: bool = True
    stale_target: bool = False
    hand_command_valid: bool = True


@dataclass(frozen=True)
class CanonicalTransition:
    command_bundle_id: int
    state: np.ndarray
    action: np.ndarray
    next_state: np.ndarray
    state_timestamp_ns: int
    decision_timestamp_ns: int
    arm_command_sent_timestamp_ns: int
    hand_command_applied_timestamp_ns: int
    action_effective_timestamp_ns: int
    next_state_timestamp_ns: int
    auxiliary: Mapping[str, Any]

    def validate(self) -> None:
        for name, value in (
            ("state", self.state),
            ("action", self.action),
            ("next_state", self.next_state),
        ):
            vector = np.asarray(value, dtype=np.float64)
            if vector.shape != (PHYSICAL_DIM,):
                raise ValueError(f"{name} must have shape ({PHYSICAL_DIM},)")
            if not np.all(np.isfinite(vector)):
                raise ValueError(f"{name} contains non-finite values")
        if not (
            self.state_timestamp_ns
            < self.action_effective_timestamp_ns
            < self.next_state_timestamp_ns
        ):
            raise ValueError("transition timestamps are not strictly causal")


@dataclass(frozen=True)
class RejectedTransition:
    command_bundle_id: int | None
    reason: RejectionReason
    timestamp_ns: int
    detail: str


def as_joint_vector(value: np.ndarray, size: int, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},), got {vector.shape}")
    return vector.copy()


def combine_joint17(arm_q: np.ndarray, hand_q: np.ndarray) -> np.ndarray:
    arm = as_joint_vector(arm_q, ARM_DIM, "arm_q")
    hand = as_joint_vector(hand_q, HAND_DIM, "hand_q")
    return np.concatenate((arm, hand))
