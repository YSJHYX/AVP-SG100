from dataclasses import dataclass
from enum import Enum
import time
from typing import Optional


class TrackingFrameStatus(str, Enum):
    CANDIDATE = "candidate"
    DUPLICATE = "duplicate"
    INVALID = "invalid"
    STALE = "stale"
    MISSING_METADATA = "missing_metadata"


@dataclass
class TrackingFrameGate:
    """Gate frame identity/validity without owning parsing or teleop math."""

    max_frame_age_seconds: float
    tracking_timeout_seconds: float
    started_monotonic_ns: Optional[int] = None

    def __post_init__(self) -> None:
        if self.started_monotonic_ns is None:
            self.started_monotonic_ns = time.monotonic_ns()
        self.last_seen_frame_sequence: Optional[int] = None
        self.last_valid_tracking_monotonic_ns: Optional[int] = None
        self.accepted_frames = 0
        self.duplicate_frames = 0
        self.invalid_frames = 0
        self.malformed_frames = 0
        self.stale_frames = 0
        self.missing_metadata_frames = 0

    def inspect(self, tracking, now_monotonic_ns: Optional[int] = None) -> TrackingFrameStatus:
        now_ns = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
        sequence = getattr(tracking, "frame_sequence", None)
        received_ns = getattr(tracking, "received_monotonic_ns", None)

        if sequence is None or received_ns is None:
            self.missing_metadata_frames += 1
            return TrackingFrameStatus.MISSING_METADATA

        sequence = int(sequence)
        if sequence == self.last_seen_frame_sequence:
            self.duplicate_frames += 1
            return TrackingFrameStatus.DUPLICATE

        # A changed identity is a newly observed source frame. Sequence resets
        # after an app reconnect are intentionally accepted as a new epoch.
        self.last_seen_frame_sequence = sequence

        if getattr(tracking, "right_hand_valid", None) is not True:
            self.invalid_frames += 1
            return TrackingFrameStatus.INVALID

        age_ns = max(0, now_ns - int(received_ns))
        if age_ns > int(self.max_frame_age_seconds * 1_000_000_000):
            self.stale_frames += 1
            return TrackingFrameStatus.STALE

        return TrackingFrameStatus.CANDIDATE

    def accept(self, now_monotonic_ns: Optional[int] = None) -> None:
        self.last_valid_tracking_monotonic_ns = (
            time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
        )
        self.accepted_frames += 1

    def reject_malformed(self) -> None:
        self.malformed_frames += 1

    def tracking_timed_out(self, now_monotonic_ns: Optional[int] = None) -> bool:
        now_ns = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
        reference_ns = (
            self.last_valid_tracking_monotonic_ns
            if self.last_valid_tracking_monotonic_ns is not None
            else self.started_monotonic_ns
        )
        return now_ns - int(reference_ns) > int(
            self.tracking_timeout_seconds * 1_000_000_000
        )
