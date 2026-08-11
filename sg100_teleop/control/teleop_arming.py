from enum import Enum
from threading import Lock
import time
from typing import Optional


class TeleopArmState(str, Enum):
    WAITING_FOR_TRACKING = "WAITING_FOR_TRACKING"
    WAITING_FOR_NEUTRAL_CONFIRM = "WAITING_FOR_NEUTRAL_CONFIRM"
    CALIBRATION_REQUESTED = "CALIBRATION_REQUESTED"
    TELEOP_ACTIVE = "TELEOP_ACTIVE"
    TRACKING_TIMEOUT_DISARMED = "TRACKING_TIMEOUT_DISARMED"


class TeleopArmingController:
    """Small, source-agnostic state gate for explicit neutral arming."""

    def __init__(self) -> None:
        self._lock = Lock()
        self._state = TeleopArmState.WAITING_FOR_TRACKING
        self._last_valid_frame_sequence: Optional[int] = None
        self._request_after_sequence: Optional[int] = None
        self._request_monotonic_ns: Optional[int] = None

    @property
    def state(self) -> TeleopArmState:
        with self._lock:
            return self._state

    @property
    def active(self) -> bool:
        return self.state == TeleopArmState.TELEOP_ACTIVE

    def observe_valid_frame(self, frame_sequence: int) -> bool:
        """Record a parseable NEW+VALID frame and report a state transition."""
        with self._lock:
            self._last_valid_frame_sequence = int(frame_sequence)
            if self._state in (
                TeleopArmState.WAITING_FOR_TRACKING,
                TeleopArmState.TRACKING_TIMEOUT_DISARMED,
            ):
                self._state = TeleopArmState.WAITING_FOR_NEUTRAL_CONFIRM
                return True
            return False

    def request_neutral_calibration(
        self,
        requested_monotonic_ns: Optional[int] = None,
    ) -> bool:
        """Request that a later valid frame become the neutral anchor."""
        with self._lock:
            if self._state not in (
                TeleopArmState.WAITING_FOR_NEUTRAL_CONFIRM,
                TeleopArmState.TELEOP_ACTIVE,
            ):
                return False
            self._request_after_sequence = self._last_valid_frame_sequence
            self._request_monotonic_ns = (
                time.monotonic_ns()
                if requested_monotonic_ns is None
                else int(requested_monotonic_ns)
            )
            self._state = TeleopArmState.CALIBRATION_REQUESTED
            return True

    def should_calibrate(
        self,
        frame_sequence: int,
        received_monotonic_ns: int,
    ) -> bool:
        """Return true only for a frame received after the operator request."""
        with self._lock:
            return (
                self._state == TeleopArmState.CALIBRATION_REQUESTED
                and int(frame_sequence) != self._request_after_sequence
                and int(received_monotonic_ns) > int(self._request_monotonic_ns)
            )

    def complete_calibration(self) -> None:
        with self._lock:
            if self._state != TeleopArmState.CALIBRATION_REQUESTED:
                raise RuntimeError("Neutral calibration was not requested")
            self._state = TeleopArmState.TELEOP_ACTIVE
            self._request_after_sequence = None
            self._request_monotonic_ns = None

    def disarm_for_tracking_timeout(self) -> bool:
        """Invalidate arming after a real timeout; reacquisition cannot re-arm."""
        with self._lock:
            if self._state not in (
                TeleopArmState.TELEOP_ACTIVE,
                TeleopArmState.CALIBRATION_REQUESTED,
            ):
                return False
            self._state = TeleopArmState.TRACKING_TIMEOUT_DISARMED
            self._request_after_sequence = None
            self._request_monotonic_ns = None
            return True
