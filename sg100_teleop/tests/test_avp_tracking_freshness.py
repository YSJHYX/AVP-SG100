from types import SimpleNamespace

import numpy as np

from avp_stream.streamer import TrackingData
from sg100_teleop.tracking.avp_source import get_hand_from_tracking
from sg100_teleop.tracking.frame_gate import (
    TrackingFrameGate,
    TrackingFrameStatus,
)


NS = 1_000_000_000


def make_tracking(sequence, received_ns, valid=True, arm=None):
    if arm is None:
        arm = np.repeat(np.eye(4)[None, :, :], 27, axis=0)
    return TrackingData(
        {
            "right_wrist": np.eye(4)[None, :, :],
            "right_arm": arm,
            "right_fingers": arm[:25],
            "frame_sequence": sequence,
            "received_monotonic_ns": received_ns,
            "right_hand_valid": valid,
        }
    )


def accept_if_parseable(gate, tracking, now_ns, state):
    if gate.inspect(tracking, now_ns) != TrackingFrameStatus.CANDIDATE:
        return False
    hand = get_hand_from_tracking(tracking, side="right")
    if hand is None or not np.all(np.isfinite(hand)):
        gate.reject_malformed()
        return False
    state["calibrated"] = True
    state["arm_goal_updates"] += 1
    state["hand_goal_updates"] += 1
    gate.accept(now_ns)
    return True


def test_repeated_cached_frame_is_accepted_once_and_times_out():
    gate = TrackingFrameGate(0.5, 0.5, started_monotonic_ns=0)
    state = {"calibrated": False, "arm_goal_updates": 0, "hand_goal_updates": 0}
    frame = make_tracking(10, 100 * NS)

    assert accept_if_parseable(gate, frame, 100 * NS, state)
    accepted_at = gate.last_valid_tracking_monotonic_ns
    for offset in range(1, 1001):
        assert not accept_if_parseable(gate, frame, 100 * NS + offset, state)

    assert gate.accepted_frames == 1
    assert gate.duplicate_frames == 1000
    assert gate.last_valid_tracking_monotonic_ns == accepted_at
    assert gate.tracking_timed_out(101 * NS)


def test_new_untracked_frames_do_not_update_or_refresh_timeout():
    gate = TrackingFrameGate(0.5, 0.5, started_monotonic_ns=10 * NS)
    state = {"calibrated": False, "arm_goal_updates": 0, "hand_goal_updates": 0}

    for sequence in range(10, 14):
        assert not accept_if_parseable(
            gate,
            make_tracking(sequence, 10 * NS, valid=False),
            10 * NS,
            state,
        )

    assert gate.last_seen_frame_sequence == 13
    assert gate.last_valid_tracking_monotonic_ns is None
    assert gate.invalid_frames == 4
    assert state == {"calibrated": False, "arm_goal_updates": 0, "hand_goal_updates": 0}
    assert gate.tracking_timed_out(11 * NS)


def test_new_valid_but_malformed_right_hand_is_rejected():
    gate = TrackingFrameGate(0.5, 0.5, started_monotonic_ns=0)
    state = {"calibrated": False, "arm_goal_updates": 0, "hand_goal_updates": 0}
    malformed = SimpleNamespace(
        frame_sequence=20,
        received_monotonic_ns=NS,
        right_hand_valid=True,
        right=np.zeros((3, 4, 4)),
    )

    assert not accept_if_parseable(gate, malformed, NS, state)
    assert gate.malformed_frames == 1
    assert gate.last_valid_tracking_monotonic_ns is None
    assert state == {"calibrated": False, "arm_goal_updates": 0, "hand_goal_updates": 0}


def test_each_new_valid_parseable_frame_is_accepted():
    gate = TrackingFrameGate(0.5, 0.5, started_monotonic_ns=0)
    state = {"calibrated": False, "arm_goal_updates": 0, "hand_goal_updates": 0}

    for sequence in range(10, 13):
        now_ns = sequence * NS
        assert accept_if_parseable(
            gate,
            make_tracking(sequence, now_ns),
            now_ns,
            state,
        )

    assert gate.accepted_frames == 3
    assert state == {"calibrated": True, "arm_goal_updates": 3, "hand_goal_updates": 3}


def test_stale_first_sample_cannot_become_calibration_candidate():
    gate = TrackingFrameGate(0.25, 0.5, started_monotonic_ns=10 * NS)
    state = {"calibrated": False, "arm_goal_updates": 0, "hand_goal_updates": 0}

    assert not accept_if_parseable(
        gate,
        make_tracking(5, 9 * NS),
        10 * NS,
        state,
    )
    assert gate.last_seen_frame_sequence == 5
    assert gate.stale_frames == 1
    assert state == {"calibrated": False, "arm_goal_updates": 0, "hand_goal_updates": 0}


def test_missing_metadata_is_not_valid_in_production_gate():
    gate = TrackingFrameGate(0.5, 0.5, started_monotonic_ns=0)
    legacy = TrackingData({"right_arm": np.repeat(np.eye(4)[None], 27, axis=0)})

    assert gate.inspect(legacy, 0) == TrackingFrameStatus.MISSING_METADATA
    assert gate.accepted_frames == 0


def test_tracking_data_exposes_frame_validity_metadata():
    tracking = make_tracking(42, 7 * NS, valid=True)

    assert tracking.frame_sequence == 42
    assert tracking.received_monotonic_ns == 7 * NS
    assert tracking.right_hand_valid is True


def main():
    test_repeated_cached_frame_is_accepted_once_and_times_out()
    test_new_untracked_frames_do_not_update_or_refresh_timeout()
    test_new_valid_but_malformed_right_hand_is_rejected()
    test_each_new_valid_parseable_frame_is_accepted()
    test_stale_first_sample_cannot_become_calibration_candidate()
    test_missing_metadata_is_not_valid_in_production_gate()
    test_tracking_data_exposes_frame_validity_metadata()
    print("AVP TRACKING FRESHNESS + VALIDITY: PASS")


if __name__ == "__main__":
    main()
