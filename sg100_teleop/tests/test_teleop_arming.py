import numpy as np

from avp_stream.streamer import TrackingData
from sg100_teleop.control.teleop_arming import (
    TeleopArmingController,
    TeleopArmState,
)
from sg100_teleop.tracking.avp_source import get_hand_from_tracking
from sg100_teleop.tracking.avp_wrist_mapper import (
    AVPWristRelativeMapper,
    make_transform,
)
from sg100_teleop.tracking.frame_gate import (
    TrackingFrameGate,
    TrackingFrameStatus,
)


NS = 1_000_000_000
ARM_READY = np.arange(6, dtype=np.float64) * 0.1
HAND_STARTUP = np.zeros(11, dtype=np.float64)


def tracking_frame(sequence, received_ns, valid=True, malformed=False):
    arm = np.repeat(np.eye(4)[None, :, :], 27, axis=0)
    if malformed:
        arm = arm[:3]
    wrist = np.eye(4)
    wrist[0, 3] = sequence * 0.001
    return TrackingData(
        {
            "right_wrist": wrist[None, :, :],
            "right_arm": arm,
            "right_fingers": arm[:25],
            "frame_sequence": sequence,
            "received_monotonic_ns": received_ns,
            "right_hand_valid": valid,
        }
    )


class Harness:
    def __init__(self):
        self.gate = TrackingFrameGate(0.5, 0.5, started_monotonic_ns=0)
        self.arming = TeleopArmingController()
        self.mapper = AVPWristRelativeMapper()
        self.arm_goal = ARM_READY.copy()
        self.hand_goal = HAND_STARTUP.copy()
        self.current_tcp = make_transform(np.array([0.4, 0.1, 0.6]), np.eye(3))
        self.q_hand0_runtime = None
        self.wrist_anchor_sequence = None
        self.hand_anchor_sequence = None
        self.active_updates = 0

    def request(self, now_ns):
        return self.arming.request_neutral_calibration(now_ns)

    def process(self, tracking, now_ns):
        if self.gate.inspect(tracking, now_ns) != TrackingFrameStatus.CANDIDATE:
            return False
        hand = get_hand_from_tracking(tracking, side="right")
        if hand is None or not np.all(np.isfinite(hand)):
            self.gate.reject_malformed()
            return False

        sequence = tracking.frame_sequence
        q_hand_candidate = np.full(11, sequence * 0.01, dtype=np.float64)
        wrist = np.asarray(hand[0], dtype=np.float64)
        self.arming.observe_valid_frame(sequence)
        self.gate.accept(now_ns)

        if self.arming.should_calibrate(
            sequence,
            tracking.received_monotonic_ns,
        ):
            self.mapper.calibrate(wrist, self.current_tcp)
            self.q_hand0_runtime = q_hand_candidate.copy()
            self.wrist_anchor_sequence = sequence
            self.hand_anchor_sequence = sequence
            self.hand_goal = q_hand_candidate.copy()
            self.arming.complete_calibration()
            return True

        if self.arming.active:
            target = self.mapper.map(wrist)
            self.arm_goal[0] = target[0, 3]
            self.hand_goal = q_hand_candidate.copy()
            self.active_updates += 1
        return True

    def timeout(self, now_ns):
        arm_before = self.arm_goal.copy()
        hand_before = self.hand_goal.copy()
        if self.gate.tracking_timed_out(now_ns):
            if self.arming.disarm_for_tracking_timeout():
                self.mapper = AVPWristRelativeMapper()
        np.testing.assert_allclose(self.arm_goal, arm_before)
        np.testing.assert_allclose(self.hand_goal, hand_before)


def test_a_startup_without_confirmation_holds_goals():
    harness = Harness()
    for sequence in (1, 2, 3):
        harness.process(tracking_frame(sequence, sequence * NS), sequence * NS)
    assert harness.arming.state == TeleopArmState.WAITING_FOR_NEUTRAL_CONFIRM
    assert not harness.mapper.calibrated
    np.testing.assert_allclose(harness.arm_goal, ARM_READY)
    np.testing.assert_allclose(harness.hand_goal, HAND_STARTUP)


def test_b_c_d_request_waits_through_duplicate_and_invalid_for_valid_new_frame():
    harness = Harness()
    frame10 = tracking_frame(10, 10 * NS)
    harness.process(frame10, 10 * NS)
    assert harness.request(11 * NS)

    assert not harness.process(frame10, 11 * NS)
    assert harness.arming.state == TeleopArmState.CALIBRATION_REQUESTED
    assert not harness.process(tracking_frame(11, 12 * NS, valid=False), 12 * NS)
    assert harness.arming.state == TeleopArmState.CALIBRATION_REQUESTED

    assert harness.process(tracking_frame(12, 13 * NS), 13 * NS)
    assert harness.arming.state == TeleopArmState.TELEOP_ACTIVE
    assert harness.wrist_anchor_sequence == 12
    assert harness.hand_anchor_sequence == 12
    np.testing.assert_allclose(harness.q_hand0_runtime, np.full(11, 0.12))


def test_request_does_not_consume_a_new_frame_received_before_request():
    harness = Harness()
    harness.process(tracking_frame(1, NS), NS)
    assert harness.request(3 * NS)
    harness.process(tracking_frame(2, 2 * NS), 3 * NS)
    assert harness.arming.state == TeleopArmState.CALIBRATION_REQUESTED
    assert not harness.mapper.calibrated
    harness.process(tracking_frame(3, 4 * NS), 4 * NS)
    assert harness.arming.state == TeleopArmState.TELEOP_ACTIVE
    assert harness.wrist_anchor_sequence == 3


def armed_harness():
    harness = Harness()
    harness.process(tracking_frame(10, 10 * NS), 10 * NS)
    assert harness.request(11 * NS)
    harness.process(tracking_frame(12, 12 * NS), 12 * NS)
    return harness


def test_e_active_new_frames_run_existing_pipeline():
    harness = armed_harness()
    for sequence in (13, 14, 15):
        harness.process(tracking_frame(sequence, sequence * NS), sequence * NS)
    assert harness.active_updates == 3
    np.testing.assert_allclose(harness.hand_goal, np.full(11, 0.15))
    assert not np.allclose(harness.arm_goal, ARM_READY)


def test_f_short_dropout_holds_goals_and_remains_active():
    harness = armed_harness()
    arm_before = harness.arm_goal.copy()
    hand_before = harness.hand_goal.copy()
    duplicate = tracking_frame(12, 12 * NS)
    assert not harness.process(duplicate, 12 * NS + 100_000_000)
    assert not harness.process(
        tracking_frame(13, 12 * NS + 200_000_000, valid=False),
        12 * NS + 200_000_000,
    )
    assert harness.arming.state == TeleopArmState.TELEOP_ACTIVE
    np.testing.assert_allclose(harness.arm_goal, arm_before)
    np.testing.assert_allclose(harness.hand_goal, hand_before)


def test_g_h_i_timeout_reacquisition_and_reconfirmation():
    harness = armed_harness()
    arm_before = harness.arm_goal.copy()
    hand_before = harness.hand_goal.copy()
    harness.timeout(13 * NS)
    assert harness.arming.state == TeleopArmState.TRACKING_TIMEOUT_DISARMED
    assert not harness.mapper.calibrated

    harness.process(tracking_frame(20, 13 * NS), 13 * NS)
    assert harness.arming.state == TeleopArmState.WAITING_FOR_NEUTRAL_CONFIRM
    np.testing.assert_allclose(harness.arm_goal, arm_before)
    np.testing.assert_allclose(harness.hand_goal, hand_before)
    assert not harness.mapper.calibrated

    assert harness.request(14 * NS)
    harness.process(tracking_frame(21, 15 * NS), 15 * NS)
    assert harness.arming.state == TeleopArmState.TELEOP_ACTIVE
    assert harness.wrist_anchor_sequence == 21
    assert harness.hand_anchor_sequence == 21
    np.testing.assert_allclose(harness.q_hand0_runtime, np.full(11, 0.21))


def test_active_recalibration_holds_current_tcp_anchor():
    harness = armed_harness()
    held_tcp = make_transform(np.array([0.55, -0.1, 0.7]), np.eye(3))
    harness.current_tcp = held_tcp.copy()
    assert harness.request(13 * NS)
    harness.process(tracking_frame(30, 14 * NS), 14 * NS)
    np.testing.assert_allclose(harness.mapper._T_tcp0, held_tcp)
    assert harness.arming.state == TeleopArmState.TELEOP_ACTIVE


def main():
    test_a_startup_without_confirmation_holds_goals()
    test_b_c_d_request_waits_through_duplicate_and_invalid_for_valid_new_frame()
    test_request_does_not_consume_a_new_frame_received_before_request()
    test_e_active_new_frames_run_existing_pipeline()
    test_f_short_dropout_holds_goals_and_remains_active()
    test_g_h_i_timeout_reacquisition_and_reconfirmation()
    test_active_recalibration_holds_current_tcp_anchor()
    print("EXPLICIT NEUTRAL CALIBRATION + TELEOP ARMING: PASS")


if __name__ == "__main__":
    main()
