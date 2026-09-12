"""Offline tests for the optional arm_not_moving stop; never connect hardware."""

from dataclasses import replace
import time
import unittest

import numpy as np

from sg100_teleop.robot.arm_state import ArmState, ArmTarget
from sg100_teleop.robot.rm65_real import RM65RealBackend
from sg100_teleop.robot.safety_monitor import SafetyMonitor
from sg100_teleop.tests.test_robot_backend_units import FakeSdkArm


class StalledFakeArm(FakeSdkArm):
    """Accept commands, but keep measured joints stationary."""

    def _q_deg(self):
        return list(self.initial)


class TestArmNotMovingStopOption(unittest.TestCase):
    def samples(self):
        now = 1_000_000_000
        return dict(
            now_monotonic_ns=now,
            latest_target=ArmTarget(np.zeros(6), now, 1),
            feedback=ArmState(
                q_rad=np.zeros(6), qd_rad_s=np.zeros(6),
                received_monotonic_ns=now, sequence=1,
                controller_error=0, arm_status=None, valid=True,
            ),
            last_cmd_rad=np.full(6, 0.1),
            commanded_velocity_rad_s=np.full(6, 0.5),
            actual_velocity_rad_s=np.zeros(6),
            dt_s=0.01,
        )

    def warning_monitor(self):
        monitor = SafetyMonitor(0.15, 0.2, arm_not_moving_stop=False)
        for _ in range(40):
            self.assertIsNone(monitor.check(**self.samples()))
        self.assertEqual(monitor.motion_warning.name, "arm_not_moving")
        return monitor

    def test_runtime_cli_defaults_off_and_accepts_explicit_on_off(self):
        from sg100_teleop.sim.run_avp_arm_hand_sim import build_arg_parser

        parser = build_arg_parser()
        base = ["--ip", "fake-avp", "--arm-backend", "real", "--arm-ip", "fake-arm"]
        self.assertFalse(parser.parse_args(base).arm_not_moving_stop)
        self.assertTrue(parser.parse_args(base + ["--arm-not-moving-stop"]).arm_not_moving_stop)
        self.assertFalse(parser.parse_args(base + ["--no-arm-not-moving-stop"]).arm_not_moving_stop)

    def test_warning_only_does_not_skip_position_or_other_stop_checks(self):
        samples = self.samples()
        faults = [
            ("follow_error", {"last_cmd_rad": np.full(6, 0.4)}),
            ("target_missing", {"latest_target": None}),
            ("target_stale", {"latest_target": replace(samples["latest_target"], source_timestamp_ns=0)}),
            ("feedback_stale", {"feedback": None}),
            ("feedback_stale", {"feedback": replace(samples["feedback"], received_monotonic_ns=0)}),
            ("feedback_stale", {"feedback": replace(samples["feedback"], valid=False)}),
            ("controller_error", {"feedback": replace(samples["feedback"], controller_error=3)}),
        ]
        for expected, changes in faults:
            with self.subTest(expected=expected, changes=changes):
                monitor = self.warning_monitor()
                trigger = monitor.check(**(samples | changes))
                self.assertIsNotNone(trigger)
                self.assertEqual(trigger.name, expected)

    def test_warning_clears_on_recovery_and_rearm_reset(self):
        monitor = self.warning_monitor()
        samples = self.samples()
        samples["actual_velocity_rad_s"] = np.full(6, 0.3)
        self.assertIsNone(monitor.check(**samples))
        self.assertIsNone(monitor.motion_warning)
        for _ in range(40):
            self.assertIsNone(monitor.check(**self.samples()))
        self.assertIsNotNone(monitor.motion_warning)
        monitor.reset_follow_error_state()
        self.assertIsNone(monitor.motion_warning)
        self.assertIsNone(monitor.check(**self.samples()))
        self.assertIsNone(monitor.motion_warning)

    def test_opt_in_stops_only_after_detection_window(self):
        monitor = SafetyMonitor(0.15, 0.2, arm_not_moving_stop=True)
        for _ in range(25):
            self.assertIsNone(monitor.check(**self.samples()))
        trigger = None
        for _ in range(10):
            trigger = monitor.check(**self.samples())
            if trigger is not None:
                break
        self.assertIsNotNone(trigger)
        self.assertEqual(trigger.name, "arm_not_moving")
        self.assertIsNone(monitor.motion_warning)

    def test_existing_direct_backend_callers_keep_stop_enabled(self):
        backend = RM65RealBackend("fake", sdk=StalledFakeArm())
        self.assertTrue(backend._monitor.arm_not_moving_stop)

    def test_sender_warns_and_continues_or_stops_when_opted_in(self):
        for stop_enabled in (False, True):
            with self.subTest(stop_enabled=stop_enabled):
                fake = StalledFakeArm()
                events = []
                backend = RM65RealBackend(
                    "fake", sdk=fake, arm_not_moving_stop=stop_enabled,
                    on_event=lambda name, detail: events.append((name, detail)),
                )
                try:
                    backend.connect()
                    # Stay inside the independent 0.35 rad gross-error bound.
                    target = np.full(6, 0.25)
                    backend.send_joint_target(target)
                    backend.start()
                    backend.set_enabled(True)
                    deadline = time.monotonic() + 0.8
                    while time.monotonic() < deadline:
                        backend.send_joint_target(target)
                        time.sleep(0.01)
                    self.assertGreater(len(fake.canfd_calls), 0)
                    warnings = [event for event in events if event[0] == "arm_not_moving_warning"]
                    if stop_enabled:
                        self.assertFalse(backend.enabled)
                        self.assertGreaterEqual(fake.slow_stop_count, 1)
                        self.assertTrue(any(
                            name == "disarm" and detail.startswith("arm_not_moving/")
                            for name, detail in events
                        ))
                        self.assertEqual(warnings, [])
                    else:
                        self.assertTrue(backend.enabled)
                        self.assertEqual(fake.slow_stop_count, 0)
                        self.assertEqual(len(warnings), 1)  # no 100 Hz log flood
                        sent_before = len(fake.canfd_calls)
                        backend.send_joint_target(target)
                        time.sleep(0.04)
                        self.assertGreater(len(fake.canfd_calls), sent_before)
                finally:
                    backend.disconnect()


if __name__ == "__main__":
    unittest.main()
