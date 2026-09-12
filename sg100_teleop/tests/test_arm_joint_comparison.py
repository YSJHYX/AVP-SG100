"""Read-only joint-comparison regressions; no hardware or GUI is started."""

from dataclasses import replace
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from sg100_teleop.recording.arm_joint_comparison import (
    ARM_JOINT_COMPARISON_LOG,
    ArmJointComparisonRecorder,
    build_arm_joint_comparison,
)
from sg100_teleop.robot.arm_state import ArmCommandSent, ArmState


NOW_NS = 1_000_000_000


def samples():
    feedback = ArmState(
        q_rad=np.deg2rad([10., 20., 30., 40., 50., 60.]),
        qd_rad_s=None,
        received_monotonic_ns=NOW_NS - 20_000_000,
        sequence=17,
        controller_error=0,
        arm_status=None,
        valid=True,
    )
    command = ArmCommandSent(
        q_rad=feedback.q_rad + np.deg2rad([1., -2., 3., -4., 5., -6.]),
        # Deliberately unlike the actually sent post-limiter command.
        limiter_input_q_rad=np.full(6, 2.0),
        sent_monotonic_ns=NOW_NS - 5_000_000,
        source_timestamp_ns=NOW_NS - 25_000_000,
        target_sequence=23,
        command_bundle_id=31,
        command_sequence=42,
    )
    return feedback, command


class CacheOnlyBackend:
    enabled = False

    def __init__(self, command):
        self.command = command
        self.cache_reads = 0

    def latest_command_sent(self):
        self.cache_reads += 1
        return self.command

    def __getattr__(self, name):
        raise AssertionError(f"diagnostics must not call backend.{name}")


class TestArmJointComparison(unittest.TestCase):
    def record(self, **overrides):
        feedback, command = samples()
        kwargs = dict(
            snapshot_monotonic_ns=NOW_NS,
            mujoco_qpos_rad=feedback.q_rad + np.deg2rad([0., .1, 0., -.2, 0., 0.]),
            feedback=feedback,
            command=command,
            arm_enabled=True,
            feedback_timeout_ms=200.,
        )
        return build_arm_joint_comparison(**(kwargs | overrides))

    def test_signed_per_joint_differences_units_and_timestamps(self):
        row = self.record()
        self.assertEqual(row["joint_names"], [f"joint_{i}" for i in range(1, 7)])
        np.testing.assert_allclose(row["mujoco_minus_feedback_deg"], [0., .1, 0., -.2, 0., 0.], atol=1e-12)
        np.testing.assert_allclose(row["sent_minus_feedback_deg"], [1., -2., 3., -4., 5., -6.], atol=1e-12)
        np.testing.assert_allclose(row["feedback_q_deg"], [10., 20., 30., 40., 50., 60.])
        self.assertAlmostEqual(row["max_abs_mujoco_minus_feedback_deg"], .2)
        self.assertAlmostEqual(row["max_abs_sent_minus_feedback_deg"], 6.)
        self.assertEqual(row["feedback_age_ms"], 20.)
        self.assertEqual(row["sent_age_ms"], 5.)
        self.assertEqual(row["feedback_minus_sent_time_ms"], -15.)
        self.assertEqual(row["sent_command_sequence"], 42)
        self.assertTrue(row["comparison_valid"])
        json.dumps(row, allow_nan=False)

    def test_missing_command_is_null_not_a_zero_target(self):
        row = self.record(command=None, arm_enabled=False)
        self.assertIsNone(row["sent_q_rad"])
        self.assertIsNone(row["sent_minus_feedback_deg"])
        self.assertIsNone(row["sent_age_ms"])
        self.assertIsNone(row["max_abs_sent_minus_feedback_deg"])
        self.assertFalse(row["arm_enabled"])
        self.assertTrue(row["comparison_valid"])

    def test_missing_stale_invalid_and_future_feedback_are_not_valid_comparisons(self):
        feedback, _ = samples()
        for state in (
            None,
            replace(feedback, received_monotonic_ns=NOW_NS - 201_000_000),
            replace(feedback, valid=False),
            replace(feedback, received_monotonic_ns=NOW_NS + 1),
        ):
            with self.subTest(state=state):
                row = self.record(feedback=state)
                self.assertFalse(row["comparison_valid"])
                if state is None:
                    self.assertIsNone(row["mujoco_minus_feedback_deg"])
                    self.assertIsNone(row["feedback_q_rad"])
                    self.assertIsNone(row["feedback_age_ms"])

    def test_snapshots_own_data_and_reject_malformed_vectors(self):
        feedback, command = samples()
        qpos = feedback.q_rad.copy()
        row = self.record(mujoco_qpos_rad=qpos, feedback=feedback, command=command)
        saved = json.dumps(row)
        qpos[:] = 0.
        feedback.q_rad[:] = 1.
        command.q_rad[:] = 2.
        self.assertEqual(saved, json.dumps(row))
        for bad in (np.zeros(5), np.full(6, np.nan), np.full(6, np.inf)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.record(mujoco_qpos_rad=bad)

    def test_wall_clock_throttle_reads_only_cache_and_does_not_mutate_inputs(self):
        feedback, command = samples()
        backend = CacheOnlyBackend(command)
        indices = np.array([1, 3, 5, 7, 9, 11])
        qpos = np.full(13, -99.)
        qpos[indices] = feedback.q_rad + .001
        original_qpos = qpos.copy()
        original_feedback = feedback.q_rad.copy()
        original_command = command.q_rad.copy()
        with tempfile.TemporaryDirectory() as directory:
            recorder = ArmJointComparisonRecorder(directory)
            kwargs = dict(mujoco_qpos=qpos, arm_qpos_indices=indices, feedback=feedback, arm_backend=backend)
            try:
                with patch("sg100_teleop.recording.arm_joint_comparison.time.monotonic_ns", return_value=NOW_NS):
                    self.assertTrue(recorder.maybe_record(**kwargs))
                    self.assertFalse(recorder.maybe_record(**kwargs))
                with patch("sg100_teleop.recording.arm_joint_comparison.time.monotonic_ns", return_value=NOW_NS + 499_999_999):
                    self.assertFalse(recorder.maybe_record(**kwargs))
                with patch("sg100_teleop.recording.arm_joint_comparison.time.monotonic_ns", return_value=NOW_NS + 500_000_000):
                    self.assertTrue(recorder.maybe_record(**kwargs))
            finally:
                self.assertTrue(recorder.close())
            rows = [json.loads(line) for line in (Path(directory) / ARM_JOINT_COMPARISON_LOG).read_text().splitlines()]
            self.assertEqual(len(rows), 2)
            self.assertEqual(backend.cache_reads, 2)
            self.assertEqual(recorder.written_samples, 2)
            self.assertEqual(recorder.dropped_samples, 0)
            self.assertFalse(rows[1]["feedback_fresh"])
            np.testing.assert_allclose(rows[0]["mujoco_qpos_rad"], original_qpos[indices])
            self.assertFalse(recorder.maybe_record(**kwargs))
        np.testing.assert_array_equal(qpos, original_qpos)
        np.testing.assert_array_equal(feedback.q_rad, original_feedback)
        np.testing.assert_array_equal(command.q_rad, original_command)
        self.assertFalse(backend.enabled)

    def test_full_queue_drops_without_waiting_and_close_is_bounded(self):
        entered_write = threading.Event()
        release_write = threading.Event()

        class SlowFile:
            def __enter__(self): return self
            def __exit__(self, *args): return False
            def flush(self): pass
            def write(self, text):
                entered_write.set()
                release_write.wait(timeout=2.)

        feedback, command = samples()
        backend = CacheOnlyBackend(command)
        kwargs = dict(mujoco_qpos=feedback.q_rad.copy(), arm_qpos_indices=np.arange(6), feedback=feedback, arm_backend=backend)
        with tempfile.TemporaryDirectory() as directory, patch.object(Path, "open", return_value=SlowFile()):
            recorder = ArmJointComparisonRecorder(directory, rate_hz=10., queue_size=1)
            try:
                with patch("sg100_teleop.recording.arm_joint_comparison.time.monotonic_ns", return_value=NOW_NS):
                    self.assertTrue(recorder.maybe_record(**kwargs))
                self.assertTrue(entered_write.wait(timeout=1.))
                with patch("sg100_teleop.recording.arm_joint_comparison.time.monotonic_ns", return_value=NOW_NS + 100_000_000):
                    self.assertTrue(recorder.maybe_record(**kwargs))
                with patch("sg100_teleop.recording.arm_joint_comparison.time.monotonic_ns", return_value=NOW_NS + 200_000_000):
                    self.assertFalse(recorder.maybe_record(**kwargs))
                self.assertEqual(recorder.dropped_samples, 1)
                started = time.perf_counter()
                self.assertFalse(recorder.close(timeout_s=.01))
                self.assertLess(time.perf_counter() - started, .2)
            finally:
                release_write.set()
                self.assertTrue(recorder.close(timeout_s=1.))

    def test_file_failure_and_cache_failure_do_not_escape_to_control_loop(self):
        with tempfile.TemporaryDirectory() as directory:
            # Opening a directory as a log file reliably fails, without
            # changing filesystem permissions or depending on the test UID.
            (Path(directory) / ARM_JOINT_COMPARISON_LOG).mkdir()
            recorder = ArmJointComparisonRecorder(directory)
            self.assertTrue(recorder.close())
            self.assertTrue(recorder._writer_failed.is_set())
            self.assertIsNotNone(recorder.last_error)
        feedback, _ = samples()
        with tempfile.TemporaryDirectory() as directory:
            recorder = ArmJointComparisonRecorder(directory)
            try:
                self.assertFalse(recorder.maybe_record(
                    mujoco_qpos=feedback.q_rad, arm_qpos_indices=np.arange(6),
                    feedback=feedback, arm_backend=object(),
                ))
                self.assertEqual(recorder.dropped_samples, 1)
                self.assertIn("latest_command_sent", recorder.last_error)
            finally:
                self.assertTrue(recorder.close())

    def test_cli_defaults_off_and_accepts_explicit_switch_and_rate(self):
        from sg100_teleop.sim.run_avp_arm_hand_sim import build_arg_parser

        parser = build_arg_parser()
        base = ["--ip", "fake-avp"]
        self.assertFalse(parser.parse_args(base).arm_joint_compare)
        self.assertEqual(parser.parse_args(base).arm_joint_compare_hz, 2.)
        self.assertTrue(parser.parse_args(base + ["--arm-joint-compare"]).arm_joint_compare)
        self.assertFalse(parser.parse_args(base + ["--no-arm-joint-compare"]).arm_joint_compare)
        self.assertEqual(parser.parse_args(base + ["--arm-joint-compare-hz", "5"]).arm_joint_compare_hz, 5.)

    def test_log_thread_start_failure_does_not_escape_or_require_cleanup(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(
            threading.Thread, "start", side_effect=RuntimeError("no log thread")
        ):
            recorder = ArmJointComparisonRecorder(directory)
            self.assertTrue(recorder._writer_failed.is_set())
            self.assertIn("no log thread", recorder.last_error)
            self.assertTrue(recorder.close())
            self.assertFalse(recorder.path.exists())

    def test_invalid_cli_is_rejected_before_hardware_initialization(self):
        from sg100_teleop.sim.run_avp_arm_hand_sim import main

        base = ["run_avp_arm_hand_sim", "--ip", "fake-avp"]
        cases = [(["--arm-joint-compare"], "requires --arm-backend real")]
        cases += [(["--arm-joint-compare-hz", rate], "must be finite") for rate in ("0", "-1", "11", "nan", "inf", "1e-300")]
        for extra, message in cases:
            with self.subTest(extra=extra), patch("sys.argv", base + extra):
                with self.assertRaisesRegex(ValueError, message):
                    main()


if __name__ == "__main__":
    unittest.main()
