"""SG100 measured-feedback DATA CONTRACT tests (MockSG100Driver).

Covers the real-hardware hand semantics restored in Phase H1:
state/next_state from MEASURED feedback, canonical action from the
hardware-sent qcmd, intervening-command rejection, and the
power-grasp lock provenance (raw 60 deg -> final 100 deg).
"""

from __future__ import annotations

import json
import tempfile
import time
from pathlib import Path

import numpy as np
import pytest

from sg100_teleop.data_collection.collector import (
    ArmHandDataCollector,
    CollectorConfig,
)
from sg100_teleop.data_collection.schema import RejectionReason
from sg100_teleop.hand.canonical_hand import CanonicalHandState
from sg100_teleop.retargeting.sg100_retargeter import SG100Retargeter
from sg100_teleop.robot.arm_state import ArmCommandSent, ArmState
from sg100_teleop.robot.sg100_mock_driver import MockSG100Driver


ARM_DIM = 6


def arm_feedback(timestamp_ns: int, sequence: int, q) -> ArmState:
    return ArmState(
        q_rad=np.asarray(q, dtype=np.float64),
        qd_rad_s=np.full(ARM_DIM, 0.0),
        received_monotonic_ns=timestamp_ns,
        sequence=sequence,
        controller_error=0,
        arm_status=0,
        valid=True,
        controller_tcp=None,
    )


def arm_sent(bundle: int, timestamp_ns: int, sequence: int, q):
    return ArmCommandSent(
        q_rad=np.asarray(q, dtype=np.float64),
        limiter_input_q_rad=np.asarray(q, dtype=np.float64),
        sent_monotonic_ns=timestamp_ns,
        source_timestamp_ns=timestamp_ns - 5,
        target_sequence=sequence,
        command_bundle_id=bundle,
    )


def make_collector(directory: Path, **driver_kwargs):
    driver = MockSG100Driver(**driver_kwargs)
    driver.connect()
    driver.start()
    # Realtime entry gates command acceptance in the driver contract
    # (MODE_NULL hold before entry); measured data collection requires it.
    driver.enter_realtime()
    collector = ArmHandDataCollector(
        directory,
        arm_lower_rad=np.full(ARM_DIM, -3.2),
        arm_upper_rad=np.full(ARM_DIM, 3.2),
        hand_lower_rad=np.full(11, -3.2),
        hand_upper_rad=np.full(11, 3.2),
        hand_measured_feedback=True,
        config=CollectorConfig(
            stale_pre_state_ns=10_000_000_000,
            stale_post_state_ns=10_000_000_000,
            stale_pre_hand_state_ns=10_000_000_000,
            stale_post_hand_state_ns=10_000_000_000,
        ),
    )
    return collector, driver


def pump_hand_feedback(collector, driver, seconds: float = 0.03):
    """Poll the mock driver briefly so fresh measured samples exist.

    Returns (first, last) observed feedback this call.
    """

    deadline = time.monotonic() + seconds
    first = None
    last = None
    while time.monotonic() < deadline:
        feedback = driver.read_feedback()
        if feedback is not None:
            collector.observe_hand_feedback(feedback)
            if first is None:
                first = feedback
            last = feedback
        time.sleep(0.005)
    return first, last


def run_happy_transition(collector, driver, *, q_hand):
    """Drive one full decision -> arm+hand send -> post feedback cycle."""

    pump_hand_feedback(collector, driver, 0.03)
    collector.observe_arm_feedback(
        arm_feedback(time.monotonic_ns(), 100, np.full(ARM_DIM, 0.1))
    )

    bundle = collector.next_command_bundle_id()
    decision_ns = time.monotonic_ns()
    collector.record_control_decision(
        command_bundle_id=bundle,
        decision_timestamp_ns=decision_ns,
        hand_command_applied_timestamp_ns=decision_ns + 1,
        hand_qcmd_rad=q_hand,
    )

    sent = driver.send_positions_rad(q_hand, command_bundle_id=bundle)
    assert sent is not None
    collector.ingest_hand_command_events(driver.drain_command_events())

    sent_ns = time.monotonic_ns()
    collector.ingest_arm_command_events([
        arm_sent(bundle, sent_ns, 200 + bundle, np.full(ARM_DIM, 0.2))
    ])

    pump_hand_feedback(collector, driver, 0.05)
    collector.observe_arm_feedback(
        arm_feedback(time.monotonic_ns(), 300, np.full(ARM_DIM, 0.3))
    )
    return bundle


# ----------------------------------------------------------------------
# A/B/C — happy causal chain with measured feedback
# ----------------------------------------------------------------------


def test_measured_transition_happy_path():
    with tempfile.TemporaryDirectory() as directory:
        collector, driver = make_collector(Path(directory))
        q_hand = np.zeros(11)
        q_hand[4] = 0.5
        bundle = run_happy_transition(collector, driver, q_hand=q_hand)

        assert len(collector.transitions) == 1
        transition = collector.transitions[0]
        assert transition.command_bundle_id == bundle
        # state/action/next_state hand slices are 11D and finite
        for vector in (
            transition.state,
            transition.action,
            transition.next_state,
        ):
            assert vector.shape == (17,)
            assert np.all(np.isfinite(vector))
        # canonical action hand == hardware-sent q
        np.testing.assert_allclose(
            transition.action[6:], q_hand, atol=1e-9
        )
        # measured state differs from the action (physical lag) but
        # is NOT fabricated from the command
        assert not np.allclose(
            transition.state[6:], q_hand, atol=1e-9
        )
        driver.disconnect()


def test_state_and_next_state_come_from_measured_feedback():
    with tempfile.TemporaryDirectory() as directory:
        collector, driver = make_collector(Path(directory))
        q_hand = np.zeros(11)
        q_hand[4] = 0.8
        run_happy_transition(collector, driver, q_hand=q_hand)

        transition = collector.transitions[0]
        state_hand = transition.state[6:]
        next_hand = transition.next_state[6:]
        # the hand moved TOWARD the command between state and
        # next_state (first-order response), proving both ends are
        # measurements around the action
        assert next_hand[4] > state_hand[4]
        driver.disconnect()


# ----------------------------------------------------------------------
# D — intervening hand command
# ----------------------------------------------------------------------


def test_intervening_hand_command_rejected():
    with tempfile.TemporaryDirectory() as directory:
        collector, driver = make_collector(Path(directory))
        pump_hand_feedback(collector, driver, 0.03)
        collector.observe_arm_feedback(
            arm_feedback(time.monotonic_ns(), 100, np.full(ARM_DIM, 0.1))
        )

        for bundle in (1, 2):
            decision_ns = time.monotonic_ns()
            collector.record_control_decision(
                command_bundle_id=bundle,
                decision_timestamp_ns=decision_ns,
                hand_command_applied_timestamp_ns=decision_ns + 1,
                hand_qcmd_rad=np.full(11, 0.1 * bundle),
            )

        q1 = np.full(11, 0.1)
        q2 = np.full(11, 0.2)
        driver.send_positions_rad(q1, command_bundle_id=1)
        collector.ingest_hand_command_events(
            driver.drain_command_events()
        )
        collector.ingest_arm_command_events([
            arm_sent(1, time.monotonic_ns(), 201, np.full(ARM_DIM, 0.2))
        ])

        # bundle 2's command lands between bundle 1's action and the
        # first post-action hand feedback -> bundle 1 must be rejected
        time.sleep(0.02)
        driver.send_positions_rad(q2, command_bundle_id=2)
        collector.ingest_hand_command_events(
            driver.drain_command_events()
        )
        collector.ingest_arm_command_events([
            arm_sent(2, time.monotonic_ns(), 202, np.full(ARM_DIM, 0.25))
        ])

        pump_hand_feedback(collector, driver, 0.08)
        collector.observe_arm_feedback(
            arm_feedback(time.monotonic_ns(), 300, np.full(ARM_DIM, 0.3))
        )

        reasons = {
            item.reason for item in collector.rejections
        }
        assert RejectionReason.INTERVENING_HAND_COMMAND in reasons
        bundle_ids = {
            item.command_bundle_id
            for item in collector.rejections
            if item.reason
            is RejectionReason.INTERVENING_HAND_COMMAND
        }
        assert bundle_ids == {1}
        driver.disconnect()


# ----------------------------------------------------------------------
# E — send failure never becomes canonical
# ----------------------------------------------------------------------


def test_send_failure_not_canonical():
    with tempfile.TemporaryDirectory() as directory:
        collector, driver = make_collector(
            Path(directory), send_failure=True
        )
        pump_hand_feedback(collector, driver, 0.03)
        collector.observe_arm_feedback(
            arm_feedback(time.monotonic_ns(), 100, np.full(ARM_DIM, 0.1))
        )
        bundle = collector.next_command_bundle_id()
        decision_ns = time.monotonic_ns()
        collector.record_control_decision(
            command_bundle_id=bundle,
            decision_timestamp_ns=decision_ns,
            hand_command_applied_timestamp_ns=decision_ns + 1,
            hand_qcmd_rad=np.zeros(11),
        )
        assert (
            driver.send_positions_rad(
                np.zeros(11), command_bundle_id=bundle
            )
            is None
        )
        collector.ingest_arm_command_events([
            arm_sent(bundle, time.monotonic_ns(), 201, np.full(ARM_DIM, 0.2))
        ])
        pump_hand_feedback(collector, driver, 0.05)
        collector.observe_arm_feedback(
            arm_feedback(time.monotonic_ns(), 300, np.full(ARM_DIM, 0.3))
        )
        collector.finalize()

        assert collector.transitions == []
        reasons = {
            item.reason for item in collector.rejections
        }
        assert RejectionReason.HAND_COMMAND_NOT_SENT in reasons
        driver.disconnect()


# ----------------------------------------------------------------------
# F — stale feedback
# ----------------------------------------------------------------------


def test_stale_hand_feedback_rejected():
    """A hand feedback gap beyond the configured post-action staleness
    rejects the transition instead of pairing distant measurements."""

    with tempfile.TemporaryDirectory() as directory:
        collector, driver = make_collector(Path(directory))
        collector.config = CollectorConfig(
            stale_pre_state_ns=10_000_000_000,
            stale_post_state_ns=10_000_000_000,
            stale_pre_hand_state_ns=10_000_000_000,
            stale_post_hand_state_ns=15_000_000,  # 15 ms post window
        )
        pump_hand_feedback(collector, driver, 0.03)
        collector.observe_arm_feedback(
            arm_feedback(time.monotonic_ns(), 100, np.full(ARM_DIM, 0.1))
        )
        q_hand = np.zeros(11)
        bundle = collector.next_command_bundle_id()
        decision_ns = time.monotonic_ns()
        collector.record_control_decision(
            command_bundle_id=bundle,
            decision_timestamp_ns=decision_ns,
            hand_command_applied_timestamp_ns=decision_ns + 1,
            hand_qcmd_rad=q_hand,
        )
        sent = driver.send_positions_rad(
            q_hand, command_bundle_id=bundle
        )
        assert sent is not None
        arm_sent_ns = time.monotonic_ns()
        time.sleep(0.05)  # post-action gap >> 15 ms
        collector.ingest_hand_command_events(
            driver.drain_command_events()
        )
        collector.ingest_arm_command_events([
            arm_sent(
                bundle,
                arm_sent_ns,
                201,
                np.full(ARM_DIM, 0.2),
            )
        ])
        pump_hand_feedback(collector, driver, 0.05)
        collector.observe_arm_feedback(
            arm_feedback(time.monotonic_ns(), 300, np.full(ARM_DIM, 0.3))
        )

        reasons = {
            item.reason for item in collector.rejections
        }
        assert RejectionReason.STALE_POST_HAND_STATE in reasons
        assert collector.transitions == []
        driver.disconnect()


# ----------------------------------------------------------------------
# G — NaN feedback
# ----------------------------------------------------------------------


def test_nan_hand_feedback_rejected():
    with tempfile.TemporaryDirectory() as directory:
        collector, driver = make_collector(
            Path(directory), nan_feedback=True
        )
        deadline = time.monotonic() + 0.5
        saw_rejection = False
        while time.monotonic() < deadline and not saw_rejection:
            feedback = driver.read_feedback()
            if feedback is not None:
                collector.observe_hand_feedback(feedback)
            saw_rejection = any(
                item.reason is RejectionReason.HAND_FEEDBACK_INVALID
                for item in collector.rejections
            )
            time.sleep(0.005)

        assert saw_rejection
        driver.disconnect()


# ----------------------------------------------------------------------
# H — out-of-order timestamps
# ----------------------------------------------------------------------


def test_out_of_order_hand_feedback_rejected():
    with tempfile.TemporaryDirectory() as directory:
        collector, driver = make_collector(
            Path(directory), time_reversal=True
        )
        deadline = time.monotonic() + 1.0
        saw_rejection = False
        while time.monotonic() < deadline and not saw_rejection:
            feedback = driver.read_feedback()
            if feedback is not None:
                collector.observe_hand_feedback(feedback)
            saw_rejection = any(
                item.reason is RejectionReason.NON_MONOTONIC_TIMESTAMP
                for item in collector.rejections
            )
            time.sleep(0.005)

        assert saw_rejection
        driver.disconnect()


# ----------------------------------------------------------------------
# I/J + §37 — power-grasp lock provenance through the whole chain
# ----------------------------------------------------------------------


NORMAL_GRASP = CanonicalHandState(
    thumb_opposition=0.7595,
    thumb_abduction=0.8610,
    thumb_distal=0.918,
    index_adduction=0.0,
    index_flexion=0.9795,
    index_distal=0.8,
    middle_flexion=1.0,
    middle_distal=0.8,
    little_outward=0.0,
    little_flexion=1.0,
    little_distal=0.8,
)


def test_power_grasp_final_qcmd_differs_from_raw():
    retargeter = SG100Retargeter()
    final, stages = retargeter.retarget_with_stages(NORMAL_GRASP)
    i1 = retargeter.joint_order.index("thumb_j1")

    raw_deg = np.degrees(stages["raw_retarget_qcmd"][i1])
    final_deg = np.degrees(final[i1])

    assert raw_deg == pytest.approx(59.58, abs=0.1)
    assert final_deg == pytest.approx(99.57, abs=0.1)
    assert final_deg - raw_deg == pytest.approx(40.0, abs=0.2)
    np.testing.assert_allclose(
        stages["final_qcmd"], final, atol=1e-12
    )
    assert stages["power_grasp_lock"]["lock_weight"] > 0.99


def test_canonical_action_is_final_sent_qcmd_not_raw():
    """§37: raw thumb_j1 = 60 deg, lock = +40 deg -> the dataset's
    canonical action MUST be ~100 deg (hardware-sent), with the raw
    60 deg kept only as audit."""

    with tempfile.TemporaryDirectory() as directory:
        collector, driver = make_collector(Path(directory))

        retargeter = SG100Retargeter()
        final, stages = retargeter.retarget_with_stages(NORMAL_GRASP)
        collector.record_hand_pipeline_stages(stages)

        run_happy_transition(collector, driver, q_hand=final)
        collector.finalize()

        transition = collector.transitions[0]
        i1 = 0  # thumb_j1 is joint_order[0]
        action_deg = np.degrees(transition.action[6 + i1])
        assert action_deg == pytest.approx(99.57, abs=0.2)
        assert abs(action_deg - 59.58) > 1.0

        # audit stream keeps the raw 60 deg value
        raw_path = (
            Path(directory) / "arm_hand_raw_events.jsonl"
        )
        events = [
            json.loads(line)
            for line in raw_path.read_text().splitlines()
        ]
        raw_events = [
            event
            for event in events
            if event["event_type"] == "hand_raw_retarget"
        ]
        assert raw_events
        raw_deg = np.degrees(
            np.asarray(raw_events[-1]["payload"]["raw_retarget_qcmd"])[
                i1
            ]
        )
        assert raw_deg == pytest.approx(59.58, abs=0.1)

        # hardware sender input equals the canonical action
        sent_events = [
            event
            for event in events
            if event["event_type"] == "hand_hardware_sent"
        ]
        assert sent_events
        sent_deg = np.degrees(
            np.asarray(sent_events[-1]["payload"]["q_rad"])[i1]
        )
        assert sent_deg == pytest.approx(99.57, abs=0.2)

        manifest = json.loads(
            (
                Path(directory) / "arm_hand_manifest.json"
            ).read_text()
        )
        assert (
            manifest["hand_state_source"]
            == "sg100_measured_joint_feedback"
        )
        assert (
            manifest["hand_action_source"]
            == "final_hardware_sent_qcmd"
        )
        assert manifest["hand_hardware_present"] is True
        assert manifest["hand_feedback_available"] is True
        assert manifest["hand_feedback_valid"] is True
        assert manifest["hand_action_hardware_applied"] is False
        assert manifest["hand_action_hardware_sent"] is True
        assert manifest["writer_version"] == 2

        arrays = np.load(
            Path(directory) / "arm_hand_joint17_v1.npz"
        )
        assert arrays["hand_state_measured"].all()
        assert arrays["hand_action_hardware_sent"].all()
        driver.disconnect()


def test_baseline_is_measured_replay_seed_not_first_state():
    """The frozen V1 bug required baseline == first accepted state.
    In measured mode the baseline is just the first measured sample
    (replay seed) and transitions build independently of it."""

    with tempfile.TemporaryDirectory() as directory:
        collector, driver = make_collector(Path(directory))
        first, _ = pump_hand_feedback(collector, driver, 0.03)
        assert first is not None

        assert collector.hand_proxy_baseline_valid is False  # proxy n/a
        run_happy_transition(
            collector, driver, q_hand=np.zeros(11)
        )
        collector.finalize()

        manifest = json.loads(
            (
                Path(directory) / "arm_hand_manifest.json"
            ).read_text()
        )
        assert (
            manifest["hand_initial_state_source"]
            == "first_measured_feedback_replay_seed"
        )
        # The first measured baseline is an AUDIT value; nothing in
        # the canonical arrays is required to equal it.
        assert (
            manifest["hand_measured_baseline_timestamp_ns"]
            == first.received_monotonic_ns
        )
        driver.disconnect()


def test_proxy_mode_unchanged_by_measured_mode():
    """No hand driver -> collector stays in V1 proxy semantics."""

    with tempfile.TemporaryDirectory() as directory:
        collector = ArmHandDataCollector(
            Path(directory),
            initial_hand_qcmd=np.zeros(11),
            arm_lower_rad=np.full(ARM_DIM, -3.2),
            arm_upper_rad=np.full(ARM_DIM, 3.2),
            hand_lower_rad=np.full(11, -2.0),
            hand_upper_rad=np.full(11, 2.0),
            started_timestamp_ns=1,
        )
        assert collector.hand_measured_feedback is False
        # observing hand feedback in proxy mode is a no-op
        driver = MockSG100Driver()
        driver.connect()
        driver.start()
        time.sleep(0.02)
        collector.observe_hand_feedback(driver.read_feedback())
        collector.finalize()
        manifest = json.loads(
            (
                Path(directory) / "arm_hand_manifest.json"
            ).read_text()
        )
        assert (
            manifest["hand_state_source"] == "previous_command_proxy"
        )
        assert manifest["hand_hardware_present"] is False
        assert manifest["writer_version"] == 1
        driver.disconnect()


def test_inspector_reports_hand_real_validation():
    """Inspector consumes a measured episode end-to-end: HAND REAL
    VALIDATION block, SG100 REAL CAUSAL CHAIN, and legacy proxy
    episodes keep working in the same session type."""

    from sg100_teleop.data_collection.inspect_episode import (
        format_summary,
        inspect_episode,
    )

    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        collector, driver = make_collector(root)
        q_hand = np.zeros(11)
        q_hand[4] = 0.6
        run_happy_transition(collector, driver, q_hand=q_hand)
        collector.finalize()

        report = inspect_episode(root)
        summary = report.summary

        assert summary["sg100"]["measured_episode"] is True
        validation = summary["sg100"]["hand_real_validation"]
        assert validation is not None
        assert validation["feedback_samples"] >= 3
        assert validation["commands_sent"] >= 1
        assert validation["canonical_actions"] == 1
        assert validation["canonical_state_measured"] is True
        assert validation["command_raw_match"] == "PASS"
        assert validation["intervening_hand_commands"] == 0
        assert (
            summary["result"]["sg100_real_causal_chain_pass"] is True
        )

        text = format_summary(report)
        assert "HAND REAL VALIDATION" in text
        assert "SG100 REAL CAUSAL CHAIN: PASS" in text

        driver.disconnect()

    # Legacy proxy episode in the same session family: inspector must
    # label it, not crash.
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        collector = ArmHandDataCollector(
            root, initial_hand_qcmd=np.zeros(11), started_timestamp_ns=50
        )
        collector.observe_arm_feedback(
            arm_feedback(100, 1, np.full(ARM_DIM, 0.1))
        )
        collector.record_control_decision(
            command_bundle_id=1,
            decision_timestamp_ns=110,
            hand_command_applied_timestamp_ns=115,
            hand_qcmd_rad=np.full(11, 0.2),
        )
        collector.ingest_arm_command_events([
            arm_sent(1, 120, 1, np.full(ARM_DIM, 0.11))
        ])
        collector.observe_arm_feedback(
            arm_feedback(150, 2, np.full(ARM_DIM, 0.105))
        )
        collector.finalize()

        report = inspect_episode(root)
        text = format_summary(report)
        assert report.passed
        assert (
            "LEGACY HAND PROXY EPISODE" in text
            or "NOT APPLICABLE" in text
        )
        assert report.summary["sg100"]["measured_episode"] is False
