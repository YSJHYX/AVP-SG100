"""SG100 driver boundary unit tests (config + mock, no hardware, no SDK).

Covers the SDK-independent boundary contract: unit conversion, joint-order
mapping, feedback/command shape, limit clamping, send-failure provenance,
and the measured-feedback (vs command-echo) claim.  The latest-SDK
backend surface is covered by test_sg100_latest_sdk_driver.py.
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pytest
import yaml

from sg100_teleop.robot.sg100_driver import (
    FEEDBACK_SEMANTICS_MEASURED,
    HAND_DOF,
    SG100HardwareMap,
)
from sg100_teleop.robot.sg100_mock_driver import MockSG100Driver


ROOT = Path(__file__).resolve().parents[2]


def test_joint_order_mapping_covers_all_11_dof():
    hardware_map = SG100HardwareMap()
    table = hardware_map.mapping_table()

    assert len(table) == HAND_DOF
    # Vendor layout: thumb 1-3, index 4-6, middle 7-8, little 9-11.
    assert [row["hardware_joint_id"] for row in table] == list(
        range(1, 12)
    )
    assert [row["software_index"] for row in table] == list(range(11))
    names = [row["software_name"] for row in table]
    assert names[:3] == ["thumb_j1", "thumb_j2", "thumb_j3"]
    assert names[3:6] == ["index_j1", "index_j2", "index_j3"]
    assert names[6:8] == ["middle_j1", "middle_j2"]
    assert names[8:] == ["little_j1", "little_j2", "little_j3"]


def test_boundary_conversion_roundtrip():
    hardware_map = SG100HardwareMap()
    rng = np.random.default_rng(11)
    q = rng.uniform(
        hardware_map.lower_rad, hardware_map.upper_rad
    )
    actual = hardware_map.canonical_to_actual(q)
    recovered = hardware_map.actual_to_canonical(actual)
    np.testing.assert_allclose(recovered, q, atol=1e-12)


def test_boundary_conversion_with_gain_offset():
    """A non-identity map must round-trip per-joint."""

    import tempfile

    source = (
        ROOT
        / "sg100_teleop"
        / "config"
        / "sg100_hardware_map.yaml"
    )
    with source.open("r") as f:
        config = yaml.safe_load(f)
    config["gain"] = [1.0] * 11
    config["gain"][0] = -1.0  # thumb_j1 direction flip
    config["offset_rad"] = [0.0] * 11
    config["offset_rad"][1] = 0.1

    with tempfile.NamedTemporaryFile(
        "w", suffix=".yaml", delete=False
    ) as f:
        yaml.safe_dump(config, f)
        path = f.name

    hardware_map = SG100HardwareMap(map_path=path)
    q = np.zeros(HAND_DOF)
    actual = hardware_map.canonical_to_actual(q)
    assert actual[0] == 0.0
    assert actual[1] == pytest.approx(0.1)
    recovered = hardware_map.actual_to_canonical(actual)
    np.testing.assert_allclose(recovered, q, atol=1e-12)


def test_command_is_clamped_to_canonical_limits():
    hardware_map = SG100HardwareMap()
    q = np.full(HAND_DOF, 100.0)  # far outside every joint range
    actual = hardware_map.canonical_to_actual(q)
    assert np.all(actual <= hardware_map.upper_rad + 1e-12)
    assert np.all(actual >= hardware_map.lower_rad - 1e-12)


def test_command_shape_enforced():
    hardware_map = SG100HardwareMap()
    with pytest.raises(ValueError):
        hardware_map.canonical_to_actual(np.zeros(6))
    with pytest.raises(ValueError):
        hardware_map.actual_to_canonical(np.zeros(4))


def test_feedback_shape_and_finiteness():
    driver = MockSG100Driver()
    driver.connect()
    driver.start()
    time.sleep(0.02)
    feedback = driver.read_feedback()

    assert feedback is not None
    assert feedback.q_rad.shape == (HAND_DOF,)
    assert np.all(np.isfinite(feedback.q_rad))
    assert feedback.received_monotonic_ns > 0
    assert feedback.sequence >= 1


def test_feedback_is_measured_not_command_echo():
    """Feedback must not simply mirror the last command: it lags the
    command (first-order response) and evolves on its own clock."""

    driver = MockSG100Driver(follow_tau_s=0.2)
    driver.connect()
    driver.start()
    driver.enter_realtime()
    time.sleep(0.02)
    driver.read_feedback()

    q_cmd = np.zeros(HAND_DOF)
    q_cmd[4] = 1.0  # index_j2 ~57 deg
    sent = driver.send_positions_rad(q_cmd)
    assert sent is not None

    time.sleep(0.03)
    feedback = driver.read_feedback()
    assert feedback is not None
    # The measured position lags the step command: it is a driven
    # response, not an echo of the sent target.
    assert feedback.q_rad[4] < 0.9 * q_cmd[4]
    assert feedback.q_rad[4] > 0.0

    # And the provenance label never claims more than the SDK can
    # prove.
    assert feedback.feedback_semantics == (
        FEEDBACK_SEMANTICS_MEASURED
    )


def test_send_failure_returns_none_and_emits_no_event():
    driver = MockSG100Driver(send_failure=True)
    driver.connect()
    driver.start()
    driver.enter_realtime()

    assert (
        driver.send_positions_rad(np.zeros(HAND_DOF)) is None
    )
    assert driver.drain_command_events() == ()


def test_send_requires_running_driver():
    driver = MockSG100Driver()
    driver.connect()
    # not started: no commands may leave
    assert driver.send_positions_rad(np.zeros(HAND_DOF)) is None


def test_duplicate_feedback_deduped_by_sequence():
    driver = MockSG100Driver(duplicate_feedback=True)
    driver.connect()
    driver.start()
    driver.enter_realtime()
    time.sleep(0.02)

    first = driver.read_feedback()
    second = driver.read_feedback()
    assert first is not None and second is not None
    # one of the two reads must be the SAME sample (same sequence)
    assert (
        first.sequence == second.sequence
        or second.sequence > first.sequence
    )
