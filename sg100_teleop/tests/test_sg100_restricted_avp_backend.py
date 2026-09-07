"""Restricted AVP SG100 backend 的无硬件安全回归。"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from sg100_teleop.robot.sg100_driver import HAND_DOF, MODE_JOINT_POSITION
from sg100_teleop.robot.sg100_restricted_avp_backend import (
    SG100RestrictedAVPBackend,
)
from sg100_teleop.robot.sg100_sdk_driver import (
    DEFAULT_SDK_DIR, load_vendor_user_ranges,
)


class FakeDriver:
    def __init__(self):
        self.vendor_user_ranges = load_vendor_user_ranges(DEFAULT_SDK_DIR)
        self.zero_calls = []
        self.active_calls = []
        self.hold_calls = 0
        self.fault_active = False
        self.crc_count = 0
        self.feedback = SimpleNamespace(
            valid=True,
            received_monotonic_ns=1_000_000_000,
            joint_modes=(MODE_JOINT_POSITION,) * HAND_DOF,
        )

    @property
    def diagnostics(self):
        return {"crc_error_count": self.crc_count}

    def send_mode7_zero_output_sdk_targets(self, positions):
        self.zero_calls.append(np.asarray(positions).copy())
        return SimpleNamespace(sent_monotonic_ns=1)

    def send_mode7_active_allowlist(self, indices, positions):
        self.active_calls.append((tuple(indices), np.asarray(positions).copy()))
        return SimpleNamespace(sent_monotonic_ns=2)

    def hold_passive(self):
        self.hold_calls += 1
        return True

    def read_feedback(self):
        return self.feedback


def make_backend(**kwargs):
    driver = FakeDriver()
    backend = SG100RestrictedAVPBackend(
        driver, joint_allowlist=("index_j3",), **kwargs
    )
    return driver, backend


def test_real_allowlist_refuses_j7_until_its_hardware_gate_passes():
    with pytest.raises(ValueError, match="without real active-command evidence"):
        SG100RestrictedAVPBackend(FakeDriver(), joint_allowlist=("middle_j1",))


def test_c_anchor_is_zero_output_and_pre_enable_never_sends_active():
    driver, backend = make_backend()
    q = np.zeros(HAND_DOF)
    result = backend.submit(q, tracking_received_ns=1, now_ns=1, dt_s=0.01)
    assert result.reason == "C_ANCHOR_REQUIRED"
    assert not driver.active_calls
    assert backend.anchor(q).accepted
    result = backend.submit(q, tracking_received_ns=1, now_ns=1, dt_s=0.01)
    assert result.reason == "NOT_ENABLED_NO_ACTIVE_COMMAND"
    assert len(driver.zero_calls) == 1 and not driver.active_calls


def test_sim_and_real_receive_the_identical_canonical_qcmd_then_fcmd_once():
    driver, backend = make_backend(max_velocity_rad_s=10.0)
    q0 = np.zeros(HAND_DOF)
    assert backend.anchor(q0).accepted
    backend.set_enabled(True)
    candidate = q0.copy()
    candidate[5] = np.deg2rad(0.25)
    result = backend.submit(
        candidate, tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    )
    assert result.accepted
    np.testing.assert_allclose(result.canonical_qcmd_rad, candidate)
    assert driver.active_calls[-1][0] == (5,)
    assert driver.active_calls[-1][1][5] == pytest.approx(0.25)
    canonical = np.zeros(HAND_DOF)
    canonical[6] = np.deg2rad(-20.0)
    assert backend._validate_and_map(canonical)[6] == pytest.approx(20.0)


def test_velocity_slew_limiter_and_per_cycle_delta_guard():
    driver, backend = make_backend(max_velocity_rad_s=0.5)
    q0 = np.zeros(HAND_DOF)
    backend.anchor(q0)
    backend.set_enabled(True)
    target = q0.copy()
    target[5] = 0.02
    result = backend.submit(
        target, tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    )
    assert result.accepted and result.slew_limited
    assert np.deg2rad(driver.active_calls[-1][1][5]) == pytest.approx(0.005)

    driver, backend = make_backend(max_cycle_delta_rad=0.03)
    backend.anchor(q0)
    backend.set_enabled(True)
    target[5] = 0.04
    result = backend.submit(
        target, tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    )
    assert result.reason == "PER_CYCLE_DELTA_EXCEEDED"
    assert driver.hold_calls == 1 and not backend.enabled


def test_disabled_per_cycle_guard_uses_velocity_slew_limiter():
    driver, backend = make_backend(
        max_cycle_delta_rad=None,
        max_velocity_rad_s=0.5,
    )
    q0 = np.zeros(HAND_DOF)
    backend.anchor(q0)
    backend.set_enabled(True)
    target = q0.copy()
    target[5] = 0.1761
    result = backend.submit(
        target,
        tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000,
        dt_s=0.01,
    )
    assert result.accepted and result.slew_limited
    assert backend.enabled
    assert np.deg2rad(driver.active_calls[-1][1][5]) == pytest.approx(0.005)


@pytest.mark.parametrize(
    ("mutation", "reason"),
    [
        ("stale", "STALE_AVP_TRACKING"),
        ("fault", "SG100_FAULT"),
        ("mode", "UNEXPECTED_RX_MODE"),
        ("transport", "TRANSPORT_FEEDBACK_MISSING"),
        ("crc", "CRC_STORM"),
    ],
)
def test_health_failures_force_mode_null(mutation, reason):
    driver, backend = make_backend()
    q = np.zeros(HAND_DOF)
    backend.anchor(q)
    backend.set_enabled(True)
    tracking_ns = 1_000_000_000
    now_ns = tracking_ns
    if mutation == "stale":
        tracking_ns = 0
    elif mutation == "fault":
        driver.fault_active = True
    elif mutation == "mode":
        driver.feedback.joint_modes = (0,) * HAND_DOF
    elif mutation == "transport":
        driver.feedback = None
    elif mutation == "crc":
        driver.crc_count = 3
    result = backend.submit(
        q, tracking_received_ns=tracking_ns, now_ns=now_ns, dt_s=0.01
    )
    assert result.reason == reason
    assert driver.hold_calls == 1 and not backend.enabled


def test_nonfinite_range_spike_and_estop_are_fail_closed():
    q0 = np.zeros(HAND_DOF)
    for target, expected in (
        (np.full(HAND_DOF, np.nan), "COMMAND_REJECTED"),
        (np.full(HAND_DOF, 99.0), "COMMAND_REJECTED"),
    ):
        driver, backend = make_backend()
        backend.anchor(q0)
        backend.set_enabled(True)
        result = backend.submit(
            target, tracking_received_ns=1_000_000_000,
            now_ns=1_000_000_000, dt_s=0.01,
        )
        assert result.reason.startswith(expected)
        assert driver.hold_calls == 1

    driver, backend = make_backend(tracking_spike_rad=0.1)
    backend.anchor(q0)
    backend.set_enabled(True)
    spike = q0.copy()
    spike[5] = 0.2
    assert backend.submit(
        spike, tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    ).reason == "TRACKING_SPIKE"

    driver, backend = make_backend()
    backend.anchor(q0)
    backend.set_enabled(True)
    assert backend.emergency_stop().reason == "EMERGENCY_STOP"
    assert backend.submit(
        q0, tracking_received_ns=1, now_ns=1, dt_s=0.01
    ).reason == "ESTOP_LATCHED"
