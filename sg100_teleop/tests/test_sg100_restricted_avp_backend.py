"""Restricted AVP SG100 backend 的无硬件安全回归。"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from sg100_teleop.robot.sg100_driver import (
    HAND_DOF,
    MODE_JOINT_POSITION,
    SG100CommandLimitContract,
    SG100HardwareMap,
)
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


@pytest.fixture(scope="module")
def command_limit_contract():
    ranges = load_vendor_user_ranges(DEFAULT_SDK_DIR)
    assert ranges is not None and len(ranges) == HAND_DOF
    return SG100CommandLimitContract(SG100HardwareMap(), ranges)


@pytest.mark.parametrize("joint_index", range(HAND_DOF))
def test_every_joint_min_mid_max_and_outside_clamp(
    command_limit_contract, joint_index
):
    row = command_limit_contract.rows[joint_index]
    low = row.canonical_min_rad
    high = row.canonical_max_rad
    mid = 0.5 * (low + high)
    delta = np.deg2rad(1.0)
    for requested, expected in (
        (low - delta, low),
        (low, low),
        (mid, mid),
        (high, high),
        (high + delta, high),
    ):
        desired = np.zeros(HAND_DOF)
        desired[joint_index] = requested
        applied = command_limit_contract.apply(desired)
        assert applied.effective_canonical_rad[joint_index] == pytest.approx(
            expected, abs=1e-12
        )
        assert row.sdk_min_deg <= applied.sdk_sent_deg[joint_index]
        assert applied.sdk_sent_deg[joint_index] <= row.sdk_max_deg


def test_thumb_j3_minus_61_and_float_boundary_clamp_not_reject(
    command_limit_contract,
):
    thumb_j3 = SG100HardwareMap().software_index_of["thumb_j3"]
    for requested_deg in (-61.0, -60.000447):
        desired = np.zeros(HAND_DOF)
        desired[thumb_j3] = np.deg2rad(requested_deg)
        applied = command_limit_contract.apply(desired)
        assert np.rad2deg(
            applied.effective_canonical_rad[thumb_j3]
        ) == pytest.approx(-60.0)
        assert applied.sdk_sent_deg[thumb_j3] == pytest.approx(-60.0)


def test_middle_j1_is_only_middle_sign_flip(command_limit_contract):
    rows = {row.joint_name: row for row in command_limit_contract.rows}
    assert rows["middle_j1"].command_gain == -1.0
    assert rows["middle_j2"].command_gain == 1.0

    middle = SG100HardwareMap().software_index_of["middle_j1"]
    for canonical_deg, expected_sdk_deg in (
        (-50.0, 50.0),
        (-125.0, 125.0),
        (1.0, 0.0),
        (-126.0, 125.0),
    ):
        desired = np.zeros(HAND_DOF)
        desired[middle] = np.deg2rad(canonical_deg)
        applied = command_limit_contract.apply(desired)
        assert applied.sdk_sent_deg[middle] == pytest.approx(expected_sdk_deg)


def _all_joint_backend():
    driver = FakeDriver()
    names = tuple(SG100HardwareMap().software_joint_order)
    backend = SG100RestrictedAVPBackend(
        driver,
        joint_allowlist=names,
        hardware_validated_joints=names,
        max_cycle_delta_rad=None,
        max_velocity_rad_s=100.0,
        tracking_spike_rad=100.0,
    )
    assert backend.anchor(np.zeros(HAND_DOF)).accepted
    backend.set_enabled(True)
    return driver, backend


@pytest.mark.parametrize("requested_deg", [-61.0, -60.000447])
def test_thumb_j3_end_to_end_accepted_effective_and_sdk(requested_deg):
    driver, backend = _all_joint_backend()
    desired = np.zeros(HAND_DOF)
    desired[2] = np.deg2rad(requested_deg)
    result = backend.submit(
        desired,
        tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000,
        dt_s=1.0,
    )
    assert result.accepted
    assert "COMMAND_REJECTED" not in result.reason
    assert np.rad2deg(result.canonical_qcmd_rad[2]) == pytest.approx(requested_deg)
    assert np.rad2deg(
        result.effective_canonical_qcmd_rad[2]
    ) == pytest.approx(-60.0)
    assert result.sdk_target_deg[2] == pytest.approx(-60.0)
    assert driver.active_calls[-1][1][2] == pytest.approx(-60.0)
    clamp_events = [
        event for event in backend.drain_safety_events()
        if event.event == "JOINT_LIMIT_CLAMP"
    ]
    assert any(
        event.joint_name == "thumb_j3"
        and event.domain == "SDK"
        and event.effective == pytest.approx(-60.0)
        for event in clamp_events
    )


def test_clamp_logging_is_on_change_and_mapping_mismatch_is_visible():
    _, backend = _all_joint_backend()
    backend.drain_safety_events()
    desired = np.zeros(HAND_DOF)
    desired[2] = np.deg2rad(-61.0)
    for _ in range(2):
        assert backend.submit(
            desired, tracking_received_ns=1_000_000_000,
            now_ns=1_000_000_000, dt_s=1.0,
        ).accepted
    events = backend.drain_safety_events()
    assert sum(event.event == "JOINT_LIMIT_CLAMP" for event in events) == 1

    backend.hardware_map.gain[0] = 2.0
    mismatch_input = np.zeros(HAND_DOF)
    mismatch_input[0] = np.deg2rad(100.0)
    applied = backend.command_limit_contract.apply(mismatch_input)
    assert applied.sdk_mapping_mismatch_indices == (0,)
    backend._record_limit_events(applied)
    assert any(
        event.event == "SDK_LIMIT_MAPPING_MISMATCH"
        for event in backend.drain_safety_events()
    )


def test_current_validated_allowlist_accepts_middle_j1():
    backend = SG100RestrictedAVPBackend(
        FakeDriver(), joint_allowlist=("middle_j1",)
    )
    assert backend.active_joint_indices == (6,)


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
    jumped_target = target.copy()
    jumped_target[5] = 0.08
    result = backend.submit(
        jumped_target, tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    )
    assert result.accepted and result.slew_limited
    assert np.deg2rad(driver.active_calls[-1][1][5]) == pytest.approx(0.005)

    driver, backend = make_backend(max_cycle_delta_rad=0.03)
    backend.anchor(q0)
    backend.set_enabled(True)
    target[5] = 0.04
    first = backend.submit(
        target, tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    )
    assert first.accepted and first.slew_limited
    assert np.deg2rad(driver.active_calls[-1][1][5]) == pytest.approx(0.005)

    jumped_target = target.copy()
    jumped_target[5] = 0.08
    result = backend.submit(
        jumped_target, tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    )
    assert result.reason == "PER_CYCLE_DELTA_EXCEEDED"
    assert driver.hold_calls == 1 and not backend.enabled


def test_first_post_enable_command_only_bypasses_per_cycle_guard():
    q0 = np.zeros(HAND_DOF)
    driver, backend = make_backend(
        max_cycle_delta_rad=0.03,
        max_velocity_rad_s=0.5,
        tracking_spike_rad=0.20,
    )
    backend.anchor(q0)
    backend.set_enabled(True)

    target = q0.copy()
    target[5] = 0.08
    first = backend.submit(
        target, tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    )
    assert first.accepted and first.slew_limited
    assert backend.enabled
    assert np.deg2rad(driver.active_calls[-1][1][5]) == pytest.approx(0.005)

    # The stable second target is compared with the first target, not with
    # the much slower velocity-limited output.
    second = backend.submit(
        target, tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    )
    assert second.accepted and second.slew_limited
    assert np.deg2rad(driver.active_calls[-1][1][5]) == pytest.approx(0.010)

    # A later raw-target jump still sees the ordinary per-cycle guard.
    jumped_target = target.copy()
    jumped_target[5] = 0.12
    third = backend.submit(
        jumped_target, tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    )
    assert third.reason == "PER_CYCLE_DELTA_EXCEEDED"
    assert driver.hold_calls == 1 and not backend.enabled

    # Re-enabling grants a new one-command exemption, but the independent
    # tracking-spike safety gate is never bypassed.
    backend.set_enabled(True)
    spike = q0.copy()
    spike[5] = 0.30
    assert backend.submit(
        spike, tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    ).reason == "TRACKING_SPIKE"


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
    driver, backend = make_backend()
    backend.anchor(q0)
    backend.set_enabled(True)
    result = backend.submit(
        np.full(HAND_DOF, np.nan), tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    )
    assert result.reason.startswith("COMMAND_REJECTED")
    assert driver.hold_calls == 1

    # Finite range overflow is clamped, never rejected.  The independent
    # tracking-spike gate may still stop an implausible 99-rad one-tick jump.
    driver, backend = make_backend()
    backend.anchor(q0)
    backend.set_enabled(True)
    result = backend.submit(
        np.full(HAND_DOF, 99.0), tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=0.01,
    )
    assert result.reason == "TRACKING_SPIKE"
    assert "COMMAND_REJECTED" not in result.reason

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
