"""Mode-aware SG100 feedback semantics regressions (offline; no hardware).

Covers the MODE7 production-path validation lists. Round 1 (driver/data):
A. Joint_Status.mode is preserved end to end into the Python feedback
B. MODE_NULL feedback: canonical_valid = False (NON_CANONICAL_DIAGNOSTIC)
C. mode-1 feedback: canonical_valid = False
D. mode-7 feedback before hardware validation: canonical_valid = False
   but domain MODE7_JOINT_POSITION_CANDIDATE
E. mixed modes: canonical_valid = False
F. MODE_NULL accumulated ~1854 deg cannot become measured_hand_q
G. the dataset rejects/quarantines non-mode7 feedback
J. no SetJointPositions usage
K. no Homing/SetJointZero
L. no startup-feedback-as-zero logic

Round 2 (full-hand mode7 feedback probe, vendor entry contract):
- governing states split: entry/control PASS and joint-angle-scale domain
  SUPPORTED vs exact canonical mapping still pending
- MODE_NULL position NEVER becomes a mode7 target (always-raises helper)
- deterministic zero-initialized 176-byte Hand_Control_t, ids 1..11 all
  mode 7, no uninitialized/raw bytes, preflight prints all 11 entries
- mode7 result decided by RX Joint_Status.mode only (not TX intent)
- multi-turn / mixed returned modes => FAIL/REJECTED
- operator movement confirmation treats Enter as no movement observed
- canonical_valid stays False before hardware validation
- zero hardware access in dry-run
"""

from __future__ import annotations

import ctypes
import dataclasses
import json
import struct
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
from sg100_teleop.robot.sg100_driver import (
    FEEDBACK_DOMAIN_MODE7_CANDIDATE,
    FEEDBACK_DOMAIN_NON_CANONICAL_DIAGNOSTIC,
    FEEDBACK_DOMAIN_UNCLASSIFIED,
    HAND_DOF,
    MODE_JOINT_POSITION,
    MODE_NULL,
    SG100HandFeedback,
    SG100HardwareMap,
    classify_feedback_domain,
)
from sg100_teleop.robot.sg100_sdk_driver import (
    DEFAULT_SDK_DIR,
    MODE_POSITION,
    SG100Mode7ProbeSubmission,
    SG100SDKDriver,
    _load_wrapper_library,
    load_vendor_user_ranges,
)
from sg100_teleop.tools import sg100_mode7_feedback_probe as probe
from sg100_teleop.tools import sg100_mode7_mapping_characterization as mapping


ROOT = Path(__file__).resolve().parents[2]


# ---------------------------------------------------------------------------
# Fake wrapper: pretends to be the loaded C library and hands read_feedback
# a canned poll result (positions in SDK callback degrees + joint modes).
# ---------------------------------------------------------------------------
class _FakeWrapper:
    def __init__(self, positions_deg, modes):
        self.positions_deg = np.asarray(positions_deg, dtype=np.float64)
        self.modes = np.asarray(modes, dtype=np.uint8)
        self._sequence = 0

    def sg100_poll_status(self, positions, currents, modes_out, recv_ns):
        positions[:] = self.positions_deg
        currents[:] = 0.0
        modes_out[:] = self.modes
        recv_ns._obj.value = time.monotonic_ns()
        self._sequence += 1
        return self._sequence


def _driver_with_modes(modes, *, positions_deg=None,
                       mode7_hardware_validated=False) -> SG100SDKDriver:
    driver = SG100SDKDriver(
        "/dev/null",
        hardware_map=SG100HardwareMap(),
        mode7_hardware_validated=mode7_hardware_validated,
    )
    if positions_deg is None:
        positions_deg = np.zeros(HAND_DOF)
    driver._lib = _FakeWrapper(positions_deg, modes)
    driver._connected = True
    return driver


def _feedback(modes, q_deg, *, canonical_valid=False,
              domain=FEEDBACK_DOMAIN_NON_CANONICAL_DIAGNOSTIC,
              sequence=1):
    q_rad = np.deg2rad(np.asarray(q_deg, dtype=np.float64))
    return SG100HandFeedback(
        q_rad=q_rad,
        received_monotonic_ns=time.monotonic_ns() + sequence,
        sequence=sequence,
        valid=True,
        raw_positions=q_rad.copy(),
        joint_modes=tuple(modes),
        feedback_domain=domain,
        canonical_valid=canonical_valid,
    )


def _make_collector(directory: Path) -> ArmHandDataCollector:
    return ArmHandDataCollector(
        directory,
        arm_lower_rad=np.full(6, -3.2),
        arm_upper_rad=np.full(6, 3.2),
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


class _StubProbeDriver:
    """Offline stub for run_phase_c/_observe_transition.

    Yields scripted FRESH frames (unique sequence + timestamp), records
    sends, and exposes the diagnostics surface the audit counters read.
    """

    realtime_active = True
    fault_active = False
    control_mode = MODE_JOINT_POSITION
    vendor_user_ranges = None

    def __init__(self, frame_modes: list[list[int]] | None = None,
                 frames: int = 50,
                 positions_deg=None,
                 repeat_last: bool = False):
        self._script = list(
            frame_modes
            if frame_modes is not None
            else [[MODE_JOINT_POSITION] * HAND_DOF for _ in range(frames)]
        )
        self._positions = positions_deg
        self._repeat_modes = (
            list(self._script[-1]) if repeat_last and self._script else None
        )
        self.sent_targets = []
        self._sequence = 0
        self._hand_control_calls = 0
        self.hold_passive_calls = 0
        self.lifecycle = []

    @property
    def diagnostics(self):
        return {"hand_control_calls": self._hand_control_calls}

    def connect(self):
        self.lifecycle.append("connect")

    def stop(self):
        self.lifecycle.append("stop")

    def disconnect(self):
        self.lifecycle.append("disconnect")

    def send_positions_rad(self, q_rad, **kwargs):
        self.sent_targets.append(np.asarray(q_rad, dtype=np.float64).copy())
        self._hand_control_calls += 1
        return object()  # truthy sentinel "event"

    def send_mode7_feedback_probe(self):
        self.sent_targets.append(np.zeros(HAND_DOF, dtype=np.float64))
        self._hand_control_calls += 1
        return SG100Mode7ProbeSubmission(
            sent_monotonic_ns=time.monotonic_ns(),
            requested_positions_deg=(0.0,) * HAND_DOF,
            requested_modes=(MODE_JOINT_POSITION,) * HAND_DOF,
        )

    def hold_passive(self) -> bool:
        self.hold_passive_calls += 1
        self.lifecycle.append("hold_passive")
        return True

    def read_feedback(self):
        if self._script:
            modes = self._script.pop(0)
        elif self._repeat_modes is not None:
            modes = list(self._repeat_modes)
        else:
            return None
        positions = (
            np.asarray(self._positions, dtype=np.float64)
            if self._positions is not None
            else np.zeros(HAND_DOF)
        )
        self._sequence += 1
        return SG100HandFeedback(
            q_rad=np.deg2rad(positions),
            received_monotonic_ns=time.monotonic_ns() + self._sequence,
            sequence=self._sequence,
            valid=True,
            raw_positions=np.deg2rad(positions),
            joint_modes=tuple(int(m) for m in modes),
            feedback_domain=(
                FEEDBACK_DOMAIN_MODE7_CANDIDATE
                if all(m == MODE_JOINT_POSITION for m in modes)
                else "NON_CANONICAL_DIAGNOSTIC"
            ),
            canonical_valid=False,
        )


# -- A. mode preserved ------------------------------------------------------


def test_a_joint_status_mode_preserved_to_python_feedback():
    for modes in ([0] * 11, [7] * 11, [1, 7, 0, 7, 7, 7, 7, 7, 7, 7, 7]):
        driver = _driver_with_modes(modes)
        feedback = driver.read_feedback()
        assert feedback is not None
        assert feedback.joint_modes == tuple(modes)


def test_a_wrapper_preserves_mode_bytes():
    # The C wrapper snapshot path keeps Joint_Status.mode[11] verbatim.
    wrapper_src = (
        ROOT / "sg100_teleop" / "robot" / "sg100_c_wrapper.cpp"
    ).read_text()
    assert "uint8_t mode[JOINT_COUNT]" in wrapper_src
    assert "g_latest.mode[id - 1] = status.joints[i].mode" in wrapper_src
    assert "std::memcpy(mode_out, g_latest.mode" in wrapper_src


# -- B/C/D/E. classification ------------------------------------------------


def test_b_mode_null_feedback_is_non_canonical_diagnostic():
    driver = _driver_with_modes([MODE_NULL] * 11)
    feedback = driver.read_feedback()
    assert feedback.feedback_domain == FEEDBACK_DOMAIN_NON_CANONICAL_DIAGNOSTIC
    assert feedback.canonical_valid is False


def test_c_mode1_feedback_is_non_canonical():
    driver = _driver_with_modes([MODE_POSITION] * 11)
    feedback = driver.read_feedback()
    assert feedback.feedback_domain == FEEDBACK_DOMAIN_NON_CANONICAL_DIAGNOSTIC
    assert feedback.canonical_valid is False


def test_d_mode7_feedback_is_candidate_but_not_canonical():
    driver = _driver_with_modes([MODE_JOINT_POSITION] * 11)
    feedback = driver.read_feedback()
    assert feedback.feedback_domain == FEEDBACK_DOMAIN_MODE7_CANDIDATE
    # Before real-hardware MODE7 validation canonical_valid must stay False.
    assert feedback.canonical_valid is False


def test_d_mode7_canonical_requires_explicit_hardware_validation_flag():
    driver = _driver_with_modes(
        [MODE_JOINT_POSITION] * 11, mode7_hardware_validated=True
    )
    feedback = driver.read_feedback()
    # The only unlock path is the explicit validated flag (default False,
    # covered by test_d_mode7_feedback_is_candidate_but_not_canonical).
    assert feedback.canonical_valid is True


def test_e_mixed_modes_are_non_canonical():
    for modes in (
        [7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 0],
        [0] * 10 + [7],
        [1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 7],
    ):
        driver = _driver_with_modes(modes)
        feedback = driver.read_feedback()
        assert feedback.feedback_domain == FEEDBACK_DOMAIN_NON_CANONICAL_DIAGNOSTIC
        assert feedback.canonical_valid is False


def test_classify_feedback_domain_unit_matrix():
    assert classify_feedback_domain(None) == FEEDBACK_DOMAIN_UNCLASSIFIED
    assert classify_feedback_domain([7] * 11) == (
        FEEDBACK_DOMAIN_MODE7_CANDIDATE
    )
    assert classify_feedback_domain([0] * 11) == (
        FEEDBACK_DOMAIN_NON_CANONICAL_DIAGNOSTIC
    )
    # Wrong length is treated as non-canonical (defensive).
    assert classify_feedback_domain([7] * 10) == (
        FEEDBACK_DOMAIN_NON_CANONICAL_DIAGNOSTIC
    )


# -- F/G. dataset quarantine ------------------------------------------------


def test_f_mode_null_accumulated_1854_cannot_become_measured_hand_q():
    with tempfile.TemporaryDirectory() as directory:
        collector = _make_collector(Path(directory))
        # Real N1 trace shape: ~1854 accumulated callback degrees.
        collector.observe_hand_feedback(
            _feedback([MODE_NULL] * 11, [1854.0] * 11, sequence=1)
        )
        assert collector.rejections, "accumulated MODE_NULL sample accepted"
        assert collector.rejections[-1].reason == (
            RejectionReason.HAND_FEEDBACK_NON_CANONICAL_MODE
        )
        assert collector.transitions == []
        assert collector._hand_measured_baseline is None


def test_g_collector_rejects_non_mode7_and_unvalidated_mode7():
    with tempfile.TemporaryDirectory() as directory:
        collector = _make_collector(Path(directory))
        cases = [
            ("mode1", [MODE_POSITION] * 11, FEEDBACK_DOMAIN_NON_CANONICAL_DIAGNOSTIC, 1),
            ("mixed", [7] * 10 + [0], FEEDBACK_DOMAIN_NON_CANONICAL_DIAGNOSTIC, 2),
            ("mode7_unvalidated", [MODE_JOINT_POSITION] * 11,
             FEEDBACK_DOMAIN_MODE7_CANDIDATE, 3),
        ]
        for name, modes, domain, sequence in cases:
            collector.observe_hand_feedback(
                _feedback(modes, np.zeros(11), sequence=sequence,
                          domain=domain, canonical_valid=False)
            )
        reasons = [r.reason for r in collector.rejections]
        assert reasons.count(
            RejectionReason.HAND_FEEDBACK_NON_CANONICAL_MODE
        ) == 3
        assert collector.transitions == []


def test_g_collector_keeps_mock_compatibility_without_modes():
    # Feedback without mode information (mock/legacy drivers) keeps the
    # previous manifest-level quarantine path: not hard-rejected here.
    with tempfile.TemporaryDirectory() as directory:
        collector = _make_collector(Path(directory))
        q_rad = np.zeros(11)
        collector.observe_hand_feedback(
            SG100HandFeedback(
                q_rad=q_rad,
                received_monotonic_ns=time.monotonic_ns(),
                sequence=42,
                valid=True,
            )
        )
        assert not [
            r for r in collector.rejections
            if r.reason == RejectionReason.HAND_FEEDBACK_NON_CANONICAL_MODE
        ]


def test_g_collector_raw_event_records_modes_and_domain():
    with tempfile.TemporaryDirectory() as directory:
        collector = _make_collector(Path(directory))
        collector.observe_hand_feedback(
            _feedback([MODE_NULL] * 11, [1854.0] * 11, sequence=7)
        )
        audit_rows = [
            event for event in collector._raw_events
            if event["event_type"] == "hand_feedback"
        ]
        assert audit_rows, "hand_feedback raw audit row missing"
        payload = audit_rows[-1]["payload"]
        assert payload["joint_modes"] == [0] * 11
        assert payload["feedback_domain"] == (
            FEEDBACK_DOMAIN_NON_CANONICAL_DIAGNOSTIC
        )
        assert payload["canonical_valid"] is False


# -- H. probe states + prohibition ------------------------------------------


def test_h_probe_governing_states_split_entry_from_validation():
    # Vendor contract confirmed => full-hand mode7 FEEDBACK probe allowed.
    assert probe.MODE7_ENTRY_CONTRACT_CONFIRMED is True
    # Real hardware supports MODE7's angle-scale domain, but exact F_fb is
    # still pending; canonical/dataset/AVP/H2 stay blocked.
    assert probe.MODE7_JOINT_ANGLE_SCALE_HARDWARE_SUPPORTED is True
    assert probe.MODE7_CANONICAL_MAPPING_HARDWARE_VALIDATED is False
    assert probe.MODE7_PHYSICAL_FEEDBACK_HARDWARE_VALIDATED == (
        "PARTIAL_ANGLE_DOMAIN_SUPPORTED"
    )
    assert probe.MODE_NULL_TARGET_REUSE == "FORBIDDEN"


def test_h_mode_null_position_never_becomes_mode7_target():
    # The prohibition helper ALWAYS raises — no validation escape hatch.
    for bad in (1854.0, 42604.57, -21243.11):
        with pytest.raises(probe.ModeNullTargetReuseError):
            probe.reject_mode_null_derived_target(bad)
    # The deterministic builder takes no feedback input at all: there is
    # no code path from MODE_NULL status into a mode7 target.
    specs = probe.build_full_hand_mode7_control()
    assert all(s.position_deg == 0.0 for s in specs)


def test_h_phase_c_operator_confirmation_semantics():
    driver = _StubProbeDriver()
    # Decline the send -> abort BEFORE any mode7 command.
    result = probe.run_phase_c(
        driver, None, confirm_fn=lambda prompt: "no"
    )
    assert result["status"] == "OPERATOR_ABORT_BEFORE_SEND"
    assert driver.sent_targets == []  # nothing handed to SetHandControl


def test_h_phase_c_enter_means_no_visible_motion_observed():
    driver = _StubProbeDriver()
    answers = iter(["YES", ""])          # send yes, movement answer default
    result = probe.run_phase_c(
        driver, None, confirm_fn=lambda prompt: next(answers)
    )
    # Empty/"n" means no movement observed; "y" means movement and FAIL.
    assert result["MODE7_STATUS_MODE"] == "PASS"
    assert result["MODE7_CONTROL_STATE"] == "PASS"
    assert result["MODE7_JOINT_ANGLE_SCALE_DOMAIN"] == "SUPPORTED"
    assert result["MODE7_CANONICAL_MAPPING"] == "PENDING"
    assert result["canonical_valid"] is False
    assert result["NO_VISIBLE_ENTRY_MOTION"] == "PASS"
    moved_driver = _StubProbeDriver()
    result_moved = probe.run_phase_c(
        moved_driver, None,
        confirm_fn=lambda prompt: "YES" if "SEND" in prompt else "y",
    )
    assert result_moved["NO_VISIBLE_ENTRY_MOTION"] == "FAIL"
    assert result_moved["MODE7_PHYSICAL_FEEDBACK_HARDWARE_VALIDATED"] == (
        "PARTIAL_ANGLE_DOMAIN_SUPPORTED"
    )
    assert result_moved["canonical_valid_autoflip"] == "NOT_PERFORMED"


# -- I. deterministic full-hand frame ---------------------------------------


def test_i_joint_count_and_struct_sizes():
    probe.assert_hand_control_contract()
    assert probe.EXPECTED_JOINT_COUNT == 11
    assert probe.EXPECTED_JOINT_CONTROL_SIZE == 16
    assert probe.EXPECTED_HAND_CONTROL_SIZE == 176
    assert HAND_DOF == 11


def test_i_full_hand_mode7_ids_are_exactly_1_to_11():
    specs = probe.build_full_hand_mode7_control()
    ids = sorted(spec.sdk_joint_id for spec in specs)
    assert ids == list(range(1, 12))            # exactly 1..11, unique
    assert all(spec.mode == MODE_JOINT_POSITION for spec in specs)


def test_i_zero_initialized_frame_is_deterministic_176_bytes():
    specs = probe.build_full_hand_mode7_control()
    frame_a = probe.serialize_hand_control(specs)
    frame_b = probe.serialize_hand_control(probe.build_full_hand_mode7_control())
    assert len(frame_a) == 176
    assert frame_a == frame_b                    # byte-identical rebuilds
    for joint_index in range(11):
        chunk = frame_a[joint_index * 16:(joint_index + 1) * 16]
        jid, mode, position, p, i, d, limit = struct.unpack(
            "<BBfHHHH", chunk[:14]
        )
        assert jid == joint_index + 1
        assert mode == MODE_JOINT_POSITION
        # Zero-initialized remaining fields, printed verbatim in preflight:
        assert position == 0.0
        assert (p, i, d, limit) == (0, 0, 0, 0)
        assert chunk[14:16] == b"\x00\x00"       # reserved[2] deterministic


def test_i_no_uninitialized_or_raw_bytes_in_frame():
    frame = probe.serialize_hand_control(probe.build_full_hand_mode7_control())
    # Exact expected template: every one of the 176 bytes accounted for.
    template = b"".join(
        struct.pack("<BBfHHHH2x", j, MODE_JOINT_POSITION, 0.0, 0, 0, 0, 0)
        for j in range(1, 12)
    )
    assert frame == template
    # Non-finite field values are rejected at serialization time.
    with pytest.raises(ValueError):
        probe.serialize_hand_control([
            probe.JointSpec(1, MODE_JOINT_POSITION, float("nan"), 0, 0, 0, 0)
        ] + [probe.JointSpec(j, MODE_NULL, 0.0, 0, 0, 0, 0)
             for j in range(2, 12)]
        )
    # Wrong id set rejected.
    with pytest.raises(ValueError):
        probe.serialize_hand_control([
            probe.JointSpec(j, MODE_NULL, 0.0, 0, 0, 0, 0)
            for j in range(2, 13)
        ])


def test_i_native_cpp_builder_serializes_exact_probe_payload():
    wrapper = _load_wrapper_library(sdk_dir=DEFAULT_SDK_DIR)
    positions = np.zeros(HAND_DOF, dtype=np.float32)
    native = np.zeros(probe.EXPECTED_HAND_CONTROL_SIZE, dtype=np.uint8)
    size = wrapper.sg100_test_serialize_hand_control(
        positions,
        HAND_DOF,
        MODE_JOINT_POSITION,
        0,
        0,
        0,
        0,
        native,
        native.size,
    )
    assert size == probe.EXPECTED_HAND_CONTROL_SIZE
    expected = probe.serialize_hand_control(
        probe.build_full_hand_mode7_control()
    )
    assert native.tobytes() == expected


def test_i_preflight_displays_all_11_entries_verbatim():
    specs = probe.build_full_hand_mode7_control()
    table = probe.format_preflight_table(specs)
    for spec in specs:
        assert f"J{spec.sdk_joint_id}" in table
    assert "all 11 mode7: YES" in table
    assert "position=0.0, p=0, i=0, d=0, output_limit=0" in table
    assert "176 bytes (11 x 16)" in table


# -- mode7 result decided by RX status.mode only ----------------------------


def test_mode7_result_from_rx_status_mode_not_tx_intent():
    good = [(i, [7] * 11, [0.0] * 11) for i in range(20)]
    ok, detail = probe.verify_all_mode7(good)
    assert ok, detail
    # TX said 7 but the board returned MODE_NULL -> FAIL (RX decides).
    bad = [(i, [0] * 11, [0.0] * 11) for i in range(20)]
    ok2, _ = probe.verify_all_mode7(bad)
    assert ok2 is False
    # Fewer than the five-frame stability criterion -> FAIL.
    ok3, _ = probe.verify_all_mode7(good[:4])
    assert ok3 is False


def test_multi_turn_mode7_feedback_fails():
    assert probe.detect_multi_turn([0.0, 1854.0])
    assert probe.detect_multi_turn([-42604.57])
    assert not probe.detect_multi_turn([0.0, 125.0, -60.0])
    series = [[0.0] * 10 for _ in range(11)]
    series[4] = [0.0, 42604.57] + [0.0] * 8     # J5 goes multi-turn
    assert probe.classify_joint_angle_scale_domain(series) == "REJECTED"
    series_nan = [[0.0] * 10 for _ in range(11)]
    series_nan[2] = [float("nan")] * 10
    assert probe.classify_joint_angle_scale_domain(series_nan) == "REJECTED"


def test_mixed_returned_modes_fail():
    mixed = [(i, [7] * 10 + [0], [0.0] * 11) for i in range(20)]
    ok, detail = probe.verify_all_mode7(mixed)
    assert ok is False and "J11=mode0" in detail


def test_angle_domain_does_not_depend_on_command_range_equality():
    # Bounded finite stable values are angle-scale SUPPORTED even when they
    # do not match SDK command ranges. Range mapping is a separate verdict.
    good = [[10.0] * 20 for _ in range(11)]
    assert probe.classify_joint_angle_scale_domain(good) == "SUPPORTED"
    shifted = [[10.0] * 20 for _ in range(11)]
    shifted[0] = [130.0] * 20
    assert probe.classify_joint_angle_scale_domain(shifted) == "SUPPORTED"
    wide = [[10.0] * 20 for _ in range(11)]
    wide[4] = [10.0] * 5 + [200.0] * 15
    assert probe.classify_joint_angle_scale_domain(wide) == "SUPPORTED"


def test_real_mode7_baselines_report_command_and_canonical_ranges_separately():
    values = np.asarray([
        10.2289457321167,
        18.083322525024414,
        7.229345321655273,
        0.13200023770332336,
        -0.4284714460372925,
        -9.283442497253418,
        -6.932403087615967,
        -0.03279173746705055,
        -0.31862491369247437,
        -3.955403566360473,
        -8.811077117919922,
    ])
    hardware_map_path = (
        ROOT / "sg100_teleop" / "config" / "sg100_hardware_map.yaml"
    )
    before = hardware_map_path.read_bytes()
    hardware_map = SG100HardwareMap()
    canonical_ranges = probe.canonical_ranges_deg(hardware_map)
    command_ranges = load_vendor_user_ranges(Path(DEFAULT_SDK_DIR))
    assert command_ranges is not None

    command_overall, command = probe.strict_range_compatibility(
        values, command_ranges
    )
    canonical_overall, canonical = probe.strict_range_compatibility(
        values, canonical_ranges
    )
    assert command_overall == "PARTIAL"
    assert canonical_overall == "PARTIAL"
    assert [item["compatible"] for item in command] == [
        True, False, True, True, False, True,
        False, True, True, False, True,
    ]
    assert [item["compatible"] for item in canonical] == [
        True, False, True, True, False, True,
        True, True, True, False, True,
    ]
    # J7: command mismatch is not a canonical-range or angle-domain failure.
    assert command[6]["compatible"] is False
    assert canonical[6] == {
        "compatible": True,
        "distance_outside_deg": 0.0,
    }
    assert canonical[1]["distance_outside_deg"] == pytest.approx(
        18.083322525024414
    )
    assert canonical[4]["distance_outside_deg"] == pytest.approx(
        0.4284714460372925
    )
    assert canonical[9]["distance_outside_deg"] == pytest.approx(
        3.955403566360473
    )
    stable_series = [[float(value)] * 17 for value in values]
    assert probe.classify_joint_angle_scale_domain(stable_series) == (
        "SUPPORTED"
    )
    frames = [
        (index, [MODE_JOINT_POSITION] * 11, values.tolist())
        for index in range(17)
    ]
    per_joint = probe.summarize_joints(
        [0.0] * 11,
        frames,
        command_ranges,
        canonical_ranges,
        joint_names=hardware_map.software_joint_order,
    )
    assert per_joint[6]["canonical_joint"] == "middle_j1"
    assert per_joint[6]["sdk_command_range_compatible"] is False
    assert per_joint[6]["strict_canonical_range_compatible"] is True
    assert per_joint[6]["interpretation"] == (
        "STRICT_CANONICAL_RANGE_COMPATIBLE_MAPPING_PENDING"
    )
    assert per_joint[4]["interpretation"] == (
        "NEAR_BOUNDARY_OFFSET_CANDIDATE_MAPPING_PENDING"
    )
    assert per_joint[1]["interpretation"] == (
        "ZERO_SIGN_OR_IDENTITY_MAPPING_PENDING"
    )
    assert per_joint[9]["interpretation"] == (
        "ZERO_SIGN_OR_IDENTITY_MAPPING_PENDING"
    )
    assert hardware_map_path.read_bytes() == before


def test_partial_ranges_leave_canonical_mapping_pending_without_domain_reject():
    values = [
        10.22895, 18.08332, 7.22935, 0.13200, -0.42847,
        -9.28344, -6.93240, -0.03279, -0.31862, -3.95540, -8.81108,
    ]
    command_ranges = load_vendor_user_ranges(Path(DEFAULT_SDK_DIR))
    canonical_ranges = probe.canonical_ranges_deg(SG100HardwareMap())
    driver = _StubProbeDriver(positions_deg=values)
    answers = iter(["YES", ""])
    result = probe.run_phase_c(
        driver,
        command_ranges,
        canonical_ranges,
        confirm_fn=lambda prompt: next(answers),
    )
    assert result["status"] == "COMPLETED_PASS_ANGLE_DOMAIN_SUPPORTED"
    assert result["MODE7_JOINT_ANGLE_SCALE_DOMAIN"] == "SUPPORTED"
    assert result["MODE7_COMMAND_RANGE_COMPATIBILITY"] == "PARTIAL"
    assert result["MODE7_CANONICAL_RANGE_COMPATIBILITY"] == "PARTIAL"
    assert result["MODE7_CANONICAL_MAPPING"] == "PENDING"
    assert result["canonical_valid"] is False


def test_canonical_valid_stays_false_before_hardware_validation():
    assert probe.MODE7_JOINT_ANGLE_SCALE_HARDWARE_SUPPORTED is True
    assert probe.MODE7_CANONICAL_MAPPING_HARDWARE_VALIDATED is False
    # Driver gate unchanged (task section 13): mode7 candidate still not
    # canonical until a separate later change after a PASSING probe.
    driver = _driver_with_modes([MODE_JOINT_POSITION] * 11)
    feedback = driver.read_feedback()
    assert feedback.feedback_domain == FEEDBACK_DOMAIN_MODE7_CANDIDATE
    assert feedback.canonical_valid is False


def test_zero_hardware_access_in_dry_run(monkeypatch):
    calls = []

    def _forbidden(*args, **kwargs):
        calls.append(args)
        raise AssertionError("dry-run must not construct a hardware driver")

    monkeypatch.setattr(probe, "SG100SDKDriver", _forbidden)
    assert probe.main([]) == 0
    assert probe.main(["--explain"]) == 0
    assert calls == []


def test_execute_requires_port():
    with pytest.raises(SystemExit):
        probe.main(["--execute"])          # argparse error: --port missing


# ===========================================================================
# Transition state machine regressions (real-run fix, task section 15 A-M)
# ===========================================================================


# -- A/B/C/F: transitional mode0 then stable mode7 => PASS -------------------


def test_a_frame0_mode0_no_immediate_failure_then_stable():
    tracker = probe.Mode7TransitionTracker(stable_frames=5)
    # The exact real-run pattern that crashed the old probe: frame0/1
    # all-mode0 (asynchronous transient) is NOT a failure.
    tracker.observe(1000, [0] * 11, [0.0] * 11)
    tracker.observe(1001, [0] * 11, [0.0] * 11)
    for i in range(5):
        tracker.observe(1002 + i, [MODE_JOINT_POSITION] * 11, [0.0] * 11)
    assert tracker.state == probe.STATE_MODE7_STABLE
    assert tracker.first_all_mode7_index == 2


@pytest.mark.parametrize("transitional", [1, 2, 3, 4])
def test_b_one_to_four_transitional_mode0_frames_then_stable(transitional):
    tracker = probe.Mode7TransitionTracker(stable_frames=5)
    for i in range(transitional):
        tracker.observe(1000 + i, [0] * 11, [0.0] * 11)
    for i in range(5):
        tracker.observe(1100 + i, [MODE_JOINT_POSITION] * 11, [0.0] * 11)
    assert tracker.state == probe.STATE_MODE7_STABLE
    assert tracker.best_stable_run == 5


def test_c_mixed_transition_then_all7_stable():
    tracker = probe.Mode7TransitionTracker(stable_frames=5)
    mixed = [7, 0, 7, 7, 0, 7, 7, 7, 7, 0, 7]
    tracker.observe(1000, mixed, [0.0] * 11)   # not all-7
    tracker.observe(1001, [0] * 11, [0.0] * 11)
    for i in range(5):
        tracker.observe(1002 + i, [MODE_JOINT_POSITION] * 11, [0.0] * 11)
    assert tracker.state == probe.STATE_MODE7_STABLE
    assert tracker.first_mode7_any_index == 0
    assert tracker.first_all_mode7_index == 2


def test_f_five_consecutive_all7_is_stable_pass():
    tracker = probe.Mode7TransitionTracker(stable_frames=5)
    for i in range(4):
        tracker.observe(1000 + i, [MODE_JOINT_POSITION] * 11, [0.0] * 11)
    assert tracker.state == probe.STATE_WAITING_FOR_MODE7_RX  # 4 is not enough
    tracker.observe(1004, [MODE_JOINT_POSITION] * 11, [0.0] * 11)
    assert tracker.state == probe.STATE_MODE7_STABLE


# -- D/E: timeout / instability => FAIL, never PASS --------------------------


def test_d_all_mode0_until_timeout_fails():
    driver = _StubProbeDriver(
        frame_modes=[[0] * 11], repeat_last=True
    )
    tracker = probe._observe_transition(driver)
    assert tracker.state == probe.STATE_TIMEOUT
    assert tracker.best_stable_run == 0
    driver2 = _StubProbeDriver(
        frame_modes=[[0] * 11], repeat_last=True
    )
    result = probe.run_phase_c(driver2, None, confirm_fn=lambda p: "YES")
    assert result["status"] == "TRANSITION_TIMEOUT_FAIL"
    assert result["MODE7_STATUS_MODE"] == "FAIL"
    # Operator never reached the movement prompt.
    assert result["NO_VISIBLE_ENTRY_MOTION"] == "NOT_CONFIRMED"


def test_e_single_all7_frame_then_mode0_is_not_stable():
    tracker = probe.Mode7TransitionTracker(stable_frames=5)
    tracker.observe(1000, [MODE_JOINT_POSITION] * 11, [0.0] * 11)
    for i in range(6):
        tracker.observe(1001 + i, [0] * 11, [0.0] * 11)
    assert tracker.state == probe.STATE_WAITING_FOR_MODE7_RX
    assert tracker.best_stable_run == 1
    tracker.mark_timeout()
    assert tracker.state == probe.STATE_TIMEOUT


# -- G/H: mode-gated multi-turn checks ---------------------------------------


def test_g_mode0_frame_with_42604_deg_allowed_during_transition():
    tracker = probe.Mode7TransitionTracker(stable_frames=5)
    frame = tracker.observe(1000, [0] * 11, [42604.57] * 11)
    assert frame.mode7_count == 0            # recorded, not position-checked
    assert tracker.state == probe.STATE_WAITING_FOR_MODE7_RX


def test_h_mode7_frame_with_42604_deg_is_position_domain_fail():
    tracker = probe.Mode7TransitionTracker(stable_frames=5)
    mode7_with_multiturn = [
        MODE_JOINT_POSITION if j != 4 else MODE_JOINT_POSITION
        for j in range(11)
    ]
    positions = [0.0] * 11
    positions[4] = 42604.57
    tracker.observe(1000, mode7_with_multiturn, positions)
    assert tracker.state == probe.STATE_SEMANTIC_REJECTED
    assert "multi-turn" in tracker.semantic_rejection_reason


# -- I: duplicate polling rows do not consume the frame budget ---------------


def test_i_duplicate_rows_do_not_consume_transition_budget():
    tracker = probe.Mode7TransitionTracker(stable_frames=5)
    assert tracker.is_duplicate(12345) is False
    assert tracker.is_duplicate(12345) is True   # duplicate: same timestamp
    tracker.observe(12345, [0] * 11, [0.0] * 11)
    assert len(tracker.frames) == 1


# -- J: fault during transition => immediate safe failure ---------------------


def test_j_fault_during_transition_is_immediate_failure():
    driver = _StubProbeDriver(
        frame_modes=[[0] * 11 for _ in range(60)]
    )
    driver.fault_active = True
    with pytest.raises(probe.Mode7ProbeHardFail, match="fault"):
        probe._observe_transition(driver)


# -- K: crash before the movement prompt => NOT_CONFIRMED ---------------------


def test_k_hard_fail_result_cannot_claim_no_motion_pass(tmp_path, monkeypatch):
    monkeypatch.setattr(
        probe, "run_phase_a",
        lambda driver, duration: {
            "feedback_rate_hz": 100.0, "fault": 0, "modes": [0] * 11,
            "null_last_deg": [0.0] * 11, "domain": "d",
        },
    )
    # Script: mode7 frame where J5 reports a multi-turn position — semantic
    # rejection happens BEFORE the operator movement prompt.
    positions = [0.0] * HAND_DOF
    positions[4] = 42604.57
    driver = _StubProbeDriver(frames=40, positions_deg=positions)
    monkeypatch.setattr(probe, "_probe_driver", lambda *a, **k: driver)
    code = probe.run_full_probe(
        "stub", 0, Path("."), 0.01, True,
        artifact_dir=tmp_path, confirm_fn=lambda p: "YES",
    )
    assert code == 7
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["status"] == "MODE7_SEMANTIC_REJECTION_FAIL"
    assert summary["NO_VISIBLE_ENTRY_MOTION"] == "NOT_CONFIRMED"
    assert summary.get("MODE7_STATUS_MODE") != "PASS"
    assert summary["mode_null_restore_after_mode7"] is True
    rows = (tmp_path / "transition_samples.csv").read_text().splitlines()
    assert len(rows) == 2  # header + rejected fresh frame


# -- L: execute path exercises the native SetHandControl call ----------------


def test_l_execute_path_uses_native_set_hand_control():
    driver = _StubProbeDriver()
    result = probe.run_phase_c(
        driver, None, confirm_fn=lambda p: "YES"
    )
    # Exactly ONE send through the driver's native path...
    assert len(driver.sent_targets) == 1
    np.testing.assert_allclose(driver.sent_targets[0], np.zeros(HAND_DOF))
    audit = result["tx_audit"]
    assert audit["set_hand_control_calls_this_send"] == 1
    assert "SG100Device::SetHandControl(Hand_Control_t)" in audit["native_path"]
    assert audit["latest_cached_requested_joint_modes"] == [7] * 11
    assert audit["scheduler_alive"] is True
    # ...and the native chain exists in the real sources.
    wrapper = (ROOT / "sg100_teleop" / "robot" / "sg100_c_wrapper.cpp").read_text()
    assert "g_device->SetHandControl(control)" in wrapper
    sdk_driver_src = (ROOT / "sg100_teleop" / "robot" / "sg100_sdk_driver.py").read_text()
    assert "sg100_set_hand_control" in sdk_driver_src


# -- M: scheduler/cache semantics (one call updates the cache) ----------------


def test_m_one_set_hand_control_call_scheduler_resends(monkeypatch):
    # The vendor scheduler re-transmits cached_control_; the probe must
    # NOT build its own periodic sender: exactly one send per phase C.
    for _ in range(2):
        driver = _StubProbeDriver()
        probe.run_phase_c(driver, None, confirm_fn=lambda p: "YES")
        assert len(driver.sent_targets) == 1
    # Static: no custom re-send loop in the probe source (exactly one
    # actual invocation; docstring/path mentions do not count).
    source = (ROOT / "sg100_teleop" / "tools" / "sg100_mode7_feedback_probe.py").read_text()
    phase_c_body = source.split("def run_phase_c")[1].split("def _write_artifacts")[0]
    assert phase_c_body.count("driver.send_mode7_feedback_probe(") == 1
    assert "driver.send_positions_rad(" not in phase_c_body
    # Static: the wrapper documents the vendor cache/resend semantics.
    wrapper = (ROOT / "sg100_teleop" / "robot" / "sg100_c_wrapper.cpp").read_text()
    assert "cached_control_" in wrapper


# -- transition evidence is recorded (task sections 7/8/14) -------------------


def test_timeout_failure_prints_full_history_and_first_all7():
    driver = _StubProbeDriver(
        frame_modes=[[0] * 11] * 3
        + [[7] + [0] * 10] * 2
        + [[0] * 11] * 5,
        repeat_last=True,
    )
    tracker = probe._observe_transition(driver)
    tracker.set_relative_time(0)
    assert tracker.state == probe.STATE_TIMEOUT
    lines = tracker.history_lines()
    assert len(lines) == 2 + len(tracker.frames)     # header + every frame
    summary = tracker.summary()
    assert summary["FIRST_MODE7_ANY"] is not None    # evidence preserved
    assert summary["FIRST_MODE7_ALL"] is None
    assert summary["TIMEOUT"] is True


def test_stability_marks_at_five_then_observation_continues_to_twenty():
    driver = _StubProbeDriver(frames=50)
    tracker = probe._observe_transition(driver)
    assert tracker.stable_reached_at_index == 4
    assert len(tracker.frames) == probe.MIN_POST_COMMAND_FRAMES == 20
    assert tracker.observation_complete is True
    frame = tracker.frames[0]
    assert frame.timestamp_ns > 0
    assert len(frame.modes) == 11 and len(frame.positions_deg) == 11
    assert tracker.stable_frames() == [
        f for f in tracker.frames if f.all_mode7
    ]


# -- J/K/L. static source guarantees ----------------------------------------


def _sources() -> dict[str, str]:
    files = {
        "sdk_driver": ROOT / "sg100_teleop" / "robot" / "sg100_sdk_driver.py",
        "driver": ROOT / "sg100_teleop" / "robot" / "sg100_driver.py",
        "probe": ROOT / "sg100_teleop" / "tools" / "sg100_mode7_feedback_probe.py",
        "mapping": (
            ROOT / "sg100_teleop" / "tools"
            / "sg100_mode7_mapping_characterization.py"
        ),
        "wrapper": ROOT / "sg100_teleop" / "robot" / "sg100_c_wrapper.cpp",
    }
    return {name: path.read_text() for name, path in files.items()}


def test_j_no_set_joint_positions_usage():
    for name, source in _sources().items():
        assert "SetJointPositions(" not in source, name


def test_k_no_homing_or_joint_zero_calls():
    for name, source in _sources().items():
        for banned in ("HomingZero(", "SetJointZero(", "CalibrateJointZero("):
            assert banned not in source, f"{name}: {banned}"


def test_l_no_startup_feedback_as_zero_logic():
    sources = _sources()
    # The driver never derives a command target from feedback state.
    driver_source = sources["sdk_driver"]
    send_body = driver_source.split("def send_positions_rad")[1].split(
        "def hold_passive"
    )[0]
    assert "read_feedback" not in send_body
    assert "_last_feedback" not in send_body
    # The probe frame's zeros are the zero-INITIALIZED struct fields, not
    # a feedback-derived startup target: the builder takes no feedback
    # input and the MODE_NULL->mode7 copy path is forbidden by design.
    assert probe.MODE7_ENTRY_CONTRACT_CONFIRMED is True
    specs = probe.build_full_hand_mode7_control()
    assert all(spec.mode == MODE_JOINT_POSITION for spec in specs)
    assert all(spec.position_deg == 0.0 for spec in specs)


def test_l_passive_mapping_uses_only_raw_mode7_feedback_and_no_target_path(
    monkeypatch,
):
    calls = []

    def _forbidden(*args, **kwargs):
        calls.append((args, kwargs))
        raise AssertionError("prepared-only path must not construct a driver")

    monkeypatch.setattr(mapping, "_probe_driver", _forbidden)
    assert mapping.main(["--joint", "J7"]) == 0
    assert calls == []

    source = _sources()["mapping"]
    run_body = source.split("def run_characterization")[1].split(
        "def _joint_index"
    )[0]
    assert run_body.count("driver.send_mode7_feedback_probe(") == 1
    assert "send_positions_rad(" not in source
    assert "canonical_to_actual(" not in source
    assert "actual_to_canonical(" not in source
    assert "sg100_set_hand_control" not in source
    for raw_transport_token in ("RS485Adapter", "SerialTask", "CRC32", "0x05"):
        assert raw_transport_token not in source

    mode7 = _feedback(
        [MODE_JOINT_POSITION] * HAND_DOF,
        [0.0] * HAND_DOF,
        domain=FEEDBACK_DOMAIN_MODE7_CANDIDATE,
    )
    np.testing.assert_array_equal(
        mapping.extract_mode7_raw_degrees(mode7),
        np.zeros(HAND_DOF),
    )
    mode_loss = dataclasses.replace(
        mode7,
        joint_modes=(MODE_JOINT_POSITION,) * 10 + (MODE_NULL,),
    )
    with pytest.raises(probe.Mode7ProbeHardFail, match="MODE7 loss"):
        mapping.extract_mode7_raw_degrees(mode_loss)


def test_l_passive_mapping_analysis_keeps_f_cmd_and_canonical_valid_blocked():
    baseline_values = np.asarray([
        10.2, 18.1, 7.2, 0.1, -0.4, -9.3,
        -6.9, -0.03, -0.3, -4.0, -8.8,
    ])

    def frame(sequence, values):
        return mapping.MappingFrame(
            phase="test",
            sequence=sequence,
            received_monotonic_ns=sequence,
            raw_mode7_positions_deg=tuple(float(value) for value in values),
        )

    baseline = [frame(index, baseline_values) for index in range(1, 6)]
    movement = []
    for index in range(6, 16):
        values = baseline_values.copy()
        values[6] = -20.0 if index < 11 else baseline_values[6]
        movement.append(frame(index, values))
    result = mapping.analyze_manual_excursion(
        baseline,
        movement,
        expected_joint_index=6,
        canonical_delta_sign=-1,
    )
    assert result["observed_dominant_channel"] == 7
    assert result["identity_candidate"] == "EXPECTED_CHANNEL_DOMINANT"
    assert result["feedback_sign_gain_candidate"] == "+1"
    assert result["F_fb_verdict"] == "PENDING_HUMAN_REVIEW"
    assert result["F_cmd_changed_or_validated"] is False
    assert result["canonical_valid"] is False


# -- independent-review regressions: exact probe API / clocks / cleanup -----


def test_probe_driver_api_bypasses_canonical_map_and_sends_exact_native_fields():
    class CaptureWrapper:
        def __init__(self):
            self.calls = []

        def sg100_set_hand_control(
            self, positions, count, mode, p, i, d, output_limit
        ):
            self.calls.append({
                "positions": np.asarray(positions).copy(),
                "count": count,
                "mode": mode,
                "p": p,
                "i": i,
                "d": d,
                "output_limit": output_limit,
            })
            return 1

    driver = SG100SDKDriver("/dev/null")
    capture = CaptureWrapper()
    driver._lib = capture
    driver._connected = True
    driver._realtime = True
    # A hostile production map proves the diagnostic API does not consult it.
    driver.hardware_map.gain[:] = -3.0
    driver.hardware_map.offset_rad[:] = 1.25
    submission = driver.send_mode7_feedback_probe()
    assert submission is not None
    assert len(capture.calls) == 1
    call = capture.calls[0]
    np.testing.assert_array_equal(call["positions"], np.zeros(11, np.float32))
    assert call["count"] == 11
    assert call["mode"] == MODE_JOINT_POSITION
    assert (call["p"], call["i"], call["d"], call["output_limit"]) == (
        0, 0, 0, 0
    )
    assert submission.requested_modes == (MODE_JOINT_POSITION,) * 11
    # Probe submission is not normal canonical action bookkeeping.
    assert driver.drain_command_events() == ()

    source = (
        ROOT / "sg100_teleop" / "robot" / "sg100_sdk_driver.py"
    ).read_text()
    body = source.split("def send_mode7_feedback_probe")[1].split(
        "def hold_passive"
    )[0]
    assert "sg100_set_hand_control" in body
    for raw_transport_token in (
        "RS485Adapter", "SerialTask", "Pack(", "CRC32", "0x05"
    ):
        assert raw_transport_token not in body


def test_precommand_cached_rx_is_excluded_and_latencies_use_callback_clock():
    cutoff = 1_000_000_000

    class TimestampDriver:
        fault_active = False

        def __init__(self):
            timestamps = [cutoff - 1] + [
                cutoff + milliseconds * 1_000_000
                for milliseconds in range(10, 201, 10)
            ]
            self.rows = [
                _feedback(
                    [MODE_JOINT_POSITION] * 11,
                    [0.0] * 11,
                    sequence=index + 1,
                )
                for index in range(len(timestamps))
            ]
            self.rows = [
                dataclasses.replace(row, received_monotonic_ns=timestamp)
                for row, timestamp in zip(self.rows, timestamps)
            ]

        def read_feedback(self):
            return self.rows.pop(0) if self.rows else None

    tracker = probe._observe_transition(
        TimestampDriver(), submitted_monotonic_ns=cutoff
    )
    assert [frame.sequence for frame in tracker.frames] == list(range(2, 22))
    summary = tracker.summary()
    assert summary["FIRST_RX"] == pytest.approx(10.0)
    assert summary["FIRST_MODE7_ALL"] == pytest.approx(10.0)
    assert summary["MODE7_STABLE"] == pytest.approx(50.0)
    assert summary["OBSERVATION_COMPLETE"] is True
    assert summary["frames_recorded"] == 20


def test_position_classification_uses_only_final_stability_run():
    tracker = probe.Mode7TransitionTracker(stable_frames=5)
    tracker.observe(1, [7] * 11, [99.0] * 11, sequence=1)
    tracker.observe(2, [0] * 11, [42604.57] * 11, sequence=2)
    for sequence in range(3, 8):
        tracker.observe(sequence, [7] * 11, [10.0] * 11,
                        sequence=sequence)
    assert tracker.state == probe.STATE_MODE7_STABLE
    stable = tracker.stable_frames()
    assert len(stable) == 5
    assert all(frame.positions_deg == (10.0,) * 11 for frame in stable)


def test_unexpected_returned_mode_is_semantic_rejection_not_timeout():
    tracker = probe.Mode7TransitionTracker()
    tracker.observe(1, [7] * 10 + [8], [0.0] * 11, sequence=1)
    assert tracker.state == probe.STATE_SEMANTIC_REJECTED
    assert tracker.semantic_rejection_reason == (
        "unexpected returned joint mode(s): J11=mode8"
    )


def test_mode_loss_after_stability_fails_post_stable_observation():
    tracker = probe.Mode7TransitionTracker()
    for sequence in range(1, 6):
        tracker.observe(sequence, [7] * 11, [0.0] * 11,
                        sequence=sequence)
    assert tracker.state == probe.STATE_MODE7_STABLE
    assert tracker.stable_reached_at_index == 4
    tracker.observe(6, [7] * 10 + [0], [0.0] * 11, sequence=6)
    assert tracker.state == probe.STATE_SEMANTIC_REJECTED
    assert "lost after the stability milestone" in (
        tracker.semantic_rejection_reason
    )
    assert tracker.observation_complete is False


def test_mode7_multiturn_after_stability_is_semantic_rejection():
    tracker = probe.Mode7TransitionTracker()
    for sequence in range(1, 6):
        tracker.observe(sequence, [7] * 11, [0.0] * 11,
                        sequence=sequence)
    positions = [0.0] * 11
    positions[4] = 42604.57
    tracker.observe(6, [7] * 11, positions, sequence=6)
    assert tracker.state == probe.STATE_SEMANTIC_REJECTED
    assert "multi-turn mode7 position" in tracker.semantic_rejection_reason


def test_fault_after_stability_still_hard_fails_with_partial_tracker():
    class FaultAfterEightFrames(_StubProbeDriver):
        @property
        def fault_active(self):
            return self._sequence >= 8

    driver = FaultAfterEightFrames(frames=30)
    with pytest.raises(probe.Mode7ProbeHardFail, match="fault") as raised:
        probe._observe_transition(driver)
    tracker = raised.value._mode7_transition_tracker
    assert tracker.stable_reached_at_index == 4
    assert len(tracker.frames) == 8
    assert tracker.observation_complete is False


def _stub_phase_a(driver, duration):
    return {
        "feedback_rate_hz": 100.0,
        "fault": 0,
        "modes": [0] * 11,
        "null_last_deg": [0.0] * 11,
        "domain": FEEDBACK_DOMAIN_NON_CANONICAL_DIAGNOSTIC,
    }


def test_pass_operator_movement_and_timeout_all_restore_and_write_artifacts(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(probe, "run_phase_a", _stub_phase_a)
    cases = [
        ("pass", _StubProbeDriver(), iter(["YES", ""]), 0),
        ("movement", _StubProbeDriver(), iter(["YES", "yes"]), 8),
        (
            "timeout",
            _StubProbeDriver(
                frame_modes=[[0] * 11], repeat_last=True
            ),
            iter(["YES"]),
            6,
        ),
    ]
    for name, driver, answers, expected_code in cases:
        driver.vendor_user_ranges = [(-1.0, 1.0)] * 11
        monkeypatch.setattr(probe, "_probe_driver", lambda *a, d=driver, **k: d)
        artifact_dir = tmp_path / name
        code = probe.run_full_probe(
            "stub", 0, Path("."), 0.01, True,
            artifact_dir=artifact_dir,
            confirm_fn=lambda prompt, a=answers: next(a),
        )
        assert code == expected_code
        assert driver.hold_passive_calls >= 2
        assert driver.lifecycle[-2:] == ["stop", "disconnect"]
        assert {
            path.name for path in artifact_dir.iterdir()
        } == {"metadata.json", "summary.json", "transition_samples.csv"}


@pytest.mark.parametrize(
    ("raised", "expected_code", "expected_status"),
    [
        (EOFError(), 10, "OPERATOR_INPUT_EOF_ABORT"),
        (KeyboardInterrupt(), 130, "KEYBOARD_INTERRUPT_ABORT"),
        (RuntimeError("prompt failed"), 11, "UNEXPECTED_EXCEPTION"),
    ],
)
def test_prompt_abort_paths_restore_disconnect_and_preserve_partial_artifact(
    tmp_path, monkeypatch, raised, expected_code, expected_status
):
    monkeypatch.setattr(probe, "run_phase_a", _stub_phase_a)
    driver = _StubProbeDriver()
    driver.vendor_user_ranges = [(-1.0, 1.0)] * 11
    monkeypatch.setattr(probe, "_probe_driver", lambda *a, **k: driver)

    def confirm(prompt):
        if "SEND" in prompt:
            return "YES"
        raise raised

    artifact_dir = tmp_path / expected_status
    code = probe.run_full_probe(
        "stub", 0, Path("."), 0.01, True,
        artifact_dir=artifact_dir, confirm_fn=confirm,
    )
    assert code == expected_code
    summary = json.loads((artifact_dir / "summary.json").read_text())
    assert summary["status"] == expected_status
    assert summary["NO_VISIBLE_ENTRY_MOTION"] == "NOT_CONFIRMED"
    assert summary["mode_null_restore_after_mode7"] is True
    assert len((artifact_dir / "transition_samples.csv").read_text().splitlines()) == 21
    assert driver.lifecycle[-2:] == ["stop", "disconnect"]


def test_fault_after_submission_restores_mode_null_and_writes_empty_csv(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(probe, "run_phase_a", _stub_phase_a)
    driver = _StubProbeDriver()
    driver.fault_active = True
    monkeypatch.setattr(probe, "_probe_driver", lambda *a, **k: driver)
    code = probe.run_full_probe(
        "stub", 0, Path("."), 0.01, True,
        artifact_dir=tmp_path, confirm_fn=lambda prompt: "YES",
    )
    assert code == 5
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["status"] == "HARD_FAIL"
    assert summary["NO_VISIBLE_ENTRY_MOTION"] == "NOT_CONFIRMED"
    assert summary["mode_null_restore_after_mode7"] is True
    assert (tmp_path / "transition_samples.csv").read_text().count("\n") == 1
    assert driver.lifecycle[-2:] == ["stop", "disconnect"]
