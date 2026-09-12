"""MODE7 单关节 active-hold / micro-motion 离线安全回归。"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from sg100_teleop.robot.sg100_driver import (
    HAND_DOF, MODE_JOINT_POSITION, SG100HandFeedback, SG100HardwareMap,
)
from sg100_teleop.robot.sg100_sdk_driver import (
    DEFAULT_SDK_DIR, SG100SDKDriver, _load_wrapper_library, load_vendor_user_ranges,
)
from sg100_teleop.tools import sg100_mode7_active_micro_motion as active


ROOT = Path(__file__).resolve().parents[2]


def _history(values=None):
    values = np.zeros(HAND_DOF) if values is None else np.asarray(values)
    return {
        "status": "COMPLETED_PASS_ANGLE_DOMAIN_SUPPORTED",
        "MODE7_JOINT_ANGLE_SCALE_DOMAIN": "SUPPORTED",
        "canonical_valid": False,
        "per_joint": [
            {
                "joint": f"J{i + 1}", "mode7_median_deg": float(values[i]),
                "multi_turn_present": False,
                "sdk_command_range_compatible": True,
            }
            for i in range(HAND_DOF)
        ],
    }


def _frame(sequence, values, *, phase="micro_motion", modes=None,
           command_index=3, timestamp=None):
    return active.ActiveFrame(
        phase=phase, sequence=sequence,
        received_monotonic_ns=sequence if timestamp is None else timestamp,
        modes=tuple([MODE_JOINT_POSITION] * HAND_DOF if modes is None else modes),
        positions_deg=tuple(float(v) for v in values),
        requested_active_target_deg=None, command_index=command_index,
    )


def _ranges():
    ranges = load_vendor_user_ranges(DEFAULT_SDK_DIR)
    assert ranges is not None
    return ranges


# test_default_cli_is_true_zero_hardware_dry_run — REMOVED 2026-09-04:
# 依赖的 recording/sg100_mode7_probe_* 历史 artifact 已按用户决定从
# 工作区删除（dry-run 的 seed 选择无数据源）；删除原因 = artifact
# 缺失，非行为回归。


def test_j7_alias_and_canonical_sign_flip_are_applied_exactly_once():
    assert active.parse_joint_selector("middle_j1") == 7
    assert active.parse_joint_selector("J7") == 7
    hardware_map = SG100HardwareMap()
    assert hardware_map.gain[6] == -1.0
    canonical = np.zeros(HAND_DOF)
    canonical[6] = -20.0
    sdk = active.apply_f_cmd_once_deg(hardware_map, canonical)
    assert sdk[6] == pytest.approx(20.0)
    plan = active.plan_j7_sign_flip(hardware_map, _ranges(), -0.5)
    assert plan["canonical_requested_delta_deg"] == -0.5
    assert plan["sdk_delta_deg"] == 0.5
    assert plan["F_cmd_application_count"] == 1
    assert plan["raw_feedback_used_as_command_seed"] is False
    assert plan["seed_source"] == "MODE7_ZERO_OUTPUT_CACHED_COMMAND_TARGET"


def test_j7_direction_and_range_are_fail_closed():
    with pytest.raises(ValueError):
        active.resolve_canonical_delta_deg(7, 0.5, +0.5)
    with pytest.raises(ValueError):
        active.resolve_canonical_delta_deg(7, 0.5, -1.01)


# test_real_j6_artifact_parser_upgrades_and_classifies_crc_transient —
# REMOVED 2026-09-04: 读取 recording/sg100_mode7_active_20260903_202121_J6
# artifact（已按用户决定删除）。删除原因 = artifact 缺失，非行为回归。


def test_crc_console_telemetry_matches_exact_sdk_line():
    wrapper = _load_wrapper_library(sdk_dir=DEFAULT_SDK_DIR)
    before = int(wrapper.sg100_crc_error_count())
    wrapper.sg100_test_observe_console_line(b"unrelated CRC words")
    assert int(wrapper.sg100_crc_error_count()) == before
    wrapper.sg100_test_observe_console_line(
        b"[RS485Adapter] CRC error: recv=0x3cd2 calc=0x73eec6d"
    )
    assert int(wrapper.sg100_crc_error_count()) == before + 1
    timestamps = np.zeros(64, dtype=np.uint64)
    count = wrapper.sg100_copy_crc_error_timestamps(timestamps, 64)
    assert count >= 1 and int(timestamps[count - 1]) > 0


def test_first_joint_filter_rejects_unresolved_and_nonidentity():
    hardware_map = SG100HardwareMap()
    eligible = active.eligible_first_joint_indices(hardware_map, _ranges(), _history())
    assert 1 not in eligible and 6 not in eligible  # J2/J7
    assert eligible == [2, 5, 7, 10]  # J3/J6/J8/J11
    hardware_map.gain[5] = -1.0
    assert 5 not in active.eligible_first_joint_indices(
        hardware_map, _ranges(), _history()
    )


def test_latest_evidence_selects_j6_by_worst_direction_headroom():
    current = np.array([1, -7, -9.8, 3, 4, -0.725, 1.6, -10.6, 3, 5, -6.3])
    choice = active.select_first_test_joint(
        SG100HardwareMap(), _ranges(), _history(current), current
    )
    assert choice.joint_id == 6
    assert choice.minimum_headroom_deg == pytest.approx(59.275)


@pytest.mark.parametrize("joint_id", [2, 7, 4])
def test_explicit_unsafe_first_joint_is_rejected(joint_id):
    with pytest.raises(active.ActiveMicroMotionAbort):
        active.select_first_test_joint(
            SG100HardwareMap(), _ranges(), _history(), np.zeros(HAND_DOF),
            requested_joint_id=joint_id,
        )


def test_current_seed_must_be_finite_inside_range_with_margin():
    current = np.zeros(HAND_DOF)
    current[5] = np.nan
    with pytest.raises(active.ActiveMicroMotionAbort):
        active.select_first_test_joint(
            SG100HardwareMap(), _ranges(), _history(), current
        )
    current = np.zeros(HAND_DOF)
    current[2] = current[5] = current[7] = current[10] = 89.0
    with pytest.raises(active.ActiveMicroMotionAbort):
        active.select_first_test_joint(
            SG100HardwareMap(), _ranges(), _history(current), current
        )


@pytest.mark.parametrize("delta", [0.0, -0.5, 1.0001, np.nan, np.inf])
def test_delta_hard_max_and_finiteness(delta):
    with pytest.raises(ValueError):
        active.validate_delta(delta)
    assert active.validate_delta(0.5) == 0.5


def test_direction_uses_larger_headroom_and_never_clamps():
    current = np.zeros(HAND_DOF)
    choice = active.select_first_test_joint(
        SG100HardwareMap(), _ranges(), _history(current), current,
        requested_joint_id=6,
    )
    plan = active.plan_micro_target(choice, 0.5)
    assert plan["selected_direction"] == "+"
    assert plan["final_target_deg"] == 0.5
    assert plan["expected_physical_motion"] == "index distal flexion toward palm"
    unsafe = active.FirstJointChoice(
        5, 6, "index_j3", 86.0, -60.0, 90.0, 146.0, 4.0, 4.0, "test"
    )
    with pytest.raises(active.ActiveMicroMotionAbort):
        active.plan_micro_target(unsafe, 0.5)


def test_control_has_exactly_one_driven_joint_and_other_ten_zero():
    control = active.build_single_driven_control(5, np.zeros(HAND_DOF))
    assert control["modes"].tolist() == [MODE_JOINT_POSITION] * HAND_DOF
    producing = np.flatnonzero(
        (control["p"] != 0) | (control["i"] != 0) |
        (control["d"] != 0) | (control["output_limit"] != 0)
    )
    assert producing.tolist() == [5]
    assert np.count_nonzero(control["output_limit"] == 0) == 10


def test_native_per_joint_serializer_matches_exact_hand_control():
    wrapper = _load_wrapper_library(sdk_dir=DEFAULT_SDK_DIR)
    control = active.build_single_driven_control(5, np.arange(HAND_DOF))
    out = np.zeros(176, dtype=np.uint8)
    size = wrapper.sg100_test_serialize_hand_control_per_joint(
        np.asarray(control["positions"], dtype=np.float32), control["modes"],
        control["p"], control["i"], control["d"], control["output_limit"],
        HAND_DOF, out, out.size,
    )
    assert size == 176
    for index in range(HAND_DOF):
        jid, mode, position, p, i, d, limit = struct.unpack_from(
            "<BBfHHHH", out, index * 16
        )
        assert jid == index + 1 and mode == MODE_JOINT_POSITION
        assert position == pytest.approx(float(index))
        if index == 5:
            assert p > 0 and limit > 0
        else:
            assert (p, i, d, limit) == (0, 0, 0, 0)


def test_driver_active_path_bypasses_maps_has_one_native_call_and_sdk_clamp():
    class Capture:
        def __init__(self): self.calls = []
        def sg100_set_hand_control(self, *args):
            self.calls.append(args)
            return 1

    driver = SG100SDKDriver("/dev/null", sdk_dir=DEFAULT_SDK_DIR)
    capture = Capture()
    driver._lib = capture
    driver._connected = driver._realtime = True
    driver.hardware_map.gain[:] = -9.0
    positions = np.zeros(HAND_DOF)
    positions[5] = 1.25
    event = driver.send_mode7_active_single_joint(5, positions)
    assert event is not None and len(capture.calls) == 1
    sent = capture.calls[0]
    np.testing.assert_allclose(sent[0], positions)
    assert np.flatnonzero(sent[5]).tolist() == [5]
    positions[5] = 900.0
    clamped = driver.send_mode7_active_single_joint(5, positions)
    assert clamped is not None
    assert len(capture.calls) == 2
    assert capture.calls[-1][0][5] == pytest.approx(
        driver.vendor_user_ranges[5][1]
    )
    positions[5] = 1.25
    with pytest.raises(ValueError, match="vendor-evidenced"):
        driver.send_mode7_active_single_joint(5, positions, output_limit=0.4)
    assert len(capture.calls) == 2


def test_two_stage_gate_blocks_skip_requires_move_and_allows_one_delta():
    gate = active.ActiveSequenceGate()
    with pytest.raises(active.ActiveMicroMotionAbort):
        gate.confirm_move("MOVE")
    with pytest.raises(active.ActiveMicroMotionAbort):
        gate.accept_hold("FAIL", False)
    gate.accept_hold("PASS", False)
    with pytest.raises(active.ActiveMicroMotionAbort):
        gate.confirm_move("")
    gate.confirm_move("MOVE")
    gate.record_micro_command()
    with pytest.raises(active.ActiveMicroMotionAbort):
        gate.record_micro_command()
    assert gate.micro_command_count == 1


def test_hold_uses_relative_delta_and_rejects_jump():
    baseline = np.full(HAND_DOF, 12.0)
    before = [_frame(i, baseline, phase="mode7_passive") for i in range(1, 7)]
    after = [_frame(20 + i, baseline + 0.1, phase="active_hold") for i in range(6)]
    result = active.analyze_hold(before, after, command_ns=10, joint_index=5)
    assert result["verdict"] == "PASS"
    assert result["hold_error_deg"] == pytest.approx(0.1)
    jumped = [_frame(30 + i, baseline + np.eye(1, HAND_DOF, 5)[0],
                     phase="active_hold") for i in range(6)]
    result = active.analyze_hold(before, jumped, command_ns=10, joint_index=5)
    assert result["verdict"] == "FAIL"
    assert result["reason"] == "CURRENT_POSITION_SEED_REJECTED"


def test_feedback_analysis_uses_fresh_samples_and_detects_dominant_direction():
    zero = np.zeros(HAND_DOF)
    before = [_frame(i, zero) for i in range(1, 8)]
    stale = _frame(8, np.eye(1, HAND_DOF, 2)[0] * 9, timestamp=99)
    moved = zero.copy(); moved[5] = 0.45; moved[2] = 0.02
    post = [stale] + [_frame(20 + i, moved, timestamp=101 + i) for i in range(6)]
    result = active.analyze_motion(
        before, post, command_ns=100, expected_joint_index=5,
        commanded_delta_deg=0.5,
    )
    assert result["verdict"] == "PASS"
    assert result["dominant_feedback_joint"] == "J6"
    assert result["observed_feedback_direction"] == "+"
    assert result["fresh_post_command_samples"] == 6


def test_wrong_joint_wrong_direction_and_excess_are_fail_closed():
    zero = np.zeros(HAND_DOF)
    before = [_frame(i, zero) for i in range(1, 8)]
    wrong_joint = zero.copy(); wrong_joint[2] = 0.5; wrong_joint[5] = 0.1
    result = active.analyze_motion(
        before, [_frame(20 + i, wrong_joint) for i in range(6)],
        command_ns=10, expected_joint_index=5, commanded_delta_deg=0.5,
    )
    assert result["reason"] == "WRONG_DOMINANT_JOINT"
    reverse = zero.copy(); reverse[5] = -0.4
    result = active.analyze_motion(
        before, [_frame(30 + i, reverse) for i in range(6)],
        command_ns=10, expected_joint_index=5, commanded_delta_deg=0.5,
    )
    assert result["reason"] == "WRONG_FEEDBACK_DIRECTION"
    runaway = zero.copy(); runaway[5] = 2.0
    result = active.analyze_motion(
        before, [_frame(40 + i, runaway) for i in range(6)],
        command_ns=10, expected_joint_index=5, commanded_delta_deg=0.5,
    )
    assert result["reason"] == "EXCESSIVE_TARGET_JOINT_EXCURSION"


def test_below_detection_is_pending_not_a_false_direction_claim():
    zero = np.zeros(HAND_DOF)
    tiny = zero.copy(); tiny[5] = 0.01
    result = active.analyze_motion(
        [_frame(i, zero) for i in range(1, 8)],
        [_frame(20 + i, tiny) for i in range(6)], command_ns=10,
        expected_joint_index=5, commanded_delta_deg=0.5,
    )
    assert result["verdict"] == "PENDING"
    assert result["observed_feedback_direction"] == "NONE"


def test_oscillation_is_not_accepted_as_bounded_motion():
    zero = np.zeros(HAND_DOF)
    samples = []
    for index, value in enumerate([0.15, 0.45, 0.20, 0.48, 0.22, 0.46]):
        positions = zero.copy(); positions[5] = value
        samples.append(_frame(20 + index, positions))
    result = active.analyze_motion(
        [_frame(i, zero) for i in range(1, 8)], samples, command_ns=10,
        expected_joint_index=5, commanded_delta_deg=0.5,
    )
    assert result["verdict"] == "FAIL"
    assert result["reason"] == "OSCILLATION_DETECTED"


class _CaptureDriver:
    def __init__(self, feedback=None, *, fault=False):
        self.feedback = feedback
        self.fault_active = fault
    def read_feedback(self): return self.feedback


def test_capture_hard_stops_fault_nonfinite_unexpected_mode_and_stall(monkeypatch):
    with pytest.raises(active.ActiveMicroMotionAbort, match="fault"):
        active._capture(_CaptureDriver(fault=True), 0, "active_hold", [],
                        command_index=2, target_deg=0, min_frames=1)
    bad_mode = SG100HandFeedback(
        q_rad=np.zeros(11), raw_positions=np.zeros(11), received_monotonic_ns=1,
        sequence=1, valid=True, joint_modes=(0,) * 11,
    )
    with pytest.raises(active.ActiveMicroMotionAbort, match="unexpected mode"):
        active._capture(_CaptureDriver(bad_mode), 0, "active_hold", [],
                        command_index=2, target_deg=0, min_frames=1,
                        require_all_mode7=True)
    nonfinite = dataclasses_replace_feedback(bad_mode, raw_positions=np.full(11, np.nan),
                                             joint_modes=(7,) * 11)
    with pytest.raises(active.ActiveMicroMotionAbort, match="non-finite"):
        active._capture(_CaptureDriver(nonfinite), 0, "active_hold", [],
                        command_index=2, target_deg=0, min_frames=1)
    monkeypatch.setattr(active, "TRANSPORT_STALL_S", -1.0)
    with pytest.raises(active.ActiveMicroMotionAbort, match="transport stall"):
        active._capture(_CaptureDriver(None), 0, "active_hold", [],
                        command_index=2, target_deg=0, min_frames=1)


def dataclasses_replace_feedback(value, **changes):
    import dataclasses
    return dataclasses.replace(value, **changes)


def test_artifact_schema_retains_separated_mapping_and_quarantine(tmp_path):
    path = tmp_path / "artifact"
    frame = _frame(1, np.zeros(11), phase="cleanup", command_index=4)
    metadata = {
        "F_cmd_candidate": "IDENTITY_CANDIDATE_NOT_PRODUCTION_VALIDATED",
        "F_fb_status": "DELTA_MAPPING_PENDING", "canonical_valid": False,
        "dataset": "QUARANTINED",
    }
    summary = {
        "active_hold_verdict": "PASS", "micro_motion_verdict": "FAIL",
        "cleanup_status": "PASS", "canonical_valid": False,
        "automatic_return_motion": False,
    }
    active._write_artifacts(path, metadata, [frame], summary)
    assert {p.name for p in path.iterdir()} == {"metadata.json", "samples.csv", "summary.json"}
    assert json.loads((path / "metadata.json").read_text())["canonical_valid"] is False
    assert "cleanup" in (path / "samples.csv").read_text()


class _SessionDriver:
    instances = []

    def __init__(self, *args, **kwargs):
        self.vendor_user_ranges = _ranges()
        self.calls = 0
        self.active_calls = 0
        self.active_targets = []
        self.cleanup_calls = 0
        self.disconnected = False
        self.fault_active = False
        self.instances.append(self)

    @property
    def diagnostics(self):
        return {"hand_control_calls": self.calls}

    def connect(self): pass
    def start(self): pass
    def enter_realtime(self): self.calls += 1
    def disconnect(self): self.disconnected = True

    def send_mode7_feedback_probe(self):
        self.calls += 1
        return SimpleNamespace(sent_monotonic_ns=1_000)

    def send_mode7_active_single_joint(self, index, positions):
        self.calls += 1
        self.active_calls += 1
        self.active_targets.append(np.asarray(positions, dtype=float).copy())
        return SimpleNamespace(
            sent_monotonic_ns=2_000 if self.active_calls == 1 else 3_000,
            requested_positions_deg=tuple(float(v) for v in positions),
        )

    def hold_passive(self):
        self.calls += 1
        self.cleanup_calls += 1
        return True


def _session_setup(tmp_path, monkeypatch, *, fail_phase=None, failure=None):
    recording_root = tmp_path / "recording"
    evidence = recording_root / "sg100_mode7_probe_20260903_999999"
    evidence.mkdir(parents=True)
    current = np.zeros(HAND_DOF)
    current[[2, 5, 7, 10]] = [-9.8, -0.725, -10.6, -6.3]
    (evidence / "summary.json").write_text(json.dumps(_history(current)))
    sdk_dir = tmp_path / "sdk"
    sdk_dir.mkdir()
    (sdk_dir / "joint_angle_map_v3.0.json").write_bytes(
        (Path(DEFAULT_SDK_DIR) / "joint_angle_map_v3.0.json").read_bytes()
    )
    _SessionDriver.instances.clear()
    monkeypatch.setattr(active, "SG100SDKDriver", _SessionDriver)

    def fake_capture(driver, duration, phase, sink, *, command_index,
                     target_deg, min_frames=1, **kwargs):
        if phase == fail_phase:
            raise failure
        values = current.copy()
        modes = [7] * HAND_DOF
        timestamp = 1_100
        if phase == "active_hold":
            timestamp = 2_100
        elif phase == "micro_motion":
            timestamp = 3_100
            values[5] += 0.5
        elif phase == "cleanup":
            timestamp = 4_100
            modes = [0] * HAND_DOF
        frames = [
            _frame(100 * command_index + i, values, phase=phase, modes=modes,
                   command_index=command_index, timestamp=timestamp + i)
            for i in range(max(min_frames, 10))
        ]
        sink.extend(frames)
        return frames

    monkeypatch.setattr(active, "_capture", fake_capture)
    return argparse.Namespace(
        port="stub", baudrate=1_000_000, joint=None, delta_deg=0.5,
        sdk_dir=sdk_dir, recording_root=recording_root, execute=True,
    )


def test_mocked_full_session_counts_commands_and_never_returns(tmp_path, monkeypatch):
    args = _session_setup(tmp_path, monkeypatch)
    answers = iter(["HOLD", "n", "MOVE", "y"])
    assert active.run_hardware(args, confirm_fn=lambda prompt: next(answers)) == 0
    driver = _SessionDriver.instances[-1]
    assert driver.active_calls == 2  # current hold + exactly one delta
    assert driver.cleanup_calls == 1 and driver.disconnected
    artifact = next(args.recording_root.glob("sg100_mode7_active_*"))
    summary = json.loads((artifact / "summary.json").read_text())
    assert summary["set_hand_control_api_calls"] == 5
    assert summary["exactly_one_micro_delta_command"] is True
    assert summary["automatic_return_motion"] is False


def test_mocked_j7_session_uses_command_zero_seed_not_raw_feedback(
    tmp_path, monkeypatch
):
    args = _session_setup(tmp_path, monkeypatch)
    args.joint = 7
    args.canonical_delta_deg = -0.5

    raw = np.zeros(HAND_DOF)
    raw[6] = 37.0  # 故意与 command-domain seed=0 明显不同。

    def capture(driver, duration, phase, sink, *, command_index,
                target_deg, min_frames=1, **kwargs):
        values = raw.copy()
        if phase == "micro_motion":
            # F_fb sign 未知，反向 callback delta 也不能否决物理方向目视证据。
            values[6] -= 0.3
        modes = [0] * HAND_DOF if phase == "cleanup" else [7] * HAND_DOF
        timestamp = {"mode7_passive": 1_100, "active_hold": 2_100,
                     "micro_motion": 3_100, "cleanup": 4_100}[phase]
        result = [
            _frame(100 * command_index + i, values, phase=phase, modes=modes,
                   command_index=command_index, timestamp=timestamp + i)
            for i in range(max(min_frames, 10))
        ]
        sink.extend(result)
        return result

    monkeypatch.setattr(active, "_capture", capture)
    answers = iter(["ZERO", "HOLD", "n", "MOVE", "y"])
    assert active.run_hardware(args, confirm_fn=lambda prompt: next(answers)) == 0
    driver = _SessionDriver.instances[-1]
    assert driver.active_targets[0][6] == pytest.approx(0.0)
    assert driver.active_targets[1][6] == pytest.approx(0.5)
    assert driver.active_targets[0][6] != raw[6]
    artifact = next(args.recording_root.glob("sg100_mode7_active_*"))
    summary = json.loads((artifact / "summary.json").read_text())
    assert [item["kind"] for item in summary["command_log"]] == [
        "realtime_mode_null_seed", "mode7_passive", "active_hold",
        "micro_motion", "cleanup_mode_null",
    ]


@pytest.mark.parametrize(
    ("failure", "expected_code"),
    [(KeyboardInterrupt(), 130), (EOFError(), 130),
     (active.ActiveMicroMotionAbort("fault"), 6), (RuntimeError("boom"), 6)],
)
def test_post_entry_failures_cleanup_disconnect_and_retain_artifact(
    tmp_path, monkeypatch, failure, expected_code
):
    args = _session_setup(
        tmp_path, monkeypatch, fail_phase="active_hold", failure=failure
    )
    answers = iter(["HOLD"])
    assert active.run_hardware(args, confirm_fn=lambda prompt: next(answers)) == expected_code
    driver = _SessionDriver.instances[-1]
    assert driver.cleanup_calls == 1 and driver.disconnected
    artifact = next(args.recording_root.glob("sg100_mode7_active_*"))
    summary = json.loads((artifact / "summary.json").read_text())
    assert summary["cleanup_status"] == "PASS"
    assert summary["canonical_valid"] is False
    assert summary["exactly_one_micro_delta_command"] is False


def test_tool_source_has_no_forbidden_motion_path_or_map_update():
    source = (ROOT / "sg100_teleop/tools/sg100_mode7_active_micro_motion.py").read_text()
    for banned in ("SetJointPositions", "Homing", "set_zero", "raw RS485",
                   "actual_to_canonical("):
        assert banned not in source
    assert "send_mode7_active_single_joint" in source
    assert '"automatic_return_motion"] = False' in source
