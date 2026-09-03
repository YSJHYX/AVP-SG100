"""Offline regressions for the blocked SG100 N1 characterization tool."""

from __future__ import annotations

import csv
import json
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from sg100_teleop.robot.sg100_driver import HAND_DOF, SG100HardwareMap
from sg100_teleop.robot.sg100_sdk_driver import (
    DEFAULT_SDK_DIR,
    MODE_IMPEDANCE,
    MODE_JOINT_POSITION,
    MODE_NULL,
    MODE_POSITION,
    _load_wrapper_library,
)
from sg100_teleop.tools.sg100_sign_characterization import (
    CANDIDATE_DIRECTION_SIGNS,
    FeedbackRecorder,
    FORBIDDEN_POSITION_MODES,
    SessionConfig,
    TEST_CANONICAL_DELTA_SIGN,
    TEST_PHYSICAL_INSTRUCTIONS,
    analyze_trial,
    expected_sdk_delta_sign,
    finalize_partial_session,
    prompt_command,
    summarize,
    verify_null_frame,
)


ROOT = Path(__file__).resolve().parents[2]
CONFIG = SessionConfig()
JOINT_ORDER = SG100HardwareMap().software_joint_order


def _delta(index: int, value: float) -> np.ndarray:
    delta = np.zeros(HAND_DOF)
    delta[index] = value
    return delta


def _trial(
    expected_index: int,
    direction_sign: float,
    move_delta: np.ndarray,
    return_offset: np.ndarray | None = None,
    name: str | None = None,
    test_delta_sign: float = +1.0,
) -> dict:
    baseline = np.zeros(HAND_DOF)
    move = np.tile(baseline, (20, 1)) + move_delta
    returned = baseline + (
        np.zeros(HAND_DOF) if return_offset is None else return_offset
    )
    return analyze_trial(
        name or JOINT_ORDER[expected_index], expected_index, direction_sign,
        baseline, move, returned, CONFIG, test_delta_sign=test_delta_sign,
    )


@pytest.fixture(scope="module")
def wrapper_lib():
    return _load_wrapper_library(sdk_dir=DEFAULT_SDK_DIR)


def test_outgoing_control_is_null_for_all_11(wrapper_lib) -> None:
    ok, modes = verify_null_frame(wrapper_lib)
    assert ok
    assert modes == [MODE_NULL] * HAND_DOF


def test_active_position_modes_are_forbidden(wrapper_lib) -> None:
    payload = np.zeros(HAND_DOF, dtype=np.float32)
    frame = np.zeros(176, dtype=np.uint8)
    for mode in (MODE_POSITION, MODE_JOINT_POSITION, MODE_IMPEDANCE):
        assert mode in FORBIDDEN_POSITION_MODES
        wrapper_lib.sg100_test_serialize_hand_control(
            payload, HAND_DOF, mode, 0, 0, 0, 0, frame, frame.size
        )
        assert all(frame[j * 16 + 1] == mode for j in range(HAND_DOF))


def test_execute_is_fail_closed_before_transport() -> None:
    result = subprocess.run(
        [sys.executable, "-m",
         "sg100_teleop.tools.sg100_sign_characterization", "--execute"],
        capture_output=True, text=True, cwd=str(ROOT),
    )
    assert result.returncode == 2
    assert "HARDWARE EXECUTION BLOCKED" in result.stdout
    assert "No SDK was loaded and no transport was opened" in result.stdout


def test_tool_has_no_position_or_homing_calls() -> None:
    source = (
        ROOT / "sg100_teleop" / "tools" / "sg100_sign_characterization.py"
    ).read_text()
    for banned in (
        "driver.send_positions_rad", "sg100_set_hand_control(",
        "sg100_c_send_positions", ".SetJointPositions(",
        ".HomingZero(", ".SetJointZero(",
    ):
        assert banned not in source


def test_dominant_channel_and_direction_are_separate_from_scale() -> None:
    result = _trial(4, +1, _delta(4, 1800.0))
    assert result["observed_sdk_joint"] == "J5"
    assert result["index_verdict"] == "SUPPORTED"
    assert result["direction_verdict"] == "SUPPORTED"
    assert result["scale_verdict"] == "UNCHARACTERIZED"
    assert result["dominant_abs_delta_sdk_units"] == pytest.approx(1800.0)


def test_continuous_large_signal_supports_index_and_direction() -> None:
    baseline = np.zeros(HAND_DOF)
    move = np.zeros((200, HAND_DOF))
    move[:, 0] = np.linspace(0.0, 1854.0, len(move))
    result = analyze_trial(
        "thumb_j1", 0, +1, baseline, move, baseline, CONFIG
    )
    assert result["index_verdict"] == "SUPPORTED"
    assert result["direction_verdict"] == "SUPPORTED"
    assert result["scale_verdict"] == "UNCHARACTERIZED"


def test_wrong_direction_is_not_supported() -> None:
    result = _trial(6, -1, _delta(6, +22.0))
    assert result["index_verdict"] == "SUPPORTED"
    assert result["direction_verdict"] == "NOT_SUPPORTED"


def test_insufficient_signal_does_not_characterize_direction_or_scale() -> None:
    result = _trial(9, +1, _delta(9, 0.8))
    assert result["index_verdict"] == "INSUFFICIENT"
    assert result["direction_verdict"] == "NOT_EVALUATED"
    assert result["scale_verdict"] == "UNCHARACTERIZED"


def test_return_error_is_not_interpreted_as_physical_degrees() -> None:
    result = _trial(
        3, +1, _delta(3, 20.0), return_offset=_delta(3, 5.5)
    )
    assert result["return_error_sdk_units"] == pytest.approx(5.5)
    assert result["return_verdict"] == "MANUAL_CONFIRMATION_REQUIRED"
    assert result["return_ok"] is None


def test_only_joint0_measured_means_other_10_not_tested() -> None:
    summary = summarize(
        [_trial(0, +1, _delta(0, 1854.0))],
        joint_order=JOINT_ORDER,
        session_status="QUIT",
    )
    assert summary["hardware_tested_joint_count"] == 1
    assert summary["hardware_not_tested_joint_count"] == 10
    assert summary["per_joint"][0]["hardware_status"] == (
        "PARTIALLY_CHARACTERIZED"
    )
    assert all(
        row["hardware_status"] == "NOT_TESTED"
        for row in summary["per_joint"][1:]
    )
    assert all(
        status == "NOT_HARDWARE_CONFIRMED"
        for name, status in summary["candidate_direction_status"].items()
        if name != "thumb_j1"
    )


def test_partial_session_mapping_is_blocked() -> None:
    summary = summarize(
        [_trial(0, +1, _delta(0, 20.0))], joint_order=JOINT_ORDER
    )
    assert summary["canonical_mapping_verdict"] == "BLOCKED"
    assert summary["physical_to_sdk_scale"] == "UNCHARACTERIZED"


def test_selftest_cannot_leak_into_hardware_state() -> None:
    offline = []
    for index in range(HAND_DOF):
        test_sign = TEST_CANONICAL_DELTA_SIGN[index]
        direction = CANDIDATE_DIRECTION_SIGNS[index]
        offline.append(_trial(
            index, direction, _delta(index, direction * test_sign * 22.0),
            test_delta_sign=test_sign,
        ))
    summary = summarize(
        offline, joint_order=JOINT_ORDER, evidence_source="SELFTEST",
        session_status="SELFTEST_COMPLETE",
    )
    assert summary["hardware_tested_joint_count"] == 0
    assert summary["canonical_mapping_verdict"] == "BLOCKED"
    assert all(
        row["hardware_status"] == "NOT_TESTED"
        for row in summary["per_joint"]
    )


@pytest.mark.parametrize(
    "input_fn",
    [lambda _: None, lambda _: (_ for _ in ()).throw(EOFError())],
)
def test_none_or_eof_at_retry_prompt_quits_safely(input_fn) -> None:
    assert prompt_command("retry? ", ("r", "s", "q"), input_fn=input_fn) == "q"


def test_retry_prompt_accepts_only_explicit_commands() -> None:
    answers = iter(["anything", "R"])
    assert prompt_command(
        "retry? ", ("r", "s", "q"), input_fn=lambda _: next(answers)
    ) == "r"


def test_feedback_recorder_deduplicates_cached_callback_sequence() -> None:
    feedback = SimpleNamespace(
        sequence=7,
        valid=True,
        raw_positions=np.zeros(HAND_DOF),
        received_monotonic_ns=123,
    )
    recorder = FeedbackRecorder(lambda: feedback)
    recorder.start()
    time.sleep(0.02)
    recorder.stop()
    assert len(recorder.all_rows()) == 1


class _FakeRecorder:
    def __init__(self) -> None:
        self.stopped = False

    def stop(self) -> None:
        self.stopped = True

    def all_rows(self) -> list[tuple]:
        return [(1, "baseline", 0, *([0.0] * HAND_DOF))]


class _FakeDriver:
    def __init__(self) -> None:
        self.stop_called = False
        self.disconnect_called = False

    def stop(self) -> None:
        self.stop_called = True

    def disconnect(self) -> None:
        self.disconnect_called = True


@pytest.mark.parametrize("session_status", ["QUIT", "EOF"])
def test_quit_or_eof_saves_partial_artifact_and_cleans_sdk(
    tmp_path, session_status
) -> None:
    recorder = _FakeRecorder()
    driver = _FakeDriver()
    summary = finalize_partial_session(
        driver=driver, recorder=recorder, out_dir=tmp_path,
        metadata={"mode": "hardware"},
        results=[_trial(0, +1, _delta(0, 1800.0))],
        joint_order=JOINT_ORDER, skipped_joints=[],
        session_status=session_status,
    )
    assert recorder.stopped
    assert driver.stop_called and driver.disconnect_called
    assert (tmp_path / "samples.csv").exists()
    saved = json.loads((tmp_path / "summary.json").read_text())
    assert saved["session_status"] == session_status
    assert saved["hardware_tested_joint_count"] == 1
    assert saved["hardware_not_tested_joint_count"] == 10
    assert summary == saved


def test_selftest_does_not_mutate_mappings_and_stays_offline(tmp_path) -> None:
    hardware_map_path = (
        ROOT / "sg100_teleop" / "config" / "sg100_hardware_map.yaml"
    )
    json_map_path = Path(DEFAULT_SDK_DIR) / "joint_angle_map.json"
    before = (hardware_map_path.read_bytes(), json_map_path.read_bytes())
    result = subprocess.run(
        [sys.executable, "-m",
         "sg100_teleop.tools.sg100_sign_characterization",
         "--selftest", "--out-root", str(tmp_path)],
        capture_output=True, text=True, cwd=str(ROOT),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert (hardware_map_path.read_bytes(), json_map_path.read_bytes()) == before
    session = next(tmp_path.glob("sg100_n1_sign_selftest_*"))
    metadata = json.loads((session / "metadata.json").read_text())
    summary = json.loads((session / "summary.json").read_text())
    assert metadata["physical_to_sdk_scale"] == "UNCHARACTERIZED"
    assert metadata["candidate_direction_signs"] == CANDIDATE_DIRECTION_SIGNS
    assert summary["hardware_tested_joint_count"] == 0
    assert summary["canonical_mapping_verdict"] == "BLOCKED"
    with (session / "samples.csv").open() as f:
        header = next(csv.reader(f))
    assert header[3:] == [
        f"j{i + 1}_sdk_callback_position" for i in range(HAND_DOF)
    ]


def test_direction_contract_for_negative_canonical_motion() -> None:
    assert expected_sdk_delta_sign(+1, -1) == -1
    assert expected_sdk_delta_sign(-1, -1) == +1
    result = _trial(
        6, -1, _delta(6, +22.0), test_delta_sign=-1,
    )
    assert result["direction_verdict"] == "SUPPORTED"


def test_direction_instructions_cover_all_11() -> None:
    assert len(TEST_CANONICAL_DELTA_SIGN) == HAND_DOF
    assert len(TEST_PHYSICAL_INSTRUCTIONS) == HAND_DOF
