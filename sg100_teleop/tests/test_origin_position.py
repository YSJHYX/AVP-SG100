"""Origin-position opt-out tests; use fake backends, never real hardware."""

from __future__ import annotations

import numpy as np
import pytest

from sg100_teleop.sim.run_avp_arm_hand_sim import (
    REAL_STARTUP_ARM_Q_RAD,
    REAL_STARTUP_ARM_SPEED_PERCENT,
    REAL_STARTUP_HAND_Q_RAD,
    _apply_real_startup_pose,
    build_arg_parser,
)


def test_origin_position_defaults_to_true() -> None:
    args = build_arg_parser().parse_args(["--ip", "fake-avp"])
    assert args.origin_position is True


@pytest.mark.parametrize("flag", ["--origin_position", "--origin-position"])
@pytest.mark.parametrize(
    "value,expected", [("true", True), ("false", False), ("TRUE", True), ("False", False)]
)
def test_origin_position_parses_explicit_boolean(flag, value, expected) -> None:
    args = build_arg_parser().parse_args(["--ip", "fake-avp", flag, value])
    assert args.origin_position is expected


def test_origin_position_accepts_equals_form() -> None:
    args = build_arg_parser().parse_args(["--ip", "fake-avp", "--origin_position=false"])
    assert args.origin_position is False


@pytest.mark.parametrize("value", ["yes", "no", "0", "1", ""])
def test_origin_position_rejects_invalid_values(value) -> None:
    with pytest.raises(SystemExit) as exc:
        build_arg_parser().parse_args(["--ip", "fake-avp", "--origin_position", value])
    assert exc.value.code == 2


def test_origin_position_requires_an_explicit_value() -> None:
    with pytest.raises(SystemExit) as exc:
        build_arg_parser().parse_args(["--ip", "fake-avp", "--origin_position"])
    assert exc.value.code == 2


def test_origin_position_is_visible_in_help() -> None:
    help_text = build_arg_parser().format_help()
    assert "--origin_position {true,false}" in help_text
    assert "--origin-position {true,false}" in help_text
    assert "default: true" in help_text


class NoHardwareAccess:
    def __getattr__(self, name):
        raise AssertionError(f"Opt-out must not access hardware or completion log: {name}")


def test_false_sends_no_arm_or_hand_commands(capsys) -> None:
    args = build_arg_parser().parse_args(["--ip", "fake-avp", "--origin_position", "false"])
    _apply_real_startup_pose(
        NoHardwareAccess(),
        NoHardwareAccess(),
        origin_position=args.origin_position,
        real_error_recorder=NoHardwareAccess(),
    )
    output = capsys.readouterr().out
    assert "origin_position=false" in output
    assert "skipping SG100 and RM65 initial-pose commands" in output
    assert "C anchor + A enable still required" in output


class FakeHand:
    def __init__(self, events, *, reject=False):
        self.events = events
        self.reject = reject

    def send_positions_rad(self, q):
        assert q is not REAL_STARTUP_HAND_Q_RAD
        self.events.append(("hand", q.copy()))
        return None if self.reject else "accepted-hand-command"


class FakeArm:
    def __init__(self, events, *, fail=False):
        self.events = events
        self.fail = fail

    def movej_blocking(self, q, *, speed_percent):
        assert q is not REAL_STARTUP_ARM_Q_RAD
        self.events.append(("arm", q.copy(), speed_percent))
        if self.fail:
            raise RuntimeError("fake planned move failed")
        return q.copy()


class FakeRecorder:
    def __init__(self):
        self.records = []

    def record(self, **kwargs):
        self.records.append(kwargs)


@pytest.mark.parametrize("explicit_true", [False, True])
def test_default_and_true_preserve_original_targets_order_and_log(explicit_true) -> None:
    events = []
    recorder = FakeRecorder()
    kwargs = {"origin_position": True} if explicit_true else {}
    _apply_real_startup_pose(
        FakeArm(events), FakeHand(events), real_error_recorder=recorder, **kwargs
    )
    assert [event[0] for event in events] == ["hand", "arm"]
    np.testing.assert_array_equal(events[0][1], REAL_STARTUP_HAND_Q_RAD)
    np.testing.assert_array_equal(events[1][1], REAL_STARTUP_ARM_Q_RAD)
    assert events[1][2] == REAL_STARTUP_ARM_SPEED_PERCENT
    assert len(recorder.records) == 1
    record = recorder.records[0]
    assert record["event"] == "INITIAL_POSE_COMPLETE"
    assert record["details"]["hand_command"] == "accepted-hand-command"
    np.testing.assert_array_equal(record["details"]["arm_measured_rad"], REAL_STARTUP_ARM_Q_RAD)


def test_rejected_hand_startup_target_never_moves_arm() -> None:
    events = []
    recorder = FakeRecorder()
    with pytest.raises(RuntimeError, match="rejected the startup hand target"):
        _apply_real_startup_pose(
            FakeArm(events), FakeHand(events, reject=True), real_error_recorder=recorder
        )
    assert [event[0] for event in events] == ["hand"]
    assert not recorder.records


def test_failed_arm_startup_propagates_to_existing_cleanup() -> None:
    events = []
    recorder = FakeRecorder()
    with pytest.raises(RuntimeError, match="fake planned move failed"):
        _apply_real_startup_pose(
            FakeArm(events, fail=True), FakeHand(events), real_error_recorder=recorder
        )
    assert [event[0] for event in events] == ["hand", "arm"]
    assert not recorder.records
