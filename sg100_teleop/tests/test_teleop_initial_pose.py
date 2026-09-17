"""Offline tests for initial-pose integration in the main teleop runner."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from sg100_teleop.sim import run_avp_arm_hand_sim as runner


def _valid_config() -> dict:
    return {
        "version": 1,
        "angle_unit": "deg",
        "arm": {
            "angles_deg": {
                name: 0.0 for name in runner.INITIAL_ARM_JOINT_NAMES
            },
            "speed_percent": 5,
            "timeout_s": 30,
            "tolerance_deg": 1.0,
            "max_start_delta_deg": 120.0,
        },
        "hand": {
            "angles_deg": {
                name: 0.0 for name in runner.INITIAL_HAND_JOINT_NAMES
            },
            "command_duration_s": 0.1,
        },
    }


def _write_config(tmp_path: Path, value: dict) -> Path:
    path = tmp_path / "initial_pose.yaml"
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    return path


def test_named_joint_config_builds_expected_plan(tmp_path: Path) -> None:
    raw = _valid_config()
    raw["arm"]["angles_deg"].update({"joint_1": 10.0, "joint_6": -20.0})
    raw["hand"]["angles_deg"].update(
        {"index_j1": 30.0, "middle_j1": -40.0, "little_j1": -50.0}
    )
    config = runner.load_teleop_initial_pose_config(
        _write_config(tmp_path, raw)
    )
    plan = runner.build_teleop_initial_pose_plan(config)

    assert config.arm.angles_deg.tolist() == [
        10.0, 0.0, 0.0, 0.0, 0.0, -20.0
    ]
    # F_cmd signs are applied exactly once for J4/J7/J9.
    assert plan.hand_sdk_user_deg[3] == pytest.approx(-30.0)
    assert plan.hand_sdk_user_deg[6] == pytest.approx(40.0)
    assert plan.hand_sdk_user_deg[8] == pytest.approx(50.0)


def test_unfilled_template_fails_before_hardware_is_opened(
    tmp_path: Path,
) -> None:
    raw = _valid_config()
    raw["hand"]["angles_deg"]["thumb_j1"] = None
    with pytest.raises(runner.TeleopInitialPoseError, match="thumb_j1"):
        runner.load_teleop_initial_pose_config(_write_config(tmp_path, raw))


def test_missing_or_unknown_joint_name_is_rejected(tmp_path: Path) -> None:
    raw = _valid_config()
    del raw["arm"]["angles_deg"]["joint_6"]
    raw["arm"]["angles_deg"]["joint_7"] = 0.0
    with pytest.raises(
        runner.TeleopInitialPoseError,
        match="missing.*joint_6.*unknown.*joint_7",
    ):
        runner.load_teleop_initial_pose_config(_write_config(tmp_path, raw))


@pytest.mark.parametrize(
    ("component", "joint", "value"),
    [
        ("arm", "joint_2", 131.0),
        ("hand", "thumb_j1", -0.1),
        ("hand", "middle_j1", 0.1),
    ],
)
def test_out_of_range_pose_is_rejected(
    tmp_path: Path,
    component: str,
    joint: str,
    value: float,
) -> None:
    raw = _valid_config()
    raw[component]["angles_deg"][joint] = value
    config = runner.load_teleop_initial_pose_config(
        _write_config(tmp_path, raw)
    )
    with pytest.raises(runner.TeleopInitialPoseError, match="invalid"):
        runner.build_teleop_initial_pose_plan(config)


def test_arm_start_delta_guard_reports_joint() -> None:
    config = runner.InitialArmPoseConfig(
        angles_deg=np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        speed_percent=5,
        timeout_s=30,
        tolerance_deg=1.0,
        max_start_delta_deg=90.0,
    )
    session = runner.RM65InitialPoseSession(
        "192.0.2.10", 8080, config
    )
    with pytest.raises(runner.TeleopInitialPoseError, match="joint_1"):
        session.validate_start_delta(np.zeros(6))


def test_parser_keeps_initial_pose_opt_in() -> None:
    parser = runner.build_arg_parser()
    ordinary = parser.parse_args(["--ip", "192.0.2.20"])
    assert ordinary.initial_pose_config is None
    assert ordinary.initial_pose_dry_run is False
    assert ordinary.initial_pose_auto_confirm is False

    configured = parser.parse_args([
        "--ip", "192.0.2.20",
        "--initial-pose-config", "pose.yaml",
        "--initial-pose-auto-confirm",
    ])
    assert configured.initial_pose_config == "pose.yaml"
    assert configured.initial_pose_auto_confirm is True


def test_main_initial_pose_dry_run_opens_no_hardware(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = _write_config(tmp_path, _valid_config())

    def forbidden(*args, **kwargs):
        raise AssertionError("initial-pose dry run must not open hardware")

    monkeypatch.setattr(runner, "RM65InitialPoseSession", forbidden)
    monkeypatch.setattr(runner, "initialize_sg100_initial_pose", forbidden)
    monkeypatch.setattr(
        runner.sys,
        "argv",
        [
            "run_avp_arm_hand_sim.py",
            "--ip",
            "192.0.2.20",
            "--initial-pose-config",
            str(path),
            "--initial-pose-dry-run",
        ],
    )
    assert runner.main() is None


def test_sg100_initial_target_observes_mode7_then_restores_passive() -> None:
    hardware_map = runner.SG100HardwareMap()
    ranges = [
        tuple(row[2:4])
        for row in np.asarray([
            [1.0, 0.0, 0.0, 125.0],
            [1.0, 0.0, -150.0, 0.0],
            [1.0, 0.0, -60.0, 90.0],
            [-1.0, 0.0, -90.0, 0.0],
            [1.0, 0.0, 0.0, 125.0],
            [1.0, 0.0, -60.0, 90.0],
            [1.0, 0.0, 0.0, 125.0],
            [1.0, 0.0, -60.0, 90.0],
            [-1.0, 0.0, 0.0, 180.0],
            [1.0, 0.0, 0.0, 125.0],
            [1.0, 0.0, -60.0, 90.0],
        ])
    ]

    class FakeDriver:
        def __init__(self):
            self.command_limit_contract = runner.SG100CommandLimitContract(
                hardware_map, ranges
            )
            self.sent = False
            self.passive = False
            self.sequence = 0

        @property
        def fault_active(self):
            return False

        def read_feedback(self):
            self.sequence += 1
            return SimpleNamespace(
                valid=True,
                sequence=self.sequence,
                received_monotonic_ns=runner.time.monotonic_ns(),
                joint_modes=(runner.MODE_JOINT_POSITION,) * runner.HAND_DOF,
            )

        def send_positions_rad(self, target):
            self.sent = True
            return SimpleNamespace(sent_monotonic_ns=runner.time.monotonic_ns())

        def hold_passive(self):
            self.passive = True
            return True

    driver = FakeDriver()
    config = runner.InitialHandPoseConfig(
        angles_deg=np.zeros(runner.HAND_DOF),
        command_duration_s=0.1,
    )
    effective = runner.initialize_sg100_initial_pose(driver, config)
    np.testing.assert_array_equal(effective, np.zeros(runner.HAND_DOF))
    assert driver.sent is True
    assert driver.passive is True
