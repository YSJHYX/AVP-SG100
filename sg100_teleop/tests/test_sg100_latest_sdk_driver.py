"""Latest-SDK SG100 backend contract tests (offline; no hardware).

Covers the migration acceptance list:
A. the new SDK library + wrapper load
B. vendor struct ABI matches the documented layout
C. canonical rad <-> SDK user-degree conversion + gain quantization
D. 11D joint order through the whole-hand control structure
E. SetHandControl receives the correct whole-hand structure
F. no legacy SetJointPositions production entry point is exposed
G. feedback -> canonical 11D conversion (mock round-trip)
H. startup state machine (fail-closed before realtime entry)
I. zero/nonzero decision semantics (reporting only)
N. cleanup / stop keeps the passive hold

C-gating (J/K), MuJoCo measured rendering (L) and action/state
distinction (M) are covered by the SDK-independent suites
(test_teleop_arming.py, run-avp hand path, test_arm_hand_data_contract_v1.py
and test_hand_data_contract_measured.py).
"""

from __future__ import annotations

import ctypes
import struct
from pathlib import Path

import numpy as np
import pytest

from sg100_teleop.robot import sg100_sdk_driver as sdk_driver_module
from sg100_teleop.robot.sg100_driver import (
    HAND_DOF,
    SG100HardwareMap,
    assess_zero_pose,
)
from sg100_teleop.robot.sg100_mock_driver import MockSG100Driver
from sg100_teleop.robot.sg100_sdk_driver import (
    DEFAULT_SDK_DIR,
    MODE_JOINT_POSITION,
    MODE_NULL,
    SG100SDKDriver,
    _load_wrapper_library,
    load_vendor_user_ranges,
    quantize_gain,
)


EXPECTED_JOINT_ORDER = [
    "thumb_j1", "thumb_j2", "thumb_j3",
    "index_j1", "index_j2", "index_j3",
    "middle_j1", "middle_j2",
    "little_j1", "little_j2", "little_j3",
]


@pytest.fixture(scope="module")
def wrapper_lib():
    """Build + load the wrapper against the installed SDK (offline)."""

    return _load_wrapper_library(sdk_dir=DEFAULT_SDK_DIR)


# -- A. library loads --------------------------------------------------------


def test_a_sdk_library_and_wrapper_load(wrapper_lib) -> None:
    assert hasattr(wrapper_lib, "sg100_connect")
    assert hasattr(wrapper_lib, "sg100_set_hand_control")
    assert hasattr(wrapper_lib, "sg100_switch_to_realtime_mode")
    assert hasattr(wrapper_lib, "sg100_poll_status")


def test_a_vendor_sdk_shared_object_exists() -> None:
    lib = Path(DEFAULT_SDK_DIR) / "lib" / "linux-x86_64" / "libsg100_sdk.so"
    assert lib.exists(), f"latest SDK library missing: {lib}"


# -- B. struct ABI -----------------------------------------------------------


def test_b_vendor_struct_sizes(wrapper_lib) -> None:
    sizes = np.zeros(6, dtype=np.int32)
    assert wrapper_lib.sg100_test_struct_sizes(sizes, 6) == 1
    # Joint_Control, Joint_Status, Hand_Control, Hand_Status, MotorDown,
    # MotorUp — documented in sg100_types.h @ V1.5_NO_TACTILE.
    assert sizes.tolist() == [16, 12, 176, 132, 14, 12]


# -- C. unit conversion + quantization --------------------------------------


def test_c_quantize_gain_matches_sg100_quant() -> None:
    # SG100_Quant::encode with range 10: round(v * 32767 / 10)
    assert quantize_gain(0.5) == int(round(0.5 * 32767 / 10.0))
    assert quantize_gain(0.0) == 0
    assert quantize_gain(-0.5) == (-int(round(0.5 * 32767 / 10.0))) & 0xFFFF
    assert quantize_gain(100.0) == quantize_gain(10.0)  # clamped


def test_c_canonical_user_degree_roundtrip() -> None:
    hardware_map = SG100HardwareMap()
    # Values inside every canonical mechanical range (sg100_joint_semantics).
    canonical = np.array(
        [1.0, -1.0, 0.5,
         0.5, 1.0, -0.5,
         -1.0, 0.5,
         -1.0, 1.0, -0.5]
    )
    user_rad = hardware_map.canonical_to_actual(canonical)
    back = hardware_map.actual_to_canonical(user_rad)
    np.testing.assert_allclose(back, canonical, atol=1e-12)
    # The driver converts user rad -> SDK degrees with a plain rad2deg.
    user_deg = np.rad2deg(user_rad)
    np.testing.assert_allclose(
        np.deg2rad(user_deg), user_rad, atol=1e-9
    )


# -- D/E. whole-hand structure through SetHandControl ------------------------


def _serialize_control(wrapper_lib, positions_deg, mode):
    payload = np.ascontiguousarray(positions_deg, dtype=np.float32)
    out = np.zeros(176, dtype=np.uint8)
    size = wrapper_lib.sg100_test_serialize_hand_control(
        payload,
        HAND_DOF,
        mode,
        quantize_gain(0.5),
        quantize_gain(0.0),
        quantize_gain(0.05),
        quantize_gain(1.0),
        out,
        out.size,
    )
    assert size == 176
    return out


def test_d_joint_order_inside_hand_control(wrapper_lib) -> None:
    hardware_map = SG100HardwareMap()
    assert hardware_map.software_joint_order == EXPECTED_JOINT_ORDER
    assert hardware_map.hardware_joint_ids == list(range(1, 12))

    positions_deg = np.arange(1, 12, dtype=np.float64) * 1.5
    frame = _serialize_control(wrapper_lib, positions_deg, MODE_JOINT_POSITION)

    # Joint_Control_t is packed: id[0], mode[1], position f32[2:6],
    # p/i/d/output_limit u16 at 6/8/10/12 — 16 bytes per joint.
    for joint in range(HAND_DOF):
        base = joint * 16
        assert frame[base] == joint + 1  # SDK joint id = software order
        assert frame[base + 1] == MODE_JOINT_POSITION
        (position,) = struct.unpack_from("<f", frame, base + 2)
        assert position == pytest.approx(
            np.float32(positions_deg[joint]), rel=1e-6
        )


def test_e_set_hand_control_structure(wrapper_lib) -> None:
    positions_deg = np.linspace(-5.0, 5.0, HAND_DOF)
    frame = _serialize_control(wrapper_lib, positions_deg, MODE_JOINT_POSITION)
    p, i, d, outlim = (
        quantize_gain(0.5),
        quantize_gain(0.0),
        quantize_gain(0.05),
        quantize_gain(1.0),
    )
    for joint in range(HAND_DOF):
        base = joint * 16
        (got_p, got_i, got_d, got_outlim) = struct.unpack_from(
            "<HHHH", frame, base + 6
        )
        assert (got_p, got_i, got_d, got_outlim) == (p, i, d, outlim)
        (position,) = struct.unpack_from("<f", frame, base + 2)
        assert position == pytest.approx(
            np.float32(positions_deg[joint]), rel=1e-6
        )


def test_e_null_control_structure(wrapper_lib) -> None:
    frame = _serialize_control(wrapper_lib, np.zeros(HAND_DOF), MODE_NULL)
    assert all(frame[j * 16 + 1] == MODE_NULL for j in range(HAND_DOF))


# -- F. no legacy production entry points ------------------------------------


def test_f_no_legacy_set_joint_positions_surface(wrapper_lib) -> None:
    # The old SetJointPositions/seed/raw-wire entry points must not exist.
    for legacy in (
        "sg100_c_send_positions",
        "sg100_seed_realtime_cache",
        "sg100_safe_switch_physical_palm_to_realtime",
        "sg100_start_safe_realtime_hold",
    ):
        assert not hasattr(wrapper_lib, legacy)


# -- G. vendor range guard ----------------------------------------------------


def test_g_load_vendor_user_ranges_from_disk() -> None:
    ranges = load_vendor_user_ranges(Path(DEFAULT_SDK_DIR))
    if ranges is not None:  # SDK ships the json
        assert len(ranges) == HAND_DOF
        for low, high in ranges:
            assert low <= high


def test_g_out_of_range_targets_are_rejected(tmp_path) -> None:
    sdk_dir = tmp_path / "sdk"
    sdk_dir.mkdir()
    (sdk_dir / "joint_angle_map.json").write_text(
        '{"joint_angle_map": ['
        + ",".join(
            f'{{"a": 1.0, "b": 0.0, "min_angle": 0.0, "max_angle": 90.0}}'
            for _ in range(HAND_DOF)
        )
        + "]}"
    )
    driver = SG100SDKDriver(
        "/dev/null", hardware_map=SG100HardwareMap(), sdk_dir=sdk_dir
    )
    assert driver.vendor_user_ranges is not None
    # In-range user degrees pass, negative (out-of-range) are rejected.
    assert driver._user_deg_within_vendor_range(np.full(HAND_DOF, 45.0))
    assert not driver._user_deg_within_vendor_range(np.full(HAND_DOF, -5.0))


# -- G/H/N. mock mirrors the real contract -----------------------------------


def test_g_mock_feedback_to_canonical_roundtrip() -> None:
    mock = MockSG100Driver(SG100HardwareMap())
    mock.connect()
    mock.start()
    mock.enter_realtime()
    actual = np.deg2rad(
        np.array([5.0, -6.0, 7.0, 8.0, -9.0, 10.0, 11.0, -12.0,
                  13.0, -14.0, 15.0])
    )
    mock.inject_actual_position(actual)
    feedback = mock.read_feedback()
    assert feedback is not None and feedback.valid
    np.testing.assert_allclose(feedback.q_rad, actual, atol=1e-9)
    assert feedback.q_rad.shape == (HAND_DOF,)


def test_h_startup_state_machine_fail_closed_before_realtime() -> None:
    mock = MockSG100Driver(SG100HardwareMap())
    mock.connect()
    mock.start()
    assert not mock.realtime_active
    # Before realtime entry the cached control is MODE_NULL: commands are
    # rejected (this is also the pre-C teleop gate at driver level).
    assert mock.send_positions_rad(np.zeros(HAND_DOF)) is None
    mock.enter_realtime()
    assert mock.realtime_active
    assert mock.send_positions_rad(np.zeros(HAND_DOF)) is not None


def test_h_enter_realtime_does_not_move_the_hand() -> None:
    mock = MockSG100Driver(
        SG100HardwareMap(),
        initial_q_rad=np.full(HAND_DOF, 0.3),
    )
    mock.connect()
    mock.start()
    mock.enter_realtime()
    feedback = mock.read_feedback()
    assert feedback is not None
    np.testing.assert_allclose(feedback.q_rad, np.full(HAND_DOF, 0.3), atol=1e-9)


def test_n_stop_keeps_passive_hold_and_disconnect_closes() -> None:
    mock = MockSG100Driver(SG100HardwareMap())
    mock.connect()
    mock.start()
    mock.enter_realtime()
    mock.send_positions_rad(np.zeros(HAND_DOF))
    mock.stop()  # MODE_NULL hold: freeze at current pose
    feedback = mock.read_feedback()
    assert feedback is not None
    frozen = feedback.q_rad.copy()
    for _ in range(5):
        time_feedback = mock.read_feedback()
        assert time_feedback is not None
        np.testing.assert_allclose(time_feedback.q_rad, frozen, atol=1e-9)
    mock.disconnect()
    assert not mock.realtime_active
    assert mock.read_feedback() is None


# -- I. zero decision semantics ----------------------------------------------


def test_i_zero_assessment_reports_without_commanding() -> None:
    at_zero = assess_zero_pose(np.zeros(HAND_DOF), tolerance_rad=0.05)
    assert at_zero.at_zero
    assert at_zero.max_abs_deviation_rad == 0.0

    off_zero = assess_zero_pose(
        np.array([0.0] * 10 + [0.2]), tolerance_rad=0.05
    )
    assert not off_zero.at_zero
    assert off_zero.max_abs_deviation_rad == pytest.approx(0.2)
    assert off_zero.per_joint_deviation_rad.shape == (HAND_DOF,)

    with pytest.raises(ValueError):
        assess_zero_pose(np.zeros(HAND_DOF + 1), tolerance_rad=0.05)


# -- driver construction defaults --------------------------------------------


def test_driver_defaults_use_vendor_teaching_gains() -> None:
    driver = SG100SDKDriver(
        "/dev/null", hardware_map=SG100HardwareMap(),
        sdk_dir=Path(DEFAULT_SDK_DIR),
    )
    assert driver.control_mode == MODE_JOINT_POSITION
    assert driver._pid_p == int(round(0.5 * 32767 / 10.0))
    assert driver._output_limit == int(round(1.0 * 32767 / 10.0))


# -- probe tool: runtime map proof + MODE_NULL purity ------------------------


def test_probe_parse_map_opens_from_strace_log(tmp_path) -> None:
    from sg100_teleop.tools.sg100_hardware_probe import (
        parse_map_opens,
        successful_map_open,
    )

    log = tmp_path / "openat.log"
    log.write_text(
        "101  openat(AT_FDCWD, \"/etc/ld.so.cache\", O_RDONLY|O_CLOEXEC) = 3\n"
        "101  openat(AT_FDCWD, \"config/joint_angle_map.json\", O_RDONLY) = -1 ENOENT (No such file or directory)\n"
        "102  openat(AT_FDCWD, \"/home/user/SG100_SDK_Publish/joint_angle_map.json\", O_RDONLY) = 9\n"
        "102  openat(3, \"unrelated.json\", O_RDONLY) = 10\n"
    )
    entries = parse_map_opens(log, tmp_path)
    assert len(entries) == 2
    assert entries[0]["fd"] == -1  # failed candidate first
    assert entries[0]["resolved"] == str(tmp_path / "config" / "joint_angle_map.json")
    ok = successful_map_open(entries)
    assert ok is not None
    assert ok["fd"] == 9
    assert ok["resolved"].endswith(
        "/SG100_SDK_Publish/joint_angle_map.json"
    )
    assert ok["resolved"].startswith("/")  # absolute, never cwd-inferred


def test_probe_parse_map_opens_empty_when_never_opened(tmp_path) -> None:
    from sg100_teleop.tools.sg100_hardware_probe import (
        parse_map_opens,
        successful_map_open,
    )

    log = tmp_path / "openat.log"
    log.write_text("101  openat(AT_FDCWD, \"/etc/passwd\", O_RDONLY) = 3\n")
    assert parse_map_opens(log, tmp_path) == []
    assert successful_map_open([]) is None


def test_probe_dry_run_states_mode_null_only() -> None:
    import subprocess
    import sys

    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "sg100_teleop.tools.sg100_hardware_probe",
        ],
        capture_output=True,
        text=True,
        cwd=str(Path(__file__).resolve().parents[2]),
    )
    assert result.returncode == 0
    assert "DRY RUN" in result.stdout
    assert "MODE_NULL" in result.stdout
    assert "strace" in result.stdout
    assert "canonical claims" in result.stdout


def test_probe_null_construction_verifies_11_mode_bytes(wrapper_lib) -> None:
    from sg100_teleop.tools.sg100_hardware_probe import (
        verify_null_construction,
    )

    ok, mode_bytes = verify_null_construction(wrapper_lib)
    assert ok
    assert mode_bytes == [MODE_NULL] * HAND_DOF
