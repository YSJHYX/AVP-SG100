"""v3/R command-chain regressions; no serial ports or hardware commands."""

import copy
import json
import struct
from types import SimpleNamespace

import numpy as np
import pytest

from sg100_teleop.robot.sg100_driver import HAND_DOF
from sg100_teleop.robot.sg100_restricted_avp_backend import SG100RestrictedAVPBackend
from sg100_teleop.robot.sg100_sdk_driver import (
    DEFAULT_SDK_DIR,
    SG100SDKDriver,
    SG100SDKDriverError,
    VENDOR_ANGLE_MAP_FILENAME,
    _load_wrapper_library,
    load_vendor_angle_map,
    load_vendor_user_ranges,
    quantize_gain,
)


@pytest.fixture
def v3_document():
    ranges = [(0, 125), (-150, 0), (-60, 90), (-90, 0), (0, 125),
              (-60, 90), (0, 125), (-60, 90), (0, 180), (0, 125), (-60, 90)]
    scales = [1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1]
    right = [dict(a=a, b=0, min_angle=lo, max_angle=hi)
             for a, (lo, hi) in zip(scales, ranges)]
    left = [dict(entry, a=-entry["a"]) for entry in right]
    return {"version": "3.0", "joint_angle_map": {"R": right, "L": left}}


def write_map(directory, document):
    (directory / VENDOR_ANGLE_MAP_FILENAME).write_text(json.dumps(document))


@pytest.fixture(scope="module")
def native():
    # Loading/serializing the bridge does NOT connect/init/start a device.
    return _load_wrapper_library(sdk_dir=DEFAULT_SDK_DIR)


def test_v3_right_map_wins_over_conflicting_legacy(tmp_path, v3_document):
    write_map(tmp_path, v3_document)
    (tmp_path / "joint_angle_map.json").write_text("not even valid JSON")
    result = load_vendor_angle_map(tmp_path)
    np.testing.assert_array_equal(result[3], [-1, 0, -90, 0])
    np.testing.assert_array_equal(result[8], [-1, 0, 0, 180])
    ranges = load_vendor_user_ranges(tmp_path)
    assert ranges[1] == (-150, 0)  # thumb_j2 also uses v3, not old [-80,60]
    assert ranges[3] == (-90, 0)
    assert ranges[8] == (0, 180)


def test_missing_v3_never_falls_back_to_legacy(tmp_path, v3_document):
    (tmp_path / "joint_angle_map.json").write_text(json.dumps({
        "joint_angle_map": v3_document["joint_angle_map"]["R"],
    }))
    assert load_vendor_user_ranges(tmp_path) is None
    with pytest.raises(SG100SDKDriverError, match="refusing legacy"):
        SG100SDKDriver("DO_NOT_OPEN", sdk_dir=tmp_path)


def test_probe_understands_v3_right_map_and_open_priority(tmp_path, v3_document):
    from sg100_teleop.tools.sg100_hardware_probe import (
        parse_map_opens, parse_map_rows, successful_map_open,
    )

    write_map(tmp_path, v3_document)
    log = tmp_path / "openat.log"
    log.write_text(
        '1 openat(AT_FDCWD, "joint_angle_map.json", O_RDONLY) = 3\n'
        '1 openat(AT_FDCWD, "joint_angle_map_v3.0.json", O_RDONLY) = 4\n'
    )
    opened = parse_map_opens(log, tmp_path)
    assert len(opened) == 2
    assert successful_map_open(opened)["resolved"] == str(tmp_path / VENDOR_ANGLE_MAP_FILENAME)
    rows = parse_map_rows(tmp_path / VENDOR_ANGLE_MAP_FILENAME)
    assert len(rows) == HAND_DOF
    assert rows[3] == v3_document["joint_angle_map"]["R"][3]


@pytest.mark.parametrize("failure", [
    "version", "missing_left", "short_right", "extra_left", "zero_scale",
    "reversed_range", "nan", "overflow", "boolean", "string", "missing_field",
])
def test_invalid_v3_fails_closed(tmp_path, v3_document, failure):
    document = copy.deepcopy(v3_document)
    maps = document["joint_angle_map"]
    joint = maps["L"][3]  # even the unselected hand must pass SDK validation
    if failure == "version":
        document["version"] = "2.0"
    elif failure == "missing_left":
        del maps["L"]
    elif failure == "short_right":
        maps["R"].pop()
    elif failure == "extra_left":
        maps["L"].append(joint.copy())
    elif failure == "zero_scale":
        joint["a"] = 0
    elif failure == "reversed_range":
        joint["min_angle"] = 1
    elif failure == "nan":
        joint["b"] = float("nan")
    elif failure == "overflow":
        joint["b"] = 1e100
    elif failure == "boolean":
        joint["a"] = True
    elif failure == "string":
        joint["a"] = "1"
    else:
        del joint["a"]
    write_map(tmp_path, document)
    (tmp_path / "joint_angle_map.json").write_text("{}")
    with pytest.raises(ValueError, match=VENDOR_ANGLE_MAP_FILENAME):
        load_vendor_angle_map(tmp_path)


def test_python_matches_installed_sdk_runtime_map_without_connecting(native):
    before = native.sg100_hand_control_calls()
    actual = np.zeros((HAND_DOF, 4), dtype=np.float64)
    assert native.sg100_load_angle_map_v3(
        str(DEFAULT_SDK_DIR).encode(), actual, HAND_DOF
    ) == 1
    np.testing.assert_array_equal(actual, load_vendor_angle_map(DEFAULT_SDK_DIR))
    np.testing.assert_array_equal(actual[3], [-1, 0, -90, 0])
    np.testing.assert_array_equal(actual[8], [-1, 0, 0, 180])
    assert native.sg100_hand_control_calls() == before


def test_native_loader_rejects_legacy_only_without_connecting(native, tmp_path):
    (tmp_path / "joint_angle_map.json").write_text("{}")
    output = np.zeros((HAND_DOF, 4), dtype=np.float64)
    assert native.sg100_load_angle_map_v3(str(tmp_path).encode(), output, HAND_DOF) == 0


def test_changed_map_prevents_connection(native, tmp_path, v3_document):
    write_map(tmp_path, v3_document)
    driver = SG100SDKDriver("DO_NOT_OPEN", sdk_dir=tmp_path)
    v3_document["joint_angle_map"]["R"][3]["min_angle"] = -80
    write_map(tmp_path, v3_document)
    calls = []
    driver._lib = SimpleNamespace(
        sg100_load_angle_map_v3=native.sg100_load_angle_map_v3,
        sg100_connect=lambda *args: calls.append(args) or 0,
    )
    with pytest.raises(SG100SDKDriverError, match="disagrees with Python"):
        driver.connect()
    assert calls == []
    assert not driver.is_connected


def test_matching_map_is_validated_before_mock_connection(native, tmp_path, v3_document):
    write_map(tmp_path, v3_document)
    driver = SG100SDKDriver("MOCK_ONLY", sdk_dir=tmp_path)
    calls = []

    def read_map(*args):
        calls.append("validate")
        return native.sg100_load_angle_map_v3(*args)

    driver._lib = SimpleNamespace(
        sg100_load_angle_map_v3=read_map,
        sg100_connect=lambda *args: calls.append("mock_connect") or 0,
    )
    driver.connect()
    driver.connect()
    assert calls == ["validate", "mock_connect"]


@pytest.mark.parametrize("index_deg,little_deg", [
    (0, 0), (30, -60), (89.95292537973766, -90.17459213229169), (90, -180),
])
def test_sides_survive_restricted_backend_driver_and_sdk_mapping(
    native, monkeypatch, index_deg, little_deg,
):
    frames = []

    def capture(positions, modes, ps, ins, ds, limits, count):
        frame = np.zeros(176, dtype=np.uint8)
        assert native.sg100_test_serialize_hand_control_per_joint(
            positions, modes, ps, ins, ds, limits, count, frame, frame.size
        ) == 176
        frames.append(frame.tobytes())
        return 1

    class OfflineTransport:
        sg100_set_hand_control = staticmethod(capture)
        sg100_is_connected = staticmethod(lambda: 1)

        def __getattr__(self, name):
            # All remaining calls are transport diagnostics; no device exists.
            assert name.startswith("sg100_")
            return lambda *args: 0

    driver = SG100SDKDriver("MOCK_ONLY")
    driver._lib = OfflineTransport()
    driver._connected = driver._realtime = True
    monkeypatch.setattr(driver, "read_feedback", lambda: SimpleNamespace(
        valid=True, received_monotonic_ns=1_000_000_000, joint_modes=(7,) * HAND_DOF,
    ))
    backend = SG100RestrictedAVPBackend(
        driver, joint_allowlist=("index_j1", "little_j1"),
        max_velocity_rad_s=100, tracking_spike_rad=100,
    )
    assert backend.hardware_map is driver.hardware_map
    canonical_neutral = driver.hardware_map.clamp_canonical(
        np.zeros(HAND_DOF)
    )
    assert backend.anchor(canonical_neutral).accepted
    backend.set_enabled(True)
    canonical = canonical_neutral.copy()
    requested = np.deg2rad([index_deg, little_deg])
    canonical[[3, 8]] = np.clip(
        requested,
        driver.hardware_map.lower_rad[[3, 8]],
        driver.hardware_map.upper_rad[[3, 8]],
    )
    original = canonical.copy()
    result = backend.submit(
        canonical, tracking_received_ns=1_000_000_000,
        now_ns=1_000_000_000, dt_s=1,
    )
    assert result.accepted
    np.testing.assert_array_equal(canonical, original)
    np.testing.assert_allclose(result.effective_canonical_qcmd_rad, canonical)
    np.testing.assert_allclose(
        result.sdk_target_deg[[3, 8]],
        -np.rad2deg(canonical[[3, 8]]),
    )
    assert not backend.drain_safety_events()

    parameters = load_vendor_angle_map(DEFAULT_SDK_DIR)
    expected_sdk_deg = np.rad2deg(
        driver.hardware_map.gain * canonical
        + driver.hardware_map.offset_rad
    )
    for j in range(HAND_DOF):
        joint_id, mode, user_deg, p, i, d, limit = struct.unpack_from(
            "<BBfHHHH", frames[-1], j * 16
        )
        assert joint_id == j + 1 and mode == 7
        a, b, lo, hi = parameters[j]
        assert lo <= user_deg <= hi
        if j in (3, 8):
            assert (p, i, d, limit) == (quantize_gain(0.1), 0, 0, quantize_gain(0.5))
            # Vendor formula after its own user-domain clamp. Previously both
            # sides became zero here despite nonzero serialized targets.
            wire_rad = np.deg2rad(a * np.clip(user_deg, lo, hi) + b)
            assert wire_rad == pytest.approx(canonical[j], abs=1e-6)
        else:
            assert user_deg == pytest.approx(expected_sdk_deg[j], abs=1e-6)
            assert (p, i, d, limit) == (0, 0, 0, 0)


def test_real_feedback_mapping_is_not_implicitly_validated():
    driver = SG100SDKDriver("DO_NOT_OPEN")
    assert driver.feedback_map is None
    assert driver._mode7_hardware_validated is False
    assert driver.feedback_mapping_status == "F_FB_NOT_PROVIDED"
