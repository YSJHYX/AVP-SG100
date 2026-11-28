"""Offline integration: real RAW writer + demo sources; never opens hardware."""
from __future__ import annotations

import json
from pathlib import Path
import re
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from sg100_teleop.camera.realsense_camera import RealSenseCamera
from sg100_teleop.sim.run_avp_arm_hand_sim import disarm_real_command_authority, shutdown_real_devices
from sg100_teleop.web_ui.app import create_app
from sg100_teleop.web_ui.channel import ControlClient, OperatorBridge, write_json
from sg100_teleop.web_ui.runtime import StationConfig, Workstation


@pytest.fixture
def station(tmp_path):
    value = Workstation(tmp_path / "station.json", demo=True, settle_seconds=.1)
    value.config.output_dir = str(tmp_path / "data")
    yield value
    value.emergency_cleanup()


@pytest.fixture
def browser(station):
    app = create_app(station)
    app.testing = True
    client = app.test_client()
    page = client.get("/")
    token = re.search(rb'name="workbench-token" content="([^"]+)"', page.data).group(1).decode()
    def action(name, **arguments):
        return client.post("/api/action", json={"action": name, **arguments}, headers={"X-Workbench-Token": token})
    return client, action


def test_page_refresh_and_status_never_initialize_hardware(browser, station):
    client, _ = browser
    for _ in range(3):
        assert client.get("/").status_code == 200
        assert client.get("/api/status").json["recording_state"] == "STOPPED"
    assert station.controller is None and station.process is None


def test_commands_require_local_origin_and_page_token(browser):
    client, _ = browser
    assert client.post("/api/action", json={"action": "start"}).status_code == 403
    assert client.get("/", headers={"Host": "evil.example"}).status_code == 403
    assert client.post("/api/action", json={"action": "start"}, headers={"Origin": "https://evil.example"}).status_code == 403


def test_record_finish_replay_save_uses_existing_raw_contract(browser, station):
    client, action = browser
    assert action("start").status_code == 200
    assert action("begin", instruction="motor placement").status_code == 400
    assert action("calibrate").status_code == 200
    assert action("arm").status_code == 200
    assert action("mode", mode="two_finger").json["control"]["mode"] == "two_finger"
    assert action("begin", instruction="motor placement").status_code == 200
    assert action("calibrate").status_code == 400
    time.sleep(.12)
    ended = action("finish")
    assert ended.status_code == 200, ended.json
    assert ended.json["recording_state"] == "PENDING_CONFIRMATION"
    assert ended.json["control"]["arm_enabled"] is False
    assert ended.json["quality"]["quality"] == "PASS", ended.json["quality"]
    assert client.get("/api/media/pending/head/1.jpg").data[:2] == b"\xff\xd8"
    assert client.get("/api/media/pending/right_wrist/1.jpg").status_code == 200
    saved = action("confirm", decision="y", dagger=True)
    assert saved.status_code == 200, saved.json
    assert saved.json["saved_count"] == 1
    path = Path(saved.json["saved"][0]["path"])
    with np.load(path, allow_pickle=False) as data:
        assert data["language_instruction"].item() == "motor placement"
        assert data["robot_qcmd_17d_rad"].shape[1] == 17
        assert not data["training_ready"].item()
        assert data["camera_modalities"].item() == "rgb"
    metadata = json.loads(path.with_name(path.stem + "_media").joinpath("metadata.json").read_text())
    assert metadata == {"episode_tags": ["dagger"]}
    assert action("close").status_code == 200


def test_finish_order_waits_between_disarm_and_off(station, monkeypatch):
    station.start(); station.control("calibrate"); station.control("arm"); station.begin("test")
    events = []
    original_request, original_off = station.client.request, station.controller.turn_off
    def request(command, **arguments):
        events.append(command)
        return original_request(command, **arguments)
    def off():
        events.append("off")
        return original_off()
    def sleep(seconds):
        events.append(("settle", seconds))
    monkeypatch.setattr(station.client, "request", request)
    monkeypatch.setattr(station.controller, "turn_off", off)
    monkeypatch.setattr("sg100_teleop.web_ui.runtime.time.sleep", sleep)
    station.finish()
    assert events[:3] == ["disarm", ("settle", .1), "off"]


def test_stop_failure_still_ends_recording_and_reports_unconfirmed(station, monkeypatch):
    station.start(); station.control("calibrate"); station.control("arm"); station.begin("test")
    original = station.client.request
    def request(command, **arguments):
        if command == "disarm":
            raise OSError("simulated stop failure")
        return original(command, **arguments)
    monkeypatch.setattr(station.client, "request", request)
    with pytest.raises(RuntimeError, match="停止未确认"):
        station.finish()
    assert station.controller.state.value == "PENDING_CONFIRMATION"


def test_retry_discards_only_pending_and_adds_recovery_tag(station):
    station.start(); station.control("calibrate"); station.control("arm"); station.begin("task")
    old = station.controller.pending_dir
    with pytest.raises(RuntimeError, match="停止遥操"):
        station.restart()
    station.control("disarm")
    station.restart()
    assert not old.exists()
    assert station.controller.failure_recovery
    station.control("arm")
    time.sleep(.1)
    station.finish()
    station.confirm("y", dagger=True)
    assert station.saved[0]["tags"] == ["failure_recovery", "dagger"]
    path = Path(station.saved[0]["path"])
    assert json.loads(path.with_name(path.stem + "_media").joinpath("metadata.json").read_text())["episode_tags"] == ["failure_recovery", "dagger"]
    assert station.controller.next_episode_index == 1


def test_failed_quality_cannot_be_saved_in_ui(station):
    station.start(); station.control("calibrate"); station.control("arm"); station.begin("test")
    station.controller.turn_off()
    assert station.controller.validation_report.quality == "FAIL"
    with pytest.raises(RuntimeError, match="检查失败"):
        station.confirm("y")
    station.confirm("n")
    assert station.controller.next_episode_index == 0


def test_camera_staleness_blocks_on_and_ready(station):
    station.start()
    head = station.cameras["head"]
    old = head.frames[-1]
    from dataclasses import replace
    head.frames.clear()
    head.frames.append(replace(old, host_receive_monotonic_ns=time.monotonic_ns()-5_000_000_000))
    # Freeze the demo producer without changing the source readiness interface.
    station.telemetry._stop.set(); station.telemetry.thread.join(1)
    head.frames.clear(); head.frames.append(replace(old, host_receive_monotonic_ns=time.monotonic_ns()-5_000_000_000))
    assert station.snapshot()["cameras"]["head"]["ready"] is False
    with pytest.raises(RuntimeError):
        station.controller.turn_on()


def test_real_camera_ready_is_not_ever_seen_a_frame(station):
    from dataclasses import replace
    station.start()
    camera = RealSenseCamera(role="head", serial="offline", expected_model="RealSense D455", enable_depth=False)
    camera._ring.append(station.cameras["head"].frames[-1])
    assert camera.ready
    camera._ring[-1] = replace(camera._ring[-1], host_receive_monotonic_ns=time.monotonic_ns()-1_000_000_000)
    assert not camera.ready


def test_preview_is_jpeg_from_same_capture_source(browser, station):
    client, action = browser
    action("start")
    response = client.get("/stream/head", buffered=False)
    try:
        item = next(response.response)
        assert b"Content-Type: image/jpeg" in item and b"\xff\xd8" in item
        assert station.preview.cameras["head"] is station.controller.head_camera
    finally:
        response.close()


def test_reference_and_media_routes_do_not_expose_arbitrary_files(browser):
    client, action = browser
    action("start")
    assert action("reference").status_code == 200
    assert client.get("/api/reference.jpg").data.startswith(b"\xff\xd8")
    assert client.get("/api/media/unknown/head/0.jpg").status_code == 404
    assert client.get("/api/media/pending/evil/0.jpg").status_code == 404


def test_second_start_and_pending_close_are_rejected(browser):
    _, action = browser
    assert action("start").status_code == 200
    assert action("start").status_code == 400
    action("calibrate"); action("arm"); action("begin", instruction="task")
    assert action("close").status_code == 400


def test_hand_failure_does_not_skip_arm_disable_or_stop():
    calls = []
    def broken(value):
        calls.append("hand"); raise OSError("hand transport unavailable")
    hand = SimpleNamespace(enabled=True, set_enabled=broken)
    arm = SimpleNamespace(enabled=True, set_enabled=lambda v: calls.append("arm-disable"), stop_smooth=lambda: calls.append("arm-stop"))
    with pytest.raises(RuntimeError, match="hand transport"):
        disarm_real_command_authority(arm, hand)
    assert calls == ["hand", "arm-disable", "arm-stop"]


def test_arm_disable_failure_does_not_skip_stop():
    calls = []
    def broken(value):
        raise OSError("disable failed")
    arm = SimpleNamespace(enabled=True, set_enabled=broken, stop_smooth=lambda: calls.append("stop"))
    with pytest.raises(RuntimeError, match="disable failed"):
        disarm_real_command_authority(arm)
    assert calls == ["stop"]


def test_ipc_roundtrip_and_expiry(tmp_path):
    bridge = OperatorBridge(tmp_path); bridge.start()
    client = ControlClient(tmp_path)
    done = threading.Event()
    def pump():
        while not done.wait(.005):
            bridge.drain(lambda command, arguments: {"command": command, **arguments})
    thread = threading.Thread(target=pump); thread.start()
    try:
        assert client.request("mode", mode="pinch") == {"command": "mode", "mode": "pinch"}
    finally:
        done.set(); thread.join(); bridge.close()
    with pytest.raises(TimeoutError):
        client.request("arm", timeout=.02)
    assert not list(tmp_path.glob("request_*.json"))


def test_priority_stop_cancels_queued_arm_and_old_keyboard_a(tmp_path):
    bridge = OperatorBridge(tmp_path)
    now = time.monotonic_ns()
    arm = {"id":"a"*32,"command":"arm","arguments":{},"issued_ns":now,"expires_ns":now+2_000_000_000}
    stop = dict(arm, id="b"*32, command="disarm")
    bridge._normal.put(arm); bridge._urgent.put(stop); bridge.key(ord("A"))
    events = []
    bridge.drain(lambda command, arguments: events.append(command))
    bridge.drain_keys(events.append)
    assert events == ["disarm"]
    receipts = [bridge._receipts.get(), bridge._receipts.get()]
    assert receipts[1][1]["ok"] is False


def test_workstation_config_rejects_unknown_and_ambiguous_values():
    with pytest.raises(ValueError):
        StationConfig.parse({"origin_position": "false"})
    with pytest.raises(ValueError):
        StationConfig.parse({"shell_command": "untrusted"})


def test_priority_stop_available_while_other_request_is_busy(browser, station):
    _, action = browser
    action("start"); action("calibrate"); action("arm")
    with station.operation_lock:
        assert action("mode", mode="claw").status_code == 409
        response = action("disarm")
        assert response.status_code == 200
        assert response.json["control"]["arm_enabled"] is False


def test_cleanup_attempts_both_devices_and_disconnect_after_hand_sdk_error():
    calls = []
    def fail():
        calls.append("hand-stop"); raise OSError("hand failure")
    hand = SimpleNamespace(stop=fail, disconnect=lambda: calls.append("hand-disconnect"))
    arm = SimpleNamespace(set_enabled=lambda _: calls.append("arm-disable"),
                          stop_smooth=lambda: calls.append("arm-stop"),
                          disconnect=lambda: calls.append("arm-disconnect"))
    with pytest.raises(RuntimeError, match="hand failure"):
        shutdown_real_devices(arm, hand)
    assert calls == ["arm-disable", "arm-stop", "hand-stop", "hand-disconnect", "arm-disconnect"]


def test_parent_exit_requests_control_loop_shutdown(tmp_path, monkeypatch):
    bridge = OperatorBridge(tmp_path, parent_pid=-1)
    bridge.start()
    try:
        deadline = time.monotonic() + 1
        while not bridge.exit_requested and time.monotonic() < deadline:
            time.sleep(.005)
        assert bridge.exit_requested
    finally:
        bridge.close()
