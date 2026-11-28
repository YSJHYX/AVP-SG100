"""Return between episodes without disconnecting: offline faults and real API."""
from __future__ import annotations

import ast
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from sg100_teleop.control.return_home import ReturnHomeCycle
from sg100_teleop.control.teleop_arming import TeleopArmingController, TeleopArmState
from sg100_teleop.robot.rm65_real import RM65RealBackend
from sg100_teleop.tests.test_robot_backend_units import FakeSdkArm
from sg100_teleop.tests.test_collection_workbench import station, browser


class Hand:
    def __init__(self):
        self.targets = []
        self.stops = 0

    def send_positions_rad(self, target):
        self.targets.append(target)
        return object()

    def stop(self):
        self.stops += 1


def test_connected_return_uses_nonblocking_sdk_and_live_feedback():
    sdk = FakeSdkArm()
    arm = RM65RealBackend("fake", sdk=sdk, on_event=lambda *args: None)
    hand = Hand()
    arm.connect(); arm.start()
    sender, poller = arm._sender_thread, arm._feedback_thread
    cycle = ReturnHomeCycle(arm, hand, np.full(6, .1), np.zeros(11), stable_s=.04)
    try:
        cycle.start()
        assert sdk.movej_calls[-1][1:] == (25, 0, 0, 0)
        assert len(hand.targets) == 1
        assert not cycle.allows_arm and not cycle.allows_calibration
        with pytest.raises(RuntimeError):
            arm.set_enabled(True)
        # An old IK producer can keep publishing, but must never override
        # planned return motion, nor replay that stale target on completion.
        arm.send_joint_target(np.full(6, .2))
        deadline = time.monotonic() + 2
        while cycle.moving and time.monotonic() < deadline:
            cycle.poll(); time.sleep(.01)
        assert cycle.stage == "WAITING_NEUTRAL", cycle.message
        assert not sdk.canfd_calls
        assert arm._buffer.latest() is None
        assert arm._sender_thread is sender and arm._feedback_thread is poller
        assert sender.is_alive() and poller.is_alive() and arm._connected
        assert not sdk.deleted and not arm.enabled
    finally:
        arm.stop_smooth(); arm.disconnect()


@pytest.fixture
def cycle_clock(monkeypatch):
    clock = SimpleNamespace(now=1_000_000_000)
    monkeypatch.setattr("sg100_teleop.control.return_home.time.monotonic_ns", lambda: clock.now)
    class Arm:
        enabled = False
        stops = 0
        completed = 0
        state = SimpleNamespace(q_rad=np.zeros(6), qd_rad_s=np.zeros(6), sequence=1,
                                received_monotonic_ns=clock.now, valid=True, controller_error=0)
        def joint_state(self): return self.state
        def begin_runtime_return(self, target, **kwargs): pass
        def stop_smooth(self): self.stops += 1; return 0
        def complete_runtime_return(self, measured): self.completed += 1
    arm, hand = Arm(), Hand()
    cycle = ReturnHomeCycle(arm, hand, np.zeros(6), np.zeros(11), timeout_s=2, stable_s=.2)
    cycle.start()
    def update(dt=.1, **fields):
        clock.now += int(dt * 1e9)
        arm.state = SimpleNamespace(**{**vars(arm.state), "sequence":arm.state.sequence+1,
                                      "received_monotonic_ns":clock.now, **fields})
        return cycle.poll()
    return cycle, arm, hand, clock, update


def test_command_ack_alone_and_repeated_frame_never_confirm_arrival(cycle_clock):
    cycle, arm, _, clock, update = cycle_clock
    assert not cycle.poll()
    assert cycle.stage == "MOVING"
    update()
    # Reusing the same frame cannot count as stationary dwell.
    clock.now += 210_000_000
    assert not cycle.poll()
    assert cycle.stage == "MOVING"
    assert update(.01)
    assert cycle.stage == "WAITING_NEUTRAL"
    assert arm.completed == 1 and not cycle.allows_arm


def test_motion_and_missing_velocity_do_not_earn_arrival(cycle_clock):
    cycle, arm, _, _, update = cycle_clock
    for _ in range(4):
        assert not update(qd_rad_s=None)
    for _ in range(4):
        assert not update(qd_rad_s=np.full(6,.2))
    assert cycle.stage == "MOVING" and not arm.completed


@pytest.mark.parametrize("fault", ["stale", "controller", "timeout"])
def test_return_fault_stops_both_devices_and_blocks_rearm(cycle_clock, fault):
    cycle, arm, hand, clock, update = cycle_clock
    if fault == "stale":
        clock.now += 300_000_000; cycle.poll()
    elif fault == "controller":
        update(controller_error=4109)
    else:
        clock.now += 3_000_000_000; cycle.poll()
    assert cycle.stage == "FAILED"
    assert arm.stops == hand.stops == 1
    assert not cycle.allows_arm and not cycle.allows_calibration


def test_stop_interrupts_return_and_requires_a_fresh_anchor(cycle_clock):
    cycle, arm, hand, _, _ = cycle_clock
    cycle.cancel()
    assert cycle.stage == "CANCELLED" and not cycle.allows_arm
    assert arm.stops == hand.stops == 1
    cycle.anchor_requested()
    assert cycle.stage == "ANCHORING" and not cycle.allows_arm
    cycle.anchor_complete()
    assert cycle.stage == "READY" and cycle.allows_arm


def test_anchor_loss_and_cancel_never_auto_enable(cycle_clock):
    cycle, arm, _, _, _ = cycle_clock
    cycle.stage = "WAITING_NEUTRAL"
    cycle.anchor_requested(); cycle.anchor_lost()
    assert cycle.stage == "WAITING_NEUTRAL" and not cycle.allows_arm
    cycle.anchor_requested(); cycle.cancel()
    assert cycle.stage == "CANCELLED" and not arm.enabled


def test_missing_hand_feedback_stops_return_after_mode_switch_grace(cycle_clock):
    cycle, arm, hand, _, update = cycle_clock
    hand.read_feedback = lambda: None
    assert not update(.05)
    assert cycle.moving
    update(.16)
    assert cycle.stage == "FAILED"
    assert arm.stops == hand.stops == 1


def test_backend_rejected_return_remains_inhibited_until_stop():
    sdk = FakeSdkArm()
    sdk.rm_movej = lambda *args: -2
    arm = RM65RealBackend("fake", sdk=sdk, on_event=lambda *args: None)
    arm.connect()
    try:
        with pytest.raises(RuntimeError, match="rm_movej"):
            arm.begin_runtime_return(np.zeros(6))
        with pytest.raises(RuntimeError):
            arm.set_enabled(True)
        assert arm.stop_smooth() == 0
        arm.set_enabled(True)
    finally:
        arm.disconnect()


def test_inflight_sender_is_fenced_before_runtime_move():
    sdk = FakeSdkArm()
    arm = RM65RealBackend("fake", sdk=sdk, on_event=lambda *args: None)
    arm.connect()
    entered, release = threading.Event(), threading.Event()
    original = sdk.rm_movej_canfd
    def slow_send(*args):
        entered.set(); assert release.wait(2)
        return original(*args)
    sdk.rm_movej_canfd = slow_send
    arm.start(); arm.send_joint_target(np.full(6,.01)); arm.set_enabled(True)
    try:
        assert entered.wait(1)
        arm.set_enabled(False)
        done = threading.Event()
        def home():
            arm.begin_runtime_return(np.zeros(6)); done.set()
        thread = threading.Thread(target=home); thread.start()
        assert not done.wait(.02)
        release.set(); thread.join(1)
        assert done.is_set()
        count = len(sdk.canfd_calls)
        time.sleep(.04)
        assert len(sdk.canfd_calls) == count
        assert sdk.movej_calls[-1][-1] == 0
    finally:
        release.set(); arm.stop_smooth(); arm.disconnect()


def test_ui_requires_data_resolution_and_keeps_same_sources(browser, station):
    _, action = browser
    action("start"); action("calibrate"); action("arm")
    head, wrist = station.cameras["head"], station.cameras["right_wrist"]
    telemetry, controller = station.telemetry, station.controller
    action("begin", instruction="test")
    assert action("return_home").status_code == 400
    time.sleep(.1); action("finish")
    assert action("return_home").status_code == 400
    action("confirm", decision="n")
    result = action("return_home")
    assert result.status_code == 200
    assert result.json["control"]["return_home"]["stage"] == "WAITING_NEUTRAL"
    assert not result.json["control"]["arm_enabled"]
    assert not result.json["control"]["avp_anchored"]
    assert action("begin", instruction="test").status_code == 400
    assert station.controller is controller and station.telemetry is telemetry
    assert station.cameras["head"] is head and station.cameras["right_wrist"] is wrist
    assert controller.next_episode_index == 0
    assert action("calibrate").json["control"]["return_home"]["stage"] == "READY"
    assert not station.client.snapshot()["arm_enabled"]
    action("arm")
    assert action("begin", instruction="next").status_code == 200


def test_reset_cancels_pending_calibration_request():
    arming = TeleopArmingController()
    arming.observe_valid_frame(1); arming.request_neutral_calibration(10)
    arming.reset()
    assert not arming.should_calibrate(2,20) and not arming.active
    assert arming.state == TeleopArmState.WAITING_FOR_TRACKING
    arming.observe_valid_frame(2)
    assert arming.request_neutral_calibration(30)
    assert not arming.should_calibrate(2,20)
    assert arming.should_calibrate(3,40)


def test_stop_cancels_an_older_queued_return_request(tmp_path):
    from sg100_teleop.web_ui.channel import OperatorBridge
    bridge = OperatorBridge(tmp_path)
    now = time.monotonic_ns()
    home = {"id":"a"*32,"command":"return_home","arguments":{},"issued_ns":now,"expires_ns":now+1_000_000_000}
    bridge._normal.put(home)
    bridge._urgent.put(dict(home,id="b"*32,command="disarm"))
    calls = []
    bridge.drain(lambda command, arguments: calls.append(command))
    assert calls == ["disarm"]
    receipts = [bridge._receipts.get(),bridge._receipts.get()]
    assert receipts[1][1]["ok"] is False


def test_actual_runner_command_gate_requires_new_neutral_after_return():
    """Exercise the production nested command handlers without a viewer/AVP."""
    from sg100_teleop.sim.run_avp_arm_hand_sim import disarm_real_command_authority
    sdk = FakeSdkArm()
    arm = RM65RealBackend("fake", sdk=sdk, on_event=lambda *args: None)
    arm.connect(); arm.start(); arm.set_enabled(True)
    hand = Hand()
    class Restricted:
        enabled = True
        estopped = False
        def set_enabled(self, value): self.enabled = value
    restricted = Restricted()
    arming = TeleopArmingController()
    arming.observe_valid_frame(1); arming.request_neutral_calibration(); arming.complete_calibration()
    cycle = ReturnHomeCycle(arm, hand, np.full(6,.1), np.zeros(11), stable_s=.04)
    bridge = SimpleNamespace(recording=False, exit_requested=False, last_error="", update=lambda **kw: None)
    script = SimpleNamespace(enabled=False, motion=SimpleNamespace(value="hold"), exit=lambda: None)
    path = Path(__file__).resolve().parents[1] / "sim/run_avp_arm_hand_sim.py"
    tree = ast.parse(path.read_text())
    names = {"request_neutral_calibration", "cancel_return_home", "operator_status", "on_operator_command"}
    functions = [node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name in names]
    bindings = ast.parse("wrist_calibrated=True\nq_hand0_runtime=None\nlatest_control_audit=None\nlatest_ik_accepted=False").body
    factory = ast.FunctionDef(name="make", args=ast.arguments(posonlyargs=[], args=[], kwonlyargs=[], kw_defaults=[], defaults=[]),
                              body=bindings+functions+[ast.Return(value=ast.Name(id="on_operator_command", ctx=ast.Load()))], decorator_list=[])
    scope = {"time":time,"return_home":cycle,"operator_bridge":bridge,"teleop_arming":arming,
             "real_arm_backend":arm,"restricted_hand_backend":restricted,
             "tcp_target_filter":SimpleNamespace(clear=lambda: None),"script_hand_controller":script,
             "tracking_gate":SimpleNamespace(last_valid_tracking_monotonic_ns=time.monotonic_ns()),
             "args":SimpleNamespace(tracking_timeout=5),"grasp_mode_controller":SimpleNamespace(mode=SimpleNamespace(value="free")),
             "disarm_real_command_authority":disarm_real_command_authority,"print_teleop_state":lambda *args: None}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[factory], type_ignores=[])),str(path),"exec"),scope)
    command = scope["make"]()
    try:
        bridge.recording = True
        with pytest.raises(RuntimeError, match="当前数据"):
            command("return_home", {})
        assert not sdk.movej_calls
        bridge.recording = False
        command("return_home", {})
        assert not arming.active and not arm.enabled and not restricted.enabled
        with pytest.raises(RuntimeError): command("arm", {})
        with pytest.raises(RuntimeError): command("recording_lock", {"active":True})
        deadline = time.monotonic()+2
        while cycle.moving and time.monotonic()<deadline:
            cycle.poll(); time.sleep(.01)
        assert cycle.stage == "WAITING_NEUTRAL"
        arming.observe_valid_frame(2)
        command("calibrate", {})
        assert cycle.stage == "ANCHORING"
        with pytest.raises(RuntimeError): command("arm", {})
        assert arming.should_calibrate(3,time.monotonic_ns()+1)
        arming.complete_calibration(); cycle.anchor_complete()
        # Complete wrist_calibrated=True exactly as the production frame
        # handler does; access the real captured variable through its cell.
        cells = dict(zip(command.__code__.co_freevars,command.__closure__))
        cells["wrist_calibrated"].cell_contents = True
        command("arm", {})
        command("arm", {})
        assert arm.enabled and restricted.enabled
        assert not sdk.deleted and arm._connected
        command("disarm", {})
        assert not arm.enabled and not restricted.enabled
    finally:
        arm.stop_smooth(); arm.disconnect()
