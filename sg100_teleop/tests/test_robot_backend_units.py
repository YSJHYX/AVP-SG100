"""No-hardware unit verification of the robot backend layer (v3 §B2.2).

Run:  python -m sg100_teleop.tests.test_robot_backend_units
No RealMan SDK, MuJoCo model or hardware is required.
"""

from __future__ import annotations

import math
import time

import numpy as np

from sg100_teleop.robot.arm_state import ArmState, ArmTarget
from sg100_teleop.robot.ik_continuity_gate import (
    GateAction,
    IKContinuityGate,
)
from sg100_teleop.robot.joint_trajectory_safety import (
    JointTrajectorySafetyLimiter,
)
from sg100_teleop.robot.latest_target_buffer import LatestTargetBuffer
from sg100_teleop.robot.rm65_real import RM65RealBackend
from sg100_teleop.robot.safety_monitor import SafetyMonitor, StopSequence

LOWER = np.array([-3.106, -2.2689, -2.356, -3.106, -2.234, -6.28])
UPPER = -LOWER


class FakeSdkArm:
    """Records commands; feedback tracks the last command (deg)."""

    arm_dof = 6

    def __init__(self, initial_deg=(0.0,) * 6):
        self.initial = list(initial_deg)
        self.last_cmd_deg: list[float] | None = None
        self.canfd_calls: list[tuple] = []
        self.slow_stop_count = 0
        self.stop_count = 0
        self.deleted = False

    def _q_deg(self):
        return list(self.last_cmd_deg) if self.last_cmd_deg else list(self.initial)

    def rm_get_joint_degree(self):
        return 0, self._q_deg()

    def rm_get_current_arm_state(self):
        return 0, {
            "joint": self._q_deg(),
            "pose": [0.1, -0.05, 0.7, 0.0, 0.0, 0.0],
            "err": {"err_len": 1, "err": ["0"]},
        }

    def rm_movej_canfd(self, joint, follow, expand=0.0, trajectory_mode=0, radio=0):
        self.last_cmd_deg = list(joint)
        self.canfd_calls.append((list(joint), follow, trajectory_mode))
        return 0

    def rm_set_arm_slow_stop(self):
        self.slow_stop_count += 1
        return 0

    def rm_set_arm_stop(self):
        self.stop_count += 1
        return 0

    def rm_delete_robot_arm(self):
        self.deleted = True
        return 0


def make_state(q_rad, *, age_s=0.0, error=0, valid=True):
    return ArmState(
        q_rad=np.asarray(q_rad, dtype=np.float64),
        qd_rad_s=None,
        received_monotonic_ns=time.monotonic_ns() - int(age_s * 1e9),
        sequence=1,
        controller_error=error,
        arm_status=None,
        valid=valid,
    )


# ---------------------------------------------------------
# 1. rad<->deg bridge (single conversion site)
# ---------------------------------------------------------

def test_deg_bridge_command_and_feedback():
    fake = FakeSdkArm(initial_deg=[10.0, -20.0, 30.0, 5.0, -33.0, -235.0])
    backend = RM65RealBackend("fake", sdk=fake)

    q_rad = np.deg2rad([10.0, -20.0, 30.0, 5.0, -33.0, -235.0])
    backend._send_canfd(q_rad)
    sent_deg = np.asarray(fake.canfd_calls[-1][0])
    assert np.allclose(sent_deg, [10.0, -20.0, 30.0, 5.0, -33.0, -235.0]), sent_deg

    backend._poll_feedback_once()
    state = backend.joint_state()
    assert state is not None
    assert np.allclose(state.q_rad, q_rad, atol=1e-12)
    assert state.controller_error == 0

    # Round trip is exact to float precision.
    assert abs(float(np.rad2deg(np.deg2rad(0.123456789))) - 0.123456789) < 1e-9


# ---------------------------------------------------------
# 2. latest-target buffer semantics
# ---------------------------------------------------------

def test_latest_target_buffer():
    buffer = LatestTargetBuffer()
    now = time.monotonic_ns()
    assert buffer.latest() is None
    assert buffer.age_s(now) is None

    first = ArmTarget(np.zeros(6), now, sequence=1)
    second = ArmTarget(np.ones(6), now, sequence=2)
    assert buffer.publish(first) is True
    assert buffer.publish(second) is True
    assert buffer.latest().sequence == 2  # newest wins

    assert buffer.publish(ArmTarget(np.zeros(6), now, sequence=2)) is False
    assert buffer.publish(ArmTarget(np.zeros(6), now, sequence=1)) is False
    assert buffer.latest().sequence == 2  # stale writes rejected

    stale_age = buffer.age_s(now + int(0.3e9))
    assert stale_age is not None and 0.29 < stale_age < 0.31

    buffer.clear()
    assert buffer.latest() is None


# ---------------------------------------------------------
# 3. limiter bounds
# ---------------------------------------------------------

def test_limiter_bounds_on_step_and_sine():
    limiter = JointTrajectorySafetyLimiter(
        lower_rad=LOWER,
        upper_rad=UPPER,
        qdot_max_rad_s=0.5,
        qddot_max_rad_s2=2.0,
        soft_margin_rad=math.radians(2.0),
    )
    assert not limiter.seeded
    try:
        limiter.update(np.zeros(6), 0.01)
    except RuntimeError:
        pass
    else:
        raise AssertionError("update before reset must raise")

    limiter.reset(np.zeros(6))
    dt = 0.01
    commands = [limiter.update(np.ones(6) * 0.8, dt) for _ in range(400)]

    velocities = np.diff(np.asarray(commands), axis=0) / dt
    assert np.all(np.abs(velocities) <= 0.5 + 1e-9), np.abs(velocities).max()
    accelerations = np.diff(velocities, axis=0) / dt
    assert np.all(np.abs(accelerations) <= 2.0 + 1e-9), np.abs(accelerations).max()
    assert np.allclose(commands[-1], np.ones(6) * 0.8, atol=1e-3)  # converged

    # Sine target stays inside the velocity/acceleration envelopes.
    limiter.reset(np.zeros(6))
    for k in range(500):
        target = 0.3 * np.sin(2.0 * np.pi * 0.5 * k * dt) * np.ones(6)
        q = limiter.update(target, dt)
        if k > 0:
            assert np.all(np.abs(q - prev) / dt <= 0.5 + 1e-9)
        prev = q


def test_limiter_soft_limits_and_reset():
    limiter = JointTrajectorySafetyLimiter(
        lower_rad=LOWER,
        upper_rad=UPPER,
        qdot_max_rad_s=2.0,
        qddot_max_rad_s2=20.0,
        soft_margin_rad=0.05,
    )
    limiter.reset(np.zeros(6))
    soft_upper = UPPER - 0.05
    for _ in range(600):
        q = limiter.update(UPPER + 1.0, 0.01)  # target beyond the limit
        assert np.all(q <= soft_upper + 1e-12)
    assert np.allclose(q, soft_upper, atol=1e-6)

    limiter.reset(np.zeros(6))
    q = limiter.update(np.zeros(6), 0.01)
    assert np.allclose(q, 0.0)  # re-seeded, no residual velocity


# ---------------------------------------------------------
# 4. continuity gate (glitch-frame semantics)
# ---------------------------------------------------------

def test_gate_glitch_drop_and_hold():
    gate = IKContinuityGate(jump_threshold_rad=0.2, consecutive_for_hold=3)
    prev = np.zeros(6)

    ok = gate.process(prev + 0.05, prev)
    assert ok.action is GateAction.ACCEPT and ok.q_rad is not None

    glitch = gate.process(prev + 0.30, prev)
    assert glitch.action is GateAction.DROP_FRAME
    # A single glitch does not poison the chain: next good frame passes.
    again = gate.process(prev + 0.05, prev)
    assert again.action is GateAction.ACCEPT

    for _ in range(3):
        result = gate.process(prev + 0.30, prev)
    assert result.action is GateAction.HOLD_AND_RESEED
    assert gate.total_holds == 1

    gate.reset()
    nan_candidate = np.full(6, np.nan)
    assert gate.process(nan_candidate, prev).action is GateAction.DROP_FRAME


# ---------------------------------------------------------
# 5. stop sequence ordering and escalation
# ---------------------------------------------------------

def test_stop_sequence_order_and_escalation():
    calls: list[str] = []

    def slow_ok():
        calls.append("slow")
        return 0

    sequence = StopSequence(
        disable_sender=lambda: calls.append("disable"),
        drain_sender=lambda: calls.append("drain"),
        slow_stop=slow_ok,
        immediate_stop=lambda: (calls.append("immediate"), 0)[1],
        clear_buffer=lambda: calls.append("clear"),
        on_disarm=lambda reason: calls.append("disarm"),
    )
    sequence.execute("test")
    assert calls == ["disable", "drain", "slow", "clear", "disarm"], calls

    # Escalation: slow stop fails twice -> immediate stop.
    calls.clear()
    sequence = StopSequence(
        disable_sender=lambda: calls.append("disable"),
        drain_sender=lambda: calls.append("drain"),
        slow_stop=lambda: (calls.append("slow"), 1)[1],
        immediate_stop=lambda: (calls.append("immediate"), 0)[1],
        clear_buffer=lambda: calls.append("clear"),
        on_disarm=lambda reason: calls.append("disarm"),
    )
    sequence.execute("test")
    assert calls == ["disable", "drain", "slow", "slow", "immediate", "clear", "disarm"], calls

    # Retry succeeds on second attempt -> no immediate stop.
    calls.clear()
    attempts = {"n": 0}

    def slow_flaky():
        calls.append("slow")
        attempts["n"] += 1
        return 1 if attempts["n"] == 1 else 0

    sequence = StopSequence(
        disable_sender=lambda: calls.append("disable"),
        drain_sender=lambda: calls.append("drain"),
        slow_stop=slow_flaky,
        immediate_stop=lambda: (calls.append("immediate"), 0)[1],
        clear_buffer=lambda: calls.append("clear"),
        on_disarm=lambda reason: calls.append("disarm"),
    )
    sequence.execute("test")
    assert calls == ["disable", "drain", "slow", "slow", "clear", "disarm"], calls


# ---------------------------------------------------------
# 6. watchdog triggers
# ---------------------------------------------------------

def test_monitor_triggers():
    monitor = SafetyMonitor(
        target_timeout_s=0.15,
        feedback_timeout_s=0.2,
        gross_follow_error_rad=0.35,
    )
    now = time.monotonic_ns()
    fresh_target = ArmTarget(np.zeros(6), now, sequence=1)
    fresh_state = make_state(np.zeros(6))

    assert monitor.check(
        now_monotonic_ns=now, latest_target=None,
        feedback=fresh_state, last_cmd_rad=None,
    ).name == "target_missing"

    stale_target = ArmTarget(np.zeros(6), now - int(0.3e9), sequence=1)
    assert monitor.check(
        now_monotonic_ns=now, latest_target=stale_target,
        feedback=fresh_state, last_cmd_rad=None,
    ).name == "target_stale"

    assert monitor.check(
        now_monotonic_ns=now, latest_target=fresh_target,
        feedback=None, last_cmd_rad=None,
    ).name == "feedback_stale"

    old_state = make_state(np.zeros(6), age_s=1.0)
    assert monitor.check(
        now_monotonic_ns=now, latest_target=fresh_target,
        feedback=old_state, last_cmd_rad=None,
    ).name == "feedback_stale"

    err_state = make_state(np.zeros(6), error=3)
    assert monitor.check(
        now_monotonic_ns=now, latest_target=fresh_target,
        feedback=err_state, last_cmd_rad=None,
    ).name == "controller_error"

    far_cmd = np.zeros(6)
    far_cmd[0] = 0.4  # beyond the 0.35 rad gross bound
    assert monitor.check(
        now_monotonic_ns=now, latest_target=fresh_target,
        feedback=fresh_state, last_cmd_rad=far_cmd,
    ).name == "follow_error"

    near_cmd = np.zeros(6)
    near_cmd[0] = 0.01
    assert monitor.check(
        now_monotonic_ns=now, latest_target=fresh_target,
        feedback=fresh_state, last_cmd_rad=near_cmd,
    ) is None


# ---------------------------------------------------------
# 7. threaded sender integration with a fake controller
# ---------------------------------------------------------

def test_real_backend_streams_then_stops_on_stale_target():
    fake = FakeSdkArm(initial_deg=[0.0] * 6)
    backend = RM65RealBackend(
        "fake",
        sdk=fake,
        send_hz=200.0,
        target_timeout_s=0.06,
        feedback_timeout_s=1.0,
        on_event=lambda name, detail: None,
    )
    backend.connect()
    backend.start()
    backend.set_enabled(True)

    target = np.full(6, 0.05)
    # Phase 1: keep the target fresh (re-publish well inside the 60 ms
    # timeout) while the limiter converges toward it.  Braking-profile
    # convergence for 0.05 rad at qdot 0.5 / qddot 2 takes ~0.32 s.
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        backend.send_joint_target(target)
        time.sleep(0.02)

    assert backend._send_count > 0
    last_deg = np.asarray(fake.canfd_calls[-1][0])
    assert np.allclose(last_deg, np.full(6, math.degrees(0.05)), atol=0.05), last_deg
    assert fake.slow_stop_count == 0
    assert backend.enabled is True
    sent_before_stop = len(fake.canfd_calls)

    # Phase 2: stop publishing; the target ages out and the stop
    # sequence fires inside the sender thread.
    time.sleep(0.4)
    assert fake.slow_stop_count >= 1
    assert backend.enabled is False
    # Sends may legitimately continue while the target ages toward the
    # timeout; after disarm nothing may be sent anymore.
    stopped_after = len(fake.canfd_calls)
    assert stopped_after >= sent_before_stop
    time.sleep(0.1)
    assert len(fake.canfd_calls) == stopped_after  # no post-stop sends
    assert backend.joint_state() is not None  # feedback kept polling

    # Self-created SDK handles are released on disconnect; injected
    # ones stay owned by the caller.  Simulate ownership here so the
    # delete path is covered without touching real hardware.
    assert fake.deleted is False
    backend._owns_sdk = True
    backend.disconnect()
    assert fake.deleted is True
    assert backend._sender_thread is None and backend._feedback_thread is None


def test_monitor_velocity_dead_arm_detection():
    # Velocity-based liveness: a lagging arm is moving (healthy), a
    # dead arm is not.  Immune to any tracking-lag time constant.
    def make_monitor():
        return SafetyMonitor(
            target_timeout_s=0.15,
            feedback_timeout_s=0.2,
            velocity_fault_threshold_rad_s=0.1,
            velocity_fault_ratio=0.3,
            velocity_fault_window_s=0.3,
        )

    now = time.monotonic_ns()
    target = ArmTarget(np.zeros(6), now, sequence=1)
    state = make_state(np.zeros(6))
    v_cmd = np.zeros(6)
    v_cmd[0] = 0.5

    # Scenario 1 — dead arm: command moves, actual does not; trips
    # after the 0.3 s window, not before.
    monitor = make_monitor()
    tripped = None
    for _ in range(60):  # 0.6 s at 10 ms ticks
        tripped = monitor.check(
            now_monotonic_ns=now, latest_target=target, feedback=state,
            last_cmd_rad=None, commanded_velocity_rad_s=v_cmd,
            actual_velocity_rad_s=np.zeros(6), dt_s=0.01,
        )
        if tripped is not None:
            break
    assert tripped is not None and tripped.name == "arm_not_moving"

    monitor = make_monitor()
    for _ in range(25):  # 0.25 s: still inside the window
        assert monitor.check(
            now_monotonic_ns=now, latest_target=target, feedback=state,
            last_cmd_rad=None, commanded_velocity_rad_s=v_cmd,
            actual_velocity_rad_s=np.zeros(6), dt_s=0.01,
        ) is None

    # Scenario 2 — lagging but alive arm: actual at 60% of commanded
    # velocity never trips, however long it runs.
    monitor = make_monitor()
    v_actual = np.zeros(6)
    v_actual[0] = 0.3
    for _ in range(500):  # 5 s
        assert monitor.check(
            now_monotonic_ns=now, latest_target=target, feedback=state,
            last_cmd_rad=None, commanded_velocity_rad_s=v_cmd,
            actual_velocity_rad_s=v_actual, dt_s=0.01,
        ) is None

    # Scenario 3 — stop transient: command just stopped (v = 0) with a
    # residual position lag far below the gross bound; no trip.
    monitor = make_monitor()
    cmd = np.zeros(6)
    cmd[0] = 0.15
    assert monitor.check(
        now_monotonic_ns=now, latest_target=target, feedback=state,
        last_cmd_rad=cmd, commanded_velocity_rad_s=np.zeros(6),
        actual_velocity_rad_s=np.zeros(6), dt_s=0.01,
    ) is None


def main():
    tests = [
        test_deg_bridge_command_and_feedback,
        test_latest_target_buffer,
        test_limiter_bounds_on_step_and_sine,
        test_limiter_soft_limits_and_reset,
        test_gate_glitch_drop_and_hold,
        test_stop_sequence_order_and_escalation,
        test_monitor_triggers,
        test_monitor_velocity_dead_arm_detection,
        test_real_backend_streams_then_stops_on_stale_target,
    ]
    for test in tests:
        test()
    print("ROBOT BACKEND UNITS: PASS")


if __name__ == "__main__":
    main()
