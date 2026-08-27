#!/usr/bin/env python3
"""RM65-B real-hardware smoke test over the RealMan SDK.

This is the phase-1 gate of the sim-to-real migration: it talks to the
RM65 controller directly, without any AVP tracking or MuJoCo in the
loop, and verifies connection, state feedback and one small planned
joint motion.

Run it inside the ``mujoco-learn`` conda environment, which is the only
interpreter on this machine that has the ``Robotic_Arm`` package.

Usage::

    # read-only: connection, controller info, joint angles, TCP pose
    python scripts/rm65_smoke.py --ip 192.168.1.18 state

    # small safe motion: rock one joint +/- 5 deg at 5% speed
    python scripts/rm65_smoke.py --ip 192.168.1.18 wiggle --joint 1

    # remote decelerated stop (network e-stop helper)
    python scripts/rm65_smoke.py --ip 192.168.1.18 stop
"""

from __future__ import annotations

import argparse
import sys
import time

from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
from Robotic_Arm.rm_robot_interface import RoboticArm


DEFAULT_IP = "192.168.1.18"
DEFAULT_PORT = 8080
RM65_DOF = 6

_STATUS_TEXT = {
    0: "success",
    1: "controller returned false (parameter error or arm state error)",
    -1: "send failed (communication problem)",
    -2: "receive failed (communication problem or controller timeout)",
    -3: "reply parsing failed",
    -4: "in-position device check failed",
    -5: "single-thread mode receive timeout",
}


def _describe(tag: int) -> str:
    return _STATUS_TEXT.get(tag, f"unknown status {tag}")


def _fail(action: str, tag: int) -> None:
    raise SystemExit(
        f"[FAIL] {action} returned {tag} ({_describe(tag)}); "
        "check IP/port, controller power and RealMan docs"
    )


def _connect(ip: str, port: int) -> RoboticArm:
    arm = RoboticArm(rm_thread_mode_e.RM_SINGLE_MODE_E)
    handle = arm.rm_create_robot_arm(ip, port)
    if handle is None or getattr(handle, "id", -1) == -1:
        arm.rm_destroy()
        raise SystemExit(
            f"[FAIL] could not create arm handle at {ip}:{port}; "
            "check that the controller is powered and reachable (ping)"
        )
    print(f"[CONN] {ip}:{port} arm_id={handle.id}")
    print(f"[CONN] arm_dof={arm.arm_dof} controller_version={arm.robot_controller_version}")
    if arm.arm_dof != RM65_DOF:
        print(
            f"[WARN] expected a 6-DoF RM65, controller reports "
            f"arm_dof={arm.arm_dof}; continuing but verify the model"
        )
    return arm


def _read_joints(arm: RoboticArm) -> list[float]:
    tag, joints = arm.rm_get_joint_degree()
    if tag != 0:
        _fail("rm_get_joint_degree", tag)
    print(f"[STATE] joint (deg): {[round(q, 3) for q in joints[:RM65_DOF]]}")
    return list(joints[:RM65_DOF])


def _read_arm_state(arm: RoboticArm) -> dict:
    tag, state = arm.rm_get_current_arm_state()
    if tag != 0:
        _fail("rm_get_current_arm_state", tag)
    # SDK 1.1.6 returns a flat dict: {"joint": [...],
    # "pose": [x, y, z, rx, ry, rz] (euler in deg), "err": {...}}.
    pose = state.get("pose", [])
    position = pose[:3] if len(pose) >= 3 else [float("nan")] * 3
    euler = pose[3:6] if len(pose) >= 6 else [float("nan")] * 3
    print(f"[STATE] TCP position (m): {[round(v, 4) for v in position]}")
    print(f"[STATE] TCP euler (deg, rx ry rz): {[round(v, 3) for v in euler]}")
    print(f"[STATE] controller error code: {state.get('err')}")
    return state


def run_state(arm: RoboticArm) -> int:
    """Read-only checks: version, joints, TCP pose."""
    _read_joints(arm)
    _read_arm_state(arm)
    print("[OK] state check complete (no motion commanded)")
    return 0


def run_stop(arm: RoboticArm) -> int:
    """Decelerated stop, for use as a network e-stop helper."""
    tag = arm.rm_set_arm_slow_stop()
    if tag != 0:
        _fail("rm_set_arm_slow_stop", tag)
    print("[OK] slow stop commanded")
    return 0


def run_wiggle(arm: RoboticArm, joint: int, amplitude_deg: float, speed_percent: int) -> int:
    """Rock one joint by +/- amplitude at low speed, then return.

    Uses the planned, blocking ``rm_movej`` on purpose: no CANFD
    streaming here.  This only verifies basic command round trips.
    """
    start = _read_joints(arm)
    index = joint - 1
    if not 0 <= index < RM65_DOF:
        raise SystemExit(f"[FAIL] --joint must be 1..{RM65_DOF}, got {joint}")

    for direction in (+1.0, -1.0):
        target = start.copy()
        target[index] += direction * amplitude_deg
        print(
            f"[MOVE] joint {joint} -> {target[index]:.3f} deg "
            f"({direction * amplitude_deg:+.1f} deg, {speed_percent}% speed)"
        )
        # block value is the timeout in seconds in single-thread mode.
        tag = arm.rm_movej(target, speed_percent, 0, 0, 10)
        if tag != 0:
            _fail("rm_movej", tag)
        time.sleep(0.2)
        reached = _read_joints(arm)
        error = abs(reached[index] - target[index])
        print(f"[MOVE] following error: {error:.3f} deg")
        if error > 1.0:
            print("[WARN] following error above 1 deg; check speed setting")

    print("[OK] wiggle complete; arm back at the start pose")
    return 0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default=DEFAULT_IP, help=f"controller IP, default {DEFAULT_IP}")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"controller port, default {DEFAULT_PORT}")
    parser.add_argument(
        "action",
        choices=("state", "wiggle", "stop"),
        default="state",
        nargs="?",
        help="state: read-only checks (default) | wiggle: small joint motion | stop: slow stop",
    )
    parser.add_argument("--joint", type=int, default=1, help="1-based joint index for wiggle, default 1")
    parser.add_argument(
        "--amplitude", type=float, default=5.0, help="wiggle amplitude in deg, default 5, max 10"
    )
    parser.add_argument(
        "--speed", type=int, default=5, help="wiggle speed percent 1-20, default 5"
    )
    args = parser.parse_args()
    if not 0.0 < args.amplitude <= 10.0:
        parser.error("--amplitude must be in (0, 10] deg")
    if not 1 <= args.speed <= 20:
        parser.error("--speed must be in [1, 20] percent")
    return args


def main() -> int:
    args = _parse_args()
    arm = _connect(args.ip, args.port)
    try:
        if args.action == "state":
            code = run_state(arm)
        elif args.action == "wiggle":
            code = run_wiggle(arm, args.joint, args.amplitude, args.speed)
        else:
            code = run_stop(arm)
    finally:
        tag = arm.rm_delete_robot_arm()
        print(f"[EXIT] rm_delete_robot_arm -> {tag} ({_describe(tag)})")
        arm.rm_destroy()
    return code


if __name__ == "__main__":
    sys.exit(main())
