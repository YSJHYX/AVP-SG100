#!/usr/bin/env python3
"""RM65 real-backend integration test WITHOUT AVP (v3 §B4 level 3).

Drives ``RM65RealBackend`` directly with a +-amplitude single-joint
sinusoid at the configured send rate, prints sender timing metrics
(mean / p95 / p99 / max period, deadline misses) and cmd-vs-actual
following error, then runs the stop-publish experiment: publishing
stops, the stale-target watchdog must fire the ordered stop sequence
on the real controller.  Finally re-arms and returns the joint to its
start pose.

Only the RM65 controller (power + ethernet) is required; no AVP, no
MuJoCo.  Run inside the ``mujoco-learn`` conda environment.

Safety: keep a 1 m clearance and the physical e-stop within reach.
Default motion is +-3 deg on joint 1 at PROFILE_10 (0.5 rad/s).

Usage::

    python scripts/rm65_backend_sinusoid.py --ip 192.168.1.18
    python scripts/rm65_backend_sinusoid.py --ip 192.168.1.18 --joint 2 \
        --amplitude-deg 3 --duration 10
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

sys.path.insert(0, ".")

from sg100_teleop.robot.rm65_real import RM65RealBackend  # noqa: E402

DEFAULT_IP = "192.168.1.18"
DEFAULT_PORT = 8080


def _stats_line(backend: RM65RealBackend) -> str:
    stats = backend.sender_stats
    if stats.get("count", 0) == 0:
        return "sender: no sends yet"
    return (
        f"sender: n={int(stats['count'])} "
        f"mean={stats['mean_s'] * 1e3:.2f}ms "
        f"p95={stats['p95_s'] * 1e3:.2f}ms "
        f"p99={stats['p99_s'] * 1e3:.2f}ms "
        f"max={stats['max_s'] * 1e3:.2f}ms "
        f"misses={int(stats['deadline_misses'])}"
    )


def _follow_error_deg(
    backend: RM65RealBackend, last_cmd_rad: np.ndarray | None
) -> float | None:
    if last_cmd_rad is None:
        return None
    state = backend.joint_state()
    if state is None:
        return None
    return float(
        np.degrees(np.max(np.abs(state.q_rad - last_cmd_rad)))
    )


def run(args: argparse.Namespace) -> int:
    joint_index = args.joint - 1
    if not 0 <= joint_index < 6:
        raise SystemExit("--joint must be 1..6")

    profiles = {10: (0.5, 2.0), 25: (1.25, 5.0), 50: (2.5, 10.0)}
    qdot_max, qddot_max = profiles[args.safety_profile]

    backend = RM65RealBackend(
        args.ip,
        args.port,
        send_hz=args.send_hz,
        follow=args.follow,
        qdot_max_rad_s=qdot_max,
        qddot_max_rad_s2=qddot_max,
        target_timeout_s=args.target_timeout_ms * 1e-3,
        on_event=lambda name, detail: print(f"[REAL-ARM] {name}: {detail}"),
    )
    backend.connect()
    backend.start()

    q0 = backend.joint_state().q_rad.copy()
    print(f"[TEST] start q (deg): {np.round(np.degrees(q0), 3).tolist()}")
    print(
        f"[TEST] joint {args.joint} will rock +-"
        f"{args.amplitude_deg} deg at {args.frequency_hz} Hz, "
        f"PROFILE_{args.safety_profile}, {args.send_hz} Hz sender"
    )
    print("[TEST] 1 m clearance + e-stop ready? starting in 2 s ...")
    time.sleep(2.0)

    failures: list[str] = []
    try:
        # ---------------------------------------------------------
        # Phase A: sinusoid streaming
        # ---------------------------------------------------------
        backend.set_enabled(True)
        last_cmd: np.ndarray | None = None
        last_target: np.ndarray | None = None
        started = time.monotonic()
        next_report = started + 1.0
        phase_a_end = started + args.duration

        while time.monotonic() < phase_a_end:
            t = time.monotonic() - started
            target = q0.copy()
            target[joint_index] += math.radians(args.amplitude_deg) * math.sin(
                2.0 * math.pi * args.frequency_hz * t
            )
            backend.send_joint_target(target)
            last_target = target
            state = backend.joint_state()
            if state is not None:
                last_cmd = target
            now = time.monotonic()
            if now >= next_report:
                error_deg = _follow_error_deg(backend, last_cmd)
                error_text = (
                    "n/a" if error_deg is None else f"{error_deg:.3f} deg"
                )
                print(f"[A] t={t:5.1f}s  {_stats_line(backend)}  "
                      f"follow_err={error_text}")
                next_report = now + 1.0
            time.sleep(0.02)

        state = backend.joint_state()
        moved_deg = (
            abs(math.degrees(state.q_rad[joint_index] - q0[joint_index]))
            if state is not None
            else 0.0
        )
        print(f"[A] phase done; |q - q0| = {moved_deg:.2f} deg")
        if not backend.enabled:
            failures.append("backend disarmed during sinusoid phase")
        if backend.sender_stats.get("count", 0) == 0:
            failures.append("no CANFD sends recorded")

        # ---------------------------------------------------------
        # Phase B: stop-publish experiment (stale-target watchdog)
        # ---------------------------------------------------------
        print(
            "[B] stopping publication; watchdog must fire the ordered "
            "stop sequence within ~"
            f"{int(args.target_timeout_ms)} ms + one sender tick"
        )
        disarmed_at = None
        watch_until = time.monotonic() + 3.0
        while time.monotonic() < watch_until:
            if not backend.enabled:
                disarmed_at = time.monotonic()
                break
            time.sleep(0.01)
        if disarmed_at is None:
            failures.append(
                "stale-target watchdog did NOT disarm the backend"
            )
        else:
            print("[B] backend disarmed by watchdog: PASS")

        # The decelerated stop plus the low-follow position loop may
        # take ~1 s to finish approaching the LAST commanded angle
        # (the stream stops ~2 deg ahead of the arm at these speeds).
        # Allow that settle tail, THEN require standstill.
        q_at_disarm = backend.joint_state().q_rad.copy()
        settle_until = time.monotonic() + 1.5
        while time.monotonic() < settle_until:
            time.sleep(0.05)
        q_hold_1 = backend.joint_state().q_rad.copy()
        time.sleep(1.0)
        q_hold_2 = backend.joint_state().q_rad.copy()

        settle_tail_deg = float(
            np.degrees(np.max(np.abs(q_hold_1 - q_at_disarm)))
        )
        drift_deg = float(
            np.degrees(np.max(np.abs(q_hold_2 - q_hold_1)))
        )
        print(
            f"[B] settle tail: {settle_tail_deg:.3f} deg; "
            f"1 s drift after settle: {drift_deg:.3f} deg"
        )
        if last_target is not None:
            residual_to_cmd = math.degrees(
                q_hold_2[joint_index] - last_target[joint_index]
            )
            print(
                "[B] final position vs last commanded target: "
                f"{residual_to_cmd:+.3f} deg"
            )
        if settle_tail_deg > 5.0:
            failures.append(
                f"runaway after stop (moved {settle_tail_deg:.2f} deg)"
            )
        if drift_deg > 0.2:
            failures.append(
                f"arm still moving after settle ({drift_deg:.2f} deg/s)"
            )

        # ---------------------------------------------------------
        # Phase C: re-arm and return to the start pose
        # ---------------------------------------------------------
        print("[C] re-arming and returning to the start pose")
        backend.set_enabled(True)
        return_end = time.monotonic() + 5.0
        while time.monotonic() < return_end:
            backend.send_joint_target(q0)
            time.sleep(0.02)
        state = backend.joint_state()
        if state is not None:
            residual = float(
                np.degrees(
                    np.max(np.abs(state.q_rad - q0))
                )
            )
            print(f"[C] return residual: {residual:.3f} deg")
            if residual > 0.2:
                failures.append(f"return residual {residual:.2f} deg")

    finally:
        backend.set_enabled(False)
        backend.stop_smooth()
        backend.disconnect()
        print("[EXIT] backend disconnected")

    print()
    if failures:
        print("SINUSOID TEST: FAIL")
        for item in failures:
            print(f"  - {item}")
        return 1
    print("SINUSOID TEST: PASS")
    return 0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default=DEFAULT_IP)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--joint", type=int, default=1, help="1-based, default 1")
    parser.add_argument("--amplitude-deg", type=float, default=3.0)
    parser.add_argument("--frequency-hz", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=10.0,
                        help="sinusoid phase length in seconds")
    parser.add_argument("--send-hz", type=float, default=100.0)
    parser.add_argument("--follow", choices=("low", "filter", "high"),
                        default="low")
    parser.add_argument("--safety-profile", type=int,
                        choices=(10, 25, 50), default=10)
    parser.add_argument("--target-timeout-ms", type=int, default=150)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if args.amplitude_deg <= 0 or args.amplitude_deg > 10:
        raise SystemExit("--amplitude-deg must be in (0, 10]")
    if args.duration <= 0:
        raise SystemExit("--duration must be > 0")
    try:
        code = run(args)
    except KeyboardInterrupt:
        print("\n[EXIT] interrupted")
        code = 130
    sys.exit(code)


if __name__ == "__main__":
    main()
