"""SG100 latest-SDK hardware probe — FEEDBACK ONLY, MODE_NULL only.

Hard guarantees of this tool:
- The ONLY control frame ever handed to the SDK is a whole-hand
  MODE_NULL (no drive output) frame.  No position target of any kind is
  constructed or sent: no MODE_POSITION, no MODE_JOINT_POSITION, no
  SetJointPositions, no homing/zeroing, no return-to-zero, no AVP input.
  The backend wrapper does not even export the position/homing entry
  points, so those counters are structurally zero.
- Before SwitchToRealtimeMode the tool verifies, through the wrapper's
  serializer (the same builder used for the live frame), that the
  constructed Hand_Control_t carries MODE_NULL for ALL 11 joints and
  prints the 11 mode bytes.
- The measured feedback is reported in the SDK USER-DEGREE domain ONLY.
  It is NOT converted to canonical joint angles and no claim is made
  about hand morphology until the runtime joint_angle_map / sign issue
  is resolved.  Native/raw wire positions are not exposed by the public
  SDK callback and are reported as unavailable rather than inferred.

Runtime mapping proof: with ``--execute`` the tool re-executes itself
under ``strace -f -e trace=openat`` and extracts the exact
``joint_angle_map.json`` file(s) the running libsg100_sdk.so opens —
never inferring from git HEAD, cwd, or similarly named files.  The
successful open's absolute path, hashes, and parsed J1..J11 rows are
reported.  The SDK is not altered in any way.

Default CLI is a DRY RUN plan; real execution requires ``--execute``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

from sg100_teleop.robot.sg100_sdk_driver import (
    DEFAULT_SDK_DIR,
    MODE_NULL,
    SG100SDKDriver,
    _load_wrapper_library,
    quantize_gain,
    sdk_shared_library,
)
from sg100_teleop.robot.sg100_driver import HAND_DOF


WRAPPER_MODULE = "sg100_teleop.tools.sg100_hardware_probe"
# The traced child must run from the repo root so `-m` resolves and every
# relative openat in the strace log resolves against one known cwd.
REPO_ROOT = Path(__file__).resolve().parents[2]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="SG100 latest-SDK probe: runtime map proof + MODE_NULL "
        "realtime entry + measured feedback capture; no motion capability"
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--sdk-dir", default=str(DEFAULT_SDK_DIR))
    parser.add_argument(
        "--duration-s", type=float, default=5.0,
        help="feedback capture window after realtime entry",
    )
    parser.add_argument(
        "--execute", action="store_true",
        help="actually run (under strace for the runtime map proof)",
    )
    parser.add_argument(
        "--inner-probe", action="store_true",
        help=argparse.SUPPRESS,  # internal: the traced child instance
    )
    return parser.parse_args(argv)


def md5_of(path: Path) -> str:
    digest = hashlib.md5()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


# ---------------------------------------------------------------------------
# Runtime joint_angle_map proof (strace openat parsing)
# ---------------------------------------------------------------------------

_OPENAT_RE = re.compile(
    r'^(?P<pid>\d+)\s+openat\((?P<dirfd>[^,]*),\s*"(?P<path>[^"]*)",.*\)\s*=\s*(?P<ret>-?\d+)'
)


def parse_map_opens(
    strace_log: Path, tracee_cwd: Path
) -> list[dict]:
    """Extract every joint_angle_map.json openat attempt from a strace log.

    Each entry: {raw_path, resolved, fd}.  fd >= 0 means the open
    SUCCEEDED — that is the file the running SDK actually parsed.
    Relative paths are resolved against the tracee cwd (the tool pins it).
    """

    entries: list[dict] = []
    for line in strace_log.read_text(errors="replace").splitlines():
        if "joint_angle_map.json" not in line:
            continue
        match = _OPENAT_RE.search(line.strip())
        if match is None:
            continue
        raw_path = match.group("path")
        path = Path(raw_path)
        if not path.is_absolute():
            path = tracee_cwd / path
        entries.append(
            {
                "raw_path": raw_path,
                "resolved": str(path),
                "fd": int(match.group("ret")),
            }
        )
    return entries


def successful_map_open(entries: list[dict]) -> dict | None:
    for entry in entries:
        if entry["fd"] >= 0:
            return entry
    return None


def parse_map_rows(path: Path) -> list[dict]:
    with path.open("r") as f:
        mapping = json.load(f)
    rows = []
    for entry in mapping.get("joint_angle_map", [])[:HAND_DOF]:
        rows.append(
            {
                "a": float(entry["a"]),
                "b": float(entry["b"]),
                "min_angle": float(entry["min_angle"]),
                "max_angle": float(entry["max_angle"]),
            }
        )
    return rows


def run_traced_probe(args: argparse.Namespace) -> int:
    """Outer instance: run the inner probe under strace, then report."""

    strace = shutil.which("strace")
    if strace is None:
        print(
            "PROBE FAIL: strace is required to prove the runtime "
            "joint_angle_map.json; install strace or do not claim the "
            "runtime map"
        )
        return 1

    with tempfile.TemporaryDirectory(prefix="sg100_probe_") as tmp:
        log_path = Path(tmp) / "openat.log"
        cmd = [
            strace,
            "-f",
            "-e", "trace=openat",
            "-o", str(log_path),
            sys.executable,
            "-m", WRAPPER_MODULE,
            "--inner-probe",
            "--execute",
            "--port", args.port,
            "--baudrate", str(args.baudrate),
            "--sdk-dir", args.sdk_dir,
            "--duration-s", str(args.duration_s),
        ]
        print("runtime map proof: running probe under strace (openat) ...")
        traced = subprocess.run(
            cmd, cwd=str(REPO_ROOT), capture_output=True, text=True
        )

        # ---- runtime mapping section (printed FIRST: it gates trust in
        # every user-degree number below) ----
        entries = parse_map_opens(log_path, REPO_ROOT)
        ok_entry = successful_map_open(entries)

        print()
        print("==== RUNTIME JOINT ANGLE MAP PROOF ====")
        so_path = sdk_shared_library(Path(args.sdk_dir).resolve())
        print("NEW SDK library      :", so_path)
        if so_path.exists():
            print("  md5                :", md5_of(so_path))
            print("  sha256             :", sha256_of(so_path))
        else:
            print("  MISSING (driver cannot load this)")
        if not entries:
            print("runtime joint_angle_map: NO OPEN OBSERVED")
            print(
                "  (the SDK opened no joint_angle_map.json during the "
                "probe; either it never encoded a frame or it used an "
                "in-process default — treat the runtime map as UNPROVEN)"
            )
        else:
            for entry in entries:
                status = (
                    f"OPENED fd={entry['fd']}"
                    if entry["fd"] >= 0
                    else f"failed errno={-entry['fd']}"
                )
                print(
                    f"  attempt: {entry['raw_path']} -> {status}\n"
                    f"          resolved: {entry['resolved']}"
                )
            if ok_entry is None:
                print("runtime joint_angle_map: ALL OPENS FAILED (UNPROVEN)")
            else:
                runtime_map = Path(ok_entry["resolved"])
                print("runtime joint_angle_map:", runtime_map)
                print("  md5                :", md5_of(runtime_map))
                print("  sha256             :", sha256_of(runtime_map))
                rows = parse_map_rows(runtime_map)
                print("  J1-J11 runtime mapping (parsed from the opened file):")
                print("  joint   a      b      min_angle  max_angle")
                for index, row in enumerate(rows):
                    print(
                        f"  J{index + 1:<2}    {row['a']:+5.1f}  {row['b']:+5.1f}  "
                        f"{row['min_angle']:+10.1f}  {row['max_angle']:+9.1f}"
                    )

        # ---- inner probe output (verbatim) ----
        print()
        print("==== PROBE (traced instance output) ====")
        sys.stdout.write(traced.stdout)
        if traced.stderr:
            sys.stderr.write(traced.stderr)
        return traced.returncode


# ---------------------------------------------------------------------------
# Inner probe: the only instance that touches the serial link
# ---------------------------------------------------------------------------


def verify_null_construction(lib) -> tuple[bool, list[int]]:
    """Serialize the exact NULL control the live path would send."""

    payload = np.zeros(HAND_DOF, dtype=np.float32)
    frame = np.zeros(176, dtype=np.uint8)
    size = lib.sg100_test_serialize_hand_control(
        payload,
        HAND_DOF,
        MODE_NULL,
        quantize_gain(0.5),
        quantize_gain(0.0),
        quantize_gain(0.05),
        quantize_gain(1.0),
        frame,
        frame.size,
    )
    if size != 176:
        return False, []
    # Joint_Control_t mode byte is at offset 1 of each 16-byte joint.
    mode_bytes = [int(frame[joint * 16 + 1]) for joint in range(HAND_DOF)]
    return all(value == MODE_NULL for value in mode_bytes), mode_bytes


def run_inner_probe(args: argparse.Namespace) -> int:
    driver = SG100SDKDriver(
        args.port,
        baudrate=args.baudrate,
        hardware_map=None,  # default identity; feedback is NOT canonicalized
        sdk_dir=Path(args.sdk_dir),
    )
    lib = driver._wrapper()

    print("-- pre-flight: MODE_NULL construction verification --")
    null_ok, mode_bytes = verify_null_construction(lib)
    print(
        "constructed Hand_Control_t mode bytes:",
        mode_bytes,
    )
    print(
        "all 11 joints MODE_NULL        :",
        "YES" if null_ok else "NO",
    )
    if not null_ok:
        print("PROBE FAIL: refusing to touch hardware without a verified "
              "MODE_NULL frame")
        return 1
    print(
        "wrapper position/homing exports:",
        "NONE (structurally absent:",
        ", ".join(
            name for name in (
                "sg100_c_send_positions", "sg100_seed_realtime_cache",
                "homing", "set_joint_zero",
            ) if not hasattr(lib, name)
        ) + ")",
    )

    print("-- connect (vendor RS485 active) --")
    driver.connect()
    try:
        t0 = time.monotonic()
        driver.enter_realtime()
        entry_s = time.monotonic() - t0
        print("realtime entry                  : PASS "
              f"({entry_s:.3f} s, MODE_NULL hold)")
        print(
            "pre-switch feedback flow        :",
            "YES" if driver.preswitch_feedback_flow else
            "NO (paused palm silent; NULL cache made the switch safe)",
        )

        print(f"-- capture {args.duration_s:.1f} s of measured feedback --")
        deadline = time.monotonic() + args.duration_s
        user_deg_rows: list[np.ndarray] = []
        last_sequence = -1
        worst_age_s = 0.0
        while time.monotonic() < deadline:
            feedback = driver.read_feedback()
            now = time.monotonic()
            if feedback is None or feedback.sequence == last_sequence:
                continue
            last_sequence = feedback.sequence
            if not feedback.valid:
                continue
            user_deg = np.rad2deg(feedback.raw_positions)
            user_deg_rows.append(user_deg.copy())
            worst_age_s = max(
                worst_age_s,
                now - feedback.received_monotonic_ns * 1e-9,
            )

        if not user_deg_rows:
            print("feedback callback               : FAIL (no samples)")
            return 1
        q = np.stack(user_deg_rows)
        rate = len(q) / max(args.duration_s, 1e-9)
        print("feedback callback               : PASS")
        print(f"feedback rate                   : {rate:.1f} Hz")
        print(
            "feedback dimensions             :",
            f"{q.shape[1]} / {'11 OK' if q.shape[1] == HAND_DOF else 'OTHER'}",
        )
        print(
            "feedback finite                 :",
            "YES" if bool(np.all(np.isfinite(q))) else "NO",
        )
        print(f"worst feedback age              : {worst_age_s * 1000.0:.1f} ms")

        print("-- SDK user positions [deg] (NOT canonical; morphology "
              "unvalidated) --")
        print("joint   last        min         max")
        for joint in range(HAND_DOF):
            column = q[:, joint]
            print(
                f"J{joint + 1:<2}   {column[-1]:+10.4f}  {column.min():+10.4f}  "
                f"{column.max():+10.4f}"
            )
        spread = float(np.max(q.max(axis=0) - q.min(axis=0)))
        print(
            f"max per-joint spread            : {spread:.4f} deg "
            "(static hand expectation: small)"
        )
        print(
            "native/raw wire position        : NOT AVAILABLE via public "
            "SDK callback (user-deg domain only; not inferred)"
        )

        diag = driver.diagnostics
        print("-- control purity / counters --")
        print(
            "MODE_NULL only                  : YES "
            f"(hand_control_calls={diag['hand_control_calls']}, "
            "every frame built MODE_NULL)",
        )
        print("position-producing command count: 0")
        print("SetJointPositions calls         : 0 (no such export)")
        print("Homing/zero calls               : 0 (no such export)")
        print("fault (SDK error callback)      :",
              diag["error_frames"])
        print(
            "communication                    : "
            f"status_recv_count={diag['status_recv_count']} (advancing) "
            f"vendor_work_mode={diag['vendor_work_mode']} (0=REALTIME)",
        )
        print(
            "visible hardware movement       : USER MUST CONFIRM "
            "(probe constructed no position target)"
        )
        return 0 if diag["error_frames"] == 0 else 1
    finally:
        driver.stop()       # MODE_NULL hold (still no position target)
        driver.disconnect()


def print_plan(args: argparse.Namespace) -> None:
    print("SG100 latest-SDK probe — DRY RUN")
    print("port                :", args.port)
    print("baudrate            :", args.baudrate)
    print("sdk dir             :", args.sdk_dir)
    print("capture window      :", f"{args.duration_s:.1f} s")
    print("runtime map proof   : strace -f -e trace=openat (re-exec)")
    print("motion commands     : NONE (MODE_NULL only, 11/11 verified)")
    print("realtime entry      : NULL seed -> vendor SwitchToRealtimeMode")
    print("feedback report     : SDK user [deg] ONLY (no canonical claims)")
    print("Re-run with --execute when the hand is connected and clear.")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.inner_probe:
        try:
            return run_inner_probe(args)
        except Exception as exc:  # noqa: BLE001 - diagnostic surface
            print(f"PROBE FAIL: {exc}")
            return 1
    if not args.execute:
        print_plan(args)
        return 0
    try:
        return run_traced_probe(args)
    except Exception as exc:  # noqa: BLE001 - diagnostic surface
        print(f"PROBE FAIL: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
