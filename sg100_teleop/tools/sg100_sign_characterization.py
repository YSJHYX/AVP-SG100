"""SG100 N1 passive teaching-mode sign characterization tool.

Session contract (all HARD guarantees):
- The ONLY outgoing control is the whole-hand MODE_NULL frame (the same
  no-drive hold the realtime entry seeds).  No MODE_POSITION, no
  MODE_JOINT_POSITION, no MODE_IMPEDANCE, no position target, no
  SetJointPositions, no homing, no SetJointZero, no return-to-zero, no
  AVP input — those entry points do not exist in the backend wrapper.
- Before the serial link opens, the tool serializes the exact
  Hand_Control_t it would hand to the SDK and ABORTS unless
  mode[0:11] == MODE_NULL.
- The operator manually moves ONE physical DOF per trial in the
  canonical POSITIVE direction (instruction text comes from
  sg100_joint_semantics.yaml); the tool only records measured SDK
  callback-position feedback and analyzes dominant channel / direction /
  cross-coupling.  Callback magnitude is NOT interpreted as a physical
  joint angle or scale.
- Zero is NOT part of this test: no offsets are derived, no pose is
  redefined, no joint_zero_rad is touched.

Artifacts: recording/sg100_n1_sign_<timestamp>/{metadata.json,
samples.csv, summary.json}.

Offline modes: default DRY RUN plan; ``--selftest`` runs a scripted
session through the SAME analysis and artifact writer (no hardware).
Hardware execution is fail-closed while the callback feedback domain is
uncharacterized.  ``--execute`` currently refuses to open the transport.
"""

from __future__ import annotations

import argparse
import csv
import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from sg100_teleop.robot.sg100_driver import (
    HAND_DOF,
    SG100HardwareMap,
)
from sg100_teleop.robot.sg100_sdk_driver import (
    DEFAULT_SDK_DIR,
    MODE_NULL,
    SG100SDKDriver,
    quantize_gain,
)


ROOT = Path(__file__).resolve().parents[2]
SEMANTICS_PATH = (
    ROOT / "sg100_teleop" / "config" / "sg100_joint_semantics.yaml"
)

# Offline direction candidates from range morphology.  These are signs,
# not gains: no physical-to-callback scale has been characterized.
CANDIDATE_DIRECTION_SIGNS = [1] * HAND_DOF
CANDIDATE_DIRECTION_SIGNS[6] = -1  # middle_j1
# Compatibility name for callers created with the initial N1 tool.  Never
# describe these values as hardware-confirmed gains.
CANDIDATE_GAINS = CANDIDATE_DIRECTION_SIGNS

HARDWARE_EXECUTION_BLOCKED_REASON = (
    "feedback domain/physical scale is UNCHARACTERIZED after real N1; "
    "hardware execution is disabled pending vendor/firmware resolution"
)

# EXPLICIT per-joint test direction (canonical order).  Several joints
# start near canonical 0 with their usable motion toward the LOWER limit
# (thumb_j2 [-149.5,0], middle_j1 [-125,0], little_j1 [-180,0]), so a
# generic "move toward the upper limit" instruction is WRONG as a sign
# source.  Each trial therefore carries its own canonical delta sign and
# a human-readable physical instruction; expected callback direction is
# sign(candidate_direction * test_canonical_delta_sign).  Scale is separate.
TEST_CANONICAL_DELTA_SIGN = (
    +1,  # thumb_j1: opposition toward palm
    -1,  # thumb_j2: outward/downward from the zero pose
    +1,  # thumb_j3: flex toward palm
    +1,  # index_j1: toward thumb
    +1,  # index_j2: flex toward palm
    +1,  # index_j3: flex toward palm
    -1,  # middle_j1: flex toward palm (range is [-125,0])
    +1,  # middle_j2: flex toward palm
    -1,  # little_j1: outward/downward
    +1,  # little_j2: flex toward palm
    +1,  # little_j3: flex toward palm
)
TEST_PHYSICAL_INSTRUCTIONS = (
    "move toward palm / opposition",
    "move outward/downward (abduction away from the hand)",
    "flex toward palm",
    "move toward thumb (adduction)",
    "flex toward palm",
    "flex toward palm",
    "flex toward palm",
    "flex toward palm",
    "move outward/downward (outward rotation)",
    "flex toward palm",
    "flex toward palm",
)


def expected_sdk_delta_sign(
    direction_sign: float, test_delta_sign: float
) -> int:
    """Expected callback direction; intentionally contains no scale."""

    product = float(direction_sign) * float(test_delta_sign)
    if product == 0.0:
        raise ValueError("direction_sign and test_delta_sign must be non-zero")
    return 1 if product > 0 else -1


def sign_word(value: float) -> str:
    if value > 0:
        return "POSITIVE"
    if value < 0:
        return "NEGATIVE"
    return "ZERO"

# Modes that must NEVER appear in an outgoing control frame.
FORBIDDEN_POSITION_MODES = (1, 7, 8, 9)  # POSITION, JOINT_POSITION, IMPEDANCE, JOINT_IMPEDANCE


@dataclass(frozen=True)
class SessionConfig:
    baseline_s: float = 1.5
    return_s: float = 1.0
    dominance_ratio: float = 2.0
    # Signal/noise discriminator in the opaque SDK callback coordinate.
    # It is NOT a physical-degree threshold.
    min_signal_sdk_units: float = 3.0


# ---------------------------------------------------------------------------
# Pure analysis core (unit-tested; no I/O)
# ---------------------------------------------------------------------------


def per_channel_signed_delta(
    baseline_center: np.ndarray, move_samples: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Signed extreme deviation in opaque SDK callback units."""

    series = move_samples - baseline_center[np.newaxis, :]
    if series.size == 0:
        return np.zeros(HAND_DOF), np.zeros(HAND_DOF)
    peak_index = np.argmax(np.abs(series), axis=0)
    signed = series[peak_index, np.arange(HAND_DOF)]
    return signed, np.abs(signed)


def analyze_trial(
    name: str,
    expected_index: int,
    expected_direction_sign: float,
    baseline_center: np.ndarray,
    move_samples: np.ndarray,
    return_center: np.ndarray,
    config: SessionConfig,
    test_delta_sign: float = +1.0,
    direction_flipped: bool = False,
) -> dict:
    signed, absolute = per_channel_signed_delta(
        baseline_center, move_samples
    )
    order = np.argsort(absolute)[::-1]
    dominant = int(order[0])
    second = int(order[1]) if HAND_DOF > 1 else -1
    dominant_abs = float(absolute[dominant])
    second_abs = float(absolute[second]) if second >= 0 else 0.0
    ratio = (
        dominant_abs / second_abs
        if second_abs > 1e-9
        else float("inf")
    )

    if dominant_abs < config.min_signal_sdk_units:
        index_verdict = "INSUFFICIENT"
    elif dominant == expected_index:
        index_verdict = "SUPPORTED"
    elif ratio >= config.dominance_ratio:
        index_verdict = "NOT_SUPPORTED"
    else:
        index_verdict = "AMBIGUOUS"

    # Magnitude cannot classify physical scale.  The real J1 artifact proved
    # that a small physical motion can span ~1854 callback units.
    scale_verdict = "UNCHARACTERIZED"

    expected_signed = float(signed[expected_index])
    expected_sdk_sign = expected_sdk_delta_sign(
        expected_direction_sign, test_delta_sign
    )
    if (
        index_verdict == "SUPPORTED"
        and dominant_abs >= config.min_signal_sdk_units
    ):
        expected_sign_ok = (
            np.sign(expected_signed) == expected_sdk_sign
            if expected_signed != 0.0
            else False
        )
        direction_verdict = (
            "SUPPORTED" if expected_sign_ok else "NOT_SUPPORTED"
        )
    else:
        expected_sign_ok = False
        direction_verdict = "NOT_EVALUATED"

    return_error_sdk_units = float(
        abs(return_center[expected_index] - baseline_center[expected_index])
    )
    secondary_channels = [
        {
            "sdk_joint": f"J{int(channel) + 1}",
            "abs_delta_sdk_units": float(absolute[channel]),
        }
        for channel in order[1:4]
    ]

    return {
        "canonical_joint": name,
        "expected_sdk_joint": f"J{expected_index + 1}",
        "expected_index": expected_index,
        "candidate_direction_sign": int(np.sign(expected_direction_sign)),
        "test_canonical_delta_sign": int(test_delta_sign),
        "test_direction_flipped": bool(direction_flipped),
        "expected_sdk_delta_sign": (
            "POSITIVE" if expected_sdk_sign > 0 else "NEGATIVE"
        ),
        "observed_sdk_joint": f"J{dominant + 1}",
        "observed_index": dominant,
        "dominant_signed_delta_sdk_units": float(signed[dominant]),
        "dominant_abs_delta_sdk_units": dominant_abs,
        "expected_channel_signed_delta_sdk_units": expected_signed,
        "second_largest_abs_delta_sdk_units": second_abs,
        "second_largest_sdk_joint": f"J{second + 1}" if second >= 0 else None,
        "dominance_ratio": ratio,
        "secondary_channels": secondary_channels,
        "return_error_sdk_units": return_error_sdk_units,
        "return_verdict": "MANUAL_CONFIRMATION_REQUIRED",
        "return_ok": None,
        "index_verdict": index_verdict,
        "direction_verdict": direction_verdict,
        "scale_verdict": scale_verdict,
        "baseline_sdk_callback": float(baseline_center[expected_index]),
        "peak_sdk_callback": float(
            baseline_center[expected_index] + expected_signed
        ),
        "sdk_delta": expected_signed,
    }


def summarize(
    results: list[dict],
    *,
    joint_order: list[str] | None = None,
    skipped_joints: list[dict] | None = None,
    evidence_source: str = "HARDWARE",
    session_status: str = "PARTIAL",
) -> dict:
    """Build an explicit 11-joint hardware state without state leakage."""

    names = list(
        joint_order
        if joint_order is not None
        else SG100HardwareMap().software_joint_order
    )
    by_name = {r["canonical_joint"]: r for r in results}
    skipped_names = {
        item["canonical_joint"] for item in (skipped_joints or [])
    }
    hardware_evidence = evidence_source == "HARDWARE"
    per_joint: list[dict] = []
    hardware_results: list[dict] = []

    for name in names:
        result = by_name.get(name)
        if result is not None and hardware_evidence:
            supported = (
                result["index_verdict"] == "SUPPORTED"
                and result["direction_verdict"] == "SUPPORTED"
            )
            row = {
                **result,
                "hardware_status": (
                    "PARTIALLY_CHARACTERIZED"
                    if supported
                    else "MEASURED_UNRESOLVED"
                ),
                "session_disposition": "COMPLETED_TRIAL",
            }
            hardware_results.append(result)
        else:
            row = {
                "canonical_joint": name,
                "hardware_status": "NOT_TESTED",
                "session_disposition": (
                    "SKIPPED_UNVERIFIED"
                    if name in skipped_names
                    else "UNFINISHED"
                ),
            }
            if result is not None:
                row["offline_selftest_result"] = result
        per_joint.append(row)

    measured_names = {r["canonical_joint"] for r in hardware_results}
    all_index = (
        len(hardware_results) == HAND_DOF
        and all(r["index_verdict"] == "SUPPORTED" for r in hardware_results)
    )
    all_direction = (
        len(hardware_results) == HAND_DOF
        and all(
            r["direction_verdict"] == "SUPPORTED"
            for r in hardware_results
        )
    )
    return {
        "session_status": session_status,
        "evidence_source": evidence_source,
        "per_joint": per_joint,
        "hardware_tested_joint_count": len(hardware_results),
        "hardware_not_tested_joint_count": HAND_DOF - len(hardware_results),
        "all_11_index_supported": bool(all_index),
        "all_11_direction_supported": bool(all_direction),
        "physical_to_sdk_scale": "UNCHARACTERIZED",
        "candidate_direction_status": {
            name: (
                "HARDWARE_SUPPORTED"
                if name in measured_names
                and by_name[name]["direction_verdict"] == "SUPPORTED"
                else "NOT_HARDWARE_CONFIRMED"
            )
            for name in names
        },
        # Direction alone does not prove a physical canonical mapping.
        "canonical_mapping_verdict": "BLOCKED",
        "skipped_joints": list(skipped_joints or []),
    }


# ---------------------------------------------------------------------------
# Test-direction semantics
# ---------------------------------------------------------------------------


def load_semantics_ranges_deg() -> list[tuple[float, float]]:
    """Canonical model ranges, used only as operator-facing context."""

    with SEMANTICS_PATH.open("r") as f:
        semantics = yaml.safe_load(f)
    ranges = []
    for name in semantics["joint_order"]:
        lo, hi = semantics["joints"][name]["range"]
        ranges.append((float(np.degrees(lo)), float(np.degrees(hi))))
    return ranges


def print_trial_semantics(
    name: str,
    index: int,
    expected_index: int,
    test_delta_sign: int,
    direction_sign: float,
    flipped: bool,
    canonical_range_deg: tuple[float, float],
) -> None:
    print(f"  canonical joint          : {name}")
    print(f"  canonical range (deg)    : [{canonical_range_deg[0]:.0f}, "
          f"{canonical_range_deg[1]:.0f}]")
    print(f"  physical instruction     : "
          f"{TEST_PHYSICAL_INSTRUCTIONS[index]}")
    print(f"  test canonical delta     : "
          f"{sign_word(test_delta_sign)}"
          + ("  (OPERATOR FLIPPED)" if flipped else ""))
    print(f"  candidate direction sign : {direction_sign:+.0f}")
    print(f"  expected SDK channel     : J{expected_index + 1}")
    print(f"  expected SDK delta       : "
          f"{sign_word(expected_sdk_delta_sign(direction_sign, test_delta_sign))}")


# ---------------------------------------------------------------------------
# NULL-purity pre-flight
# ---------------------------------------------------------------------------


def serialize_null_frame(lib) -> np.ndarray:
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
        raise RuntimeError("cannot serialize NULL control frame")
    return frame


def verify_null_frame(lib) -> tuple[bool, list[int]]:
    frame = serialize_null_frame(lib)
    modes = [int(frame[joint * 16 + 1]) for joint in range(HAND_DOF)]
    return all(mode == MODE_NULL for mode in modes), modes


# ---------------------------------------------------------------------------
# Feedback recorder
# ---------------------------------------------------------------------------


class FeedbackRecorder:
    """Background sampler; rows carry the live phase label.

    Each row also carries a trial key ("<joint>:<attempt>") so retries of
    the same joint never mix baseline samples across attempts.  The CSV
    artifact keeps only (phase, active_requested_joint) per the schema.
    """

    def __init__(self, read_feedback):
        self._read_feedback = read_feedback
        self._lock = threading.Lock()
        self._rows: list[tuple] = []  # (ts, phase, trial_key, joint, deg11)
        self._phase = "idle"
        self._active_joint: int | None = None
        self._trial_key = ""
        self._running = False
        self._thread: threading.Thread | None = None
        self._last_sequence: int | None = None

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        while self._running:
            feedback = self._read_feedback()
            if feedback is not None and feedback.valid:
                if feedback.sequence == self._last_sequence:
                    time.sleep(0.002)
                    continue
                self._last_sequence = feedback.sequence
                # Undo the driver's storage conversion and retain the exact
                # SDK callback number.  The suffix in the historical CSV
                # schema says "deg", but this is not a physical joint angle.
                callback_position = np.rad2deg(feedback.raw_positions)
                with self._lock:
                    self._rows.append(
                        (
                            int(feedback.received_monotonic_ns),
                            self._phase,
                            self._trial_key,
                            self._active_joint,
                            *callback_position.tolist(),
                        )
                    )
            time.sleep(0.002)

    def set_phase(
        self,
        phase: str,
        active_joint: int | None = None,
        trial_key: str = "",
    ) -> None:
        with self._lock:
            self._phase = phase
            self._active_joint = active_joint
            self._trial_key = trial_key

    def phase_samples(
        self, phase: str, trial_key: str | None = None
    ) -> np.ndarray:
        with self._lock:
            rows = [
                r for r in self._rows
                if r[1] == phase
                and (trial_key is None or r[2] == trial_key)
            ]
        if not rows:
            return np.zeros((0, HAND_DOF))
        return np.asarray([row[4:] for row in rows], dtype=np.float64)

    def all_rows(self) -> list[tuple]:
        """CSV-shaped rows: (ts, phase, active_requested_joint, deg11)."""

        with self._lock:
            return [(r[0], r[1], r[3], *r[4:]) for r in self._rows]


# ---------------------------------------------------------------------------
# Artifacts
# ---------------------------------------------------------------------------


def _sdk_fingerprint(sdk_dir: Path) -> dict:
    import hashlib

    def digest(path: Path, algo: str) -> str:
        d = hashlib.new(algo)
        with path.open("rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                d.update(chunk)
        return d.hexdigest()

    import subprocess

    try:
        commit = subprocess.run(
            ["git", "-C", str(sdk_dir), "rev-parse", "HEAD"],
            capture_output=True, text=True, check=False,
        ).stdout.strip()
    except OSError:
        commit = ""
    lib = sdk_dir / "lib" / "linux-x86_64" / "libsg100_sdk.so"
    mapping = sdk_dir / "joint_angle_map.json"
    return {
        "sdk_dir": str(sdk_dir),
        "sdk_commit": commit or "UNKNOWN",
        "lib_path": str(lib),
        "lib_md5": digest(lib, "md5") if lib.exists() else "MISSING",
        "lib_sha256": digest(lib, "sha256") if lib.exists() else "MISSING",
        "joint_angle_map_path": str(mapping),
        "joint_angle_map_md5": (
            digest(mapping, "md5") if mapping.exists() else "MISSING"
        ),
        "joint_angle_map_sha256": (
            digest(mapping, "sha256") if mapping.exists() else "MISSING"
        ),
        "joint_angle_map_proof": (
            "runtime identity strace-proven by N0; re-hashed at session "
            "start"
        ),
    }


def write_artifacts(
    out_dir: Path,
    metadata: dict,
    rows: list[tuple],
    summary: dict,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n"
    )
    with (out_dir / "samples.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["timestamp_ns", "phase", "active_requested_joint"]
            + [
                f"j{index + 1}_sdk_callback_position"
                for index in range(HAND_DOF)
            ]
        )
        writer.writerows(rows)
    (out_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    )


def print_summary_table(summary: dict) -> None:
    print()
    print(
        f"{'canonical_joint':12s} {'hardware_status':24s} "
        f"{'index':14s} {'direction':14s} {'scale'}"
    )
    for row in summary["per_joint"]:
        print(
            f"{row['canonical_joint']:12s} "
            f"{row['hardware_status']:24s} "
            f"{row.get('index_verdict', '-'):14s} "
            f"{row.get('direction_verdict', '-'):14s} "
            f"{row.get('scale_verdict', '-')}"
        )
    print()
    print("HARDWARE TESTED JOINTS          :",
          summary["hardware_tested_joint_count"])
    print("HARDWARE NOT TESTED JOINTS      :",
          summary["hardware_not_tested_joint_count"])
    print("PHYSICAL -> SDK SCALE           :",
          summary["physical_to_sdk_scale"])
    print("CANONICAL MAPPING               :",
          summary["canonical_mapping_verdict"])


# ---------------------------------------------------------------------------
# Sessions
# ---------------------------------------------------------------------------


def _read_input(text: str, input_fn=input) -> str | None:
    """Return None for EOF or an injected None instead of crashing."""

    try:
        value = input_fn(text)
    except EOFError:
        return None
    return value if isinstance(value, str) else None


def prompt_command(
    text: str,
    allowed: tuple[str, ...],
    *,
    input_fn=input,
) -> str:
    """Read one explicit command; EOF/None always means safe quit."""

    while True:
        value = _read_input(text, input_fn)
        if value is None:
            return "q"
        command = value.strip().lower()
        if command in allowed:
            return command
        print("  invalid command; choose " + "/".join(allowed))


def finalize_partial_session(
    *, driver, recorder: FeedbackRecorder, out_dir: Path, metadata: dict,
    results: list[dict], joint_order: list[str], skipped_joints: list[dict],
    session_status: str,
) -> dict:
    """Save partial state and close the SDK on every exit path."""

    recorder.stop()
    summary = summarize(
        results,
        joint_order=joint_order,
        skipped_joints=skipped_joints,
        evidence_source="HARDWARE",
        session_status=session_status,
    )
    try:
        write_artifacts(out_dir, metadata, recorder.all_rows(), summary)
    finally:
        try:
            driver.stop()
        finally:
            driver.disconnect()
    return summary


def run_hardware_session(args: argparse.Namespace) -> int:
    # This guard is deliberately before wrapper loading, serial discovery,
    # driver construction, or any transport call.
    del args
    print("HARDWARE EXECUTION BLOCKED:", HARDWARE_EXECUTION_BLOCKED_REASON)
    print("No SDK was loaded and no transport was opened.")
    return 2


def run_selftest(out_root: str) -> int:
    """Scripted session through the same analysis/artifact code (offline)."""

    hardware_map = SG100HardwareMap()
    config = SessionConfig()
    rng = np.random.default_rng(20260903)
    rows: list[tuple] = []
    results: list[dict] = []
    base_ts = time.monotonic_ns()

    for index, name in enumerate(hardware_map.software_joint_order):
        expected = hardware_map.hardware_joint_ids[index] - 1
        direction_sign = CANDIDATE_DIRECTION_SIGNS[index]
        test_sign = TEST_CANONICAL_DELTA_SIGN[index]
        baseline = rng.normal(0.0, 0.02, HAND_DOF)
        # Opaque callback-coordinate delta.  Magnitude is synthetic and
        # deliberately has no physical-degree meaning.
        amplitude = direction_sign * test_sign * 22.0
        for step in range(20):
            phase = "baseline"
            rows.append(
                (base_ts + step * 10_000_000, phase, index,
                 *(baseline + rng.normal(0, 0.01, HAND_DOF)))
            )
        move = np.tile(baseline, (30, 1)) + rng.normal(0, 0.05, (30, HAND_DOF))
        move[:, expected] += np.linspace(0.05, amplitude, 30)
        coupling = np.zeros(HAND_DOF)
        coupling[(expected + 1) % HAND_DOF] = 0.02 * abs(amplitude)
        move += coupling
        for step in range(30):
            rows.append(
                (base_ts + (20 + step) * 10_000_000, "manual_move", index,
                 *move[step])
            )
        returned = baseline + rng.normal(0, 0.1, HAND_DOF)
        for step in range(15):
            rows.append(
                (base_ts + (50 + step) * 10_000_000, "return", index,
                 *(returned + rng.normal(0, 0.02, HAND_DOF)))
            )
        results.append(
            analyze_trial(
                name, expected, direction_sign,
                baseline, move, returned, config,
                test_delta_sign=test_sign,
            )
        )

    assert all(r["index_verdict"] == "SUPPORTED" for r in results)
    assert all(r["direction_verdict"] == "SUPPORTED" for r in results)
    summary = summarize(
        results,
        joint_order=hardware_map.software_joint_order,
        evidence_source="SELFTEST",
        session_status="SELFTEST_COMPLETE",
    )
    out_dir = (
        ROOT / out_root
        / f"sg100_n1_sign_selftest_{time.strftime('%Y%m%d_%H%M%S')}"
    )
    metadata = {
        "tool": "sg100_sign_characterization",
        "mode": "selftest",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "session_contract": "MODE_NULL x 11 only; scripted offline data",
        **_sdk_fingerprint(Path(DEFAULT_SDK_DIR)),
        "joint_order": hardware_map.software_joint_order,
        "candidate_direction_signs": CANDIDATE_DIRECTION_SIGNS,
        "physical_to_sdk_scale": "UNCHARACTERIZED",
        "test_canonical_delta_sign": list(TEST_CANONICAL_DELTA_SIGN),
        "test_physical_instructions": list(TEST_PHYSICAL_INSTRUCTIONS),
        "config": {"source": "scripted"},
    }
    write_artifacts(out_dir, metadata, rows, summary)
    print("selftest artifacts:", out_dir)
    print_summary_table(summary)
    assert summary["hardware_tested_joint_count"] == 0
    assert all(
        row["hardware_status"] == "NOT_TESTED"
        for row in summary["per_joint"]
    )
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="SG100 N1 passive sign characterization "
        "(MODE_NULL only; operator moves the hand)"
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--sdk-dir", default=str(DEFAULT_SDK_DIR))
    parser.add_argument("--baseline-s", type=float, default=1.5)
    parser.add_argument("--dominance-ratio", type=float, default=2.0)
    parser.add_argument("--out-root", default="recording")
    parser.add_argument("--selftest", action="store_true",
                        help="offline scripted session (no hardware)")
    parser.add_argument("--execute", action="store_true",
                        help="refuses safely while feedback is uncharacterized")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.selftest:
        return run_selftest(args.out_root)
    if not args.execute:
        print("SG100 N1 sign characterization — DRY RUN")
        print("port                 :", args.port)
        print("sdk dir              :", args.sdk_dir)
        print("hardware execution   : BLOCKED")
        print("reason               :", HARDWARE_EXECUTION_BLOCKED_REASON)
        print("position commands    : NONE (structurally impossible)")
        print("candidate directions : offline only; no physical scale")
        print("artifacts            :", args.out_root
              + "/sg100_n1_sign_<timestamp>/")
        print("Offline scripted run : --selftest")
        print("--execute            : fail-closed; opens no transport")
        return 0
    try:
        return run_hardware_session(args)
    except Exception as exc:  # noqa: BLE001 - diagnostic surface
        print(f"SESSION FAIL: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
