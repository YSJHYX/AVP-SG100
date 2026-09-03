"""Real SG100 driver over the LATEST vendor SDK native RS485 path.

Backend contract (SG100_LATEST_SDK_INTEGRATION.md is the authoritative
description; this docstring summarizes the load-bearing facts):

- One vendor ``SG100Device`` owns the serial port in RS485 ACTIVE mode.
  Its CommScheduler drives the 10 ms realtime exchange; this module never
  builds protocol frames, parses CRC, or manages raw commands (0x05/0x25/
  0x26/0x27 are vendor-internal details that upper layers must not see).
- ``SetHandControl`` input domain is the SDK USER-ANGLE domain in DEGREES.
  The SDK applies ``joint_angle_map.json`` (per-joint clamp + a*x + b) and
  converts to wire radians internally.  The status callback returns the
  same user-degree domain (inverse map, no receive clamp).
- This module is the ONLY place where canonical software radians meet the
  vendor user-degree domain, through ``SG100HardwareMap`` (canonical rad
  <-> user rad) plus a single rad<->deg conversion.
- Because the vendor send-side clamp is not round-trip safe (a target
  outside the json user range would be silently clamped, making the
  recorded action differ from the encoded one), targets outside the
  loaded ``joint_angle_map.json`` ranges are REJECTED here instead.
- Realtime entry follows the vendor production pattern
  (teaching_mode_example.cpp): seed the control cache with MODE_NULL
  (no drive output), wait one scheduler cycle so the cache is what the
  palm sees, then ``SwitchToRealtimeMode``.  After entry the palm applies
  NULL until the first real target, so entering realtime cannot move the
  hand by itself.
"""

from __future__ import annotations

import ctypes
import json
import os
import subprocess
import threading
import time
from pathlib import Path

import numpy as np

from sg100_teleop.robot.sg100_driver import (
    HAND_DOF,
    SG100CommandSent,
    SG100Driver,
    SG100HandFeedback,
    SG100HardwareMap,
)


ROOT = Path(__file__).resolve().parents[2]

DEFAULT_SDK_DIR = Path.home() / "SG100_SDK_Publish"
WRAPPER_SOURCE = Path(__file__).resolve().parent / "sg100_c_wrapper.cpp"
WRAPPER_LIBRARY = (
    Path(__file__).resolve().parent / "_sg100_c_wrapper.so"
)

DEFAULT_BAUDRATE = 1_000_000

# CtrlMode_e (sg100_types.h).  MODE_NULL = no drive output (safe passive
# hold); MODE_JOINT_POSITION = absolute joint-position control referenced
# to the joint board zero — the vendor's own teleop-style examples
# (teaching_mode, interpolated motion) use it.
MODE_NULL = 0
MODE_POSITION = 1
MODE_JOINT_POSITION = 7

# Whole-hand control gains, matching teaching_mode_example.cpp.
DEFAULT_KP = 0.5
DEFAULT_KI = 0.0
DEFAULT_KD = 0.05
DEFAULT_OUTPUT_LIMIT = 1.0

# SG100_Quant (sg100_types.h): symmetric int16, [-range, +range] with
# QUANT_MAX = 32767; PID and output_limit both use range 10.
_QUANT_MAX = 32767
_PID_RANGE = 10.0

# Waiting for one 10 ms scheduler cycle before/after the mode switch.
_REALTIME_SETTLE_TIMEOUT_S = 2.0
_PRESWITCH_POLL_TIMEOUT_S = 0.5
_POLL_PERIOD_S = 0.002


def quantize_gain(value: float, range_: float = _PID_RANGE) -> int:
    """SG100_Quant::encode — clamp to ±range, round to int16 steps."""

    clipped = max(-range_, min(range_, float(value)))
    return int(round(clipped * _QUANT_MAX / range_)) & 0xFFFF


def load_vendor_user_ranges(sdk_dir: Path) -> list[tuple[float, float]] | None:
    """Per-joint [min, max] user-degree ranges from joint_angle_map.json.

    Returns None when the file is absent (no vendor range guard possible).
    The values mirror what the SDK clamps on send; rejecting outside-range
    targets here keeps the recorded action equal to the encoded action.
    """

    path = Path(sdk_dir) / "joint_angle_map.json"
    if not path.exists():
        return None
    with path.open("r") as f:
        mapping = json.load(f)
    entries = mapping.get("joint_angle_map", [])
    if not entries:
        return None
    ranges: list[tuple[float, float]] = []
    for entry in entries[:HAND_DOF]:
        ranges.append(
            (float(entry["min_angle"]), float(entry["max_angle"]))
        )
    return ranges


def _load_wrapper_library(
    library_path: Path | None = None,
    sdk_dir: Path | str = DEFAULT_SDK_DIR,
) -> ctypes.CDLL:
    """Load (building if needed) the wrapper shared library."""

    path = (
        Path(library_path)
        if library_path is not None
        else WRAPPER_LIBRARY
    )
    if not path.exists() or (
        library_path is None
        and WRAPPER_SOURCE.stat().st_mtime_ns > path.stat().st_mtime_ns
    ):
        _build_wrapper(sdk_dir=sdk_dir, output_path=path)

    sdk_lib_dir = str(Path(sdk_dir) / "lib" / "linux-x86_64")
    if sdk_lib_dir not in (
        os.environ.get("LD_LIBRARY_PATH", "").split(":")
    ):
        os.environ["LD_LIBRARY_PATH"] = (
            f"{sdk_lib_dir}:{os.environ.get('LD_LIBRARY_PATH', '')}"
        )
        try:
            # Re-exec is unacceptable; preload by absolute path instead.
            ctypes.CDLL(
                str(Path(sdk_lib_dir) / "libsg100_sdk.so"),
                mode=ctypes.RTLD_GLOBAL,
            )
        except OSError as exc:
            raise SG100SDKDriverError(
                f"cannot preload libsg100_sdk.so: {exc}"
            ) from exc

    lib = ctypes.CDLL(str(path))

    lib.sg100_connect.argtypes = [
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
    ]
    lib.sg100_connect.restype = ctypes.c_int
    lib.sg100_disconnect.argtypes = []
    lib.sg100_disconnect.restype = ctypes.c_int
    lib.sg100_is_connected.argtypes = []
    lib.sg100_is_connected.restype = ctypes.c_int

    lib.sg100_poll_status.argtypes = [
        np.ctypeslib.ndpointer(dtype=np.float64, shape=(HAND_DOF,)),
        np.ctypeslib.ndpointer(dtype=np.float64, shape=(HAND_DOF,)),
        np.ctypeslib.ndpointer(dtype=np.uint8, shape=(HAND_DOF,)),
        ctypes.POINTER(ctypes.c_uint64),
    ]
    lib.sg100_poll_status.restype = ctypes.c_uint64
    lib.sg100_status_recv_count.argtypes = []
    lib.sg100_status_recv_count.restype = ctypes.c_uint32
    lib.sg100_request_hand_status.argtypes = []
    lib.sg100_request_hand_status.restype = ctypes.c_int

    lib.sg100_set_hand_control.argtypes = [
        np.ctypeslib.ndpointer(dtype=np.float32, shape=(HAND_DOF,)),
        ctypes.c_int,
        ctypes.c_uint8,
        ctypes.c_uint16,
        ctypes.c_uint16,
        ctypes.c_uint16,
        ctypes.c_uint16,
    ]
    lib.sg100_set_hand_control.restype = ctypes.c_int
    lib.sg100_switch_to_realtime_mode.argtypes = []
    lib.sg100_switch_to_realtime_mode.restype = ctypes.c_int
    lib.sg100_get_work_mode.argtypes = []
    lib.sg100_get_work_mode.restype = ctypes.c_int

    for name in (
        "sg100_error_frames",
        "sg100_hand_status_callbacks",
        "sg100_hand_control_calls",
        "sg100_switch_to_realtime_calls",
    ):
        func = getattr(lib, name)
        func.argtypes = []
        func.restype = ctypes.c_uint64

    lib.sg100_test_struct_sizes.argtypes = [
        np.ctypeslib.ndpointer(dtype=np.int32, shape=(6,)),
        ctypes.c_int,
    ]
    lib.sg100_test_struct_sizes.restype = ctypes.c_int
    lib.sg100_test_serialize_hand_control.argtypes = [
        np.ctypeslib.ndpointer(dtype=np.float32, shape=(HAND_DOF,)),
        ctypes.c_int,
        ctypes.c_uint8,
        ctypes.c_uint16,
        ctypes.c_uint16,
        ctypes.c_uint16,
        ctypes.c_uint16,
        np.ctypeslib.ndpointer(dtype=np.uint8, shape=(176,)),
        ctypes.c_int,
    ]
    lib.sg100_test_serialize_hand_control.restype = ctypes.c_int
    return lib


def _build_wrapper(
    sdk_dir: Path | str = DEFAULT_SDK_DIR,
    output_path: Path | str = WRAPPER_LIBRARY,
) -> Path:
    sdk_dir = Path(sdk_dir)
    output_path = Path(output_path)
    if not (sdk_dir / "include" / "sg100_device.h").exists():
        raise SG100SDKDriverError(
            f"SG100 SDK not found at {sdk_dir} "
            "(expected include/ + lib/linux-x86_64)"
        )
    cmd = [
        "g++",
        "-std=c++14",
        "-O2",
        "-shared",
        "-fPIC",
        f"-I{sdk_dir / 'include'}",
        str(WRAPPER_SOURCE),
        f"-L{sdk_dir / 'lib' / 'linux-x86_64'}",
        "-lsg100_sdk",
        "-o",
        str(output_path),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        raise SG100SDKDriverError(
            "failed to compile SG100 C wrapper:\n"
            f"{' '.join(cmd)}\n{result.stderr}"
        )
    return output_path


class SG100SDKDriverError(RuntimeError):
    pass


class SG100SDKDriver(SG100Driver):
    """Canonical-rad driver over the vendor SDK's native RS485 backend."""

    def __init__(
        self,
        port: str,
        *,
        baudrate: int = DEFAULT_BAUDRATE,
        hardware_map: SG100HardwareMap | None = None,
        sdk_dir: Path | str | None = None,
        control_mode: int = MODE_JOINT_POSITION,
        kp: float = DEFAULT_KP,
        ki: float = DEFAULT_KI,
        kd: float = DEFAULT_KD,
        output_limit: float = DEFAULT_OUTPUT_LIMIT,
    ) -> None:
        self.port = str(port)
        self.baudrate = int(baudrate)
        self.hardware_map = (
            hardware_map
            if hardware_map is not None
            else SG100HardwareMap()
        )
        self.sdk_dir = Path(
            sdk_dir if sdk_dir is not None else DEFAULT_SDK_DIR
        )
        self.control_mode = int(control_mode)
        self._pid_p = quantize_gain(kp)
        self._pid_i = quantize_gain(ki)
        self._pid_d = quantize_gain(kd)
        self._output_limit = quantize_gain(output_limit)

        self._vendor_user_ranges = load_vendor_user_ranges(self.sdk_dir)

        self._lib: ctypes.CDLL | None = None
        self._connected = False
        self._realtime = False
        self._last_sequence = 0
        self._last_feedback: SG100HandFeedback | None = None
        self._preswitch_feedback_flow: bool | None = None
        self._events: list[SG100CommandSent] = []
        self._events_lock = threading.Lock()
        self._rejected_commands = 0

    # -- wrapper access ------------------------------------------------------

    def _wrapper(self) -> ctypes.CDLL:
        if self._lib is None:
            self._lib = _load_wrapper_library(sdk_dir=self.sdk_dir)
            self._assert_struct_abi()
        return self._lib

    def _assert_struct_abi(self) -> None:
        """Fail loudly if the vendor struct layout changed."""

        sizes = np.zeros(6, dtype=np.int32)
        if self._lib.sg100_test_struct_sizes(sizes, 6) != 1:
            raise SG100SDKDriverError("cannot query vendor struct sizes")
        expected = (16, 12, 176, 132, 14, 12)
        names = (
            "Joint_Control_t", "Joint_Status_t", "Hand_Control_t",
            "Hand_Status_t", "MotorDown_t", "MotorUp_t",
        )
        for name, want, got in zip(names, expected, sizes):
            if int(got) != want:
                raise SG100SDKDriverError(
                    f"vendor {name} size changed: expected {want}, "
                    f"SDK reports {got} — regenerate the integration"
                )

    # -- lifecycle ----------------------------------------------------------

    def connect(self) -> None:
        if self._connected:
            return
        code = self._wrapper().sg100_connect(
            self.port.encode(),
            self.baudrate,
            str(self.sdk_dir).encode(),
        )
        if code != 0:
            raise ConnectionError(
                f"sg100_connect({self.port}) -> {code}"
            )
        self._connected = True

    def start(self) -> None:
        if not self._connected:
            raise RuntimeError("connect() before start()")

    def stop(self) -> None:
        """Safe passive hold: seed MODE_NULL (no drive output).

        The vendor scheduler keeps transmitting the NULL control at its own
        cadence, so the hand neither tracks a stale target nor receives a
        zero-position target.
        """

        if self._connected:
            self._send_null_control()

    def disconnect(self) -> None:
        if not self._connected:
            return
        self._wrapper().sg100_disconnect()
        self._connected = False
        self._realtime = False
        self._last_sequence = 0
        self._last_feedback = None
        self._preswitch_feedback_flow = None

    # -- realtime entry -----------------------------------------------------

    def enter_realtime(self, timeout_s: float = _REALTIME_SETTLE_TIMEOUT_S) -> None:
        """Vendor-native realtime entry with a motionless cache.

        Order (see SG100_LATEST_SDK_INTEGRATION.md): seed MODE_NULL so the
        cached control carries no drive output, then
        ``SwitchToRealtimeMode`` (0x25 -> ACK -> scheduler restart) and
        require the status flow to resume.  The palm applies NULL targets
        after the resume, so entry itself cannot move the hand regardless
        of the physical pose.

        The pre-switch wait is best-effort: a NONRT_PAUSED palm is not
        guaranteed to answer the scheduler's frames, and the vendor's own
        flow does not gate on pre-switch status (the NULL cache is what
        makes the restart safe, not a pre-switch response).  The post-switch
        wait is mandatory evidence that realtime feedback flows.
        """

        if not self._connected:
            raise RuntimeError("connect() before enter_realtime()")
        if self._realtime:
            return
        if not self._send_null_control():
            raise SG100SDKDriverError(
                "MODE_NULL cache seed rejected by the SDK"
            )
        self._preswitch_feedback_flow = (
            self._wait_fresh_sequence(_PRESWITCH_POLL_TIMEOUT_S)
            is not None
        )
        if not self._wrapper().sg100_switch_to_realtime_mode():
            raise SG100SDKDriverError(
                "vendor SwitchToRealtimeMode failed"
            )
        if self._wait_fresh_sequence(timeout_s) is None:
            raise SG100SDKDriverError(
                "no status flow after realtime switch"
            )
        self._realtime = True

    @property
    def realtime_active(self) -> bool:
        return self._realtime

    @property
    def preswitch_feedback_flow(self) -> bool | None:
        """Whether status flowed after the NULL seed but BEFORE the mode
        switch (best-effort evidence; None before enter_realtime)."""

        return self._preswitch_feedback_flow

    # -- feedback -----------------------------------------------------------

    def read_feedback(self) -> SG100HandFeedback | None:
        """Newest SDK callback sample in canonical form (deduped)."""

        if not self._connected:
            return None
        positions_deg = np.zeros(HAND_DOF, dtype=np.float64)
        currents = np.zeros(HAND_DOF, dtype=np.float64)
        modes = np.zeros(HAND_DOF, dtype=np.uint8)
        recv_ns = ctypes.c_uint64(0)
        sequence = int(
            self._wrapper().sg100_poll_status(
                positions_deg, currents, modes, ctypes.byref(recv_ns)
            )
        )
        if sequence == 0:
            return None
        if sequence == self._last_sequence:
            return self._last_feedback  # cache not refreshed

        sdk_user_rad = np.deg2rad(positions_deg)
        canonical = self.hardware_map.actual_to_canonical(sdk_user_rad)
        feedback = SG100HandFeedback(
            q_rad=canonical,
            received_monotonic_ns=int(recv_ns.value),
            sequence=sequence,
            valid=bool(
                np.all(np.isfinite(canonical))
                and np.all(np.isfinite(positions_deg))
            ),
            raw_positions=sdk_user_rad.copy(),
            currents=currents.copy(),
        )
        self._last_sequence = sequence
        self._last_feedback = feedback
        return feedback

    def request_status(self) -> bool:
        """Queue an explicit whole-hand status query (vendor 0x26)."""

        if not self._connected:
            return False
        return bool(self._wrapper().sg100_request_hand_status())

    def _wait_fresh_sequence(
        self, timeout_s: float
    ) -> SG100HandFeedback | None:
        deadline = time.monotonic() + timeout_s
        start_sequence = self._last_sequence
        while time.monotonic() < deadline:
            feedback = self.read_feedback()
            if feedback is not None and feedback.sequence > start_sequence:
                return feedback
            time.sleep(_POLL_PERIOD_S)
        return None

    # -- command ------------------------------------------------------------

    def send_positions_rad(
        self,
        q_rad: np.ndarray,
        *,
        command_bundle_id: int | None = None,
    ) -> SG100CommandSent | None:
        """Send one whole-hand target through vendor ``SetHandControl``.

        Fail-closed until ``enter_realtime`` succeeded: before that the
        cached control is MODE_NULL and no position target may overwrite
        it.  Targets whose vendor user-domain degrees fall outside the
        ``joint_angle_map.json`` range are rejected (never silently
        clamped) so the recorded action always equals the encoded action.
        """

        if not self._connected or not self._realtime:
            return None

        sdk_user_rad = self.hardware_map.canonical_to_actual(q_rad)
        positions_deg = np.rad2deg(sdk_user_rad)
        if not self._user_deg_within_vendor_range(positions_deg):
            self._rejected_commands += 1
            return None
        payload = np.ascontiguousarray(
            positions_deg, dtype=np.float32
        )
        accepted = int(
            self._wrapper().sg100_set_hand_control(
                payload,
                HAND_DOF,
                self.control_mode,
                self._pid_p,
                self._pid_i,
                self._pid_d,
                self._output_limit,
            )
        )
        if accepted != 1:
            return None
        event = SG100CommandSent(
            q_rad=self.hardware_map.clamp_canonical(
                np.asarray(q_rad, dtype=np.float64)
            ),
            sent_monotonic_ns=time.monotonic_ns(),
            command_bundle_id=command_bundle_id,
        )
        with self._events_lock:
            self._events.append(event)
        return event

    def hold_passive(self) -> bool:
        """Seed MODE_NULL without leaving realtime (safe no-drive hold)."""

        if not self._connected:
            return False
        return self._send_null_control()

    def drain_command_events(self) -> tuple[SG100CommandSent, ...]:
        with self._events_lock:
            events = tuple(self._events)
            self._events.clear()
        return events

    # -- internals ------------------------------------------------------------

    def _send_null_control(self) -> bool:
        payload = np.zeros(HAND_DOF, dtype=np.float32)
        return bool(
            self._wrapper().sg100_set_hand_control(
                payload,
                HAND_DOF,
                MODE_NULL,
                self._pid_p,
                self._pid_i,
                self._pid_d,
                self._output_limit,
            )
        )

    def _user_deg_within_vendor_range(self, positions_deg: np.ndarray) -> bool:
        if self._vendor_user_ranges is None:
            return True
        for joint, (low, high) in enumerate(self._vendor_user_ranges):
            value = float(positions_deg[joint])
            if value < low or value > high:
                return False
        return True

    # -- diagnostics ----------------------------------------------------------

    @property
    def diagnostics(self) -> dict[str, int]:
        lib = self._wrapper()
        return {
            "hand_status_callbacks": int(
                lib.sg100_hand_status_callbacks()
            ),
            "hand_control_calls": int(lib.sg100_hand_control_calls()),
            "switch_to_realtime_calls": int(
                lib.sg100_switch_to_realtime_calls()
            ),
            "error_frames": int(lib.sg100_error_frames()),
            "status_recv_count": int(lib.sg100_status_recv_count()),
            "vendor_work_mode": int(lib.sg100_get_work_mode()),
            "rejected_commands": int(self._rejected_commands),
        }

    @property
    def fault_active(self) -> bool:
        return bool(
            self._connected
            and int(self._wrapper().sg100_error_frames()) > 0
        )

    @property
    def serial_open(self) -> bool:
        return bool(
            self._connected and self._wrapper().sg100_is_connected()
        )

    @property
    def is_connected(self) -> bool:
        return bool(self._connected)

    @property
    def vendor_user_ranges(self) -> list[tuple[float, float]] | None:
        return self._vendor_user_ranges
