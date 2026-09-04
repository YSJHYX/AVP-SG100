"""Teleop control plane -> recorder data plane 的只读 UDP 遥测。

UDP publisher 是 non-blocking / best-effort。任何序列化或发送异常都只计数，
绝不向 robot-control loop 传播。subscriber 使用同一台主机的
``CLOCK_MONOTONIC`` 做 causal latest-before-tick 选择。
"""

from __future__ import annotations

from collections import deque
import json
import select
import socket
import threading
import time
from typing import Any, Mapping

import numpy as np


TELEMETRY_SCHEMA_VERSION = 1
MAX_UDP_PAYLOAD = 65_507


def parse_endpoint(endpoint: str) -> tuple[str, int]:
    value = str(endpoint).strip()
    if not value or ":" not in value:
        raise ValueError("endpoint 必须采用 HOST:PORT 格式")
    host, port_text = value.rsplit(":", 1)
    if not host:
        raise ValueError("endpoint host 不能为空")
    try:
        port = int(port_text)
    except ValueError as exc:
        raise ValueError("endpoint port 必须是整数") from exc
    if not 1 <= port <= 65_535:
        raise ValueError("endpoint port 必须在 1..65535")
    return host, port


def _json_value(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    return value


def build_teleop_snapshot(
    *,
    now_ns: int,
    tracking_received_ns: int | None,
    tracking_timeout_ns: int,
    avp_anchored: bool,
    arm_backend,
    arm_q_goal_rad,
    arm_sender_hz: float,
    hand_driver,
    fallback_hand_feedback,
    restricted_hand_backend,
    hand_qcmd_retargeted_rad,
    latest_hand_command: Mapping[str, Any] | None,
) -> dict[str, Any]:
    """只读取 backend cache，组装一个完整 telemetry schema snapshot。"""
    arm_command = (
        arm_backend.latest_command_sent() if arm_backend is not None else None
    )
    arm_feedback = arm_backend.joint_state() if arm_backend is not None else None
    hand_feedback = (
        getattr(hand_driver, "latest_feedback", fallback_hand_feedback)
        if hand_driver is not None else None
    )
    tracking_valid = bool(
        tracking_received_ns is not None
        and int(now_ns) - int(tracking_received_ns) <= int(tracking_timeout_ns)
    )
    configured_backends = [
        backend for backend in (arm_backend, restricted_hand_backend)
        if backend is not None
    ]
    motion_enabled = bool(
        configured_backends
        and all(bool(backend.enabled) for backend in configured_backends)
    )
    hand_canonical_valid = bool(
        hand_feedback is not None
        and hand_feedback.canonical_valid
        and hand_feedback.valid
    )
    return {
        "tracking_valid": tracking_valid,
        "avp_anchored": bool(avp_anchored),
        "motion_enabled": motion_enabled,
        "estop_latched": bool(
            restricted_hand_backend is not None
            and restricted_hand_backend.estopped
        ),
        "arm_q_goal_rad": arm_q_goal_rad,
        "arm_qcmd_sent_rad": arm_command.q_rad if arm_command is not None else None,
        "arm_qcmd_sent_timestamp_ns": (
            arm_command.sent_monotonic_ns if arm_command is not None else 0
        ),
        "arm_qcmd_sequence": (
            arm_command.command_sequence if arm_command is not None else -1
        ),
        "arm_command_valid": arm_command is not None,
        "arm_qpos_rad": arm_feedback.q_rad if arm_feedback is not None else None,
        "arm_qpos_timestamp_ns": (
            arm_feedback.received_monotonic_ns if arm_feedback is not None else 0
        ),
        "arm_feedback_sequence": (
            arm_feedback.sequence if arm_feedback is not None else -1
        ),
        "arm_feedback_valid": bool(arm_feedback is not None and arm_feedback.valid),
        "controller_error": (
            arm_feedback.controller_error if arm_feedback is not None else None
        ),
        "arm_status": arm_feedback.arm_status if arm_feedback is not None else None,
        "feedback_channel": "TCP_POLL" if arm_feedback is not None else "UNAVAILABLE",
        "sender_hz": float(arm_sender_hz) if arm_backend is not None else 0.0,
        "hand_qcmd_retargeted_rad": hand_qcmd_retargeted_rad,
        "hand_qcmd_effective_canonical_rad": (
            latest_hand_command["effective"] if latest_hand_command is not None else None
        ),
        "hand_qcmd_sdk_deg": (
            latest_hand_command["sdk_deg"] if latest_hand_command is not None else None
        ),
        "hand_qcmd_timestamp_ns": (
            latest_hand_command["timestamp_ns"] if latest_hand_command is not None else 0
        ),
        "hand_command_sequence": (
            latest_hand_command["sequence"] if latest_hand_command is not None else -1
        ),
        "hand_command_valid": latest_hand_command is not None,
        "hand_feedback_sdk_rad": (
            hand_feedback.sdk_callback_positions_rad
            if hand_feedback is not None else None
        ),
        "hand_feedback_modes": (
            hand_feedback.joint_modes if hand_feedback is not None else None
        ),
        "hand_feedback_timestamp_ns": (
            hand_feedback.received_monotonic_ns if hand_feedback is not None else 0
        ),
        "hand_feedback_sequence": (
            hand_feedback.sequence if hand_feedback is not None else -1
        ),
        "hand_feedback_valid": bool(
            hand_feedback is not None and hand_feedback.valid
        ),
        "hand_qpos_canonical_rad": (
            hand_feedback.q_rad if hand_canonical_valid else None
        ),
        "hand_feedback_canonical_valid": hand_canonical_valid,
        "hand_feedback_mapping_status": (
            hand_feedback.feedback_mapping_status
            if hand_feedback is not None else "F_FB_NOT_PROVIDED"
        ),
    }


class TeleopTelemetryPublisher:
    """不会阻塞或中断 teleop 的 UDP publisher。"""

    def __init__(self, endpoint: str) -> None:
        self.endpoint = str(endpoint).strip()
        self.enabled = bool(self.endpoint)
        self.publish_sequence = 0
        self.sent_count = 0
        self.drop_count = 0
        self.last_error = ""
        self._destination: tuple[str, int] | None = None
        self._socket: socket.socket | None = None
        if self.enabled:
            self._destination = parse_endpoint(self.endpoint)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setblocking(False)
            self._socket = sock

    def publish(self, snapshot: Mapping[str, Any]) -> bool:
        if not self.enabled:
            return False
        try:
            self.publish_sequence += 1
            packet = dict(snapshot)
            packet["schema_version"] = TELEMETRY_SCHEMA_VERSION
            packet["publish_sequence"] = self.publish_sequence
            packet["publish_monotonic_ns"] = time.monotonic_ns()
            payload = json.dumps(
                _json_value(packet), separators=(",", ":"), allow_nan=True
            ).encode("utf-8")
            if len(payload) > MAX_UDP_PAYLOAD:
                raise ValueError(f"telemetry UDP payload 过大: {len(payload)} bytes")
            assert self._socket is not None and self._destination is not None
            self._socket.sendto(payload, self._destination)
            self.sent_count += 1
            return True
        except Exception as exc:  # robot control 必须完全不受遥测故障影响
            self.drop_count += 1
            self.last_error = f"{type(exc).__name__}: {exc}"
            return False

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None


class TeleopTelemetrySubscriber:
    """后台接收 UDP，并保存可做 causal lookup 的有限 ring buffer。"""

    def __init__(
        self,
        endpoint: str,
        *,
        ring_size: int = 4096,
        alive_timeout_ns: int = 500_000_000,
    ) -> None:
        if ring_size < 2:
            raise ValueError("ring_size 必须 >= 2")
        self.endpoint = str(endpoint)
        self._address = parse_endpoint(endpoint)
        self.alive_timeout_ns = int(alive_timeout_ns)
        self._ring: deque[dict[str, Any]] = deque(maxlen=int(ring_size))
        self._lock = threading.Lock()
        self._shutdown = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None
        self.received_packets = 0
        self.invalid_packets = 0
        self.sequence_gap_count = 0
        self._last_sequence: int | None = None
        self._last_receive_monotonic_ns = 0

    def start(self) -> None:
        if self._thread is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(self._address)
        sock.setblocking(False)
        self._socket = sock
        self._shutdown.clear()
        self._thread = threading.Thread(
            target=self._receive_loop, name="teleop-telemetry-rx", daemon=True
        )
        self._thread.start()

    def _receive_loop(self) -> None:
        assert self._socket is not None
        while not self._shutdown.is_set():
            try:
                readable, _, _ = select.select([self._socket], [], [], 0.05)
                if not readable:
                    continue
                payload, _ = self._socket.recvfrom(MAX_UDP_PAYLOAD)
                packet = json.loads(payload.decode("utf-8"))
                sequence = int(packet["publish_sequence"])
                timestamp = int(packet["publish_monotonic_ns"])
                if int(packet.get("schema_version", -1)) != TELEMETRY_SCHEMA_VERSION:
                    raise ValueError("unsupported telemetry schema")
                if timestamp <= 0:
                    raise ValueError("invalid publish timestamp")
                with self._lock:
                    if self._last_sequence is not None and sequence > self._last_sequence + 1:
                        self.sequence_gap_count += sequence - self._last_sequence - 1
                    self._last_sequence = sequence
                    self._last_receive_monotonic_ns = time.monotonic_ns()
                    self.received_packets += 1
                    self._ring.append(packet)
            except (BlockingIOError, InterruptedError):
                continue
            except Exception:
                self.invalid_packets += 1

    def latest_before(self, timestamp_ns: int) -> dict[str, Any] | None:
        tick = int(timestamp_ns)
        with self._lock:
            eligible = (
                packet for packet in self._ring
                if int(packet["publish_monotonic_ns"]) <= tick
            )
            selected = max(
                eligible,
                key=lambda packet: int(packet["publish_monotonic_ns"]),
                default=None,
            )
            return dict(selected) if selected is not None else None

    def latest(self) -> dict[str, Any] | None:
        with self._lock:
            if not self._ring:
                return None
            selected = max(
                self._ring,
                key=lambda packet: int(packet["publish_monotonic_ns"]),
            )
            return dict(selected)

    def is_alive(self, now_ns: int | None = None) -> bool:
        now = time.monotonic_ns() if now_ns is None else int(now_ns)
        with self._lock:
            received = self._last_receive_monotonic_ns
        return received > 0 and now - received <= self.alive_timeout_ns

    def clear(self) -> None:
        with self._lock:
            self._ring.clear()
            self._last_sequence = None
            self._last_receive_monotonic_ns = 0
            self.sequence_gap_count = 0

    def stop(self) -> None:
        self._shutdown.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
        self._socket = None

    close = stop

    @property
    def stats(self) -> dict[str, int]:
        with self._lock:
            buffered = len(self._ring)
        return {
            "received_packets": self.received_packets,
            "invalid_packets": self.invalid_packets,
            "sequence_gap_count": self.sequence_gap_count,
            "buffered_packets": buffered,
        }

    def __enter__(self) -> "TeleopTelemetrySubscriber":
        self.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self.stop()
