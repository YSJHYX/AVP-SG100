"""单一 30 Hz master timeline 上的 causal latest-before-tick sampler。"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np

from sg100_teleop.data_collection.vla_raw_buffer import VlaRawBuffer
from sg100_teleop.data_collection.vla_raw_schema import (
    FixedRateTimeline,
    FreshnessThresholds,
    parse_camera_modalities,
    vector,
)
from sg100_teleop.data_collection.vla_raw_writer import BoundedMediaWriter


def _age_and_valid(
    tick_ns: int,
    source_ns: int,
    threshold_ns: int,
    declared_valid: bool,
    floor_ns: int = 0,
) -> tuple[int, bool]:
    """Causal freshness gate。

    source 必须满足 ``floor_ns <= source <= tick_ns``：floor 是 episode start
    （On 边界），保证正式 episode 永不把 pre-On 产生的 source sample 标成
    valid；pre-On sample 仍可被 latest-before 选中并保留在 RAW 中，但只以
    invalid 诚实出现，不参与 Curated/训练。
    """
    source = int(source_ns)
    if source <= 0 or source > int(tick_ns) or source < int(floor_ns):
        return -1, False
    age = int(tick_ns) - source
    return age, bool(declared_valid and age <= int(threshold_ns))


class SynchronizedSampler:
    """30 Hz 仅属于 learning/data plane，不改变任何 robot control Hz。"""

    def __init__(
        self,
        *,
        hz: float,
        start_monotonic_ns: int,
        telemetry,
        head_camera,
        wrist_camera,
        camera_roles: tuple[str, ...] = ("head", "right_wrist"),
        camera_modalities: tuple[str, ...] = ("rgb", "depth"),
        buffer: VlaRawBuffer,
        media_writer: BoundedMediaWriter,
        thresholds: FreshnessThresholds,
    ) -> None:
        self.timeline = FixedRateTimeline(hz, start_monotonic_ns)
        self.telemetry = telemetry
        self.head_camera = head_camera
        self.wrist_camera = wrist_camera
        self.camera_roles = tuple(camera_roles)
        self.camera_modalities = parse_camera_modalities(camera_modalities)
        if "head" not in self.camera_roles:
            raise ValueError("当前 recorder 必须启用 head camera")
        if set(self.camera_roles) - {"head", "right_wrist"}:
            raise ValueError("未知 camera role")
        self.buffer = buffer
        self.media_writer = media_writer
        self.thresholds = thresholds
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._start_index = 0
        self.telemetry_publish_hz_requested = float("nan")

    @staticmethod
    def _depth_source(frame) -> tuple[int, float, int, bool]:
        if frame is None or frame.depth_uint16 is None:
            return -1, float("nan"), -1, False
        host_ns = int(getattr(frame, "depth_host_receive_monotonic_ns", -1))
        if host_ns <= 0:
            host_ns = int(frame.host_receive_monotonic_ns)
        device_ms = float(getattr(frame, "depth_device_timestamp_ms", np.nan))
        if not np.isfinite(device_ms):
            device_ms = float(frame.device_timestamp_ms)
        frame_number = int(getattr(frame, "depth_frame_number", -1))
        if frame_number < 0:
            frame_number = int(frame.frame_number)
        return host_ns, device_ms, frame_number, True

    def start(self, *, start_index: int = 0) -> None:
        if self._thread is not None:
            raise RuntimeError("sampler 已启动")
        self._stop.clear()
        self._start_index = int(start_index)
        self._thread = threading.Thread(
            target=self._run, name="vla-30hz-master-sampler", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        index = self._start_index
        while not self._stop.is_set():
            tick_ns = self.timeline.tick_ns(index)
            delay_s = (tick_ns - time.monotonic_ns()) * 1e-9
            if delay_s > 0 and self._stop.wait(delay_s):
                break
            if self._stop.is_set():
                break
            self.sample_tick(index, tick_ns, time.monotonic_ns())
            index += 1

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                raise RuntimeError("30 Hz sampler 无法结束")
        self._thread = None

    @staticmethod
    def _packet_value(packet: dict[str, Any] | None, name: str, size: int):
        return vector(None if packet is None else packet.get(name), size)

    def sample_tick(self, index: int, tick_ns: int, wakeup_ns: int) -> None:
        floor_ns = self.timeline.start_monotonic_ns
        packet = self.telemetry.latest_before(tick_ns)
        packet_ts = 0 if packet is None else int(packet["publish_monotonic_ns"])
        telemetry_age, telemetry_valid = _age_and_valid(
            tick_ns,
            packet_ts,
            self.thresholds.max_telemetry_age_ns,
            packet is not None,
            floor_ns=floor_ns,
        )
        try:
            requested_hz = (
                float(packet.get("telemetry_publish_hz_requested", np.nan))
                if packet is not None else float("nan")
            )
        except (TypeError, ValueError):
            requested_hz = float("nan")
        if np.isfinite(requested_hz) and requested_hz > 0:
            self.telemetry_publish_hz_requested = requested_hz

        arm_cmd_ts = 0 if packet is None else int(packet.get("arm_qcmd_sent_timestamp_ns", 0))
        arm_cmd_age, arm_cmd_valid = _age_and_valid(
            tick_ns, arm_cmd_ts, self.thresholds.max_arm_command_age_ns,
            bool(packet and packet.get("arm_command_valid", False)),
            floor_ns=floor_ns,
        )
        arm_fb_ts = 0 if packet is None else int(packet.get("arm_qpos_timestamp_ns", 0))
        arm_fb_age, arm_fb_valid = _age_and_valid(
            tick_ns, arm_fb_ts, self.thresholds.max_arm_feedback_age_ns,
            bool(packet and packet.get("arm_feedback_valid", False)),
            floor_ns=floor_ns,
        )
        hand_cmd_ts = 0 if packet is None else int(packet.get("hand_qcmd_timestamp_ns", 0))
        hand_cmd_age, hand_cmd_valid = _age_and_valid(
            tick_ns, hand_cmd_ts, self.thresholds.max_hand_command_age_ns,
            bool(packet and packet.get("hand_command_valid", False)),
            floor_ns=floor_ns,
        )
        hand_fb_ts = 0 if packet is None else int(packet.get("hand_feedback_timestamp_ns", 0))
        hand_fb_age, hand_fb_valid = _age_and_valid(
            tick_ns, hand_fb_ts, self.thresholds.max_hand_feedback_age_ns,
            bool(packet and packet.get("hand_feedback_valid", False)),
            floor_ns=floor_ns,
        )

        arm_desired = self._packet_value(packet, "arm_q_goal_rad", 6)
        arm_qcmd = self._packet_value(packet, "arm_qcmd_sent_rad", 6)
        arm_qpos = self._packet_value(packet, "arm_qpos_rad", 6)
        hand_desired = self._packet_value(packet, "hand_qcmd_retargeted_rad", 11)
        hand_effective = self._packet_value(
            packet, "hand_qcmd_effective_canonical_rad", 11
        )
        hand_sdk = self._packet_value(packet, "hand_qcmd_sdk_deg", 11)
        hand_feedback_sdk = self._packet_value(packet, "hand_feedback_sdk_rad", 11)
        raw_modes = None if packet is None else packet.get("hand_feedback_modes")
        mode_array = np.asarray(raw_modes) if raw_modes is not None else np.empty(0)
        hand_modes = (
            mode_array.astype(np.int16, copy=True)
            if mode_array.shape == (11,) and np.all(np.isfinite(mode_array))
            else np.full(11, -1, dtype=np.int16)
        )
        hand_canonical_declared = bool(
            packet and packet.get("hand_feedback_canonical_valid", False)
        )
        hand_canonical_valid = bool(hand_fb_valid and hand_canonical_declared)
        hand_qpos = (
            self._packet_value(packet, "hand_qpos_canonical_rad", 11)
            if hand_canonical_valid
            else np.full(11, np.nan)
        )
        desired_source_valid = bool(
            telemetry_valid
            and packet
            and packet.get("tracking_valid", False)
            and packet.get("avp_anchored", False)
        )
        arm_desired_valid = bool(
            desired_source_valid and np.all(np.isfinite(arm_desired))
        )
        hand_desired_valid = bool(
            desired_source_valid and np.all(np.isfinite(hand_desired))
        )
        robot_desired_qcmd_valid = bool(
            arm_desired_valid and hand_desired_valid
        )
        robot_desired_qcmd = (
            np.concatenate((arm_desired, hand_desired))
            if robot_desired_qcmd_valid else np.full(17, np.nan)
        )
        robot_qcmd_valid = bool(
            arm_cmd_valid and hand_cmd_valid
            and np.all(np.isfinite(arm_qcmd))
            and np.all(np.isfinite(hand_effective))
        )
        robot_qpos_valid = bool(
            arm_fb_valid and hand_canonical_valid
            and np.all(np.isfinite(arm_qpos))
            and np.all(np.isfinite(hand_qpos))
        )
        robot_qcmd = np.concatenate((
            arm_qcmd if arm_cmd_valid else np.full(6, np.nan),
            hand_effective if hand_cmd_valid else np.full(11, np.nan),
        ))
        robot_qpos = np.concatenate((
            arm_qpos if arm_fb_valid else np.full(6, np.nan),
            hand_qpos if hand_canonical_valid else np.full(11, np.nan),
        ))

        head = self.head_camera.latest_before(tick_ns)
        wrist_enabled = "right_wrist" in self.camera_roles
        wrist = (
            self.wrist_camera.latest_before(tick_ns)
            if wrist_enabled and self.wrist_camera is not None
            else None
        )
        head_ts = 0 if head is None else int(head.host_receive_monotonic_ns)
        wrist_ts = (
            -1
            if not wrist_enabled
            else (0 if wrist is None else int(wrist.host_receive_monotonic_ns))
        )
        head_age, head_valid = _age_and_valid(
            tick_ns, head_ts, self.thresholds.max_camera_age_ns, head is not None,
            floor_ns=floor_ns,
        )
        wrist_age, wrist_valid = (
            _age_and_valid(
                tick_ns,
                wrist_ts,
                self.thresholds.max_camera_age_ns,
                wrist is not None,
                floor_ns=floor_ns,
            )
            if wrist_enabled
            else (-1, False)
        )
        depth_enabled = "depth" in self.camera_modalities
        head_depth_ts, head_depth_device_ms, head_depth_number, head_has_depth = (
            self._depth_source(head) if depth_enabled
            else (-1, float("nan"), -1, False)
        )
        wrist_depth_ts, wrist_depth_device_ms, wrist_depth_number, wrist_has_depth = (
            self._depth_source(wrist)
            if depth_enabled and wrist_enabled
            else (-1, float("nan"), -1, False)
        )
        head_depth_age, head_depth_valid = (
            _age_and_valid(
                tick_ns, head_depth_ts, self.thresholds.max_camera_age_ns,
                head_has_depth, floor_ns=floor_ns,
            )
            if depth_enabled else (-1, False)
        )
        wrist_depth_age, wrist_depth_valid = (
            _age_and_valid(
                tick_ns, wrist_depth_ts, self.thresholds.max_camera_age_ns,
                wrist_has_depth, floor_ns=floor_ns,
            )
            if depth_enabled and wrist_enabled else (-1, False)
        )
        backlog_before = self.media_writer.backlog
        media_enqueued = bool(
            head is not None and (not wrist_enabled or wrist is not None)
            and self.media_writer.enqueue(index, head, wrist)
        )
        if not media_enqueued:
            head_valid = False
            head_depth_valid = False
            if wrist_enabled:
                wrist_valid = False
                wrist_depth_valid = False

        row = {
            "timestamp_ns": int(tick_ns),
            "timestamp_relative_ns": int(tick_ns - self.timeline.start_monotonic_ns),
            "sampler_wakeup_timestamp_ns": int(wakeup_ns),
            "sampler_jitter_ns": int(wakeup_ns - tick_ns),
            "arm_q_goal_rad": arm_desired,
            "arm_qcmd_sent_rad": arm_qcmd,
            "arm_qcmd_source_timestamp_ns": arm_cmd_ts,
            "arm_qcmd_age_ns": arm_cmd_age,
            "arm_qcmd_sequence": -1 if packet is None else int(packet.get("arm_qcmd_sequence", -1)),
            "arm_command_valid": arm_cmd_valid,
            "arm_qpos_rad": arm_qpos,
            "arm_qpos_source_timestamp_ns": arm_fb_ts,
            "arm_qpos_age_ns": arm_fb_age,
            "arm_feedback_sequence": -1 if packet is None else int(packet.get("arm_feedback_sequence", -1)),
            "arm_feedback_valid": arm_fb_valid,
            "arm_controller_error": -1 if packet is None or packet.get("controller_error") is None else int(packet["controller_error"]),
            "arm_status": -1 if packet is None or packet.get("arm_status") is None else int(packet["arm_status"]),
            "arm_feedback_channel": "UNAVAILABLE" if packet is None else str(packet.get("feedback_channel", "UNAVAILABLE")),
            "arm_sender_hz": 0.0 if packet is None else float(packet.get("sender_hz", 0.0)),
            "hand_qcmd_retargeted_canonical_rad": hand_desired,
            "hand_qcmd_effective_canonical_rad": hand_effective,
            "hand_qcmd_sdk_deg": hand_sdk,
            "hand_qcmd_source_timestamp_ns": hand_cmd_ts,
            "hand_qcmd_age_ns": hand_cmd_age,
            "hand_command_sequence": -1 if packet is None else int(packet.get("hand_command_sequence", -1)),
            "hand_command_valid": hand_cmd_valid,
            "hand_feedback_sdk_rad": hand_feedback_sdk,
            "hand_feedback_modes": hand_modes,
            "hand_feedback_source_timestamp_ns": hand_fb_ts,
            "hand_feedback_age_ns": hand_fb_age,
            "hand_feedback_sequence": -1 if packet is None else int(packet.get("hand_feedback_sequence", -1)),
            "hand_feedback_valid": hand_fb_valid,
            "hand_qpos_canonical_rad": hand_qpos,
            "hand_feedback_canonical_valid": hand_canonical_valid,
            "robot_qcmd_17d_rad": robot_qcmd,
            "robot_qcmd_17d_valid": robot_qcmd_valid,
            "robot_desired_qcmd_17d_rad": robot_desired_qcmd,
            "robot_desired_qcmd_17d_valid": robot_desired_qcmd_valid,
            "robot_qpos_17d_rad": robot_qpos,
            "robot_qpos_17d_valid": robot_qpos_valid,
            "tracking_valid": bool(packet and packet.get("tracking_valid", False) and telemetry_valid),
            "avp_anchored": bool(packet and packet.get("avp_anchored", False)),
            "motion_enabled": bool(packet and packet.get("motion_enabled", False)),
            "estop_latched": bool(packet and packet.get("estop_latched", False)),
            "telemetry_valid": telemetry_valid,
            "telemetry_age_ns": telemetry_age,
            "telemetry_publish_sequence": -1 if packet is None else int(packet.get("publish_sequence", -1)),
            "telemetry_publish_timestamp_ns": packet_ts,
            "head_camera_host_timestamp_ns": head_ts,
            "head_camera_device_timestamp_ms": np.nan if head is None else float(head.device_timestamp_ms),
            "head_camera_frame_number": -1 if head is None else int(head.frame_number),
            "head_camera_age_ns": head_age,
            "head_camera_valid": head_valid,
            "head_depth_host_timestamp_ns": head_depth_ts,
            "head_depth_device_timestamp_ms": head_depth_device_ms,
            "head_depth_frame_number": head_depth_number,
            "head_depth_age_ns": head_depth_age,
            "head_depth_valid": head_depth_valid,
            "wrist_camera_host_timestamp_ns": wrist_ts,
            "wrist_camera_device_timestamp_ms": np.nan if wrist is None else float(wrist.device_timestamp_ms),
            "wrist_camera_frame_number": -1 if wrist is None else int(wrist.frame_number),
            "wrist_camera_age_ns": wrist_age,
            "wrist_camera_valid": wrist_valid,
            "wrist_depth_host_timestamp_ns": wrist_depth_ts,
            "wrist_depth_device_timestamp_ms": wrist_depth_device_ms,
            "wrist_depth_frame_number": wrist_depth_number,
            "wrist_depth_age_ns": wrist_depth_age,
            "wrist_depth_valid": wrist_depth_valid,
            "head_rgb_frame_index": int(index),
            "head_depth_frame_index": (
                int(index) if depth_enabled and head_has_depth else -1
            ),
            "wrist_rgb_frame_index": int(index) if wrist_enabled else -1,
            "wrist_depth_frame_index": (
                int(index)
                if depth_enabled and wrist_enabled and wrist_has_depth else -1
            ),
            "writer_backlog": int(backlog_before),
            "writer_drop_count": int(self.media_writer.drop_count),
        }
        self.buffer.append(row)
