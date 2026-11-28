"""Explicit offline demo sources. No SDK, camera device or socket is opened."""
from __future__ import annotations

from collections import deque
import threading
import time

import cv2
import numpy as np

from sg100_teleop.camera.realsense_camera import RealSenseFrame


class DemoCamera:
    def __init__(self, role):
        self.role = role
        self.frames = deque(maxlen=120)
        self.lock = threading.Lock()
        self.capture_error_count = 0
        self.last_error = ""

    def update(self, number, stamp):
        image = np.full((480, 640, 3), (24, 35, 46), dtype=np.uint8)
        for x in range(0, 640, 40):
            cv2.line(image, (x, 0), (x, 480), (35, 49, 61), 1)
        for y in range(0, 480, 40):
            cv2.line(image, (0, y), (640, y), (35, 49, 61), 1)
        offset = int(35 * np.sin(number / 30))
        cv2.rectangle(image, (180 + offset, 180), (425 + offset, 330), (62, 150, 166), -1)
        cv2.rectangle(image, (200 + offset, 200), (405 + offset, 310), (33, 77, 93), 3)
        cv2.putText(image, "OFFLINE DEMO / " + self.role.upper(), (25, 45), cv2.FONT_HERSHEY_SIMPLEX, .7, (220, 235, 240), 1)
        cv2.putText(image, f"frame {number:06d}", (25, 450), cv2.FONT_HERSHEY_SIMPLEX, .5, (170, 190, 200), 1)
        frame = RealSenseFrame(self.role, stamp, number * 1000 / 30, number, image, None)
        with self.lock:
            self.frames.append(frame)

    @property
    def ready(self):
        with self.lock:
            return bool(self.frames and time.monotonic_ns() - self.frames[-1].host_receive_monotonic_ns < 100_000_000)

    def latest_before(self, stamp):
        with self.lock:
            return next((f for f in reversed(self.frames) if f.host_receive_monotonic_ns <= stamp), None)

    def clear(self):
        with self.lock:
            self.frames.clear()

    def metadata(self):
        return {"model": "OFFLINE DEMO", "serial": self.role, "width": 640, "height": 480,
                "fps": 30, "color_intrinsics": np.eye(3), "color_format": "RGB8"}

    def start(self):
        pass

    def stop(self):
        pass


class DemoSources:
    def __init__(self):
        self.cameras = {role: DemoCamera(role) for role in ("head", "right_wrist")}
        self.packets = deque(maxlen=4096)
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self.number = 0
        self.control = {"phase": "READY", "fresh": True, "tracking_valid": True,
                        "avp_anchored": False, "arm_enabled": False, "hand_enabled": False,
                        "mode": "free", "estop_latched": False, "recording_locked": False}
        self.thread = threading.Thread(target=self._run, name="offline-demo", daemon=True)

    def start(self):
        self._update()
        self.thread.start()

    def _update(self):
        self.number += 1
        stamp = time.monotonic_ns()
        for camera in self.cameras.values():
            camera.update(self.number, stamp)
        q = [0.] * 11
        packet = {"publish_monotonic_ns": stamp, "publish_sequence": self.number,
                  "telemetry_publish_hz_requested": 30., "tracking_valid": True,
                  "avp_anchored": self.control["avp_anchored"],
                  "motion_enabled": self.control["arm_enabled"], "estop_latched": False,
                  "arm_q_goal_rad": [0.] * 6, "arm_qcmd_sent_rad": [0.] * 6,
                  "arm_qcmd_sent_timestamp_ns": stamp, "arm_qcmd_sequence": self.number,
                  "arm_command_valid": self.control["arm_enabled"], "arm_qpos_rad": [0.] * 6,
                  "arm_qpos_timestamp_ns": stamp, "arm_feedback_sequence": self.number,
                  "arm_feedback_valid": True, "controller_error": 0, "arm_status": 0,
                  "sender_hz": 30., "hand_qcmd_retargeted_rad": q,
                  "hand_qcmd_effective_canonical_rad": q, "hand_qcmd_sdk_deg": q,
                  "hand_qcmd_timestamp_ns": stamp, "hand_command_sequence": self.number,
                  "hand_command_valid": self.control["hand_enabled"], "hand_feedback_sdk_rad": q,
                  "hand_feedback_modes": [7] * 11, "hand_feedback_timestamp_ns": stamp,
                  "hand_feedback_sequence": self.number, "hand_feedback_valid": True,
                  "hand_feedback_canonical_valid": False}
        with self.lock:
            self.packets.append(packet)

    def _run(self):
        while not self._stop.wait(1 / 30):
            self._update()

    def latest_before(self, stamp):
        with self.lock:
            return next((dict(p) for p in reversed(self.packets) if p["publish_monotonic_ns"] <= stamp), None)

    def latest(self):
        with self.lock:
            return dict(self.packets[-1]) if self.packets else None

    def is_alive(self):
        value = self.latest()
        return bool(value and time.monotonic_ns() - value["publish_monotonic_ns"] < 500_000_000)

    @property
    def stats(self):
        return {"received_packets": self.number, "sequence_gap_count": 0, "invalid_packets": 0}

    def clear(self):
        with self.lock:
            self.packets.clear()

    def snapshot(self):
        return dict(self.control, updated_ns=time.monotonic_ns())

    def request(self, command, **arguments):
        if command in ("disarm", "shutdown"):
            self.control.update(arm_enabled=False, hand_enabled=False)
        elif command == "calibrate":
            if self.control["recording_locked"]:
                raise RuntimeError("录制中不能重建锚点")
            self.control.update(avp_anchored=True)
        elif command == "arm":
            if not self.control["avp_anchored"]:
                raise RuntimeError("请先建立锚点")
            self.control.update(arm_enabled=True, hand_enabled=True)
        elif command == "mode":
            self.control["mode"] = arguments["mode"]
        elif command == "recording_lock":
            self.control["recording_locked"] = arguments["active"]
        elif command == "script":
            if self.control["mode"] != "script":
                raise RuntimeError("请先选择 SCRIPT")
        else:
            raise ValueError("未知操作")
        return self.snapshot()

    def stop(self):
        self._stop.set()
        self.thread.join(timeout=2)
