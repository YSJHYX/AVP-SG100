"""A bounded latest-frame RGB tap; one encoder per workstation, not per tab."""
from __future__ import annotations

import threading
import time

import cv2


class PreviewHub:
    def __init__(self, cameras, fps=12):
        self.cameras = cameras
        self.period = 1.0 / fps
        self.condition = threading.Condition()
        self.frames = {}
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="rgb-ui-preview", daemon=True)

    def start(self):
        self._thread.start()

    def _run(self):
        previous = {}
        while not self._stop.is_set():
            started = time.monotonic()
            for role, camera in self.cameras.items():
                frame = camera.latest_before(time.monotonic_ns())
                if frame is None or frame.frame_number == previous.get(role):
                    continue
                previous[role] = frame.frame_number
                ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(frame.color_rgb, cv2.COLOR_RGB2BGR),
                                           [cv2.IMWRITE_JPEG_QUALITY, 78])
                if ok:
                    with self.condition:
                        self.frames[role] = (frame.frame_number, encoded.tobytes())
                        self.condition.notify_all()
            self._stop.wait(max(0, self.period - (time.monotonic() - started)))

    def stream(self, role):
        previous = None
        while not self._stop.is_set():
            with self.condition:
                self.condition.wait_for(lambda: self._stop.is_set() or self.frames.get(role, (None,))[0] != previous,
                                        timeout=1)
                value = self.frames.get(role)
            if value is None or value[0] == previous:
                continue
            previous, data = value
            yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n"

    def close(self):
        self._stop.set()
        with self.condition:
            self.condition.notify_all()
        self._thread.join(timeout=2)
