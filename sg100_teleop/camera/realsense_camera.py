"""严格 profile、独立 capture thread 的 RealSense 相机封装。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Any

import numpy as np


_FIXED_COLOR_CONTROLS: dict[str, tuple[float, float]] = {
    "D405": (30000.0, 62.0),
    "D455": (300.0, 40.0),
}
"""D405曝光不能高于30000,D455曝光不能高于300,否则曝光时间过长达不到30帧"""


@dataclass(frozen=True)
class RealSenseFrame:
    role: str
    host_receive_monotonic_ns: int
    device_timestamp_ms: float
    frame_number: int
    color_rgb: np.ndarray
    depth_uint16: np.ndarray | None
    depth_host_receive_monotonic_ns: int = -1
    depth_device_timestamp_ms: float = float("nan")
    depth_frame_number: int = -1


class RealSenseCamera:
    """一台 RealSense 的 capture ring；绝不 silent profile fallback。"""

    def __init__(
        self,
        *,
        role: str,
        serial: str,
        expected_model: str,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        enable_depth: bool = True,
        ring_size: int = 120,
        rs_module: Any | None = None,
    ) -> None:
        if not role or not serial or not expected_model:
            raise ValueError("role、serial、expected_model 均不能为空")
        if width <= 0 or height <= 0 or fps <= 0 or ring_size < 2:
            raise ValueError("camera profile/ring_size 非法")
        self.role = str(role)
        self.serial = str(serial)
        self.expected_model = str(expected_model)
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.enable_depth = bool(enable_depth)
        self._rs = rs_module
        self._pipeline = None
        self._align = None
        self._ring: deque[RealSenseFrame] = deque(maxlen=int(ring_size))
        self._lock = threading.Lock()
        self._shutdown = threading.Event()
        self._thread: threading.Thread | None = None
        self.capture_error_count = 0
        self.last_error = ""
        self._metadata: dict[str, Any] = {}

    def _load_rs(self):
        if self._rs is None:
            try:
                import pyrealsense2 as rs
            except ImportError as exc:
                raise RuntimeError(
                    "缺少 pyrealsense2；RealSense recorder 无法启动"
                ) from exc
            self._rs = rs
        return self._rs

    def _device_and_profiles(self, rs):
        context = rs.context()
        for device in context.query_devices():
            found_serial = str(device.get_info(rs.camera_info.serial_number))
            if found_serial == self.serial:
                profiles: list[str] = []
                for sensor in device.query_sensors():
                    for profile in sensor.get_stream_profiles():
                        try:
                            video = profile.as_video_stream_profile()
                            profiles.append(
                                f"{profile.stream_type()} {video.width()}x"
                                f"{video.height()} @{video.fps()} {profile.format()}"
                            )
                        except Exception:
                            profiles.append(str(profile))
                return device, sorted(set(profiles))
        raise RuntimeError(f"未找到 RealSense serial={self.serial}")

    @staticmethod
    def _intrinsics_matrix(intrinsics) -> np.ndarray:
        return np.asarray(
            [
                [intrinsics.fx, 0.0, intrinsics.ppx],
                [0.0, intrinsics.fy, intrinsics.ppy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _configure_fixed_color_controls(rs, device, model: str) -> dict[str, Any]:
        """Apply the production color controls; no runtime override exists."""

        camera_family = next(
            (name for name in _FIXED_COLOR_CONTROLS if model.endswith(name)),
            None,
        )
        if camera_family is None:
            raise RuntimeError(f"没有为 {model!r} 配置固定彩色相机参数")
        exposure_us, gain = _FIXED_COLOR_CONTROLS[camera_family]

        color_sensors = []
        for sensor in device.query_sensors():
            if any(
                profile.stream_type() == rs.stream.color
                for profile in sensor.get_stream_profiles()
            ):
                color_sensors.append(sensor)
        if len(color_sensors) != 1:
            raise RuntimeError(
                f"{model}: 应有且仅有一个 color stream sensor，"
                f"实际找到 {len(color_sensors)} 个"
            )
        sensor = color_sensors[0]
        controls = (
            (rs.option.enable_auto_exposure, 0.0, "enable_auto_exposure"),
            (rs.option.exposure, exposure_us, "exposure_us"),
            (rs.option.gain, gain, "gain"),
            (
                rs.option.enable_auto_white_balance,
                1.0,
                "enable_auto_white_balance",
            ),
        )
        applied: dict[str, Any] = {}
        for option, value, name in controls:
            if not sensor.supports(option):
                raise RuntimeError(f"{model}: 彩色传感器不支持固定参数 {name}")
            sensor.set_option(option, value)
            actual = float(sensor.get_option(option))
            if not np.isclose(actual, value, atol=1e-6, rtol=0.0):
                raise RuntimeError(
                    f"{model}: 固定参数 {name} 写入失败: "
                    f"requested={value}, actual={actual}"
                )
            applied[name] = bool(actual) if name.startswith("enable_") else actual
        applied["white_balance_mode"] = "AUTO"
        return applied

    def start(self) -> None:
        if self._thread is not None:
            return
        rs = self._load_rs()
        device, supported = self._device_and_profiles(rs)
        model = str(device.get_info(rs.camera_info.name))
        if model != self.expected_model:
            raise RuntimeError(
                f"{self.role}: serial={self.serial} model={model!r}，"
                f"预期 {self.expected_model!r}"
            )
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.serial)
        config.enable_stream(
            rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps
        )
        if self.enable_depth:
            config.enable_stream(
                rs.stream.depth, self.width, self.height, rs.format.z16, self.fps
            )
        try:
            active = pipeline.start(config)
        except Exception as exc:
            listing = "\n  ".join(supported) or "<none>"
            raise RuntimeError(
                f"{self.role}: 不支持请求的严格 profile "
                f"{self.width}x{self.height}@{self.fps}; supported profiles:\n  "
                f"{listing}"
            ) from exc
        try:
            color_profile = active.get_stream(
                rs.stream.color
            ).as_video_stream_profile()
            actual = (
                color_profile.width(), color_profile.height(), color_profile.fps(),
            )
            expected = (self.width, self.height, self.fps)
            depth_profile = None
            depth_sensor = None
            if self.enable_depth:
                depth_profile = active.get_stream(
                    rs.stream.depth
                ).as_video_stream_profile()
                actual += (
                    depth_profile.width(), depth_profile.height(), depth_profile.fps(),
                )
                expected += (self.width, self.height, self.fps)
            if actual != expected:
                raise RuntimeError(
                    f"{self.role}: SDK profile 与请求不一致: actual={actual}, "
                    f"expected={expected}"
                )
            active_device = active.get_device()
            color_controls = self._configure_fixed_color_controls(
                rs, active_device, model
            )
            if self.enable_depth:
                depth_sensor = active_device.first_depth_sensor()
            self._metadata = {
                "role": self.role,
                "model": model,
                "serial": self.serial,
                "width": self.width,
                "height": self.height,
                "fps": self.fps,
                "depth_enabled": self.enable_depth,
                "depth_scale": (
                    float(depth_sensor.get_depth_scale())
                    if depth_sensor is not None else float("nan")
                ),
                "color_intrinsics": self._intrinsics_matrix(
                    color_profile.get_intrinsics()
                ),
                "depth_intrinsics": (
                    self._intrinsics_matrix(depth_profile.get_intrinsics())
                    if depth_profile is not None else np.full((3, 3), np.nan)
                ),
                "color_format": "RGB8",
                "depth_format": "Z16_UINT16" if self.enable_depth else "",
                "color_stream_profile": (
                    f"{self.width}x{self.height}@{self.fps}:RGB8"
                ),
                "depth_stream_profile": (
                    f"{self.width}x{self.height}@{self.fps}:Z16"
                    if self.enable_depth else ""
                ),
                "depth_aligned_to": "color" if self.enable_depth else "DISABLED",
                "extrinsics_to_robot": "UNAVAILABLE / NOT_CALIBRATED",
                "synchronization": "HOST MONOTONIC SOFTWARE ALIGNMENT",
                "color_controls": color_controls,
            }
        except Exception:
            pipeline.stop()
            raise
        self._pipeline = pipeline
        self._align = rs.align(rs.stream.color) if self.enable_depth else None
        self._shutdown.clear()
        self._thread = threading.Thread(
            target=self._capture_loop,
            name=f"realsense-{self.role}-capture",
            daemon=True,
        )
        self._thread.start()
        deadline = time.monotonic() + 5.0
        while not self.ready and time.monotonic() < deadline:
            time.sleep(0.01)
        if not self.ready:
            expected_frames = "color+depth frameset" if self.enable_depth else "color frame"
            detail = self.last_error or f"2 秒内未收到完整 {expected_frames}"
            self.stop()
            raise RuntimeError(f"{self.role}: stream 未 READY: {detail}")

    def _capture_loop(self) -> None:
        assert self._pipeline is not None
        while not self._shutdown.is_set():
            try:
                frameset = self._pipeline.wait_for_frames(timeout_ms=250)
                host_ns = time.monotonic_ns()
                if self.enable_depth:
                    assert self._align is not None
                    selected = self._align.process(frameset)
                    color = selected.get_color_frame()
                    depth = selected.get_depth_frame()
                    if not color or not depth:
                        raise RuntimeError("frameset 缺少 color/depth")
                else:
                    color = frameset.get_color_frame()
                    depth = None
                    if not color:
                        raise RuntimeError("frameset 缺少 color")
                frame = RealSenseFrame(
                    role=self.role,
                    host_receive_monotonic_ns=host_ns,
                    device_timestamp_ms=float(color.get_timestamp()),
                    frame_number=int(color.get_frame_number()),
                    color_rgb=np.asanyarray(color.get_data()).copy(),
                    depth_uint16=(
                        np.asanyarray(depth.get_data()).astype(np.uint16, copy=True)
                        if depth is not None else None
                    ),
                    depth_host_receive_monotonic_ns=(
                        host_ns if depth is not None else -1
                    ),
                    depth_device_timestamp_ms=(
                        float(depth.get_timestamp()) if depth is not None
                        else float("nan")
                    ),
                    depth_frame_number=(
                        int(depth.get_frame_number()) if depth is not None else -1
                    ),
                )
                with self._lock:
                    # The persisted depth image is the aligned frame, so its
                    # intrinsics must describe that image rather than the
                    # native pre-align depth stream.
                    try:
                        if depth is None:
                            raise ValueError("depth disabled")
                        aligned_depth_profile = (
                            depth.profile.as_video_stream_profile()
                        )
                        self._metadata["depth_intrinsics"] = (
                            self._intrinsics_matrix(
                                aligned_depth_profile.get_intrinsics()
                            )
                        )
                    except Exception:
                        # Active profile metadata remains available; capture
                        # continues and validation can inspect provenance.
                        pass
                    self._ring.append(frame)
            except Exception as exc:
                self.capture_error_count += 1
                self.last_error = f"{type(exc).__name__}: {exc}"

    @property
    def ready(self) -> bool:
        with self._lock:
            return bool(self._ring)

    def latest_before(self, timestamp_ns: int) -> RealSenseFrame | None:
        tick = int(timestamp_ns)
        with self._lock:
            for frame in reversed(self._ring):
                if frame.host_receive_monotonic_ns <= tick:
                    return frame
        return None

    def clear(self) -> None:
        with self._lock:
            self._ring.clear()

    def metadata(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._metadata)

    def stop(self) -> None:
        self._shutdown.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            finally:
                self._pipeline = None
        self._align = None

    close = stop
