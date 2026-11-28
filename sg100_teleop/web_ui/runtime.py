"""One workstation owner, independent robot process, existing RAW controller."""
from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
import uuid
from zoneinfo import ZoneInfo

import numpy as np

from sg100_teleop.data_collection.episode_controller import EpisodeController, EpisodeState
from .channel import ControlClient, write_json
from .preview import PreviewHub


ROOT = Path(__file__).resolve().parents[2]
MODES = ("free", "hand", "claw", "script", "pinch", "two_finger")


def json_safe(value):
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list, deque)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


@dataclass
class StationConfig:
    avp_ip: str = "192.168.2.106"
    arm_ip: str = "192.168.1.18"
    hand_port: str = "/dev/ttyUSB0"
    head_serial: str = ""
    wrist_serial: str = ""
    hand_sdk_dir: str = ""
    instruction: str = "Place the motor support bracket into the fixture."
    output_dir: str = "recording/vla_raw"
    origin_position: bool = True

    @classmethod
    def parse(cls, value):
        if not isinstance(value, dict) or set(value) - set(cls.__dataclass_fields__):
            raise ValueError("未知工位配置字段")
        config = cls(**value)
        for name in cls.__dataclass_fields__:
            item = getattr(config, name)
            if name == "origin_position":
                if type(item) is not bool:
                    raise ValueError("回初始位必须为布尔值")
            elif not isinstance(item, str) or len(item) > 2048:
                raise ValueError(f"无效配置: {name}")
            else:
                setattr(config, name, item.strip())
        if not config.instruction or not config.output_dir:
            raise ValueError("任务指令和保存目录不能为空")
        return config

    def output_path(self):
        path = Path(self.output_dir).expanduser()
        return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


class Workstation:
    def __init__(self, config_path: str | Path, *, demo=False, settle_seconds=.75):
        self.config_path = Path(config_path)
        self.demo = demo
        self.settle_seconds = settle_seconds
        self.config = StationConfig(output_dir="recording/ui-demo/vla_raw" if demo else "recording/vla_raw")
        if self.config_path.exists():
            self.config = StationConfig.parse(json.loads(self.config_path.read_text()))
        self.controller = None
        self.telemetry = None
        self.cameras = {}
        self.preview = None
        self.client = None
        self.process = None
        self.log_handle = None
        self.log_path = None
        self.phase = "STOPPED"
        self.message = "选择任务并准备设备"
        self.events = deque(maxlen=100)
        self.saved = deque(maxlen=100)
        self.saved_count = 0
        self._fps = {}
        self._station_lock = None
        self.operation_lock = threading.Lock()

    def event(self, message):
        self.message = message
        self.events.append({"time": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(timespec="seconds"), "message": message})

    def configure(self, value):
        if self.controller is not None:
            raise RuntimeError("请先关闭本轮设备再修改工位配置")
        self.config = StationConfig.parse(value)
        if self.demo:
            # Offline data must never land in the production default folder.
            if self.config.output_path() == (ROOT / "recording/vla_raw").resolve():
                raise ValueError("演示数据请使用独立保存目录")
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        write_json(self.config_path, asdict(self.config))
        self.event("工位配置已保存")

    def start(self):
        if self.controller is not None:
            raise RuntimeError("设备已启动")
        self.phase = "STARTING"
        self.event("正在准备相机和遥测")
        try:
            if self.demo:
                if self.config.output_path() == (ROOT / "recording/vla_raw").resolve():
                    raise ValueError("演示数据请使用独立保存目录")
                from .demo import DemoSources
                sources = DemoSources()
                sources.start()
                self.telemetry = self.client = sources
                self.cameras = sources.cameras
            else:
                import fcntl
                lock_path = ROOT / "recording/ui/workstation.lock"
                lock_path.parent.mkdir(parents=True, exist_ok=True)
                self._station_lock = lock_path.open("a")
                try:
                    fcntl.flock(self._station_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError as exc:
                    raise RuntimeError("另一工作台正在使用本机设备") from exc
                if not self.config.head_serial or not self.config.wrist_serial:
                    raise ValueError("请填写两路相机的实际序列号")
                if self.config.head_serial == self.config.wrist_serial:
                    raise ValueError("头部与腕部相机不能使用同一序列号")
                from sg100_teleop.camera.realsense_camera import RealSenseCamera
                from sg100_teleop.telemetry.teleop_telemetry import TeleopTelemetrySubscriber
                self.telemetry = TeleopTelemetrySubscriber("127.0.0.1:5560")
                self.cameras = {
                    "head": RealSenseCamera(role="head", serial=self.config.head_serial, expected_model="RealSense D455", enable_depth=False),
                    "right_wrist": RealSenseCamera(role="right_wrist", serial=self.config.wrist_serial, expected_model="RealSense D405", enable_depth=False),
                }
                self.telemetry.start()
                for camera in self.cameras.values():
                    camera.start()
            self.controller = EpisodeController(
                output_dir=self.config.output_path(), hz=30, language_instruction=self.config.instruction,
                telemetry=self.telemetry, head_camera=self.cameras["head"],
                wrist_camera=self.cameras["right_wrist"], camera_modalities=("rgb",))
            self.preview = PreviewHub(self.cameras)
            self.preview.start()
            if not self.demo:
                directory = ROOT / "recording/ui/sessions" / uuid.uuid4().hex
                directory.mkdir(parents=True, mode=0o700)
                self.client = ControlClient(directory)
                self.log_path = directory / "control.log"
                self.log_handle = self.log_path.open("w")
                command = [sys.executable, "-m", "sg100_teleop.web_ui.teleop_worker", "--channel", str(directory),
                           "--parent-pid", str(os.getpid()),
                           "--ip", self.config.avp_ip, "--calibration", "sg100_teleop/config/avp_right_calibration.yaml",
                           "--usdz", "sg100_teleop/assets/rm65_sg100/usdz/rm65_sg100_realworld_teleop_compat.usdz",
                           "--wrist-roll-ik", "--arm-backend", "real", "--arm-ip", self.config.arm_ip,
                           "--arm-safety-profile", "10", "--arm-tool-profile", "sg100_attached",
                           "--hand-backend", "real", "--hand-port", self.config.hand_port,
                           "--origin_position", str(self.config.origin_position).lower(),
                           "--telemetry-publish", "127.0.0.1:5560", "--telemetry-publish-hz", "100"]
                if self.config.hand_sdk_dir:
                    command.extend(["--hand-sdk-dir", self.config.hand_sdk_dir])
                self.process = subprocess.Popen(command, cwd=ROOT, stdout=self.log_handle,
                                                stderr=subprocess.STDOUT, start_new_session=True)
                self.event("控制端正在连接；" + ("将执行当前 Biyi 回初始位流程" if self.config.origin_position else "跳过回初始位"))
            else:
                self.event("离线演示设备已就绪，未连接真机")
            self.phase = "RUNNING"
        except Exception:
            self._close_sources()
            self.phase = "ERROR"
            raise

    def _require_ready_control(self):
        if self.client is None:
            raise RuntimeError("请先准备设备")
        state = self.client.snapshot()
        if self.process is not None and self.process.poll() is not None:
            raise RuntimeError("控制进程已退出，请查看诊断日志")
        if state.get("phase") != "READY" or not state.get("fresh"):
            raise RuntimeError("控制端尚未就绪或状态已过期")
        return state

    def control(self, command, **arguments):
        if command not in ("calibrate", "arm", "disarm", "mode", "script"):
            raise ValueError("不支持的遥操操作")
        if command == "mode" and arguments.get("mode") not in MODES:
            raise ValueError("未知手部模式")
        if command == "script" and arguments.get("motion") not in ("close", "hold", "open"):
            raise ValueError("未知脚本动作")
        if command == "calibrate" and self.controller and self.controller.state is not EpisodeState.READY:
            raise RuntimeError("请先结束并处理当前数据，再建立锚点")
        self._require_ready_control()
        result = self.client.request(command, **arguments)
        self.event({"calibrate": "锚点请求已提交，等待完成", "arm": "已启用遥操", "disarm": "已停止遥操",
                    "mode": "手部模式已切换", "script": "脚本动作已设置"}[command])
        return result

    def begin(self, instruction):
        if self.controller is None or self.controller.state is not EpisodeState.READY:
            raise RuntimeError("当前状态不能开始录制")
        state = self._require_ready_control()
        if not all(state.get(key) for key in ("tracking_valid", "avp_anchored", "arm_enabled", "hand_enabled")):
            raise RuntimeError("请先完成锚点、启用遥操并确认初始场景")
        instruction = str(instruction).strip()
        if not instruction or len(instruction) > 2000:
            raise ValueError("请填写有效任务指令")
        self.client.request("recording_lock", active=True)
        try:
            self.controller.language_instruction = instruction
            self.controller.turn_on()
        except Exception:
            self.client.request("recording_lock", active=False)
            raise
        self.event("正在录制，任务指令已冻结")

    def finish(self, *, stop_motion=True):
        if self.controller is None or self.controller.state is not EpisodeState.RECORDING:
            raise RuntimeError("当前没有正在录制的片段")
        stop_error = None
        if stop_motion:
            self.event("正在停止遥操")
            try:
                self.control("disarm")
                self.event("已撤权，保留结束后的稳定画面")
                time.sleep(self.settle_seconds)
            except Exception as exc:
                stop_error = exc
        self.event("正在结束录制并检查数据")
        self.controller.turn_off()
        # Keep C locked until this candidate has been saved or discarded.
        self.event("数据检查完成，请回放并选择保存或丢弃")
        if stop_error:
            raise RuntimeError(f"录制已结束，但停止未确认：{stop_error}；请立即检查现场")

    def restart(self):
        if self.controller is None or self.controller.state is not EpisodeState.RECORDING:
            raise RuntimeError("当前没有正在录制的片段")
        state = self._require_ready_control()
        if state.get("arm_enabled") or state.get("hand_enabled"):
            raise RuntimeError("请先停止遥操、复原场景并退出相机画面，再重录")
        self.controller.restart_recording()
        self.event("旧的未保存片段已丢弃，新片段正在录制；failure_recovery 已标记")

    def confirm(self, decision, *, dagger=False):
        if self.controller is None:
            raise RuntimeError("设备未启动")
        if decision not in ("y", "n", "reset"):
            raise ValueError("未知保存操作")
        if decision == "y" and self.controller.validation_report and self.controller.validation_report.quality == "FAIL":
            raise RuntimeError("本条数据检查失败，请丢弃并重新采集")
        tags = (["failure_recovery"] if self.controller.failure_recovery else []) + (["dagger"] if dagger else [])
        instruction = self.controller.frozen_instruction
        if decision == "y" and self.demo:
            self.controller.candidate_arrays["capture_source"] = np.asarray("offline_demo")
            self.controller.candidate_arrays["dataset_status"] = np.asarray("OFFLINE_DEMO_NOT_TRAINING")
        result = self.controller.confirm(decision, is_dagger=bool(dagger))
        try:
            self.client.request("recording_lock", active=False)
        finally:
            if result:
                self.saved_count += 1
                self.saved.appendleft({"id": result[0].stem, "path": str(result[0]), "demo": self.demo,
                                       "instruction": instruction, "tags": tags})
                self.event(f"已保存 {result[0].stem}")
            else:
                self.event("当前未保存片段已丢弃")

    def close(self):
        if self.controller is not None and self.controller.state is not EpisodeState.READY:
            raise RuntimeError("请先结束录制并保存或丢弃当前片段")
        if self.client is not None and (self.process is None or self.process.poll() is None):
            self.control("disarm")
            self.client.request("shutdown")
        if self.process is not None:
            try:
                self.process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                raise RuntimeError("控制端仍在清理资源；保留进程，请查看诊断窗口")
        self._close_sources()
        self.phase = "STOPPED"
        self.event("本轮设备已关闭")

    def _close_sources(self):
        if self.preview is not None:
            self.preview.close()
        for camera in self.cameras.values():
            try:
                camera.stop()
            except Exception:
                pass
        if self.telemetry is not None:
            self.telemetry.stop()
        if self.log_handle is not None:
            self.log_handle.close()
        if self._station_lock is not None:
            self._station_lock.close()
            self._station_lock = None
        self.controller = self.telemetry = self.client = self.preview = self.log_handle = None
        self.cameras = {}
        self._fps = {}
        self.process = None

    def emergency_cleanup(self):
        """Server exit, not browser refresh: request disarm before any cleanup."""
        if self.client is not None:
            try:
                self.client.request("disarm", timeout=2)
                self.client.request("shutdown", timeout=2)
            except Exception as exc:
                print(f"[UI] stop unconfirmed: {exc}", file=sys.stderr)
        if self.controller is not None:
            try:
                self.controller.abort_pending()
            except Exception:
                pass
        if self.process is not None:
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("[UI] control process still cleaning up; not forcibly killed", file=sys.stderr)
        self._close_sources()

    def snapshot(self):
        now = time.monotonic_ns()
        # HTTP polling can overlap a completed save or a source shutdown.
        # Retain local references instead of re-reading fields after a guard.
        telemetry, client, process = self.telemetry, self.client, self.process
        controller = self.controller
        buffer = controller.buffer if controller else None
        writer = controller.media_writer if controller else None
        validation = controller.validation_report if controller else None
        cameras = {}
        problems = []
        for role, camera in self.cameras.items():
            frame = camera.latest_before(now)
            age = (now - frame.host_receive_monotonic_ns) / 1e6 if frame else None
            if frame:
                prior = self._fps.get(role)
                if prior is None:
                    self._fps[role] = (frame.frame_number, frame.host_receive_monotonic_ns, None)
                elif frame.host_receive_monotonic_ns - prior[1] >= 250_000_000:
                    self._fps[role] = (frame.frame_number, frame.host_receive_monotonic_ns,
                                       (frame.frame_number - prior[0]) * 1e9 / (frame.host_receive_monotonic_ns - prior[1]))
            cameras[role] = {"ready": bool(frame and age <= 100), "age_ms": age,
                             "frame": frame.frame_number if frame else None,
                             "fps": self._fps.get(role, (0, 0, None))[2],
                             "error": camera.last_error, "errors": camera.capture_error_count}
            if not cameras[role]["ready"]:
                problems.append(("头部" if role == "head" else "腕部") + "相机画面已过期或尚未到达")
        packet = telemetry.latest() if telemetry is not None else None
        if packet and packet.get("controller_error") not in (None, 0):
            problems.append(f"机械臂控制器错误：{packet['controller_error']}")
        control = client.snapshot() if client else {}
        if control.get("operator_error"):
            problems.append(control["operator_error"])
        report = asdict(validation) if validation else None
        frames = len(buffer) if buffer is not None else 0
        return json_safe({
            "demo": self.demo, "phase": self.phase, "message": self.message,
            "problems": problems,
            "config": asdict(self.config), "cameras": cameras,
            "control": control,
            "control_exit_code": process.poll() if process else None,
            "telemetry_alive": bool(telemetry and telemetry.is_alive()),
            "recording_state": controller.state.value if controller else "STOPPED",
            "episode_index": controller.next_episode_index if controller else None,
            "instruction": controller.frozen_instruction if controller else None,
            "frames": frames, "duration_s": frames / 30,
            "failure_recovery": bool(controller and controller.failure_recovery),
            "writer_backlog": writer.backlog if writer else 0,
            "writer_errors": writer.error_count if writer else 0,
            "writer_drops": writer.drop_count if writer else 0,
            "quality": report, "training_ready": False,
            "disk_free_gb": round(shutil.disk_usage(ROOT).free / 1e9, 1),
            "saved": list(self.saved), "events": list(self.events),
            "saved_count": self.saved_count,
            "reference_updated_ns": self.reference_path.stat().st_mtime_ns if self.reference_path.exists() else None,
            "log_path": str(self.log_path) if self.log_path else None,
        })

    def media_path(self, episode, role, index):
        if role not in ("head", "right_wrist") or not 0 <= index <= 999999:
            raise ValueError("无效媒体索引")
        if episode == "pending":
            if self.controller is None or self.controller.state is not EpisodeState.PENDING_CONFIRMATION:
                raise FileNotFoundError("没有待确认数据")
            root = self.controller.pending_dir
            if root is None:
                raise FileNotFoundError("候选片段已处理")
        elif len(episode) == 14 and episode.startswith("episode_") and episode[8:].isdigit():
            root = self.config.output_path() / f"{episode}_media"
        else:
            raise ValueError("无效 episode")
        path = root / role / "rgb" / f"{index:06d}.jpg"
        if not path.is_file():
            raise FileNotFoundError("该帧不存在")
        return path

    @property
    def reference_path(self):
        return self.config_path.with_name(self.config_path.stem + "-head-reference.jpg")

    def capture_reference(self):
        if self.controller is None or self.controller.state is not EpisodeState.READY:
            raise RuntimeError("请在开始录制前设置参考画面")
        camera = self.cameras["head"]
        if not camera.ready:
            raise RuntimeError("头部相机画面已过期")
        import cv2
        frame = camera.latest_before(time.monotonic_ns())
        ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(frame.color_rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 97])
        if not ok:
            raise RuntimeError("参考图编码失败")
        self.reference_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.reference_path.with_suffix(".tmp")
        temporary.write_bytes(jpeg.tobytes())
        os.replace(temporary, self.reference_path)
        self.event("头部初始参考画面已保存，仅用于网页叠加")
