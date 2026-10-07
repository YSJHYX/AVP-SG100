"""独立 30 Hz Arm-Hand + RealSense raw VLA recorder 入口。

Production 默认仍为 D455 + D405；head-only 仅用于正式 bring-up。
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import select
import sys
import termios
import tty

from sg100_teleop.camera.realsense_camera import RealSenseCamera
from sg100_teleop.data_collection.episode_controller import (
    EpisodeController,
    EpisodeState,
)
from sg100_teleop.data_collection.vla_raw_schema import (
    DEFAULT_CAMERA_MODALITIES,
    DEFAULT_REQUIRED_CAMERAS,
    FreshnessThresholds,
    parse_camera_modalities,
    parse_required_cameras,
)
from sg100_teleop.data_collection.vla_raw_validator import format_report
from sg100_teleop.telemetry.teleop_telemetry import (
    NullTelemetrySubscriber,
    TeleopTelemetrySubscriber,
)


DEFAULT_LANGUAGE_INSTRUCTION = ""


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Passive 30 Hz Arm-Hand + RealSense raw VLA recorder"
    )
    parser.add_argument(
        "--telemetry",
        default="127.0.0.1:5560",
        help="HOST:PORT；camera-only bring-up 使用字面值 optional",
    )
    parser.add_argument(
        "--required-cameras",
        type=parse_required_cameras,
        default=DEFAULT_REQUIRED_CAMERAS,
        help="正式支持 head 或 head,right_wrist（默认 production 双相机）",
    )
    parser.add_argument(
        "--camera-modalities",
        type=parse_camera_modalities,
        default=DEFAULT_CAMERA_MODALITIES,
        help="支持 rgb 或 rgb,depth（默认 D455/D405 均为 RGB-only）",
    )
    parser.add_argument("--head-serial", required=True)
    parser.add_argument(
        "--wrist-serial",
        default=None,
        help="D405 serial；production 双相机模式必需，head-only 模式不使用",
    )
    parser.add_argument("--instruction", default=None)
    parser.add_argument("--output-dir", default="recording/vla_raw")
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--max-arm-command-age-ms", type=float, default=100.0)
    parser.add_argument("--max-arm-feedback-age-ms", type=float, default=100.0)
    parser.add_argument("--max-hand-command-age-ms", type=float, default=100.0)
    parser.add_argument("--max-hand-feedback-age-ms", type=float, default=100.0)
    parser.add_argument("--max-camera-age-ms", type=float, default=100.0)
    parser.add_argument("--max-telemetry-age-ms", type=float, default=100.0)
    parser.add_argument("--writer-queue-size", type=int, default=180)
    return parser


def _instruction(args) -> str:
    value = args.instruction
    if value is None:
        value = DEFAULT_LANGUAGE_INSTRUCTION
    if not str(value).strip():
        value = input("Language instruction: ")
    value = str(value).strip()
    if not value:
        raise ValueError("Language instruction 不能为空")
    return value


def _thresholds(args) -> FreshnessThresholds:
    ms = lambda value: int(float(value) * 1_000_000)
    return FreshnessThresholds(
        max_arm_command_age_ns=ms(args.max_arm_command_age_ms),
        max_arm_feedback_age_ns=ms(args.max_arm_feedback_age_ms),
        max_hand_command_age_ns=ms(args.max_hand_command_age_ms),
        max_hand_feedback_age_ns=ms(args.max_hand_feedback_age_ms),
        max_camera_age_ns=ms(args.max_camera_age_ms),
        max_telemetry_age_ns=ms(args.max_telemetry_age_ms),
    )


def create_runtime_sources(
    args,
    *,
    camera_factory=RealSenseCamera,
    subscriber_factory=TeleopTelemetrySubscriber,
):
    """按显式 role 配置创建 source；disabled wrist 不会被实例化。"""
    roles = parse_required_cameras(args.required_cameras)
    modalities = parse_camera_modalities(args.camera_modalities)
    if "right_wrist" in roles and not str(args.wrist_serial or "").strip():
        raise ValueError(
            "--required-cameras head,right_wrist 要求 --wrist-serial"
        )
    telemetry_required = str(args.telemetry).strip().lower() != "optional"
    telemetry = (
        subscriber_factory(args.telemetry)
        if telemetry_required
        else NullTelemetrySubscriber()
    )
    common = {
        "width": args.width,
        "height": args.height,
        "fps": args.camera_fps,
        "enable_depth": "depth" in modalities,
    }
    head = camera_factory(
        role="head",
        serial=args.head_serial,
        expected_model="RealSense D455",
        **common,
    )
    wrist = (
        camera_factory(
            role="right_wrist",
            serial=args.wrist_serial,
            expected_model="RealSense D405",
            **common,
        )
        if "right_wrist" in roles
        else None
    )
    return telemetry, head, wrist, roles, telemetry_required


def _read_recording_command(controller: EpisodeController, prompt: str) -> str:
    """Read Off as a line while recognizing I/i immediately on the recorder TTY."""
    fd = sys.stdin.fileno()
    saved_tty = termios.tcgetattr(fd) if sys.stdin.isatty() else None
    command = bytearray()
    print(prompt, end="", flush=True)
    try:
        if saved_tty is not None:
            tty.setcbreak(fd)
        while True:
            readable, _, _ = select.select([fd], [], [], 0.05)
            if not readable:
                continue
            key = os.read(fd, 1)
            if not key or key == b"\x04":
                raise EOFError("recorder input closed")
            if key in (b"I", b"i"):
                event, timestamp_ns = controller.toggle_intervention()
                print(f"\n[INTERVENTION {event}] timestamp_ns={timestamp_ns}")
                continue
            if key in (b"\x7f", b"\x08"):
                if command:
                    command.pop()
                continue
            if key in (b"\r", b"\n"):
                return command.decode("utf-8", errors="replace").strip()
            command.extend(key)
    finally:
        if saved_tty is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved_tty)


def main() -> None:
    args = build_arg_parser().parse_args()
    (
        telemetry,
        head,
        wrist,
        required_cameras,
        telemetry_required,
    ) = create_runtime_sources(args)
    instruction = _instruction(args)
    controller = None
    try:
        telemetry.start()
        head.start()
        if wrist is not None:
            wrist.start()
        controller = EpisodeController(
            output_dir=Path(args.output_dir),
            hz=args.hz,
            language_instruction=instruction,
            telemetry=telemetry,
            head_camera=head,
            wrist_camera=wrist,
            required_cameras=required_cameras,
            camera_modalities=args.camera_modalities,
            telemetry_required=telemetry_required,
            thresholds=_thresholds(args),
            writer_queue_size=args.writer_queue_size,
        )
        print("HEAD D455: READY")
        print(f"CAMERA MODALITIES: {','.join(args.camera_modalities)}")
        print(
            "WRIST D405: READY"
            if wrist is not None
            else "WRIST D405: DISABLED (head-only bring-up)"
        )
        print(
            "Robot telemetry: WAITING (On 会执行 alive 检查)"
            if telemetry_required
            else "Robot telemetry: OPTIONAL / ABSENT (camera-only bring-up)"
        )
        if wrist is not None:
            print("DUAL CAMERA SYNCHRONIZATION: HOST MONOTONIC SOFTWARE ALIGNMENT")
        print("STATE: READY")
        while True:
            if controller.state is EpisodeState.PENDING_CONFIRMATION:
                prompt = "Keep this episode? [Y/N/Reset]: "
            elif controller.state is EpisodeState.RECORDING:
                prompt = "RECORDING (输入 Off): "
            else:
                prompt = "READY (输入 On): "
            command = (
                _read_recording_command(controller, prompt)
                if controller.state is EpisodeState.RECORDING
                else input(prompt).strip()
            )
            try:
                result = controller.handle_input(command)
                if command.lower() == "on" and controller.state is EpisodeState.RECORDING:
                    print("RECORDING")
                    print(f"candidate episode index: {result}")
                    print(f"language (frozen): {controller.frozen_instruction}")
                elif command.lower() == "off" and controller.state is EpisodeState.PENDING_CONFIRMATION:
                    print(format_report(result))
                elif command.lower() == "y" and result is not None:
                    print(f"SAVED: {result[0]}")
                    print(f"MEDIA: {result[1]}")
                    print(f"next episode index: {controller.next_episode_index}")
                elif command.lower() in ("n", "reset") and controller.state is EpisodeState.READY:
                    print("DISCARDED; permanent episode index unchanged")
                    if command.lower() == "reset":
                        print("rings/timing/validation state cleared; cameras remain warm")
                elif result is None:
                    print(f"当前状态 {controller.state.value} 忽略输入: {command!r}")
            except (RuntimeError, ValueError, OSError) as exc:
                print(f"拒绝操作: {exc}")
    except KeyboardInterrupt:
        print("\nRecorder interrupted; pending episode will be discarded.")
    finally:
        if controller is not None and controller.state is not EpisodeState.READY:
            controller.abort_pending()
        if wrist is not None:
            wrist.stop()
        head.stop()
        telemetry.stop()


if __name__ == "__main__":
    main()
