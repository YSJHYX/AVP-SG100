"""独立 30 Hz Arm-Hand + Dual RealSense raw VLA recorder 入口。"""

from __future__ import annotations

import argparse
from pathlib import Path

from sg100_teleop.camera.realsense_camera import RealSenseCamera
from sg100_teleop.data_collection.episode_controller import (
    EpisodeController,
    EpisodeState,
)
from sg100_teleop.data_collection.vla_raw_schema import FreshnessThresholds
from sg100_teleop.data_collection.vla_raw_validator import format_report
from sg100_teleop.telemetry.teleop_telemetry import TeleopTelemetrySubscriber


DEFAULT_LANGUAGE_INSTRUCTION = ""


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Passive 30 Hz RM65 + SG100 + D455 + D405 raw VLA recorder"
    )
    parser.add_argument("--telemetry", default="127.0.0.1:5560")
    parser.add_argument("--head-serial", required=True)
    parser.add_argument("--wrist-serial", required=True)
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


def main() -> None:
    args = build_arg_parser().parse_args()
    instruction = _instruction(args)
    telemetry = TeleopTelemetrySubscriber(args.telemetry)
    head = RealSenseCamera(
        role="head",
        serial=args.head_serial,
        expected_model="Intel RealSense D455",
        width=args.width,
        height=args.height,
        fps=args.camera_fps,
        enable_depth=True,
    )
    wrist = RealSenseCamera(
        role="right_wrist",
        serial=args.wrist_serial,
        expected_model="Intel RealSense D405",
        width=args.width,
        height=args.height,
        fps=args.camera_fps,
        enable_depth=True,
    )
    controller = None
    try:
        telemetry.start()
        head.start()
        wrist.start()
        controller = EpisodeController(
            output_dir=Path(args.output_dir),
            hz=args.hz,
            language_instruction=instruction,
            telemetry=telemetry,
            head_camera=head,
            wrist_camera=wrist,
            thresholds=_thresholds(args),
            writer_queue_size=args.writer_queue_size,
        )
        print("HEAD D455: READY")
        print("WRIST D405: READY")
        print("Robot telemetry: WAITING (On 会执行 alive 检查)")
        print("DUAL CAMERA SYNCHRONIZATION: HOST MONOTONIC SOFTWARE ALIGNMENT")
        print("STATE: READY")
        while True:
            if controller.state is EpisodeState.PENDING_CONFIRMATION:
                prompt = "Keep this episode? [Y/N/Reset]: "
            elif controller.state is EpisodeState.RECORDING:
                prompt = "RECORDING (输入 Off): "
            else:
                prompt = "READY (输入 On): "
            command = input(prompt).strip()
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
        wrist.stop()
        head.stop()
        telemetry.stop()


if __name__ == "__main__":
    main()
