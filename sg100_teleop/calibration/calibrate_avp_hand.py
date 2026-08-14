import argparse
import sys
import time
from pathlib import Path

import numpy as np
from avp_stream import VisionProStreamer

from sg100_teleop.calibration.io import (
    save_calibration,
)
from sg100_teleop.calibration.session import (
    CalibrationSession,
    REQUIRED_POSES,
)
from sg100_teleop.tracking.avp_hand_adapter import (
    AVPHandFeatureExtractor,
)
from sg100_teleop.tracking.avp_source import (
    get_hand_from_tracking,
)


POSE_INSTRUCTIONS = (
    (
        "neutral",
        "【中立姿态】请保持右手自然放松、手掌张开。"
        "食指、中指和小指保持自然、轻微弯曲，不要刻意完全伸直，也不要握拳；"
        "拇指自然放置。保持姿势稳定，等待采样完成。",
    ),
    (
        "thumb_opposition",
        "【拇指对掌】保持其他手指自然放松。将拇指向掌心内侧移动，"
        "尽量向小指根部方向靠近；不要明显弯曲手腕。保持姿势稳定。",
    ),
    (
        "thumb_abduction",
        "【拇指外展】保持手掌和其他手指稳定。将拇指从手掌向外侧充分张开，"
        "不要旋转手腕。保持姿势稳定。",
    ),
    (
        "thumb_distal_negative",
        "【拇指末端伸展】保持拇指根部位置尽量稳定。将拇指末端关节向伸展方向运动，"
        "使末端比中立姿态更直；不要改变手腕姿态。保持姿势稳定。",
    ),
    (
        "thumb_distal_positive",
        "【拇指末端弯曲】保持拇指根部位置尽量稳定。明显弯曲拇指末端关节，"
        "使末端比中立姿态更弯曲。保持姿势稳定。",
    ),
    (
        "index_adduction",
        "【食指内收】保持手掌稳定。将食指向中指方向靠拢，"
        "尽量减少食指弯曲，只做侧向内收动作。保持姿势稳定。",
    ),
    (
        "finger_flexion",
        "【手指主屈曲】保持手掌稳定。同时弯曲食指、中指和小指的主要屈曲关节，"
        "做自然抓握方向的动作，但不要刻意强调最末端关节，也不要用力握到极限。",
    ),
    (
        "finger_distal_negative",
        "【手指末端伸展】保持手掌以及各手指近端姿态尽量稳定。将食指、中指和小指"
        "的末端关节向伸展方向运动，使末端明显比中立姿态更直；"
        "不要强行做不舒服的反向过伸。保持姿势稳定。",
    ),
    (
        "finger_distal_positive",
        "【手指末端弯曲】保持手掌以及各手指近端姿态尽量稳定。明显弯曲食指、中指和"
        "小指的末端关节，使末端明显比中立姿态更弯曲。保持姿势稳定。",
    ),
    (
        "little_outward",
        "【小指外展】保持手掌和其他手指稳定。将小指向手掌外侧充分张开，"
        "尽量减少小指屈曲，只做侧向外展动作。保持姿势稳定。",
    ),
)

POSE_INSTRUCTION_MAP = dict(POSE_INSTRUCTIONS)
MAX_BIDIRECTIONAL_RETRIES = 3


class ChineseArgumentParser(argparse.ArgumentParser):
    """Keep argparse's standard behavior while localizing CLI scaffolding."""

    @staticmethod
    def _localize(text: str) -> str:
        return (
            text.replace("usage:", "用法：")
            .replace("options:", "选项：")
        )

    def format_usage(self) -> str:
        return self._localize(super().format_usage())

    def format_help(self) -> str:
        return self._localize(super().format_help())

    def error(self, message):
        self.print_usage(sys.stderr)
        self.exit(
            2,
            f"{self.prog}: 错误：{message}\n",
        )


def collect_pose(
    streamer,
    extractor,
    session,
    pose_name,
    duration,
    hz,
    hand_side,
):
    period = 1.0 / hz
    start = time.perf_counter()
    valid = 0
    invalid = 0

    while time.perf_counter() - start < duration:
        loop_start = time.perf_counter()
        tracking = streamer.get_latest()
        hand = get_hand_from_tracking(
            tracking,
            side=hand_side,
        )

        if hand is None:
            invalid += 1
        else:
            try:
                raw = extractor.extract(hand)
                session.add_sample(pose_name, raw)
                valid += 1
            except (
                ValueError,
                np.linalg.LinAlgError,
            ):
                invalid += 1

        elapsed = time.perf_counter() - loop_start
        sleep_time = period - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)

    return valid, invalid


def print_pose_summary(
    session,
    pose_name,
):
    median = session.pose_median(pose_name)

    print(f"\n姿态 {pose_name} 的原始特征中位数：")

    for name, value in median.items():
        print(f"  {name:20s}: {value:+.6f}")


def retry_pose_names(
    failed_feature_names,
) -> tuple[str, ...]:
    """Return only the poses capable of repairing the failed features."""

    failed = set(failed_feature_names)
    required = {"neutral"}

    if "thumb_distal" in failed:
        required.update(
            {
                "thumb_distal_negative",
                "thumb_distal_positive",
            }
        )

    if failed.intersection(
        {
            "index_distal",
            "middle_distal",
            "little_distal",
        }
    ):
        required.update(
            {
                "finger_distal_negative",
                "finger_distal_positive",
            }
        )

    return tuple(
        pose
        for pose in REQUIRED_POSES
        if pose in required
    )


def print_bidirectional_diagnostics(
    diagnostics,
) -> None:
    print("\n===== 双向端点诊断 =====")

    for item in diagnostics.values():
        print(
            f"[{item.status}] {item.feature_name:16s} "
            f"negative={item.negative:+.6f} "
            f"neutral={item.neutral:+.6f} "
            f"positive={item.positive:+.6f} "
            f"neg_delta={item.negative_delta:+.6f} "
            f"pos_delta={item.positive_delta:+.6f} "
            f"product={item.product:+.6f}"
        )

        if "negative_too_close_to_neutral" in item.failure_reasons:
            print("  原因：negative 动作幅度过小，端点仍接近放松姿态。")

        if "positive_too_close_to_neutral" in item.failure_reasons:
            print("  原因：positive 动作幅度过小，端点仍接近放松姿态。")

        if "endpoints_on_same_side" in item.failure_reasons:
            print("  原因：两个语义方向落在 neutral 的同一侧，请确认动作方向相反。")


def sample_pose_interactively(
    *,
    streamer,
    extractor,
    session,
    pose_name,
    duration,
    hz,
    hand_side,
    countdown,
) -> None:
    print()
    print("=" * 60)
    print(f"姿态：{pose_name}")
    print(POSE_INSTRUCTION_MAP[pose_name])
    input("\n准备好后按回车键开始……")

    for n in range(countdown, 0, -1):
        print(f"{n} 秒后开始……")
        time.sleep(1.0)

    # A retry must replace the old median, not silently mix good and
    # bad attempts into one endpoint.
    session.clear_pose(pose_name)
    print("正在采样……")

    valid, invalid = collect_pose(
        streamer=streamer,
        extractor=extractor,
        session=session,
        pose_name=pose_name,
        duration=duration,
        hz=hz,
        hand_side=hand_side,
    )

    print(f"有效帧数：{valid}")
    print(f"无效帧数：{invalid}")
    minimum_valid = max(
        20,
        int(duration * hz * 0.5),
    )

    if valid < minimum_valid:
        raise RuntimeError(
            f"姿态 {pose_name} 的有效跟踪帧过少："
            f"需要至少 {minimum_valid} 帧，实际 {valid} 帧。"
            "请检查手部是否完整进入视野后重新校准。"
        )

    print_pose_summary(session, pose_name)


def _format_failed_features(diagnostics) -> str:
    return "、".join(
        item.feature_name
        for item in diagnostics.values()
        if not item.passed
    )


def main():
    parser = ChineseArgumentParser(
        description="采集 Apple Vision Pro 右手姿态并生成 SG100 手部校准文件。",
        add_help=False,
    )
    parser.add_argument(
        "-h",
        "--help",
        action="help",
        help="显示帮助信息并退出",
    )
    parser.add_argument(
        "--ip",
        required=True,
        help="Apple Vision Pro 的 IP 地址",
    )
    parser.add_argument(
        "--hand",
        choices=["right"],
        default="right",
        help="当前已验证的 SG100 形态仅支持右手",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.5,
        help="每个姿态的采样时长（默认：1.5 秒）",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=60.0,
        help="跟踪采样频率（默认：60 Hz）",
    )
    parser.add_argument(
        "--countdown",
        type=int,
        default=2,
        help="每个姿态采样前的倒计时（默认：2 秒）",
    )
    parser.add_argument(
        "--output",
        default=(
            "sg100_teleop/config/"
            "avp_right_calibration.yaml"
        ),
        help="输出校准 YAML 路径",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="显示 AVP 跟踪库的详细日志",
    )
    args = parser.parse_args()

    if args.duration <= 0.0:
        parser.error("--duration 必须大于 0")

    if args.hz <= 0.0:
        parser.error("--hz 必须大于 0")

    if args.countdown < 0:
        parser.error("--countdown 不能为负数")

    print(f"正在连接 Vision Pro：{args.ip}")
    streamer = VisionProStreamer(
        ip=args.ip,
        record=False,
        verbose=args.verbose,
    )
    extractor = AVPHandFeatureExtractor()
    session = CalibrationSession()

    print("\n===== SG100 / AVP 手部校准 =====\n")
    print(f"每个姿态将采样 {args.duration:.1f} 秒。")
    print("请采用舒适、可重复的动作端点，不要强迫手部达到生理极限。")
    print("neutral 是自然放松、可以轻微弯曲的手型，不要刻意完全伸直。")

    for pose_name, _ in POSE_INSTRUCTIONS:
        sample_pose_interactively(
            streamer=streamer,
            extractor=extractor,
            session=session,
            pose_name=pose_name,
            duration=args.duration,
            hz=args.hz,
            hand_side=args.hand,
            countdown=args.countdown,
        )

    retries_used = 0

    while True:
        diagnostics = session.bidirectional_diagnostics()
        print_bidirectional_diagnostics(diagnostics)
        failed_features = tuple(
            name
            for name, item in diagnostics.items()
            if not item.passed
        )

        if not failed_features:
            break

        if retries_used >= MAX_BIDIRECTIONAL_RETRIES:
            raise RuntimeError(
                "双向端点校准连续失败，已达到最多 "
                f"{MAX_BIDIRECTIONAL_RETRIES} 次重采限制。"
                f"失败特征：{_format_failed_features(diagnostics)}。"
                "未生成或覆盖校准 YAML；请根据上方原因检查动作方向与幅度后重试。"
            )

        retries_used += 1
        poses_to_retry = retry_pose_names(failed_features)
        print(
            f"\n第 {retries_used}/{MAX_BIDIRECTIONAL_RETRIES} 次定向重采。"
            f"失败特征：{'、'.join(failed_features)}"
        )
        print("仅重采以下相关姿态：" + "、".join(poses_to_retry))

        for pose_name in poses_to_retry:
            sample_pose_interactively(
                streamer=streamer,
                extractor=extractor,
                session=session,
                pose_name=pose_name,
                duration=args.duration,
                hz=args.hz,
                hand_side=args.hand,
                countdown=args.countdown,
            )

    print("\n===== 正在生成校准 =====")
    calibration = session.build()
    output = Path(args.output)
    save_calibration(
        calibration,
        output,
        metadata={
            "device": "apple_vision_pro",
            "tracking_library": "avp_stream",
            "hand": args.hand,
            "sampling_hz": args.hz,
            "duration_per_pose_sec": args.duration,
            "samples_per_pose": {
                name: session.sample_count(name)
                for name, _ in POSE_INSTRUCTIONS
            },
        },
    )

    print(f"\n校准文件已保存至：\n{output}")
    print("\n===== 校准完成 =====")


if __name__ == "__main__":
    main()
