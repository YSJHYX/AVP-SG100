import argparse
import time
from pathlib import Path

from avp_stream import VisionProStreamer

from sg100_teleop.calibration.io import (
    save_calibration,
)
from sg100_teleop.calibration.session import (
    CalibrationSession,
)
from sg100_teleop.tracking.avp_hand_adapter import (
    AVPHandFeatureExtractor,
)
from sg100_teleop.tracking.avp_source import (
    get_hand_from_tracking,
)


POSE_INSTRUCTIONS = [
    (
        "neutral",
        "Natural open hand. Keep fingers comfortably straight.",
    ),
    (
        "thumb_opposition",
        "Move thumb inward toward the palm.",
    ),
    (
        "thumb_abduction",
        "Spread thumb outward/downward.",
    ),
    (
        "thumb_distal_negative",
        "Extend the distal thumb outward.",
    ),
    (
        "thumb_distal_positive",
        "Flex the distal thumb inward.",
    ),
    (
        "index_adduction",
        "Move the index finger laterally toward the thumb.",
    ),
    (
        "finger_flexion",
        "Flex index, middle and little fingers toward the palm.",
    ),
    (
        "finger_distal_negative",
        "Extend distal finger joints toward the back of the hand.",
    ),
    (
        "finger_distal_positive",
        "Flex distal finger joints toward the palm.",
    ),
    (
        "little_outward",
        "Move the little finger outward.",
    ),
]


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

    while (
        time.perf_counter() - start
        < duration
    ):
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
                raw = extractor.extract(
                    hand
                )

                session.add_sample(
                    pose_name,
                    raw,
                )

                valid += 1

            except (
                ValueError,
                np.linalg.LinAlgError,
            ):
                invalid += 1

        elapsed = (
            time.perf_counter()
            - loop_start
        )

        sleep_time = period - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)

    return valid, invalid


def print_pose_summary(
    session,
    pose_name,
):
    median = session.pose_median(
        pose_name
    )

    print(
        f"\nMedian raw features "
        f"for {pose_name}:"
    )

    for name, value in median.items():
        print(
            f"  {name:20s}: "
            f"{value:+.6f}"
        )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--ip",
        required=True,
        help="Apple Vision Pro IP address",
    )

    parser.add_argument(
        "--hand",
        choices=["right"],
        default="right",
        help=(
            "Currently validated SG100 morphology. "
            "Right hand only."
        ),
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=1.5,
        help=(
            "Sampling duration per pose "
            "(default: 1.5 s)"
        ),
    )

    parser.add_argument(
        "--hz",
        type=float,
        default=60.0,
        help=(
            "Tracking sampling rate "
            "(default: 60 Hz)"
        ),
    )

    parser.add_argument(
        "--countdown",
        type=int,
        default=2,
        help=(
            "Countdown before each pose "
            "(default: 2 s)"
        ),
    )

    parser.add_argument(
        "--output",
        default=(
            "sg100_teleop/config/"
            "avp_right_calibration.yaml"
        ),
    )

    parser.add_argument(
        "--verbose",
        action="store_true",
    )

    args = parser.parse_args()

    print(
        "Connecting to Vision Pro:",
        args.ip,
    )

    streamer = VisionProStreamer(
        ip=args.ip,
        record=False,
        verbose=args.verbose,
    )

    extractor = (
        AVPHandFeatureExtractor()
    )

    session = CalibrationSession()

    print()
    print(
        "===== SG100 / AVP HAND "
        "CALIBRATION ====="
    )
    print()
    print(
        "Each pose will be sampled "
        f"for {args.duration:.1f} seconds."
    )
    print(
        "Use comfortable, repeatable "
        "motion endpoints."
    )
    print(
        "Do NOT force the hand to its "
        "physiological limit."
    )

    for pose_name, instruction in (
        POSE_INSTRUCTIONS
    ):
        print()
        print("=" * 60)
        print(
            f"POSE: {pose_name}"
        )
        print(instruction)

        input(
            "\nPress ENTER when ready..."
        )

        for n in range(
            args.countdown,
            0,
            -1,
        ):
            print(
                f"Starting in {n}..."
            )
            time.sleep(1.0)

        print("SAMPLING...")

        valid, invalid = collect_pose(
            streamer=streamer,
            extractor=extractor,
            session=session,
            pose_name=pose_name,
            duration=args.duration,
            hz=args.hz,
            hand_side=args.hand,
        )

        print(
            f"valid frames   : {valid}"
        )
        print(
            f"invalid frames : {invalid}"
        )

        if valid < max(
            20,
            int(
                args.duration
                * args.hz
                * 0.5
            ),
        ):
            raise RuntimeError(
                "Too few valid tracking "
                f"frames for {pose_name}. "
                "Repeat calibration."
            )

        print_pose_summary(
            session,
            pose_name,
        )

    print()
    print(
        "===== BUILDING CALIBRATION ====="
    )

    calibration = session.build()

    output = Path(args.output)

    save_calibration(
        calibration,
        output,
        metadata={
            "device": "apple_vision_pro",
            "tracking_library":
                "avp_stream",
            "hand": args.hand,
            "sampling_hz": args.hz,
            "duration_per_pose_sec":
                args.duration,
            "samples_per_pose": {
                name:
                    session.sample_count(
                        name
                    )
                for name, _
                in POSE_INSTRUCTIONS
            },
        },
    )

    print()
    print(
        "Calibration saved to:"
    )
    print(output)

    print()
    print("===== CALIBRATION COMPLETE =====")


if __name__ == "__main__":
    # Local import here keeps error messages
    # clear if numpy is unavailable.
    import numpy as np

    main()
