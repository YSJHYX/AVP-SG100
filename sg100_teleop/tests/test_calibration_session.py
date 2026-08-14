from pathlib import Path
import tempfile

import numpy as np

from sg100_teleop.calibration.session import (
    CalibrationSession,
    MIN_BIDIRECTIONAL_SEPARATION_RAD,
    REQUIRED_POSES,
)
from sg100_teleop.calibration.calibrate_avp_hand import (
    POSE_INSTRUCTIONS,
    retry_pose_names,
)
from sg100_teleop.calibration.io import (
    save_calibration,
    load_calibration,
)
from sg100_teleop.tracking.avp_hand_adapter import (
    AVPRawHandFeatures,
)


BASE = {
    "thumb_opposition": 1.40,
    "thumb_abduction": -0.20,
    "thumb_distal": 0.0,

    "index_adduction": 0.0,
    "index_flexion": 0.0,
    "index_distal": 0.0,

    "middle_flexion": 0.0,
    "middle_distal": 0.0,

    "little_outward": 0.0,
    "little_flexion": 0.0,
    "little_distal": 0.0,
}


POSE_VALUES = {
    "neutral": {},

    "thumb_opposition": {
        "thumb_opposition": 0.40,
    },

    "thumb_abduction": {
        "thumb_abduction": 0.60,
    },

    "thumb_distal_negative": {
        "thumb_distal": -0.80,
    },

    "thumb_distal_positive": {
        "thumb_distal": 1.00,
    },

    "index_adduction": {
        "index_adduction": -0.50,
    },

    "finger_flexion": {
        "index_flexion": 1.40,
        "middle_flexion": 1.50,
        "little_flexion": 1.50,
    },

    "finger_distal_negative": {
        "index_distal": -1.00,
        "middle_distal": -1.00,
        "little_distal": -1.00,
    },

    "finger_distal_positive": {
        "index_distal": 1.20,
        "middle_distal": 1.20,
        "little_distal": 1.20,
    },

    "little_outward": {
        "little_outward": 0.60,
    },
}


def make_raw(
    pose_name,
    noise=None,
):
    values = dict(BASE)

    values.update(
        POSE_VALUES[pose_name]
    )

    if noise is not None:
        for key in values:
            values[key] += float(
                noise.get(key, 0.0)
            )

    return AVPRawHandFeatures(
        **values
    )


def populate_once(
    session,
    pose_noise=None,
):
    pose_noise = pose_noise or {}

    for pose_name in POSE_VALUES:
        session.add_sample(
            pose_name,
            make_raw(
                pose_name,
                noise=pose_noise.get(pose_name),
            ),
        )


def main():
    rng = np.random.default_rng(42)

    session = CalibrationSession()

    # 101 samples per pose.
    # Median should remain robust.
    for pose_name in POSE_VALUES:

        for _ in range(100):

            noise = {
                key: rng.normal(
                    0.0,
                    0.01,
                )
                for key in BASE
            }

            session.add_sample(
                pose_name,
                make_raw(
                    pose_name,
                    noise=noise,
                ),
            )

        # Add one extreme outlier.
        outlier_noise = {
            key: 100.0
            for key in BASE
        }

        session.add_sample(
            pose_name,
            make_raw(
                pose_name,
                noise=outlier_noise,
            ),
        )

    print(
        "missing poses =",
        session.missing_poses(),
    )

    assert not session.missing_poses()

    calibration = session.build()

    # Neutral should map close to zero.
    neutral = calibration.normalize(
        make_raw("neutral")
    )

    print()
    print("===== NEUTRAL =====")
    print(neutral)

    assert (
        abs(neutral.index_flexion)
        < 0.05
    )

    # Active flexion should map close to one.
    active = calibration.normalize(
        make_raw("finger_flexion")
    )

    print()
    print("===== FLEXION =====")
    print(active)

    assert (
        abs(
            active.index_flexion
            - 1.0
        )
        < 0.05
    )

    assert (
        abs(
            active.middle_flexion
            - 1.0
        )
        < 0.05
    )

    # Test YAML roundtrip.
    with tempfile.TemporaryDirectory() as d:

        path = (
            Path(d)
            / "calibration.yaml"
        )

        save_calibration(
            calibration,
            path,
            metadata={
                "device": "synthetic_avp",
                "hand": "right",
            },
        )

        loaded = load_calibration(
            path
        )

        before = calibration.normalize(
            make_raw(
                "finger_distal_negative"
            )
        )

        after = loaded.normalize(
            make_raw(
                "finger_distal_negative"
            )
        )

        print()
        print("===== ROUNDTRIP =====")
        print("before =", before)
        print("after  =", after)

        assert (
            before.as_dict()
            == after.as_dict()
        )

    # Structured diagnostics must identify an endpoint that is too
    # close to neutral independently from a same-side failure.
    too_small_session = CalibrationSession()
    populate_once(
        too_small_session,
        pose_noise={
            "finger_distal_negative": {
                "index_distal": 1.0 - (
                    MIN_BIDIRECTIONAL_SEPARATION_RAD / 2.0
                ),
            },
        },
    )
    too_small = too_small_session.bidirectional_diagnostics()[
        "index_distal"
    ]
    assert not too_small.passed
    assert (
        "negative_too_close_to_neutral"
        in too_small.failure_reasons
    )

    same_side_session = CalibrationSession()
    populate_once(
        same_side_session,
        pose_noise={
            "neutral": {
                "index_distal": -2.0,
            },
        },
    )
    same_side = same_side_session.bidirectional_diagnostics()[
        "index_distal"
    ]
    assert not same_side.passed
    assert "endpoints_on_same_side" in same_side.failure_reasons
    assert same_side.product > 0.0

    try:
        same_side_session.build()
    except ValueError as exc:
        message = str(exc)
        assert "index_distal" in message
        assert "product=" in message
        assert "endpoints_on_same_side" in message
    else:
        raise AssertionError(
            "Same-side bidirectional endpoints were not rejected"
        )

    # Retry selection is minimal, stable, and always includes a fresh
    # neutral sample.  Replacing a pose must discard the old attempt.
    assert retry_pose_names(("thumb_distal",)) == (
        "neutral",
        "thumb_distal_negative",
        "thumb_distal_positive",
    )
    assert retry_pose_names(("middle_distal",)) == (
        "neutral",
        "finger_distal_negative",
        "finger_distal_positive",
    )
    assert retry_pose_names(
        ("thumb_distal", "little_distal")
    ) == (
        "neutral",
        "thumb_distal_negative",
        "thumb_distal_positive",
        "finger_distal_negative",
        "finger_distal_positive",
    )

    replace_session = CalibrationSession()
    replace_session.add_sample("neutral", make_raw("neutral"))
    replace_session.add_sample("neutral", make_raw("neutral"))
    replace_session.clear_pose("neutral")
    assert replace_session.sample_count("neutral") == 0
    replace_session.add_sample("neutral", make_raw("neutral"))
    assert replace_session.sample_count("neutral") == 1

    # Prompt keys/order are an external workflow contract.  Every
    # instruction is Chinese and neutral explicitly permits slight bend.
    assert tuple(name for name, _ in POSE_INSTRUCTIONS) == REQUIRED_POSES
    assert "轻微弯曲" in dict(POSE_INSTRUCTIONS)["neutral"]
    assert "不要刻意完全伸直" in dict(POSE_INSTRUCTIONS)["neutral"]
    assert all(
        any("\u4e00" <= char <= "\u9fff" for char in instruction)
        for _, instruction in POSE_INSTRUCTIONS
    )
    prompt_text = "\n".join(
        instruction
        for _, instruction in POSE_INSTRUCTIONS
    )
    assert "Natural open hand" not in prompt_text
    assert "Move thumb inward" not in prompt_text
    assert "Extend distal finger joints" not in prompt_text

    print()
    print("PASS")
    print(
        "Multi-frame median calibration, "
        "endpoint diagnostics, retry selection, "
        "prompt contract and YAML roundtrip all work."
    )


if __name__ == "__main__":
    main()
