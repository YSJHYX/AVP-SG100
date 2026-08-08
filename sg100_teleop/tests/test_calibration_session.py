from pathlib import Path
import tempfile

import numpy as np

from sg100_teleop.calibration.session import (
    CalibrationSession,
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

    print()
    print("PASS")
    print(
        "Multi-frame median calibration, "
        "outlier robustness and YAML "
        "roundtrip all work."
    )


if __name__ == "__main__":
    main()
