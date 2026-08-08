from sg100_teleop.hand.hand_calibration import (
    HandCalibration,
    UnidirectionalCalibration,
    BidirectionalCalibration,
)
from sg100_teleop.tracking.avp_hand_adapter import (
    AVPRawHandFeatures,
)


def build_calibration():
    return HandCalibration(
        unidirectional={
            "thumb_opposition":
                UnidirectionalCalibration(
                    neutral=1.40,
                    active=0.40,
                ),

            "thumb_abduction":
                UnidirectionalCalibration(
                    neutral=-0.20,
                    active=0.60,
                ),

            "index_adduction":
                UnidirectionalCalibration(
                    neutral=0.0,
                    active=-0.5,
                ),

            "index_flexion":
                UnidirectionalCalibration(
                    neutral=0.0,
                    active=1.4,
                ),

            "middle_flexion":
                UnidirectionalCalibration(
                    neutral=0.0,
                    active=1.5,
                ),

            "little_outward":
                UnidirectionalCalibration(
                    neutral=0.0,
                    active=0.6,
                ),

            "little_flexion":
                UnidirectionalCalibration(
                    neutral=0.0,
                    active=1.5,
                ),
        },

        bidirectional={
            "thumb_distal":
                BidirectionalCalibration(
                    negative=-0.8,
                    neutral=0.0,
                    positive=1.0,
                ),

            "index_distal":
                BidirectionalCalibration(
                    negative=-1.0,
                    neutral=0.0,
                    positive=1.2,
                ),

            "middle_distal":
                BidirectionalCalibration(
                    negative=-1.0,
                    neutral=0.0,
                    positive=1.2,
                ),

            "little_distal":
                BidirectionalCalibration(
                    negative=-1.0,
                    neutral=0.0,
                    positive=1.2,
                ),
        },
    )


def main():
    calibration = build_calibration()

    neutral_raw = AVPRawHandFeatures(
        thumb_opposition=1.40,
        thumb_abduction=-0.20,
        thumb_distal=0.0,

        index_adduction=0.0,
        index_flexion=0.0,
        index_distal=0.0,

        middle_flexion=0.0,
        middle_distal=0.0,

        little_outward=0.0,
        little_flexion=0.0,
        little_distal=0.0,
    )

    neutral = calibration.normalize(
        neutral_raw
    )

    print("===== NEUTRAL =====")
    print(neutral)

    for value in neutral.as_dict().values():
        assert abs(value) < 1e-8

    active_raw = AVPRawHandFeatures(
        thumb_opposition=0.40,
        thumb_abduction=0.60,
        thumb_distal=1.0,

        index_adduction=-0.50,
        index_flexion=1.40,
        index_distal=1.20,

        middle_flexion=1.50,
        middle_distal=1.20,

        little_outward=0.60,
        little_flexion=1.50,
        little_distal=1.20,
    )

    active = calibration.normalize(
        active_raw
    )

    print()
    print("===== ACTIVE =====")
    print(active)

    assert abs(
        active.thumb_opposition - 1.0
    ) < 1e-8

    assert abs(
        active.index_flexion - 1.0
    ) < 1e-8

    assert abs(
        active.index_distal - 1.0
    ) < 1e-8

    negative_raw = AVPRawHandFeatures(
        thumb_opposition=1.40,
        thumb_abduction=-0.20,
        thumb_distal=-0.80,

        index_adduction=0.0,
        index_flexion=0.0,
        index_distal=-1.0,

        middle_flexion=0.0,
        middle_distal=-1.0,

        little_outward=0.0,
        little_flexion=0.0,
        little_distal=-1.0,
    )

    negative = calibration.normalize(
        negative_raw
    )

    print()
    print("===== NEGATIVE DISTAL =====")
    print(negative)

    assert abs(
        negative.thumb_distal + 1.0
    ) < 1e-8

    assert abs(
        negative.index_distal + 1.0
    ) < 1e-8

    print()
    print("PASS")
    print(
        "Raw features -> CanonicalHandState "
        "normalization works."
    )


if __name__ == "__main__":
    main()
