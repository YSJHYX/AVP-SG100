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

            "index_flexion":
                UnidirectionalCalibration(
                    neutral=0.0,
                    active=1.4,
                ),

            # Unidirectional distal flexion: 0 = straight baseline,
            # 1 = distal flexed toward palm.
            "index_distal":
                UnidirectionalCalibration(
                    neutral=0.0,
                    active=1.2,
                ),

            "middle_distal":
                UnidirectionalCalibration(
                    neutral=0.0,
                    active=1.2,
                ),

            "little_distal":
                UnidirectionalCalibration(
                    neutral=0.0,
                    active=1.2,
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

            # negative endpoint = toward thumb, positive = toward middle
            "index_adduction":
                BidirectionalCalibration(
                    negative=-0.5,
                    neutral=0.0,
                    positive=0.5,
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

    # active_raw carries index_adduction=-0.50, the TOWARD THUMB
    # endpoint, so it must normalize to canonical -1 (not +1).
    assert abs(
        active.index_adduction + 1.0
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

    # Distal features are unidirectional: raw below the straight
    # baseline clips to canonical 0, never to a negative endpoint.
    assert abs(negative.index_distal) < 1e-8
    assert abs(negative.middle_distal) < 1e-8
    assert abs(negative.little_distal) < 1e-8

    # Toward-middle raw value normalizes to canonical +1.
    toward_middle = calibration.normalize(
        AVPRawHandFeatures(
            thumb_opposition=1.40,
            thumb_abduction=-0.20,
            thumb_distal=0.0,
            index_adduction=0.50,
            index_flexion=0.0,
            index_distal=0.0,
            middle_flexion=0.0,
            middle_distal=0.0,
            little_outward=0.0,
            little_flexion=0.0,
            little_distal=0.0,
        )
    )
    assert abs(
        toward_middle.index_adduction - 1.0
    ) < 1e-8

    # Semantic negative/positive labels, rather than raw numerical
    # signs, define the normalized output direction.
    reversed_raw_signs = BidirectionalCalibration(
        negative=0.5,
        neutral=0.0,
        positive=-0.8,
    )
    assert abs(
        reversed_raw_signs.normalize(0.5) + 1.0
    ) < 1e-8
    assert abs(
        reversed_raw_signs.normalize(-0.8) - 1.0
    ) < 1e-8
    assert abs(
        reversed_raw_signs.normalize(0.0)
    ) < 1e-8

    try:
        BidirectionalCalibration(
            negative=0.4,
            neutral=0.0,
            positive=0.9,
        )
    except ValueError as exc:
        message = str(exc)
        assert "opposite sides" in message
        assert "product=" in message
        assert "neg_delta=" in message
    else:
        raise AssertionError(
            "Same-side endpoints must remain a hard invariant"
        )

    print()
    print("PASS")
    print(
        "Raw features -> CanonicalHandState "
        "normalization and bidirectional semantics work."
    )


if __name__ == "__main__":
    main()
