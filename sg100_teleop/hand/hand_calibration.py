from dataclasses import dataclass
from typing import Mapping

import numpy as np

from sg100_teleop.hand.canonical_hand import CanonicalHandState


UNIDIRECTIONAL_FEATURES = (
    "thumb_opposition",
    "thumb_abduction",

    "index_flexion",

    "middle_flexion",

    "little_outward",
    "little_flexion",

    # Finger distal flexion is unidirectional: human DIP joints are not
    # operable in reverse (no hyperextension), so calibration captures
    #   neutral = straight/open baseline
    #   active  = distal flexed toward palm
    "index_distal",
    "middle_distal",
    "little_distal",
)


BIDIRECTIONAL_FEATURES = (
    "thumb_distal",

    # Human anatomy semantics:
    #   negative endpoint = toward thumb
    #   neutral           = relaxed lateral neutral
    #   positive endpoint = toward middle
    "index_adduction",
)


_EPS = 1e-8


@dataclass(frozen=True)
class UnidirectionalCalibration:
    """
    neutral -> canonical 0
    active  -> canonical 1

    active can be numerically either larger or smaller
    than neutral.
    """

    neutral: float
    active: float

    def normalize(self, value: float) -> float:
        denom = self.active - self.neutral

        if abs(denom) < _EPS:
            raise ValueError(
                "Invalid unidirectional calibration: "
                "neutral and active are identical"
            )

        x = (
            (float(value) - self.neutral)
            / denom
        )

        return float(
            np.clip(x, 0.0, 1.0)
        )


@dataclass(frozen=True)
class BidirectionalCalibration:
    """
    negative -> canonical -1
    neutral  -> canonical  0
    positive -> canonical +1

    Raw numerical signs do not matter.

    Example:
        raw negative endpoint may actually be +0.5 rad,
        raw positive endpoint may be -0.8 rad.

    What matters is semantic labeling during calibration.
    """

    negative: float
    neutral: float
    positive: float

    def __post_init__(self):
        neg_delta = self.negative - self.neutral
        pos_delta = self.positive - self.neutral

        if abs(neg_delta) < _EPS:
            raise ValueError(
                "negative endpoint equals neutral"
            )

        if abs(pos_delta) < _EPS:
            raise ValueError(
                "positive endpoint equals neutral"
            )

        # For a scalar feature the two semantic directions
        # should normally lie on opposite sides of neutral.
        if neg_delta * pos_delta >= 0:
            raise ValueError(
                "Bidirectional raw endpoints must lie on "
                "opposite sides of neutral. "
                f"negative={self.negative:+.6f}, "
                f"neutral={self.neutral:+.6f}, "
                f"positive={self.positive:+.6f}, "
                f"neg_delta={neg_delta:+.6f}, "
                f"pos_delta={pos_delta:+.6f}, "
                f"product={neg_delta * pos_delta:+.6f}."
            )

    def normalize(self, value: float) -> float:
        value = float(value)

        neg_delta = self.negative - self.neutral
        pos_delta = self.positive - self.neutral

        raw_delta = value - self.neutral

        # Determine semantic branch using raw direction.
        if raw_delta * pos_delta >= 0:
            x = raw_delta / pos_delta

            return float(
                np.clip(x, 0.0, 1.0)
            )

        x = raw_delta / neg_delta

        return -float(
            np.clip(x, 0.0, 1.0)
        )


class HandCalibration:
    """
    Device-independent normalization from raw semantic
    features to CanonicalHandState.
    """

    def __init__(
        self,
        unidirectional: Mapping[
            str,
            UnidirectionalCalibration,
        ],
        bidirectional: Mapping[
            str,
            BidirectionalCalibration,
        ],
    ):
        self.unidirectional = dict(
            unidirectional
        )

        self.bidirectional = dict(
            bidirectional
        )

        missing_uni = (
            set(UNIDIRECTIONAL_FEATURES)
            - set(self.unidirectional)
        )

        missing_bi = (
            set(BIDIRECTIONAL_FEATURES)
            - set(self.bidirectional)
        )

        if missing_uni:
            raise ValueError(
                f"Missing unidirectional calibration: "
                f"{sorted(missing_uni)}"
            )

        if missing_bi:
            raise ValueError(
                f"Missing bidirectional calibration: "
                f"{sorted(missing_bi)}"
            )

    def normalize(
        self,
        raw_features,
    ) -> CanonicalHandState:

        raw = raw_features.as_dict()

        values = {}

        for name in UNIDIRECTIONAL_FEATURES:
            values[name] = (
                self.unidirectional[
                    name
                ].normalize(
                    raw[name]
                )
            )

        for name in BIDIRECTIONAL_FEATURES:
            values[name] = (
                self.bidirectional[
                    name
                ].normalize(
                    raw[name]
                )
            )

        return CanonicalHandState(
            **values
        ).clipped()
