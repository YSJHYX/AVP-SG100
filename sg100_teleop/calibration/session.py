from collections import defaultdict
from dataclasses import dataclass

import numpy as np

from sg100_teleop.hand.hand_calibration import (
    HandCalibration,
    UnidirectionalCalibration,
    BidirectionalCalibration,
)


# 10 semantic poses.
#
# Each pose can contain many frames.
# The median raw feature value becomes the calibration endpoint.
#
# index/middle/little distal calibration is UNIDIRECTIONAL: human DIP
# joints are not operable in reverse, so only a straight/open baseline
# (neutral) and a distal-flexion endpoint are captured.
REQUIRED_POSES = (
    "neutral",
    "thumb_opposition",
    "thumb_abduction",
    "thumb_distal_negative",
    "thumb_distal_positive",
    "index_adduction_negative",
    "index_adduction_positive",
    "finger_flexion",
    "finger_distal_flexion",
    "little_outward",
)


# Raw bidirectional features are angles in radians.  About 1.1 degrees
# is large enough to reject endpoints that are effectively neutral at
# AVP tracking noise scale, while remaining conservative for users with
# a limited comfortable range of motion.
MIN_BIDIRECTIONAL_SEPARATION_RAD = 0.02

# Robust noise scale: median absolute deviation converted to an
# equivalent Gaussian sigma.
ROBUST_SIGMA_MAD_SCALE = 1.4826

# An endpoint median must separate from the neutral median by at least
# this multiple of the larger robust sigma of the two poses.  This
# rejects mathematically unstable endpoints such as a thumb_distal
# negative span of ~0.07 rad recorded with comparable hand jitter.
MIN_SPAN_TO_NOISE_RATIO = 5.0


BIDIRECTIONAL_POSE_SOURCES = {
    "thumb_distal": (
        "thumb_distal_negative",
        "thumb_distal_positive",
    ),
    # Human anatomy semantics:
    #   negative pose = index toward THUMB
    #   positive pose = index toward MIDDLE
    "index_adduction": (
        "index_adduction_negative",
        "index_adduction_positive",
    ),
}


@dataclass(frozen=True)
class BidirectionalEndpointDiagnostic:
    feature_name: str
    negative: float
    neutral: float
    positive: float
    negative_delta: float
    positive_delta: float
    product: float
    failure_reasons: tuple[str, ...]
    neutral_sigma: float = 0.0
    negative_sigma: float = 0.0
    positive_sigma: float = 0.0

    @property
    def passed(self) -> bool:
        return not self.failure_reasons

    @property
    def status(self) -> str:
        return "PASS" if self.passed else "FAIL"


class CalibrationSession:
    """
    Collect AVPRawHandFeatures over multiple frames and
    build a robust HandCalibration from pose medians.
    """

    def __init__(self):
        self._samples = defaultdict(list)

    def add_sample(
        self,
        pose_name: str,
        raw_features,
    ) -> None:

        if pose_name not in REQUIRED_POSES:
            raise ValueError(
                f"Unknown calibration pose: {pose_name}"
            )

        values = raw_features.as_dict()

        if not all(
            np.isfinite(v)
            for v in values.values()
        ):
            raise ValueError(
                "Calibration sample contains NaN/Inf"
            )

        self._samples[pose_name].append(
            {
                k: float(v)
                for k, v in values.items()
            }
        )

    def sample_count(
        self,
        pose_name: str,
    ) -> int:
        return len(
            self._samples.get(
                pose_name,
                [],
            )
        )

    def clear_pose(
        self,
        pose_name: str,
    ) -> None:
        """Discard a pose so a retry replaces, rather than mixes, samples."""

        if pose_name not in REQUIRED_POSES:
            raise ValueError(
                f"Unknown calibration pose: {pose_name}"
            )

        self._samples[pose_name].clear()

    def missing_poses(self) -> list[str]:
        return [
            pose
            for pose in REQUIRED_POSES
            if self.sample_count(pose) == 0
        ]

    def pose_median(
        self,
        pose_name: str,
    ) -> dict[str, float]:

        samples = self._samples.get(
            pose_name,
            [],
        )

        if not samples:
            raise ValueError(
                f"No samples for pose: {pose_name}"
            )

        feature_names = list(
            samples[0].keys()
        )

        result = {}

        for feature_name in feature_names:
            values = np.asarray(
                [
                    sample[feature_name]
                    for sample in samples
                ],
                dtype=np.float64,
            )

            result[feature_name] = float(
                np.median(values)
            )

        return result

    def pose_robust_sigma(
        self,
        pose_name: str,
        feature_name: str,
    ) -> float:
        """MAD-based robust noise scale for one pose/feature."""

        samples = self._samples.get(
            pose_name,
            [],
        )

        if not samples:
            raise ValueError(
                f"No samples for pose: {pose_name}"
            )

        values = np.asarray(
            [
                sample[feature_name]
                for sample in samples
            ],
            dtype=np.float64,
        )

        mad = float(
            np.median(
                np.abs(
                    values
                    - np.median(values)
                )
            )
        )

        return ROBUST_SIGMA_MAD_SCALE * mad

    def bidirectional_diagnostics(
        self,
        minimum_separation: float = (
            MIN_BIDIRECTIONAL_SEPARATION_RAD
        ),
        min_span_to_noise_ratio: float = (
            MIN_SPAN_TO_NOISE_RATIO
        ),
    ) -> dict[str, BidirectionalEndpointDiagnostic]:
        """Validate raw bidirectional endpoint geometry by feature.

        An endpoint is rejected unless its span from neutral exceeds
        BOTH the absolute minimum separation AND a conservative
        multiple of the robust (MAD-derived) noise scale of the two
        poses involved.  This rejects mathematically unstable
        endpoints, e.g. a thumb_distal negative span of ~0.07 rad
        recorded with comparable hand jitter.
        """

        if minimum_separation <= 0.0:
            raise ValueError(
                "minimum_separation must be positive"
            )

        if min_span_to_noise_ratio <= 0.0:
            raise ValueError(
                "min_span_to_noise_ratio must be positive"
            )

        missing = self.missing_poses()

        if missing:
            raise ValueError(
                "Cannot diagnose calibration. "
                f"Missing poses: {missing}"
            )

        neutral = self.pose_median("neutral")
        diagnostics = {}

        for feature_name, (
            negative_pose,
            positive_pose,
        ) in BIDIRECTIONAL_POSE_SOURCES.items():
            negative = self.pose_median(
                negative_pose
            )[feature_name]
            neutral_value = neutral[feature_name]
            positive = self.pose_median(
                positive_pose
            )[feature_name]

            neutral_sigma = self.pose_robust_sigma(
                "neutral",
                feature_name,
            )
            negative_sigma = self.pose_robust_sigma(
                negative_pose,
                feature_name,
            )
            positive_sigma = self.pose_robust_sigma(
                positive_pose,
                feature_name,
            )

            negative_delta = negative - neutral_value
            positive_delta = positive - neutral_value
            product = negative_delta * positive_delta
            reasons = []

            if abs(negative_delta) < minimum_separation:
                reasons.append("negative_too_close_to_neutral")

            if abs(positive_delta) < minimum_separation:
                reasons.append("positive_too_close_to_neutral")

            negative_noise = max(
                neutral_sigma,
                negative_sigma,
            )

            if abs(negative_delta) < (
                min_span_to_noise_ratio
                * negative_noise
            ):
                reasons.append(
                    "negative_span_below_noise_scale"
                )

            positive_noise = max(
                neutral_sigma,
                positive_sigma,
            )

            if abs(positive_delta) < (
                min_span_to_noise_ratio
                * positive_noise
            ):
                reasons.append(
                    "positive_span_below_noise_scale"
                )

            if product >= 0.0:
                reasons.append("endpoints_on_same_side")

            diagnostics[feature_name] = (
                BidirectionalEndpointDiagnostic(
                    feature_name=feature_name,
                    negative=negative,
                    neutral=neutral_value,
                    positive=positive,
                    negative_delta=negative_delta,
                    positive_delta=positive_delta,
                    product=product,
                    failure_reasons=tuple(reasons),
                    neutral_sigma=neutral_sigma,
                    negative_sigma=negative_sigma,
                    positive_sigma=positive_sigma,
                )
            )

        return diagnostics

    def build(self) -> HandCalibration:
        missing = self.missing_poses()

        if missing:
            raise ValueError(
                "Cannot build calibration. "
                f"Missing poses: {missing}"
            )

        diagnostics = self.bidirectional_diagnostics()
        failures = [
            diagnostic
            for diagnostic in diagnostics.values()
            if not diagnostic.passed
        ]

        if failures:
            details = "; ".join(
                (
                    f"{item.feature_name}: "
                    f"negative={item.negative:+.6f}, "
                    f"neutral={item.neutral:+.6f}, "
                    f"positive={item.positive:+.6f}, "
                    f"neg_delta={item.negative_delta:+.6f} "
                    f"(span), "
                    f"pos_delta={item.positive_delta:+.6f} "
                    f"(span), "
                    f"neutral_sigma={item.neutral_sigma:.6f}, "
                    f"negative_sigma={item.negative_sigma:.6f}, "
                    f"positive_sigma={item.positive_sigma:.6f}, "
                    f"required_span>=noise*"
                    f"{MIN_SPAN_TO_NOISE_RATIO}, "
                    f"product={item.product:+.6f}, "
                    f"reasons={list(item.failure_reasons)}"
                )
                for item in failures
            )
            raise ValueError(
                "Invalid bidirectional calibration endpoints: "
                + details
            )

        neutral = self.pose_median(
            "neutral"
        )

        thumb_opposition = self.pose_median(
            "thumb_opposition"
        )

        thumb_abduction = self.pose_median(
            "thumb_abduction"
        )

        thumb_distal_negative = (
            self.pose_median(
                "thumb_distal_negative"
            )
        )

        thumb_distal_positive = (
            self.pose_median(
                "thumb_distal_positive"
            )
        )

        index_adduction_negative = (
            self.pose_median(
                "index_adduction_negative"
            )
        )

        index_adduction_positive = (
            self.pose_median(
                "index_adduction_positive"
            )
        )

        finger_flexion = self.pose_median(
            "finger_flexion"
        )

        finger_distal_flexion = (
            self.pose_median(
                "finger_distal_flexion"
            )
        )

        little_outward = self.pose_median(
            "little_outward"
        )

        return HandCalibration(
            unidirectional={
                "thumb_opposition":
                    UnidirectionalCalibration(
                        neutral=neutral[
                            "thumb_opposition"
                        ],
                        active=thumb_opposition[
                            "thumb_opposition"
                        ],
                    ),

                "thumb_abduction":
                    UnidirectionalCalibration(
                        neutral=neutral[
                            "thumb_abduction"
                        ],
                        active=thumb_abduction[
                            "thumb_abduction"
                        ],
                    ),

                "index_flexion":
                    UnidirectionalCalibration(
                        neutral=neutral[
                            "index_flexion"
                        ],
                        active=finger_flexion[
                            "index_flexion"
                        ],
                    ),

                "middle_flexion":
                    UnidirectionalCalibration(
                        neutral=neutral[
                            "middle_flexion"
                        ],
                        active=finger_flexion[
                            "middle_flexion"
                        ],
                    ),

                "little_outward":
                    UnidirectionalCalibration(
                        neutral=neutral[
                            "little_outward"
                        ],
                        active=little_outward[
                            "little_outward"
                        ],
                    ),

                "little_flexion":
                    UnidirectionalCalibration(
                        neutral=neutral[
                            "little_flexion"
                        ],
                        active=finger_flexion[
                            "little_flexion"
                        ],
                    ),

                # Unidirectional distal flexion: neutral = straight/open
                # baseline, active = distal flexed toward palm.
                "index_distal":
                    UnidirectionalCalibration(
                        neutral=neutral[
                            "index_distal"
                        ],
                        active=finger_distal_flexion[
                            "index_distal"
                        ],
                    ),

                "middle_distal":
                    UnidirectionalCalibration(
                        neutral=neutral[
                            "middle_distal"
                        ],
                        active=finger_distal_flexion[
                            "middle_distal"
                        ],
                    ),

                "little_distal":
                    UnidirectionalCalibration(
                        neutral=neutral[
                            "little_distal"
                        ],
                        active=finger_distal_flexion[
                            "little_distal"
                        ],
                    ),
            },

            bidirectional={
                "thumb_distal":
                    BidirectionalCalibration(
                        negative=(
                            thumb_distal_negative[
                                "thumb_distal"
                            ]
                        ),
                        neutral=neutral[
                            "thumb_distal"
                        ],
                        positive=(
                            thumb_distal_positive[
                                "thumb_distal"
                            ]
                        ),
                    ),

                # negative endpoint = index toward THUMB,
                # positive endpoint = index toward MIDDLE.
                "index_adduction":
                    BidirectionalCalibration(
                        negative=(
                            index_adduction_negative[
                                "index_adduction"
                            ]
                        ),
                        neutral=neutral[
                            "index_adduction"
                        ],
                        positive=(
                            index_adduction_positive[
                                "index_adduction"
                            ]
                        ),
                    ),
            },
        )

    def summary(self) -> dict:
        result = {}

        for pose in REQUIRED_POSES:
            result[pose] = {
                "samples": self.sample_count(
                    pose
                ),
            }

            if self.sample_count(pose) > 0:
                result[pose]["median"] = (
                    self.pose_median(pose)
                )

        return result
