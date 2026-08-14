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
REQUIRED_POSES = (
    "neutral",
    "thumb_opposition",
    "thumb_abduction",
    "thumb_distal_negative",
    "thumb_distal_positive",
    "index_adduction",
    "finger_flexion",
    "finger_distal_negative",
    "finger_distal_positive",
    "little_outward",
)


# Raw bidirectional features are angles in radians.  About 1.1 degrees
# is large enough to reject endpoints that are effectively neutral at
# AVP tracking noise scale, while remaining conservative for users with
# a limited comfortable range of motion.
MIN_BIDIRECTIONAL_SEPARATION_RAD = 0.02


BIDIRECTIONAL_POSE_SOURCES = {
    "thumb_distal": (
        "thumb_distal_negative",
        "thumb_distal_positive",
    ),
    "index_distal": (
        "finger_distal_negative",
        "finger_distal_positive",
    ),
    "middle_distal": (
        "finger_distal_negative",
        "finger_distal_positive",
    ),
    "little_distal": (
        "finger_distal_negative",
        "finger_distal_positive",
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

    def bidirectional_diagnostics(
        self,
        minimum_separation: float = (
            MIN_BIDIRECTIONAL_SEPARATION_RAD
        ),
    ) -> dict[str, BidirectionalEndpointDiagnostic]:
        """Validate raw bidirectional endpoint geometry by feature."""

        if minimum_separation <= 0.0:
            raise ValueError(
                "minimum_separation must be positive"
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
            negative_delta = negative - neutral_value
            positive_delta = positive - neutral_value
            product = negative_delta * positive_delta
            reasons = []

            if abs(negative_delta) < minimum_separation:
                reasons.append("negative_too_close_to_neutral")

            if abs(positive_delta) < minimum_separation:
                reasons.append("positive_too_close_to_neutral")

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
                    f"neg_delta={item.negative_delta:+.6f}, "
                    f"pos_delta={item.positive_delta:+.6f}, "
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

        index_adduction = self.pose_median(
            "index_adduction"
        )

        finger_flexion = self.pose_median(
            "finger_flexion"
        )

        finger_distal_negative = (
            self.pose_median(
                "finger_distal_negative"
            )
        )

        finger_distal_positive = (
            self.pose_median(
                "finger_distal_positive"
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

                "index_adduction":
                    UnidirectionalCalibration(
                        neutral=neutral[
                            "index_adduction"
                        ],
                        active=index_adduction[
                            "index_adduction"
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

                "index_distal":
                    BidirectionalCalibration(
                        negative=(
                            finger_distal_negative[
                                "index_distal"
                            ]
                        ),
                        neutral=neutral[
                            "index_distal"
                        ],
                        positive=(
                            finger_distal_positive[
                                "index_distal"
                            ]
                        ),
                    ),

                "middle_distal":
                    BidirectionalCalibration(
                        negative=(
                            finger_distal_negative[
                                "middle_distal"
                            ]
                        ),
                        neutral=neutral[
                            "middle_distal"
                        ],
                        positive=(
                            finger_distal_positive[
                                "middle_distal"
                            ]
                        ),
                    ),

                "little_distal":
                    BidirectionalCalibration(
                        negative=(
                            finger_distal_negative[
                                "little_distal"
                            ]
                        ),
                        neutral=neutral[
                            "little_distal"
                        ],
                        positive=(
                            finger_distal_positive[
                                "little_distal"
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
