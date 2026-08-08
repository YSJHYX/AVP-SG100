from collections import defaultdict

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

    def build(self) -> HandCalibration:
        missing = self.missing_poses()

        if missing:
            raise ValueError(
                "Cannot build calibration. "
                f"Missing poses: {missing}"
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
