from sg100_teleop.hand.canonical_hand import (
    CanonicalHandState,
)
from sg100_teleop.hand.hand_calibration import (
    HandCalibration,
)
from sg100_teleop.tracking.avp_hand_adapter import (
    AVPHandFeatureExtractor,
    AVPRawHandFeatures,
)


class AVPHandAdapter:
    """
    VisionProTeleop HandData
        -> AVP raw geometric features
        -> calibrated CanonicalHandState
    """

    def __init__(
        self,
        calibration: HandCalibration,
    ):
        self.extractor = (
            AVPHandFeatureExtractor()
        )

        self.calibration = calibration

    def extract_raw(
        self,
        hand,
    ) -> AVPRawHandFeatures:
        return self.extractor.extract(hand)

    def convert(
        self,
        hand,
    ) -> CanonicalHandState:

        raw = self.extract_raw(hand)

        return self.calibration.normalize(
            raw
        )
