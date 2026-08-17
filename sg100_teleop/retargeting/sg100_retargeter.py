from pathlib import Path

import numpy as np
import yaml

from sg100_teleop.hand.canonical_hand import CanonicalHandState


ROOT = Path(__file__).resolve().parents[2]

DEFAULT_SEMANTICS_PATH = (
    ROOT
    / "sg100_teleop"
    / "config"
    / "sg100_joint_semantics.yaml"
)


class SG100Retargeter:
    """
    Map CanonicalHandState to SG100 absolute joint-position targets.

    Output convention:
        q_target.shape == (11,)
        unit == radians
        representation == absolute_joint_position

    Two distinct "neutral" concepts (do not conflate them):

        HUMAN FEATURE NEUTRAL -> canonical value 0
        ROBOT JOINT NEUTRAL   -> q = semantics ``zero`` per joint

    Canonical 0 for a feature always commands q == semantics zero for the
    mapped joint.  The zero comes from the SG100 semantics config (a robot
    morphology property), never from the per-user calibration YAML.

    Mapping modes (all neutral-centered on semantics zero):

        positive:
            canonical [0, 1] -> q from zero toward the numeric upper limit

        negative:
            canonical [0, 1] -> q from zero toward the numeric lower limit

        bidirectional:
            canonical [-1, 1] -> q from zero toward upper (+) / lower (-)

        bidirectional_inverted:
            canonical [-1, 1] -> q from zero toward lower (+) / upper (-)

            Used when the robot's numeric q+ is the SEMANTIC negative
            direction: canonical +1 commands the numeric q_min side.
            Example: index_j1 — canonical index_adduction +1 means
            "toward middle" while the robot's numeric q+ is physically
            toward thumb.
    """

    FEATURE_MAPPING = {
        "thumb_j1": ("thumb_opposition", "positive"),
        "thumb_j2": ("thumb_abduction", "negative"),
        "thumb_j3": ("thumb_distal", "bidirectional"),

        "index_j1": ("index_adduction", "bidirectional_inverted"),
        "index_j2": ("index_flexion", "positive"),
        # Finger distal flexion is unidirectional (human DIP joints have
        # no reverse motion): canonical 0 = straight baseline -> robot
        # zero; canonical 1 = distal flexed toward palm -> flex limit.
        "index_j3": ("index_distal", "positive"),

        "middle_j1": ("middle_flexion", "negative"),
        "middle_j2": ("middle_distal", "positive"),

        "little_j1": ("little_outward", "negative"),
        "little_j2": ("little_flexion", "positive"),
        "little_j3": ("little_distal", "positive"),
    }

    def __init__(
        self,
        semantics_path: Path | str = DEFAULT_SEMANTICS_PATH,
    ):
        semantics_path = Path(semantics_path)

        with semantics_path.open("r") as f:
            config = yaml.safe_load(f)

        self.joint_order = list(config["joint_order"])
        self.joints = config["joints"]

        if len(self.joint_order) != 11:
            raise RuntimeError(
                f"Expected 11 SG100 joints, "
                f"got {len(self.joint_order)}"
            )

        missing = set(self.joint_order) - set(self.FEATURE_MAPPING)

        if missing:
            raise RuntimeError(
                f"Missing retarget mapping for joints: {missing}"
            )

        # Robot joint neutral offsets (q at canonical 0), validated
        # against each joint's mechanical range.
        self.joint_zero = {}

        for joint_name in self.joint_order:
            lower, upper = map(
                float,
                self.joints[joint_name]["range"],
            )

            zero = float(
                self.joints[joint_name].get("zero", 0.0)
            )

            if not (lower <= zero <= upper):
                raise RuntimeError(
                    f"Semantics zero for {joint_name} ({zero}) is "
                    f"outside its range [{lower}, {upper}]"
                )

            self.joint_zero[joint_name] = zero

    @staticmethod
    def _positive(value: float, zero: float, upper: float) -> float:
        """
        Canonical [0, 1] -> q from zero toward upper.
        """
        value = float(np.clip(value, 0.0, 1.0))
        return zero + value * (upper - zero)

    @staticmethod
    def _negative(value: float, zero: float, lower: float) -> float:
        """
        Canonical [0, 1] -> q from zero toward lower.
        """
        value = float(np.clip(value, 0.0, 1.0))
        return zero + value * (lower - zero)

    @staticmethod
    def _bidirectional(
        value: float,
        zero: float,
        lower: float,
        upper: float,
    ) -> float:
        """
        Canonical [-1, 1] -> q from zero toward upper (+) / lower (-).
        """
        value = float(np.clip(value, -1.0, 1.0))

        if value >= 0.0:
            return zero + value * (upper - zero)

        return zero + (-value) * (lower - zero)

    @staticmethod
    def _bidirectional_inverted(
        value: float,
        zero: float,
        lower: float,
        upper: float,
    ) -> float:
        """
        Canonical [-1, 1] -> q from zero toward lower (+) / upper (-).

        The robot's numeric q+ is the semantic NEGATIVE direction, so
        canonical + commands the numeric q_min side.  If zero equals the
        lower limit, the canonical + side saturates at zero (the robot
        cannot physically move further in that direction).
        """
        value = float(np.clip(value, -1.0, 1.0))

        if value >= 0.0:
            return zero + value * (lower - zero)

        return zero + (-value) * (upper - zero)

    def retarget(
        self,
        hand: CanonicalHandState,
    ) -> np.ndarray:

        hand = hand.clipped()
        features = hand.as_dict()

        q_target = np.zeros(
            len(self.joint_order),
            dtype=np.float64,
        )

        for i, joint_name in enumerate(self.joint_order):

            joint_cfg = self.joints[joint_name]

            lower, upper = map(
                float,
                joint_cfg["range"],
            )

            zero = self.joint_zero[joint_name]

            feature_name, mapping_type = (
                self.FEATURE_MAPPING[joint_name]
            )

            value = features[feature_name]

            if mapping_type == "positive":

                q = self._positive(
                    value=value,
                    zero=zero,
                    upper=upper,
                )

            elif mapping_type == "negative":

                q = self._negative(
                    value=value,
                    zero=zero,
                    lower=lower,
                )

            elif mapping_type == "bidirectional":

                q = self._bidirectional(
                    value=value,
                    zero=zero,
                    lower=lower,
                    upper=upper,
                )

            elif mapping_type == "bidirectional_inverted":

                q = self._bidirectional_inverted(
                    value=value,
                    zero=zero,
                    lower=lower,
                    upper=upper,
                )

            else:
                raise RuntimeError(
                    f"Unknown mapping type: {mapping_type}"
                )

            q_target[i] = np.clip(
                q,
                lower,
                upper,
            )

        return q_target

    def as_joint_dict(
        self,
        q_target: np.ndarray,
    ) -> dict[str, float]:
        q_target = np.asarray(q_target)

        if q_target.shape != (11,):
            raise ValueError(
                f"Expected shape (11,), "
                f"got {q_target.shape}"
            )

        return {
            joint: float(value)
            for joint, value
            in zip(self.joint_order, q_target)
        }
