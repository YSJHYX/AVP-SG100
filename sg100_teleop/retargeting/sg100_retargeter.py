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
    """

    FEATURE_MAPPING = {
        "thumb_j1": ("thumb_opposition", "positive"),
        "thumb_j2": ("thumb_abduction", "negative"),
        "thumb_j3": ("thumb_distal", "bidirectional"),

        "index_j1": ("index_adduction", "positive"),
        "index_j2": ("index_flexion", "positive"),
        "index_j3": ("index_distal", "bidirectional"),

        "middle_j1": ("middle_flexion", "negative"),
        "middle_j2": ("middle_distal", "bidirectional"),

        "little_j1": ("little_outward", "negative"),
        "little_j2": ("little_flexion", "positive"),
        "little_j3": ("little_distal", "bidirectional"),
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

    @staticmethod
    def _positive(value: float, upper: float) -> float:
        """
        Canonical [0, 1] -> SG100 [0, upper].
        """
        value = float(np.clip(value, 0.0, 1.0))
        return value * upper

    @staticmethod
    def _negative(value: float, lower: float) -> float:
        """
        Canonical [0, 1] -> SG100 [0, lower].
        """
        value = float(np.clip(value, 0.0, 1.0))
        return value * lower

    @staticmethod
    def _bidirectional(
        value: float,
        lower: float,
        upper: float,
    ) -> float:
        """
        Canonical [-1, 1] -> asymmetric SG100 range.

        -1 -> lower
         0 -> zero
        +1 -> upper
        """
        value = float(np.clip(value, -1.0, 1.0))

        if value >= 0.0:
            return value * upper

        return (-value) * lower

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

            feature_name, mapping_type = (
                self.FEATURE_MAPPING[joint_name]
            )

            value = features[feature_name]

            if mapping_type == "positive":

                q = self._positive(
                    value=value,
                    upper=upper,
                )

            elif mapping_type == "negative":

                q = self._negative(
                    value=value,
                    lower=lower,
                )

            elif mapping_type == "bidirectional":

                q = self._bidirectional(
                    value=value,
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
