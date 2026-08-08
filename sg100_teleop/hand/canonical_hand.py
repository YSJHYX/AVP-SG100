from dataclasses import dataclass, fields


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


@dataclass
class CanonicalHandState:
    """
    Device-independent semantic representation of a human hand.

    Convention
    ----------
    Unidirectional features:
        0.0 = neutral
        1.0 = maximum semantic motion

    Bidirectional distal features:
       -1.0 = dorsal / outward extension
        0.0 = neutral
       +1.0 = palmar / inward flexion

    This class contains no Vision Pro, MediaPipe, Manus,
    MuJoCo or SG100-specific data structures.
    """

    # Thumb
    thumb_opposition: float = 0.0
    thumb_abduction: float = 0.0
    thumb_distal: float = 0.0

    # Index
    index_adduction: float = 0.0
    index_flexion: float = 0.0
    index_distal: float = 0.0

    # Middle
    middle_flexion: float = 0.0
    middle_distal: float = 0.0

    # Little
    little_outward: float = 0.0
    little_flexion: float = 0.0
    little_distal: float = 0.0

    def clipped(self) -> "CanonicalHandState":
        return CanonicalHandState(
            thumb_opposition=_clip(self.thumb_opposition, 0.0, 1.0),
            thumb_abduction=_clip(self.thumb_abduction, 0.0, 1.0),
            thumb_distal=_clip(self.thumb_distal, -1.0, 1.0),

            index_adduction=_clip(self.index_adduction, 0.0, 1.0),
            index_flexion=_clip(self.index_flexion, 0.0, 1.0),
            index_distal=_clip(self.index_distal, -1.0, 1.0),

            middle_flexion=_clip(self.middle_flexion, 0.0, 1.0),
            middle_distal=_clip(self.middle_distal, -1.0, 1.0),

            little_outward=_clip(self.little_outward, 0.0, 1.0),
            little_flexion=_clip(self.little_flexion, 0.0, 1.0),
            little_distal=_clip(self.little_distal, -1.0, 1.0),
        )

    def as_dict(self) -> dict[str, float]:
        return {
            field.name: float(getattr(self, field.name))
            for field in fields(self)
        }

    @classmethod
    def neutral(cls) -> "CanonicalHandState":
        return cls()
