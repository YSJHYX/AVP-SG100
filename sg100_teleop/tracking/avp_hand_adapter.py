from dataclasses import dataclass, fields

import numpy as np


# Current avp_stream 25/27-joint ordering.
AVP_JOINT_INDEX = {
    "wrist": 0,

    "thumbKnuckle": 1,
    "thumbIntermediateBase": 2,
    "thumbIntermediateTip": 3,
    "thumbTip": 4,

    "indexMetacarpal": 5,
    "indexKnuckle": 6,
    "indexIntermediateBase": 7,
    "indexIntermediateTip": 8,
    "indexTip": 9,

    "middleMetacarpal": 10,
    "middleKnuckle": 11,
    "middleIntermediateBase": 12,
    "middleIntermediateTip": 13,
    "middleTip": 14,

    "ringMetacarpal": 15,
    "ringKnuckle": 16,
    "ringIntermediateBase": 17,
    "ringIntermediateTip": 18,
    "ringTip": 19,

    "littleMetacarpal": 20,
    "littleKnuckle": 21,
    "littleIntermediateBase": 22,
    "littleIntermediateTip": 23,
    "littleTip": 24,

    "forearmWrist": 25,
    "forearmArm": 26,
}


_EPS = 1e-8


def _normalize(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64)

    n = np.linalg.norm(v)

    if n < _EPS:
        raise ValueError(
            f"Cannot normalize near-zero vector: {v}"
        )

    return v / n


def _safe_angle(
    v1: np.ndarray,
    v2: np.ndarray,
) -> float:
    """
    Unsigned angle between vectors, radians.
    """

    a = _normalize(v1)
    b = _normalize(v2)

    cosine = np.clip(
        np.dot(a, b),
        -1.0,
        1.0,
    )

    return float(np.arccos(cosine))


def _bend_angle(
    a: np.ndarray,
    b: np.ndarray,
    c: np.ndarray,
) -> float:
    """
    Finger bend at point b.

    Straight chain:
        a -> b -> c
        bend = 0

    Increasing bend:
        bend > 0
    """

    proximal = b - a
    distal = c - b

    return _safe_angle(
        proximal,
        distal,
    )


def _signed_angle(
    v1: np.ndarray,
    v2: np.ndarray,
    axis: np.ndarray,
) -> float:
    """
    Signed rotation from v1 to v2 around axis.
    """

    a = _normalize(v1)
    b = _normalize(v2)
    axis = _normalize(axis)

    sin_value = np.dot(
        axis,
        np.cross(a, b),
    )

    cos_value = np.clip(
        np.dot(a, b),
        -1.0,
        1.0,
    )

    return float(
        np.arctan2(
            sin_value,
            cos_value,
        )
    )


def _project_to_plane(
    v: np.ndarray,
    normal: np.ndarray,
) -> np.ndarray:
    normal = _normalize(normal)

    projected = (
        v
        - np.dot(v, normal) * normal
    )

    return _normalize(projected)


@dataclass(frozen=True)
class AVPRawHandFeatures:
    """
    Raw geometric features extracted from AVP HandSkeleton.

    IMPORTANT:
    These are NOT normalized SG100 commands.

    Values remain in geometric units:
      - angular quantities: radians
      - thumb coordinates: normalized by palm width

    Calibration maps these values into CanonicalHandState.
    """

    thumb_opposition: float
    thumb_abduction: float
    thumb_distal: float

    index_adduction: float
    index_flexion: float
    index_distal: float

    middle_flexion: float
    middle_distal: float

    little_outward: float
    little_flexion: float
    little_distal: float

    def as_dict(self) -> dict[str, float]:
        return {
            f.name: float(getattr(self, f.name))
            for f in fields(self)
        }

    def as_array(self) -> np.ndarray:
        return np.asarray(
            list(self.as_dict().values()),
            dtype=np.float64,
        )


class AVPHandFeatureExtractor:
    """
    Extract device geometry from a VisionProTeleop HandData
    or compatible Nx4x4 transform array.

    Main design rule:

        world hand skeleton
            ↓
        wrist-local skeleton
            ↓
        palm-local geometric features

    Therefore global:
      - translation
      - hand pose
      - wrist orientation

    do NOT directly change finger features.
    """

    def __init__(self):
        self.joint_index = AVP_JOINT_INDEX

    def _validate(
        self,
        hand,
    ) -> np.ndarray:

        joints = np.asarray(
            hand,
            dtype=np.float64,
        )

        if joints.ndim != 3:
            raise ValueError(
                f"Expected Nx4x4 array, "
                f"got shape {joints.shape}"
            )

        if joints.shape[1:] != (4, 4):
            raise ValueError(
                f"Expected Nx4x4 array, "
                f"got shape {joints.shape}"
            )

        if joints.shape[0] < 25:
            raise ValueError(
                "AVP hand skeleton must contain "
                f"at least 25 joints, got {joints.shape[0]}"
            )

        if not np.all(np.isfinite(joints)):
            raise ValueError(
                "Hand skeleton contains NaN/Inf"
            )

        return joints

    def _to_wrist_local(
        self,
        world_joints: np.ndarray,
    ) -> np.ndarray:
        """
        Remove global AVP/world/wrist transform.
        """

        wrist_world = world_joints[
            self.joint_index["wrist"]
        ]

        wrist_inv = np.linalg.inv(
            wrist_world
        )

        return (
            wrist_inv[None, :, :]
            @ world_joints
        )

    def _position(
        self,
        local_joints: np.ndarray,
        name: str,
    ) -> np.ndarray:

        idx = self.joint_index[name]

        return local_joints[
            idx,
            :3,
            3,
        ].copy()

    def extract(
        self,
        hand,
    ) -> AVPRawHandFeatures:

        world = self._validate(hand)
        local = self._to_wrist_local(world)

        # ---------------------------------------------------------
        # Named positions
        # ---------------------------------------------------------

        wrist = self._position(
            local,
            "wrist",
        )

        thumb_knuckle = self._position(
            local,
            "thumbKnuckle",
        )
        thumb_base = self._position(
            local,
            "thumbIntermediateBase",
        )
        thumb_mid = self._position(
            local,
            "thumbIntermediateTip",
        )
        thumb_tip = self._position(
            local,
            "thumbTip",
        )

        index_meta = self._position(
            local,
            "indexMetacarpal",
        )
        index_knuckle = self._position(
            local,
            "indexKnuckle",
        )
        index_base = self._position(
            local,
            "indexIntermediateBase",
        )
        index_mid = self._position(
            local,
            "indexIntermediateTip",
        )
        index_tip = self._position(
            local,
            "indexTip",
        )

        middle_meta = self._position(
            local,
            "middleMetacarpal",
        )
        middle_knuckle = self._position(
            local,
            "middleKnuckle",
        )
        middle_base = self._position(
            local,
            "middleIntermediateBase",
        )
        middle_mid = self._position(
            local,
            "middleIntermediateTip",
        )
        middle_tip = self._position(
            local,
            "middleTip",
        )

        little_meta = self._position(
            local,
            "littleMetacarpal",
        )
        little_knuckle = self._position(
            local,
            "littleKnuckle",
        )
        little_base = self._position(
            local,
            "littleIntermediateBase",
        )
        little_mid = self._position(
            local,
            "littleIntermediateTip",
        )
        little_tip = self._position(
            local,
            "littleTip",
        )

        # ---------------------------------------------------------
        # Palm frame
        #
        # x: little -> index
        # y: wrist -> middle knuckle
        # z: palm normal
        #
        # Sign does not need to be hardcoded into SG100 here.
        # Calibration will establish neutral/active signs.
        # ---------------------------------------------------------

        palm_x = _normalize(
            index_knuckle
            - little_knuckle
        )

        palm_y_seed = (
            middle_knuckle
            - wrist
        )

        palm_y_seed = (
            palm_y_seed
            - np.dot(
                palm_y_seed,
                palm_x,
            ) * palm_x
        )

        palm_y = _normalize(
            palm_y_seed
        )

        palm_z = _normalize(
            np.cross(
                palm_x,
                palm_y,
            )
        )

        palm_width = np.linalg.norm(
            index_knuckle
            - little_knuckle
        )

        if palm_width < _EPS:
            raise ValueError(
                "Invalid palm width"
            )

        palm_origin = middle_knuckle

        # ---------------------------------------------------------
        # Thumb proximal features
        #
        # Do not prematurely claim a perfect anatomical mapping.
        #
        # Instead use two independent palm-relative coordinates
        # which are later calibrated by explicit user poses.
        # ---------------------------------------------------------

        thumb_relative = (
            thumb_tip
            - palm_origin
        )

        thumb_opposition = float(
            np.dot(
                thumb_relative,
                palm_x,
            )
            / palm_width
        )

        thumb_abduction = float(
            np.dot(
                thumb_relative,
                palm_z,
            )
            / palm_width
        )

        # Thumb distal signed bend.
        #
        # palm_z is used as geometric reference axis.
        # Calibration determines which sign maps to SG100 +/-.
        thumb_distal = _signed_angle(
            thumb_mid - thumb_base,
            thumb_tip - thumb_mid,
            palm_z,
        )

        # ---------------------------------------------------------
        # Index lateral motion
        #
        # Compare index proximal direction against middle proximal
        # direction in palm plane.
        # ---------------------------------------------------------

        index_direction = _project_to_plane(
            index_base
            - index_knuckle,
            palm_z,
        )

        middle_direction = _project_to_plane(
            middle_base
            - middle_knuckle,
            palm_z,
        )

        index_adduction = _signed_angle(
            middle_direction,
            index_direction,
            palm_z,
        )

        # ---------------------------------------------------------
        # Main finger flexion
        # ---------------------------------------------------------

        index_flexion = _bend_angle(
            index_meta,
            index_knuckle,
            index_base,
        )

        middle_flexion = _bend_angle(
            middle_meta,
            middle_knuckle,
            middle_base,
        )

        little_flexion = _bend_angle(
            little_meta,
            little_knuckle,
            little_base,
        )

        # ---------------------------------------------------------
        # Distal signed bend
        #
        # Common lateral palm axis is used as the signed reference.
        # Exact sign is calibrated later.
        # ---------------------------------------------------------

        index_distal = _signed_angle(
            index_mid
            - index_base,
            index_tip
            - index_mid,
            palm_x,
        )

        middle_distal = _signed_angle(
            middle_mid
            - middle_base,
            middle_tip
            - middle_mid,
            palm_x,
        )

        little_distal = _signed_angle(
            little_mid
            - little_base,
            little_tip
            - little_mid,
            palm_x,
        )

        # ---------------------------------------------------------
        # Little lateral/outward motion
        # ---------------------------------------------------------

        little_direction = _project_to_plane(
            little_base
            - little_knuckle,
            palm_z,
        )

        little_outward = _signed_angle(
            middle_direction,
            little_direction,
            palm_z,
        )

        return AVPRawHandFeatures(
            thumb_opposition=thumb_opposition,
            thumb_abduction=thumb_abduction,
            thumb_distal=thumb_distal,

            index_adduction=index_adduction,
            index_flexion=index_flexion,
            index_distal=index_distal,

            middle_flexion=middle_flexion,
            middle_distal=middle_distal,

            little_outward=little_outward,
            little_flexion=little_flexion,
            little_distal=little_distal,
        )
