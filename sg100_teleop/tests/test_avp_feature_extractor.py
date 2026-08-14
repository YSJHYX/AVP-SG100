import numpy as np
from scipy.spatial.transform import Rotation

from sg100_teleop.tracking.avp_hand_adapter import (
    AVPHandFeatureExtractor,
    AVP_JOINT_INDEX,
    _finger_lateral_flexion_axis,
    _signed_angle,
)


def T(position):
    mat = np.eye(4)
    mat[:3, 3] = position
    return mat


def build_straight_hand():
    """
    Synthetic right-hand-like geometry.

    This is NOT intended to model real AVP dimensions.
    It only tests geometric invariants.
    """

    joints = np.repeat(
        np.eye(4)[None, :, :],
        27,
        axis=0,
    )

    def setp(name, xyz):
        joints[
            AVP_JOINT_INDEX[name]
        ] = T(xyz)

    setp("wrist", [0.0, 0.0, 0.0])

    # Thumb
    setp("thumbKnuckle", [0.045, 0.020, 0.000])
    setp("thumbIntermediateBase", [0.060, 0.035, -0.004])
    setp("thumbIntermediateTip", [0.075, 0.050, -0.008])
    setp("thumbTip", [0.090, 0.065, -0.012])

    # Index
    setp("indexMetacarpal", [0.030, 0.025, 0.0])
    setp("indexKnuckle", [0.030, 0.050, 0.0])
    setp("indexIntermediateBase", [0.030, 0.080, 0.0])
    setp("indexIntermediateTip", [0.030, 0.110, 0.0])
    setp("indexTip", [0.030, 0.140, 0.0])

    # Middle
    setp("middleMetacarpal", [0.000, 0.025, 0.0])
    setp("middleKnuckle", [0.000, 0.055, 0.0])
    setp("middleIntermediateBase", [0.000, 0.090, 0.0])
    setp("middleIntermediateTip", [0.000, 0.125, 0.0])
    setp("middleTip", [0.000, 0.160, 0.0])

    # Ring
    setp("ringMetacarpal", [-0.020, 0.025, 0.0])
    setp("ringKnuckle", [-0.020, 0.052, 0.0])
    setp("ringIntermediateBase", [-0.020, 0.083, 0.0])
    setp("ringIntermediateTip", [-0.020, 0.113, 0.0])
    setp("ringTip", [-0.020, 0.143, 0.0])

    # Little
    setp("littleMetacarpal", [-0.035, 0.022, 0.0])
    setp("littleKnuckle", [-0.035, 0.047, 0.0])
    setp("littleIntermediateBase", [-0.035, 0.074, 0.0])
    setp("littleIntermediateTip", [-0.035, 0.100, 0.0])
    setp("littleTip", [-0.035, 0.126, 0.0])

    return joints


def apply_global_transform(
    joints,
    rotation_xyz_deg,
    translation,
):
    rot = Rotation.from_euler(
        "xyz",
        rotation_xyz_deg,
        degrees=True,
    ).as_matrix()

    world = np.eye(4)
    world[:3, :3] = rot
    world[:3, 3] = translation

    return (
        world[None, :, :]
        @ joints
    )


def scale_hand(joints, scale):
    result = joints.copy()

    result[:, :3, 3] *= scale

    return result


def bend_index(joints):
    """
    Introduce synthetic proximal index flexion.
    """

    result = joints.copy()

    knuckle = result[
        AVP_JOINT_INDEX["indexKnuckle"],
        :3,
        3,
    ].copy()

    # Bend following chain toward -z.
    result[
        AVP_JOINT_INDEX["indexIntermediateBase"],
        :3,
        3,
    ] = knuckle + np.array(
        [0.0, 0.020, -0.020]
    )

    result[
        AVP_JOINT_INDEX["indexIntermediateTip"],
        :3,
        3,
    ] = knuckle + np.array(
        [0.0, 0.040, -0.040]
    )

    result[
        AVP_JOINT_INDEX["indexTip"],
        :3,
        3,
    ] = knuckle + np.array(
        [0.0, 0.060, -0.060]
    )

    return result


def set_finger_distal_angle(
    joints,
    finger_name,
    angle,
):
    """Rotate a synthetic distal segment about that finger's own axis."""

    result = joints.copy()
    knuckle = result[
        AVP_JOINT_INDEX[f"{finger_name}Knuckle"],
        :3,
        3,
    ]
    base = result[
        AVP_JOINT_INDEX[f"{finger_name}IntermediateBase"],
        :3,
        3,
    ]
    mid = result[
        AVP_JOINT_INDEX[f"{finger_name}IntermediateTip"],
        :3,
        3,
    ]
    proximal = base - knuckle
    palm_z = np.array([0.0, 0.0, 1.0])
    palm_x = np.array([1.0, 0.0, 0.0])
    axis = _finger_lateral_flexion_axis(
        proximal,
        palm_z,
        palm_x,
        context=f"test_{finger_name}",
    )
    distal = mid - base
    rotated = Rotation.from_rotvec(
        angle * axis
    ).apply(distal)
    result[
        AVP_JOINT_INDEX[f"{finger_name}Tip"],
        :3,
        3,
    ] = mid + rotated
    return result


def build_spread_distal_pose(angle):
    hand = build_straight_hand()

    # Give each finger a distinct proximal direction in the palm plane.
    for name, x_step in (
        ("index", 0.006),
        ("middle", 0.002),
        ("little", -0.005),
    ):
        base_index = AVP_JOINT_INDEX[
            f"{name}IntermediateBase"
        ]
        mid_index = AVP_JOINT_INDEX[
            f"{name}IntermediateTip"
        ]
        tip_index = AVP_JOINT_INDEX[f"{name}Tip"]
        hand[base_index, 0, 3] += x_step
        hand[mid_index, 0, 3] += 2.0 * x_step
        hand[tip_index, 0, 3] += 3.0 * x_step
        hand = set_finger_distal_angle(
            hand,
            name,
            angle,
        )

    return hand


def main():
    extractor = AVPHandFeatureExtractor()

    hand = build_straight_hand()

    base = extractor.extract(hand)

    print("===== BASE FEATURES =====")
    for name, value in base.as_dict().items():
        print(
            f"{name:20s}: "
            f"{value:+.6f}"
        )

    # ---------------------------------------------------------
    # Test 1:
    # Global rigid transformation must not change features.
    # ---------------------------------------------------------

    transformed_hand = apply_global_transform(
        hand,
        rotation_xyz_deg=[
            37.0,
            -21.0,
            64.0,
        ],
        translation=[
            0.8,
            -1.2,
            0.5,
        ],
    )

    transformed = extractor.extract(
        transformed_hand
    )

    diff = np.max(
        np.abs(
            base.as_array()
            - transformed.as_array()
        )
    )

    print()
    print(
        "rigid-transform max diff =",
        diff,
    )

    assert diff < 1e-8

    # ---------------------------------------------------------
    # Test 2:
    # Global human hand scale should not change normalized
    # geometric features.
    # Angles are scale-independent; thumb coordinates are
    # divided by palm width.
    # ---------------------------------------------------------

    scaled_hand = scale_hand(
        hand,
        1.7,
    )

    scaled = extractor.extract(
        scaled_hand
    )

    scale_diff = np.max(
        np.abs(
            base.as_array()
            - scaled.as_array()
        )
    )

    print(
        "scale max diff          =",
        scale_diff,
    )

    assert scale_diff < 1e-8

    # ---------------------------------------------------------
    # Test 3:
    # Synthetic index bending must increase raw
    # index flexion.
    # ---------------------------------------------------------

    bent_hand = bend_index(hand)

    bent = extractor.extract(
        bent_hand
    )

    print()
    print(
        "straight index flexion =",
        base.index_flexion,
    )
    print(
        "bent index flexion     =",
        bent.index_flexion,
    )

    assert (
        bent.index_flexion
        > base.index_flexion + 0.3
    )

    # ---------------------------------------------------------
    # Test 4:
    # Signed angles must use the axis-perpendicular projections.
    # Large axial contamination must not change the result.
    # ---------------------------------------------------------

    axis = np.array([0.0, 0.0, 1.0])
    positive_angle = _signed_angle(
        np.array([1.0, 0.0, 8.0]),
        np.array([np.cos(0.6), np.sin(0.6), -11.0]),
        axis,
        context="positive_projection_test",
    )
    negative_angle = _signed_angle(
        np.array([1.0, 0.0, -5.0]),
        np.array([np.cos(-0.4), np.sin(-0.4), 9.0]),
        axis,
        context="negative_projection_test",
    )

    assert abs(positive_angle - 0.6) < 1e-8
    assert abs(negative_angle + 0.4) < 1e-8
    assert positive_angle > 0.0
    assert negative_angle < 0.0

    try:
        _signed_angle(
            axis,
            np.array([1.0, 0.0, 0.0]),
            axis,
            context="degenerate_test",
        )
    except ValueError as exc:
        assert "degenerate_test" in str(exc)
        assert "projection" in str(exc)
    else:
        raise AssertionError(
            "Degenerate signed-angle projection was not rejected"
        )

    # ---------------------------------------------------------
    # Test 5:
    # Per-finger axes remain in the palm plane and share palm-x
    # handedness for naturally spread proximal directions.
    # ---------------------------------------------------------

    palm_z = np.array([0.0, 0.0, 1.0])
    palm_x = np.array([1.0, 0.0, 0.0])

    for i, direction in enumerate(
        (
            np.array([0.30, 1.0, 0.15]),
            np.array([0.05, 1.0, -0.20]),
            np.array([-0.35, 1.0, 0.10]),
        )
    ):
        finger_axis = _finger_lateral_flexion_axis(
            direction,
            palm_z,
            palm_x,
            context=f"axis_test_{i}",
        )
        assert abs(np.linalg.norm(finger_axis) - 1.0) < 1e-8
        assert abs(np.dot(finger_axis, palm_z)) < 1e-8
        assert np.dot(finger_axis, palm_x) > 0.0

    # ---------------------------------------------------------
    # Test 6:
    # A relaxed, slightly bent neutral pose must lie between
    # realistic negative and positive distal endpoints.
    # ---------------------------------------------------------

    negative_features = extractor.extract(
        build_spread_distal_pose(-0.25)
    )
    neutral_features = extractor.extract(
        build_spread_distal_pose(0.12)
    )
    positive_features = extractor.extract(
        build_spread_distal_pose(0.55)
    )

    for feature_name in (
        "index_distal",
        "middle_distal",
        "little_distal",
    ):
        negative_delta = (
            getattr(negative_features, feature_name)
            - getattr(neutral_features, feature_name)
        )
        positive_delta = (
            getattr(positive_features, feature_name)
            - getattr(neutral_features, feature_name)
        )
        assert negative_delta < -0.3
        assert positive_delta > 0.3
        assert negative_delta * positive_delta < 0.0

    print()
    print("PASS")
    print(
        "AVP geometric feature extractor "
        "is rigid-transform invariant, "
        "scale invariant, uses projected signed angles, "
        "and preserves realistic distal endpoint signs."
    )


if __name__ == "__main__":
    main()
