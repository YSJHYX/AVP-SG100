import numpy as np
from scipy.spatial.transform import Rotation

from sg100_teleop.tracking.avp_hand_adapter import (
    AVPHandFeatureExtractor,
    AVP_JOINT_INDEX,
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

    print()
    print("PASS")
    print(
        "AVP geometric feature extractor "
        "is rigid-transform invariant, "
        "scale invariant, and responds "
        "to synthetic index flexion."
    )


if __name__ == "__main__":
    main()
