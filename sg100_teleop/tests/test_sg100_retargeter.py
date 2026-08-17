import numpy as np

from sg100_teleop.hand.canonical_hand import (
    CanonicalHandState,
)
from sg100_teleop.retargeting.sg100_retargeter import (
    SG100Retargeter,
)


def main():

    r = SG100Retargeter()

    print("===== NEUTRAL =====")

    hand = CanonicalHandState.neutral()
    q = r.retarget(hand)

    print(q)

    assert q.shape == (11,)
    assert np.allclose(q, 0.0)

    print("\n===== MAX POSITIVE / FLEXION =====")

    hand = CanonicalHandState(
        thumb_opposition=1.0,
        thumb_abduction=1.0,
        thumb_distal=1.0,

        index_adduction=1.0,
        index_flexion=1.0,
        index_distal=1.0,

        middle_flexion=1.0,
        middle_distal=1.0,

        little_outward=1.0,
        little_flexion=1.0,
        little_distal=1.0,
    )

    q = r.retarget(hand)

    for name, value in r.as_joint_dict(q).items():
        print(f"{name:12s}: {value:+.4f}")

    # Thumb root joints interpolate onto their TELEOP semantic ranges
    # (not the full mechanical ROM): thumb_j1 0 -> +90 deg through the
    # gamma=1.5 response curve, thumb_j2 0 -> -60 deg.  All other
    # joints keep the mechanical limits until a teleop_range is
    # configured for them.
    #
    # index_adduction=+1 means TOWARD MIDDLE.  index_j1's numeric q+ is
    # physically toward thumb, so canonical +1 commands the q_min side:
    # q = zero + 1.0 * (q_min - zero) = 0.0 with the current semantics
    # zero (0.0 == q_min, the adducted extreme saturates at zero).
    expected = np.array([
         1.5708,
        -1.0472,
         1.570,

         0.0,
         2.180,
         1.570,

        -2.180,
         1.570,

        -3.140,
         2.180,
         1.570,
    ])

    assert np.allclose(q, expected)

    # Teleop ranges must stay strict subsets of the mechanical ranges.
    for joint in ("thumb_j1", "thumb_j2"):
        m_lower, m_upper = map(
            float, r.joints[joint]["range"]
        )
        t_lower, t_upper = r.joint_teleop_range[joint]
        assert m_lower <= t_lower < t_upper <= m_upper, joint

    print("\n===== INDEX ADDUCTION NEGATIVE (TOWARD THUMB) =====")

    hand = CanonicalHandState(
        index_adduction=-1.0,
    )

    q = r.retarget(hand)

    for name, value in r.as_joint_dict(q).items():
        print(f"{name:12s}: {value:+.4f}")

    # Canonical -1 (toward thumb) commands the numeric q_max side.
    assert np.isclose(q[3], 1.570)
    assert np.allclose(
        np.delete(q, 3),
        np.zeros(10),
    )

    print("\n===== DISTAL: THUMB EXTENSION / FINGER FLEXION =====")

    # thumb_distal stays bidirectional (IP extension -> -1.048).
    hand = CanonicalHandState(thumb_distal=-1.0)
    q = r.retarget(hand)
    assert np.isclose(q[2], -1.048)

    # Finger distals are unidirectional: straight (canonical 0) maps to
    # semantics zero; flexion (canonical 1) maps to the flex limit.
    # Negative canonical input clips to zero, never to -60 deg limits.
    for feature, joint, upper in (
        ("index_distal", "index_j3", 1.57),
        ("middle_distal", "middle_j2", 1.57),
        ("little_distal", "little_j3", 1.57),
    ):
        index = r.joint_order.index(joint)

        q = r.retarget(CanonicalHandState(**{feature: 1.0}))
        assert np.isclose(q[index], upper), (joint, q[index])

        for value in (0.0, -1.0):
            q = r.retarget(CanonicalHandState(**{feature: value}))
            assert np.isclose(q[index], 0.0), (joint, value, q[index])

    print("\nPASS")
    print(
        "CanonicalHandState -> SG100Retargeter "
        "mapping is internally consistent."
    )


if __name__ == "__main__":
    main()
