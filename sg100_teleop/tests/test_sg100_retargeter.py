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

    expected = np.array([
         2.182,
        -2.610,
         1.570,

         1.570,
         2.180,
         1.570,

        -2.180,
         1.570,

        -3.140,
         2.180,
         1.570,
    ])

    assert np.allclose(q, expected)

    print("\n===== DISTAL EXTENSION =====")

    hand = CanonicalHandState(
        thumb_distal=-1.0,
        index_distal=-1.0,
        middle_distal=-1.0,
        little_distal=-1.0,
    )

    q = r.retarget(hand)

    for name, value in r.as_joint_dict(q).items():
        print(f"{name:12s}: {value:+.4f}")

    assert np.isclose(q[2], -1.048)
    assert np.isclose(q[5], -1.05)
    assert np.isclose(q[7], -1.05)
    assert np.isclose(q[10], -1.05)

    print("\nPASS")
    print(
        "CanonicalHandState -> SG100Retargeter "
        "mapping is internally consistent."
    )


if __name__ == "__main__":
    main()
