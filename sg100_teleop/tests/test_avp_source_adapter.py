import numpy as np

from avp_stream.streamer import TrackingData
from sg100_teleop.tracking.avp_source import get_hand_from_tracking


def main():
    wrist = np.eye(4)[None, :, :]
    arm = np.repeat(np.eye(4)[None, :, :], 27, axis=0)
    arm[:, 0, 3] = np.linspace(0.0, 0.026, 27)

    current = TrackingData(
        {
            "right_wrist": wrist,
            "right_arm": arm,
            "right_fingers": arm[:25],
        }
    )
    current_hand = get_hand_from_tracking(current, side="right")
    np.testing.assert_allclose(current_hand, arm)

    legacy = {"right_wrist": wrist, "right_arm": arm}
    legacy_hand = get_hand_from_tracking(legacy, side="right")
    np.testing.assert_allclose(legacy_hand, arm)

    malformed = {"right_wrist": wrist, "right_arm": arm[:3]}
    assert get_hand_from_tracking(malformed, side="right") is None

    print("AVP SOURCE ADAPTER: PASS")


if __name__ == "__main__":
    main()
