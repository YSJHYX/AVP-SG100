import numpy as np


def get_hand_from_tracking(
    tracking,
    side: str = "right",
):
    """
    Convert VisionProStreamer.get_latest() output into
    an Nx4x4 world-frame hand skeleton.

    Supports:
      1. Current TrackingData API:
             tracking.right
      2. Backward-compatible dictionary API:
             right_wrist + right_arm
    """

    if tracking is None:
        return None

    if side not in ("left", "right"):
        raise ValueError(
            f"Unsupported hand side: {side}"
        )

    # Current API
    if hasattr(tracking, side):
        hand = getattr(tracking, side)

        if hand is not None:
            arr = np.asarray(
                hand,
                dtype=np.float64,
            )

            if (
                arr.ndim == 3
                and arr.shape[0] >= 25
                and arr.shape[1:] == (4, 4)
            ):
                return arr

    # Backward-compatible dictionary API
    wrist_key = f"{side}_wrist"
    arm_key = f"{side}_arm"

    try:
        wrist = tracking[wrist_key]
        arm = tracking[arm_key]
    except (KeyError, TypeError):
        return None

    if wrist is None or arm is None:
        return None

    wrist = np.asarray(
        wrist,
        dtype=np.float64,
    )

    arm = np.asarray(
        arm,
        dtype=np.float64,
    )

    wrist_mat = (
        wrist[0]
        if wrist.ndim == 3
        else wrist
    )

    if wrist_mat.shape != (4, 4):
        return None

    if (
        arm.ndim != 3
        or arm.shape[0] < 25
        or arm.shape[1:] != (4, 4)
    ):
        return None

    return (
        wrist_mat[None, :, :]
        @ arm
    )
