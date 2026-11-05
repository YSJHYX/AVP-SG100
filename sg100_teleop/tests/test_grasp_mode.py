import numpy as np

from sg100_teleop.control.grasp_mode import (
    CLAW_THUMB_J2_LITTLE_OFFSET_RAD,
    GraspMode,
    GraspModeController,
)
from sg100_teleop.sim.combined_robot_layout import SG100_JOINT_ORDER


def _index(name: str) -> int:
    return SG100_JOINT_ORDER.index(name)


def test_claw_couples_thumb_to_little_during_initialization():
    controller = GraspModeController(joint_order=SG100_JOINT_ORDER)
    seed = np.linspace(-0.4, 0.6, len(SG100_JOINT_ORDER))
    controller.apply(seed, dt_s=1.0 / 60.0)
    controller.set_mode(GraspMode.CLAW)

    incoming = seed.copy()
    incoming[_index("thumb_j2")] = 1.2
    incoming[_index("thumb_j3")] = -0.9
    output = controller.apply(incoming, dt_s=1.0 / 60.0)

    assert output[_index("thumb_j2")] == (
        output[_index("little_j2")] - CLAW_THUMB_J2_LITTLE_OFFSET_RAD
    )
    assert output[_index("thumb_j3")] == output[_index("little_j3")]


def test_claw_couples_thumb_to_live_little_targets_after_initialization():
    controller = GraspModeController(joint_order=SG100_JOINT_ORDER)
    controller.set_mode(GraspMode.CLAW)
    controller.apply(np.zeros(len(SG100_JOINT_ORDER)), dt_s=0.0)

    incoming = np.zeros(len(SG100_JOINT_ORDER))
    incoming[_index("thumb_j2")] = -1.9
    incoming[_index("thumb_j3")] = 0.8
    incoming[_index("little_j2")] = 0.45
    incoming[_index("little_j3")] = -0.35
    output = controller.apply(incoming, dt_s=1.0 / 60.0)

    assert output[_index("thumb_j2")] == (
        incoming[_index("little_j2")] - CLAW_THUMB_J2_LITTLE_OFFSET_RAD
    )
    assert output[_index("thumb_j3")] == incoming[_index("little_j3")]


def test_free_and_hand_modes_do_not_couple_thumb_to_little():
    incoming = np.zeros(len(SG100_JOINT_ORDER))
    incoming[_index("thumb_j2")] = -1.7
    incoming[_index("thumb_j3")] = 0.7
    incoming[_index("little_j2")] = 0.3
    incoming[_index("little_j3")] = -0.2

    for mode in (GraspMode.FREE, GraspMode.HAND):
        controller = GraspModeController(
            joint_order=SG100_JOINT_ORDER,
            initial_mode=mode,
        )
        output = controller.apply(incoming, dt_s=1.0 / 60.0)
        assert output[_index("thumb_j2")] == incoming[_index("thumb_j2")]
        assert output[_index("thumb_j3")] == incoming[_index("thumb_j3")]
