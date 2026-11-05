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


def test_pinch_keeps_exactly_four_live_joints_and_parks_the_rest():
    from sg100_teleop.control.grasp_mode import PINCH_ACTIVE_JOINTS

    # Resolve by name even when a caller uses a different joint order.
    order = tuple(reversed(SG100_JOINT_ORDER))
    controller = GraspModeController(joint_order=order, initial_mode=GraspMode.PINCH)
    incoming = np.linspace(-1.1, 1.1, len(order))
    original = incoming.copy()
    output = controller.apply(incoming, dt_s=0.0)
    for i, name in enumerate(order):
        if name in PINCH_ACTIVE_JOINTS:
            assert output[i] == incoming[i]
        else:
            assert output[i] == (np.pi / 2 if name == "thumb_j1" else 0.0)
    np.testing.assert_array_equal(incoming, original)


def test_pinch_uses_base_mapping_so_automatic_coordination_cannot_freeze_j2():
    from sg100_teleop.hand.canonical_hand import CanonicalHandState
    from sg100_teleop.retargeting.sg100_retargeter import SG100Retargeter

    retargeter = SG100Retargeter()
    hand = CanonicalHandState(
        thumb_opposition=1.0, thumb_abduction=1.0, thumb_distal=0.2,
        index_flexion=0.7, index_distal=0.4,
    )
    final, stages = retargeter.retarget_with_stages(hand)
    raw = stages["raw_retarget_qcmd"]
    # The existing automatic pinch branch parks J2 at -30 deg.
    assert not np.isclose(final[_index("thumb_j2")], raw[_index("thumb_j2")])
    controller = GraspModeController(initial_mode=GraspMode.PINCH)
    output = controller.apply(final, dt_s=0.0, raw_retarget_qcmd=raw)
    assert output[_index("thumb_j2")] == raw[_index("thumb_j2")]


def test_pinch_open_close_maps_thumb_and_index_independently():
    from sg100_teleop.control.grasp_mode import PINCH_ACTIVE_JOINTS
    from sg100_teleop.hand.canonical_hand import CanonicalHandState
    from sg100_teleop.retargeting.sg100_retargeter import SG100Retargeter

    retargeter = SG100Retargeter()
    controller = GraspModeController(initial_mode=GraspMode.PINCH)
    outputs = []
    for hand in (
        CanonicalHandState.neutral(),
        CanonicalHandState(thumb_abduction=0.8, thumb_distal=0.7,
                           index_flexion=0.9, index_distal=0.8),
        CanonicalHandState.neutral(),
    ):
        final, stages = retargeter.retarget_with_stages(hand)
        output = controller.apply(final, dt_s=0.0,
                                  raw_retarget_qcmd=stages["raw_retarget_qcmd"])
        for name in PINCH_ACTIVE_JOINTS:
            i = _index(name)
            assert output[i] == stages["raw_retarget_qcmd"][i]
        outputs.append(output)
    for name in PINCH_ACTIVE_JOINTS:
        i = _index(name)
        assert not np.isclose(outputs[0][i], outputs[1][i])
    np.testing.assert_array_equal(outputs[0], outputs[2])


def test_pinch_is_not_affected_by_middle_or_little_finger_motion():
    from sg100_teleop.hand.canonical_hand import CanonicalHandState
    from sg100_teleop.retargeting.sg100_retargeter import SG100Retargeter

    retargeter = SG100Retargeter()
    controller = GraspModeController(initial_mode=GraspMode.PINCH)
    outputs = []
    for background in (0.0, 0.5, 1.0):
        hand = CanonicalHandState(
            thumb_opposition=0.9, thumb_abduction=0.8, thumb_distal=0.4,
            index_flexion=0.5, index_distal=0.6,
            middle_flexion=background, middle_distal=background,
            little_outward=background, little_flexion=background,
            little_distal=background,
        )
        final, stages = retargeter.retarget_with_stages(hand)
        outputs.append(controller.apply(final, dt_s=0.0,
                       raw_retarget_qcmd=stages["raw_retarget_qcmd"]))
    for output in outputs[1:]:
        np.testing.assert_array_equal(output, outputs[0])


def test_pinch_transition_respects_slew_and_releases_on_free():
    from sg100_teleop.control.grasp_mode import PINCH_ACTIVE_JOINTS

    controller = GraspModeController(hold_slew_rad_s=3.0)
    seed = np.linspace(-0.8, 0.8, len(SG100_JOINT_ORDER))
    controller.apply(seed, dt_s=1 / 60)
    controller.set_mode(GraspMode.PINCH)
    incoming = seed[::-1].copy()
    first = controller.apply(incoming, dt_s=1 / 60)
    for i, name in enumerate(SG100_JOINT_ORDER):
        if name not in PINCH_ACTIVE_JOINTS:
            assert abs(first[i] - seed[i]) <= 3.0 / 60 + 1e-12
    for _ in range(120):
        output = controller.apply(incoming, dt_s=1 / 60)
    for i, name in enumerate(SG100_JOINT_ORDER):
        if name not in PINCH_ACTIVE_JOINTS:
            assert output[i] == (np.pi / 2 if name == "thumb_j1" else 0.0)
    controller.set_mode(GraspMode.FREE)
    np.testing.assert_array_equal(controller.apply(incoming, dt_s=1 / 60), incoming)


def test_raw_pinch_source_does_not_change_existing_modes():
    for mode in (GraspMode.FREE, GraspMode.HAND, GraspMode.CLAW):
        incoming = np.linspace(-0.5, 0.5, len(SG100_JOINT_ORDER))
        raw = incoming[::-1].copy()
        normal = GraspModeController(initial_mode=mode)
        with_raw = GraspModeController(initial_mode=mode)
        np.testing.assert_array_equal(
            normal.apply(incoming, dt_s=1 / 60),
            with_raw.apply(incoming, dt_s=1 / 60, raw_retarget_qcmd=raw),
        )


def test_key_5_exits_script_source_and_selects_pinch():
    # Exercise the actual nested viewer callback without creating a viewer,
    # importing the runner, opening serial, or connecting to Vision Pro.
    import ast
    from pathlib import Path

    runner = Path(__file__).resolve().parents[1] / "sim/run_avp_arm_hand_sim.py"
    tree = ast.parse(runner.read_text())
    callback = next(n for n in ast.walk(tree)
                    if isinstance(n, ast.FunctionDef) and n.name == "on_viewer_key")
    factory = ast.FunctionDef(
        name="make_handler",
        args=ast.arguments(posonlyargs=[], args=[], vararg=None,
                           kwonlyargs=[], kw_defaults=[], kwarg=None, defaults=[]),
        body=[ast.Assign(targets=[ast.Name(id="q_hand_goal", ctx=ast.Store())],
                         value=ast.Constant(value=None)),
              callback, ast.Return(value=ast.Name(id="on_viewer_key", ctx=ast.Load()))],
        decorator_list=[],
    )
    class ScriptStub:
        enabled = True
        def exit(self):
            self.enabled = False
    script = ScriptStub()
    controller = GraspModeController(initial_mode=GraspMode.CLAW)
    scope = {"GraspMode": GraspMode, "grasp_mode_controller": controller,
             "script_hand_controller": script}
    module = ast.fix_missing_locations(ast.Module(body=[factory], type_ignores=[]))
    exec(compile(module, str(runner), "exec"), scope)
    handler = scope["make_handler"]()
    handler(ord("5"))
    assert controller.mode == GraspMode.PINCH
    assert not script.enabled
    handler(ord("1"))
    assert controller.mode == GraspMode.FREE
