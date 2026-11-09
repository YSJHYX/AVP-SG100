import numpy as np
import pytest

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


@pytest.mark.parametrize("closure", (0.0, 0.1, 0.5, 0.9, 1.0))
@pytest.mark.parametrize("order", (SG100_JOINT_ORDER, tuple(reversed(SG100_JOINT_ORDER))))
def test_pinch_synchronizes_two_live_joints_and_parks_the_rest(closure, order):
    from sg100_teleop.control.grasp_mode import (
        PINCH_ACTIVE_JOINTS, PINCH_J2_TRAVEL_RAD, PINCH_LOCK_RAD,
        PINCH_THUMB_J2_OPEN_RAD,
    )

    # Resolve by name even when a caller uses a different joint order.
    controller = GraspModeController(joint_order=order, initial_mode=GraspMode.PINCH)
    incoming = np.linspace(-1.1, 1.1, len(order))
    original = incoming.copy()
    output = controller.apply(incoming, dt_s=0.0, pinch_closure=closure)
    expected = dict(PINCH_LOCK_RAD)
    expected["index_j2"] = closure * PINCH_J2_TRAVEL_RAD
    expected["thumb_j2"] = PINCH_THUMB_J2_OPEN_RAD + expected["index_j2"]
    assert PINCH_ACTIVE_JOINTS == ("thumb_j2", "index_j2")
    for i, name in enumerate(order):
        assert output[i] == pytest.approx(expected[name], abs=1e-14)
    np.testing.assert_array_equal(incoming, original)


def test_pinch_shared_input_bypasses_automatic_coordination():
    from sg100_teleop.control.grasp_mode import (
        PINCH_J2_TRAVEL_RAD, PINCH_THUMB_J2_OPEN_RAD,
    )
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
    output = controller.apply(final, dt_s=0.0, pinch_closure=hand.index_flexion)
    travel = hand.index_flexion * PINCH_J2_TRAVEL_RAD
    assert output[_index("thumb_j2")] == PINCH_THUMB_J2_OPEN_RAD + travel
    assert output[_index("index_j2")] == travel


def test_pinch_open_close_uses_one_input_for_both_j2_joints():
    from sg100_teleop.control.grasp_mode import PINCH_ACTIVE_JOINTS, PINCH_LOCK_RAD
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
        final = retargeter.retarget(hand)
        output = controller.apply(final, dt_s=0.0, pinch_closure=hand.index_flexion)
        for name, target in PINCH_LOCK_RAD:
            assert output[_index(name)] == target
        outputs.append(output)
    for name in PINCH_ACTIVE_JOINTS:
        i = _index(name)
        assert not np.isclose(outputs[0][i], outputs[1][i])
    np.testing.assert_allclose(
        outputs[1][_index("thumb_j2")] - outputs[0][_index("thumb_j2")],
        outputs[1][_index("index_j2")] - outputs[0][_index("index_j2")],
    )
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
        final = retargeter.retarget(hand)
        outputs.append(controller.apply(final, dt_s=0.0,
                       pinch_closure=hand.index_flexion))
    for output in outputs[1:]:
        np.testing.assert_array_equal(output, outputs[0])


def test_pinch_transition_respects_slew_and_releases_on_free():
    from sg100_teleop.control.grasp_mode import PINCH_ACTIVE_JOINTS, PINCH_LOCK_RAD

    controller = GraspModeController(hold_slew_rad_s=3.0)
    seed = np.linspace(-0.8, 0.8, len(SG100_JOINT_ORDER))
    controller.apply(seed, dt_s=1 / 60)
    controller.set_mode(GraspMode.PINCH)
    incoming = seed[::-1].copy()
    first = controller.apply(incoming, dt_s=1 / 60, pinch_closure=0.6)
    for i, name in enumerate(SG100_JOINT_ORDER):
        if name not in PINCH_ACTIVE_JOINTS:
            assert abs(first[i] - seed[i]) <= 3.0 / 60 + 1e-12
    for _ in range(120):
        output = controller.apply(incoming, dt_s=1 / 60, pinch_closure=0.6)
    for i, name in enumerate(SG100_JOINT_ORDER):
        if name not in PINCH_ACTIVE_JOINTS:
            assert output[i] == dict(PINCH_LOCK_RAD)[name]
    controller.set_mode(GraspMode.FREE)
    np.testing.assert_array_equal(controller.apply(incoming, dt_s=1 / 60), incoming)


def test_shared_pinch_input_does_not_change_existing_modes():
    for mode in (GraspMode.FREE, GraspMode.HAND, GraspMode.CLAW):
        incoming = np.linspace(-0.5, 0.5, len(SG100_JOINT_ORDER))
        normal = GraspModeController(initial_mode=mode)
        with_closure = GraspModeController(initial_mode=mode)
        np.testing.assert_array_equal(
            normal.apply(incoming, dt_s=1 / 60),
            with_closure.apply(incoming, dt_s=1 / 60, pinch_closure=0.8),
        )


def _viewer_key_handler(scope):
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
    module = ast.fix_missing_locations(ast.Module(body=[factory], type_ignores=[]))
    exec(compile(module, str(runner), "exec"), scope)
    return scope["make_handler"]()


def test_key_5_exits_script_source_and_selects_pinch():
    class ScriptStub:
        enabled = True
        def exit(self):
            self.enabled = False
    script = ScriptStub()
    controller = GraspModeController(initial_mode=GraspMode.CLAW)
    handler = _viewer_key_handler({
        "GraspMode": GraspMode, "grasp_mode_controller": controller,
        "script_hand_controller": script,
    })
    handler(ord("5"))
    assert controller.mode == GraspMode.PINCH
    assert not script.enabled
    handler(ord("1"))
    assert controller.mode == GraspMode.FREE


@pytest.mark.parametrize("source_key", "12345")
@pytest.mark.parametrize("target_key", "12345")
def test_c_then_a_allows_direct_switch_between_all_five_modes(source_key, target_key):
    from types import SimpleNamespace

    from sg100_teleop.control.scripted_hand import ScriptHandController
    from sg100_teleop.control.teleop_arming import TeleopArmingController

    arming = TeleopArmingController()
    arming.observe_valid_frame(1)
    calibration_requests = []

    def request_calibration():
        calibration_requests.append(1)
        return arming.request_neutral_calibration(requested_monotonic_ns=10)

    # Only the hardware boundary is stubbed: no serial port or robot is opened.
    class BackendStub:
        enabled = False
        anchored = False

        def __init__(self):
            self.enable_calls = []
            self.anchor_token = None

        def set_enabled(self, enabled):
            assert self.anchored
            self.enable_calls.append(enabled)
            self.enabled = enabled

    backend = BackendStub()
    controller = GraspModeController()
    script = ScriptHandController()
    seed = np.linspace(-0.4, 0.4, len(SG100_JOINT_ORDER))
    handler = _viewer_key_handler({
        "request_neutral_calibration": request_calibration,
        "real_arm_backend": None,
        "restricted_hand_backend": backend,
        "real_error_recorder": None,
        "GraspMode": GraspMode,
        "grasp_mode_controller": controller,
        "script_hand_controller": script,
        "hand_command_filter": SimpleNamespace(value=seed),
    })

    handler(ord("C"))
    assert not backend.enabled
    # C is asynchronous: the main tracking loop captures the next valid frame.
    assert arming.should_calibrate(2, received_monotonic_ns=11)
    backend.anchored = True
    backend.anchor_token = object()
    anchor = backend.anchor_token
    arming.complete_calibration()
    handler(ord("A"))
    assert backend.enabled

    for key in (source_key, target_key):
        handler(ord(key))
        if key == "4":
            assert script.enabled
            np.testing.assert_array_equal(script.step(0.0), seed)
        else:
            assert not script.enabled
            assert controller.mode == {
                "1": GraspMode.FREE,
                "2": GraspMode.HAND,
                "3": GraspMode.CLAW,
                "5": GraspMode.PINCH,
            }[key]
        assert backend.enabled
        assert backend.anchored
        assert backend.anchor_token is anchor
        assert arming.active
        assert calibration_requests == [1]
        assert backend.enable_calls == [True]
