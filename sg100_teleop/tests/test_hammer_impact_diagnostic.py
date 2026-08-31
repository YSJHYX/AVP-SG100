"""Unit tests for the non-invasive Hammer impact diagnostics."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

from sg100_teleop.sim.combined_robot_layout import resolve_robot_layout
from sg100_teleop.sim.hammer_impact_diagnostic import (
    HAMMER_HEAD_GEOM,
    HammerImpactDiagnostic,
    contact_matches_pair,
    integrate_normal_impulse,
    object_velocity,
    peg_displacement,
)


ROOT = Path(__file__).resolve().parents[2]
SCENE = (
    ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/"
    "rm65_sg100_demo_hammer_peg_assembly_scene.xml"
)


def _model_data():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "demo_ready")
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)
    return model, data


def test_correct_impact_geom_pair_detected_and_order_independent():
    contact = SimpleNamespace(geom1=12, geom2=34)
    assert contact_matches_pair(contact, 12, 34)
    contact.geom1, contact.geom2 = contact.geom2, contact.geom1
    assert contact_matches_pair(contact, 12, 34)


def test_unrelated_contact_ignored():
    assert not contact_matches_pair(SimpleNamespace(geom1=12, geom2=99), 12, 34)
    assert not contact_matches_pair(SimpleNamespace(geom1=12, geom2=12), 12, 34)


def test_hammer_head_velocity_is_finite():
    model, data = _model_data()
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, HAMMER_HEAD_GEOM)
    linear, angular = object_velocity(
        model, data, mujoco.mjtObj.mjOBJ_GEOM, geom
    )
    assert linear.shape == (3,)
    assert angular.shape == (3,)
    assert np.all(np.isfinite(linear))
    assert np.all(np.isfinite(angular))


def test_hammer_head_velocity_uses_rot_lin_order_and_geom_offset():
    model, data = _model_data()
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "demo_hammer")
    joint = int(model.body_jntadr[body])
    dof = int(model.jnt_dofadr[joint])
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, HAMMER_HEAD_GEOM)
    body_linear = np.array([0.1, 0.2, 0.3])
    body_angular = np.array([0.0, 0.0, 2.0])
    data.qvel[dof : dof + 3] = body_linear
    data.qvel[dof + 3 : dof + 6] = body_angular
    mujoco.mj_forward(model, data)
    linear, angular = object_velocity(
        model, data, mujoco.mjtObj.mjOBJ_GEOM, geom
    )
    body_frame_linear, body_frame_angular = object_velocity(
        model, data, mujoco.mjtObj.mjOBJ_BODY, body
    )
    expected = body_frame_linear + np.cross(
        body_frame_angular, data.geom_xpos[geom] - data.xipos[body]
    )
    assert np.linalg.norm(body_frame_linear) > 0.0
    assert np.linalg.norm(body_frame_angular) > 0.0
    assert np.allclose(angular, body_frame_angular, atol=1e-12)
    assert np.allclose(linear, expected, atol=1e-12)


def test_peg_displacement_sign_and_magnitude():
    displacement_m, displacement_mm = peg_displacement(0.010, 0.026)
    assert np.isclose(displacement_m, 0.016)
    assert np.isclose(displacement_mm, 16.0)
    reverse_m, reverse_mm = peg_displacement(0.026, 0.010)
    assert reverse_m < 0.0
    assert np.isclose(reverse_mm, -16.0)


def test_impulse_integration_uses_dt():
    assert np.isclose(integrate_normal_impulse([10.0, 20.0, 30.0], 0.002), 0.12)


def test_no_contact_episode_does_not_crash_or_write(tmp_path):
    model, data = _model_data()
    layout = resolve_robot_layout(model)
    diagnostic = HammerImpactDiagnostic(model, tmp_path, print_events=False)
    diagnostic.sample(data, data.qpos[layout.arm_qpos_indices])
    assert diagnostic.close() is None
    assert diagnostic.event_count == 0
    assert not list(tmp_path.glob("impact_*"))


def test_real_contact_force_and_trace_are_finite(tmp_path):
    model, data = _model_data()
    layout = resolve_robot_layout(model)
    hammer_body = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "demo_hammer"
    )
    joint = int(model.body_jntadr[hammer_body])
    qadr = int(model.jnt_qposadr[joint])
    dadr = int(model.jnt_dofadr[joint])
    data.qpos[qadr : qadr + 7] = [0.415, 0.209, 0.152, 1, 0, 0, 0]
    data.qvel[dadr : dadr + 6] = 0.0
    data.qvel[dadr + 2] = -0.5
    mujoco.mj_forward(model, data)
    diagnostic = HammerImpactDiagnostic(
        model, tmp_path, post_frames=5, print_events=False
    )
    q_goal = data.qpos[layout.arm_qpos_indices].copy()
    for _ in range(300):
        mujoco.mj_forward(model, data)
        data.qfrc_applied[:] = 0.0
        data.qfrc_applied[layout.dof_indices] = -data.qfrc_bias[
            layout.dof_indices
        ]
        mujoco.mj_step(model, data)
        diagnostic.sample(data, q_goal)
        if diagnostic.events:
            break
    diagnostic.close()
    assert diagnostic.events
    event = diagnostic.events[0]
    assert np.isfinite(event["peak_normal_force"])
    assert event["peak_normal_force"] > 0.0
    assert np.isfinite(event["estimated_normal_impulse_Ns"])
    assert event["head_normal_impact_velocity"] > 0.0
    assert Path(event["trace_csv"]).is_file()
    trace = np.load(event["trace_npz"])
    assert np.all(np.isfinite(trace["hammer_head_vel_xyz"]))
    assert set(np.unique(trace["contact_active"])).issubset({0, 1})
    assert any(
        "hammer_head_collision<->peg_strike_cap" in str(pair)
        or "peg_strike_cap<->hammer_head_collision" in str(pair)
        for pair in trace["contact_target_geom_pairs"]
    )


def test_diagnostics_flags_are_disabled_by_default():
    from sg100_teleop.sim.run_avp_arm_hand_sim import build_arg_parser

    args = build_arg_parser().parse_args(["--ip", "127.0.0.1"])
    assert args.hammer_impact_diagnostic is False
    assert args.show_hammer_collision_debug is False
