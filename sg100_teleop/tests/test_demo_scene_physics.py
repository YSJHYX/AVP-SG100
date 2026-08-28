"""Physics validation for the demo scenes (PART 69-72).

Per scene, from the ``demo_ready`` keyframe with the robot held by
exact bias compensation (the offline equivalent of the V8 runtime
gravity feedforward):

  * 3 s unactuated: no NaN, no explosion, no floor penetration,
    no spontaneous task-state change (peg / drawer / objects);
  * Demo 01: peg holds under gravity; a scripted mallet tap drives
    it past the success threshold within three strikes;
  * Demo 02: drawer holds closed, opens fully under a 25 N pull
    with the arm parked clear, and holds the opened position;
  * Demo 03: the three kit objects rest still, and gentle placement
    into the zones satisfies the success criteria.
"""

from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
import pytest

from sg100_teleop.sim.combined_robot_layout import (
    resolve_robot_layout,
)
from sg100_teleop.sim.demo_task_validation import (
    V8_READY_ARM_Q,
    evaluate_demo01,
    evaluate_demo02,
    evaluate_demo03,
)


ROOT = Path(__file__).resolve().parents[2]
MJCF = ROOT / "sg100_teleop/assets/rm65_sg100/mjcf"

DEMO_SCENES = {
    "01": MJCF / "rm65_sg100_demo_hammer_peg_assembly_scene.xml",
    "02": (
        MJCF
        / "rm65_sg100_demo_drawer_retrieval_and_loading_scene.xml"
    ),
    "03": (
        MJCF
        / "rm65_sg100_demo_maintenance_kit_packing_scene.xml"
    ),
}

PARK_ARM_Q = np.array(
    [
        0.32488,
        -0.76769,
        0.97621,
        0.22463,
        -1.80649,
        -0.12430,
    ]
)

TABLE_XY = ((-0.15, 0.75), (-0.35, 0.65))


def _reset(model, arm_q=V8_READY_ARM_Q):
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_KEY, "demo_ready"
    )
    assert key >= 0
    mujoco.mj_resetDataKeyframe(model, data, key)
    layout = resolve_robot_layout(model)
    data.ctrl[layout.arm_actuator_ids] = arm_q
    data.ctrl[layout.hand_actuator_ids] = 0.0
    return data, layout


def _hold_and_step(model, data, layout, steps, extra_force=None):
    for _ in range(steps):
        mujoco.mj_forward(model, data)
        data.qfrc_applied[:] = 0.0
        data.qfrc_applied[layout.dof_indices] = -data.qfrc_bias[
            layout.dof_indices
        ]
        if extra_force is not None:
            extra_force(data)
        mujoco.mj_step(model, data)


def _qadr(model, joint):
    jid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, joint
    )
    assert jid >= 0, joint
    return int(model.jnt_qposadr[jid])


def _assert_no_explosion(model, data):
    assert np.all(np.isfinite(data.qpos)), "NaN in qpos"
    for body_id in range(1, model.nbody):
        pos = data.xpos[body_id]
        assert -0.30 < pos[2] < 1.20, (
            f"body {model.body(body_id).name} exploded: {pos}"
        )


@pytest.mark.parametrize("demo", sorted(DEMO_SCENES))
def test_static_stability_3s(demo):
    """PART 69: unactuated 3 s from the task initial state."""
    model = mujoco.MjModel.from_xml_path(str(DEMO_SCENES[demo]))
    data, layout = _reset(model)

    task_state0 = data.qpos[17:].copy()
    for _ in range(1500):
        _hold_and_step(model, data, layout, 1)

    _assert_no_explosion(model, data)
    mujoco.mj_forward(model, data)

    # no task object below the table surface (penetration guard)
    for body, name in (
        ("demo_hammer", "01"),
        ("demo_drawer_item", "02"),
        ("kit_object_a", "03"),
        ("kit_object_b", "03"),
        ("kit_object_c", "03"),
    ):
        if demo != name:
            continue
        bid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, body
        )
        assert data.xpos[bid][2] > -0.01, (
            f"{body} penetrated the table"
        )
        assert (
            TABLE_XY[0][0] - 0.05
            <= data.xpos[bid][0]
            <= TABLE_XY[0][1] + 0.05
        ), f"{body} left the table"

    # no spontaneous task-state drift beyond the settle tolerance
    drift = float(
        np.max(np.abs(data.qpos[17:] - task_state0))
    )
    assert drift < 0.004, (
        f"{demo}: task state drifted {drift * 1000:.2f} mm in 3 s"
    )


def test_demo01_peg_holds_and_strikes():
    model = mujoco.MjModel.from_xml_path(str(DEMO_SCENES["01"]))
    peg_q = _qadr(model, "demo_peg_slide")
    hammer_j = _qadr(model, "demo_hammer_free")

    # gravity hold: 3 s, no contact -> < 1 mm
    data, layout = _reset(model)
    for _ in range(1500):
        _hold_and_step(model, data, layout, 1)
    assert abs(data.qpos[peg_q]) < 0.001

    # three controlled 0.5 m/s taps drive the peg home
    data, layout = _reset(model)
    for _ in range(3):
        data.qpos[hammer_j:hammer_j + 7] = [
            0.415, 0.209, 0.152, 1, 0, 0, 0,
        ]
        data.qvel[hammer_j:hammer_j + 6] = 0.0
        data.qvel[hammer_j + 2] = -0.5
        _hold_and_step(model, data, layout, 300)
        assert np.all(np.isfinite(data.qpos))
    mujoco.mj_forward(model, data)
    report = evaluate_demo01(model, data)
    assert report["peg_success"], report
    assert report["hammer_on_table"], report


def test_demo02_drawer_cycle():
    model = mujoco.MjModel.from_xml_path(str(DEMO_SCENES["02"]))
    drawer_q = _qadr(model, "demo_drawer_slide")
    drawer_dof = int(
        model.jnt_dofadr[
            mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT,
                "demo_drawer_slide",
            )
        ]
    )

    # closed stability
    data, layout = _reset(model)
    for _ in range(1500):
        _hold_and_step(model, data, layout, 1)
    assert abs(data.qpos[drawer_q]) < 0.0015, "drawer self-opened"

    # park the arm clear (operator moves the hand before pulling),
    # then pull with 25 N along the joint axis for 1 s
    data, layout = _reset(model, arm_q=PARK_ARM_Q)
    _hold_and_step(model, data, layout, 1000)

    def pull(_data):
        _data.qfrc_applied[drawer_dof] = 25.0

    _hold_and_step(model, data, layout, 500, extra_force=pull)
    opened = float(data.qpos[drawer_q])
    assert opened > 0.145, f"drawer only opened {opened * 1000:.1f} mm"

    # release: holds, no spring-back
    _hold_and_step(model, data, layout, 500)
    held = float(data.qpos[drawer_q])
    assert held > 0.130, f"drawer slid back to {held * 1000:.1f} mm"

    # item rode along, is exposed and reachable inside the workspace
    mujoco.mj_forward(model, data)
    report = evaluate_demo02(model, data)
    assert report["stage1_drawer_open"], report
    assert 0.35 < report["item_position_m"][0] < 0.60


def test_demo03_objects_rest_and_pack():
    model = mujoco.MjModel.from_xml_path(str(DEMO_SCENES["03"]))
    qa = _qadr(model, "kit_object_a_free")
    qb = _qadr(model, "kit_object_b_free")
    qc = _qadr(model, "kit_object_c_free")

    # objects rest still on the table
    data, layout = _reset(model)
    start = data.qpos[17:].copy()
    for _ in range(1500):
        _hold_and_step(model, data, layout, 1)
    drift = float(np.max(np.abs(data.qpos[17:] - start)))
    assert drift < 0.004

    # gentle zone placement satisfies the task success criteria
    data, layout = _reset(model)
    data.qpos[qa:qa + 7] = [0.32, -0.12, 0.0125, 1, 0, 0, 0]
    data.qpos[qb:qb + 7] = [0.515, -0.115, 0.0125, 1, 0, 0, 0]
    data.qpos[qc:qc + 7] = [0.42, -0.295, 0.004, 1, 0, 0, 0]
    for _ in range(1500):
        _hold_and_step(model, data, layout, 1)
    mujoco.mj_forward(model, data)
    report = evaluate_demo03(model, data)
    assert report["task_success"], report
