"""Robot-contract and ready-pose regression for the demo scenes.

Every demo scene must keep the frozen V8 robot identical and must
start in the V8 grasp-ready posture via its ``demo_ready``
keyframe:

  * nu == 17 controllable DoF (6 RM65 + 11 SG100);
  * the 17 named joints keep their axis / range / parent body;
  * the 17 position actuators keep their names, joint bindings and
    ctrlranges;
  * the keyframe 17-D robot qpos equals the V8 runtime startup
    solution (by joint name, not by index);
  * link_6 and sg100_base_link FK agree with the V8 startup FK.

Joint zero semantics are untouched: joint ``ref`` values are
compared too, and only the keyframe carries the posture.
"""

from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
import pytest

from sg100_teleop.kinematics.rm65_ik import RM65IKSolver
from sg100_teleop.sim.combined_robot_layout import (
    HAND_DOF,
    ROBOT_JOINT_ORDER,
    resolve_robot_layout,
)
from sg100_teleop.sim.run_avp_arm_hand_sim import (
    TCP_BODY,
    compute_startup_arm_q,
)


ROOT = Path(__file__).resolve().parents[2]
MJCF = ROOT / "sg100_teleop/assets/rm65_sg100/mjcf"

V8_SCENE = MJCF / "rm65_sg100_avp_grasp_scene.xml"
DEMO_SCENES = {
    "demo01_hammer_peg_assembly": (
        MJCF / "rm65_sg100_demo_hammer_peg_assembly_scene.xml"
    ),
    "demo02_drawer_retrieval": (
        MJCF
        / "rm65_sg100_demo_drawer_retrieval_and_loading_scene.xml"
    ),
    "demo03_maintenance_kit_packing": (
        MJCF / "rm65_sg100_demo_maintenance_kit_packing_scene.xml"
    ),
}

FK_BODIES = ("link_6", "sg100_base_link")
QPOS_TOLERANCE = 1e-9
FK_POSITION_TOLERANCE = 1e-9
FK_ROTATION_TOLERANCE = 1e-9


def _joint_record(model: mujoco.MjModel) -> dict[str, dict]:
    records = {}
    for name in ROBOT_JOINT_ORDER:
        jid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        assert jid >= 0, f"robot joint missing: {name}"
        records[name] = {
            "axis": model.jnt_axis[jid].copy(),
            "range": model.jnt_range[jid].copy(),
            "ref": float(model.qpos0[model.jnt_qposadr[jid]]),
            "body": mujoco.mj_id2name(
                model,
                mujoco.mjtObj.mjOBJ_BODY,
                int(model.jnt_bodyid[jid]),
            ),
            "damping": float(model.dof_damping[
                model.jnt_dofadr[jid]
            ]),
            "armature": float(model.dof_armature[
                model.jnt_dofadr[jid]
            ]),
        }
        act = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_pos"
        )
        assert act >= 0, f"actuator missing: {name}_pos"
        records[name]["actuator_joint"] = int(
            model.actuator_trnid[act, 0]
        )
        records[name]["ctrlrange"] = (
            model.actuator_ctrlrange[act].copy()
        )
        records[name]["actuator_id"] = act
    return records


def _ready_qpos_by_name(model, data) -> np.ndarray:
    key = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_KEY, "demo_ready"
    )
    assert key >= 0, "demo_ready keyframe missing"
    mujoco.mj_resetDataKeyframe(model, data, key)
    layout = resolve_robot_layout(model)
    return data.qpos[layout.qpos_indices].copy()


@pytest.fixture(scope="module")
def v8_reference():
    model = mujoco.MjModel.from_xml_path(str(V8_SCENE))
    data = mujoco.MjData(model)
    solver = RM65IKSolver(model, tcp_body_name=TCP_BODY)
    startup_q = compute_startup_arm_q(model, data, solver)
    assert startup_q is not None, (
        "V8 startup IK failed - reference unavailable"
    )
    q17 = np.concatenate(
        [startup_q, np.zeros(HAND_DOF)]
    )
    layout = resolve_robot_layout(model)
    data.qpos[:] = model.qpos0
    data.qpos[layout.qpos_indices] = q17
    mujoco.mj_forward(model, data)
    fk = {
        body: (
            data.xpos[
                mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_BODY, body
                )
            ].copy(),
            data.xmat[
                mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_BODY, body
                )
            ].reshape(3, 3).copy(),
        )
        for body in FK_BODIES
    }
    return {
        "joints": _joint_record(model),
        "q17": q17,
        "fk": fk,
    }


@pytest.mark.parametrize("demo_name", sorted(DEMO_SCENES))
def test_demo_nu_is_17(demo_name):
    model = mujoco.MjModel.from_xml_path(
        str(DEMO_SCENES[demo_name])
    )
    assert model.nu == 17


@pytest.mark.parametrize("demo_name", sorted(DEMO_SCENES))
def test_demo_robot_joint_and_actuator_contract(
    demo_name, v8_reference
):
    model = mujoco.MjModel.from_xml_path(
        str(DEMO_SCENES[demo_name])
    )
    demo = _joint_record(model)
    v8 = v8_reference["joints"]

    for name in ROBOT_JOINT_ORDER:
        d, v = demo[name], v8[name]
        assert d["body"] == v["body"], (
            f"{demo_name}/{name}: body {d['body']} != {v['body']}"
        )
        assert np.array_equal(d["axis"], v["axis"]), (
            f"{demo_name}/{name}: axis changed"
        )
        assert np.array_equal(d["range"], v["range"]), (
            f"{demo_name}/{name}: range changed"
        )
        assert d["ref"] == pytest.approx(v["ref"], abs=0.0), (
            f"{demo_name}/{name}: joint zero semantics (ref) changed"
        )
        assert d["damping"] == pytest.approx(v["damping"]), (
            f"{demo_name}/{name}: damping changed"
        )
        assert d["armature"] == pytest.approx(v["armature"]), (
            f"{demo_name}/{name}: armature changed"
        )
        assert d["actuator_joint"] == mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name
        ), f"{demo_name}/{name}: actuator binding changed"
        assert np.array_equal(d["ctrlrange"], v["ctrlrange"]), (
            f"{demo_name}/{name}: ctrlrange changed"
        )


@pytest.mark.parametrize("demo_name", sorted(DEMO_SCENES))
def test_demo_ready_pose_matches_v8(demo_name, v8_reference):
    model = mujoco.MjModel.from_xml_path(
        str(DEMO_SCENES[demo_name])
    )
    data = mujoco.MjData(model)
    q17 = _ready_qpos_by_name(model, data)
    reference = v8_reference["q17"]

    err = float(np.max(np.abs(q17 - reference)))
    assert err <= QPOS_TOLERANCE, (
        f"{demo_name}: demo_ready 17-D qpos max error {err}"
    )


@pytest.mark.parametrize("demo_name", sorted(DEMO_SCENES))
def test_demo_ready_fk_matches_v8(demo_name, v8_reference):
    model = mujoco.MjModel.from_xml_path(
        str(DEMO_SCENES[demo_name])
    )
    data = mujoco.MjData(model)
    _ready_qpos_by_name(model, data)
    mujoco.mj_forward(model, data)

    for body, (ref_pos, ref_mat) in v8_reference["fk"].items():
        bid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, body
        )
        pos_err = float(
            np.max(np.abs(data.xpos[bid] - ref_pos))
        )
        mat_err = float(
            np.max(
                np.abs(
                    data.xmat[bid].reshape(3, 3) - ref_mat
                )
            )
        )
        assert pos_err <= FK_POSITION_TOLERANCE, (
            f"{demo_name}/{body}: FK position error {pos_err}"
        )
        assert mat_err <= FK_ROTATION_TOLERANCE, (
            f"{demo_name}/{body}: FK rotation error {mat_err}"
        )


@pytest.mark.parametrize("demo_name", sorted(DEMO_SCENES))
def test_demo_model_report(demo_name):
    """PART 66 model report: dimension summary stays sane."""
    model = mujoco.MjModel.from_xml_path(
        str(DEMO_SCENES[demo_name])
    )
    assert model.nu == 17
    assert model.ncam == 3
    assert model.nkey == 1
    # robot qpos block is identical 17 in every scene; the rest is
    # task state (free/slide joints)
    resolve_robot_layout(model)
