"""Validate the AVP grasp scene XML.

Run from the repo root:

    python -m sg100_teleop.sim.validate_avp_grasp_scene

Loads ``rm65_sg100_avp_grasp_scene.xml`` and asserts that it is structurally
correct and dynamically sane: it compiles, gravity is on, the table, three
production task objects, and open crate collision geoms exist, the 17-DoF
robot layout still passes
``resolve_robot_layout`` (so it can later plug into ``run_avp_arm_hand_sim``),
the task objects are free rigid bodies, the collision bitmasks produce the
expected pairs, the fingers are force-limited, and all three objects rest on
the table under gravity.

Each check prints PASS / FAIL; any failure exits non-zero.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Callable

import mujoco
import numpy as np

from sg100_teleop.sim.combined_robot_layout import (
    ARM_DOF,
    HAND_DOF,
    initialize_robot_state,
    resolve_robot_layout,
)

# Mirrors RM65_TELEOP_READY_Q in run_avp_arm_hand_sim.py. Inlined (not imported)
# so this validator stays free of the AVP/hardware imports that module pulls in
# at load time. At this pose the hand sits clear of the props, so any motion of
# the props during the test is due to gravity + table contact, not the arm.
READY_Q = np.array(
    [0.32488, -0.76769, 0.97621, 0.22463, -1.80649, -0.12430],
    dtype=np.float64,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_PATH = (
    REPO_ROOT
    / "sg100_teleop"
    / "assets"
    / "rm65_sg100"
    / "mjcf"
    / "rm65_sg100_avp_grasp_scene.xml"
)

GRAVITY = np.array([0.0, 0.0, -9.81], dtype=np.float64)


def _body_id(model: mujoco.MjModel, name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def _geom_id(model: mujoco.MjModel, name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)


def _joint_id(model: mujoco.MjModel, name: str) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)


def _can_collide(model: mujoco.MjModel, ga: str, gb: str) -> bool:
    """Replicate MuJoCo's contype/conaffinity collision test for a geom pair."""
    ia, ib = _geom_id(model, ga), _geom_id(model, gb)
    if ia < 0 or ib < 0:
        raise AssertionError(f"missing geom for collision test: {ga!r}/{gb!r}")
    ca, aa = int(model.geom_contype[ia]), int(model.geom_conaffinity[ia])
    cb, ab = int(model.geom_contype[ib]), int(model.geom_conaffinity[ib])
    return bool((ca & ab) | (cb & aa))


def check_compiles(_: mujoco.MjModel) -> None:
    # The model was already loaded before this check runs; reaching here means
    # from_xml_path succeeded (valid XML + resolvable mesh paths).
    return


def check_gravity(model: mujoco.MjModel) -> None:
    assert np.allclose(model.opt.gravity, GRAVITY), (
        f"gravity = {model.opt.gravity}, expected {GRAVITY}"
    )


def check_bodies(model: mujoco.MjModel) -> None:
    missing = [
        n for n in (
            "obj_coke", "obj_apple", "obj_mug", "drop_bin",
            "grasp_table", "sg100_base_link", "link_6",
        )
        if _body_id(model, n) < 0
    ]
    assert not missing, f"missing bodies: {missing}"


def check_geoms(model: mujoco.MjModel) -> None:
    required = (
        # palm backstop (new)
        "palm_collision",
        # link-2 collision capsules (new)
        "thumb_link2_collision",
        "index_link2_collision",
        "little_llink2_collision",
        # fingertip / middle collision (carried over)
        "thumb_link3_collision",
        "index_link3_collision",
        "middle_link2_collision",
        "little_linik3_collision",
        # environment
        "obj_coke_collision",
        "obj_apple_collision",
        "mug_coll_0",
        "wood_crate_bottom",
        "wood_crate_wall_pX",
        "grasp_table_collision",
    )
    missing = [n for n in required if _geom_id(model, n) < 0]
    assert not missing, f"missing geoms: {missing}"


def check_layout(model: mujoco.MjModel) -> None:
    layout = resolve_robot_layout(model)
    assert layout.qpos_indices.shape == (ARM_DOF + HAND_DOF,)
    assert layout.dof_indices.tolist() == list(range(ARM_DOF + HAND_DOF)), (
        "robot DoFs must occupy the first 17 slots so prop freejoints stay "
        "behind them"
    )


def check_task_object_freejoints(model: mujoco.MjModel) -> None:
    for name in ("obj_coke", "obj_apple", "obj_mug"):
        jid = _joint_id(model, f"{name}_free")
        assert jid >= 0, f"{name}_free joint missing"
        assert model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE
        assert int(model.jnt_dofadr[jid]) >= ARM_DOF + HAND_DOF


def check_collision_masks(model: mujoco.MjModel) -> None:
    object_geoms = (
        "obj_coke_collision",
        "obj_apple_collision",
        *(f"mug_coll_{index}" for index in range(32)),
    )
    # The unified task-object class is 2/11.  It accepts finger bit 1,
    # object bit 2, and RM65 bit 8 while keeping visual geoms at 0/0.
    for object_geom in object_geoms:
        object_id = _geom_id(model, object_geom)
        assert (
            int(model.geom_contype[object_id]),
            int(model.geom_conaffinity[object_id]),
        ) == (2, 11)
        assert _can_collide(model, "index_link3_collision", object_geom)
        assert _can_collide(model, "index_link2_collision", object_geom)
        assert _can_collide(model, "palm_collision", object_geom)
        assert _can_collide(model, "rm65_visual_3", object_geom)
        assert _can_collide(model, object_geom, "grasp_table_collision")
        assert _can_collide(model, object_geom, "wood_crate_bottom")
    assert _can_collide(model, "obj_coke_collision", "obj_apple_collision")
    assert _can_collide(model, "obj_coke_collision", "mug_coll_0")
    assert _can_collide(model, "obj_apple_collision", "mug_coll_0")
    # finger <-> finger : OFF (no self-collision)
    assert not _can_collide(
        model, "index_link3_collision", "middle_link2_collision"
    ), "finger capsules must not collide with each other"
    # arm (link meshes) <-> table/objects: ON; <-> hand/arm: OFF.
    assert _can_collide(model, "rm65_visual_3", "grasp_table_collision"), (
        "arm links must collide with the table"
    )
    assert _can_collide(
        model, "rm65_visual_3", "obj_coke_collision"
    ), "arm links must collide with task objects"
    assert not _can_collide(
        model, "rm65_visual_3", "index_link3_collision"
    ), "arm links must NOT collide with the hand"
    assert not _can_collide(
        model, "rm65_visual_2", "rm65_visual_5"
    ), "arm links must NOT self-collide"


def check_arm_collision_mask(model: mujoco.MjModel) -> None:
    """Arm links retain 8/4; objects opt into that class via affinity 11."""
    for i in range(1, 7):
        name = f"rm65_visual_{i}"
        gid = _geom_id(model, name)
        assert gid >= 0, f"{name} geom missing"
        ct = int(model.geom_contype[gid])
        ca = int(model.geom_conaffinity[gid])
        assert (ct, ca) == (8, 4), (
            f"{name} collision class = ({ct}, {ca}), expected (8, 4) "
            "[unchanged production arm class]"
        )


def check_finger_force_range(model: mujoco.MjModel) -> None:
    layout = resolve_robot_layout(model)
    for aid in layout.hand_actuator_ids:
        fr = model.actuator_forcerange[aid]
        assert np.allclose(fr, [-20.0, 20.0]), (
            f"hand actuator {aid} forcerange = {fr}, expected [-20, 20]"
        )


def check_dynamics_stability(model: mujoco.MjModel) -> None:
    """Step 1 s; all production props must settle without falling through."""
    layout = resolve_robot_layout(model)
    arm_dof = layout.dof_indices[:ARM_DOF]
    data = mujoco.MjData(model)

    # Start in the real AVP startup configuration: arm at the teleop-ready
    # pose, hand open, props at their authored table placements.
    initialize_robot_state(model, data, layout, READY_Q, np.zeros(HAND_DOF))

    object_ids = {
        name: _body_id(model, name) for name in ("obj_coke", "obj_apple", "obj_mug")
    }
    z0 = {name: float(data.xpos[body][2]) for name, body in object_ids.items()}

    # Hold the arm with gravity-compensation feedforward (exactly what the AVP
    # script will do), so only the free props respond to gravity.
    max_ncon = 0
    for _ in range(500):  # 500 * 0.002 s = 1 s
        data.qfrc_applied[arm_dof] = data.qfrc_bias[arm_dof]
        mujoco.mj_step(model, data)
        max_ncon = max(max_ncon, int(data.ncon))

    z1 = {name: float(data.xpos[body][2]) for name, body in object_ids.items()}
    for name, z in z1.items():
        assert np.isfinite(z), f"{name} z is non-finite: {z}"
        assert z > -0.005, f"{name} fell through table: z0={z0[name]:.4f}, z1={z:.4f}"
        assert abs(z - z0[name]) < 0.015, (
            f"{name} did not settle near its authored pose: "
            f"z0={z0[name]:.4f}, z1={z:.4f}, max_ncon={max_ncon}"
        )

    # Arm must stay near the ready pose. With arm<->table collision now on, a
    # ready pose that penetrates the table would be kicked and explode here.
    arm_q = data.qpos[layout.arm_qpos_indices]
    arm_drift = float(np.max(np.abs(arm_q - READY_Q))) if arm_q.size else 0.0
    assert np.all(np.isfinite(arm_q)), "arm qpos non-finite (instability)"
    assert arm_drift < 0.3, (
        f"arm drifted {arm_drift:.3f} rad from ready pose under gravity + "
        f"contacts (likely arm<->table penetration at READY_Q)"
    )

    # Stash diagnostics for the per-check printout.
    check_dynamics_stability.z0 = z0  # type: ignore[attr-defined]
    check_dynamics_stability.z1 = z1  # type: ignore[attr-defined]
    check_dynamics_stability.max_ncon = max_ncon  # type: ignore[attr-defined]
    check_dynamics_stability.arm_drift = arm_drift  # type: ignore[attr-defined]


def check_crate_contact_stability(model: mujoco.MjModel) -> None:
    """Drop the apple into the crate and verify bottom contact is stable."""
    layout = resolve_robot_layout(model)
    data = mujoco.MjData(model)
    initialize_robot_state(model, data, layout, READY_Q, np.zeros(HAND_DOF))
    joint = _joint_id(model, "obj_apple_free")
    qadr = int(model.jnt_qposadr[joint])
    data.qpos[qadr : qadr + 7] = [0.55, -0.15, 0.12, 1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    arm_dof = layout.dof_indices[:ARM_DOF]
    apple_geom = _geom_id(model, "obj_apple_collision")
    crate_geoms = {
        _geom_id(model, name)
        for name in (
            "wood_crate_bottom", "wood_crate_wall_pX", "wood_crate_wall_nX",
            "wood_crate_wall_pY", "wood_crate_wall_nY",
        )
    }
    crate_contact = False
    for _ in range(750):
        data.qfrc_applied[arm_dof] = data.qfrc_bias[arm_dof]
        mujoco.mj_step(model, data)
        crate_contact |= any(
            apple_geom in (contact.geom1, contact.geom2)
            and bool({contact.geom1, contact.geom2} & crate_geoms)
            for contact in data.contact
        )
    apple = _body_id(model, "obj_apple")
    position = data.xpos[apple]
    assert crate_contact, "apple never contacted crate bottom/walls"
    assert np.linalg.norm(position[:2] - [0.55, -0.15]) < 0.03
    assert 0.01 < float(position[2]) < 0.03
    assert np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))
    check_crate_contact_stability.position = position.copy()  # type: ignore[attr-defined]


CHECKS: list[tuple[str, Callable[[mujoco.MjModel], None]]] = [
    ("compiles (valid XML + mesh paths)", check_compiles),
    ("gravity == 0 0 -9.81", check_gravity),
    ("key bodies present", check_bodies),
    ("key geoms present (palm / link-2 / tips / env)", check_geoms),
    ("resolve_robot_layout passes (17 DoF contiguous)", check_layout),
    ("three task objects are free rigid bodies after robot DoFs", check_task_object_freejoints),
    ("collision bitmasks (object-object/finger/arm/table/crate ON; robot self OFF)",
     check_collision_masks),
    ("arm link collision class remains (8, 4)", check_arm_collision_mask),
    ("hand actuator forcerange == [-20, 20]", check_finger_force_range),
    ("three task objects rest on table under gravity (1 s sim)", check_dynamics_stability),
    ("apple settles inside open crate without instability", check_crate_contact_stability),
]


def main() -> int:
    print(f"scene: {SCENE_PATH}")
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))  # raises if invalid

    failures: list[str] = []
    for title, fn in CHECKS:
        try:
            fn(model)
        except AssertionError as exc:
            print(f"  FAIL  {title}  ({exc})")
            failures.append(title)
            continue
        extra = ""
        if title.startswith("three task objects rest"):
            extra = (
                f"  [z0={check_dynamics_stability.z0}, "
                f"z1={check_dynamics_stability.z1}, "
                f"max_ncon={check_dynamics_stability.max_ncon}, "
                f"arm_drift={check_dynamics_stability.arm_drift:.4f}]"
            )
        elif title.startswith("apple settles"):
            extra = f"  [position={check_crate_contact_stability.position}]"
        print(f"  PASS  {title}{extra}")

    if failures:
        print(f"\n{len(failures)} check(s) FAILED.")
        return 1
    print("\nOK — all checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
