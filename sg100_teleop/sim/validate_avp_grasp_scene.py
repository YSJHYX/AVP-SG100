"""Validate the AVP grasp scene XML.

Run from the repo root:

    python -m sg100_teleop.sim.validate_avp_grasp_scene

Loads ``rm65_sg100_avp_grasp_scene.xml`` and asserts that it is structurally
correct and dynamically sane: it compiles, gravity is on, the table/bottle and
all collision geoms exist, the 17-DoF robot layout still passes
``resolve_robot_layout`` (so it can later plug into ``run_avp_arm_hand_sim``),
the water bottle is a free rigid body, the collision bitmasks produce the
expected pairs, the fingers are force-limited, and the bottle actually rests
on the table under gravity.

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
# at load time. At this pose the hand sits clear of the bottle, so any motion of
# the bottle during the test is due to gravity + table contact, not the arm.
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
        n for n in ("water_bottle", "grasp_table", "sg100_base_link", "link_6")
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
        "water_bottle_collision",
        "grasp_table_collision",
    )
    missing = [n for n in required if _geom_id(model, n) < 0]
    assert not missing, f"missing geoms: {missing}"


def check_layout(model: mujoco.MjModel) -> None:
    layout = resolve_robot_layout(model)
    assert layout.qpos_indices.shape == (ARM_DOF + HAND_DOF,)
    assert layout.dof_indices.tolist() == list(range(ARM_DOF + HAND_DOF)), (
        "robot DoFs must occupy the first 17 slots so the bottle freejoint "
        "stays behind them"
    )


def check_bottle_freejoint(model: mujoco.MjModel) -> None:
    jid = _joint_id(model, "water_bottle_free")
    assert jid >= 0, "water_bottle_free joint missing"
    assert model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE, (
        "water_bottle_free must be a free joint"
    )
    assert int(model.jnt_dofadr[jid]) >= ARM_DOF + HAND_DOF, (
        "bottle freejoint DoF must start after the 17 robot DoFs"
    )


def check_collision_masks(model: mujoco.MjModel) -> None:
    # finger (incl. new link-2) <-> bottle : ON
    assert _can_collide(model, "index_link3_collision", "water_bottle_collision")
    assert _can_collide(model, "index_link2_collision", "water_bottle_collision")
    assert _can_collide(model, "palm_collision", "water_bottle_collision")
    # bottle <-> table : ON
    assert _can_collide(model, "water_bottle_collision", "grasp_table_collision")
    # finger <-> finger : OFF (no self-collision)
    assert not _can_collide(
        model, "index_link3_collision", "middle_link2_collision"
    ), "finger capsules must not collide with each other"
    # arm (link meshes) <-> table : ON ; <-> bottle : OFF ; <-> hand : OFF
    assert _can_collide(model, "rm65_visual_3", "grasp_table_collision"), (
        "arm links must collide with the table"
    )
    assert not _can_collide(
        model, "rm65_visual_3", "water_bottle_collision"
    ), "arm links must NOT collide with the bottle"
    assert not _can_collide(
        model, "rm65_visual_3", "index_link3_collision"
    ), "arm links must NOT collide with the hand"
    assert not _can_collide(
        model, "rm65_visual_2", "rm65_visual_5"
    ), "arm links must NOT self-collide"


def check_arm_collision_mask(model: mujoco.MjModel) -> None:
    """Arm link meshes get their own collision class (8 / 4): collide with the
    table only, not the bottle, hand, or other arm links."""
    for i in range(1, 7):
        name = f"rm65_visual_{i}"
        gid = _geom_id(model, name)
        assert gid >= 0, f"{name} geom missing"
        ct = int(model.geom_contype[gid])
        ca = int(model.geom_conaffinity[gid])
        assert (ct, ca) == (8, 4), (
            f"{name} collision class = ({ct}, {ca}), expected (8, 4) "
            "[collide with table only]"
        )


def check_finger_force_range(model: mujoco.MjModel) -> None:
    layout = resolve_robot_layout(model)
    for aid in layout.hand_actuator_ids:
        fr = model.actuator_forcerange[aid]
        assert np.allclose(fr, [-20.0, 20.0]), (
            f"hand actuator {aid} forcerange = {fr}, expected [-20, 20]"
        )


def check_dynamics_stability(model: mujoco.MjModel) -> None:
    """Step 1 s of physics; the bottle must rest on the table, not fall through."""
    layout = resolve_robot_layout(model)
    arm_dof = layout.dof_indices[:ARM_DOF]
    data = mujoco.MjData(model)

    # Start in the real AVP startup configuration: arm at the teleop-ready
    # pose, hand open, bottle at its tuned rest spot on the table.
    initialize_robot_state(model, data, layout, READY_Q, np.zeros(HAND_DOF))

    bottle_id = _body_id(model, "water_bottle")
    z0 = float(data.xpos[bottle_id][2])

    # Hold the arm with gravity-compensation feedforward (exactly what the AVP
    # script will do), so only the free bottle responds to gravity.
    max_ncon = 0
    for _ in range(500):  # 500 * 0.002 s = 1 s
        data.qfrc_applied[arm_dof] = data.qfrc_bias[arm_dof]
        mujoco.mj_step(model, data)
        max_ncon = max(max_ncon, int(data.ncon))

    z1 = float(data.xpos[bottle_id][2])
    assert np.isfinite(z1), f"bottle z is non-finite: {z1}"
    assert 0.09 < z1 < 0.13, (
        f"bottle did not rest on the table: z0={z0:.4f} z1={z1:.4f} "
        f"(max_ncon={max_ncon})"
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


CHECKS: list[tuple[str, Callable[[mujoco.MjModel], None]]] = [
    ("compiles (valid XML + mesh paths)", check_compiles),
    ("gravity == 0 0 -9.81", check_gravity),
    ("key bodies present", check_bodies),
    ("key geoms present (palm / link-2 / tips / env)", check_geoms),
    ("resolve_robot_layout passes (17 DoF contiguous)", check_layout),
    ("water_bottle is a free rigid body after robot DoFs", check_bottle_freejoint),
    ("collision bitmasks (finger-bottle ON, bottle-table ON, arm-table ON, "
     "rest OFF)", check_collision_masks),
    ("arm link collision class == (8, 4) [table only]", check_arm_collision_mask),
    ("hand actuator forcerange == [-20, 20]", check_finger_force_range),
    ("bottle rests on table under gravity (1 s sim)", check_dynamics_stability),
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
        if title.startswith("bottle rests"):
            extra = (
                f"  [z0={check_dynamics_stability.z0:.4f}, "
                f"z1={check_dynamics_stability.z1:.4f}, "
                f"max_ncon={check_dynamics_stability.max_ncon}, "
                f"arm_drift={check_dynamics_stability.arm_drift:.4f}]"
            )
        print(f"  PASS  {title}{extra}")

    if failures:
        print(f"\n{len(failures)} check(s) FAILED.")
        return 1
    print("\nOK — all checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
