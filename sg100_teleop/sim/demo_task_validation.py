"""Offline task-success metrics and IK reachability audit for the
V8 demo scene suite.

This module is deliberately decoupled from every control runner: it
loads a demo scene MJCF, reads its ``demo_ready`` keyframe (or a
caller-supplied state), and evaluates the per-demo success criteria
defined in DEMO_SCENES.md.  It also runs a position-only IK audit of
each demo's task waypoints using the production RM65 solver - the
same 4-DoF task the AVP wrist-roll teleop mode drives, so "reachable"
here means wrist-roll-teleop-reachable (orientation free apart from
the unconstrained roll), not full-pose reachable.

Usage::

    python -m sg100_teleop.sim.demo_task_validation --demo 01
    python -m sg100_teleop.sim.demo_task_validation --demo 02 \\
        --qpos state.npy            # evaluate a recorded state
    python -m sg100_teleop.sim.demo_task_validation --demo 03 --ik-audit
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from sg100_teleop.kinematics.rm65_ik import RM65IKSolver
from sg100_teleop.sim.combined_robot_layout import (
    resolve_robot_layout,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MJCF_DIR = (
    REPOSITORY_ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf"
)

DEMO_SCENES = {
    "01": "rm65_sg100_demo_hammer_peg_assembly_scene.xml",
    "02": "rm65_sg100_demo_drawer_loading_and_closing_scene.xml",
    "03": "rm65_sg100_demo_maintenance_kit_packing_scene.xml",
}

# V8 grasp-ready arm solution (runtime startup IK result for the
# frozen V8 scene, identical robot + table in every demo scene).
V8_READY_ARM_Q = np.array(
    [
        0.34818753987135775,
        -1.3614850477620644,
        -2.286025521790937,
        -0.10246395780782178,
        2.0744881064029075,
        -0.049588596290682636,
    ],
    dtype=np.float64,
)


# ------------------------------------------------------------------
# Success thresholds (mirrored in DEMO_SCENES.md)
# ------------------------------------------------------------------

PEG_TRAVEL_M = 0.040
PEG_SUCCESS_TRAVEL_M = 0.030          # >= 75 % insertion
DRAWER_TRAVEL_M = 0.150
DRAWER_OPEN_THRESHOLD_M = 0.120      # >= 80 % open
DRAWER_CLOSE_THRESHOLD_M = 0.030     # <= 20 % travel = closed
TRAY_CENTRE_XY = (0.33, -0.22)
TRAY_HALF_XY = (0.105, 0.075)
ZONE_A_CENTRE_XY = (0.32, -0.12)      # box zone (world)
ZONE_A_HALF_XY = (0.090, 0.060)
ZONE_B_CENTRE_XY = (0.42, -0.295)     # screwdriver slot (world)
ZONE_B_HALF_XY = (0.160, 0.044)
ZONE_C_YAW_TOLERANCE_DEG = 20.0


def _joint_qposadr(model: mujoco.MjModel, joint: str) -> int:
    jid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, joint
    )
    if jid < 0:
        raise KeyError(f"joint not found in demo model: {joint}")
    return int(model.jnt_qposadr[jid])


def _body_xpos_of(model, data, body: str) -> np.ndarray:
    bid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, body
    )
    if bid < 0:
        raise KeyError(f"body not found in demo model: {body}")
    return data.xpos[bid].copy()


def _twist_yaw_deg(quat_wxyz: np.ndarray) -> float:
    """Signed twist of a world-frame quaternion about world +Z."""
    w, x, y, z = (float(v) for v in quat_wxyz)
    proj = z
    twist_norm = np.hypot(w, proj)
    if twist_norm < 1e-12:
        return 0.0
    yaw = 2.0 * np.arctan2(proj, w)
    return float(np.degrees(np.arctan2(np.sin(yaw), np.cos(yaw))))


# ------------------------------------------------------------------
# Demo 01 - Hammer Peg Assembly
# ------------------------------------------------------------------

def evaluate_demo01(model, data) -> dict:
    peg_q = float(
        data.qpos[_joint_qposadr(model, "demo_peg_slide")]
    )
    hammer_pos = _body_xpos_of(model, data, "demo_hammer")
    hammer_on_table = bool(
        -0.15 <= hammer_pos[0] <= 0.75
        and -0.35 <= hammer_pos[1] <= 0.65
        and hammer_pos[2] > -0.02
    )
    insertion_fraction = peg_q / PEG_TRAVEL_M
    return {
        "peg_travel_m": peg_q,
        "peg_insertion_fraction": insertion_fraction,
        "peg_success": peg_q >= PEG_SUCCESS_TRAVEL_M,
        "hammer_on_table": hammer_on_table,
        "task_success": bool(
            peg_q >= PEG_SUCCESS_TRAVEL_M and hammer_on_table
        ),
    }


# ------------------------------------------------------------------
# Demo 02 - Drawer Retrieval & Tray Loading
# ------------------------------------------------------------------

def evaluate_demo02(model, data, *, ever_opened=None) -> dict:
    """V2 Drawer Loading & Closing metrics.

    PHASE 1 drawer opened (>= 80% travel, tracked by the caller via
    ``ever_opened`` or read from the current state), PHASE 2 tape
    COM inside the drawer's usable interior (drawer-LOCAL frame -
    the region travels with the drawer), PHASE 3 drawer closed
    (<= 20% travel) with the tape still inside.
    """
    drawer_q = float(
        data.qpos[_joint_qposadr(model, "demo_drawer_slide")]
    )
    item_pos = _body_xpos_of(model, data, "demo_drawer_item")
    drawer_pos = _body_xpos_of(model, data, "demo_drawer")

    local_x = item_pos[0] - drawer_pos[0]
    local_y = item_pos[1] - drawer_pos[1]
    inside_drawer = bool(
        -0.091 < local_x < 0.078
        and abs(local_y) < 0.126
        and 0.030 < item_pos[2] < 0.150
    )
    opened_now = drawer_q >= DRAWER_OPEN_THRESHOLD_M
    was_opened = bool(opened_now if ever_opened is None
                      else ever_opened)
    closed_now = drawer_q <= DRAWER_CLOSE_THRESHOLD_M
    return {
        "drawer_open_m": drawer_q,
        "drawer_open_fraction": drawer_q / DRAWER_TRAVEL_M,
        "phase1_drawer_opened": was_opened,
        "item_position_m": item_pos.tolist(),
        "item_drawer_local_m": [local_x, local_y, item_pos[2]],
        "phase2_item_inside_drawer": inside_drawer,
        "phase3_drawer_closed": bool(closed_now),
        "task_success": bool(
            was_opened and inside_drawer and closed_now
        ),
    }


# ------------------------------------------------------------------
# Demo 03 - Maintenance Kit Packing
# ------------------------------------------------------------------

def evaluate_demo03(model, data) -> dict:
    """V2 two-object kit metrics: box in zone A, screwdriver in
    zone B with the tool-axis yaw within tolerance."""
    pa = _body_xpos_of(model, data, "kit_object_a")
    pb_body = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "kit_object_b"
    )
    pb = data.xpos[pb_body].copy()
    # yaw of the tool axis (body local +X) vs the zone-B slot axis (+X)
    tool_axis_world = data.xmat[pb_body].reshape(3, 3)[:, 0]
    yaw_deg = float(np.degrees(
        np.arctan2(tool_axis_world[1], tool_axis_world[0])
    ))

    zone_a = bool(
        abs(pa[0] - ZONE_A_CENTRE_XY[0]) < ZONE_A_HALF_XY[0]
        and abs(pa[1] - ZONE_A_CENTRE_XY[1]) < ZONE_A_HALF_XY[1]
        and 0.005 < pa[2] < 0.25
    )
    zone_b = bool(
        abs(pb[0] - ZONE_B_CENTRE_XY[0]) < ZONE_B_HALF_XY[0]
        and abs(pb[1] - ZONE_B_CENTRE_XY[1]) < ZONE_B_HALF_XY[1]
        and 0.002 < pb[2] < 0.10
    )
    yaw_ok = abs(yaw_deg) <= ZONE_C_YAW_TOLERANCE_DEG
    return {
        "object_a_position_m": pa.tolist(),
        "object_b_position_m": pb.tolist(),
        "object_b_yaw_deg": yaw_deg,
        "zone_a_membership": zone_a,
        "zone_b_membership": zone_b,
        "object_b_yaw_ok": yaw_ok,
        "task_success": bool(zone_a and zone_b and yaw_ok),
    }


EVALUATORS = {
    "01": evaluate_demo01,
    "02": evaluate_demo02,
    "03": evaluate_demo03,
}


# ------------------------------------------------------------------
# Task waypoints + position-only IK audit (PART 62-64)
# ------------------------------------------------------------------

@dataclass(frozen=True)
class Waypoint:
    name: str
    position: tuple[float, float, float]
    note: str = ""


DEMO_WAYPOINTS = {
    "01": (
        Waypoint(
            "hammer_pregrasp",
            (0.34, -0.13, 0.12),
            "palm above the raised handle grasp centre",
        ),
        Waypoint(
            "hammer_grasp",
            (0.34, -0.13, 0.077),
            "power-grasp contact height on the raised handle",
        ),
        Waypoint(
            "hammer_lift",
            (0.34, -0.13, 0.22),
            "lifted clear of the cradle",
        ),
        Waypoint(
            "hammer_prestrike",
            (0.415, 0.209, 0.22),
            "mallet face aligned over the peg cap, raised",
        ),
        Waypoint(
            "hammer_strike",
            (0.415, 0.209, 0.155),
            "face contacts the peg cap (tap depth)",
        ),
        Waypoint(
            "hammer_return",
            (0.34, -0.13, 0.13),
            "back above the cradle",
        ),
    ),
    "02": (
        Waypoint(
            "handle_pregrasp",
            (0.458, 0.18, 0.19),
            "above the bar handle, hand open",
        ),
        Waypoint(
            "handle_grasp",
            (0.458, 0.18, 0.145),
            "hook grasp on the horizontal bar",
        ),
        Waypoint(
            "drawer_open",
            (0.33, 0.18, 0.145),
            "handle retracted to full open",
        ),
        Waypoint(
            "table_object_pregrasp",
            (0.30, -0.18, 0.14),
            "above the tape on the table",
        ),
        Waypoint(
            "table_object_grasp",
            (0.30, -0.18, 0.08),
            "power grasp on the tape body",
        ),
        Waypoint(
            "table_object_lift",
            (0.30, -0.18, 0.18),
            "lifted clear of the table",
        ),
        Waypoint(
            "drawer_load_prepose",
            (0.47, 0.18, 0.20),
            "above the open drawer cavity",
        ),
        Waypoint(
            "drawer_load",
            (0.47, 0.18, 0.11),
            "tape lowered into the drawer",
        ),
        Waypoint(
            "handle_regrasp",
            (0.35, 0.18, 0.145),
            "re-approach the opened handle",
        ),
        Waypoint(
            "drawer_close",
            (0.458, 0.18, 0.145),
            "push the drawer fully closed",
        ),
    ),
    "03": (
        Waypoint(
            "box_pregrasp",
            (0.44, 0.34, 0.20),
            "above the standing box",
        ),
        Waypoint(
            "box_grasp",
            (0.44, 0.34, 0.13),
            "pinch grasp on the box mid-height",
        ),
        Waypoint(
            "box_goal",
            (0.32, -0.12, 0.14),
            "above zone A",
        ),
        Waypoint(
            "screwdriver_pregrasp",
            (0.54, 0.20, 0.16),
            "above the screwdriver grip",
        ),
        Waypoint(
            "screwdriver_grasp",
            (0.54, 0.20, 0.10),
            "tripod grasp on the cushion grip",
        ),
        Waypoint(
            "screwdriver_lift",
            (0.54, 0.20, 0.20),
            "lifted clear",
        ),
        Waypoint(
            "screwdriver_reorient",
            (0.44, 0.05, 0.20),
            "mid-carry rotation point toward the slot axis",
        ),
        Waypoint(
            "screwdriver_goal",
            (0.42, -0.295, 0.12),
            "above zone B slot",
        ),
    ),
}


def audit_ik(model, demo: str) -> list[dict]:
    """Position-only IK from the V8 ready seed for every waypoint.

    Position-only is the 4-DoF wrist-roll teleop task: TCP xyz drives
    the arm, joint 6 roll is free, wrist pitch/yaw unconstrained.  A
    converged solution with joint margin therefore certifies the
    waypoint is reachable under AVP wrist-roll teleop.
    """
    solver = RM65IKSolver(
        model, tcp_body_name="sg100_base_link"
    )
    results = []
    for wp in DEMO_WAYPOINTS[demo]:
        q, res = solver.solve(
            q_seed=V8_READY_ARM_Q.copy(),
            target_position=np.asarray(wp.position),
            target_rotation=None,
            position_only=True,
            posture_reference=V8_READY_ARM_Q,
        )
        converged = bool(res.converged and np.all(np.isfinite(q)))
        margin = None
        if converged:
            margin = float(
                np.min(
                    np.minimum(
                        q - solver.lower,
                        solver.upper - q,
                    )
                )
            )
        entry = {
            "waypoint": wp.name,
            "target": wp.position,
            "note": wp.note,
            "converged": converged,
            "position_error_m": (
                float(getattr(res, "pos_err", np.nan))
                if converged
                else None
            ),
            "min_joint_margin_rad": margin,
        }
        if converged:
            per_joint = np.minimum(
                q - solver.lower, solver.upper - q
            )
            entry["per_joint_margin_rad"] = {
                name: float(m)
                for name, m in zip(
                    (
                        "joint_1", "joint_2", "joint_3",
                        "joint_4", "joint_5", "joint_6",
                    ),
                    per_joint,
                )
            }
        results.append(entry)
    return results


# ------------------------------------------------------------------
# CLI
# ------------------------------------------------------------------

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Demo scene task-success metrics and waypoint IK audit"
        )
    )
    parser.add_argument(
        "--demo", required=True, choices=sorted(DEMO_SCENES),
        help="Demo number (01 / 02 / 03)",
    )
    parser.add_argument(
        "--qpos", default=None,
        help=(
            "Optional .npy file with a full qpos vector to evaluate; "
            "defaults to the scene's demo_ready keyframe"
        ),
    )
    parser.add_argument(
        "--ik-audit", action="store_true",
        help="Run the position-only waypoint IK audit",
    )
    parser.add_argument(
        "--mjcf", default=None,
        help="Override scene XML path (defaults to the repo demo scene)",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    xml = (
        Path(args.mjcf)
        if args.mjcf
        else MJCF_DIR / DEMO_SCENES[args.demo]
    )
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)

    # The 17-DoF layout validates the robot contract implicitly.
    resolve_robot_layout(model)

    if args.qpos:
        state = np.load(args.qpos)
        if state.shape != (model.nq,):
            raise SystemExit(
                f"qpos vector shape {state.shape} != nq {model.nq}"
            )
        data.qpos[:] = state
    else:
        key = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_KEY, "demo_ready"
        )
        mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)

    print(f"DEMO {args.demo}: {xml.name}")
    print(f"nq={model.nq} nv={model.nv} nu={model.nu} "
          f"nbody={model.nbody} ngeom={model.ngeom} ncam={model.ncam}")
    report = EVALUATORS[args.demo](model, data)
    for key, value in report.items():
        print(f"  {key}: {value}")

    if args.ik_audit:
        print("\nIK audit (position-only, wrist-roll teleop task):")
        all_ok = True
        for entry in audit_ik(model, args.demo):
            status = "REACHABLE" if entry["converged"] else "FAIL"
            margin = entry["min_joint_margin_rad"]
            print(
                f"  {entry['waypoint']:>18}: {status}"
                + (
                    f"  min joint margin {margin:.3f} rad"
                    if margin is not None
                    else ""
                )
            )
            if entry["converged"]:
                pj = entry["per_joint_margin_rad"]
                tight = {
                    k: round(v, 3)
                    for k, v in pj.items()
                    if v < 0.20
                }
                if tight:
                    print(
                        f"{'':>20} tight joints (<0.20 rad): {tight}"
                    )
            else:
                all_ok = False
        print(f"  IK AUDIT: {'PASS' if all_ok else 'FAIL'}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
