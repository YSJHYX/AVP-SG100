"""
RM65 + SG100 grasp demo.

  1. Bottle stands on the table.
  2. Hand is placed at the grasp pose (open fingers), palm backstop behind it.
  3. Fingers close (compliant, so they curl onto the bottle).
  4. Arm lifts.
  5. Viewer watches; prints PASS/FAIL by how far the bottle was lifted.

Default hand pose / palm backstop are tuned values; override with the flags.

Run (watch the grasp):
    python -m sg100_teleop.sim.run_grasp_demo
Inspect / re-tune the static pose (no close/lift):
    python -m sg100_teleop.sim.run_grasp_demo --inspect
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from sg100_teleop.hand.canonical_hand import CanonicalHandState
from sg100_teleop.kinematics.rm65_ik import RM65IKSolver
from sg100_teleop.retargeting.sg100_retargeter import SG100Retargeter


REPO_ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = (
    REPO_ROOT / "sg100_teleop" / "assets" / "rm65_sg100" / "mjcf"
    / "rm65_sg100_avp_grasp_scene.xml"
)

ARM_DOF, HAND_DOF = 6, 11
DT = 0.002  # 500 Hz

# ---- tuned defaults ----------------------------------------------------
Q_READY = np.array([0.32488, -0.76769, 0.97621, 0.22463, -1.80649, -0.12430])
BOTTLE_DR = 0.15
BOTTLE_Z = 0.64030
HAND_DX, HAND_DY, HAND_DZ = -0.08, -0.13, 0.0     # hand offset from bottle center (world)
HAND_ROLL = 0.0                                     # wrist roll [deg]
THUMB_J2_OPEN = -0.6                                # initial (open) thumb_j2 [rad]; 0=neutral, range [-2.61,0]
PALM_POS = (0.0, -0.03, 0.05)                       # palm backstop in palm-local
PALM_SIZE = (0.045, 0.035, 0.010)
PALM_QUAT = (0.5, 0.5, 0.5, -0.5)                   # 90 about X, then 90 about Y

# ---- grasp behaviour --------------------------------------------------
GRAVITY = (0.0, 0.0, -9.81)
FINGER_FORCE_N = 20.0     # compliant fingers: high enough to grip, not so high it flicks the bottle
CLOSE_AMOUNT = 0.7
LIFT_M = 0.03
LIFT_PASS_M = 0.02
T_SETTLE, T_CLOSE, T_LIFT, T_HOLD = 0.6, 1.5, 1.5, 0.6


def _rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def _approach_R(approach, ref=(0.0, 0.0, 1.0)):
    z = np.asarray(approach, dtype=np.float64)
    z = z / np.linalg.norm(z)
    r = np.asarray(ref, dtype=np.float64)
    x = r - np.dot(r, z) * z
    if np.linalg.norm(x) < 1e-9:
        r = np.array([1.0, 0.0, 0.0])
        x = r - np.dot(r, z) * z
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    y = y / np.linalg.norm(y)
    R = np.column_stack([x, y, z])
    if np.linalg.det(R) < 0:
        R = np.column_stack([x, -y, z])
    return R


def _ik(solver, target, R_grasp):
    """Multi-seed IK -> q (raises if truly unreachable)."""
    best = None
    for off in (
        (0, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 0, math.pi),
        (0, 0, 0, 0, math.pi / 2, 0),
        (0, 0, 0, 0, -math.pi / 2, 0),
    ):
        seed = np.clip(Q_READY + np.array(off), solver.lower, solver.upper)
        q, res = solver.solve(q_seed=seed, target_position=target, target_rotation=R_grasp)
        if res.converged and np.all(np.isfinite(q)):
            margin = float(np.min(np.minimum(q - solver.lower, solver.upper - q)))
            if best is None or margin > best[1]:
                best = (q, margin)
    if best is None:
        raise RuntimeError(f"IK unreachable for {np.round(target, 3)}")
    return best[0]


def _add_palm_collision(spec, palm_pos, palm_size, palm_quat):
    for b in spec.bodies:
        try:
            if b.name == "sg100_base_link":
                # The AVP grasp scene already bakes the palm backstop into the
                # XML; skip injecting a duplicate in that case. (--palm-pos is
                # then a no-op; edit the XML to move the baked backstop.)
                if any(getattr(g, "name", "") == "palm_collision" for g in b.geoms):
                    return
                g = b.add_geom()
                g.name = "palm_collision"
                g.type = mujoco.mjtGeom.mjGEOM_BOX
                g.pos = list(palm_pos)
                g.size = list(palm_size)
                g.quat = list(palm_quat)
                g.contype = 1
                g.conaffinity = 2
                g.group = 3
                g.rgba = [0.0, 1.0, 0.0, 0.45]
                return
        except Exception:
            pass


def _hand_closed(retargeter, amount):
    return np.asarray(retargeter.retarget(CanonicalHandState(
        thumb_opposition=0.75 * amount, thumb_abduction=0.25 * amount, thumb_distal=0.60 * amount,
        index_adduction=0.15 * amount, index_flexion=0.80 * amount, index_distal=0.75 * amount,
        middle_flexion=0.85 * amount, middle_distal=0.75 * amount,
        little_outward=0.15 * amount, little_flexion=0.80 * amount, little_distal=0.75 * amount,
    )))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dx", type=float, default=HAND_DX)
    ap.add_argument("--dy", type=float, default=HAND_DY)
    ap.add_argument("--dz", type=float, default=HAND_DZ)
    ap.add_argument("--roll", type=float, default=HAND_ROLL)
    ap.add_argument("--palm-pos", type=float, nargs=3, default=list(PALM_POS))
    ap.add_argument("--finger-force", type=float, default=FINGER_FORCE_N,
                    help="finger force limit [N] -- raise to grip harder, lower if it flicks the bottle")
    ap.add_argument("--inspect", action="store_true", help="static pose only (no close/lift)")
    ap.add_argument("--no-viewer", action="store_true")
    args = ap.parse_args()

    spec = mujoco.MjSpec.from_file(str(MODEL_PATH))
    _add_palm_collision(spec, args.palm_pos, PALM_SIZE, PALM_QUAT)
    model = spec.compile()
    model.opt.gravity[:] = GRAVITY
    for i in range(ARM_DOF, ARM_DOF + HAND_DOF):
        model.actuator_forcerange[i] = [-args.finger_force, args.finger_force]
    data = mujoco.MjData(model)

    solver = RM65IKSolver(model, tcp_body_name="sg100_base_link")
    retargeter = SG100Retargeter()
    bottle_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "water_bottle")

    # bottle on the table
    p_ready, _ = solver.fk(Q_READY)
    radial = np.array([p_ready[0], p_ready[1], 0.0])
    radial = radial / np.linalg.norm(radial)
    bottle = np.array([p_ready[0] + BOTTLE_DR * radial[0],
                       p_ready[1] + BOTTLE_DR * radial[1], BOTTLE_Z])
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "water_bottle_free")
    qa, va = int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])
    data.qpos[qa:qa + 3] = bottle
    data.qpos[qa + 3:qa + 7] = (1.0, 0.0, 0.0, 0.0)
    data.qvel[va:va + 6] = 0.0

    # hand grasp pose (+ lift pose, if the arm has room)
    R_grasp = _approach_R(radial) @ _rot_z(math.radians(args.roll))
    palm_target = bottle + np.array([args.dx, args.dy, args.dz])
    q_grasp = _ik(solver, palm_target, R_grasp)
    try:
        q_lift = _ik(solver, palm_target + np.array([0, 0, LIFT_M]), R_grasp)
    except RuntimeError:
        q_lift = q_grasp  # no room to lift; just hold

    hand_open = np.zeros(HAND_DOF)
    hand_open[1] = THUMB_J2_OPEN   # thumb_j2 (index 1 in SG100_JOINT_ORDER)
    hand_closed = _hand_closed(retargeter, CLOSE_AMOUNT)

    # start at the grasp pose, fingers open
    data.qpos[:ARM_DOF] = q_grasp
    # set HAND joints to the open pose -- do NOT touch qpos[17:] (bottle freejoint)
    data.qpos[ARM_DOF:ARM_DOF + HAND_DOF] = hand_open
    data.qvel[:] = 0.0
    data.ctrl[:ARM_DOF] = q_grasp
    data.ctrl[ARM_DOF:] = hand_open
    mujoco.mj_forward(model, data)

    print("=" * 56)
    print("RM65 + SG100  grasp demo")
    print("=" * 56)
    print(f"bottle = {np.round(bottle, 3)}   hand = {np.round(palm_target, 3)}")
    if args.inspect:
        print("INSPECT mode: hand held static (no close/lift).")

    # warn if anything already penetrates at the grasp pose -- that is what
    # launches the bottle on the first physics step.
    pens = [data.contact[i] for i in range(data.ncon) if data.contact[i].dist < 0]
    if pens:
        _gname = lambda gi: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gi) or ""
        print("[warn] penetrating contacts at grasp pose (will launch the bottle):")
        for c in pens:
            print(f"  {_gname(c.geom1)} <-> {_gname(c.geom2)} : {c.dist*1000:+.1f} mm")

    def step():
        # gravity-compensate the ARM (first 6 DOFs) so it holds the tuned pose
        # instead of sagging under gravity (which is what moved the hand and
        # knocked the bottle off). Hand stays compliant; bottle keeps gravity.
        data.qfrc_applied[:ARM_DOF] = data.qfrc_bias[:ARM_DOF]
        mujoco.mj_step(model, data)

    def ramp(arm_goal, hand_goal, seconds, viewer) -> bool:
        n = max(1, int(seconds / DT))
        a0, h0 = data.ctrl[:ARM_DOF].copy(), data.ctrl[ARM_DOF:].copy()
        for i in range(1, n + 1):
            t = i / n
            data.ctrl[:ARM_DOF] = (1 - t) * a0 + t * arm_goal
            data.ctrl[ARM_DOF:] = (1 - t) * h0 + t * hand_goal
            step()
            z = float(data.xpos[bottle_id][2])
            if not np.isfinite(z) or z < 0.2 or z > 1.5:
                return False
            if viewer is not None:
                viewer.sync()
                time.sleep(DT)
        return True

    def run_grasp(viewer) -> dict:
        ramp(q_grasp, hand_open, T_SETTLE, viewer)        # SETTLE
        z0 = float(data.xpos[bottle_id][2])
        ok = ramp(q_grasp, hand_closed, T_CLOSE, viewer)  # CLOSE
        if ok:
            ok = ramp(q_lift, hand_closed, T_LIFT, viewer)  # LIFT
        for _ in range(int(T_HOLD / DT)):                 # HOLD
            step()
            if viewer is not None:
                viewer.sync()
                time.sleep(DT)
        z1 = float(data.xpos[bottle_id][2])
        return {"z0": z0, "z1": z1, "lift": z1 - z0, "ok": ok, "ncon": int(data.ncon)}

    def report(d):
        print("-" * 56)
        if not d["ok"]:
            print("GRASP: FAIL (sim went unstable -- a geom penetrated deeply)")
            return
        print(f"bottle z: {d['z0']:.4f} -> {d['z1']:.4f},  lifted {d['lift']*1000:.1f} mm,  contacts {d['ncon']}")
        if d["lift"] > LIFT_PASS_M:
            print(f"GRASP: PASS (lifted {d['lift']*1000:.1f} mm)")
        else:
            print(f"GRASP: FAIL (lifted {d['lift']*1000:.1f} mm <= {LIFT_PASS_M*1000:.0f} mm)")

    def run(viewer):
        if args.inspect:
            if viewer is not None:
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.02)
            return
        report(run_grasp(viewer))

    if args.no_viewer:
        run(None)
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 1.1
        viewer.cam.azimuth = 130
        viewer.cam.elevation = -25
        viewer.cam.lookat[:] = [bottle[0] * 0.7, bottle[1] * 0.7, 0.6]
        run(viewer)
        if not args.inspect:
            print("\nclose the viewer to exit.")
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.02)


if __name__ == "__main__":
    main()
