"""
RM65 + SG100 combined grasp-pose edge-case reachability.

LEVEL 1 -- kinematic / IK only.

This test does NOT connect Apple Vision Pro and does NOT model contacts.
It separates four different failure causes that a single PASS/FAIL would
otherwise conflate:

    1. grasp / pre-grasp endpoint reachability  (multi-seed global probe)
    2. local IK seed convergence                (single seed chain)
    3. READY -> pre-grasp transition feasibility
    4. final grasp approach feasibility

For every case it reports three independent states:

    GRASP_ENDPOINT    : REACHABLE / UNREACHABLE
    PREGRASP_ENDPOINT : REACHABLE / UNREACHABLE
    TRAJECTORY        : PASS / WARNING / FAIL

It reuses the existing RM65IKSolver (TCP = sg100_base_link) and does NOT
re-implement IK, nor modify the solver / model / retargeter.

SG100 grasp frame (confirmed empirically by FK on the combined model at
Q_TELEOP_READY with hand q = 0):

    Every SG100 finger link (index/middle/little link1 -> link3) extends
    along sg100_base_link local +Z. Therefore:

        GRASP_FORWARD_LOCAL_AXIS = +Z   (hand-forward / finger approach)
        local +X                  ~ palm-normal  (points up at Q_TELEOP_READY)
        local +Y                  ~ lateral

A grasp target rotation R is constructed so that

        R[:, 2] == desired approach direction in world

because local +Z is the hand-forward axis.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import mujoco
import numpy as np

from sg100_teleop.kinematics.rm65_ik import (
    RM65IKSolver,
    so3_log,
)


REPO_ROOT = Path(__file__).resolve().parents[2]

MODEL_PATH = (
    REPO_ROOT
    / "sg100_teleop"
    / "assets"
    / "rm65_sg100"
    / "mjcf"
    / "rm65_sg100_sim.xml"
)

TCP_BODY = "sg100_base_link"
ARM_DOF = 6
TOTAL_DOF = 17

# Final teleoperation-ready configuration (validated: sigma_min ~ 0.063,
# condition ~ 9.76 at center; local +-20 mm / +-5 deg => 0 IK failures).
Q_TELEOP_READY = np.array(
    [
        0.32488,
        -0.76769,
        0.97621,
        0.22463,
        -1.80649,
        -0.12430,
    ],
    dtype=np.float64,
)

# SG100 local axis that points along the fingers.
GRASP_FORWARD_LOCAL_AXIS = "+Z"

# Common validated reference seed, read verbatim from the two existing tests
# (test_rm65_ik.py and test_rm65_ik_reachable_targets.py both use this q_seed).
# It is NOT modified here -- only read as a deterministic multi-seed member.
VALIDATED_REF_Q = np.array(
    [0.30, -0.40, 0.35, -0.25, 0.30, 0.20],
    dtype=np.float64,
)

RM65_JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]

# Direct (first-round) trajectory resolution.
PREGRASP_DISTANCE = 0.10        # m, along -approach from grasp pose
READY_PREGRASP_STEPS = 25
APPROACH_STEPS = 50

# Staged TOP-down trajectory resolution.
STAGE_SAFE_STEPS = 15           # READY -> SAFE_HIGH (translate +Z)
STAGE_ORIENT_STEPS = 40         # SAFE_HIGH -> TOP_ORIENTED (rotate R_ready -> R_top)
STAGE_XY_STEPS = 20             # TOP_ORIENTED -> ABOVE_OBJECT (horizontal XY)
STAGE_DESCEND_STEPS = 20        # ABOVE_OBJECT -> PREGRASP (descend -Z)
STAGE_APPROACH_STEPS = 40       # PREGRASP -> GRASP (final -Z approach)

# Classification thresholds.
SIGMA_MIN_FLOOR = 0.015
CONDITION_CEIL = 80.0
JOINT_MARGIN_FLOOR = 0.15       # rad
POS_ERR_TOL = 2e-4              # 0.2 mm
ROT_ERR_TOL = math.radians(0.1)

# Deterministic TOP roll sweep about the approach axis (degrees).
TOP_ROLLS = [-180, -150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150]


# ---------------------------------------------------------------------
# Small geometry helpers (no external deps beyond numpy)
# ---------------------------------------------------------------------

def _normalize(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("Cannot normalize near-zero vector")
    return v / n


def _skew(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=np.float64)
    return np.array(
        [
            [0.0, -a[2], a[1]],
            [a[2], 0.0, -a[0]],
            [-a[1], a[0], 0.0],
        ],
        dtype=np.float64,
    )


def _axis_angle_exp(rv: np.ndarray) -> np.ndarray:
    """Rodrigues exponential map of a rotation vector (axis * angle)."""
    rv = np.asarray(rv, dtype=np.float64)
    angle = float(np.linalg.norm(rv))
    if angle < 1e-12:
        return np.eye(3, dtype=np.float64)
    axis = rv / angle
    K = _skew(axis)
    return (
        np.eye(3, dtype=np.float64)
        + math.sin(angle) * K
        + (1.0 - math.cos(angle)) * (K @ K)
    )


def rotation_interp(R0: np.ndarray, R1: np.ndarray, t: float) -> np.ndarray:
    """Shortest-arc interpolation R0 -> R1 at parameter t in [0, 1]."""
    R0 = np.asarray(R0, dtype=np.float64)
    R1 = np.asarray(R1, dtype=np.float64)
    R_delta = R1 @ R0.T
    rv = so3_log(R_delta)
    return _axis_angle_exp(t * rv) @ R0


def rot_z(theta: float) -> np.ndarray:
    """Rotation about the canonical Z axis."""
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def make_grasp_rotation(
    approach_world: np.ndarray,
    reference_world: np.ndarray,
) -> np.ndarray:
    """
    Build an orthonormal right-handed rotation whose local +Z column equals
    the desired approach direction.

        R[:, 2] == normalize(approach_world)   (hand-forward)
        R[:, 0] ~ reference_world projected onto the plane normal to approach
        det(R) == +1
    """
    z = _normalize(approach_world)

    ref = np.asarray(reference_world, dtype=np.float64)
    x = ref - np.dot(ref, z) * z

    if np.linalg.norm(x) < 1e-9:
        candidates = [
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 1.0, 0.0]),
            np.array([0.0, 0.0, 1.0]),
        ]
        best = max(
            candidates,
            key=lambda c: np.linalg.norm(c - np.dot(c, z) * z),
        )
        x = best - np.dot(best, z) * z

    x = _normalize(x)
    y = _normalize(np.cross(z, x))
    R = np.column_stack([x, y, z])
    if np.linalg.det(R) < 0.0:
        R = np.column_stack([x, -y, z])
    return R


def joint_limit_margin(
    q: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> float:
    """Minimum distance of any joint from its nearest limit."""
    q = np.asarray(q, dtype=np.float64)
    return float(np.min(np.minimum(q - lower, upper - q)))


def limiting_joint_info(
    q: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    names: list[str],
) -> dict:
    """Per-joint margin detail for the most limit-bound joint in q."""
    q = np.asarray(q, dtype=np.float64)
    per = np.minimum(q - lower, upper - q)
    i = int(np.argmin(per))
    return {
        "joint": names[i],
        "q": float(q[i]),
        "lower": float(lower[i]),
        "upper": float(upper[i]),
        "margin": float(per[i]),
    }


# ---------------------------------------------------------------------
# Workspace / case definitions (unchanged geometry from round 1)
# ---------------------------------------------------------------------

def build_cases(
    p_ready: np.ndarray,
    R_ready: np.ndarray,
) -> list[dict]:
    """
    Define the 9 grasp cases in a tabletop workspace frame derived from the
    teleop-ready TCP pose. Geometry is identical to round 1 (SIDE/DIAGONAL
    cases are intentionally not changed so results stay comparable).
    """
    radial = _normalize(np.array([p_ready[0], p_ready[1], 0.0]))
    lateral = np.array([-radial[1], radial[0], 0.0])
    # Actual SG100 hand-forward at Q_TELEOP_READY (== R_ready[:, 2]): roughly
    # horizontal and ~= radial. Used as the side-grasp approach so the baseline
    # keeps the validated wrist roll (spec CASE 1 / section 5 / section 10).
    ready_fwd = np.asarray(R_ready[:, 2], dtype=np.float64).copy()
    up = np.array([0.0, 0.0, 1.0])
    down = np.array([0.0, 0.0, -1.0])

    diag_left = _normalize(radial + 0.7 * lateral)    # ~35 deg off radial
    diag_right = _normalize(radial - 0.7 * lateral)

    dz = np.array([0.0, 0.0, -0.05])                  # TOP hand slightly lower

    cases: list[dict] = []

    def add(name, p, approach, reference):
        cases.append(
            {
                "name": name,
                "p_grasp": np.asarray(p, dtype=np.float64),
                "approach": np.asarray(approach, dtype=np.float64),
                "reference": np.asarray(reference, dtype=np.float64),
            }
        )

    # ---- Side grasps (approach along validated hand-forward, ~horizontal) ----
    add("SIDE_CENTER", p_ready, ready_fwd, up)
    add("SIDE_FAR", p_ready + 0.17 * radial, ready_fwd, up)
    add("SIDE_NEAR", p_ready - 0.10 * radial, ready_fwd, up)
    add("SIDE_LOW", p_ready + np.array([0.0, 0.0, -0.12]), ready_fwd, up)

    # ---- Top-down grasps (approach ~ world -Z; roll explored separately) ----
    add("TOP_CENTER", p_ready + dz, down, radial)
    add("TOP_FAR", p_ready + 0.17 * radial + dz, down, radial)
    add("TOP_NEAR", p_ready - 0.10 * radial + dz, down, radial)

    # ---- Diagonal grasps (horizontal approach yawed ~35 deg) ----
    add(
        "DIAGONAL_LEFT",
        p_ready + 0.10 * radial + 0.10 * lateral,
        diag_left,
        up,
    )
    add(
        "DIAGONAL_RIGHT",
        p_ready + 0.10 * radial - 0.10 * lateral,
        diag_right,
        up,
    )

    return cases


# ---------------------------------------------------------------------
# Deterministic multi-seed set for endpoint reachability
# ---------------------------------------------------------------------

def build_seed_set(side_center_final_q: np.ndarray) -> list[tuple[str, np.ndarray]]:
    """
    Fixed, deterministic seed list (no randomness). Every member is either an
    already-validated configuration or a small fixed joint offset of one.

    Seed failure does NOT count as a trajectory IK failure -- it only means a
    particular local solver start did not converge to that endpoint.
    """
    seeds: list[tuple[str, np.ndarray]] = [
        ("Q_TELEOP_READY", Q_TELEOP_READY.copy()),
        ("SIDE_CENTER_final", np.asarray(side_center_final_q, dtype=np.float64).copy()),
        (
            "validated_ref_q (test_rm65_ik / ik_reachable)",
            VALIDATED_REF_Q.copy(),
        ),
    ]

    # Fixed offsets of the validated reference q to probe alternative wrist
    # branches. All offsets stay inside joint limits.
    for tag, off in [
        ("j5+0.6", np.array([0.0, 0.0, 0.0, 0.0, 0.6, 0.0])),
        ("j5-0.6", np.array([0.0, 0.0, 0.0, 0.0, -0.6, 0.0])),
        ("j4+0.6", np.array([0.0, 0.0, 0.0, 0.6, 0.0, 0.0])),
    ]:
        seeds.append((f"validated_ref_q_{tag}", VALIDATED_REF_Q + off))

    return seeds


def solve_endpoint(
    solver: RM65IKSolver,
    seeds: list[tuple[str, np.ndarray]],
    p: np.ndarray,
    R: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> dict | None:
    """
    Probe a single Cartesian endpoint from every deterministic seed.

    Returns the healthiest valid solution found, or None if no seed converges
    to a finite, in-limit solution. Health ranking = (joint margin, sigma_min).
    """
    best: dict | None = None
    for label, q_seed in seeds:
        q, res = solver.solve(
            np.clip(np.asarray(q_seed, dtype=np.float64), lower, upper),
            p,
            R,
        )
        if not res.converged or not np.all(np.isfinite(q)):
            continue

        margin = joint_limit_margin(q, lower, upper)
        if margin < -1e-9:
            # Hard joint-limit violation -> not a usable endpoint solution.
            continue

        sigma = res.sigma_min if np.isfinite(res.sigma_min) else -1.0
        score = (margin, sigma)
        if best is None or score > best["score"]:
            best = {
                "seed": label,
                "q": q.copy(),
                "sigma": float(res.sigma_min),
                "cond": float(res.condition_number),
                "margin": margin,
                "pos_err": float(res.position_error),
                "rot_err": float(res.rotation_error),
                "score": score,
            }
    return best


# ---------------------------------------------------------------------
# Waypoint sequences
# ---------------------------------------------------------------------

def build_direct_waypoints(
    p_ready: np.ndarray,
    R_ready: np.ndarray,
    p_grasp: np.ndarray,
    approach: np.ndarray,
    R_grasp: np.ndarray,
) -> list[tuple[str, int, np.ndarray, np.ndarray]]:
    """First-round READY -> PREGRASP -> GRASP direct Cartesian sequence."""
    p_pre = p_grasp - PREGRASP_DISTANCE * approach
    ws: list[tuple[str, int, np.ndarray, np.ndarray]] = []
    for i in range(1, READY_PREGRASP_STEPS + 1):
        t = i / READY_PREGRASP_STEPS
        ws.append(
            (
                "READY->PREGRASP",
                i,
                (1.0 - t) * p_ready + t * p_pre,
                rotation_interp(R_ready, R_grasp, t),
            )
        )
    for i in range(1, APPROACH_STEPS + 1):
        t = i / APPROACH_STEPS
        ws.append(
            (
                "PREGRASP->GRASP",
                i,
                (1.0 - t) * p_pre + t * p_grasp,
                R_grasp,
            )
        )
    return ws


def build_staged_top_waypoints(
    p_ready: np.ndarray,
    R_ready: np.ndarray,
    p_grasp: np.ndarray,
    R_top: np.ndarray,
) -> list[tuple[str, int, np.ndarray, np.ndarray]]:
    """
    Realistic staged top-down manipulation sequence.

        READY
          -> SAFE_HIGH            (translate +0.10 Z, keep R_ready)
          -> TOP_ORIENTED_SAFE    (hold position, rotate R_ready -> R_top)
          -> ABOVE_OBJECT         (keep R_top, move XY over the target)
          -> PREGRASP             (keep R_top, descend to pre-grasp)
          -> GRASP                (keep R_top, final -Z approach)
    """
    p_safe = p_ready + np.array([0.0, 0.0, 0.10])
    p_above = np.array([p_grasp[0], p_grasp[1], p_safe[2]])
    # approach = world -Z, so pre-grasp sits +PREGRASP_DISTANCE above grasp.
    p_pre = p_grasp - PREGRASP_DISTANCE * np.array([0.0, 0.0, -1.0])

    ws: list[tuple[str, int, np.ndarray, np.ndarray]] = []

    for i in range(1, STAGE_SAFE_STEPS + 1):
        t = i / STAGE_SAFE_STEPS
        ws.append(
            (
                "READY->SAFE_HIGH",
                i,
                (1.0 - t) * p_ready + t * p_safe,
                R_ready,
            )
        )

    for i in range(1, STAGE_ORIENT_STEPS + 1):
        t = i / STAGE_ORIENT_STEPS
        ws.append(
            (
                "SAFE_HIGH->TOP_ORIENTED",
                i,
                p_safe.copy(),
                rotation_interp(R_ready, R_top, t),
            )
        )

    for i in range(1, STAGE_XY_STEPS + 1):
        t = i / STAGE_XY_STEPS
        ws.append(
            (
                "TOP_ORIENTED->ABOVE_OBJECT",
                i,
                (1.0 - t) * p_safe + t * p_above,
                R_top,
            )
        )

    for i in range(1, STAGE_DESCEND_STEPS + 1):
        t = i / STAGE_DESCEND_STEPS
        ws.append(
            (
                "ABOVE_OBJECT->PREGRASP",
                i,
                (1.0 - t) * p_above + t * p_pre,
                R_top,
            )
        )

    for i in range(1, STAGE_APPROACH_STEPS + 1):
        t = i / STAGE_APPROACH_STEPS
        ws.append(
            (
                "PREGRASP->GRASP",
                i,
                (1.0 - t) * p_pre + t * p_grasp,
                R_top,
            )
        )

    return ws


# ---------------------------------------------------------------------
# Waypoint runner (shared by direct + staged trajectories)
# ---------------------------------------------------------------------

def run_waypoints(
    solver: RM65IKSolver,
    waypoints: list[tuple[str, int, np.ndarray, np.ndarray]],
    q_ready: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> dict:
    """
    Execute a waypoint sequence with previous-successive-solution seeding and
    collect diagnostics that separate the four failure causes.

    IK failures here are trajectory (single-seed-chain) failures only --
    endpoint multi-seed failures are accounted for separately and never folded
    into these counts.
    """
    q = q_ready.copy()
    q_prev = q_ready.copy()

    ik_failures = 0
    nonfinite = False
    limit_violation = False

    min_sigma = float("inf")
    max_condition = 0.0
    min_margin = float("inf")
    max_dq = 0.0
    max_pos_err = 0.0
    max_rot_err = 0.0

    first_fail_phase: str | None = None
    first_fail_idx: int | None = None
    first_ik_fail: tuple[str, int] | None = None
    first_sigma_below: tuple[str, int] | None = None
    first_cond_above: tuple[str, int] | None = None

    last_success_gi = -1
    last_success_p: np.ndarray | None = None

    limiting: dict | None = None

    total = len(waypoints)

    for gi, (phase, idx, p_t, R_t) in enumerate(waypoints):
        q_cand, res = solver.solve(q, p_t, R_t)

        if not res.converged or not np.all(np.isfinite(q_cand)):
            ik_failures += 1
            if not np.all(np.isfinite(q_cand)):
                nonfinite = True
            if first_fail_phase is None:
                first_fail_phase = phase
                first_fail_idx = idx
            if first_ik_fail is None:
                first_ik_fail = (phase, idx)
            # HOLD last valid q (simulation-only policy), keep seeding from it.
            continue

        margin = joint_limit_margin(q_cand, lower, upper)
        if margin < min_margin:
            min_margin = margin
            limiting = limiting_joint_info(q_cand, lower, upper, RM65_JOINT_NAMES)
        if margin < -1e-9:
            limit_violation = True

        if np.isfinite(res.sigma_min):
            if res.sigma_min < SIGMA_MIN_FLOOR and first_sigma_below is None:
                first_sigma_below = (phase, idx)
            min_sigma = min(min_sigma, float(res.sigma_min))
        if np.isfinite(res.condition_number):
            if res.condition_number > CONDITION_CEIL and first_cond_above is None:
                first_cond_above = (phase, idx)
            max_condition = max(max_condition, float(res.condition_number))

        max_dq = max(max_dq, float(np.max(np.abs(q_cand - q_prev))))
        max_pos_err = max(max_pos_err, float(res.position_error))
        max_rot_err = max(max_rot_err, float(res.rotation_error))

        q = q_cand.copy()
        q_prev = q_cand.copy()
        last_success_gi = gi
        last_success_p = p_t.copy()

    if min_sigma == float("inf"):
        min_sigma = float("nan")
    if min_margin == float("inf"):
        min_margin = float("nan")

    last_success_fraction = (
        (last_success_gi + 1) / total if last_success_gi >= 0 else 0.0
    )

    return {
        "ik_failures": ik_failures,
        "nonfinite": nonfinite,
        "limit_violation": limit_violation,
        "min_sigma": min_sigma,
        "max_condition": max_condition,
        "min_margin": min_margin,
        "max_dq": max_dq,
        "max_pos_err": max_pos_err,
        "max_rot_err": max_rot_err,
        "first_fail_phase": first_fail_phase,
        "first_fail_idx": first_fail_idx,
        "first_ik_fail": first_ik_fail,
        "first_sigma_below": first_sigma_below,
        "first_cond_above": first_cond_above,
        "last_success_fraction": last_success_fraction,
        "last_success_position": last_success_p,
        "limiting": limiting,
        "final_q": q.copy(),
        "n_waypoints": total,
    }


def trajectory_verdict(t: dict) -> str:
    if t["ik_failures"] > 0 or t["nonfinite"] or t["limit_violation"]:
        return "FAIL"
    if (
        t["min_sigma"] < SIGMA_MIN_FLOOR
        or t["max_condition"] > CONDITION_CEIL
        or t["min_margin"] < JOINT_MARGIN_FLOOR
        or t["max_pos_err"] > POS_ERR_TOL
        or t["max_rot_err"] > ROT_ERR_TOL
    ):
        return "WARNING"
    return "PASS"


# ---------------------------------------------------------------------
# TOP roll sweep
# ---------------------------------------------------------------------

def top_roll_sweep(
    solver: RM65IKSolver,
    seeds: list[tuple[str, np.ndarray]],
    p_grasp: np.ndarray,
    radial: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> list[dict]:
    """
    Deterministic sweep of wrist roll about the world -Z approach axis.

    For every roll:
        R_target[:, 2] == [0, 0, -1]   (approach fixed)
        only the roll about the approach axis changes (R_base @ Rz(roll))

    Each roll is probed at the grasp and pre-grasp endpoints with the full
    deterministic multi-seed set.
    """
    down = np.array([0.0, 0.0, -1.0])
    R_base = make_grasp_rotation(down, radial)
    p_pre = p_grasp - PREGRASP_DISTANCE * down

    rows: list[dict] = []
    for roll in TOP_ROLLS:
        R_target = R_base @ rot_z(math.radians(roll))
        grasp = solve_endpoint(solver, seeds, p_grasp, R_target, lower, upper)
        pre = solve_endpoint(solver, seeds, p_pre, R_target, lower, upper)
        rows.append(
            {
                "roll": roll,
                "R": R_target,
                "grasp": grasp,
                "pregrasp": pre,
            }
        )
    return rows


def select_top_roll(sweep: list[dict]) -> dict | None:
    """
    Pick the reachable roll with the healthiest grasp endpoint
    (max joint margin, then max sigma_min). Returns None if no roll reaches
    the grasp endpoint.
    """
    candidates = [r for r in sweep if r["grasp"] is not None]
    if not candidates:
        return None
    return max(candidates, key=lambda r: (r["grasp"]["margin"], r["grasp"]["sigma"]))


# ---------------------------------------------------------------------
# Per-case execution
# ---------------------------------------------------------------------

def run_side_case(
    solver: RM65IKSolver,
    case: dict,
    seeds: list[tuple[str, np.ndarray]],
    q_ready: np.ndarray,
    p_ready: np.ndarray,
    R_ready: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> dict:
    approach = case["approach"]
    R_grasp = make_grasp_rotation(approach, case["reference"])
    p_grasp = case["p_grasp"]
    p_pre = p_grasp - PREGRASP_DISTANCE * approach

    grasp_ep = solve_endpoint(solver, seeds, p_grasp, R_grasp, lower, upper)
    pre_ep = solve_endpoint(solver, seeds, p_pre, R_grasp, lower, upper)

    waypoints = build_direct_waypoints(p_ready, R_ready, p_grasp, approach, R_grasp)
    traj = run_waypoints(solver, waypoints, q_ready, lower, upper)

    return {
        "name": case["name"],
        "kind": "SIDE",
        "approach": approach,
        "p_grasp": p_grasp,
        "R_grasp": R_grasp,
        "grasp_endpoint": "REACHABLE" if grasp_ep else "UNREACHABLE",
        "pregrasp_endpoint": "REACHABLE" if pre_ep else "UNREACHABLE",
        "trajectory": trajectory_verdict(traj),
        "grasp_ep": grasp_ep,
        "pre_ep": pre_ep,
        "traj": traj,
    }


def run_top_case(
    solver: RM65IKSolver,
    case: dict,
    seeds: list[tuple[str, np.ndarray]],
    q_ready: np.ndarray,
    p_ready: np.ndarray,
    R_ready: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> dict:
    p_grasp = case["p_grasp"]
    radial = _normalize(np.array([p_ready[0], p_ready[1], 0.0]))

    sweep = top_roll_sweep(solver, seeds, p_grasp, radial, lower, upper)
    selected = select_top_roll(sweep)

    grasp_endpoint_reachable = any(r["grasp"] is not None for r in sweep)
    pregrasp_endpoint_reachable = any(r["pregrasp"] is not None for r in sweep)

    if selected is not None:
        R_top = selected["R"]
        sel_roll = selected["roll"]
        waypoints = build_staged_top_waypoints(p_ready, R_ready, p_grasp, R_top)
        traj = run_waypoints(solver, waypoints, q_ready, lower, upper)
        traj_verdict = trajectory_verdict(traj)
    else:
        R_top = None
        sel_roll = None
        traj = None
        traj_verdict = "NO PATH"

    # Final TOP semantic (spec section J). Never claim absolute unreachability.
    if not grasp_endpoint_reachable:
        final = "NO REACHABLE ENDPOINT FOUND WITH TESTED ROLLS/SEEDS"
    elif traj_verdict == "PASS":
        final = "TOP GRASP FEASIBLE"
    else:
        final = "POSE REACHABLE / STAGED PATH FAIL"

    return {
        "name": case["name"],
        "kind": "TOP",
        "p_grasp": p_grasp,
        "sweep": sweep,
        "selected_roll": sel_roll,
        "grasp_endpoint": "REACHABLE" if grasp_endpoint_reachable else "UNREACHABLE",
        "pregrasp_endpoint": "REACHABLE" if pregrasp_endpoint_reachable else "UNREACHABLE",
        "trajectory": traj_verdict,
        "final": final,
        "traj": traj,
    }


# ---------------------------------------------------------------------
# Reporting helpers
# ---------------------------------------------------------------------

def _fmt(x, p=5):
    if x is None:
        return "n/a"
    if isinstance(x, float) and (math.isnan(x) or math.isinf(x)):
        return "nan"
    return np.round(x, p)


def print_roll_sweep_report(top_results: list[dict]) -> None:
    print("\n" + "=" * 78)
    print("TOP-DOWN ROLL SWEEP")
    print("=" * 78)

    for r in top_results:
        print(f"\n--- {r['name']}  (grasp pos = {np.round(r['p_grasp'], 4)}) ---")
        print(
            f"{'roll':>6} | {'grasp':>8} | {'pre':>8} | "
            f"{'sigma':>9} | {'cond':>8} | {'margin':>8} | best seed"
        )
        print("-" * 78)
        for row in r["sweep"]:
            g = row["grasp"]
            pre = row["pregrasp"]
            g_sigma = f"{g['sigma']:.5f}" if g else "n/a"
            g_cond = f"{g['cond']:.2f}" if g else "n/a"
            g_margin = f"{g['margin']:.4f}" if g else "n/a"
            g_seed = g["seed"] if g else "-"
            print(
                f"{row['roll']:>6} | "
                f"{'OK' if g else '--':>8} | "
                f"{'OK' if pre else '--':>8} | "
                f"{g_sigma:>9} | "
                f"{g_cond:>8} | "
                f"{g_margin:>8} | "
                f"{g_seed}"
            )
        sel = r["selected_roll"]
        print(
            f"  selected roll = "
            f"{sel if sel is not None else 'NONE (no reachable grasp endpoint)'}"
        )


def limiting_factor(traj: dict | None) -> str:
    if traj is None:
        return "no staged trajectory"
    if traj["ik_failures"] > 0:
        f = traj["first_ik_fail"]
        return f"{traj['ik_failures']} IK failures (first at {f[0]} #{f[1]})"
    reasons = []
    if traj["min_sigma"] < SIGMA_MIN_FLOOR:
        reasons.append(f"sigma_min={traj['min_sigma']:.5f}")
    if traj["max_condition"] > CONDITION_CEIL:
        reasons.append(f"condition={traj['max_condition']:.1f}")
    if traj["min_margin"] < JOINT_MARGIN_FLOOR and traj.get("limiting"):
        lj = traj["limiting"]
        reasons.append(
            f"{lj['joint']} margin={lj['margin']:.4f}"
        )
    return "; ".join(reasons) if reasons else "none"


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))

    if model.nq != TOTAL_DOF or model.nu != TOTAL_DOF:
        raise RuntimeError(
            f"Expected combined model nq=nu={TOTAL_DOF}, "
            f"got nq={model.nq}, nu={model.nu}"
        )

    solver = RM65IKSolver(model, tcp_body_name=TCP_BODY)
    lower = solver.lower
    upper = solver.upper

    p_ready, R_ready = solver.fk(Q_TELEOP_READY)

    print("=" * 78)
    print("RM65 + SG100 GRASP POSE EDGE CASES (LEVEL 1, kinematic only)")
    print("=" * 78)

    print("\nTeleop-ready configuration:")
    print("  Q_TELEOP_READY =", Q_TELEOP_READY)
    print("  TCP world pos  =", np.round(p_ready, 5))
    print("  TCP local +X world =", np.round(R_ready[:, 0], 5), "(palm-normal ref)")
    print("  TCP local +Y world =", np.round(R_ready[:, 1], 5), "(lateral)")
    print("  TCP local +Z world =", np.round(R_ready[:, 2], 5), "(hand-forward)")
    print(
        "  Grasp approach axis : local "
        + GRASP_FORWARD_LOCAL_AXIS
        + "  (R[:,2] := approach)"
    )

    cases = build_cases(p_ready, R_ready)

    # Obtain SIDE_CENTER final q (validated baseline) as a deterministic seed.
    sc = cases[0]
    sc_R = make_grasp_rotation(sc["approach"], sc["reference"])
    sc_waypoints = build_direct_waypoints(
        p_ready, R_ready, sc["p_grasp"], sc["approach"], sc_R
    )
    sc_traj = run_waypoints(solver, sc_waypoints, Q_TELEOP_READY, lower, upper)
    side_center_final_q = sc_traj["final_q"]

    seeds = build_seed_set(side_center_final_q)

    print("\nDeterministic seed set for endpoint reachability:")
    for label, q_seed in seeds:
        print(f"  {label:42s} = {np.round(q_seed, 5)}")

    # Run every case.
    results: list[dict] = []
    for case in cases:
        if case["name"].startswith("TOP"):
            results.append(
                run_top_case(
                    solver, case, seeds, Q_TELEOP_READY, p_ready, R_ready, lower, upper
                )
            )
        else:
            results.append(
                run_side_case(
                    solver, case, seeds, Q_TELEOP_READY, p_ready, R_ready, lower, upper
                )
            )

    # ---- TOP roll sweep report ----
    top_results = [r for r in results if r["kind"] == "TOP"]
    print_roll_sweep_report(top_results)

    # ---- Per-case detail (SIDE / DIAGONAL) ----
    print("\n" + "=" * 78)
    print("SIDE / DIAGONAL CASE DETAIL (direct READY->PREGRASP->GRASP)")
    print("=" * 78)

    for r in results:
        if r["kind"] != "SIDE":
            continue
        t = r["traj"]
        print(f"\n--- {r['name']} ---")
        print("  grasp pos        :", np.round(r["p_grasp"], 5))
        print("  approach dir     :", np.round(r["approach"], 5))
        print("  GRASP_ENDPOINT   :", r["grasp_endpoint"], end="")
        if r["grasp_ep"]:
            print(
                f"  (best seed={r['grasp_ep']['seed']}, "
                f"margin={r['grasp_ep']['margin']:.4f}, "
                f"sigma={r['grasp_ep']['sigma']:.5f})"
            )
        else:
            print()
        print("  PREGRASP_ENDPOINT:", r["pregrasp_endpoint"], end="")
        if r["pre_ep"]:
            print(
                f"  (best seed={r['pre_ep']['seed']}, "
                f"margin={r['pre_ep']['margin']:.4f})"
            )
        else:
            print()
        print("  TRAJECTORY       :", r["trajectory"])
        print("    IK failures        :", t["ik_failures"])
        print("    min sigma_min      :", _fmt(t["min_sigma"]))
        print("    max condition      :", _fmt(t["max_condition"]))
        print("    min joint margin   :", _fmt(t["min_margin"]))
        print("    max q step         :", _fmt(t["max_dq"], 6), "rad")
        print("    max pos error      :", _fmt(t["max_pos_err"] * 1000, 4), "mm")
        print("    max rot error      :", _fmt(math.degrees(t["max_rot_err"]), 5), "deg")
        if t["first_fail_phase"] is not None:
            print(
                "    first failure      :",
                f"{t['first_fail_phase']} #{t['first_fail_idx']}",
            )
        print(
            "    last success frac  :",
            f"{t['last_success_fraction']:.3f}",
        )
        if t["limiting"] is not None:
            lj = t["limiting"]
            print(
                "    limiting joint     :",
                f"{lj['joint']}  q={lj['q']:.5f}  "
                f"range=[{lj['lower']:.4f}, {lj['upper']:.4f}]  "
                f"margin={lj['margin']:.5f}",
            )

    # ---- TOP staged detail ----
    print("\n" + "=" * 78)
    print("TOP CASE DETAIL (staged READY->SAFE_HIGH->TOP_ORIENTED->ABOVE->PREGRASP->GRASP)")
    print("=" * 78)

    for r in top_results:
        t = r["traj"]
        print(f"\n--- {r['name']} ---")
        print("  grasp pos          :", np.round(r["p_grasp"], 5))
        print("  selected_top_roll  :", r["selected_roll"], "deg")
        print("  GRASP_ENDPOINT     :", r["grasp_endpoint"])
        print("  PREGRASP_ENDPOINT  :", r["pregrasp_endpoint"])
        print("  staged TRAJECTORY  :", r["trajectory"])
        print("  FINAL VERDICT      :", r["final"])
        if t is not None:
            print("    IK failures        :", t["ik_failures"], "/", t["n_waypoints"])
            print("    min sigma_min      :", _fmt(t["min_sigma"]))
            print("    max condition      :", _fmt(t["max_condition"]))
            print("    min joint margin   :", _fmt(t["min_margin"]))
            print("    max q step         :", _fmt(t["max_dq"], 6), "rad")
            if t["first_fail_phase"] is not None:
                print(
                    "    first failure      :",
                    f"{t['first_fail_phase']} #{t['first_fail_idx']}",
                )
            print(
                "    last success frac  :",
                f"{t['last_success_fraction']:.3f}",
            )
            if t["limiting"] is not None:
                lj = t["limiting"]
                print(
                    "    limiting joint     :",
                    f"{lj['joint']}  q={lj['q']:.5f}  margin={lj['margin']:.5f}",
                )

    # ---- Special diagnostics ----
    side_low = next((r for r in results if r["name"] == "SIDE_LOW"), None)
    if side_low and side_low["traj"]["limiting"] is not None:
        lj = side_low["traj"]["limiting"]
        print("\n" + "-" * 78)
        print("SIDE_LOW limiting-joint detail:")
        print(
            f"  limiting_joint = {lj['joint']}   q = {lj['q']:.5f} rad   "
            f"range = [{lj['lower']:.4f}, {lj['upper']:.4f}]   "
            f"margin = {lj['margin']:.5f} rad"
        )

    side_far = next((r for r in results if r["name"] == "SIDE_FAR"), None)
    if side_far:
        t = side_far["traj"]
        print("\n" + "-" * 78)
        print("SIDE_FAR singularity progression (direct trajectory):")
        print(
            "  first sigma_min < 0.015 at:",
            f"{t['first_sigma_below']}" if t["first_sigma_below"] else "never",
        )
        print(
            "  first condition  > 80    at:",
            f"{t['first_cond_above']}" if t["first_cond_above"] else "never",
        )
        print(
            "  first IK failure         at:",
            f"{t['first_ik_fail']}" if t["first_ik_fail"] else "none",
        )
        print(
            "  => distinguishes gradual singularity boundary vs abrupt",
            "local-solver convergence failure.",
        )

    # ---- Edge case summary ----
    print("\n" + "=" * 78)
    print("GRASP EDGE CASE SUMMARY")
    print("=" * 78)
    print(
        f"{'case':<16} | {'grasp ep':<12} | {'pregrasp ep':<12} | "
        f"{'trajectory':<10} | limiting factor"
    )
    print("-" * 78)
    for r in results:
        if r["kind"] == "SIDE":
            lf = limiting_factor(r["traj"])
        else:
            lf = r["final"]
        print(
            f"{r['name']:<16} | {r['grasp_endpoint']:<12} | "
            f"{r['pregrasp_endpoint']:<12} | {r['trajectory']:<10} | {lf}"
        )

    # ---- TOP_CENTER special block ----
    tc = next((r for r in results if r["name"] == "TOP_CENTER"), None)
    if tc:
        print("\n" + "-" * 78)
        print("TOP_CENTER:")
        print("  selected roll        :", tc["selected_roll"], "deg")
        print("  endpoint reachable?  :", tc["grasp_endpoint"])
        print("  staged trajectory?   :", tc["trajectory"])
        print("  final verdict        :", tc["final"])
        if tc["traj"] is not None:
            print("  min sigma            :", _fmt(tc["traj"]["min_sigma"]))
            print("  max condition        :", _fmt(tc["traj"]["max_condition"]))
            print("  min joint margin     :", _fmt(tc["traj"]["min_margin"]))

    # ---- Baseline gate: SIDE_CENTER trajectory must PASS ----
    side_center = next((r for r in results if r["name"] == "SIDE_CENTER"), None)
    if (
        side_center is None
        or side_center["trajectory"] != "PASS"
        or side_center["grasp_endpoint"] != "REACHABLE"
    ):
        verdict = side_center["trajectory"] if side_center else "MISSING"
        print("\n" + "=" * 78)
        print("BASELINE GRASP VALIDATION: FAIL")
        print(f"SIDE_CENTER did not reach PASS (trajectory={verdict}).")
        print("=" * 78)
        sys.exit(1)

    print("\nGRASP POSE EDGE CASE SUITE: COMPLETE")


if __name__ == "__main__":
    main()
