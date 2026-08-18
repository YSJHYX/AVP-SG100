"""Position-only IK mode tests for RM65.

Verifies that --position-only-ik is a TRUE 3D translation task:
orientation is not a task residual, the Jacobian is 3x6, convergence
ignores rotation, redundant solutions stay continuous, and the full-pose
/ fixed-rotation modes are unchanged.
"""

import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from sg100_teleop.kinematics.rm65_ik import (
    POSITION_ONLY_TOLERANCE_M,
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
RUNNER = (
    REPO_ROOT
    / "sg100_teleop"
    / "sim"
    / "run_avp_arm_hand_sim.py"
)

Q_HOME = np.array(
    [0.0, -0.5, 1.0, 0.0, 0.8, 0.0],
    dtype=np.float64,
)


def make_solver():
    return RM65IKSolver(
        mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    )


# ----------------------------------------------------------------------
# TEST 1 — backward compatibility (default full pose)
# ----------------------------------------------------------------------

def test_full_pose_backward_compatibility():
    s = make_solver()
    rng = np.random.default_rng(5)

    for _ in range(5):
        q_true = np.clip(
            Q_HOME + rng.uniform(-0.4, 0.4, size=6),
            s.lower,
            s.upper,
        )
        p_true, R_true = s.fk(q_true)

        q_sol, res = s.solve(
            Q_HOME,
            p_true,
            R_true,
        )

        assert res.converged
        assert res.position_error < 2e-4
        assert res.rotation_error < 2e-3
        assert np.all(q_sol >= s.lower - 1e-9)
        assert np.all(q_sol <= s.upper + 1e-9)

        # Explicit default kwarg must behave identically.
        q_sol2, res2 = s.solve(
            Q_HOME,
            p_true,
            R_true,
            position_only=False,
        )
        assert np.allclose(q_sol, q_sol2, atol=1e-12)
        assert res2.converged


# ----------------------------------------------------------------------
# TEST 2 — fixed-rotation compatibility (uses anchor rotation input)
# ----------------------------------------------------------------------

def test_fixed_rotation_mode_uses_supplied_rotation():
    s = make_solver()
    p0, R0 = s.fk(Q_HOME)

    # --lock-tcp-rotation path in the runner passes R_anchor into the
    # same full-pose solve; that behaviour is untouched.
    q_sol, res = s.solve(Q_HOME, p0, R0)

    assert res.converged
    _, R_actual = s.fk(q_sol)
    assert np.linalg.norm(so3_log(R0 @ R_actual.T)) < 2e-3


# ----------------------------------------------------------------------
# TEST 3 — position-only ignores orientation entirely
# ----------------------------------------------------------------------

def test_position_only_ignores_orientation():
    s = make_solver()
    p0, _ = s.fk(Q_HOME)
    p_target = p0 + np.array([0.10, 0.02, -0.05])

    R_a = Rotation.from_euler(
        "xyz", [140.0, -80.0, 60.0], degrees=True
    ).as_matrix()
    R_b = Rotation.from_euler(
        "xyz", [-30.0, 170.0, 10.0], degrees=True
    ).as_matrix()

    q_none, res_none = s.solve(
        Q_HOME, p_target, None, position_only=True
    )
    q_a, res_a = s.solve(
        Q_HOME, p_target, R_a, position_only=True
    )
    q_b, res_b = s.solve(
        Q_HOME, p_target, R_b, position_only=True
    )

    assert res_none.converged and res_a.converged and res_b.converged

    # The solver API must not consume orientation: identical outputs.
    assert np.array_equal(q_none, q_a)
    assert np.array_equal(q_none, q_b)


# ----------------------------------------------------------------------
# TEST 4 — 3D task Jacobian shape
# ----------------------------------------------------------------------

def test_position_jacobian_shape():
    s = make_solver()
    s._set_arm_q(Q_HOME)

    J = s._position_jacobian()

    assert J.shape == (3, 6)


# ----------------------------------------------------------------------
# TEST 5 — convergence ignores rotation, full-pose does not
# ----------------------------------------------------------------------

def test_convergence_ignores_rotation():
    s = make_solver()
    p0, R0 = s.fk(Q_HOME)

    # An orientation far away from the current one.
    R_far = Rotation.from_euler(
        "xyz", [150.0, 60.0, -120.0], degrees=True
    ).as_matrix()

    # Position-only: converges to the (already-satisfied) position and
    # never looks at the orientation.
    q_po, res_po = s.solve(
        Q_HOME, p0, R_far, position_only=True
    )
    assert res_po.converged
    assert res_po.position_error <= POSITION_ONLY_TOLERANCE_M

    # Full pose against the far orientation must NOT report a valid
    # solution at the same position (HOLD keeps the seed with a large
    # rotation error or the solve fails outright).
    q_full, res_full = s.solve(Q_HOME, p0, R_far)
    _, R_full = s.fk(q_full)
    rot_err = np.linalg.norm(so3_log(R_far @ R_full.T))
    assert (not res_full.converged) or rot_err > 1e-2


# ----------------------------------------------------------------------
# TEST 6 — redundant solution continuity (no branch flips)
# ----------------------------------------------------------------------

def test_redundant_solution_continuity():
    s = make_solver()
    p0, _ = s.fk(Q_HOME)

    dx = np.array([0.015, -0.006, -0.008])
    q_prev = Q_HOME.copy()
    max_step = 0.0

    for k in range(1, 13):
        q_k, res_k = s.solve(
            q_prev,
            p0 + k * dx,
            None,
            position_only=True,
        )
        assert res_k.converged, f"step {k}"

        step = float(np.abs(q_k - q_prev).max())
        max_step = max(max_step, step)
        assert step < 0.15, (
            f"branch flip at step {k}: max |dq| = {step}"
        )
        q_prev = q_k

    assert max_step > 0.0, "solution never moved"


# ----------------------------------------------------------------------
# TEST 7 — joint safety
# ----------------------------------------------------------------------

def test_joint_safety():
    s = make_solver()
    rng = np.random.default_rng(11)
    p0, _ = s.fk(Q_HOME)

    for _ in range(10):
        target = p0 + rng.uniform(-0.25, 0.25, size=3)
        q_sol, res = s.solve(
            Q_HOME, target, None, position_only=True
        )
        assert np.all(np.isfinite(q_sol))
        assert np.all(q_sol >= s.lower - 1e-9)
        assert np.all(q_sol <= s.upper + 1e-9)


# ----------------------------------------------------------------------
# TEST 8 — CLI mutual exclusion
# ----------------------------------------------------------------------

def test_cli_mutual_exclusion():
    result = subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--ip", "127.0.0.1",
            "--lock-tcp-rotation",
            "--position-only-ik",
        ],
        capture_output=True,
        text=True,
        cwd=str(REPO_ROOT),
    )

    assert result.returncode == 2
    assert "not allowed with argument" in result.stderr


# ----------------------------------------------------------------------
# Reachability regression — position-only never has fewer solutions
# ----------------------------------------------------------------------

def test_reachability_regression():
    s = make_solver()
    p0, R_anchor = s.fk(Q_HOME)

    offsets = []
    for axis in np.eye(3):
        for dist in (0.10, 0.20, 0.30):
            offsets.append(axis * dist)
            offsets.append(-axis * dist)

    po_successes = 0
    full_successes = 0
    freed_targets = []

    for off in offsets:
        target = p0 + off

        _, res_po = s.solve(
            Q_HOME, target, None, position_only=True
        )
        _, res_full = s.solve(
            Q_HOME, target, R_anchor
        )

        po_successes += int(res_po.converged)
        full_successes += int(res_full.converged)

        # Dominance property: orientation-free task never fails where
        # the fixed-orientation task succeeds.
        if res_full.converged:
            assert res_po.converged, (
                f"position-only failed where fixed-orientation "
                f"succeeded at offset {off}"
            )

        if res_po.converged and not res_full.converged:
            freed_targets.append(off)

    assert po_successes >= full_successes
    # The regression must capture at least one target that the
    # orientation constraint made unreachable but position-only
    # reaches — the whole point of the mode.
    assert freed_targets, (
        "no target freed by dropping the orientation constraint; "
        "regression lost its discriminative power"
    )
    print(
        f"    reachability: position-only {po_successes}/{len(offsets)} "
        f"targets, fixed-orientation {full_successes}/{len(offsets)}, "
        f"freed by orientation removal: {len(freed_targets)}"
    )


def main():
    tests = [
        ("full_pose_backward_compatibility",
         test_full_pose_backward_compatibility),
        ("fixed_rotation_mode_uses_supplied_rotation",
         test_fixed_rotation_mode_uses_supplied_rotation),
        ("position_only_ignores_orientation",
         test_position_only_ignores_orientation),
        ("position_jacobian_shape",
         test_position_jacobian_shape),
        ("convergence_ignores_rotation",
         test_convergence_ignores_rotation),
        ("redundant_solution_continuity",
         test_redundant_solution_continuity),
        ("joint_safety", test_joint_safety),
        ("cli_mutual_exclusion", test_cli_mutual_exclusion),
        ("reachability_regression", test_reachability_regression),
    ]

    failures = []
    for name, fn in tests:
        try:
            fn()
        except AssertionError as exc:
            failures.append(name)
            print(f"[FAIL] {name}: {exc}")
        else:
            print(f"[PASS] {name}")

    if failures:
        print(f"\nPOSITION-ONLY IK: {len(failures)} FAILED")
        raise SystemExit(1)
    print("\nPOSITION-ONLY IK: ALL PASS")


if __name__ == "__main__":
    main()
