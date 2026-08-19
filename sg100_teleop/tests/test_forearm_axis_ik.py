"""Forearm-axis constrained 5-DoF IK tests.

Covers the AVP forearm (joint[25]/[26]) direction cue mapped onto the
RM65 TCP local +Z with free roll, plus backward compatibility of the
existing IK modes.
"""

import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

from sg100_teleop.kinematics.rm65_ik import RM65IKSolver
from sg100_teleop.tracking.avp_wrist_mapper import (
    AVPWristRelativeMapper,
    make_transform,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = (
    REPO_ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/rm65_sg100_sim.xml"
)
RUNNER = (
    REPO_ROOT / "sg100_teleop/sim/run_avp_arm_hand_sim.py"
)

Q_HOME = np.array([0.0, -0.5, 1.0, 0.0, 0.8, 0.0])
E_Z = np.array([0.0, 0.0, 1.0])


def make_solver():
    return RM65IKSolver(
        mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    )


def rot_axis(axis, angle):
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    K = np.array(
        [[0, -axis[2], axis[1]],
         [axis[2], 0, -axis[0]],
         [-axis[1], axis[0], 0]]
    )
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def make_mapper():
    return AVPWristRelativeMapper()


# ----------------------------------------------------------------------
# TEST 1 — 27-joint semantics guard
# ----------------------------------------------------------------------

def test_27_joint_semantics_guard():
    from sg100_teleop.sim.run_avp_arm_hand_sim import (
        extract_forearm_axis,
    )

    ok27 = np.tile(np.eye(4), (27, 1, 1))
    ok27[25, :3, 3] = [0.1, 0.1, 1.30]
    ok27[26, :3, 3] = [0.1, 0.1, 1.00]

    n = extract_forearm_axis(ok27)
    assert n is not None
    assert np.allclose(n, [0, 0, 1]), n

    legacy = np.tile(np.eye(4), (25, 1, 1))
    assert extract_forearm_axis(legacy) is None

    # AVP joint index table must still map forearm joints to 25/26.
    from sg100_teleop.tracking.avp_hand_adapter import AVP_JOINT_INDEX
    assert AVP_JOINT_INDEX["forearmWrist"] == 25
    assert AVP_JOINT_INDEX["forearmArm"] == 26


# ----------------------------------------------------------------------
# TEST 2 — forearm vector direction
# ----------------------------------------------------------------------

def test_forearm_vector_direction():
    from sg100_teleop.sim.run_avp_arm_hand_sim import (
        extract_forearm_axis,
    )

    hand = np.tile(np.eye(4), (27, 1, 1))
    hand[25, :3, 3] = [0, 0, 1]   # forearmWrist (distal)
    hand[26, :3, 3] = [0, 0, 0]   # forearmArm  (proximal)

    n = extract_forearm_axis(hand)

    assert n is not None
    assert np.allclose(n, [0, 0, 1])


# ----------------------------------------------------------------------
# TEST 3 — calibration zero error (most important)
# ----------------------------------------------------------------------

def test_calibration_zero_error():
    rng = np.random.default_rng(3)

    for _ in range(10):
        mapper = make_mapper()
        R_avp0 = rot_axis(rng.normal(size=3), rng.uniform(0, np.pi))
        R_tcp0 = rot_axis(rng.normal(size=3), rng.uniform(0, np.pi))
        mapper.calibrate(
            make_transform(rng.normal(size=3), R_avp0),
            make_transform(rng.normal(size=3), R_tcp0),
        )

        n0 = rng.normal(size=3)
        n0 /= np.linalg.norm(n0)

        assert mapper.calibrate_forearm_axis(n0)

        a = mapper.map_forearm_direction(n0)
        assert a is not None
        expected = R_tcp0 @ E_Z
        assert np.allclose(a, expected, atol=1e-12), (
            f"calibration instant must be zero-error: {a} vs {expected}"
        )


# ----------------------------------------------------------------------
# TEST 4 — known relative rotation
# ----------------------------------------------------------------------

def test_known_relative_rotation():
    mapper = make_mapper()
    mapper.calibrate(
        make_transform(np.zeros(3), np.eye(3)),
        make_transform(np.zeros(3), np.eye(3)),
    )
    assert mapper.calibrate_forearm_axis(E_Z)

    a = mapper.map_forearm_direction(np.array([1.0, 0.0, 0.0]))

    assert np.allclose(a, [1, 0, 0], atol=1e-9)

    # Anti-parallel input exercises the stable 180-degree branch.
    opposite = mapper.map_forearm_direction(np.array([0.0, 0.0, -1.0]))
    assert opposite is not None
    assert np.all(np.isfinite(opposite))
    assert np.allclose(opposite, [0, 0, -1], atol=1e-9)


# ----------------------------------------------------------------------
# TEST 5 — roll invariance
# ----------------------------------------------------------------------

def test_roll_invariance():
    from sg100_teleop.kinematics.rm65_ik import (
        axis_alignment_residual,
    )

    rng = np.random.default_rng(7)

    for _ in range(5):
        a_target = rng.normal(size=3)
        a_target /= np.linalg.norm(a_target)
        R1 = rot_axis(rng.normal(size=3), rng.uniform(0, np.pi))
        a1 = R1 @ E_Z
        # align R1's +Z with the target, then roll about local +Z
        R_align = np.eye(3)
        for roll in (30.0, 90.0, 180.0):
            R2 = R1 @ rot_axis(E_Z, np.deg2rad(roll))
            r1 = axis_alignment_residual(R1 @ E_Z, a_target)
            r2 = axis_alignment_residual(R2 @ E_Z, a_target)
            assert np.allclose(r1, r2, atol=1e-12), (
                "roll about TCP +Z must not change the axis residual"
            )
            assert np.allclose(R1 @ E_Z, R2 @ E_Z, atol=1e-12)

        # aligned case: residual ~ 0 for any roll
        a = a_target
        for roll in (0.0, 90.0, 180.0):
            R = rot_axis(np.cross(E_Z, a), np.arccos(np.clip(a[2], -1, 1)))
            R = R @ rot_axis(E_Z, np.deg2rad(roll))
            assert np.linalg.norm(
                axis_alignment_residual(R @ E_Z, a)
            ) < 1e-9


# ----------------------------------------------------------------------
# TEST 6 — task rank
# ----------------------------------------------------------------------

def test_task_rank():
    s = make_solver()
    s._set_arm_q(Q_HOME)
    J_pos = s._position_jacobian()
    a = s.data.xmat[s.tcp_body_id].reshape(3, 3) @ E_Z
    J_rot = s._jacobian()[3:, :]
    P_perp = np.eye(3) - np.outer(a, a)
    J_axis = P_perp @ J_rot
    J_task = np.vstack([J_pos, J_axis])

    rank = np.linalg.matrix_rank(J_task, tol=1e-9)
    assert rank <= 5
    assert rank == 5, f"expected task rank 5, got {rank}"

    # pure axis Jacobian rank must be 2 (roll is the null direction)
    assert np.linalg.matrix_rank(J_axis, tol=1e-9) == 2


# ----------------------------------------------------------------------
# TEST 7/8/9 — backward compatibility of existing modes
# ----------------------------------------------------------------------

def test_existing_modes_compatibility():
    s = make_solver()
    rng = np.random.default_rng(11)

    for _ in range(3):
        q_true = np.clip(
            Q_HOME + rng.uniform(-0.3, 0.3, 6), s.lower, s.upper
        )
        p, R = s.fk(q_true)

        # full pose unchanged
        q1, r1 = s.solve(Q_HOME, p, R)
        assert r1.converged and r1.position_error < 2e-4

        # position-only unchanged
        q2, r2 = s.solve(Q_HOME, p, None, position_only=True)
        assert r2.converged and r2.position_error < 1e-4

        # fixed-orientation mode is the same full-pose API with a stable
        # calibration rotation target.
        q3, r3 = s.solve(Q_HOME, p, R)
        assert r3.converged and r3.position_error < 2e-4


# ----------------------------------------------------------------------
# TEST 10 — CLI mutual exclusion (three flags)
# ----------------------------------------------------------------------

def test_cli_mutual_exclusion():
    import itertools

    flags = ["--lock-tcp-rotation", "--position-only-ik",
             "--forearm-axis-ik"]
    for a, b in itertools.combinations(flags, 2):
        result = subprocess.run(
            [sys.executable, str(RUNNER), "--ip", "127.0.0.1", a, b],
            capture_output=True, text=True, cwd=str(REPO_ROOT),
        )
        assert result.returncode == 2, (a, b)
        assert "not allowed with argument" in result.stderr


# ----------------------------------------------------------------------
# TEST 11/12 — legacy & invalid forearm fallback
# ----------------------------------------------------------------------

def test_forearm_fallback_guards():
    from sg100_teleop.sim.run_avp_arm_hand_sim import (
        extract_forearm_axis,
        solve_forearm_axis_or_position,
    )

    # legacy 25-joint: no access to 25/26
    legacy = np.tile(np.eye(4), (25, 1, 1))
    assert extract_forearm_axis(legacy) is None

    # coincident points
    degenerate = np.tile(np.eye(4), (27, 1, 1))
    assert extract_forearm_axis(degenerate) is None

    # NaN positions
    nan_hand = np.tile(np.eye(4), (27, 1, 1))
    nan_hand[25, :3, 3] = [np.nan, 0, 0]
    assert extract_forearm_axis(nan_hand) is None

    # Production feeds only the established 25 hand joints into the hand
    # adapter, so an invalid optional forearm cue does not break SG100 input.
    from sg100_teleop.tests.test_avp_feature_extractor import build_straight_hand
    from sg100_teleop.tests.test_hand_calibration import build_calibration
    from sg100_teleop.tracking.avp_adapter import AVPHandAdapter

    invalid_optional_forearm = build_straight_hand()
    invalid_optional_forearm[25, :3, 3] = [np.nan, 0, 0]
    canonical = AVPHandAdapter(build_calibration()).convert(
        invalid_optional_forearm[:25]
    )
    assert canonical is not None

    class RecordingSolver:
        def __init__(self):
            self.kwargs = None

        def solve(self, **kwargs):
            self.kwargs = kwargs
            return kwargs["q_seed"], object()

    mapper = make_mapper()
    mapper.calibrate(
        make_transform(np.zeros(3), np.eye(3)),
        make_transform(np.zeros(3), np.eye(3)),
    )
    solver = RecordingSolver()
    solve_forearm_axis_or_position(
        solver,
        mapper,
        Q_HOME,
        np.zeros(3),
        None,
        False,
    )
    assert solver.kwargs["position_only"] is True
    assert solver.kwargs["target_rotation"] is None
    assert "target_axis" not in solver.kwargs


def test_mapper_without_forearm_anchor():
    mapper = make_mapper()
    mapper.calibrate(
        make_transform(np.zeros(3), np.eye(3)),
        make_transform(np.zeros(3), np.eye(3)),
    )
    # no forearm calibration -> direction mapping unavailable
    assert mapper.map_forearm_direction(E_Z) is None


# ----------------------------------------------------------------------
# TEST 13 — recalibration re-anchors
# ----------------------------------------------------------------------

def test_recalibration_reanchors():
    mapper = make_mapper()
    R_tcp_A = rot_axis([0, 1, 0], np.deg2rad(30))
    mapper.calibrate(
        make_transform(np.zeros(3), np.eye(3)),
        make_transform(np.zeros(3), R_tcp_A),
    )
    n0_A = np.array([0.0, 1.0, 0.0])
    assert mapper.calibrate_forearm_axis(n0_A)
    a_A = mapper.map_forearm_direction(n0_A)
    assert np.allclose(a_A, R_tcp_A @ E_Z, atol=1e-12)

    # second calibration: different TCP pose and different forearm
    R_tcp_B = rot_axis([1, 0, 0], np.deg2rad(-50))
    mapper.calibrate(
        make_transform(np.array([0.2, 0.1, 0.0]), np.eye(3)),
        make_transform(np.zeros(3), R_tcp_B),
    )
    n0_B = np.array([1.0, 0.3, -0.2])
    n0_B /= np.linalg.norm(n0_B)
    assert mapper.calibrate_forearm_axis(n0_B)

    a_B = mapper.map_forearm_direction(n0_B)
    assert np.allclose(a_B, R_tcp_B @ E_Z, atol=1e-12), (
        "recalibration must re-anchor to the NEW TCP +Z, not inherit "
        "the first axis anchor"
    )


# ----------------------------------------------------------------------
# TEST 14 — continuity
# ----------------------------------------------------------------------

def test_continuity():
    mapper = make_mapper()
    R_tcp0 = rot_axis([0, 1, 0], np.deg2rad(20))
    mapper.calibrate(
        make_transform(np.zeros(3), np.eye(3)),
        make_transform(np.zeros(3), R_tcp0),
    )
    n0 = np.array([0.0, 0.2, 1.0])
    n0 /= np.linalg.norm(n0)
    mapper.calibrate_forearm_axis(n0)

    prev = None
    axis = np.array([0.0, 1.0, 0.0])
    n_start = n0.copy()
    for deg in range(0, 21, 5):
        R = rot_axis(axis, np.deg2rad(deg))
        n = R @ n_start
        a = mapper.map_forearm_direction(n)
        assert a is not None
        if prev is not None:
            assert np.linalg.norm(a - prev) < 0.2, (
                f"axis target jumped at {deg} deg"
            )
        prev = a

    # IK-level continuity for small position+axis steps
    s = make_solver()
    p0, R0 = s.fk(Q_HOME)
    a0 = R0 @ E_Z
    q_prev = Q_HOME.copy()
    for k in range(1, 7):
        dp = np.array([0.01, -0.004, -0.006]) * k
        a_k = rot_axis([0, 1, 0], np.deg2rad(2 * k)) @ a0
        q_k, res_k = s.solve(
            q_prev, p0 + dp, None, target_axis=a_k,
            tcp_axis_local=E_Z,
        )
        assert res_k.converged, k
        step = float(np.abs(q_k - q_prev).max())
        assert step < 0.2, f"branch flip at step {k}: {step}"
        q_prev = q_k


# ----------------------------------------------------------------------
# Solver behaviour: axis task actually tracks the target axis
# ----------------------------------------------------------------------

def test_axis_task_tracking():
    s = make_solver()
    p0, R0 = s.fk(Q_HOME)

    a_target = rot_axis([0, 1, 0], np.deg2rad(25)) @ (R0 @ E_Z)
    q_sol, res = s.solve(
        Q_HOME, p0, None, target_axis=a_target, tcp_axis_local=E_Z,
    )

    assert res.converged
    assert res.position_error < 5e-4

    _, R_sol = s.fk(q_sol)
    a_sol = R_sol @ E_Z
    angle = np.arccos(np.clip(np.dot(a_sol, a_target), -1, 1))
    assert angle < np.deg2rad(2.0), f"axis error {np.rad2deg(angle):.2f} deg"


def main():
    tests = [
        ("27_joint_semantics_guard", test_27_joint_semantics_guard),
        ("forearm_vector_direction", test_forearm_vector_direction),
        ("calibration_zero_error", test_calibration_zero_error),
        ("known_relative_rotation", test_known_relative_rotation),
        ("roll_invariance", test_roll_invariance),
        ("task_rank", test_task_rank),
        ("existing_modes_compatibility", test_existing_modes_compatibility),
        ("cli_mutual_exclusion", test_cli_mutual_exclusion),
        ("forearm_fallback_guards", test_forearm_fallback_guards),
        ("mapper_without_forearm_anchor", test_mapper_without_forearm_anchor),
        ("recalibration_reanchors", test_recalibration_reanchors),
        ("continuity", test_continuity),
        ("axis_task_tracking", test_axis_task_tracking),
    ]

    failures = []
    for name, fn in tests:
        try:
            fn()
        except AssertionError as exc:
            failures.append(name)
            print(f"[FAIL] {name}: {exc}")
        except Exception as exc:  # noqa: BLE001
            failures.append(name)
            print(f"[ERROR] {name}: {type(exc).__name__}: {exc}")
        else:
            print(f"[PASS] {name}")

    if failures:
        print(f"\nFOREARM-AXIS IK: {len(failures)} FAILED")
        raise SystemExit(1)
    print("\nFOREARM-AXIS IK: ALL PASS")


if __name__ == "__main__":
    main()
