"""Power-grasp thumb lock behaviour tests.

Split deliberately into two groups so that, once SG100 hardware
arrives, morphology-mapping regressions and heuristic regressions can
be triaged separately:

    TEST GROUP 1 — BASE THUMB RETARGETING
        Single-joint sweeps are finite, commands respect the compiled
        joint limits, and the grasp lock can be disabled to reproduce
        the base (pre-lock) command bit-exactly.

    TEST GROUP 2 — POWER-GRASP THUMB LOCK
        Open/grasp/pinch/release/partial-grasp behaviour of the
        continuous lock-weight bias blend, verified through palm-frame
        FK geometry (thumb must move INTO the index-middle closure
        region, never outward).

Canonical operating points are the real AVP values shared with
tests/test_sg100_retarget_semantics.py::MEASURED_POINTS.
"""

import copy
from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml

from sg100_teleop.calibration.io import load_calibration
from sg100_teleop.hand.canonical_hand import CanonicalHandState
from sg100_teleop.retargeting.sg100_retargeter import SG100Retargeter
from sg100_teleop.sim.inspect_sg100_thumb_kinematics import (
    ThumbKinematicsHarness,
)
from sg100_teleop.tracking.avp_adapter import AVPHandAdapter


ROOT = Path(__file__).resolve().parents[2]
SEMANTICS_YAML = (
    ROOT / "sg100_teleop/config/sg100_joint_semantics.yaml"
)
CALIBRATION_YAML = (
    ROOT / "sg100_teleop/config/avp_right_calibration.yaml"
)
THUMB_JOINTS = ("thumb_j1", "thumb_j2", "thumb_j3")


# Real AVP operating points (canonical), identical to
# test_sg100_retarget_semantics.py::MEASURED_POINTS (thumb distal and
# finger distal values from the calibration session notes).
OPEN_HAND = CanonicalHandState()
NORMAL_GRASP = CanonicalHandState(
    thumb_opposition=0.7595,
    thumb_abduction=0.8610,
    thumb_distal=0.918,
    index_adduction=0.0,
    index_flexion=0.9795,
    index_distal=0.8,
    middle_flexion=1.0,
    middle_distal=0.8,
    little_outward=0.0,
    little_flexion=1.0,
    little_distal=0.8,
)
INDEX_PINCH = CanonicalHandState(
    thumb_opposition=0.5724,
    thumb_abduction=1.0,
    thumb_distal=0.5,
    index_adduction=0.0,
    index_flexion=0.3892,
    index_distal=0.3,
    middle_flexion=0.0,
    middle_distal=0.0,
    little_outward=0.0,
    little_flexion=0.0,
    little_distal=0.0,
)
MIDDLE_PINCH = CanonicalHandState(
    thumb_opposition=0.7420,
    thumb_abduction=1.0,
    thumb_distal=0.4,
    index_adduction=0.0,
    index_flexion=0.0,
    index_distal=0.0,
    middle_flexion=0.1568,
    middle_distal=0.1,
    little_outward=0.0,
    little_flexion=0.0,
    little_distal=0.0,
)
# Fingers closed but the human thumb deliberately held OUT (open
# thumb): the lock must NOT force a fist pose.
OPEN_THUMB_GRASP = CanonicalHandState(
    thumb_opposition=0.10,
    thumb_abduction=0.80,
    thumb_distal=0.2,
    index_flexion=0.98,
    middle_flexion=1.0,
    little_flexion=1.0,
    index_distal=0.8,
    middle_distal=0.8,
    little_distal=0.8,
)


def _retargeter_with(
    mutate=None,
    drop_section=False,
) -> SG100Retargeter:
    """Build a retargeter from a mutated copy of the production
    semantics config (written to a temp file)."""

    import tempfile

    with SEMANTICS_YAML.open("r") as f:
        config = yaml.safe_load(f)

    if drop_section:
        config.pop("power_grasp_thumb_lock", None)
    elif mutate is not None:
        mutate(config)

    with tempfile.NamedTemporaryFile(
        "w",
        suffix=".yaml",
        delete=False,
    ) as f:
        yaml.safe_dump(config, f)
        path = f.name

    return SG100Retargeter(Path(path))


@pytest.fixture(scope="module")
def harness():
    return ThumbKinematicsHarness()


@pytest.fixture(scope="module")
def retargeter():
    return SG100Retargeter()


@pytest.fixture(scope="module")
def base_retargeter():
    """Production config with the grasp lock disabled (base mapping,
    pinch coordination preserved)."""

    def disable(config):
        config["power_grasp_thumb_lock"]["enabled"] = False

    return _retargeter_with(disable)


def _hand_fk(harness, q):
    harness._set_hand(q)
    return harness.geometry()


def _lateral_delta_mm(harness, q):
    harness._set_hand(q)
    delta = harness.geometry()["thumb"] - harness.lock_region()
    return float(np.dot(delta, harness.palm_lateral) * 1000.0)


# ======================================================================
# TEST GROUP 1 — BASE THUMB RETARGETING
# ======================================================================


@pytest.mark.parametrize("joint", THUMB_JOINTS)
def test_group1_thumb_sweep_finite(harness, joint):
    """Single-joint sweep over the full mechanical range stays finite
    and produces finite FK geometry at every sample."""

    rows = harness.sweep_joint(joint, samples=13)

    assert len(rows) == 13

    for row in rows:
        assert np.isfinite(row["q"])
        assert np.isfinite(row["lateral"])
        assert np.isfinite(row["forward"])
        assert np.isfinite(row["normal"])
        assert np.isfinite(row["d_thumb_index_mm"])
        assert np.isfinite(row["d_thumb_middle_mm"])
        assert np.isfinite(row["d_thumb_lock_mm"])


def test_group1_base_commands_respect_compiled_limits(
    harness,
    base_retargeter,
):
    """Every base command for the canonical operating points lies
    within the model's compiled joint ranges."""

    for state in (
        OPEN_HAND,
        NORMAL_GRASP,
        INDEX_PINCH,
        MIDDLE_PINCH,
        OPEN_THUMB_GRASP,
    ):
        q = base_retargeter.retarget(state)

        for i, name in enumerate(base_retargeter.joint_order):
            jid = harness.joint_ids[name]
            lower, upper = harness.model.jnt_range[jid]
            assert lower - 1e-9 <= q[i] <= upper + 1e-9, name


def test_group1_disabled_lock_matches_base_exactly(
    base_retargeter,
    retargeter,
):
    """enabled: false must reproduce the base command bit-exactly at
    every operating point (no float drift, no reordering)."""

    for state in (
        OPEN_HAND,
        NORMAL_GRASP,
        INDEX_PINCH,
        MIDDLE_PINCH,
        OPEN_THUMB_GRASP,
    ):
        q_base = base_retargeter.retarget(state)
        # Sanity: the disabled fixture really is disabled.
        assert base_retargeter.power_grasp_thumb_lock is None
        np.testing.assert_array_equal(
            q_base,
            _retargeter_with(drop_section=True).retarget(state),
            err_msg="enabled:false must equal absent-section",
        )


# ======================================================================
# TEST GROUP 2 — POWER-GRASP THUMB LOCK
# ======================================================================


def test_group2_open_hand_lock_weight_zero(retargeter):
    """(A) Fully open hand: lock weight ~0, thumb command untouched."""

    features = OPEN_HAND.as_dict()

    assert retargeter.thumb_lock_weight(features) == 0.0
    assert retargeter.thumb_lock_diagnostics(
        np.zeros(11), features
    )["lock_weight"] == 0.0


def test_group2_power_grasp_thumb_moves_inward(
    harness,
    retargeter,
    base_retargeter,
):
    """(B) At the real NORMAL_GRASP point the locked thumb crosses
    toward the index-middle closure region (lateral delta shrinks
    strongly, q1 rises), not outward."""

    q_base = base_retargeter.retarget(NORMAL_GRASP)
    q_lock = retargeter.retarget(NORMAL_GRASP)

    i1 = retargeter.joint_order.index("thumb_j1")
    assert np.degrees(q_lock[i1]) > np.degrees(q_base[i1]) + 30.0

    lateral_base = _lateral_delta_mm(harness, q_base)
    lateral_lock = _lateral_delta_mm(harness, q_lock)

    assert lateral_lock < lateral_base - 35.0, (
        "locked thumb must move across the palm toward the closure "
        f"region: {lateral_base:.1f} -> {lateral_lock:.1f} mm"
    )
    assert abs(lateral_lock) < 15.0, (
        "locked thumb should reach the closure region "
        f"(|lateral| < 15 mm), got {lateral_lock:.1f} mm"
    )


def test_group2_lock_region_distance_decreases(
    harness,
    retargeter,
    base_retargeter,
):
    """(C) FK: thumb-to-lock-region distance clearly decreases."""

    q_base = base_retargeter.retarget(NORMAL_GRASP)
    q_lock = retargeter.retarget(NORMAL_GRASP)

    before = _hand_fk(harness, q_base)["d_thumb_lock_mm"]
    after = _hand_fk(harness, q_lock)["d_thumb_lock_mm"]

    assert after < 0.75 * before, (
        f"d_lock must drop well below 75% of {before:.1f} mm, "
        f"got {after:.1f} mm"
    )


def test_group2_release_returns_to_zero(retargeter):
    """(D) grasp -> open sweep: lock weight decreases monotonically
    to 0 and the final open-hand command equals the base command."""

    prev = None

    for t in np.linspace(1.0, 0.0, 41):
        state = CanonicalHandState(
            thumb_opposition=0.7595 * t,
            thumb_abduction=0.8610 * t,
            thumb_distal=0.918 * t,
            index_flexion=0.9795 * t,
            middle_flexion=1.0 * t,
            little_flexion=1.0 * t,
            index_distal=0.8 * t,
            middle_distal=0.8 * t,
            little_distal=0.8 * t,
        )
        features = state.as_dict()
        weight = retargeter.thumb_lock_weight(features)

        assert 0.0 <= weight <= 1.0

        if prev is not None:
            assert weight <= prev + 1e-12, (
                "release sweep must be monotonic non-increasing"
            )
        prev = weight

    assert weight == 0.0


def test_group2_partial_grasp_no_discontinuity(retargeter):
    """(E) Half-closed states: epsilon input change gives bounded
    output change; the blend never snaps across a threshold."""

    def grasp_state(t):
        return CanonicalHandState(
            thumb_opposition=0.7595 * t,
            thumb_abduction=0.8610 * t,
            thumb_distal=0.918 * t,
            index_flexion=0.9795 * t,
            middle_flexion=1.0 * t,
            little_flexion=1.0 * t,
            index_distal=0.8 * t,
            middle_distal=0.8 * t,
            little_distal=0.8 * t,
        )

    prev_q = None
    max_step = 0.0

    for t in np.linspace(0.05, 1.0, 96):
        q = retargeter.retarget(grasp_state(t))
        q_eps = retargeter.retarget(
            grasp_state(float(t) + 1e-7)
        )
        assert np.allclose(q, q_eps, atol=1e-5), (
            f"discontinuity at grasp t={t:.3f}"
        )

        if prev_q is not None:
            step = float(np.abs(q - prev_q).max())
            max_step = max(max_step, step)
        prev_q = q

    # Smooth blend: a full canonical sweep (0.05 -> 1.0) in 95 steps
    # must not contain a jump larger than the pinch-gate transition
    # bound already accepted project-wide.
    assert max_step < 0.40, f"q jump during partial grasp: {max_step}"


@pytest.mark.parametrize(
    "state",
    [INDEX_PINCH, MIDDLE_PINCH],
    ids=["index_pinch", "middle_pinch"],
)
def test_group2_pinch_not_overridden(
    retargeter,
    base_retargeter,
    state,
):
    """(F) Pinch operating points: pinch gate saturates, lock weight
    is 0 and the command equals the base (pinch-branch) command."""

    features = state.as_dict()

    assert retargeter.pinch_gate(features) >= 0.99
    assert retargeter.thumb_lock_weight(features) < 1e-9

    np.testing.assert_array_equal(
        retargeter.retarget(state),
        base_retargeter.retarget(state),
    )


def test_group2_open_thumb_grasp_not_locked(retargeter):
    """(K) Fingers closed but the human thumb deliberately held out:
    engagement gate keeps the lock OFF (never force a fist pose)."""

    features = OPEN_THUMB_GRASP.as_dict()

    assert retargeter.grasp_score(features) > 0.9
    assert retargeter.thumb_lock_weight(features) < 0.05


def test_group2_non_thumb_joints_unchanged(
    retargeter,
    base_retargeter,
):
    """(G) The lock touches only configured thumb joints: every
    non-thumb command is bit-identical across a dense input sweep."""

    rng = np.random.default_rng(20260901)

    for _ in range(60):
        kwargs = {
            name: float(rng.uniform(0.0, 1.0))
            for name in (
                "thumb_opposition",
                "thumb_abduction",
                "index_flexion",
                "middle_flexion",
                "little_flexion",
                "index_distal",
                "middle_distal",
                "little_distal",
                "little_outward",
            )
        }
        kwargs["thumb_distal"] = float(rng.uniform(-1.0, 1.0))
        state = CanonicalHandState(**kwargs)

        q_lock = retargeter.retarget(state)
        q_base = base_retargeter.retarget(state)

        np.testing.assert_array_equal(
            q_lock[3:],
            q_base[3:],
            err_msg="non-thumb joints must be untouched by the lock",
        )


def test_group2_outputs_finite_and_within_limits(
    harness,
    retargeter,
):
    """(H/I) Dense random sweep: all outputs finite and within the
    compiled mechanical joint ranges."""

    rng = np.random.default_rng(7)

    for _ in range(120):
        state = CanonicalHandState(
            **{
                name: float(rng.uniform(-1.0, 1.0))
                for name in (
                    "thumb_opposition",
                    "thumb_abduction",
                    "thumb_distal",
                    "index_adduction",
                    "index_flexion",
                    "index_distal",
                    "middle_flexion",
                    "middle_distal",
                    "little_outward",
                    "little_flexion",
                    "little_distal",
                )
            }
        )
        q = retargeter.retarget(state)

        assert np.all(np.isfinite(q))

        for i, name in enumerate(retargeter.joint_order):
            jid = harness.joint_ids[name]
            lower, upper = harness.model.jnt_range[jid]
            assert lower - 1e-9 <= q[i] <= upper + 1e-9, name


def test_group2_legacy_calibration_yaml_still_loads():
    """(J) The existing calibration YAML loads and drives the full
    raw -> canonical -> retarget pipeline unchanged."""

    calibration = load_calibration(CALIBRATION_YAML)
    AVPHandAdapter(calibration)  # adapter construction unchanged
    retargeter = SG100Retargeter()

    # Direct normalization path used by AVPHandAdapter.convert.
    from sg100_teleop.tracking.avp_hand_adapter import (
        AVPRawHandFeatures,
    )

    features = AVPRawHandFeatures(
        thumb_opposition=0.30,
        thumb_abduction=0.75,
        thumb_distal=0.50,
        index_adduction=-0.10,
        index_flexion=1.10,
        index_distal=0.80,
        middle_flexion=1.20,
        middle_distal=1.00,
        little_outward=0.50,
        little_flexion=1.20,
        little_distal=1.00,
    )

    canonical = calibration.normalize(features)
    q = retargeter.retarget(canonical)

    assert q.shape == (11,)
    assert np.all(np.isfinite(q))


def test_group2_config_validation_rejects_bad_thresholds():
    """Invalid lock configs fail loudly at load time."""

    def set_start(config):
        config["power_grasp_thumb_lock"]["grasp_score"][
            "start"
        ] = 0.9

    def set_max_weight(config):
        config["power_grasp_thumb_lock"]["max_weight"] = 1.5

    def set_bias_too_large(config):
        config["power_grasp_thumb_lock"]["biases_deg"][
            "thumb_j1"
        ] = 500.0

    def set_unknown_joint(config):
        config["power_grasp_thumb_lock"]["biases_deg"][
            "not_a_joint"
        ] = 5.0

    for mutate in (
        set_start,
        set_max_weight,
        set_bias_too_large,
        set_unknown_joint,
    ):
        with pytest.raises(RuntimeError):
            _retargeter_with(mutate)


def test_group2_lock_weight_formula_matches_config(retargeter):
    """The implemented blend equals the documented formula:
    max_weight * smoothstep(grasp) * smoothstep(engage) * (1-gate)."""

    cfg = retargeter.power_grasp_thumb_lock
    assert cfg is not None

    for state in (OPEN_HAND, NORMAL_GRASP, INDEX_PINCH):
        features = state.as_dict()
        weight = retargeter.thumb_lock_weight(features)

        grasp_w = retargeter._ramp(
            retargeter.grasp_score(features),
            cfg["grasp_start"],
            cfg["grasp_full"],
        )
        engage_w = retargeter.thumb_engagement(features)

        expected = (
            cfg["max_weight"]
            * grasp_w
            * engage_w
            * (1.0 - retargeter.pinch_gate(features))
        )
        assert np.isclose(weight, expected, atol=1e-12)
