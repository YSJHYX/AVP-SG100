"""Semantic regression tests for AVP -> SG100 hand retargeting.

These tests lock the HUMAN semantic contract through to physical MuJoCo
motion:

    index_adduction canonical
        +1 = maximally TOWARD MIDDLE
         0 = NEUTRAL
        -1 = maximally TOWARD THUMB

    thumb_distal canonical +1 = thumb IP FLEX_TOWARD_PALM

Two distinct "neutral" concepts are asserted separately:

    HUMAN FEATURE NEUTRAL  -> canonical = 0
    ROBOT JOINT NEUTRAL    -> q = semantics zero (sg100_joint_semantics.yaml)

Direction verdicts are made with palm-relative FK geometry, never with a
single world-axis component.
"""

from pathlib import Path

import mujoco
import numpy as np
import yaml

from sg100_teleop.calibration.io import (
    calibration_from_dict,
)
from sg100_teleop.calibration.session import (
    CalibrationSession,
    MIN_SPAN_TO_NOISE_RATIO,
)
from sg100_teleop.hand.canonical_hand import CanonicalHandState
from sg100_teleop.hand.hand_calibration import BidirectionalCalibration
from sg100_teleop.retargeting.sg100_retargeter import SG100Retargeter
from sg100_teleop.tracking.avp_hand_adapter import AVPRawHandFeatures


ROOT = Path(__file__).resolve().parents[2]
SIM_XML = ROOT / "sg100_teleop/assets/rm65_sg100/mjcf/rm65_sg100_sim.xml"
SEMANTICS_YAML = ROOT / "sg100_teleop/config/sg100_joint_semantics.yaml"

THUMB_JOINTS = ("thumb_j1", "thumb_j2", "thumb_j3")
INDEX_JOINTS = ("index_j1", "index_j2", "index_j3")


def semantics_zero():
    with SEMANTICS_YAML.open("r") as f:
        config = yaml.safe_load(f)
    return {
        name: float(cfg["zero"])
        for name, cfg in config["joints"].items()
    }


class PalmFrameHarness:
    """Palm-relative FK measurement on the combined sim model."""

    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(SIM_XML))
        self.data = mujoco.MjData(self.model)
        self.retargeter = SG100Retargeter()
        self.zero = semantics_zero()

        assert tuple(self.retargeter.joint_order[:3]) == THUMB_JOINTS
        assert tuple(self.retargeter.joint_order[3:6]) == INDEX_JOINTS

        self._set_hand(np.zeros(11))

        index_mcp = self._body("index_link1")
        middle_mcp = self._body("middle_link1")
        little_mcp = self._body("little_link1")
        base = self._body("sg100_base_link")

        origin = (index_mcp + middle_mcp + little_mcp) / 3.0
        lateral_raw = index_mcp - little_mcp
        forward = origin - base
        forward /= np.linalg.norm(forward)
        lateral = lateral_raw - np.dot(lateral_raw, forward) * forward
        lateral /= np.linalg.norm(lateral)
        normal = np.cross(lateral, forward)
        normal /= np.linalg.norm(normal)

        # Sign the palm normal physically: index MCP flexion curls toward
        # the palm interior.  Capture the straight tip BEFORE applying
        # the flexed pose (reading it afterwards would zero the delta).
        straight = self._geom("index_link3_visual")
        flexed = self._with_hand(
            np.deg2rad(np.array([0, 0, 0, 0, 60, 45, 0, 0, 0, 0, 0]))
        )
        normal *= np.sign(np.dot(flexed - straight, normal))
        assert np.linalg.norm(normal) > 0.5, "palm normal degenerated"

        self.origin = origin
        self.lateral = lateral  # + = thumb side
        self.normal = normal    # + = palm interior

    # -- FK helpers ------------------------------------------------------

    def _set_hand(self, q_hand):
        self.data.qpos[:] = 0.0
        for i, name in enumerate(self.retargeter.joint_order):
            jid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            self.data.qpos[int(self.model.jnt_qposadr[jid])] = q_hand[i]
        mujoco.mj_forward(self.model, self.data)

    def _with_hand(self, q_hand):
        self._set_hand(q_hand)
        return self._geom("index_link3_visual")

    def _body(self, name):
        bid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, name
        )
        return self.data.xpos[bid].copy()

    def _geom(self, name):
        gid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, name
        )
        return self.data.geom_xpos[gid].copy()

    # -- semantic measurements -------------------------------------------

    def q_for(self, **features):
        return self.retargeter.retarget(CanonicalHandState(**features))

    def index_middle_separation(self):
        """Signed lateral angle between index and middle proximal segments.

        Larger = index further toward the THUMB side.
        """
        i_dir = self._body("index_link2") - self._body("index_link1")
        m_dir = self._body("middle_link2") - self._body("middle_link1")
        i_dir /= np.linalg.norm(i_dir)
        m_dir /= np.linalg.norm(m_dir)
        return float(
            np.arctan2(
                np.dot(self.normal, np.cross(m_dir, i_dir)),
                np.dot(m_dir, i_dir),
            )
        )

    def index_distance_to_thumb(self):
        tip = self._geom("index_link3_visual")
        return float(np.linalg.norm(tip - self._body("thumb_link1")))

    def index_distance_to_middle(self):
        tip = self._geom("index_link3_visual")
        return float(np.linalg.norm(tip - self._body("middle_link1")))

    def thumb_distance_to_palm_center(self):
        tip = self._geom("thumb_link3_visual")
        return float(np.linalg.norm(tip - self.origin))

    def measure(self, q_hand):
        self._set_hand(q_hand)
        return {
            "separation": self.index_middle_separation(),
            "dist_thumb": self.index_distance_to_thumb(),
            "dist_middle": self.index_distance_to_middle(),
            "thumb_palm": self.thumb_distance_to_palm_center(),
        }


# ----------------------------------------------------------------------
# TEST 1 — index adduction POSITIVE semantic (toward middle)
# ----------------------------------------------------------------------

def test_index_adduction_positive_moves_toward_middle():
    h = PalmFrameHarness()

    q_neutral = h.q_for()
    q_positive = h.q_for(index_adduction=+1.0)
    q_negative = h.q_for(index_adduction=-1.0)

    i_j1 = h.retargeter.joint_order.index("index_j1")

    # Canonical + (toward middle) must command the numeric q toward the
    # q_min side, never toward q_max (the toward-thumb side).
    assert q_positive[i_j1] <= q_neutral[i_j1] + 1e-12
    assert q_neutral[i_j1] <= q_negative[i_j1] + 1e-12

    m_pos = h.measure(q_positive)
    m_neg = h.measure(q_negative)

    # Physically: canonical + must be closer to the middle finger and
    # further from the thumb than canonical -.
    assert m_pos["dist_middle"] < m_neg["dist_middle"], (
        "canonical index_adduction=+1 (toward middle) must not command "
        "a pose further from the middle finger than index_adduction=-1"
    )
    assert m_pos["dist_thumb"] > m_neg["dist_thumb"]


# ----------------------------------------------------------------------
# TEST 2 — index adduction NEGATIVE semantic (toward thumb)
# ----------------------------------------------------------------------

def test_index_adduction_negative_moves_toward_thumb():
    h = PalmFrameHarness()

    q_neutral = h.q_for()
    q_negative = h.q_for(index_adduction=-1.0)

    i_j1 = h.retargeter.joint_order.index("index_j1")
    lower, upper = map(
        float, h.retargeter.joints["index_j1"]["range"]
    )

    assert np.isclose(q_negative[i_j1], upper), (
        "canonical index_adduction=-1 (toward thumb) must reach the "
        "numeric q_max side (robot q+ = physically toward thumb)"
    )

    m_neutral = h.measure(q_neutral)
    m_negative = h.measure(q_negative)

    assert m_negative["dist_thumb"] < m_neutral["dist_thumb"]
    assert m_negative["dist_middle"] > m_neutral["dist_middle"]


# ----------------------------------------------------------------------
# TEST 3 — index neutral maps to ROBOT zero, not implicit q=0
# ----------------------------------------------------------------------

def test_index_adduction_neutral_uses_semantics_zero():
    h = PalmFrameHarness()
    q = h.q_for()
    i_j1 = h.retargeter.joint_order.index("index_j1")
    assert np.isclose(q[i_j1], h.zero["index_j1"]), (
        "canonical index neutral must map to semantics zero "
        f"({h.zero['index_j1']}), not an implicit numerical zero"
    )


# ----------------------------------------------------------------------
# TEST 4 — thumb robot neutral from semantics
# ----------------------------------------------------------------------

def test_thumb_neutral_uses_semantics_zero():
    h = PalmFrameHarness()
    q = h.q_for()
    for joint in ("thumb_j1", "thumb_j2"):
        i = h.retargeter.joint_order.index(joint)
        assert np.isclose(q[i], h.zero[joint]), (
            f"canonical thumb neutral must map {joint} to semantics zero "
            f"({h.zero[joint]})"
        )


# ----------------------------------------------------------------------
# TEST 5 — thumb_j3 sign regression guard
# ----------------------------------------------------------------------

def test_thumb_distal_positive_flexes_toward_palm():
    h = PalmFrameHarness()

    q_neutral = h.q_for()
    q_flex = h.q_for(thumb_distal=+1.0)

    m_neutral = h.measure(q_neutral)
    m_flex = h.measure(q_flex)

    # Robust metric from the direction re-audit: thumb_j3 q+ curls the
    # thumb tip toward the palm center (in-plane curl).
    assert m_flex["thumb_palm"] < m_neutral["thumb_palm"], (
        "canonical thumb_distal=+1 must physically flex the thumb IP "
        "toward the palm; do NOT flip this mapping or the MJCF axis"
    )


# ----------------------------------------------------------------------
# Finger distal redesign — unidirectional [0, 1] flexion contract
# ----------------------------------------------------------------------

FINGER_DISTAL_JOINTS = {
    "index_distal": "index_j3",
    "middle_distal": "middle_j2",
    "little_distal": "little_j3",
}

FEATURE_FOR_JOINT = {
    joint: feature
    for feature, joint in FINGER_DISTAL_JOINTS.items()
}


def test_finger_distal_straight_maps_to_robot_zero():
    """Human distal straight/open (canonical 0) -> robot zero."""
    h = PalmFrameHarness()
    q = h.q_for()

    for feature, joint in FINGER_DISTAL_JOINTS.items():
        i = h.retargeter.joint_order.index(joint)
        assert np.isclose(q[i], h.zero[joint]), (
            f"canonical {feature}=0 (straight) must map {joint} to "
            f"semantics zero ({h.zero[joint]}), got {q[i]}"
        )


def test_finger_distal_negative_input_never_hits_mechanical_limit():
    """Relaxed/straight poses must not map to the -60 deg lower limit.

    Human distal joints are not physiologically operable in reverse
    (hyperextension), so canonical input can only be [0, 1].  Any
    negative raw-derived canonical value clips to 0 -> robot zero,
    never to the mechanical lower limit.
    """
    h = PalmFrameHarness()

    for feature, joint in FINGER_DISTAL_JOINTS.items():
        i = h.retargeter.joint_order.index(joint)
        lower, upper = map(float, h.retargeter.joints[joint]["range"])

        for canonical_value in (0.0, -0.3, -1.0):
            q = h.q_for(**{feature: canonical_value})
            assert np.isclose(q[i], h.zero[joint]), (
                f"{feature}={canonical_value} must clip to zero "
                f"({h.zero[joint]}), got {q[i]}"
            )
            assert not np.isclose(q[i], lower), (
                f"{feature}={canonical_value} must never reach the "
                f"mechanical lower limit {lower} (old -60 deg bug)"
            )


def test_finger_distal_flexion_moves_toward_palm():
    """Canonical distal flexion (1) curls the fingertip toward the palm."""
    h = PalmFrameHarness()

    # Measure the distal body inertial COM (xipos): unlike the visual
    # geom center, it is offset from the hinge axis at the straight
    # configuration and therefore shows the physical curl direction.
    tips = {
        "index_j3": "index_link3",
        "middle_j2": "middle_link2",
        "little_j3": "little_linik3",
    }

    def palm_normal_of(body_name):
        bid = mujoco.mj_name2id(
            h.model, mujoco.mjtObj.mjOBJ_BODY, body_name
        )
        return float(
            np.dot(
                h.data.xipos[bid] - h.origin,
                h.normal,
            )
        )

    q_neutral = h.q_for()
    h.measure(q_neutral)
    for joint, body in tips.items():
        neutral_normal = palm_normal_of(body)

        q_flex = h.q_for(**{FEATURE_FOR_JOINT[joint]: 1.0})
        h.measure(q_flex)
        flexed_normal = palm_normal_of(body)

        assert flexed_normal > neutral_normal, (
            f"canonical {FEATURE_FOR_JOINT[joint]}=1 (distal flex toward "
            f"palm) must move the {joint} fingertip toward the palm "
            f"interior; {neutral_normal:.4f} -> {flexed_normal:.4f}"
        )


# ----------------------------------------------------------------------
# TEST 6 — bidirectional index_adduction calibration endpoints
# ----------------------------------------------------------------------

def test_bidirectional_index_adduction_calibration():
    # Values shaped like the real AVP differential probe:
    # toward middle -> raw increases, toward thumb -> raw decreases.
    calibration = BidirectionalCalibration(
        negative=-0.33,     # toward thumb
        neutral=-0.1325,
        positive=+0.0706,   # toward middle
    )

    assert np.isclose(calibration.normalize(-0.33), -1.0)
    assert np.isclose(calibration.normalize(-0.1325), 0.0)
    assert np.isclose(calibration.normalize(+0.0706), +1.0)

    # Human semantics: toward middle is canonical POSITIVE.
    assert calibration.normalize(0.0) > 0.0
    assert calibration.normalize(-0.2) < 0.0


# ----------------------------------------------------------------------
# TEST 7 / TEST 8 — calibration quality gate
# ----------------------------------------------------------------------

BASE_RAW = {
    "thumb_opposition": 1.2,
    "thumb_abduction": 0.7,
    "thumb_distal": -0.145,
    "index_adduction": -0.13,
    "index_flexion": 0.4,
    "index_distal": 0.03,
    "middle_flexion": 0.4,
    "middle_distal": 0.01,
    "little_outward": 0.3,
    "little_flexion": 0.25,
    "little_distal": 0.05,
}

# Per-pose feature offsets for a stable, plausible full calibration.
POSE_OFFSETS = {
    "neutral": {},

    "thumb_opposition": {"thumb_opposition": -1.0},
    "thumb_abduction": {"thumb_abduction": 0.55},

    "thumb_distal_negative": {"thumb_distal": -0.069},
    "thumb_distal_positive": {"thumb_distal": +0.9},

    "index_adduction_negative": {"index_adduction": -0.20},
    "index_adduction_positive": {"index_adduction": +0.20},

    "finger_flexion": {
        "index_flexion": 0.41,
        "middle_flexion": 0.43,
        "little_flexion": 0.09,
    },

    "finger_distal_flexion": {
        "index_distal": 0.35,
        "middle_distal": 0.50,
        "little_distal": 0.27,
    },

    "little_outward": {"little_outward": 0.43},
}


def _populate(session, jitter, rng, pose_overrides=None):
    """Add jittered samples for every required pose."""

    overrides = pose_overrides or {}

    for pose, offsets in POSE_OFFSETS.items():
        offsets = dict(offsets)
        offsets.update(overrides.get(pose, {}))

        for _ in range(90):
            values = dict(BASE_RAW)
            for key, delta in offsets.items():
                values[key] += delta
            for key in values:
                values[key] += rng.normal(0.0, jitter)
            session.add_sample(pose, AVPRawHandFeatures(**values))


def test_quality_gate_rejects_tiny_span():
    """thumb_distal negative span ~= 0.069 rad with matching-scale noise."""
    rng = np.random.default_rng(7)
    session = CalibrationSession()
    _populate(session, jitter=0.03, rng=rng)

    diagnostics = session.bidirectional_diagnostics()
    item = diagnostics["thumb_distal"]

    assert not item.passed, (
        "span (0.069 rad) comparable to noise scale must be rejected"
    )
    assert "negative_span_below_noise_scale" in item.failure_reasons

    try:
        session.build()
    except ValueError as exc:
        message = str(exc)
        assert "thumb_distal" in message
        assert "span" in message or "sigma" in message
    else:
        raise AssertionError(
            "Unstable bidirectional endpoint must not build a calibration"
        )


def test_quality_gate_rejects_index_distal_style_instability():
    """Same generic mechanism must catch index_distal-style endpoints."""
    rng = np.random.default_rng(11)
    session = CalibrationSession()

    # thumb_distal negative span ~0.079 rad with 0.05 rad jitter:
    # the same failure mode the old index_distal negative endpoint had
    # (span comparable to noise).
    _populate(
        session,
        jitter=0.05,
        rng=rng,
        pose_overrides={
            "thumb_distal_negative": {"thumb_distal": -0.079},
        },
    )

    item = session.bidirectional_diagnostics()["thumb_distal"]
    assert not item.passed
    assert "negative_span_below_noise_scale" in item.failure_reasons


def test_quality_gate_accepts_stable_endpoint():
    rng = np.random.default_rng(3)
    session = CalibrationSession()
    _populate(session, jitter=0.005, rng=rng)

    diagnostics = session.bidirectional_diagnostics()
    for name, item in diagnostics.items():
        assert item.passed, (
            f"{name}: large, stable spans must pass; "
            f"reasons={item.failure_reasons}"
        )

    # The gate constant must stay conservative, not silently loosened.
    assert MIN_SPAN_TO_NOISE_RATIO >= 5.0


# ----------------------------------------------------------------------
# TEST 9 — legacy calibration detection
# ----------------------------------------------------------------------

def test_legacy_calibration_detection():
    legacy = {
        "schema_version": 1,
        "metadata": {},
        "unidirectional": {
            "index_adduction": {"neutral": -0.174, "active": -0.0255},
        },
        "bidirectional": {
            "thumb_distal": {
                "negative": -0.214,
                "neutral": -0.145,
                "positive": 0.916,
            },
        },
    }

    try:
        calibration_from_dict(legacy)
    except ValueError as exc:
        message = str(exc)
        assert "index_adduction" in message
        assert "legacy" in message.lower()
        assert "rerun" in message.lower() or "重新" in message
    else:
        raise AssertionError(
            "Legacy unidirectional index_adduction must be rejected with "
            "an actionable message, not silently reinterpreted"
        )


# ----------------------------------------------------------------------
# Integration — canonical -> retarget -> MuJoCo FK
# ----------------------------------------------------------------------

def test_retarget_fk_integration():
    h = PalmFrameHarness()

    cases = {
        "neutral": h.q_for(),
        "index_toward_middle": h.q_for(index_adduction=+1.0),
        "index_toward_thumb": h.q_for(index_adduction=-1.0),
        "thumb_distal_flex": h.q_for(thumb_distal=+1.0),
    }

    for label, q in cases.items():
        assert q.shape == (11,)
        assert np.all(np.isfinite(q)), label

        for i, joint in enumerate(h.retargeter.joint_order):
            lower, upper = map(
                float, h.retargeter.joints[joint]["range"]
            )
            assert lower - 1e-9 <= q[i] <= upper + 1e-9, (
                f"{label}: {joint} command {q[i]} outside joint range"
            )

    m = {label: h.measure(q) for label, q in cases.items()}

    assert m["index_toward_middle"]["dist_middle"] <= m["neutral"]["dist_middle"] + 1e-12
    assert m["index_toward_thumb"]["dist_thumb"] < m["neutral"]["dist_thumb"]
    assert m["index_toward_thumb"]["dist_middle"] > m["neutral"]["dist_middle"]
    assert m["thumb_distal_flex"]["thumb_palm"] < m["neutral"]["thumb_palm"]


def main():
    results = []

    def run(name, fn):
        try:
            fn()
        except AssertionError as exc:
            results.append((name, "FAIL", exc))
            print(f"[FAIL] {name}: {exc}")
        else:
            results.append((name, "PASS", None))
            print(f"[PASS] {name}")

    run("index_adduction_positive_moves_toward_middle",
        test_index_adduction_positive_moves_toward_middle)
    run("index_adduction_negative_moves_toward_thumb",
        test_index_adduction_negative_moves_toward_thumb)
    run("index_adduction_neutral_uses_semantics_zero",
        test_index_adduction_neutral_uses_semantics_zero)
    run("thumb_neutral_uses_semantics_zero",
        test_thumb_neutral_uses_semantics_zero)
    run("thumb_distal_positive_flexes_toward_palm",
        test_thumb_distal_positive_flexes_toward_palm)
    run("finger_distal_straight_maps_to_robot_zero",
        test_finger_distal_straight_maps_to_robot_zero)
    run("finger_distal_negative_input_never_hits_mechanical_limit",
        test_finger_distal_negative_input_never_hits_mechanical_limit)
    run("finger_distal_flexion_moves_toward_palm",
        test_finger_distal_flexion_moves_toward_palm)
    run("bidirectional_index_adduction_calibration",
        test_bidirectional_index_adduction_calibration)
    run("quality_gate_rejects_tiny_span",
        test_quality_gate_rejects_tiny_span)
    run("quality_gate_rejects_index_distal_style_instability",
        test_quality_gate_rejects_index_distal_style_instability)
    run("quality_gate_accepts_stable_endpoint",
        test_quality_gate_accepts_stable_endpoint)
    run("legacy_calibration_detection",
        test_legacy_calibration_detection)
    run("retarget_fk_integration",
        test_retarget_fk_integration)

    failed = [r for r in results if r[1] == "FAIL"]
    print()
    if failed:
        print(f"SEMANTIC REGRESSION: {len(failed)} FAILED")
        raise SystemExit(1)
    print("SEMANTIC REGRESSION: ALL PASS")


if __name__ == "__main__":
    main()
