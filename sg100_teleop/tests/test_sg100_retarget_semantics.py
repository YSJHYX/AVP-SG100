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
# Thumb root gain — teleop semantic range contract
#
# Canonical [0, 1] maps onto the TELEOP semantic range from the SG100
# semantics config (a subset of the mechanical range), NOT onto the full
# mechanical range.  A small human root motion must not consume the
# robot's entire mechanical ROM.
# ----------------------------------------------------------------------

def test_thumb_neutral_maps_to_robot_neutral():
    h = PalmFrameHarness()
    q = h.q_for()
    i1 = h.retargeter.joint_order.index("thumb_j1")
    i2 = h.retargeter.joint_order.index("thumb_j2")
    assert np.isclose(q[i1], h.zero["thumb_j1"])
    assert np.isclose(q[i2], h.zero["thumb_j2"])


def test_thumb_opposition_small_input_produces_bounded_root_motion():
    h = PalmFrameHarness()
    r = h.retargeter
    i1 = r.joint_order.index("thumb_j1")

    t_lower, t_upper = r.joint_teleop_range["thumb_j1"]
    m_lower, m_upper = map(float, r.joints["thumb_j1"]["range"])
    gamma = r.joint_gamma["thumb_j1"]

    # The teleop semantic limit must be a strict subset of mechanical ROM.
    assert m_lower <= t_lower < t_upper <= m_upper

    # Nonlinear response curve (pinch audit outcome):
    #   c_eff = c ** gamma,  q = zero + c_eff * (teleop_max - zero)
    # Keeps the 90 deg pinch reach while softening the low/mid gain that
    # over-folded the MCP during normal grasp.
    for canonical, expected_deg in (
        (0.0, 0.0),
        (0.1, 2.85),
        (0.2, 8.05),
        (0.3, 14.79),
        (0.5, 31.82),
        (0.7, 52.71),
        (0.9126, 78.5),   # real probe COMFORTABLE_OPPOSITION ~ FK pinch optimum 79 deg
        (1.0, 90.0),
    ):
        q = h.q_for(thumb_opposition=canonical)
        assert np.isclose(np.degrees(q[i1]), expected_deg, atol=0.05), (
            f"canonical thumb_opposition={canonical} must command "
            f"{expected_deg} deg, got {np.degrees(q[i1]):.2f} deg"
        )
        expected = h.zero["thumb_j1"] + (
            canonical ** gamma
        ) * (t_upper - h.zero["thumb_j1"])
        assert np.isclose(q[i1], expected)

    # Full-scale reach still equals the teleop span.
    gain = abs(
        h.q_for(thumb_opposition=1.0)[i1]
        - h.q_for(thumb_opposition=0.0)[i1]
    )
    assert np.isclose(gain, t_upper - t_lower)

    # The mechanical clamp on thumb_j1 itself remains the original
    # 0-125.02 deg ROM (teleop limits only restrict interpolation).
    assert np.isclose(np.degrees(m_upper), 125.02, atol=0.01)
    assert np.isclose(np.degrees(m_lower), 0.0)


def test_gamma_response_defaults_and_equivalence():
    """gamma=1.0 must reproduce the old linear behaviour exactly."""
    import tempfile

    h = PalmFrameHarness()
    r = h.retargeter

    # Only thumb_j1 carries an explicit gamma; everything else defaults.
    assert np.isclose(r.joint_gamma["thumb_j1"], 1.5)
    for joint in r.joint_order:
        if joint != "thumb_j1":
            assert np.isclose(r.joint_gamma[joint], 1.0), joint

    # A config without any gamma must be bit-for-bit the linear mapping.
    with tempfile.TemporaryDirectory() as tmp:
        linear_path = Path(tmp) / "linear_semantics.yaml"
        with SEMANTICS_YAML.open("r") as f:
            config = yaml.safe_load(f)
        config["joints"]["thumb_j1"].pop("gamma", None)
        with linear_path.open("w") as f:
            yaml.safe_dump(config, f)

        linear = SG100Retargeter(semantics_path=linear_path)
        assert all(np.isclose(g, 1.0) for g in linear.joint_gamma.values())

        for canonical in (0.0, 0.2, 0.5, 0.9126, 1.0):
            q_lin = linear.retarget(
                CanonicalHandState(thumb_opposition=canonical)
            )
            expected = np.deg2rad(canonical * 90.0)
            assert np.isclose(q_lin[0], expected, atol=1e-12), (
                f"gamma=1.0 must equal the linear mapping at {canonical}"
            )


def test_gamma_leaves_other_joints_unchanged():
    """thumb_j2/j3 and index/middle/little outputs are numerically
    identical to the gamma-free linear formula."""
    h = PalmFrameHarness()
    r = h.retargeter

    sweep = (
        CanonicalHandState(),
        CanonicalHandState(
            thumb_opposition=0.7, thumb_abduction=0.6, thumb_distal=0.5,
            index_adduction=0.3, index_flexion=0.8, index_distal=0.4,
            middle_flexion=0.7, middle_distal=0.3,
            little_outward=0.5, little_flexion=0.9, little_distal=0.2,
        ),
        CanonicalHandState(thumb_opposition=-0.2, index_adduction=-0.5),
        CanonicalHandState(
            thumb_distal=-1.0, index_distal=1.0, little_outward=1.0,
        ),
    )

    for hand in sweep:
        q = r.retarget(hand)
        for joint in r.joint_order:
            if joint == "thumb_j1":
                continue
            i = r.joint_order.index(joint)
            feature, mode = r.FEATURE_MAPPING[joint]
            value = hand.clipped().as_dict()[feature]
            lower, upper = r.joint_teleop_range[joint]
            zero = r.joint_zero[joint]

            if mode == "positive":
                expected = zero + max(0.0, min(1.0, value)) * (upper - zero)
            elif mode == "negative":
                expected = zero + max(0.0, min(1.0, value)) * (lower - zero)
            elif mode == "bidirectional":
                c = max(-1.0, min(1.0, value))
                expected = (
                    zero + c * (upper - zero)
                    if c >= 0 else
                    zero + (-c) * (lower - zero)
                )
            else:  # bidirectional_inverted
                c = max(-1.0, min(1.0, value))
                expected = (
                    zero + c * (lower - zero)
                    if c >= 0 else
                    zero + (-c) * (upper - zero)
                )

            m_lower, m_upper = map(float, r.joints[joint]["range"])
            expected = min(max(expected, m_lower), m_upper)
            assert np.isclose(q[i], expected, atol=1e-12), (
                f"{joint} output changed under gamma config: {q[i]} vs "
                f"{expected}"
            )


def test_thumb_abduction_small_input_produces_bounded_root_motion():
    h = PalmFrameHarness()
    r = h.retargeter
    i2 = r.joint_order.index("thumb_j2")

    t_lower, t_upper = r.joint_teleop_range["thumb_j2"]
    m_lower, m_upper = map(float, r.joints["thumb_j2"]["range"])

    assert m_lower <= t_lower < t_upper <= m_upper

    for canonical in (0.2, 0.5, 1.0):
        q = h.q_for(thumb_abduction=canonical)
        expected = h.zero["thumb_j2"] + canonical * (
            t_lower - h.zero["thumb_j2"]
        )
        assert np.isclose(q[i2], expected), (
            f"canonical thumb_abduction={canonical} must command "
            f"{np.degrees(expected):.1f} deg (teleop), got "
            f"{np.degrees(q[i2]):.1f} deg"
        )

    gain = abs(
        h.q_for(thumb_abduction=1.0)[i2]
        - h.q_for(thumb_abduction=0.0)[i2]
    )
    assert np.isclose(gain, t_upper - t_lower)


def test_thumb_root_motion_does_not_modify_thumb_j3_mapping():
    h = PalmFrameHarness()
    r = h.retargeter
    i3 = r.joint_order.index("thumb_j3")

    for canonical in (0.0, 0.2, 0.5, 1.0):
        q_opp = h.q_for(thumb_opposition=canonical)
        q_abd = h.q_for(thumb_abduction=canonical)
        assert np.isclose(q_opp[i3], h.zero["thumb_j3"]), (
            "thumb_opposition input must not move thumb_j3"
        )
        assert np.isclose(q_abd[i3], h.zero["thumb_j3"]), (
            "thumb_abduction input must not move thumb_j3"
        )


# ----------------------------------------------------------------------
# Pinch-conditioned thumb coordination (gated blend)
#
# gate = smoothstep(score), score = opp * abd * (1 - clamp(dst, 0, 1))
# q1 = (1-g)*q1_base(gamma curve)  + g*q1_pinch(monotonic anchors)
# q2 = (1-g)*q2_base               + g*q2_pinch (configured 0 deg)
# gate == 0 must reproduce the base branch EXACTLY.
# ----------------------------------------------------------------------

# Real AVP operating points, INCLUDING the non-thumb finger flexions
# the grasp-context pinch gate depends on (thumb_distal is no longer a
# gate input).
MEASURED_POINTS = {
    "NEUTRAL": dict(thumb_opposition=0.0000, thumb_abduction=0.0000,
                    index_flexion=0.0, middle_flexion=0.0,
                    little_flexion=0.0),
    "NORMAL_GRASP": dict(thumb_opposition=0.7595, thumb_abduction=0.8610,
                         index_flexion=0.9795, middle_flexion=1.0000,
                         little_flexion=1.0000),
    "INDEX_PINCH": dict(thumb_opposition=0.5724, thumb_abduction=1.0000,
                        index_flexion=0.3892, middle_flexion=0.0000,
                        little_flexion=0.0000),
    "MIDDLE_PINCH": dict(thumb_opposition=0.7420, thumb_abduction=1.0000,
                         index_flexion=0.0000, middle_flexion=0.1568,
                         little_flexion=0.0000),
    "LITTLE_PINCH": dict(thumb_opposition=0.9264, thumb_abduction=0.9190,
                         index_flexion=0.0000, middle_flexion=0.0000,
                         little_flexion=0.1596),
}


def _thumb_q(h, point):
    return h.q_for(**point)


def test_pinch_gate_preserves_neutral_and_grasp():
    """(A) NORMAL GRASP — global context high, signal ~0, gate == 0,
    q1/q2 exactly equal to the old grasp branch."""
    h = PalmFrameHarness()
    r = h.retargeter
    i1 = r.joint_order.index("thumb_j1")
    i2 = r.joint_order.index("thumb_j2")

    g_neutral = r.pinch_gate(MEASURED_POINTS["NEUTRAL"])
    assert g_neutral == 0.0
    q = _thumb_q(h, MEASURED_POINTS["NEUTRAL"])
    assert np.isclose(q[i1], 0.0) and np.isclose(q[i2], 0.0)

    grasp = MEASURED_POINTS["NORMAL_GRASP"]
    ctx = r.global_grasp_context(grasp)
    sig = r.pinch_signal(grasp)
    g = r.pinch_gate(grasp)

    assert ctx > 0.97, f"grasp context {ctx:.4f}"
    assert sig < 0.02, f"grasp pinch_signal {sig:.4f}"
    assert g == 0.0 or g < 1e-12

    # Exact base-branch values: 90 * 0.7595^1.5 and -60 * 0.861.
    q = _thumb_q(h, grasp)
    assert np.isclose(np.degrees(q[i1]), 59.58, atol=0.05)
    assert np.isclose(np.degrees(q[i2]), -51.66, atol=0.05)


def test_pinch_gate_activates_on_pinch_points():
    """(B/C/D) INDEX / MIDDLE / LITTLE pinches — context ~0, signal high,
    gate ~1."""
    h = PalmFrameHarness()
    r = h.retargeter
    i1 = r.joint_order.index("thumb_j1")
    i2 = r.joint_order.index("thumb_j2")

    for name in ("INDEX_PINCH", "MIDDLE_PINCH", "LITTLE_PINCH"):
        point = MEASURED_POINTS[name]
        assert r.global_grasp_context(point) < 0.01, name
        expected_sig = (
            point["thumb_opposition"]
            * point["thumb_abduction"]
        )
        assert np.isclose(r.pinch_signal(point), expected_sig,
                          atol=1e-9), name
        g = r.pinch_gate(point)
        assert g >= 0.99, f"{name} gate too low: {g:.4f}"

        q = _thumb_q(h, point)
        q1_base = np.deg2rad(90.0) * point["thumb_opposition"] ** 1.5
        anchors = r.pinch_coordination["joints"]["thumb_j1"]["anchors"]
        q1_pinch = float(np.interp(
            point["thumb_opposition"],
            [a[0] for a in anchors],
            [a[1] for a in anchors],
        ))
        expected_q1 = (1.0 - g) * q1_base + g * q1_pinch
        assert np.isclose(q[i1], expected_q1, atol=1e-12), name

        # q2 blends toward the configured pinch target (0 deg).
        q2_base = np.deg2rad(-60.0) * point["thumb_abduction"]
        expected_q2 = (1.0 - g) * q2_base
        assert np.isclose(q[i2], expected_q2, atol=1e-12), name

    # Middle pinch: gate saturates -> q1 lands exactly on the pinch
    # anchor (frozen anchors: interp at opp=0.742 -> 100.7 deg) and
    # q2 exactly on the configured pinch target 0.
    q = _thumb_q(h, MEASURED_POINTS["MIDDLE_PINCH"])
    anchors = r.pinch_coordination["joints"]["thumb_j1"]["anchors"]
    expected_middle = float(np.interp(
        0.7420,
        [a[0] for a in anchors],
        [a[1] for a in anchors],
    ))
    assert np.isclose(q[i1], expected_middle, atol=1e-12)
    assert np.isclose(np.degrees(q[i1]), 100.7, atol=0.1)
    assert np.isclose(q[i2], 0.0, atol=1e-12)
    assert np.degrees(q[i1]) > 79.0  # exceeds index despite larger reach need

    # Index pinch: q1 must exceed the grasp value even though the
    # opposition canonical (0.5724) is LOWER than grasp (0.7595).
    q_idx = _thumb_q(h, MEASURED_POINTS["INDEX_PINCH"])
    q_grasp = _thumb_q(h, MEASURED_POINTS["NORMAL_GRASP"])
    assert np.degrees(q_idx[i1]) > np.degrees(q_grasp[i1]) + 10.0


def test_pinch_gate_continuity():
    """(E) gate must be monotonic, continuous, in [0, 1] across the
    smoothstep transition (sweep pinch_signal 0.15 -> 0.55 via
    opposition with abduction=1 and zero grasp context)."""
    h = PalmFrameHarness()
    r = h.retargeter

    prev_g, prev_q = None, None
    for opp in (0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55):
        point = dict(thumb_opposition=opp, thumb_abduction=1.0,
                     index_flexion=0.0, middle_flexion=0.0,
                     little_flexion=0.0)
        g = r.pinch_gate(point)
        assert 0.0 <= g <= 1.0
        if prev_g is not None:
            assert g >= prev_g, "gate must be monotonic"
        a = _thumb_q(h, point)
        b = _thumb_q(h, dict(point, thumb_opposition=opp + 1e-9))
        assert np.allclose(a, b, atol=1e-6), (
            f"discontinuity at pinch_signal~{opp:.2f}"
        )
        if prev_q is not None:
            step = float(np.abs(a[:2] - prev_q[:2]).max())
            # Bound derived from the max gate slope: |dq1/dsignal| <=
            # (anchor span ~1.4 rad) * smoothstep'(1.5) / (0.30 band)
            # ~ 7 rad/unit -> per 0.05 sample <= 0.36 rad.
            assert step < 0.40, f"q jump at signal~{opp:.2f}: {step}"
        prev_g, prev_q = g, a

    # True continuity: epsilon-pair check across the whole transition.
    for opp in np.arange(0.10, 0.65, 0.01):
        point = dict(thumb_opposition=float(opp), thumb_abduction=1.0,
                     index_flexion=0.0, middle_flexion=0.0,
                     little_flexion=0.0)
        a = _thumb_q(h, point)
        b = _thumb_q(h, dict(point, thumb_opposition=float(opp) + 1e-7))
        assert np.allclose(a, b, atol=1e-5), (
            f"discontinuity at pinch_signal~{opp:.2f}"
        )


def test_pinch_coordination_backward_compatibility():
    """Without the pinch_coordination config, every joint output must
    match the previous (single power curve) implementation exactly —
    including at pinch-like points, where no gating may occur."""
    import copy
    import tempfile

    with SEMANTICS_YAML.open("r") as f:
        config = yaml.safe_load(f)
    assert "pinch_coordination" in config

    stripped = copy.deepcopy(config)
    stripped.pop("pinch_coordination")
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "no_coord.yaml"
        with path.open("w") as f:
            yaml.safe_dump(stripped, f)
        old = SG100Retargeter(semantics_path=path)

    new = SG100Retargeter()

    # Gate==0 operating points (neutral, grasp): bitwise identical.
    for name in ("NEUTRAL", "NORMAL_GRASP"):
        hand = CanonicalHandState(**MEASURED_POINTS[name])
        assert np.array_equal(new.retarget(hand), old.retarget(hand)), name

    # Stripped config must ignore pinch-like geometry entirely.
    for name in ("INDEX_PINCH", "MIDDLE_PINCH", "LITTLE_PINCH"):
        hand = CanonicalHandState(**MEASURED_POINTS[name])
        q = old.retarget(hand)
        expected = np.deg2rad(
            90.0 * MEASURED_POINTS[name]["thumb_opposition"] ** 1.5
        )
        assert np.isclose(q[0], expected, atol=1e-12), name


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
    run("thumb_neutral_maps_to_robot_neutral",
        test_thumb_neutral_maps_to_robot_neutral)
    run("thumb_opposition_small_input_produces_bounded_root_motion",
        test_thumb_opposition_small_input_produces_bounded_root_motion)
    run("thumb_abduction_small_input_produces_bounded_root_motion",
        test_thumb_abduction_small_input_produces_bounded_root_motion)
    run("thumb_root_motion_does_not_modify_thumb_j3_mapping",
        test_thumb_root_motion_does_not_modify_thumb_j3_mapping)
    run("gamma_response_defaults_and_equivalence",
        test_gamma_response_defaults_and_equivalence)
    run("gamma_leaves_other_joints_unchanged",
        test_gamma_leaves_other_joints_unchanged)
    run("pinch_gate_preserves_neutral_and_grasp",
        test_pinch_gate_preserves_neutral_and_grasp)
    run("pinch_gate_activates_on_pinch_points",
        test_pinch_gate_activates_on_pinch_points)
    run("pinch_gate_continuity",
        test_pinch_gate_continuity)
    run("pinch_coordination_backward_compatibility",
        test_pinch_coordination_backward_compatibility)
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
