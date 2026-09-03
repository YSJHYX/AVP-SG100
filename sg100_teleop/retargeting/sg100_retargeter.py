from pathlib import Path

import numpy as np
import yaml

from sg100_teleop.hand.canonical_hand import CanonicalHandState


ROOT = Path(__file__).resolve().parents[2]

DEFAULT_SEMANTICS_PATH = (
    ROOT
    / "sg100_teleop"
    / "config"
    / "sg100_joint_semantics.yaml"
)


class SG100Retargeter:
    """
    Map CanonicalHandState to SG100 absolute joint-position targets.

    Output convention:
        q_target.shape == (11,)
        unit == radians
        representation == absolute_joint_position

    Two distinct "neutral" concepts (do not conflate them):

        HUMAN FEATURE NEUTRAL -> canonical value 0
        ROBOT JOINT NEUTRAL   -> q = semantics ``zero`` per joint

    Canonical 0 for a feature always commands q == semantics zero for the
    mapped joint.  The zero comes from the SG100 semantics config (a robot
    morphology property), never from the per-user calibration YAML.

    Two distinct "range" concepts (do not conflate them):

        mechanical range -> [range] per joint (the robot's physical ROM)
        teleop range     -> optional [teleop_range] per joint (the
                            semantic subset used for teleoperation)

    Mapping modes (all neutral-centered on semantics zero, interpolating
    onto the TELEOP limits when configured, the mechanical limits
    otherwise; output is always clipped to the mechanical range):

        positive:
            canonical [0, 1] -> q from zero toward the upper limit

        negative:
            canonical [0, 1] -> q from zero toward the lower limit

        bidirectional:
            canonical [-1, 1] -> q from zero toward upper (+) / lower (-)

        bidirectional_inverted:
            canonical [-1, 1] -> q from zero toward lower (+) / upper (-)

            Used when the robot's numeric q+ is the SEMANTIC negative
            direction: canonical +1 commands the numeric q_min side.
            Example: index_j1 — canonical index_adduction +1 means
            "toward middle" while the robot's numeric q+ is physically
            toward thumb.

    Two optional config-driven coordination layers may post-process
    the base mapping (both reproduce the base output exactly when
    their config section is absent or disabled):

        pinch_coordination      — pinch-gated blend toward pinch
                                  anchors/targets (see
                                  _load_coordination)
        power_grasp_thumb_lock  — grasp-gated ADDITIVE bias that
                                  completes thumb opposition for
                                  power grasps (see
                                  _load_power_grasp_lock)
    """

    FEATURE_MAPPING = {
        "thumb_j1": ("thumb_opposition", "positive"),
        "thumb_j2": ("thumb_abduction", "negative"),
        "thumb_j3": ("thumb_distal", "bidirectional"),

        "index_j1": ("index_adduction", "bidirectional_inverted"),
        "index_j2": ("index_flexion", "positive"),
        # Finger distal flexion is unidirectional (human DIP joints have
        # no reverse motion): canonical 0 = straight baseline -> robot
        # zero; canonical 1 = distal flexed toward palm -> flex limit.
        "index_j3": ("index_distal", "positive"),

        "middle_j1": ("middle_flexion", "negative"),
        "middle_j2": ("middle_distal", "positive"),

        "little_j1": ("little_outward", "negative"),
        "little_j2": ("little_flexion", "positive"),
        "little_j3": ("little_distal", "positive"),
    }

    def __init__(
        self,
        semantics_path: Path | str = DEFAULT_SEMANTICS_PATH,
    ):
        semantics_path = Path(semantics_path)

        with semantics_path.open("r") as f:
            config = yaml.safe_load(f)

        self.joint_order = list(config["joint_order"])
        self.joints = config["joints"]

        if len(self.joint_order) != 11:
            raise RuntimeError(
                f"Expected 11 SG100 joints, "
                f"got {len(self.joint_order)}"
            )

        missing = set(self.joint_order) - set(self.FEATURE_MAPPING)

        if missing:
            raise RuntimeError(
                f"Missing retarget mapping for joints: {missing}"
            )

        # Optional pinch-conditioned coordination (generic, fully
        # config-driven; absent section = previous behaviour).
        self.pinch_coordination = self._load_coordination(config)

        # Optional grasp-conditioned thumb lock bias (config-driven;
        # absent section or enabled: false = previous behaviour).
        self.power_grasp_thumb_lock = (
            self._load_power_grasp_lock(config)
        )

        # Robot joint neutral offsets (q at canonical 0), validated
        # against each joint's mechanical range.
        self.joint_zero = {}

        # Teleop semantic ranges: canonical full scale maps onto these
        # subsets of the mechanical ROM when configured.  Joints without
        # an explicit teleop_range keep the full mechanical range.
        self.joint_teleop_range = {}

        # Nonlinear response exponents: canonical magnitude is raised to
        # gamma before interpolation (q = zero + sign(c)*|c|**gamma *
        # limit_span).  Default gamma 1.0 reproduces the linear mapping
        # exactly, so joints without an explicit gamma are unaffected.
        self.joint_gamma = {}

        for joint_name in self.joint_order:
            lower, upper = map(
                float,
                self.joints[joint_name]["range"],
            )

            zero = float(
                self.joints[joint_name].get("zero", 0.0)
            )

            if not (lower <= zero <= upper):
                raise RuntimeError(
                    f"Semantics zero for {joint_name} ({zero}) is "
                    f"outside its range [{lower}, {upper}]"
                )

            self.joint_zero[joint_name] = zero

            gamma = float(
                self.joints[joint_name].get("gamma", 1.0)
            )

            if not (gamma > 0.0):
                raise RuntimeError(
                    f"Response gamma for {joint_name} ({gamma}) must "
                    f"be positive"
                )

            self.joint_gamma[joint_name] = gamma

            teleop = self.joints[joint_name].get("teleop_range")

            if teleop is None:
                t_lower, t_upper = lower, upper
            else:
                t_lower, t_upper = map(float, teleop)

            if not (
                lower <= t_lower <= t_upper <= upper
            ):
                raise RuntimeError(
                    f"Teleop range for {joint_name} "
                    f"[{t_lower}, {t_upper}] is not a valid subset of "
                    f"its mechanical range [{lower}, {upper}]"
                )

            if not (t_lower <= zero <= t_upper):
                raise RuntimeError(
                    f"Semantics zero for {joint_name} ({zero}) is "
                    f"outside its teleop range "
                    f"[{t_lower}, {t_upper}]"
                )

            self.joint_teleop_range[joint_name] = (t_lower, t_upper)

    _CANONICAL_FEATURE_NAMES = frozenset({
        "thumb_opposition", "thumb_abduction", "thumb_distal",
        "index_adduction", "index_flexion", "index_distal",
        "middle_flexion", "middle_distal",
        "little_outward", "little_flexion", "little_distal",
    })

    def _load_coordination(self, config: dict):
        """Validate the optional pinch_coordination section."""

        section = config.get("pinch_coordination")

        if section is None:
            return None

        gate_cfg = section["gate"]
        pinch_terms = gate_cfg["pinch_terms"]
        context_features = list(gate_cfg["context_features"])
        context_combiner = gate_cfg["context_combiner"]
        signal_low = float(gate_cfg["signal_low"])
        signal_high = float(gate_cfg["signal_high"])

        if not (0.0 <= signal_low < signal_high <= 1.0):
            raise RuntimeError(
                "pinch_coordination gate thresholds must satisfy "
                f"0 <= signal_low < signal_high <= 1, got "
                f"[{signal_low}, {signal_high}]"
            )

        for feature in pinch_terms:
            if feature not in self._CANONICAL_FEATURE_NAMES:
                raise RuntimeError(
                    f"pinch_coordination references unknown feature "
                    f"{feature!r}"
                )
            if pinch_terms[feature] not in ("direct", "inverted"):
                raise RuntimeError(
                    f"pinch term {feature!r} must be 'direct' or "
                    f"'inverted', got {pinch_terms[feature]!r}"
                )

        for feature in context_features:
            if feature not in self._CANONICAL_FEATURE_NAMES:
                raise RuntimeError(
                    f"pinch_coordination references unknown context "
                    f"feature {feature!r}"
                )

        if context_combiner != "geometric_mean":
            raise RuntimeError(
                "pinch_coordination context_combiner must be "
                f"'geometric_mean', got {context_combiner!r}"
            )

        joints_cfg = {}

        for joint_name, joint_cfg in section["joints"].items():
            if joint_name not in self.joints:
                raise RuntimeError(
                    f"pinch_coordination references unknown joint "
                    f"{joint_name!r}"
                )

            m_lower, m_upper = map(
                float, self.joints[joint_name]["range"]
            )

            parsed = {}

            if "anchors" in joint_cfg:
                # Anchor q values are configured in degrees.
                anchors = [
                    (float(c), np.deg2rad(float(q)))
                    for c, q in joint_cfg["anchors"]
                ]
                anchors.sort(key=lambda item: item[0])

                cs = [c for c, _ in anchors]
                qs = [q for _, q in anchors]

                if any(
                    later <= earlier
                    for earlier, later in zip(cs, cs[1:])
                ):
                    raise RuntimeError(
                        f"pinch_coordination anchors for {joint_name} "
                        "must have strictly increasing canonical values"
                    )
                if any(
                    later < earlier
                    for earlier, later in zip(qs, qs[1:])
                ):
                    raise RuntimeError(
                        f"pinch_coordination anchors for {joint_name} "
                        "must be monotonic non-decreasing in q"
                    )
                if any(
                    not (m_lower <= q <= m_upper) for _, q in anchors
                ):
                    raise RuntimeError(
                        f"pinch_coordination anchors for {joint_name} "
                        f"exceed the mechanical range [{m_lower}, {m_upper}]"
                    )
                parsed["anchors"] = anchors

            if "pinch_target_deg" in joint_cfg:
                target = float(
                    np.deg2rad(joint_cfg["pinch_target_deg"])
                )
                if not (m_lower <= target <= m_upper):
                    raise RuntimeError(
                        f"pinch_coordination target for {joint_name} "
                        f"({target}) is outside the mechanical range"
                    )
                parsed["pinch_target_deg"] = target

            joints_cfg[joint_name] = parsed

        return {
            "pinch_terms": pinch_terms,
            "context_features": context_features,
            "signal_low": signal_low,
            "signal_high": signal_high,
            "joints": joints_cfg,
        }

    def _load_power_grasp_lock(self, config: dict):
        """Validate the optional power_grasp_thumb_lock section.

        Schema:

            power_grasp_thumb_lock:
              enabled: true
              grasp_score:
                features: [index_flexion, middle_flexion, little_flexion]
                start: 0.60
                full: 0.85
              thumb_engagement:
                feature: thumb_opposition
                low: 0.35
                high: 0.60
              max_weight: 1.0
              biases_deg: {thumb_j1: 40.0, thumb_j2: 0.0, thumb_j3: 0.0}

        Behaviour contract:

        - absent section or ``enabled: false`` -> retarget() output is
          bit-identical to the previous implementation;
        - weight is a CONTINUOUS blend (no fixed lock pose): each bias
          is added proportionally to
          w = max_weight * grasp_w * engagement_w * (1 - pinch_gate);
        - the result is clipped to each joint's mechanical range.
        """

        section = config.get("power_grasp_thumb_lock")

        if section is None:
            return None

        if not section.get("enabled", True):
            return None

        grasp_cfg = section["grasp_score"]
        grasp_features = list(grasp_cfg["features"])
        grasp_start = float(grasp_cfg["start"])
        grasp_full = float(grasp_cfg["full"])

        if not (0.0 <= grasp_start < grasp_full <= 1.0):
            raise RuntimeError(
                "power_grasp_thumb_lock grasp_score thresholds must "
                f"satisfy 0 <= start < full <= 1, got "
                f"[{grasp_start}, {grasp_full}]"
            )

        if grasp_cfg.get("combiner", "geometric_mean") != (
            "geometric_mean"
        ):
            raise RuntimeError(
                "power_grasp_thumb_lock grasp_score combiner must be "
                f"'geometric_mean', got "
                f"{grasp_cfg.get('combiner')!r}"
            )

        for feature in grasp_features:
            if feature not in self._CANONICAL_FEATURE_NAMES:
                raise RuntimeError(
                    f"power_grasp_thumb_lock references unknown "
                    f"grasp feature {feature!r}"
                )

        engage_cfg = section["thumb_engagement"]
        engage_feature = engage_cfg["feature"]

        if engage_feature not in self._CANONICAL_FEATURE_NAMES:
            raise RuntimeError(
                "power_grasp_thumb_lock references unknown "
                f"engagement feature {engage_feature!r}"
            )

        engage_low = float(engage_cfg["low"])
        engage_high = float(engage_cfg["high"])

        if not (0.0 <= engage_low < engage_high <= 1.0):
            raise RuntimeError(
                "power_grasp_thumb_lock thumb_engagement thresholds "
                f"must satisfy 0 <= low < high <= 1, got "
                f"[{engage_low}, {engage_high}]"
            )

        max_weight = float(section["max_weight"])

        if not (0.0 < max_weight <= 1.0):
            raise RuntimeError(
                "power_grasp_thumb_lock max_weight must lie in "
                f"(0, 1], got {max_weight}"
            )

        biases = {}

        for joint_name, bias_deg in section["biases_deg"].items():
            if joint_name not in self.joints:
                raise RuntimeError(
                    f"power_grasp_thumb_lock references unknown "
                    f"joint {joint_name!r}"
                )

            m_lower, m_upper = map(
                float, self.joints[joint_name]["range"]
            )

            bias_rad = float(np.deg2rad(bias_deg))

            if not np.isfinite(bias_rad):
                raise RuntimeError(
                    f"power_grasp_thumb_lock bias for {joint_name} "
                    "must be finite"
                )

            # A bias larger than the mechanical span could push any
            # raw command to the same saturated limit and would be a
            # config smell, not a calibrated correction.
            if abs(bias_rad) > (m_upper - m_lower):
                raise RuntimeError(
                    f"power_grasp_thumb_lock bias for {joint_name} "
                    f"({bias_deg} deg) exceeds the mechanical span "
                    f"[{m_lower}, {m_upper}] rad"
                )

            biases[joint_name] = bias_rad

        if not biases:
            raise RuntimeError(
                "power_grasp_thumb_lock must configure at least one "
                "joint bias in biases_deg"
            )

        return {
            "grasp_features": grasp_features,
            "grasp_start": grasp_start,
            "grasp_full": grasp_full,
            "engage_feature": engage_feature,
            "engage_low": engage_low,
            "engage_high": engage_high,
            "max_weight": max_weight,
            "biases": biases,
        }

    @staticmethod
    def _smoothstep(x: float) -> float:
        return float(x * x * (3.0 - 2.0 * x))

    @staticmethod
    def _ramp(
        value: float,
        low: float,
        high: float,
    ) -> float:
        """Clamped linear ramp then smoothstep; 0 below ``low``, 1
        above ``high``."""

        if value <= low:
            return 0.0

        t = min(1.0, (value - low) / (high - low))

        return SG100Retargeter._smoothstep(t)

    def grasp_score(self, features: dict) -> float:
        """Geometric mean of the configured finger-closure features.

        Reuses the closure semantics already proven by the pinch gate
        context: ~1 when all fingers close together (power grasp),
        ~0 when at least two fingers stay open (pinch / open hand).
        """

        if self.power_grasp_thumb_lock is None:
            return 0.0

        cfg = self.power_grasp_thumb_lock
        product = 1.0

        for feature in cfg["grasp_features"]:
            product *= float(
                np.clip(features[feature], 0.0, 1.0)
            )

        return product ** (1.0 / len(cfg["grasp_features"]))

    def thumb_engagement(self, features: dict) -> float:
        """Smoothstep engagement of the configured human thumb feature.

        Keeps the lock OFF while the user's thumb is not actually
        opposing (deliberate open-thumb grasp) — the lock must follow
        the human, never force a fist pose.
        """

        if self.power_grasp_thumb_lock is None:
            return 0.0

        cfg = self.power_grasp_thumb_lock

        return self._ramp(
            float(np.clip(
                features[cfg["engage_feature"]],
                0.0,
                1.0,
            )),
            cfg["engage_low"],
            cfg["engage_high"],
        )

    def thumb_lock_weight(self, features: dict) -> float:
        """Continuous lock weight in [0, max_weight].

        w = max_weight
            * smoothstep(grasp_score)
            * smoothstep(thumb_engagement)
            * (1 - pinch_gate)

        The pinch-gate suppression term guarantees pinch operating
        points (gate ~ 1) keep the exact pinch-branch command and are
        never dragged toward the power-grasp lock.
        """

        if self.power_grasp_thumb_lock is None:
            return 0.0

        cfg = self.power_grasp_thumb_lock

        grasp_w = self._ramp(
            self.grasp_score(features),
            cfg["grasp_start"],
            cfg["grasp_full"],
        )

        engage_w = self.thumb_engagement(features)

        pinch = self.pinch_gate(features)

        return (
            cfg["max_weight"]
            * grasp_w
            * engage_w
            * (1.0 - pinch)
        )

    def _apply_power_grasp_lock(
        self,
        q_target: np.ndarray,
        features: dict,
    ) -> tuple[np.ndarray, dict]:
        """Add the grasp-conditioned bias blend onto ``q_target``.

        Returns the blended command (clipped to the mechanical ranges)
        plus a diagnostics dict with the exact raw/bias/final values.
        """

        cfg = self.power_grasp_thumb_lock

        if cfg is None:
            return q_target, {
                "enabled": False,
                "grasp_score": 0.0,
                "thumb_engagement": 0.0,
                "pinch_gate": self.pinch_gate(features),
                "lock_weight": 0.0,
                "raw_q": {},
                "bias_q": {},
                "final_q": {},
            }

        weight = self.thumb_lock_weight(features)

        raw_q = {}
        bias_q = {}
        final_q = {}

        for joint_name, bias_rad in cfg["biases"].items():
            i = self.joint_order.index(joint_name)

            raw = float(q_target[i])
            applied = weight * bias_rad

            m_lower, m_upper = map(
                float, self.joints[joint_name]["range"]
            )

            final = float(
                np.clip(raw + applied, m_lower, m_upper)
            )

            q_target[i] = final

            raw_q[joint_name] = raw
            bias_q[joint_name] = applied
            final_q[joint_name] = final

        return q_target, {
            "enabled": True,
            "grasp_score": self.grasp_score(features),
            "thumb_engagement": self.thumb_engagement(features),
            "pinch_gate": self.pinch_gate(features),
            "lock_weight": weight,
            "raw_q": raw_q,
            "bias_q": bias_q,
            "final_q": final_q,
        }

    def thumb_lock_diagnostics(
        self,
        q_target: np.ndarray,
        features: dict,
    ) -> dict:
        """Recompute the power-grasp lock blend for ``q_target``.

        ``q_target`` must be the PRE-lock command (base retargeting +
        pinch coordination, before the grasp lock bias) for the same
        canonical features.  The returned dict mirrors
        ``_apply_power_grasp_lock`` and is used by the
        ``--thumb-retarget-debug`` runners at low frequency.
        """

        _, info = self._apply_power_grasp_lock(
            np.array(q_target, dtype=np.float64, copy=True),
            features,
        )

        return info

    def global_grasp_context(self, features: dict) -> float:
        """Geometric mean of the clamped non-thumb finger flexions.

        ~1 when all fingers close together (normal grasp), ~0 when at
        least two fingers stay open (pinch).  Never uses distal
        features.
        """

        if self.pinch_coordination is None:
            return 0.0

        product = 1.0

        for feature in self.pinch_coordination["context_features"]:
            product *= float(np.clip(features[feature], 0.0, 1.0))

        return product ** (
            1.0 / len(self.pinch_coordination["context_features"])
        )

    def pinch_signal(self, features: dict) -> float:
        """thumb_opposition * thumb_abduction * (1 - grasp context)."""

        if self.pinch_coordination is None:
            return 0.0

        signal = 1.0

        for feature, mode in self.pinch_coordination[
            "pinch_terms"
        ].items():
            value = features[feature]
            term = (
                float(np.clip(value, 0.0, 1.0))
                if mode == "direct"
                else 1.0 - float(np.clip(value, 0.0, 1.0))
            )
            signal *= term

        return signal * (
            1.0 - self.global_grasp_context(features)
        )

    def pinch_gate(self, features: dict) -> float:
        """Smoothstep gate in [0, 1]; exactly 0 below signal_low."""

        if self.pinch_coordination is None:
            return 0.0

        signal = self.pinch_signal(features)

        low = self.pinch_coordination["signal_low"]
        high = self.pinch_coordination["signal_high"]

        if signal <= low:
            return 0.0

        t = min(1.0, (signal - low) / (high - low))

        return self._smoothstep(t)

    @staticmethod
    def _positive(value: float, zero: float, upper: float) -> float:
        """
        Canonical [0, 1] -> q from zero toward upper.
        """
        value = float(np.clip(value, 0.0, 1.0))
        return zero + value * (upper - zero)

    @staticmethod
    def _negative(value: float, zero: float, lower: float) -> float:
        """
        Canonical [0, 1] -> q from zero toward lower.
        """
        value = float(np.clip(value, 0.0, 1.0))
        return zero + value * (lower - zero)

    @staticmethod
    def _bidirectional(
        value: float,
        zero: float,
        lower: float,
        upper: float,
    ) -> float:
        """
        Canonical [-1, 1] -> q from zero toward upper (+) / lower (-).
        """
        value = float(np.clip(value, -1.0, 1.0))

        if value >= 0.0:
            return zero + value * (upper - zero)

        return zero + (-value) * (lower - zero)

    @staticmethod
    def _bidirectional_inverted(
        value: float,
        zero: float,
        lower: float,
        upper: float,
    ) -> float:
        """
        Canonical [-1, 1] -> q from zero toward lower (+) / upper (-).

        The robot's numeric q+ is the semantic NEGATIVE direction, so
        canonical + commands the numeric q_min side.  If zero equals the
        lower limit, the canonical + side saturates at zero (the robot
        cannot physically move further in that direction).
        """
        value = float(np.clip(value, -1.0, 1.0))

        if value >= 0.0:
            return zero + value * (lower - zero)

        return zero + (-value) * (upper - zero)

    def retarget(
        self,
        hand: CanonicalHandState,
    ) -> np.ndarray:
        return self.retarget_with_stages(hand)[0]

    def retarget_with_stages(
        self,
        hand: CanonicalHandState,
    ) -> tuple[np.ndarray, dict]:
        """Retarget with full command provenance.

        Returned stages (all canonical rad, mechanical-range clipped):

            raw_retarget_qcmd   base semantic mapping only
            coordinated_qcmd    after pinch coordination
            final_qcmd          after the power-grasp lock bias
                                (== retarget() return value)

        The downstream JointCommandFilter (rate/tau) and the hardware
        send add two more provenance levels that live in the runner,
        not here: final_filtered_qcmd and hardware_sent_qcmd.
        """

        hand = hand.clipped()
        features = hand.as_dict()

        q_target = np.zeros(
            len(self.joint_order),
            dtype=np.float64,
        )

        for i, joint_name in enumerate(self.joint_order):

            joint_cfg = self.joints[joint_name]

            lower, upper = map(
                float,
                joint_cfg["range"],
            )

            zero = self.joint_zero[joint_name]

            # Interpolation limits: teleop semantic range when
            # configured, mechanical range otherwise.  The final clip
            # below always enforces the mechanical range.
            t_lower, t_upper = self.joint_teleop_range[joint_name]

            feature_name, mapping_type = (
                self.FEATURE_MAPPING[joint_name]
            )

            value = features[feature_name]

            # Nonlinear response curve (identity when gamma == 1.0).
            gamma = self.joint_gamma[joint_name]

            if gamma != 1.0:
                value = float(
                    np.sign(value)
                    * np.abs(value) ** gamma
                )

            if mapping_type == "positive":

                q = self._positive(
                    value=value,
                    zero=zero,
                    upper=t_upper,
                )

            elif mapping_type == "negative":

                q = self._negative(
                    value=value,
                    zero=zero,
                    lower=t_lower,
                )

            elif mapping_type == "bidirectional":

                q = self._bidirectional(
                    value=value,
                    zero=zero,
                    lower=t_lower,
                    upper=t_upper,
                )

            elif mapping_type == "bidirectional_inverted":

                q = self._bidirectional_inverted(
                    value=value,
                    zero=zero,
                    lower=t_lower,
                    upper=t_upper,
                )

            else:
                raise RuntimeError(
                    f"Unknown mapping type: {mapping_type}"
                )

            q_target[i] = np.clip(
                q,
                lower,
                upper,
            )

        raw_retarget_qcmd = q_target.copy()

        # Optional pinch-conditioned coordination: blend the base
        # branch toward pinch-specific targets using the continuous
        # pinch gate.  gate == 0 leaves the base output exactly as-is.
        if self.pinch_coordination is not None:
            gate = self.pinch_gate(features)

            if gate > 0.0:
                for joint_name, joint_cfg in self.pinch_coordination[
                    "joints"
                ].items():
                    i = self.joint_order.index(joint_name)

                    if "anchors" in joint_cfg:
                        anchors = joint_cfg["anchors"]
                        feature_name = self.FEATURE_MAPPING[joint_name][0]
                        c = float(
                            np.clip(features[feature_name], 0.0, 1.0)
                        )
                        q_pinch = float(np.interp(
                            c,
                            [a[0] for a in anchors],
                            [a[1] for a in anchors],
                        ))
                    else:
                        q_pinch = joint_cfg["pinch_target_deg"]

                    q_target[i] = (
                        (1.0 - gate) * q_target[i]
                        + gate * q_pinch
                    )

                    m_lower, m_upper = map(
                        float, self.joints[joint_name]["range"]
                    )
                    q_target[i] = np.clip(
                        q_target[i], m_lower, m_upper
                    )

        coordinated_qcmd = q_target.copy()

        # Optional grasp-conditioned thumb lock: continuous inward
        # opposition bias proportional to the grasp/thumb-engagement
        # blend.  Absent section = base output exactly as-is.
        q_target, lock_info = self._apply_power_grasp_lock(
            q_target,
            features,
        )

        stages = {
            "raw_retarget_qcmd": raw_retarget_qcmd,
            "coordinated_qcmd": coordinated_qcmd,
            "final_qcmd": q_target.copy(),
            "power_grasp_lock": lock_info,
        }
        return q_target, stages

    def as_joint_dict(
        self,
        q_target: np.ndarray,
    ) -> dict[str, float]:
        q_target = np.asarray(q_target)

        if q_target.shape != (11,):
            raise ValueError(
                f"Expected shape (11,), "
                f"got {q_target.shape}"
            )

        return {
            joint: float(value)
            for joint, value
            in zip(self.joint_order, q_target)
        }
