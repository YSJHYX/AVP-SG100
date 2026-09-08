"""SG100 hand driver boundary: units, joint order, and command provenance.

Vendor facts (SG100_SDK_Publish @ V1.5_NO_TACTILE, docs/SG100_SDK_User_Guide.md,
docs/Realtime_Performance_Analysis.md, include/sg100_types.h):

- transport: UART serial (PC 0xFE -> palm board 0xFD -> 4x RS485 ->
  11 joint boards), default baudrate 1000000; the vendor CommScheduler
  drives the 10 ms realtime exchange in RS485 active mode
- SDK joint IDs 1..11 map to thumb(1,2,3), index(4,5,6),
  middle(7,8), little(9,10,11)
- the production command API is ``SetHandControl(Hand_Control_t)``:
  per-joint position is the SDK USER-ANGLE domain in DEGREES; the SDK
  applies ``joint_angle_map.json`` (clamp + a*x + b) and converts to
  wire radians when the scheduler transmits
- the hand status callback applies the SDK inverse map and rad-to-degree
  conversion without consulting returned joint mode. Real N1 proved
  MODE_NULL position is raw/accumulated-like. The 2026-09-03 real MODE7
  probe established bounded joint-angle-scale feedback, but exact inbound
  identity/sign/zero/range mapping to canonical coordinates is pending. It
  is measured state, never a command echo.

This module defines the ONLY place where software-canonical radians meet
vendor-domain values.  Outbound commands use F_cmd.  Inbound callbacks use
an independently validated F_fb; until it exists, callback values remain an
explicit non-canonical diagnostic domain.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]

DEFAULT_HARDWARE_MAP_PATH = (
    ROOT / "sg100_teleop" / "config" / "sg100_hardware_map.yaml"
)
DEFAULT_SEMANTICS_PATH = (
    ROOT / "sg100_teleop" / "config" / "sg100_joint_semantics.yaml"
)

HAND_DOF = 11

# Runtime command-limit comparisons use degrees because that is the public
# SetHandControl user-angle domain.  This epsilon is comparison/diagnostic
# tolerance only; every submitted value is still clipped to the exact vendor
# boundary.
SDK_LIMIT_COMPARE_EPSILON_DEG = 1e-6

# Per-joint control modes actually used by this stack (CtrlMode_e in the
# vendor header include/sg100_types.h @ V1.5_NO_TACTILE, lines 165-176:
# MODE_NULL=0, MODE_POSITION=1, ..., MODE_JOINT_POSITION=7).  These are the
# JOINT-BOARD control modes carried by Joint_Control_t.mode and echoed by
# Joint_Status_t.mode — a different concept from the palm runtime mode
# (Palm_Runtime_Mode_e / SwitchToRealtimeMode).
MODE_NULL = 0
MODE_POSITION = 1
MODE_JOINT_POSITION = 7

# Feedback-domain classification (mode-aware, conservative):
#   MODE7_JOINT_POSITION_CANDIDATE — every expected joint reported
#     MODE_JOINT_POSITION(7).  Vendor confirmed mode-7 feedback is the
#     intended correct joint angle; real hardware now supports the bounded
#     joint-angle-scale domain. Exact feedback-to-canonical mapping F_fb is
#     NOT yet validated, so canonical_valid stays False until that separate
#     validation passes (historical driver flag mode7_hardware_validated).
#   NON_CANONICAL_DIAGNOSTIC — any joint reported another mode (e.g.
#     MODE_NULL).  Real N1 proved MODE_NULL position is a raw/accumulated
#     multi-turn coordinate (~1854 callback deg for ~15-30 physical deg),
#     so such feedback is diagnostic only and may never become
#     measured_hand_q / dataset state / MuJoCo physical joint angle.
FEEDBACK_DOMAIN_MODE7_CANDIDATE = "MODE7_JOINT_POSITION_CANDIDATE"
FEEDBACK_DOMAIN_NON_CANONICAL_DIAGNOSTIC = "NON_CANONICAL_DIAGNOSTIC"
FEEDBACK_DOMAIN_UNCLASSIFIED = "UNCLASSIFIED_NO_MODE_INFORMATION"

# Independent inbound feedback-map status.  F_fb is deliberately separate
# from SG100HardwareMap (F_cmd); the absence of F_fb is not an identity-map
# claim.
FEEDBACK_MAP_STATUS_NOT_PROVIDED = "F_FB_NOT_PROVIDED"
FEEDBACK_MAP_STATUS_UNVALIDATED = "F_FB_PROVIDED_NOT_VALIDATED"
FEEDBACK_MAP_STATUS_VALIDATED = "F_FB_VALIDATED"

# Feedback provenance labels (DATA CONTRACT metadata).
FEEDBACK_SEMANTICS_MEASURED = "sg100_driver_reported_joint_feedback"
FEEDBACK_EVIDENCE = (
    "Hand_Status_t reaches the SDK status callback from the palm response "
    "to the realtime control exchange and is never a command echo; "
    "position semantics are returned-mode-dependent: MODE_NULL is "
    "real-hardware-proven raw/accumulated-like; MODE_JOINT_POSITION is "
    "real-hardware-supported as bounded joint-angle-scale feedback, while "
    "its exact feedback-to-canonical mapping remains pending"
)


def classify_feedback_domain(
    joint_modes: tuple[int, ...] | list[int] | np.ndarray | None,
) -> str:
    """Conservative mode-aware classification of one feedback sample.

    all == MODE_JOINT_POSITION -> MODE7_JOINT_POSITION_CANDIDATE
    anything else with modes   -> NON_CANONICAL_DIAGNOSTIC
    no mode information        -> UNCLASSIFIED_NO_MODE_INFORMATION
    """

    if joint_modes is None:
        return FEEDBACK_DOMAIN_UNCLASSIFIED
    modes = [int(m) for m in joint_modes]
    if len(modes) != HAND_DOF:
        return FEEDBACK_DOMAIN_NON_CANONICAL_DIAGNOSTIC
    if all(m == MODE_JOINT_POSITION for m in modes):
        return FEEDBACK_DOMAIN_MODE7_CANDIDATE
    return FEEDBACK_DOMAIN_NON_CANONICAL_DIAGNOSTIC


@dataclass(frozen=True)
class SG100FeedbackMap:
    """Independent MODE7 callback -> canonical mapping (``F_fb``).

    ``sdk_callback_index_by_canonical[i]`` selects the SDK callback channel
    for canonical joint ``i``.  The conversion is then::

        canonical[i] = sign[i] * (
            sdk_callback[selected_channel] - zero_offset_rad[i]
        )

    This represents identity/permutation, sign, and zero independently from
    the outbound command map.  ``validated`` must mean that all 11 entries
    were validated against physical joint identity, direction, and zero on
    the actual unit; constructing this object alone makes no such claim.
    """

    sdk_callback_index_by_canonical: tuple[int, ...]
    sign: tuple[int, ...]
    zero_offset_rad: tuple[float, ...]
    validated: bool = False

    def __post_init__(self) -> None:
        index_values = tuple(
            float(v) for v in self.sdk_callback_index_by_canonical
        )
        sign_values = tuple(float(v) for v in self.sign)
        zeros = tuple(float(v) for v in self.zero_offset_rad)
        if (
            len(index_values) != HAND_DOF
            or not np.all(np.isfinite(index_values))
            or any(not v.is_integer() for v in index_values)
        ):
            raise ValueError(
                "sdk_callback_index_by_canonical must be a permutation "
                "of 0..10"
            )
        indices = tuple(int(v) for v in index_values)
        if sorted(indices) != list(range(HAND_DOF)):
            raise ValueError(
                "sdk_callback_index_by_canonical must be a permutation "
                "of 0..10"
            )
        if (
            len(sign_values) != HAND_DOF
            or not np.all(np.isfinite(sign_values))
            or any(v not in (-1.0, 1.0) for v in sign_values)
        ):
            raise ValueError(
                "feedback-map sign must contain 11 values in {-1, +1}"
            )
        signs = tuple(int(v) for v in sign_values)
        if len(zeros) != HAND_DOF or not np.all(np.isfinite(zeros)):
            raise ValueError(
                "feedback-map zero_offset_rad must be 11 finite values"
            )
        if not isinstance(self.validated, (bool, np.bool_)):
            raise ValueError("feedback-map validated must be a boolean")
        object.__setattr__(self, "sdk_callback_index_by_canonical", indices)
        object.__setattr__(self, "sign", signs)
        object.__setattr__(self, "zero_offset_rad", zeros)
        object.__setattr__(self, "validated", bool(self.validated))

    def callback_to_canonical(self, sdk_callback_rad: np.ndarray) -> np.ndarray:
        """Apply a fully validated independent map to SDK callback values."""

        if not self.validated:
            raise RuntimeError(
                "refusing to apply an unvalidated feedback map as canonical"
            )
        callback = np.asarray(sdk_callback_rad, dtype=np.float64)
        if callback.shape != (HAND_DOF,):
            raise ValueError(
                f"expected shape (11,) SDK callback values, got {callback.shape}"
            )
        if not np.all(np.isfinite(callback)):
            raise ValueError("non-finite SDK callback values")
        ordered = callback[np.asarray(self.sdk_callback_index_by_canonical)]
        return np.asarray(self.sign, dtype=np.float64) * (
            ordered - np.asarray(self.zero_offset_rad, dtype=np.float64)
        )


@dataclass(frozen=True)
class SG100HandFeedback:
    """One measured SG100 callback sample with explicit mapping state.

    ``q_rad`` is canonical only when ``canonical_valid`` is true.  Otherwise
    it remains the SDK callback value converted degree-for-degree to radians
    for backwards-compatible diagnostics; use
    :attr:`sdk_callback_positions_rad` when consuming that raw domain.
    """

    q_rad: np.ndarray
    received_monotonic_ns: int
    sequence: int
    valid: bool
    # SDK callback value converted by pi/180 for storage. Physical meaning is
    # returned-mode-dependent; it is not validated canonical state until the
    # mode-7 gate and hardware verification exist.
    raw_positions: np.ndarray | None = None
    currents: np.ndarray | None = None       # decoded ampere, audit
    # Returned Joint_Status.mode bytes (hardware joint-board state), or None
    # when the producing driver carries no mode information (mock drivers).
    joint_modes: tuple[int, ...] | None = None
    # Mode-aware classification of this sample (classify_feedback_domain).
    feedback_domain: str = FEEDBACK_DOMAIN_UNCLASSIFIED
    # FALSE until the exact mode-7 feedback-to-canonical identity/sign/zero/
    # range map is validated on this unit. Angle-scale support alone is not
    # sufficient.
    canonical_valid: bool = False
    # Status of the independently configured F_fb.  This does not describe
    # (and must never be inferred from) the outbound command map.
    feedback_mapping_status: str = FEEDBACK_MAP_STATUS_NOT_PROVIDED
    # True only when a validated F_fb was actually applied to this MODE7
    # sample and the legacy hardware-validation gate was also enabled.
    feedback_mapping_applied: bool = False

    @property
    def sdk_callback_positions_rad(self) -> np.ndarray | None:
        """Exact SDK callback values after only the degree->radian unit step."""

        return self.raw_positions

    @property
    def feedback_semantics(self) -> str:
        return FEEDBACK_SEMANTICS_MEASURED


@dataclass(frozen=True)
class SG100CommandSent:
    """A command successfully handed to the vendor transport.

    ``transmission_confirmed`` documents what the SDK can prove:
    ``SetHandControl`` returning true means the frame was accepted into
    the vendor send cache ("successfully_transmitted"), NOT that the
    joint physically executed the motion.  Only measured feedback can
    prove execution, and this event never claims it.
    """

    q_rad: np.ndarray
    sent_monotonic_ns: int
    command_bundle_id: int | None
    transmission_confirmed: str = "successfully_transmitted"


@dataclass(frozen=True)
class SG100JointLimit:
    """One row of the authoritative canonical -> SDK command contract."""

    joint_name: str
    canonical_min_rad: float
    canonical_max_rad: float
    sdk_min_deg: float
    sdk_max_deg: float
    command_gain: float
    command_offset_rad: float


@dataclass(frozen=True)
class SG100LimitApplication:
    """Desired, clamped and mapped values for one whole-hand command."""

    desired_canonical_rad: np.ndarray
    canonical_clamped_rad: np.ndarray
    sdk_before_guard_deg: np.ndarray
    sdk_sent_deg: np.ndarray
    effective_canonical_rad: np.ndarray
    canonical_clamped_indices: tuple[int, ...]
    sdk_clamped_indices: tuple[int, ...]
    sdk_mapping_mismatch_indices: tuple[int, ...]


class SG100CommandLimitContract:
    """Limits derived once from runtime SDK ranges and the validated F_cmd.

    The retargeting/morphology limits remain untouched.  Command-safe
    canonical limits are the exact inverse image of the production SDK's
    current ``joint_angle_map.json`` user-degree ranges, so there is no second
    hand-maintained table to drift.
    """

    def __init__(
        self,
        hardware_map: "SG100HardwareMap",
        vendor_user_ranges: list[tuple[float, float]],
    ) -> None:
        if len(vendor_user_ranges) != HAND_DOF:
            raise ValueError("11 SDK command ranges are required")
        self.hardware_map = hardware_map
        sdk_min = np.asarray([v[0] for v in vendor_user_ranges], dtype=np.float64)
        sdk_max = np.asarray([v[1] for v in vendor_user_ranges], dtype=np.float64)
        if (
            sdk_min.shape != (HAND_DOF,)
            or not np.all(np.isfinite(sdk_min))
            or not np.all(np.isfinite(sdk_max))
            or np.any(sdk_min > sdk_max)
        ):
            raise ValueError("SDK command ranges must be 11 finite [min,max] pairs")
        self.sdk_min_deg = sdk_min
        self.sdk_max_deg = sdk_max

        sdk_min_rad = np.deg2rad(sdk_min)
        sdk_max_rad = np.deg2rad(sdk_max)
        inverse_a = (sdk_min_rad - hardware_map.offset_rad) / hardware_map.gain
        inverse_b = (sdk_max_rad - hardware_map.offset_rad) / hardware_map.gain
        self.canonical_min_rad = np.minimum(inverse_a, inverse_b)
        self.canonical_max_rad = np.maximum(inverse_a, inverse_b)

        self.rows = tuple(
            SG100JointLimit(
                joint_name=hardware_map.software_joint_order[i],
                canonical_min_rad=float(self.canonical_min_rad[i]),
                canonical_max_rad=float(self.canonical_max_rad[i]),
                sdk_min_deg=float(self.sdk_min_deg[i]),
                sdk_max_deg=float(self.sdk_max_deg[i]),
                command_gain=float(hardware_map.gain[i]),
                command_offset_rad=float(hardware_map.offset_rad[i]),
            )
            for i in range(HAND_DOF)
        )

    def apply(self, desired_canonical_rad: np.ndarray) -> SG100LimitApplication:
        desired = np.asarray(desired_canonical_rad, dtype=np.float64)
        if desired.shape != (HAND_DOF,) or not np.all(np.isfinite(desired)):
            raise ValueError("canonical qcmd must be finite 11D")

        canonical = np.clip(
            desired, self.canonical_min_rad, self.canonical_max_rad
        )
        sdk_before = np.rad2deg(
            self.hardware_map.gain * canonical
            + self.hardware_map.offset_rad
        )
        # Second, SDK-domain guard immediately before serialization.  np.clip
        # guarantees the physical range is never enlarged by epsilon.
        sdk_sent = np.clip(sdk_before, self.sdk_min_deg, self.sdk_max_deg)
        effective = (
            np.deg2rad(sdk_sent) - self.hardware_map.offset_rad
        ) / self.hardware_map.gain

        canonical_changed = np.flatnonzero(
            np.abs(canonical - desired) > 0.0
        )
        sdk_delta = np.abs(sdk_sent - sdk_before)
        sdk_changed = np.flatnonzero(sdk_delta > 0.0)
        mismatch = np.flatnonzero(
            sdk_delta > SDK_LIMIT_COMPARE_EPSILON_DEG
        )
        return SG100LimitApplication(
            desired_canonical_rad=desired.copy(),
            canonical_clamped_rad=canonical.copy(),
            sdk_before_guard_deg=sdk_before.copy(),
            sdk_sent_deg=sdk_sent.copy(),
            effective_canonical_rad=effective.copy(),
            canonical_clamped_indices=tuple(int(i) for i in canonical_changed),
            sdk_clamped_indices=tuple(int(i) for i in sdk_changed),
            sdk_mapping_mismatch_indices=tuple(int(i) for i in mismatch),
        )


@dataclass(frozen=True)
class ZeroPoseAssessment:
    """Startup zero decision input (reporting only — no zeroing action).

    Production SOP: the hand should already sit at the authoritative
    mechanical zero before the program starts.  This assessment compares
    MEASURED canonical feedback against canonical zero and reports the
    deviation; it never redefines zero from the startup pose and never
    commands motion.  The controlled return-to-zero path remains blocked
    until the vendor zero semantics are validated on hardware.
    """

    at_zero: bool
    max_abs_deviation_rad: float
    per_joint_deviation_rad: np.ndarray


def assess_zero_pose(
    q_rad: np.ndarray,
    tolerance_rad: float,
) -> ZeroPoseAssessment:
    """Compare measured canonical feedback against canonical zero."""

    q = np.asarray(q_rad, dtype=np.float64)
    if q.shape != (HAND_DOF,) or not np.all(np.isfinite(q)):
        raise ValueError("finite 11D canonical feedback required")
    if not np.isfinite(tolerance_rad) or tolerance_rad < 0.0:
        raise ValueError("tolerance_rad must be finite and >= 0")
    deviation = np.abs(q)
    return ZeroPoseAssessment(
        at_zero=bool(float(deviation.max()) <= tolerance_rad),
        max_abs_deviation_rad=float(deviation.max()),
        per_joint_deviation_rad=deviation.copy(),
    )


class SG100HardwareMap:
    """Authoritative outbound command mapping (``F_cmd``) and limits.

    conversion (send):     sdk_user = gain * canonical + offset
    legacy inverse helper: canonical = gain * (sdk_user - offset)

    The result is the SDK API coordinate expressed in radians; the vendor
    then applies ``joint_angle_map.json`` on commands.  An identity map does
    NOT establish physical 1:1 scale. Real MODE7 entry established the
    joint-angle-scale domain but left the physical canonical mapping blocked;
    this class must not be read as, or automatically reused for, inbound
    mapping validation.  Real SDK feedback uses :class:`SG100FeedbackMap`.
    """

    def __init__(
        self,
        map_path: Path | str = DEFAULT_HARDWARE_MAP_PATH,
        semantics_path: Path | str = DEFAULT_SEMANTICS_PATH,
    ):
        map_path = Path(map_path)
        semantics_path = Path(semantics_path)

        # encoding="utf-8": the YAML files carry non-ASCII comments and
        # Windows' locale default (e.g. GBK) cannot decode them.
        with map_path.open("r", encoding="utf-8") as f:
            hardware = yaml.safe_load(f)
        with semantics_path.open("r", encoding="utf-8") as f:
            semantics = yaml.safe_load(f)

        self.software_joint_order = list(
            semantics["joint_order"]
        )
        if len(self.software_joint_order) != HAND_DOF:
            raise ValueError(
                "expected 11 software hand joints, got "
                f"{len(self.software_joint_order)}"
            )

        hw_order = list(hardware["software_joint_order"])
        if hw_order != self.software_joint_order:
            raise ValueError(
                "sg100_hardware_map software_joint_order disagrees with "
                "sg100_joint_semantics joint_order"
            )

        self.hardware_joint_ids = [
            int(v) for v in hardware["hardware_joint_ids"]
        ]
        if len(self.hardware_joint_ids) != HAND_DOF:
            raise ValueError("hardware_joint_ids must have 11 entries")
        if sorted(self.hardware_joint_ids) != list(range(1, 12)):
            raise ValueError(
                "hardware_joint_ids must be a permutation of 1..11"
            )

        self.gain = np.asarray(hardware["gain"], dtype=np.float64)
        self.offset_rad = np.asarray(
            hardware["offset_rad"], dtype=np.float64
        )
        if self.gain.shape != (HAND_DOF,) or self.offset_rad.shape != (
            HAND_DOF,
        ):
            raise ValueError("gain/offset_rad must have 11 entries")
        if not np.all(np.isfinite(self.gain)) or not np.all(
            np.isfinite(self.offset_rad)
        ):
            raise ValueError("gain/offset_rad must be finite")
        if not np.all(np.abs(self.gain) > 0.0):
            raise ValueError("gain entries must be non-zero")

        self.software_index_of = {
            name: i
            for i, name in enumerate(self.software_joint_order)
        }
        # hardware joint id (SDK 1..11) -> software index
        self.hardware_id_to_software_index = {
            joint_id: i for i, joint_id in enumerate(self.hardware_joint_ids)
        }

        # Canonical-domain mechanical limits from the semantics config;
        # every outbound command is clamped here regardless of what the
        # SDK would allow.
        self.lower_rad = np.zeros(HAND_DOF)
        self.upper_rad = np.zeros(HAND_DOF)
        for i, name in enumerate(self.software_joint_order):
            lo, hi = semantics["joints"][name]["range"]
            self.lower_rad[i] = float(lo)
            self.upper_rad[i] = float(hi)

    # -- boundary conversions -------------------------------------------

    def canonical_to_actual(self, q_rad: np.ndarray) -> np.ndarray:
        q = np.asarray(q_rad, dtype=np.float64)
        if q.shape != (HAND_DOF,):
            raise ValueError(f"expected shape (11,), got {q.shape}")
        if not np.all(np.isfinite(q)):
            raise ValueError("non-finite canonical command")
        clipped = np.clip(q, self.lower_rad, self.upper_rad)
        return self.gain * clipped + self.offset_rad

    def actual_to_canonical(self, actual: np.ndarray) -> np.ndarray:
        """Legacy algebraic inverse of F_cmd; not a validated F_fb.

        Retained for mock/round-trip compatibility.  Production SDK callback
        handling must use an independent :class:`SG100FeedbackMap`.
        """

        a = np.asarray(actual, dtype=np.float64)
        if a.shape != (HAND_DOF,):
            raise ValueError(
                f"expected shape (11,) vendor feedback, got {a.shape}"
            )
        return self.gain * (a - self.offset_rad)

    def clamp_canonical(self, q_rad: np.ndarray) -> np.ndarray:
        q = np.asarray(q_rad, dtype=np.float64)
        return np.clip(q, self.lower_rad, self.upper_rad)

    def mapping_table(self) -> list[dict]:
        """Human-readable software <-> hardware mapping (all 11 DoF)."""

        return [
            {
                "software_name": name,
                "software_index": i,
                "hardware_joint_id": self.hardware_joint_ids[i],
                "hardware_name": f"J{self.hardware_joint_ids[i]}",
                "gain": float(self.gain[i]),
                "offset_rad": float(self.offset_rad[i]),
            }
            for i, name in enumerate(self.software_joint_order)
        ]


class SG100Driver:
    """Abstract canonical-rad driver for one SG100 hand.

    Contract:
    - ``connect/start/stop/disconnect`` lifecycle;
    - ``read_feedback`` returns measured joint feedback (never a command
      echo), with a host monotonic RECEIVE timestamp taken immediately after
      the read.  Its ``q_rad`` is canonical only when ``canonical_valid``;
      otherwise the SDK callback domain is explicitly exposed for diagnosis;
    - ``send_positions_rad`` returns ``SG100CommandSent`` only when the
      transport accepted the frame; failures return ``None`` and are
      never canonicalized;
    - ``drain_command_events`` mirrors the RM65 backend event drain so
      the collector can join arm and hand provenance.
    """

    feedback_semantics = FEEDBACK_SEMANTICS_MEASURED
    feedback_timestamp_source = "host_monotonic_receive"

    def connect(self) -> None:  # pragma: no cover - interface
        raise NotImplementedError

    def start(self) -> None:  # pragma: no cover - interface
        raise NotImplementedError

    def stop(self) -> None:  # pragma: no cover - interface
        raise NotImplementedError

    def disconnect(self) -> None:  # pragma: no cover - interface
        raise NotImplementedError

    def read_feedback(self) -> SG100HandFeedback | None:  # pragma: no cover
        raise NotImplementedError

    def send_positions_rad(
        self,
        q_rad: np.ndarray,
        *,
        command_bundle_id: int | None = None,
    ) -> SG100CommandSent | None:  # pragma: no cover - interface
        raise NotImplementedError

    def drain_command_events(self) -> tuple[SG100CommandSent, ...]:
        return ()

    @property
    def fault_active(self) -> bool:
        return False

    @property
    def hardware_limits_available(self) -> bool:
        return False
