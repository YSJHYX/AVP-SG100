"""Offline inspection for real ``rm65_sg100_joint17_v1`` episodes.

The inspector is deliberately independent from the live collector.  It
recomputes schema, causality, command-bundle, raw/canonical, finite-value, and
stored-filter checks from files already on disk.  It never repairs, filters,
or deletes episode data.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass
import json
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping

import numpy as np

from sg100_teleop.data_collection.schema import (
    ARM_DIM,
    ARM_SLICE,
    HAND_DIM,
    HAND_SLICE,
    PHYSICAL_DIM,
    SCHEMA_NAME,
)


CANONICAL_FILENAME = "arm_hand_joint17_v1.npz"
MANIFEST_FILENAME = "arm_hand_manifest.json"
RAW_FILENAME = "arm_hand_raw_events.jsonl"
MATCH_ATOL = 1e-7
MAX_DEFAULT_ERRORS = 20
NOT_RECORDED = "NOT RECORDED"
NOT_AVAILABLE = "NOT AVAILABLE"
NOT_VERIFIABLE = "NOT VERIFIABLE FROM CURRENT RAW STORAGE"


@dataclass(frozen=True)
class EpisodeFiles:
    requested_path: Path
    root: Path
    canonical: Path
    manifest: Path | None
    raw: Path | None


@dataclass
class InspectionReport:
    files: EpisodeFiles
    summary: dict[str, Any]
    transitions: list[dict[str, Any]]
    failures: list[dict[str, Any]]

    @property
    def passed(self) -> bool:
        return bool(self.summary["result"]["data_contract_pass"])

    def json_payload(
        self,
        *,
        dump_transition: int | None = None,
        dump_failures: bool = False,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = dict(self.summary)
        if dump_transition is not None:
            payload["transition"] = self.transition(dump_transition)
        if dump_failures:
            payload["failures"] = self.failures
        return payload

    def transition(self, index: int) -> dict[str, Any]:
        if index < 0 or index >= len(self.transitions):
            raise IndexError(
                f"transition index {index} outside [0, {len(self.transitions)})"
            )
        return self.transitions[index]


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, np.generic):
        return _jsonable(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        if np.isnan(value):
            return "NaN"
        return "+Inf" if value > 0.0 else "-Inf"
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


def resolve_episode_files(path: str | Path) -> EpisodeFiles:
    requested = Path(path).expanduser().resolve()
    if not requested.exists():
        raise FileNotFoundError(f"episode path does not exist: {requested}")

    if requested.is_file():
        if requested.suffix.lower() != ".npz":
            raise ValueError(
                "current DATA CONTRACT V1 episodes use .npz; "
                f"unsupported data file: {requested}"
            )
        canonical = requested
        root = canonical.parent
    else:
        direct = requested / CANONICAL_FILENAME
        nested = requested / "arm_hand_data" / CANONICAL_FILENAME
        if direct.is_file():
            canonical = direct
        elif nested.is_file():
            canonical = nested
        else:
            candidates = sorted(requested.rglob(CANONICAL_FILENAME))
            if len(candidates) != 1:
                raise FileNotFoundError(
                    f"expected one {CANONICAL_FILENAME} below {requested}, "
                    f"found {len(candidates)}; use --all for a dataset root"
                )
            canonical = candidates[0]
        root = canonical.parent

    manifest = root / MANIFEST_FILENAME
    raw = root / RAW_FILENAME
    return EpisodeFiles(
        requested_path=requested,
        root=root,
        canonical=canonical,
        manifest=manifest if manifest.is_file() else None,
        raw=raw if raw.is_file() else None,
    )


def discover_episodes(path: str | Path) -> list[Path]:
    root = Path(path).expanduser().resolve()
    if not root.is_dir():
        raise ValueError("--all requires a directory")
    return sorted(item.parent for item in root.rglob(CANONICAL_FILENAME))


def _load_manifest(path: Path | None) -> tuple[dict[str, Any], list[str]]:
    if path is None:
        return {}, [f"missing {MANIFEST_FILENAME}"]
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {}, [f"could not parse manifest: {exc}"]
    if not isinstance(value, dict):
        return {}, ["manifest root must be a JSON object"]
    return value, []


def _load_raw(path: Path | None) -> tuple[list[dict[str, Any]], list[str]]:
    if path is None:
        return [], [f"missing {RAW_FILENAME}"]
    events: list[dict[str, Any]] = []
    errors: list[str] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        return [], [f"could not read raw stream: {exc}"]
    for line_number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError as exc:
            errors.append(f"raw line {line_number}: {exc}")
            continue
        if not isinstance(event, dict):
            errors.append(f"raw line {line_number}: event must be an object")
            continue
        events.append(event)
    return events, errors


def _payload(event: Mapping[str, Any]) -> Mapping[str, Any]:
    value = event.get("payload", {})
    return value if isinstance(value, Mapping) else {}


def _event_timestamp(event: Mapping[str, Any]) -> int | None:
    try:
        return int(event["master_timestamp_ns"])
    except (KeyError, TypeError, ValueError):
        return None


def _bundle_id(payload: Mapping[str, Any]) -> int | None:
    value = payload.get("command_bundle_id")
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _vector(payload: Mapping[str, Any], key: str, size: int) -> np.ndarray | None:
    try:
        value = np.asarray(payload[key], dtype=np.float64)
    except (KeyError, TypeError, ValueError):
        return None
    if value.shape != (size,):
        return None
    return value


def _array_indices(mask: np.ndarray) -> list[int]:
    return np.flatnonzero(np.asarray(mask, dtype=bool)).astype(int).tolist()


def _timing_stats(values_ms: np.ndarray) -> dict[str, float | None]:
    values = np.asarray(values_ms, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {name: None for name in ("mean", "p50", "p95", "p99", "max")}
    return {
        "mean": float(np.mean(values)),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "max": float(np.max(values)),
    }


def _aggregate_verification(values: Iterable[bool | str | None]) -> bool | str:
    values = list(values)
    if not values or any(value is None or value == NOT_VERIFIABLE for value in values):
        if not any(value is False for value in values):
            return NOT_VERIFIABLE
    if any(value is False for value in values):
        return False
    return True


def _limits_from_manifest(
    manifest: Mapping[str, Any],
) -> tuple[np.ndarray, np.ndarray] | None:
    joint_limits = manifest.get("joint_limits")
    if isinstance(joint_limits, Mapping):
        try:
            lower = np.asarray(joint_limits["lower"], dtype=np.float64)
            upper = np.asarray(joint_limits["upper"], dtype=np.float64)
        except (KeyError, TypeError, ValueError):
            return None
        if lower.shape == (PHYSICAL_DIM,) and upper.shape == (PHYSICAL_DIM,):
            return lower, upper

    keys = ("arm_lower_rad", "arm_upper_rad", "hand_lower_rad", "hand_upper_rad")
    if not all(key in manifest for key in keys):
        return None
    try:
        arm_lower = np.asarray(manifest[keys[0]], dtype=np.float64)
        arm_upper = np.asarray(manifest[keys[1]], dtype=np.float64)
        hand_lower = np.asarray(manifest[keys[2]], dtype=np.float64)
        hand_upper = np.asarray(manifest[keys[3]], dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if (
        arm_lower.shape != (ARM_DIM,)
        or arm_upper.shape != (ARM_DIM,)
        or hand_lower.shape != (HAND_DIM,)
        or hand_upper.shape != (HAND_DIM,)
    ):
        return None
    return (
        np.concatenate((arm_lower, hand_lower)),
        np.concatenate((arm_upper, hand_upper)),
    )


def _raw_counts(events: list[dict[str, Any]], raw_available: bool) -> dict[str, Any]:
    if not raw_available:
        return {
            key: NOT_RECORDED
            for key in (
                "arm_feedback",
                "arm_goal",
                "arm_sent_command",
                "hand_command",
                "hand_proxy_baseline",
                "avp_target",
                "safety_event",
                "tracking_loss",
            )
        }
    event_types = Counter(str(event.get("event_type", "")) for event in events)
    decisions = [_payload(event) for event in events if event.get("event_type") == "control_decision"]
    goals_recorded = any(
        isinstance(item.get("audit"), Mapping)
        and "ik_q_arm_goal" in item["audit"]
        for item in decisions
    )
    avp_recorded = any(
        isinstance(item.get("audit"), Mapping)
        and "raw_avp_wrist_pose" in item["audit"]
        for item in decisions
    )
    safety_recorded = any("safety_ok" in item for item in decisions)
    tracking_recorded = any("tracking_valid" in item for item in decisions)
    return {
        "arm_feedback": int(event_types["arm_feedback"]),
        "arm_goal": (
            sum(
                isinstance(item.get("audit"), Mapping)
                and "ik_q_arm_goal" in item["audit"]
                for item in decisions
            )
            if goals_recorded
            else NOT_RECORDED
        ),
        "arm_sent_command": int(event_types["arm_command_sent"]),
        "hand_command": int(event_types["control_decision"]),
        "hand_proxy_baseline": int(event_types["hand_proxy_baseline"]),
        "avp_target": (
            sum(
                isinstance(item.get("audit"), Mapping)
                and "raw_avp_wrist_pose" in item["audit"]
                for item in decisions
            )
            if avp_recorded
            else NOT_RECORDED
        ),
        "safety_event": (
            sum(not bool(item.get("safety_ok")) for item in decisions)
            if safety_recorded
            else NOT_RECORDED
        ),
        "tracking_loss": (
            sum(not bool(item.get("tracking_valid")) for item in decisions)
            if tracking_recorded
            else NOT_RECORDED
        ),
    }


def inspect_episode(path: str | Path, *, atol: float = MATCH_ATOL) -> InspectionReport:
    """Inspect one episode without modifying it."""

    files = resolve_episode_files(path)
    manifest, manifest_errors = _load_manifest(files.manifest)
    events, raw_errors = _load_raw(files.raw)
    load_errors = manifest_errors + raw_errors

    try:
        with np.load(files.canonical, allow_pickle=False) as archive:
            arrays = {key: np.asarray(archive[key]) for key in archive.files}
    except (OSError, ValueError) as exc:
        raise ValueError(f"could not load canonical NPZ: {exc}") from exc

    required_vectors = ("state", "action", "next_state")
    missing_vectors = [name for name in required_vectors if name not in arrays]
    vector_shapes = {
        name: list(arrays[name].shape) if name in arrays else None
        for name in required_vectors
    }
    shape_valid = not missing_vectors and all(
        arrays[name].ndim == 2 and arrays[name].shape[1] == PHYSICAL_DIM
        for name in required_vectors
    )
    lengths = [arrays[name].shape[0] for name in required_vectors if name in arrays]
    same_length = len(lengths) == 3 and len(set(lengths)) == 1
    shape_valid = bool(shape_valid and same_length)
    n = int(lengths[0]) if same_length else 0

    archive_schema = None
    if "schema_name" in arrays:
        try:
            archive_schema = str(arrays["schema_name"].item())
        except ValueError:
            archive_schema = None
    manifest_schema = manifest.get("schema", manifest.get("schema_version"))
    arm_unit = manifest.get("arm_unit", manifest.get("unit"))
    hand_unit = manifest.get("hand_unit", manifest.get("unit"))
    action_representation = manifest.get(
        "action_representation", manifest.get("representation")
    )
    hand_measured_episode = (
        manifest.get("hand_state_source")
        == "sg100_measured_joint_feedback"
    )

    metadata_checks = {
        "archive_schema": archive_schema == SCHEMA_NAME,
        "manifest_schema": manifest_schema == SCHEMA_NAME,
        "physical_dimension": manifest.get("physical_dimension") == PHYSICAL_DIM,
        "arm_unit": arm_unit == "rad",
        "hand_unit": hand_unit == "rad",
        "action_representation": action_representation == "absolute_joint_position",
        "arm_state_source": manifest.get("arm_state_source")
        == "rm65_measured_joint_feedback",
        "arm_action_source": manifest.get("arm_action_source")
        == "real_joint_command_sent",
    }
    if hand_measured_episode:
        metadata_checks.update({
            "hand_state_source": manifest.get("hand_state_source")
            == "sg100_measured_joint_feedback",
            "hand_initial_state_source": manifest.get("hand_initial_state_source")
            == "first_measured_feedback_replay_seed",
            "hand_action_source": manifest.get("hand_action_source")
            == "final_hardware_sent_qcmd",
            "hand_feedback_available": manifest.get("hand_feedback_available")
            is True,
            "hand_feedback_valid": manifest.get("hand_feedback_valid") is True,
            "hand_action_not_hardware_applied": manifest.get(
                "hand_action_hardware_applied"
            ) is False,
            "hand_hardware_present": manifest.get("hand_hardware_present") is True,
            "hand_action_hardware_sent": manifest.get("hand_action_hardware_sent")
            is True,
        })
    else:
        metadata_checks.update({
            "hand_state_source": manifest.get("hand_state_source")
            == "previous_command_proxy",
            "hand_initial_state_source": manifest.get("hand_initial_state_source")
            == "serialized_command_proxy_baseline",
            "hand_action_source": manifest.get("hand_action_source")
            in ("final_teleop_qcmd", "final_hand_qcmd"),
            "hand_feedback_unavailable": manifest.get("hand_feedback_available") is False,
            "hand_feedback_invalid": manifest.get("hand_feedback_valid") is False,
            "hand_action_not_hardware_applied": manifest.get(
                "hand_action_hardware_applied"
            ) is False,
            "hand_hardware_absent": manifest.get("hand_hardware_present") is False,
        })
    metadata_valid = bool(all(metadata_checks.values()))

    timestamp_names = (
        "state_timestamp_ns",
        "decision_timestamp_ns",
        "arm_command_sent_timestamp_ns",
        "hand_command_applied_timestamp_ns",
        "action_effective_timestamp_ns",
        "next_state_timestamp_ns",
    )
    timestamp_valid = shape_valid and all(
        name in arrays and arrays[name].shape == (n,) for name in timestamp_names
    )
    bundle_array_valid = (
        shape_valid
        and "command_bundle_id" in arrays
        and arrays["command_bundle_id"].shape == (n,)
    )

    arm_events = [event for event in events if event.get("event_type") == "arm_command_sent"]
    hand_events = [event for event in events if event.get("event_type") == "control_decision"]
    feedback_events = [event for event in events if event.get("event_type") == "arm_feedback"]
    rejection_events = [event for event in events if event.get("event_type") == "hard_rejection"]
    baseline_events = [
        event for event in events if event.get("event_type") == "hand_proxy_baseline"
    ]
    hand_sent_events = [
        event
        for event in events
        if event.get("event_type") == "hand_hardware_sent"
    ]
    hand_feedback_events = [
        event for event in events if event.get("event_type") == "hand_feedback"
    ]
    hand_measured_baseline_events = [
        event
        for event in events
        if event.get("event_type") == "hand_measured_baseline"
    ]

    arm_exact: dict[tuple[int, int], list[Mapping[str, Any]]] = defaultdict(list)
    hand_exact: dict[tuple[int, int], list[Mapping[str, Any]]] = defaultdict(list)
    feedback_exact: dict[int, list[Mapping[str, Any]]] = defaultdict(list)
    arm_bundle_ids: set[int] = set()
    hand_bundle_ids: set[int] = set()
    hand_history: list[tuple[int, int, Mapping[str, Any]]] = []

    for event in arm_events:
        payload = _payload(event)
        bundle_id = _bundle_id(payload)
        timestamp = _event_timestamp(event)
        if bundle_id is not None:
            arm_bundle_ids.add(bundle_id)
        if bundle_id is not None and timestamp is not None:
            arm_exact[(bundle_id, timestamp)].append(payload)

    for event in hand_events:
        payload = _payload(event)
        bundle_id = _bundle_id(payload)
        try:
            timestamp = int(payload["applied_timestamp_ns"])
        except (KeyError, TypeError, ValueError):
            timestamp = None
        if bundle_id is not None:
            hand_bundle_ids.add(bundle_id)
        if bundle_id is not None and timestamp is not None:
            hand_exact[(bundle_id, timestamp)].append(payload)
            hand_history.append((timestamp, bundle_id, payload))
    hand_history.sort(key=lambda item: (item[0], item[1]))

    if hand_measured_episode:
        # Measured episodes: canonical hand action is the
        # hardware-sent qcmd, so per-transition hand commands are
        # matched against hand_hardware_sent events at
        # (bundle_id, master_timestamp_ns == sent time).
        hand_exact = defaultdict(list)
        for event in hand_sent_events:
            payload = _payload(event)
            bundle_id = _bundle_id(payload)
            timestamp = _event_timestamp(event)
            if bundle_id is not None and timestamp is not None:
                hand_exact[(bundle_id, timestamp)].append(payload)

    hand_feedback_timeline: list[tuple[int, Mapping[str, Any]]] = []
    for event in hand_feedback_events:
        timestamp = _event_timestamp(event)
        if timestamp is not None:
            hand_feedback_timeline.append((timestamp, _payload(event)))
    hand_feedback_timeline.sort(key=lambda item: item[0])

    intervening_hand_commands = sum(
        1
        for event in rejection_events
        if _payload(event).get("reason") == "INTERVENING_HAND_COMMAND"
    )

    for event in feedback_events:
        timestamp = _event_timestamp(event)
        if timestamp is not None:
            feedback_exact[timestamp].append(_payload(event))

    canonical_ids = (
        [int(value) for value in arrays["command_bundle_id"].tolist()]
        if bundle_array_valid
        else []
    )
    duplicate_ids = sorted(
        bundle_id for bundle_id, count in Counter(canonical_ids).items() if count > 1
    )

    transitions: list[dict[str, Any]] = []
    failures: list[dict[str, Any]] = []
    state_to_action_ms: list[float] = []
    action_to_next_ms: list[float] = []
    feedback_interval_ms: list[float] = []
    arm_to_feedback_ms: list[float] = []
    hand_to_feedback_ms: list[float] = []
    bundle_skew_ms: list[float] = []

    def add_failure(index: int | None, bundle_id: int | None, reason: str, timestamps: Any) -> None:
        failures.append(
            {
                "index": index,
                "bundle_id": bundle_id,
                "reason": reason,
                "timestamps": _jsonable(timestamps),
            }
        )

    baseline_field_present = "hand_proxy_baseline_serialized" in manifest
    baseline_declared = manifest.get("hand_proxy_baseline_serialized") is True
    legacy_baseline = not baseline_field_present
    baseline_event: Mapping[str, Any] | None = None
    baseline_q: np.ndarray | None = None
    baseline_timestamp: int | None = None
    baseline_format_valid = False
    baseline_manifest_timestamp_valid = False
    baseline_temporal_valid = False
    baseline_state_match: bool | str = NOT_VERIFIABLE
    baseline_status = "NOT SERIALIZED — LEGACY EPISODE" if legacy_baseline else "INVALID"

    if hand_measured_episode:
        # Measured-feedback episode: the replay seed is the FIRST
        # MEASURED SG100 feedback, recorded as audit only.  The
        # canonical state_hand comes from measured feedback strictly
        # before each action, so the frozen V1 requirement
        # "baseline == state[0]" deliberately does NOT apply here.
        baseline_status = "MEASURED SG100 FEEDBACK (REPLAY SEED)"
        baseline_format_valid = bool(hand_measured_baseline_events)
        first_hand_action_timestamp = (
            int(arrays["hand_command_applied_timestamp_ns"][0])
            if timestamp_valid and n > 0
            else None
        )
        if not baseline_format_valid:
            add_failure(None, None, "MISSING_HAND_MEASURED_BASELINE", {})
        else:
            measured_baseline_event = hand_measured_baseline_events[0]
            baseline_timestamp = _event_timestamp(measured_baseline_event)
            baseline_q = _vector(
                _payload(measured_baseline_event), "q_rad", HAND_DIM
            )
            baseline_manifest_timestamp_valid = (
                manifest.get("hand_measured_baseline_timestamp_ns")
                == baseline_timestamp
            )
            baseline_temporal_valid = bool(
                first_hand_action_timestamp is not None
                and baseline_timestamp is not None
                and baseline_timestamp < first_hand_action_timestamp
            )
            if not baseline_temporal_valid:
                add_failure(
                    0 if n else None,
                    canonical_ids[0] if canonical_ids else None,
                    "HAND_BASELINE_AFTER_ACTION",
                    {
                        "baseline_timestamp_ns": baseline_timestamp,
                        "first_hand_action_timestamp_ns": first_hand_action_timestamp,
                    },
                )
        baseline_valid = bool(
            baseline_format_valid
            and baseline_manifest_timestamp_valid
            and baseline_temporal_valid
        )
    elif legacy_baseline:
        add_failure(None, None, "MISSING_HAND_PROXY_BASELINE", {"legacy_episode": True})
    elif not baseline_declared or not baseline_events:
        add_failure(None, None, "MISSING_HAND_PROXY_BASELINE", {})
        baseline_status = "NOT SERIALIZED"
    elif len(baseline_events) != 1:
        add_failure(
            None,
            None,
            "INVALID_HAND_PROXY_BASELINE",
            {"baseline_event_count": len(baseline_events)},
        )
    else:
        baseline_event = baseline_events[0]
        baseline_payload = _payload(baseline_event)
        baseline_q = _vector(baseline_payload, "hand_qcmd", HAND_DIM)
        baseline_timestamp = _event_timestamp(baseline_event)
        try:
            baseline_sequence = int(baseline_event["sequence_id"])
        except (KeyError, TypeError, ValueError):
            baseline_sequence = 0
        baseline_format_valid = bool(
            baseline_q is not None
            and np.all(np.isfinite(baseline_q))
            and baseline_timestamp is not None
            and baseline_timestamp > 0
            and baseline_sequence > 0
            and baseline_payload.get("source") == "final_teleop_hand_qcmd"
            and baseline_payload.get("measured") is False
            and baseline_payload.get("hardware_feedback") is False
            and baseline_payload.get("hardware_applied") is False
        )
        if not baseline_format_valid:
            add_failure(
                None,
                None,
                "INVALID_HAND_PROXY_BASELINE",
                {"baseline_timestamp_ns": baseline_timestamp},
            )
        else:
            baseline_manifest_timestamp_valid = bool(
                manifest.get("hand_proxy_baseline_timestamp_ns")
                == baseline_timestamp
            )
            if not baseline_manifest_timestamp_valid:
                add_failure(
                    None,
                    None,
                    "INVALID_HAND_PROXY_BASELINE",
                    {
                        "raw_baseline_timestamp_ns": baseline_timestamp,
                        "manifest_baseline_timestamp_ns": manifest.get(
                            "hand_proxy_baseline_timestamp_ns"
                        ),
                    },
                )
            first_hand_action_timestamp = (
                int(arrays["hand_command_applied_timestamp_ns"][0])
                if timestamp_valid and n > 0
                else None
            )
            baseline_temporal_valid = bool(
                first_hand_action_timestamp is not None
                and baseline_timestamp < first_hand_action_timestamp
            )
            if not baseline_temporal_valid:
                add_failure(
                    0 if n else None,
                    canonical_ids[0] if canonical_ids else None,
                    "HAND_BASELINE_AFTER_ACTION",
                    {
                        "baseline_timestamp_ns": baseline_timestamp,
                        "first_hand_action_timestamp_ns": first_hand_action_timestamp,
                    },
                )
            else:
                baseline_payload_for_history = dict(baseline_payload)
                baseline_payload_for_history["q_rad"] = baseline_payload_for_history[
                    "hand_qcmd"
                ]
                hand_history.append(
                    (int(baseline_timestamp), -1, baseline_payload_for_history)
                )
                hand_history.sort(key=lambda item: (item[0], item[1]))

    if not hand_measured_episode:
        # Proxy episodes only: the serialized command proxy must equal
        # the first canonical hand state.  Measured episodes derive
        # state_hand from MEASURED feedback, so this equality check
        # (the frozen V1 baseline-replay bug) does not apply.
        if (
            baseline_format_valid
            and baseline_temporal_valid
            and shape_valid
            and n > 0
            and baseline_q is not None
        ):
            baseline_state_match = bool(
                np.allclose(
                    arrays["state"][0, HAND_SLICE],
                    baseline_q,
                    rtol=0.0,
                    atol=atol,
                    equal_nan=False,
                )
            )
            if not baseline_state_match:
                add_failure(
                    0,
                    canonical_ids[0] if canonical_ids else None,
                    "HAND_BASELINE_STATE_MISMATCH",
                    {"baseline_timestamp_ns": baseline_timestamp},
                )
            else:
                baseline_status = "SERIALIZED COMMAND PROXY BASELINE"

        baseline_valid = bool(
            baseline_declared
            and baseline_format_valid
            and baseline_manifest_timestamp_valid
            and baseline_temporal_valid
            and baseline_state_match is True
        )

    for event in rejection_events:
        payload = _payload(event)
        add_failure(
            None,
            _bundle_id(payload),
            f"STORED_REJECTION:{payload.get('reason', 'UNKNOWN')}",
            {"rejection_timestamp_ns": _event_timestamp(event)},
        )

    nonfinite_indices = {name: [] for name in required_vectors}
    if shape_valid:
        for name in required_vectors:
            nonfinite_indices[name] = _array_indices(
                np.any(~np.isfinite(arrays[name]), axis=1)
            )
            reason = f"NONFINITE_{name.upper()}"
            for index in nonfinite_indices[name]:
                bundle_id = canonical_ids[index] if bundle_array_valid else None
                add_failure(index, bundle_id, reason, {})

    if shape_valid and timestamp_valid and bundle_array_valid:
        for index in range(n):
            bundle_id = canonical_ids[index]
            state_ts = int(arrays["state_timestamp_ns"][index])
            decision_ts = int(arrays["decision_timestamp_ns"][index])
            arm_ts = int(arrays["arm_command_sent_timestamp_ns"][index])
            hand_ts = int(arrays["hand_command_applied_timestamp_ns"][index])
            effective_ts = int(arrays["action_effective_timestamp_ns"][index])
            next_ts = int(arrays["next_state_timestamp_ns"][index])
            timestamps = {
                "state_timestamp_ns": state_ts,
                "decision_timestamp_ns": decision_ts,
                "arm_command_sent_timestamp_ns": arm_ts,
                "hand_command_applied_timestamp_ns": hand_ts,
                "action_effective_timestamp_ns": effective_ts,
                "next_state_timestamp_ns": next_ts,
            }
            reasons: list[str] = []
            if state_ts >= effective_ts:
                reasons.append("STATE_AFTER_ACTION")
            if effective_ts >= next_ts:
                reasons.append("NEXT_STATE_BEFORE_ACTION")
            if min(state_ts, decision_ts, arm_ts, hand_ts, effective_ts, next_ts) <= 0:
                reasons.append("NON_MONOTONIC_TIMESTAMP")
            expected_effective = max(arm_ts, hand_ts)
            effective_match = effective_ts == expected_effective
            if not effective_match:
                reasons.append("ACTION_EFFECTIVE_TIMESTAMP_MISMATCH")
            if bundle_id in duplicate_ids:
                reasons.append("DUPLICATE_BUNDLE_ID")

            arm_candidates = arm_exact.get((bundle_id, arm_ts), [])
            hand_candidates = hand_exact.get((bundle_id, hand_ts), [])
            arm_event = arm_candidates[0] if len(arm_candidates) == 1 else None
            hand_event = hand_candidates[0] if len(hand_candidates) == 1 else None
            arm_present = arm_event is not None
            hand_present = hand_event is not None
            if not arm_present:
                reasons.append("MISSING_OR_AMBIGUOUS_ARM_COMMAND")
            if not hand_present:
                reasons.append("MISSING_OR_AMBIGUOUS_HAND_COMMAND")

            arm_raw_match: bool | None = None
            if arm_event is not None:
                raw_arm = _vector(arm_event, "q_rad", ARM_DIM)
                arm_raw_match = raw_arm is not None and bool(
                    np.allclose(
                        arrays["action"][index, ARM_SLICE],
                        raw_arm,
                        rtol=0.0,
                        atol=atol,
                        equal_nan=False,
                    )
                )
                if not arm_raw_match:
                    reasons.append("ARM_CANONICAL_RAW_MISMATCH")

            hand_action_match: bool | None = None
            control_decision_accepted: bool | str = NOT_VERIFIABLE
            if hand_event is not None:
                raw_hand = _vector(hand_event, "q_rad", HAND_DIM)
                hand_action_match = raw_hand is not None and bool(
                    np.allclose(
                        arrays["action"][index, HAND_SLICE],
                        raw_hand,
                        rtol=0.0,
                        atol=atol,
                        equal_nan=False,
                    )
                )
                if not hand_action_match:
                    reasons.append("HAND_CANONICAL_RAW_MISMATCH")
                required_flags = {
                    "tracking_valid": True,
                    "safety_ok": True,
                    "ik_accepted": True,
                    "stale_target": False,
                    "hand_command_valid": True,
                }
                if not all(name in hand_event for name in required_flags):
                    reasons.append("CONTROL_DECISION_FLAGS_NOT_VERIFIABLE")
                else:
                    mismatched_flags = [
                        name
                        for name, expected in required_flags.items()
                        if bool(hand_event[name]) is not expected
                    ]
                    control_decision_accepted = not mismatched_flags
                    for name in mismatched_flags:
                        reasons.append(f"ACCEPTED_{name.upper()}_VIOLATION")

            def feedback_match(timestamp: int, canonical: np.ndarray) -> bool | None:
                candidates = feedback_exact.get(timestamp, [])
                if len(candidates) != 1:
                    return None
                raw_q = _vector(candidates[0], "q_rad", ARM_DIM)
                return raw_q is not None and bool(
                    np.allclose(canonical, raw_q, rtol=0.0, atol=atol, equal_nan=False)
                )

            state_raw_match = feedback_match(
                state_ts, arrays["state"][index, ARM_SLICE]
            )
            next_raw_match = feedback_match(
                next_ts, arrays["next_state"][index, ARM_SLICE]
            )
            if state_raw_match is not True:
                reasons.append(
                    "STATE_FEEDBACK_NOT_VERIFIABLE"
                    if state_raw_match is None
                    else "STATE_FEEDBACK_RAW_MISMATCH"
                )
            if next_raw_match is not True:
                reasons.append(
                    "NEXT_FEEDBACK_NOT_VERIFIABLE"
                    if next_raw_match is None
                    else "NEXT_FEEDBACK_RAW_MISMATCH"
                )

            state_hand_match: bool | None
            next_hand_match: bool | None = None
            if hand_measured_episode:
                # Measured semantics: state_hand = latest measured
                # feedback strictly BEFORE the action; next_state_hand
                # = first measured feedback strictly AFTER it.
                earlier_state_feedback = [
                    item
                    for item in hand_feedback_timeline
                    if item[0] < effective_ts
                ]
                later_state_feedback = [
                    item
                    for item in hand_feedback_timeline
                    if item[0] > effective_ts
                ]
                if earlier_state_feedback:
                    raw_state_hand = _vector(
                        earlier_state_feedback[-1][1], "q_rad", HAND_DIM
                    )
                    state_hand_match = raw_state_hand is not None and bool(
                        np.allclose(
                            arrays["state"][index, HAND_SLICE],
                            raw_state_hand,
                            rtol=0.0,
                            atol=atol,
                            equal_nan=False,
                        )
                    )
                    if not state_hand_match:
                        reasons.append("HAND_STATE_MEASURED_MISMATCH")
                else:
                    state_hand_match = None
                    reasons.append("HAND_STATE_MEASURED_NOT_VERIFIABLE")

                if later_state_feedback:
                    raw_next_hand = _vector(
                        later_state_feedback[0][1], "q_rad", HAND_DIM
                    )
                    next_hand_match = raw_next_hand is not None and bool(
                        np.allclose(
                            arrays["next_state"][index, HAND_SLICE],
                            raw_next_hand,
                            rtol=0.0,
                            atol=atol,
                            equal_nan=False,
                        )
                    )
                    if not next_hand_match:
                        reasons.append("HAND_NEXT_STATE_MEASURED_MISMATCH")
                else:
                    reasons.append("HAND_NEXT_STATE_MEASURED_NOT_VERIFIABLE")
            else:
                earlier_state_hands = [item for item in hand_history if item[0] <= state_ts]
                earlier_next_hands = [item for item in hand_history if item[0] <= next_ts]
                if earlier_state_hands:
                    raw_state_hand = _vector(earlier_state_hands[-1][2], "q_rad", HAND_DIM)
                    state_hand_match = raw_state_hand is not None and bool(
                        np.allclose(
                            arrays["state"][index, HAND_SLICE],
                            raw_state_hand,
                            rtol=0.0,
                            atol=atol,
                            equal_nan=False,
                        )
                    )
                    if not state_hand_match:
                        reasons.append("HAND_STATE_PROXY_MISMATCH")
                else:
                    state_hand_match = None

                if earlier_next_hands:
                    raw_next_hand = _vector(earlier_next_hands[-1][2], "q_rad", HAND_DIM)
                    next_hand_match = raw_next_hand is not None and bool(
                        np.allclose(
                            arrays["next_state"][index, HAND_SLICE],
                            raw_next_hand,
                            rtol=0.0,
                            atol=atol,
                            equal_nan=False,
                        )
                    )
                if next_hand_match is not True:
                    reasons.append(
                        "HAND_NEXT_PROXY_NOT_VERIFIABLE"
                        if next_hand_match is None
                        else "HAND_NEXT_PROXY_MISMATCH"
                    )

            causal = state_ts < effective_ts < next_ts
            bundle = arm_present and hand_present and effective_match and bundle_id not in duplicate_ids
            if state_raw_match is True and next_raw_match is True:
                arm_state_verification: bool | str = True
            elif state_raw_match is False or next_raw_match is False:
                arm_state_verification = False
            else:
                arm_state_verification = NOT_VERIFIABLE
            if (
                hand_action_match is True
                and next_hand_match is True
                and state_hand_match is not False
            ):
                hand_verification: bool | str = True
            elif (
                hand_action_match is False
                or next_hand_match is False
                or state_hand_match is False
            ):
                hand_verification = False
            else:
                hand_verification = NOT_VERIFIABLE
            for reason in dict.fromkeys(reasons):
                add_failure(index, bundle_id, reason, timestamps)

            state_to_action_ms.append((effective_ts - state_ts) * 1e-6)
            action_to_next_ms.append((next_ts - effective_ts) * 1e-6)
            feedback_interval_ms.append((next_ts - state_ts) * 1e-6)
            arm_to_feedback_ms.append((next_ts - arm_ts) * 1e-6)
            hand_to_feedback_ms.append((next_ts - hand_ts) * 1e-6)
            bundle_skew_ms.append(abs(arm_ts - hand_ts) * 1e-6)
            transitions.append(
                {
                    "index": index,
                    "bundle_id": bundle_id,
                    "state": {
                        "timestamp_ns": state_ts,
                        "rm65_measured": arrays["state"][index, ARM_SLICE].tolist(),
                        "sg100_proxy": arrays["state"][index, HAND_SLICE].tolist(),
                        "hand_initial_state_source": (
                            "SERIALIZED COMMAND PROXY BASELINE"
                            if index == 0 and baseline_declared
                            else "PREVIOUS COMMAND PROXY"
                        ),
                        "baseline_timestamp_ns": baseline_timestamp if index == 0 else None,
                        "baseline_qcmd": (
                            baseline_q.tolist()
                            if index == 0 and baseline_q is not None
                            else None
                        ),
                        "baseline_raw_match": (
                            baseline_state_match if index == 0 else None
                        ),
                        "measured": False,
                        "sg100_hardware_present": False,
                        "sg100_hardware_executed": False,
                    },
                    "action": {
                        "arm_send_timestamp_ns": arm_ts,
                        "hand_applied_timestamp_ns": hand_ts,
                        "effective_timestamp_ns": effective_ts,
                        "rm65_q_cmd_sent": arrays["action"][index, ARM_SLICE].tolist(),
                        "sg100_q_cmd_applied": arrays["action"][index, HAND_SLICE].tolist(),
                    },
                    "next_state": {
                        "timestamp_ns": next_ts,
                        "rm65_measured": arrays["next_state"][index, ARM_SLICE].tolist(),
                        "sg100_proxy": arrays["next_state"][index, HAND_SLICE].tolist(),
                    },
                    "timing_ms": {
                        "state_to_action": (effective_ts - state_ts) * 1e-6,
                        "arm_hand_skew": abs(arm_ts - hand_ts) * 1e-6,
                        "action_to_next_state": (next_ts - effective_ts) * 1e-6,
                    },
                    "validation": {
                        "schema": shape_valid and metadata_valid,
                        "causal": causal,
                        "arm_raw_canonical": arm_raw_match,
                        "arm_state_raw": arm_state_verification,
                        "hand_proxy": hand_verification,
                        "control_decision_accepted": control_decision_accepted,
                        "hand_state_initial_baseline_verifiable": state_hand_match,
                        "bundle": bundle,
                        "stored_valid": (
                            bool(arrays["valid_mask"][index])
                            if "valid_mask" in arrays
                            and arrays["valid_mask"].shape == (n,)
                            else NOT_RECORDED
                        ),
                        "stored_reject_reason": (
                            str(arrays["reject_reason"][index])
                            if "reject_reason" in arrays
                            and arrays["reject_reason"].shape == (n,)
                            else NOT_RECORDED
                        ),
                        "reasons": list(dict.fromkeys(reasons)),
                    },
                }
            )

    if not shape_valid:
        add_failure(None, None, "INVALID_SHAPE", {"shapes": vector_shapes})
    if not timestamp_valid:
        add_failure(None, None, "MISSING_OR_INVALID_TIMESTAMP_ARRAY", {})
    if not bundle_array_valid:
        add_failure(None, None, "MISSING_OR_INVALID_COMMAND_BUNDLE_ID", {})
    for error in load_errors:
        add_failure(None, None, "STORAGE_ERROR", {"detail": error})

    causal_valid = sum(item["validation"]["causal"] for item in transitions)
    bundle_complete_rows = sum(item["validation"]["bundle"] for item in transitions)
    arm_raw_matches = [item["validation"]["arm_raw_canonical"] for item in transitions]
    arm_state_matches = [item["validation"]["arm_state_raw"] for item in transitions]
    hand_matches = [item["validation"]["hand_proxy"] for item in transitions]

    all_bundle_ids = sorted(arm_bundle_ids | hand_bundle_ids | set(canonical_ids))
    complete_logical_bundles = sum(
        bundle_id in arm_bundle_ids and bundle_id in hand_bundle_ids
        for bundle_id in all_bundle_ids
    )
    missing_arm = sum(bundle_id not in arm_bundle_ids for bundle_id in all_bundle_ids)
    missing_hand = sum(bundle_id not in hand_bundle_ids for bundle_id in all_bundle_ids)

    stored_rejection_counts = manifest.get("rejection_counts")
    raw_rejection_counts = Counter(
        str(_payload(event).get("reason")) for event in rejection_events
    )
    rejection_consistent = (
        isinstance(stored_rejection_counts, Mapping)
        and {str(k): int(v) for k, v in stored_rejection_counts.items()}
        == dict(raw_rejection_counts)
        and manifest.get("rejection_count") == len(rejection_events)
    )
    transition_count_consistent = manifest.get("transition_count") == n

    recomputed_valid_mask = np.asarray(
        [
            item["validation"]["causal"]
            and item["validation"]["bundle"]
            and item["validation"]["arm_raw_canonical"] is True
            and item["validation"]["arm_state_raw"] is True
            and item["validation"]["hand_proxy"] is True
            and item["validation"]["control_decision_accepted"] is True
            for item in transitions
        ],
        dtype=bool,
    )
    valid_mask_consistent: bool | str = NOT_RECORDED
    if "valid_mask" in arrays:
        stored = np.asarray(arrays["valid_mask"])
        valid_mask_consistent = bool(
            stored.shape == (n,)
            and np.array_equal(stored.astype(bool), recomputed_valid_mask)
        )
    reject_reason_consistent: bool | str = NOT_RECORDED
    if "reject_reason" in arrays:
        stored = np.asarray(arrays["reject_reason"])
        reject_reason_consistent = bool(
            stored.shape == (n,)
            and all(
                (not recomputed_valid_mask[index]) or not str(stored[index]).strip()
                for index in range(n)
            )
        )
    filter_consistent = bool(
        transition_count_consistent
        and rejection_consistent
        and (valid_mask_consistent in (True, NOT_RECORDED))
        and (reject_reason_consistent in (True, NOT_RECORDED))
        and (recomputed_valid_mask.size == n)
        and bool(np.all(recomputed_valid_mask))
    )
    if not filter_consistent:
        add_failure(None, None, "STORED_FILTER_RECOMPUTE_MISMATCH", {})

    limits = _limits_from_manifest(manifest)
    range_result: str | bool
    out_of_range_indices: list[int] | str
    if limits is None or not shape_valid:
        range_result = NOT_AVAILABLE
        out_of_range_indices = NOT_AVAILABLE
    else:
        lower, upper = limits
        masks = [
            np.any((arrays[name] < lower) | (arrays[name] > upper), axis=1)
            for name in required_vectors
        ]
        out_of_range_indices = _array_indices(np.logical_or.reduce(masks))
        range_result = not bool(out_of_range_indices)
        for index in out_of_range_indices:
            add_failure(index, canonical_ids[index], "OUT_OF_RANGE", {})

    first_transition_pre_roll = bool(
        n > 0
        and len(transitions) == n
        and transitions[0]["validation"]["causal"]
        and transitions[0]["validation"]["arm_state_raw"] is True
    )
    if n > 0 and not first_transition_pre_roll:
        add_failure(0, canonical_ids[0] if canonical_ids else None, "FIRST_TRANSITION_NO_VERIFIABLE_PRE_ROLL", {})

    finite_valid = shape_valid and all(not values for values in nonfinite_indices.values())
    causal_result = timestamp_valid and len(transitions) == n and causal_valid == n
    bundle_result = (
        bundle_array_valid
        and len(transitions) == n
        and bundle_complete_rows == n
        and not duplicate_ids
    )
    arm_action_result = _aggregate_verification(arm_raw_matches)
    arm_state_result = _aggregate_verification(arm_state_matches)
    hand_proxy_result = _aggregate_verification(hand_matches)
    control_decision_result = _aggregate_verification(
        item["validation"]["control_decision_accepted"] for item in transitions
    )
    rm65_real_hardware = bool(
        manifest.get("rm65_real_hardware") is True
        or manifest.get("arm_backend") == "real"
    )
    arm_finite_valid = bool(
        shape_valid
        and all(
            np.all(np.isfinite(arrays[name][:, ARM_SLICE]))
            for name in required_vectors
        )
    )
    rm65_metadata_valid = bool(
        manifest.get("arm_state_source") == "rm65_measured_joint_feedback"
        and manifest.get("arm_action_source") == "real_joint_command_sent"
        and arm_unit == "rad"
        and action_representation == "absolute_joint_position"
    )
    rm65_real_causal_chain_pass = bool(
        rm65_real_hardware
        and n > 0
        and shape_valid
        and rm65_metadata_valid
        and causal_result
        and bundle_result
        and arm_action_result is True
        and arm_state_result is True
        and control_decision_result is True
        and arm_finite_valid
        and first_transition_pre_roll
        and not load_errors
    )
    data_contract_pass = bool(
        n > 0
        and shape_valid
        and metadata_valid
        and causal_result
        and bundle_result
        and arm_action_result is True
        and arm_state_result is True
        and hand_proxy_result is True
        and baseline_valid
        and control_decision_result is True
        and finite_valid
        and filter_consistent
        and first_transition_pre_roll
        and not load_errors
        and range_result is not False
    )
    low_dim_training_valid = bool(
        data_contract_pass and manifest.get("quarantined") is not True
    )

    camera = manifest.get("camera", {})
    if not isinstance(camera, Mapping):
        camera = {}
    sg100_hardware_present = bool(manifest.get("hand_hardware_present", False))
    head_camera_present = bool(camera.get("head_camera_present", False))
    wrist_camera_present = bool(camera.get("wrist_camera_present", False))
    rejection_counts = {
        str(key): int(value)
        for key, value in manifest.get("rejection_counts", {}).items()
    } if isinstance(manifest.get("rejection_counts"), Mapping) else {}
    skew_stats = _timing_stats(np.asarray(bundle_skew_ms))
    skew_threshold = manifest.get("command_bundle_skew_warning_ms")
    skew_warning: bool | str = "NO THRESHOLD CONFIGURED"
    if isinstance(skew_threshold, (int, float)) and skew_stats["max"] is not None:
        skew_warning = bool(skew_stats["max"] > float(skew_threshold))

    summary: dict[str, Any] = {
        "hand_proxy_baseline_serialized": baseline_declared,
        "hand_proxy_baseline_valid": baseline_valid,
        "rm65_real_hardware": rm65_real_hardware,
        "sg100_hardware_present": sg100_hardware_present,
        "head_camera_present": head_camera_present,
        "wrist_camera_present": wrist_camera_present,
        "rm65_real_causal_chain_pass": rm65_real_causal_chain_pass,
        "full_arm_hand_hardware_validated": False,
        "episode": {
            "requested_path": str(files.requested_path),
            "root": str(files.root),
            "canonical_file": str(files.canonical),
            "name": files.root.name,
            "status": manifest.get("episode_status", NOT_RECORDED),
            "task": manifest.get("task", manifest.get("task_prompt", NOT_RECORDED)),
        },
        "schema": manifest_schema or archive_schema,
        "schema_expected": SCHEMA_NAME,
        "num_transitions": n,
        "accepted": n,
        "rejected": int(manifest.get("rejection_count", len(rejection_events))),
        "accept_rate_percent": (
            100.0 * n / (n + int(manifest.get("rejection_count", 0)))
            if n + int(manifest.get("rejection_count", 0)) > 0
            else 0.0
        ),
        "shapes": {
            **vector_shapes,
            "valid": shape_valid,
            "same_transition_count": same_length,
        },
        "metadata": {
            "schema_version": manifest_schema,
            "arm_unit": arm_unit,
            "hand_unit": hand_unit,
            "action_representation": action_representation,
            "checks": metadata_checks,
            "valid": metadata_valid,
        },
        "sources": {
            "rm65_state": manifest.get("arm_state_source", NOT_RECORDED),
            "rm65_action": manifest.get("arm_action_source", NOT_RECORDED),
            "sg100_state": manifest.get("hand_state_source", NOT_RECORDED),
            "sg100_action": manifest.get("hand_action_source", NOT_RECORDED),
            "sg100_measured_feedback": bool(manifest.get("hand_feedback_available", False)),
        },
        "causality": {
            "state_before_action": sum(
                item["state"]["timestamp_ns"] < item["action"]["effective_timestamp_ns"]
                for item in transitions
            ),
            "action_before_next_state": sum(
                item["action"]["effective_timestamp_ns"] < item["next_state"]["timestamp_ns"]
                for item in transitions
            ),
            "strict_valid": causal_valid,
            "invalid": n - causal_valid if len(transitions) == n else n,
            "result": causal_result,
        },
        "first_transition": (
            {
                "bundle_id": transitions[0]["bundle_id"],
                "state_timestamp_ns": transitions[0]["state"]["timestamp_ns"],
                "arm_command_sent_timestamp_ns": transitions[0]["action"][
                    "arm_send_timestamp_ns"
                ],
                "hand_command_applied_timestamp_ns": transitions[0]["action"][
                    "hand_applied_timestamp_ns"
                ],
                "action_effective_timestamp_ns": transitions[0]["action"][
                    "effective_timestamp_ns"
                ],
                "next_state_timestamp_ns": transitions[0]["next_state"][
                    "timestamp_ns"
                ],
                "pre_roll": first_transition_pre_roll,
            }
            if transitions
            else None
        ),
        "bundles": {
            "total": len(all_bundle_ids),
            "complete": complete_logical_bundles,
            "incomplete": len(all_bundle_ids) - complete_logical_bundles,
            "canonical_complete": bundle_complete_rows,
            "duplicate_bundle_ids": duplicate_ids,
            "missing_arm_command": missing_arm,
            "missing_hand_command": missing_hand,
            "effective_timestamp_definition": "max(arm_sent, hand_applied)",
            "result": bundle_result,
        },
        "timing_ms": {
            "state_to_action": _timing_stats(np.asarray(state_to_action_ms)),
            "action_to_next_state": _timing_stats(np.asarray(action_to_next_ms)),
            "feedback_interval": _timing_stats(np.asarray(feedback_interval_ms)),
            "arm_send_to_feedback": _timing_stats(np.asarray(arm_to_feedback_ms)),
            "hand_applied_to_feedback": _timing_stats(np.asarray(hand_to_feedback_ms)),
            "command_bundle_skew": skew_stats,
            "command_bundle_skew_warning": skew_warning,
        },
        "rm65": {
            "feedback_valid": sum(
                bool(_payload(event).get("valid")) for event in feedback_events
            ),
            "commands_sent": len(arm_events),
            "canonical_actions": n,
            "canonical_action_raw_match": arm_action_result,
            "canonical_state_raw_match": arm_state_result,
            "real_hardware": rm65_real_hardware,
            "real_causal_chain_pass": rm65_real_causal_chain_pass,
        },
        "sg100": {
            "hand_commands": len(hand_events),
            "proxy_state_transitions": n,
            "measured_feedback": (
                "AVAILABLE"
                if hand_measured_episode
                else "UNAVAILABLE"
            ),
            "measured_episode": hand_measured_episode,
            "proxy_consistency": hand_proxy_result,
            "initial_proxy_baseline": baseline_status,
            "initial_proxy_baseline_serialized": baseline_declared,
            "initial_proxy_baseline_valid": baseline_valid,
            "initial_proxy_baseline_timestamp_ns": baseline_timestamp,
            "initial_proxy_baseline_raw_match": baseline_state_match,
            "hardware_present": sg100_hardware_present,
            "hardware_execution": False,
            "action_hardware_applied": bool(
                manifest.get("hand_action_hardware_applied", False)
            ),
            "hand_real_validation": (
                {
                    "feedback_samples": len(hand_feedback_events),
                    "commands_sent": len(hand_sent_events),
                    "canonical_actions": n,
                    "canonical_state_measured": bool(
                        arrays["hand_state_measured"].all()
                    )
                    if "hand_state_measured" in arrays
                    and arrays["hand_state_measured"].size
                    else False,
                    "command_raw_match": (
                        "NOT VERIFIABLE"
                        if not hand_measured_episode
                        else (
                            "PASS"
                            if not any(
                                item["reason"]
                                in (
                                    "MISSING_OR_AMBIGUOUS_HAND_COMMAND",
                                    "HAND_CANONICAL_RAW_MISMATCH",
                                )
                                for item in failures
                            )
                            else "FAIL"
                        )
                    ),
                    "feedback_raw_match": (
                        "NOT VERIFIABLE"
                        if not hand_measured_episode
                        else (
                            "PASS"
                            if not any(
                                "MEASURED_MISMATCH" in item["reason"]
                                or "MEASURED_NOT_VERIFIABLE" in item["reason"]
                                for item in failures
                            )
                            else "FAIL"
                        )
                    ),
                    "intervening_hand_commands": intervening_hand_commands,
                    "first_pre_roll": (
                        "PASS"
                        if baseline_temporal_valid and hand_measured_episode
                        else (
                            _pass_fail(first_transition_pre_roll)
                            if first_transition_pre_roll is not None
                            else "NOT VERIFIABLE"
                        )
                    ),
                }
                if hand_measured_episode
                else None
            ),
        },
        "data_quality": {
            "nonfinite_state_indices": nonfinite_indices["state"],
            "nonfinite_action_indices": nonfinite_indices["action"],
            "nonfinite_next_state_indices": nonfinite_indices["next_state"],
            "range_check": range_result,
            "out_of_range_indices": out_of_range_indices,
            "stale_pre_state": rejection_counts.get("STALE_PRE_STATE", 0),
            "stale_post_state": rejection_counts.get("STALE_POST_STATE", 0),
            "tracking_loss": rejection_counts.get("TRACKING_LOST", 0),
            "safety_fault": rejection_counts.get("SAFETY_FAULT", 0),
            "accepted_tracking_loss": sum(
                item["validation"]["control_decision_accepted"] is False
                and "ACCEPTED_TRACKING_VALID_VIOLATION"
                in item["validation"]["reasons"]
                for item in transitions
            ),
            "accepted_safety_fault": sum(
                item["validation"]["control_decision_accepted"] is False
                and "ACCEPTED_SAFETY_OK_VIOLATION"
                in item["validation"]["reasons"]
                for item in transitions
            ),
            "accepted_control_decisions": control_decision_result,
            "first_transition_pre_roll": first_transition_pre_roll,
            "filter_consistency": filter_consistent,
            "stored_valid_mask_consistency": valid_mask_consistent,
            "stored_reject_reason_consistency": reject_reason_consistent,
            "stored_rejection_counts_consistency": rejection_consistent,
            "markov_diagnostic": manifest.get("markov_diagnostic", NOT_RECORDED),
            "quarantined": bool(manifest.get("quarantined", False)),
        },
        "raw_counts": _raw_counts(events, files.raw is not None),
        "cameras": {
            "head_camera_present": head_camera_present,
            "wrist_camera_present": wrist_camera_present,
            "multimodal_training_ready": bool(
                camera.get("multimodal_training_ready", False)
            ),
        },
        "failure_count": len(failures),
        "failure_indices": sorted(
            {int(item["index"]) for item in failures if item["index"] is not None}
        ),
        "failures_preview": failures[:MAX_DEFAULT_ERRORS],
        "result": {
            "data_contract_pass": data_contract_pass,
            "low_dim_training_data_valid": low_dim_training_valid,
            "pi05_multimodal_training_ready": bool(
                low_dim_training_valid
                and camera.get("multimodal_training_ready", False)
                and camera.get("head_camera_present", False)
                and camera.get("wrist_camera_present", False)
            ),
            "rm65_real_causal_chain_pass": rm65_real_causal_chain_pass,
            "rm65_hardware_validation": rm65_real_causal_chain_pass,
            "sg100_real_causal_chain_pass": (
                bool(
                    n > 0
                    and metadata_valid
                    and causal_valid == n
                    and not any(
                        item["reason"].startswith("HAND_")
                        or item["reason"]
                        in (
                            "MISSING_OR_AMBIGUOUS_HAND_COMMAND",
                            "HAND_CANONICAL_RAW_MISMATCH",
                        )
                        for item in failures
                    )
                )
                if hand_measured_episode
                else False
            ),
            "sg100_hardware_validation": (
                "MEASURED-FEEDBACK EPISODE VALIDATED"
                if hand_measured_episode
                and n > 0
                and not failures
                else (
                    "MEASURED-FEEDBACK EPISODE WITH FAILURES"
                    if hand_measured_episode
                    else "NOT TESTED — HARDWARE NOT AVAILABLE"
                )
            ),
            "camera_hardware_validation": "NOT TESTED — HARDWARE NOT AVAILABLE",
            "full_arm_hand_hardware_validated": bool(
                rm65_real_causal_chain_pass and hand_measured_episode
            ),
        },
    }
    return InspectionReport(files, _jsonable(summary), transitions, failures)


def _pass_fail(value: Any) -> str:
    if value is True:
        return "PASS"
    if value is False:
        return "FAIL"
    return str(value)


def _yes_no(value: Any) -> str:
    return "YES" if value else "NO"


def _format_stats(stats: Mapping[str, Any]) -> str:
    if stats.get("mean") is None:
        return NOT_AVAILABLE
    return (
        f"mean={stats['mean']:.3f}  P50={stats['p50']:.3f}  "
        f"P95={stats['p95']:.3f}  P99={stats['p99']:.3f}  "
        f"max={stats['max']:.3f} ms"
    )


def format_summary(report: InspectionReport, *, show_all_errors: bool = False) -> str:
    value = report.summary
    n = value["num_transitions"]
    rejected = value["rejected"]
    lines = [
        "=" * 50,
        "ARM-HAND DATASET EPISODE INSPECTION",
        "=" * 50,
        f"Episode: {value['episode']['name']}",
        f"Path: {value['episode']['root']}",
        f"Schema: {value['schema']}",
        f"Episode status: {value['episode']['status']}",
        f"Task: {value['episode']['task']}",
        f"Transitions: accepted={n} rejected={rejected} "
        f"accept_rate={value['accept_rate_percent']:.2f}%",
        "",
        "-" * 50,
        "SHAPES",
        "-" * 50,
        f"state: {tuple(value['shapes']['state']) if value['shapes']['state'] is not None else 'MISSING'}",
        f"action: {tuple(value['shapes']['action']) if value['shapes']['action'] is not None else 'MISSING'}",
        f"next_state: {tuple(value['shapes']['next_state']) if value['shapes']['next_state'] is not None else 'MISSING'}",
        f"schema valid: {_yes_no(value['shapes']['valid'] and value['metadata']['valid'])}",
        "",
        "-" * 50,
        "HARDWARE STATUS",
        "-" * 50,
        f"RM65: {'REAL HARDWARE' if value['rm65_real_hardware'] else 'NOT VERIFIED AS REAL HARDWARE'}",
        f"SG100: {'PRESENT' if value['sg100_hardware_present'] else 'NOT PRESENT'}",
        "SG100 data source: SOFTWARE COMMAND PROXY",
        f"Head camera: {'PRESENT' if value['head_camera_present'] else 'NOT PRESENT'}",
        f"Wrist camera: {'PRESENT' if value['wrist_camera_present'] else 'NOT PRESENT'}",
        "",
        "-" * 50,
        "STATE SOURCES",
        "-" * 50,
        f"RM65 state: {value['sources']['rm65_state']}",
        f"SG100 state: {value['sources']['sg100_state']}",
        f"SG100 measured feedback: {_yes_no(value['sources']['sg100_measured_feedback'])}",
        "",
        "-" * 50,
        "CAUSALITY",
        "-" * 50,
        f"state < action: {value['causality']['state_before_action']} / {n}",
        f"action < next_state: {value['causality']['action_before_next_state']} / {n}",
        f"strict state < action < next_state: {value['causality']['strict_valid']} / {n}",
        f"non-monotonic transitions: {value['causality']['invalid']}",
        f"causal result: {_pass_fail(value['causality']['result'])}",
        "",
        "-" * 50,
        "FIRST TRANSITION",
        "-" * 50,
        (
            "NOT AVAILABLE"
            if value["first_transition"] is None
            else (
                f"bundle={value['first_transition']['bundle_id']}  "
                f"state={value['first_transition']['state_timestamp_ns']}  "
                f"arm={value['first_transition']['arm_command_sent_timestamp_ns']}  "
                f"hand={value['first_transition']['hand_command_applied_timestamp_ns']}  "
                f"effective={value['first_transition']['action_effective_timestamp_ns']}  "
                f"next={value['first_transition']['next_state_timestamp_ns']}"
            )
        ),
        "first transition pre-roll: "
        + _pass_fail(
            False
            if value["first_transition"] is None
            else value["first_transition"]["pre_roll"]
        ),
        "",
        "-" * 50,
        "COMMAND BUNDLES",
        "-" * 50,
        f"total bundles: {value['bundles']['total']}",
        f"complete bundles: {value['bundles']['complete']}",
        f"incomplete bundles: {value['bundles']['incomplete']}",
        f"duplicate bundle IDs: {value['bundles']['duplicate_bundle_ids']}",
        f"missing arm command: {value['bundles']['missing_arm_command']}",
        f"missing hand command: {value['bundles']['missing_hand_command']}",
        f"effective timestamp: {value['bundles']['effective_timestamp_definition']}",
        f"bundle result: {_pass_fail(value['bundles']['result'])}",
        "",
        "-" * 50,
        "TIMING",
        "-" * 50,
        f"state -> action: {_format_stats(value['timing_ms']['state_to_action'])}",
        f"action -> next_state: {_format_stats(value['timing_ms']['action_to_next_state'])}",
        f"feedback interval: {_format_stats(value['timing_ms']['feedback_interval'])}",
        f"arm send -> feedback: {_format_stats(value['timing_ms']['arm_send_to_feedback'])}",
        f"hand applied -> feedback: {_format_stats(value['timing_ms']['hand_applied_to_feedback'])}",
        f"arm/hand bundle skew: {_format_stats(value['timing_ms']['command_bundle_skew'])}",
        f"skew warning: {value['timing_ms']['command_bundle_skew_warning']}",
        "",
        "-" * 50,
        "RM65 REAL VALIDATION",
        "-" * 50,
        f"feedback valid: {value['rm65']['feedback_valid']}",
        f"commands sent: {value['rm65']['commands_sent']}",
        f"canonical actions: {value['rm65']['canonical_actions']}",
        f"command/action match: {_pass_fail(value['rm65']['canonical_action_raw_match'])}",
        f"canonical/raw feedback match: {_pass_fail(value['rm65']['canonical_state_raw_match'])}",
        f"first pre-roll: {_pass_fail(value['data_quality']['first_transition_pre_roll'])}",
        "RM65 REAL CAUSAL CHAIN: "
        + _pass_fail(value["rm65_real_causal_chain_pass"]),
        "",
        "-" * 50,
        "HAND SOFTWARE DATA",
        "-" * 50,
        f"hand commands: {value['sg100']['hand_commands']}",
        f"proxy-state transitions: {value['sg100']['proxy_state_transitions']}",
        f"measured feedback: {value['sg100']['measured_feedback']}",
        f"proxy consistency: {_pass_fail(value['sg100']['proxy_consistency'])}",
        f"initial proxy baseline: {value['sg100']['initial_proxy_baseline']}",
        "initial proxy baseline serialized: "
        + _yes_no(value["hand_proxy_baseline_serialized"]),
        "baseline/raw match: "
        + _pass_fail(value["sg100"]["initial_proxy_baseline_raw_match"]),
        "hardware execution: NO",
        "",
        "-" * 50,
        "RAW COUNTS",
        "-" * 50,
    ]
    lines.extend(
        f"{name.replace('_', ' ')}: {count}"
        for name, count in value["raw_counts"].items()
    )
    lines.extend(
        [
            "",
            "-" * 50,
            "DATA QUALITY",
            "-" * 50,
            f"non-finite state: {value['data_quality']['nonfinite_state_indices']}",
            f"non-finite action: {value['data_quality']['nonfinite_action_indices']}",
            f"non-finite next_state: {value['data_quality']['nonfinite_next_state_indices']}",
            f"range check: {_pass_fail(value['data_quality']['range_check'])}",
            f"stale pre-state: {value['data_quality']['stale_pre_state']}",
            f"stale post-state: {value['data_quality']['stale_post_state']}",
            f"tracking loss: {value['data_quality']['tracking_loss']}",
            f"safety fault: {value['data_quality']['safety_fault']}",
            f"accepted tracking loss: {value['data_quality']['accepted_tracking_loss']}",
            f"accepted safety fault: {value['data_quality']['accepted_safety_fault']}",
            f"Markov diagnostic: {value['data_quality']['markov_diagnostic']}",
            f"quarantined: {_yes_no(value['data_quality']['quarantined'])}",
            f"first transition pre-roll: {_pass_fail(value['data_quality']['first_transition_pre_roll'])}",
            f"filter consistency: {_pass_fail(value['data_quality']['filter_consistency'])}",
            "",
            "-" * 50,
            "CAMERAS",
            "-" * 50,
            f"head camera: {'AVAILABLE' if value['cameras']['head_camera_present'] else 'NOT AVAILABLE'}",
            f"wrist camera: {'AVAILABLE' if value['cameras']['wrist_camera_present'] else 'NOT AVAILABLE'}",
            f"multimodal training ready: {_yes_no(value['cameras']['multimodal_training_ready'])}",
        ]
    )
    hand_validation = value["sg100"].get("hand_real_validation")
    if hand_validation is not None:
        lines.extend(
            [
                "",
                "-" * 50,
                "HAND REAL VALIDATION (SG100 measured feedback)",
                "-" * 50,
                f"feedback samples     : {hand_validation['feedback_samples']}",
                f"commands sent        : {hand_validation['commands_sent']}",
                f"canonical actions    : {hand_validation['canonical_actions']}",
                "canonical state measured: "
                + _yes_no(hand_validation["canonical_state_measured"]),
                "command/raw match    : "
                + str(hand_validation["command_raw_match"]),
                "feedback/raw match   : "
                + str(hand_validation["feedback_raw_match"]),
                "intervening command  : "
                f"{hand_validation['intervening_hand_commands']}",
                "first pre-roll       : "
                + str(hand_validation["first_pre_roll"]),
                "SG100 REAL CAUSAL CHAIN: "
                + _pass_fail(value["result"]["sg100_real_causal_chain_pass"]),
            ]
        )
    elif value["sg100"].get("measured_episode"):
        lines.extend(
            [
                "",
                "HAND REAL VALIDATION: no transitions in episode",
            ]
        )
    else:
        lines.extend(
            [
                "",
                "HAND REAL VALIDATION: NOT APPLICABLE — "
                "LEGACY HAND PROXY EPISODE "
                "(command proxy, hand_hardware_present=false)",
            ]
        )
    if report.failures:
        selected = report.failures if show_all_errors else report.failures[:MAX_DEFAULT_ERRORS]
        lines.extend(["", "-" * 50, "FAILURES", "-" * 50])
        for item in selected:
            lines.append(
                f"index={item['index']} bundle={item['bundle_id']} reason={item['reason']}"
            )
        if not show_all_errors and len(report.failures) > len(selected):
            lines.append(
                f"... {len(report.failures) - len(selected)} more; use --show-all-errors"
            )
    lines.extend(
        [
            "",
            "-" * 50,
            "FULL SYSTEM",
            "-" * 50,
            f"DATA CONTRACT V1: {'PASS' if value['result']['data_contract_pass'] else 'FAIL'}",
            "RM65 HARDWARE VALIDATION: "
            + _pass_fail(value["result"]["rm65_hardware_validation"]),
            "SG100 HARDWARE VALIDATION: "
            + value["result"]["sg100_hardware_validation"],
            "CAMERA HARDWARE VALIDATION: "
            + value["result"]["camera_hardware_validation"],
            "LOW-DIM ROBOT TRAJECTORY: "
            + ("VALID" if value['result']['low_dim_training_data_valid'] else "INVALID"),
            "PI0.5 MULTIMODAL TRAINING READY: "
            + _yes_no(value['result']['pi05_multimodal_training_ready']),
            "=" * 50,
        ]
    )
    return "\n".join(lines)


def format_transition(value: Mapping[str, Any]) -> str:
    validation = value["validation"]
    return "\n".join(
        [
            "=" * 50,
            f"TRANSITION {value['index']}",
            "=" * 50,
            f"bundle_id: {value['bundle_id']}",
            "",
            "STATE",
            f"timestamp_ns: {value['state']['timestamp_ns']}",
            f"RM65 measured: {value['state']['rm65_measured']}",
            f"SG100 proxy: {value['state']['sg100_proxy']}",
            f"HAND INITIAL STATE SOURCE: {value['state']['hand_initial_state_source']}",
            f"baseline timestamp: {value['state']['baseline_timestamp_ns']}",
            f"baseline qcmd: {value['state']['baseline_qcmd']}",
            f"canonical state hand: {value['state']['sg100_proxy']}",
            f"baseline/raw match: {_pass_fail(value['state']['baseline_raw_match'])}",
            f"measured: {_yes_no(value['state']['measured'])}",
            f"SG100 hardware present: {_yes_no(value['state']['sg100_hardware_present'])}",
            f"SG100 hardware executed: {_yes_no(value['state']['sg100_hardware_executed'])}",
            "",
            "ACTION",
            f"arm_send_timestamp_ns: {value['action']['arm_send_timestamp_ns']}",
            f"hand_applied_timestamp_ns: {value['action']['hand_applied_timestamp_ns']}",
            f"effective_timestamp_ns: {value['action']['effective_timestamp_ns']}",
            f"RM65 q_cmd_sent: {value['action']['rm65_q_cmd_sent']}",
            f"SG100 final teleop software qcmd: {value['action']['sg100_q_cmd_applied']}",
            "",
            "NEXT STATE",
            f"timestamp_ns: {value['next_state']['timestamp_ns']}",
            f"RM65 measured: {value['next_state']['rm65_measured']}",
            f"SG100 proxy: {value['next_state']['sg100_proxy']}",
            "",
            "TIMING",
            f"state -> action: {value['timing_ms']['state_to_action']:.6f} ms",
            f"arm/hand skew: {value['timing_ms']['arm_hand_skew']:.6f} ms",
            f"action -> next_state: {value['timing_ms']['action_to_next_state']:.6f} ms",
            "",
            "VALIDATION",
            f"schema: {_pass_fail(validation['schema'])}",
            f"causal: {_pass_fail(validation['causal'])}",
            f"arm raw/canonical: {_pass_fail(validation['arm_raw_canonical'])}",
            f"arm feedback raw/canonical: {_pass_fail(validation['arm_state_raw'])}",
            f"hand proxy: {_pass_fail(validation['hand_proxy'])}",
            f"bundle: {_pass_fail(validation['bundle'])}",
            f"stored valid: {validation['stored_valid']}",
            f"stored reject reason: {validation['stored_reject_reason']}",
            f"reasons: {validation['reasons']}",
        ]
    )


def format_failures(failures: Iterable[Mapping[str, Any]]) -> str:
    lines = ["=" * 50, "FAILED / REJECTED TRANSITIONS", "=" * 50]
    failures = list(failures)
    if not failures:
        lines.append("none")
    for item in failures:
        lines.append(
            f"index={item['index']} bundle={item['bundle_id']} "
            f"reason={item['reason']} timestamps={item['timestamps']}"
        )
    return "\n".join(lines)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inspect causal RM65 + SG100 DATA CONTRACT V1 episodes"
    )
    parser.add_argument("episode", help="episode directory, recording session, or canonical .npz")
    parser.add_argument("--dump-transition", type=int, metavar="INDEX")
    parser.add_argument("--dump-failures", action="store_true")
    parser.add_argument("--show-all-errors", action="store_true")
    parser.add_argument("--json", action="store_true", dest="json_output")
    parser.add_argument(
        "--all",
        action="store_true",
        help="scan every arm_hand_joint17_v1.npz below a dataset directory",
    )
    return parser


def _batch_payload(reports: list[InspectionReport]) -> list[dict[str, Any]]:
    return [
        {
            "episode": report.summary["episode"]["root"],
            "status": report.summary["episode"]["status"],
            "transitions": report.summary["num_transitions"],
            "causal_percent": (
                100.0
                * report.summary["causality"]["strict_valid"]
                / report.summary["num_transitions"]
                if report.summary["num_transitions"]
                else 0.0
            ),
            "bundle_percent": (
                100.0
                * report.summary["bundles"]["canonical_complete"]
                / report.summary["num_transitions"]
                if report.summary["num_transitions"]
                else 0.0
            ),
            "data_contract_pass": report.passed,
            "multimodal_ready": report.summary["result"][
                "pi05_multimodal_training_ready"
            ],
        }
        for report in reports
    ]


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    try:
        if args.all:
            roots = discover_episodes(args.episode)
            if not roots:
                raise FileNotFoundError(
                    f"no {CANONICAL_FILENAME} found below {args.episode}"
                )
            reports = [inspect_episode(root) for root in roots]
            payload = _batch_payload(reports)
            if args.json_output:
                print(json.dumps(_jsonable(payload), indent=2, sort_keys=True))
            else:
                print("episode\tstatus\ttransitions\tcausal%\tbundle%\tcontract\tmultimodal")
                for item in payload:
                    print(
                        f"{item['episode']}\t{item['status']}\t{item['transitions']}\t"
                        f"{item['causal_percent']:.2f}\t{item['bundle_percent']:.2f}\t"
                        f"{'PASS' if item['data_contract_pass'] else 'FAIL'}\t"
                        f"{'YES' if item['multimodal_ready'] else 'NO'}"
                    )
            return 0 if all(report.passed for report in reports) else 1

        report = inspect_episode(args.episode)
        if args.json_output:
            print(
                json.dumps(
                    report.json_payload(
                        dump_transition=args.dump_transition,
                        dump_failures=args.dump_failures,
                    ),
                    indent=2,
                    sort_keys=True,
                )
            )
        else:
            print(format_summary(report, show_all_errors=args.show_all_errors))
            if args.dump_transition is not None:
                print()
                print(format_transition(report.transition(args.dump_transition)))
            if args.dump_failures:
                print()
                print(format_failures(report.failures))
        return 0 if report.passed else 1
    except (FileNotFoundError, ValueError, IndexError) as exc:
        print(f"inspection error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
