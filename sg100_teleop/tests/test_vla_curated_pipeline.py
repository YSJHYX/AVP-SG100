"""RAW → Curated 数据管线纯软件测试；不得连接任何硬件。"""

from __future__ import annotations

import json
from pathlib import Path
import re

import numpy as np
import pytest

from sg100_teleop.data_collection.build_curated_episode import (
    build_arg_parser as build_builder_arg_parser,
    build_curated_episode,
)
from sg100_teleop.data_collection.export_replay_commands import (
    export_replay_commands,
    load_replay_commands,
)
from sg100_teleop.data_collection.inspect_vla_episode import (
    inspect_curated_episode,
    inspect_vla_episode,
)
from sg100_teleop.data_collection.vla_curated_validator import (
    validate_curated_episode,
)
from sg100_teleop.data_collection.vla_curated_schema import (
    ARM_SLICE,
    CURATED_SCHEMA_NAME,
    DEFAULT_CURATED_DIR,
    DEFAULT_MODEL_READY_DIR,
    HAND_SLICE,
    FORBIDDEN_TRAJECTORY_KEYS,
    TRAJECTORY_CORE_KEYS,
)
from sg100_teleop.data_collection.vla_raw_schema import (
    ROBOT_JOINT_NAMES,
    SCHEMA_NAME as RAW_SCHEMA_NAME,
)
from sg100_teleop.data_collection.vla_raw_validator import validate_episode
from sg100_teleop.data_collection.vla_raw_writer import save_episode_atomic
from sg100_teleop.data_collection.vla_timing_metrics import (
    action_next_state_dt_ns,
    arm_hand_action_skew_ns,
    arm_hand_state_skew_ns,
    desired_effective_summary,
    sequence_rate_hz,
    state_action_dt_ns,
    summary_stats_ns,
)
from sg100_teleop.data_collection.run_vla_raw_recorder import (
    build_arg_parser as build_recorder_arg_parser,
)

README_PATH = Path(__file__).resolve().parents[1] / "README.md"
PERIOD_NS = 1_000_000_000 // 30
START_NS = 10_000_000_000_000


def _write_jpgs(directory: Path, count: int, skip: tuple[int, ...] = ()) -> None:
    import cv2

    directory.mkdir(parents=True, exist_ok=True)
    image = np.full((4, 4, 3), 127, dtype=np.uint8)
    for index in range(count):
        if index in skip:
            continue
        cv2.imwrite(str(directory / f"{index:06d}.jpg"), image)


def _camera_metadata(prefix: str, *, present: bool, model: str) -> dict:
    if not present:
        return {
            f"{prefix}_camera_present": np.asarray(False, dtype=bool),
            f"{prefix}_depth_present": np.asarray(False, dtype=bool),
            f"{prefix}_camera_model": np.asarray(""),
            f"{prefix}_camera_serial": np.asarray(""),
            f"{prefix}_camera_width": np.asarray(0, dtype=np.int32),
            f"{prefix}_camera_height": np.asarray(0, dtype=np.int32),
            f"{prefix}_camera_fps": np.asarray(0, dtype=np.int32),
            f"{prefix}_depth_scale": np.asarray(np.nan, dtype=np.float64),
            f"{prefix}_color_intrinsics": np.full((3, 3), np.nan),
            f"{prefix}_depth_intrinsics": np.full((3, 3), np.nan),
            f"{prefix}_color_format": np.asarray(""),
            f"{prefix}_depth_format": np.asarray(""),
            f"{prefix}_color_stream_profile": np.asarray(""),
            f"{prefix}_depth_stream_profile": np.asarray(""),
            f"{prefix}_depth_aligned_to": np.asarray(""),
            f"{prefix}_extrinsics_to_robot": np.asarray(
                "UNAVAILABLE / NOT_CALIBRATED"
            ),
        }
    return {
        f"{prefix}_camera_present": np.asarray(True, dtype=bool),
        f"{prefix}_depth_present": np.asarray(False, dtype=bool),
        f"{prefix}_camera_model": np.asarray(model),
        f"{prefix}_camera_serial": np.asarray(f"serial-{prefix}"),
        f"{prefix}_camera_width": np.asarray(640, dtype=np.int32),
        f"{prefix}_camera_height": np.asarray(480, dtype=np.int32),
        f"{prefix}_camera_fps": np.asarray(30, dtype=np.int32),
        f"{prefix}_depth_scale": np.asarray(np.nan, dtype=np.float64),
        f"{prefix}_color_intrinsics": np.eye(3),
        f"{prefix}_depth_intrinsics": np.full((3, 3), np.nan),
        f"{prefix}_color_format": np.asarray("RGB8"),
        f"{prefix}_depth_format": np.asarray(""),
        f"{prefix}_color_stream_profile": np.asarray("RGB8 640x480x30"),
        f"{prefix}_depth_stream_profile": np.asarray(""),
        f"{prefix}_depth_aligned_to": np.asarray("DISABLED"),
        f"{prefix}_extrinsics_to_robot": np.asarray(
            "UNAVAILABLE / NOT_CALIBRATED"
        ),
    }


def _full_v3_arrays(
    n: int = 8,
    *,
    wrist: bool = False,
    qpos_invalid: tuple[int, ...] = (),
    qcmd_invalid: tuple[int, ...] = (),
    desired_invalid: tuple[int, ...] = (),
    head_invalid: tuple[int, ...] = (),
    wrist_invalid: tuple[int, ...] = (),
    estop_ticks: tuple[int, ...] = (),
    tracking_invalid_ticks: tuple[int, ...] = (),
    telemetry_invalid_ticks: tuple[int, ...] = (),
) -> dict[str, np.ndarray]:
    """构造 validator-clean 的 v3 RAW 数组（RGB-only，30 Hz，全 causal）。

    component-wise causality：qpos source ts (t-9/-8ms) <= qcmd source ts
    (t-5/-4ms) < next qpos source ts (t+period-9/-8ms)。
    """
    tick = START_NS + (np.arange(n) * 1_000_000_000) // 30
    k = np.arange(n)
    arm_qpos = 0.10 + 0.05 * k[:, None] + 0.01 * np.arange(6)[None, :]
    hand_qpos = 0.20 + 0.04 * k[:, None] + 0.01 * np.arange(11)[None, :]
    arm_goal = 0.30 + 0.03 * k[:, None] + 0.01 * np.arange(6)[None, :]
    hand_goal = 0.40 + 0.02 * k[:, None] + 0.01 * np.arange(11)[None, :]
    arm_sent = arm_goal + 0.50
    hand_effective = hand_goal + 0.25
    qpos17 = np.concatenate((arm_qpos, hand_qpos), axis=1)
    qcmd17 = np.concatenate((arm_sent, hand_effective), axis=1)
    desired17 = np.concatenate((arm_goal, hand_goal), axis=1)

    arm_fb_valid = np.ones(n, dtype=bool)
    hand_canonical_valid = np.ones(n, dtype=bool)
    arm_cmd_valid = np.ones(n, dtype=bool)
    hand_cmd_valid = np.ones(n, dtype=bool)
    desired_valid = np.ones(n, dtype=bool)
    head_valid = np.ones(n, dtype=bool)
    wrist_valid_arr = np.ones(n, dtype=bool) if wrist else np.zeros(n, dtype=bool)

    for index in qpos_invalid:
        arm_fb_valid[index] = False
        hand_canonical_valid[index] = False
        arm_qpos[index] = np.nan
        hand_qpos[index] = np.nan
        qpos17[index] = np.nan
    for index in qcmd_invalid:
        arm_cmd_valid[index] = False
        hand_cmd_valid[index] = False
        arm_sent[index] = np.nan
        hand_effective[index] = np.nan
        qcmd17[index] = np.nan
    for index in desired_invalid:
        desired_valid[index] = False
        arm_goal[index] = np.nan
        hand_goal[index] = np.nan
        desired17[index] = np.nan
    for index in head_invalid:
        head_valid[index] = False
    for index in wrist_invalid:
        wrist_valid_arr[index] = False

    arm_qpos_ts = tick - 9_000_000
    hand_fb_ts = tick - 8_000_000
    arm_cmd_ts = tick - 5_000_000
    hand_cmd_ts = tick - 4_000_000
    camera_ts = tick - 2_000_000
    qpos17_valid = arm_fb_valid & hand_canonical_valid
    qcmd17_valid = arm_cmd_valid & hand_cmd_valid

    arrays: dict[str, np.ndarray] = {
        "schema_name": np.asarray(RAW_SCHEMA_NAME),
        "schema_version": np.asarray(3, dtype=np.int32),
        "episode_index": np.asarray(0, dtype=np.int64),
        "language_instruction": np.asarray("pick and place"),
        "dataset_hz": np.asarray(30.0, dtype=np.float64),
        "episode_start_monotonic_ns": np.asarray(int(tick[0]), dtype=np.int64),
        "episode_start_wall_ns": np.asarray(int(tick[0]), dtype=np.int64),
        "episode_stop_request_monotonic_ns": np.asarray(
            int(tick[-1] + PERIOD_NS), dtype=np.int64
        ),
        "robot_joint_names": np.asarray(ROBOT_JOINT_NAMES),
        "robot_desired_qcmd_semantics": np.asarray("desired before safety"),
        "robot_effective_qcmd_semantics": np.asarray("effective backend"),
        "robot_qpos_semantics": np.asarray("measured feedback"),
        "dataset_status": np.asarray("RAW_CAPTURE_QUARANTINED"),
        "training_ready": np.asarray(False, dtype=bool),
        "hardware_execution": np.asarray(False, dtype=bool),
        "robot_telemetry_present": np.asarray(True, dtype=bool),
        "required_cameras": np.asarray(
            ("head", "right_wrist") if wrist else ("head",)
        ),
        "camera_modalities": np.asarray("rgb"),
        "record_depth": np.asarray(False, dtype=bool),
        "camera_configuration_mode": np.asarray(
            "DUAL_PRODUCTION" if wrist else "HEAD_ONLY_BRINGUP"
        ),
        "dual_camera_synchronization": np.asarray(
            "HOST MONOTONIC SOFTWARE ALIGNMENT" if wrist else "NOT_APPLICABLE_HEAD_ONLY"
        ),
        "camera_device_clock_domains_shared": np.asarray(False, dtype=bool),
        "sg100_feedback_map_status": np.asarray("F_FB_PENDING"),
        "media_writer_max_backlog": np.asarray(0, dtype=np.int64),
        "media_writer_drop_count": np.asarray(0, dtype=np.int64),
        "media_writer_error_count": np.asarray(0, dtype=np.int64),
        "telemetry_received_packets": np.asarray(10 * n, dtype=np.int64),
        "telemetry_sequence_gap_count": np.asarray(0, dtype=np.int64),
        "telemetry_publish_hz_requested": np.asarray(100.0, dtype=np.float64),
        "telemetry_publish_hz_effective": np.asarray(90.0, dtype=np.float64),
        "validation_quality": np.asarray("PASS"),
        "max_arm_command_age_ns": np.asarray(100_000_000, dtype=np.int64),
        "max_arm_feedback_age_ns": np.asarray(100_000_000, dtype=np.int64),
        "max_hand_command_age_ns": np.asarray(100_000_000, dtype=np.int64),
        "max_hand_feedback_age_ns": np.asarray(100_000_000, dtype=np.int64),
        "max_camera_age_ns": np.asarray(100_000_000, dtype=np.int64),
        "max_telemetry_age_ns": np.asarray(100_000_000, dtype=np.int64),
        "timestamp_ns": tick.astype(np.int64),
        "timestamp_relative_ns": (tick - START_NS).astype(np.int64),
        "sampler_wakeup_timestamp_ns": (tick + 1000 + k).astype(np.int64),
        "sampler_jitter_ns": (1000 + k).astype(np.int64),
        "arm_q_goal_rad": arm_goal,
        "arm_qcmd_sent_rad": arm_sent,
        "arm_qcmd_source_timestamp_ns": arm_cmd_ts,
        "arm_qcmd_age_ns": (tick - arm_cmd_ts).astype(np.int64),
        "arm_qcmd_sequence": (3 * k).astype(np.int64),
        "arm_command_valid": arm_cmd_valid,
        "arm_qpos_rad": arm_qpos,
        "arm_qpos_source_timestamp_ns": arm_qpos_ts,
        "arm_qpos_age_ns": (tick - arm_qpos_ts).astype(np.int64),
        "arm_feedback_sequence": (2 * k).astype(np.int64),
        "arm_feedback_valid": arm_fb_valid,
        "arm_controller_error": np.zeros(n, dtype=np.int64),
        "arm_status": np.ones(n, dtype=np.int64),
        "arm_feedback_channel": np.asarray(["TCP_POLL"] * n),
        "arm_sender_hz": np.full(n, 100.0),
        "hand_qcmd_retargeted_canonical_rad": hand_goal,
        "hand_qcmd_effective_canonical_rad": hand_effective,
        "hand_qcmd_sdk_deg": hand_goal * 180.0 / np.pi + 90.0,
        "hand_qcmd_source_timestamp_ns": hand_cmd_ts,
        "hand_qcmd_age_ns": (tick - hand_cmd_ts).astype(np.int64),
        "hand_command_sequence": k.astype(np.int64),
        "hand_command_valid": hand_cmd_valid,
        "hand_feedback_sdk_rad": hand_qpos + 1.0,
        "hand_feedback_modes": np.full((n, 11), 7, dtype=np.int16),
        "hand_feedback_source_timestamp_ns": hand_fb_ts,
        "hand_feedback_age_ns": (tick - hand_fb_ts).astype(np.int64),
        "hand_feedback_sequence": (3 * k).astype(np.int64),
        "hand_feedback_valid": np.ones(n, dtype=bool),
        "hand_qpos_canonical_rad": hand_qpos,
        "hand_feedback_canonical_valid": hand_canonical_valid,
        "robot_qcmd_17d_rad": qcmd17,
        "robot_qcmd_17d_valid": qcmd17_valid,
        "robot_desired_qcmd_17d_rad": desired17,
        "robot_desired_qcmd_17d_valid": desired_valid,
        "robot_qpos_17d_rad": qpos17,
        "robot_qpos_17d_valid": qpos17_valid,
        "tracking_valid": np.asarray(
            [
                index not in tracking_invalid_ticks
                for index in range(n)
            ],
            dtype=bool,
        ),
        "avp_anchored": np.ones(n, dtype=bool),
        "motion_enabled": np.zeros(n, dtype=bool),
        "estop_latched": np.asarray(
            [index in estop_ticks for index in range(n)], dtype=bool
        ),
        "telemetry_valid": np.asarray(
            [
                index not in telemetry_invalid_ticks
                for index in range(n)
            ],
            dtype=bool,
        ),
        "telemetry_age_ns": np.full(n, 1_000_000, dtype=np.int64),
        "telemetry_publish_sequence": k.astype(np.int64),
        "telemetry_publish_timestamp_ns": (tick - 1_000_000).astype(np.int64),
        "head_camera_host_timestamp_ns": camera_ts,
        "head_camera_device_timestamp_ms": k * 33.3333,
        "head_camera_frame_number": (k + 1).astype(np.int64),
        "head_camera_age_ns": (tick - camera_ts).astype(np.int64),
        "head_camera_valid": head_valid,
        "head_rgb_frame_index": k.astype(np.int64),
        "wrist_camera_host_timestamp_ns": (
            camera_ts if wrist else np.full(n, -1, dtype=np.int64)
        ),
        "wrist_camera_device_timestamp_ms": (
            k * 33.3333 if wrist else np.full(n, np.nan)
        ),
        "wrist_camera_frame_number": (
            (100 + k).astype(np.int64) if wrist else np.full(n, -1, dtype=np.int64)
        ),
        "wrist_camera_age_ns": (
            (tick - camera_ts).astype(np.int64)
            if wrist else np.full(n, -1, dtype=np.int64)
        ),
        "wrist_camera_valid": wrist_valid_arr if wrist else np.zeros(n, dtype=bool),
        "wrist_rgb_frame_index": (
            k.astype(np.int64) if wrist else np.full(n, -1, dtype=np.int64)
        ),
        "writer_backlog": np.zeros(n, dtype=np.int64),
        "writer_drop_count": np.zeros(n, dtype=np.int64),
    }
    for prefix in ("head", "wrist"):
        arrays[f"{prefix}_depth_host_timestamp_ns"] = np.full(n, -1, dtype=np.int64)
        arrays[f"{prefix}_depth_device_timestamp_ms"] = np.full(n, np.nan)
        arrays[f"{prefix}_depth_frame_number"] = np.full(n, -1, dtype=np.int64)
        arrays[f"{prefix}_depth_age_ns"] = np.full(n, -1, dtype=np.int64)
        arrays[f"{prefix}_depth_valid"] = np.zeros(n, dtype=bool)
        arrays[f"{prefix}_depth_frame_index"] = np.full(n, -1, dtype=np.int64)
        arrays[f"{prefix}_depth_present"] = np.asarray(False, dtype=bool)
    arrays.update(
        _camera_metadata(
            "head", present=True, model="Intel RealSense D455"
        )
    )
    arrays.update(
        _camera_metadata(
            "wrist",
            present=wrist,
            model="Intel RealSense D405",
        )
    )
    return arrays


def _save_raw_episode(
    tmp_path: Path,
    arrays: dict[str, np.ndarray],
    *,
    index: int = 0,
    wrist: bool = False,
    missing_head_media: tuple[int, ...] = (),
    missing_wrist_media: tuple[int, ...] = (),
) -> Path:
    n = int(arrays["timestamp_ns"].size)
    pending = tmp_path / f".pending_test_{index}"
    _write_jpgs(pending / "head" / "rgb", n, skip=missing_head_media)
    if wrist:
        _write_jpgs(pending / "right_wrist" / "rgb", n, skip=missing_wrist_media)
    npz_path, _ = save_episode_atomic(tmp_path, index, arrays, pending)
    return npz_path


@pytest.fixture
def clean_raw_episode(tmp_path: Path) -> Path:
    arrays = _full_v3_arrays(8, wrist=True)
    return _save_raw_episode(tmp_path, arrays, index=0, wrist=True)


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


# ---------------------------------------------------------------- tests 1-3


def test_1_curated_stores_no_duplicate_arm_or_hand_arrays(
    tmp_path: Path, clean_raw_episode: Path
) -> None:
    result = build_curated_episode(
        clean_raw_episode, tmp_path / "curated", include_desired=True
    )
    assert result.written
    with np.load(result.episode_dir / "trajectory.npz") as trajectory:
        keys = set(trajectory.files)
    assert not keys & FORBIDDEN_TRAJECTORY_KEYS
    for duplicate in (
        "arm_qcmd_6d_rad", "hand_qcmd_11d_rad",
        "arm_qpos_6d_rad", "hand_qpos_11d_rad",
        "arm_replay", "hand_replay",
    ):
        assert duplicate not in keys
    assert "robot_qcmd_17d_rad" in keys and "robot_qpos_17d_rad" in keys


def test_2_curated_17d_equals_arm_first6_hand_last11(
    tmp_path: Path,
) -> None:
    arrays = _full_v3_arrays(8, wrist=True)
    raw_path = _save_raw_episode(tmp_path, arrays, index=0, wrist=True)
    result = build_curated_episode(raw_path, tmp_path / "curated")
    with np.load(result.episode_dir / "trajectory.npz") as trajectory:
        qcmd = trajectory["robot_qcmd_17d_rad"]
        qpos = trajectory["robot_qpos_17d_rad"]
    # 17D 唯一存储：前 6 = RM65 effective，后 11 = SG100 effective canonical
    np.testing.assert_allclose(
        qcmd[:, :6], arrays["arm_qcmd_sent_rad"][:7]
    )
    np.testing.assert_allclose(
        qcmd[:, 6:], arrays["hand_qcmd_effective_canonical_rad"][:7]
    )
    np.testing.assert_allclose(qpos[:, :6], arrays["arm_qpos_rad"][:7])
    np.testing.assert_allclose(qpos[:, 6:], arrays["hand_qpos_canonical_rad"][:7])
    assert qcmd.shape == (7, 17) and qpos.shape == (7, 17)


def test_3_ahfe_slices_exact(tmp_path: Path, clean_raw_episode: Path) -> None:
    assert ARM_SLICE == (0, 6) and HAND_SLICE == (6, 17)
    result = build_curated_episode(clean_raw_episode, tmp_path / "curated")
    metadata = _load_json(result.episode_dir / "metadata.json")
    assert metadata["arm_slice"] == [0, 6]
    assert metadata["hand_slice"] == [6, 17]
    assert len(metadata["joint_names"]) == 17
    with np.load(result.episode_dir / "trajectory.npz") as trajectory:
        qcmd = trajectory["robot_qcmd_17d_rad"]
    arm_action = qcmd[..., ARM_SLICE[0]:ARM_SLICE[1]]
    hand_action = qcmd[..., HAND_SLICE[0]:HAND_SLICE[1]]
    assert arm_action.shape == (7, 6) and hand_action.shape == (7, 11)


# ---------------------------------------------------------------- tests 4-6


def test_4_derived_state_and_action_skew_rebuilt_from_source_timestamps(
    tmp_path: Path, clean_raw_episode: Path
) -> None:
    result = build_curated_episode(clean_raw_episode, tmp_path / "curated")
    with np.load(result.episode_dir / "trajectory.npz") as trajectory:
        files = set(trajectory.files)
        state_skew = arm_hand_state_skew_ns(
            trajectory["arm_qpos_source_timestamp_ns"],
            trajectory["hand_qpos_source_timestamp_ns"],
        )
        action_skew = arm_hand_action_skew_ns(
            trajectory["arm_qcmd_source_timestamp_ns"],
            trajectory["hand_qcmd_source_timestamp_ns"],
        )
    # synthetic：qpos skew = |9ms-8ms| = 1ms；qcmd skew = |5ms-4ms| = 1ms
    np.testing.assert_allclose(state_skew, 1_000_000)
    np.testing.assert_allclose(action_skew, 1_000_000)
    assert summary_stats_ns(state_skew)["mean_ms"] == pytest.approx(1.0)
    assert summary_stats_ns(action_skew)["p95_ms"] == pytest.approx(1.0)
    assert "arm_hand_state_skew_ns" not in files
    assert "arm_hand_action_skew_ns" not in files


def test_5_derived_dt_correct_without_stored_arrays(
    tmp_path: Path, clean_raw_episode: Path
) -> None:
    result = build_curated_episode(clean_raw_episode, tmp_path / "curated")
    with np.load(result.episode_dir / "trajectory.npz") as trajectory:
        state_ts = trajectory["state_timestamp_ns"]
        action_ts = trajectory["action_timestamp_ns"]
        next_ts = trajectory["next_state_timestamp_ns"]
        state_action = state_action_dt_ns(state_ts, action_ts)
        action_next = action_next_state_dt_ns(action_ts, next_ts)
    # unified：state=max(qpos ts)=t-8ms，action=max(qcmd ts)=t-4ms
    np.testing.assert_allclose(state_action, 4_000_000)
    np.testing.assert_allclose(action_next, (next_ts - state_ts) - 4_000_000)
    assert summary_stats_ns(state_action)["mean_ms"] == pytest.approx(4.0)
    assert summary_stats_ns(action_next)["mean_ms"] == pytest.approx(
        (PERIOD_NS - 4_000_000) / 1e6
    )
    with np.load(result.episode_dir / "trajectory.npz") as trajectory:
        assert "state_action_dt_ns" not in trajectory.files
        assert "action_next_state_dt_ns" not in trajectory.files


def test_6_desired_effective_metrics_rebuildable(
    tmp_path: Path, clean_raw_episode: Path
) -> None:
    result = build_curated_episode(
        clean_raw_episode, tmp_path / "curated", include_desired=True
    )
    with np.load(result.episode_dir / "trajectory.npz") as trajectory:
        desired = trajectory["robot_desired_qcmd_17d_rad"]
        effective = trajectory["robot_qcmd_17d_rad"]
        files = set(trajectory.files)
    summary = desired_effective_summary(desired, effective)
    delta = desired - effective
    expected_l2 = np.linalg.norm(delta, axis=1)
    assert summary["desired_effective_l2"]["mean"] == pytest.approx(
        float(expected_l2.mean())
    )
    assert summary["desired_effective_max_abs"]["max"] == pytest.approx(
        float(np.max(np.abs(delta)))
    )
    assert "desired_effective_l2" not in files
    assert "desired_effective_max_abs" not in files


# ------------------------------------------------------------- tests 7-10


def test_7_curated_contains_only_valid_transitions(tmp_path: Path) -> None:
    arrays = _full_v3_arrays(8, qpos_invalid=(3,))
    raw_path = _save_raw_episode(tmp_path, arrays, index=0)
    result = build_curated_episode(raw_path, tmp_path / "curated")
    assert result.accepted_count == 5  # 7 candidates - 2 rejected (tick 2,3)
    with np.load(result.episode_dir / "trajectory.npz") as trajectory:
        source_tick = trajectory["source_tick_index"]
        qpos = trajectory["robot_qpos_17d_rad"]
        qcmd = trajectory["robot_qcmd_17d_rad"]
    assert source_tick.tolist() == [0, 1, 4, 5, 6]
    assert np.all(np.isfinite(qpos)) and np.all(np.isfinite(qcmd))
    assert report_segments(result.report) == 2
    assert result.report["segment_offsets"] == [0, 2, 5]
    metadata = _load_json(result.episode_dir / "metadata.json")
    assert metadata["segment_offsets"] == [0, 2, 5]


def report_segments(report: dict) -> int:
    return int(report["segment_count"])


def test_8_cleaning_report_counts_rejected_transitions(tmp_path: Path) -> None:
    arrays = _full_v3_arrays(8, qpos_invalid=(3,), qcmd_invalid=(6,))
    raw_path = _save_raw_episode(tmp_path, arrays, index=0)
    result = build_curated_episode(raw_path, tmp_path / "curated")
    report = result.report
    assert report["raw_tick_count"] == 8
    assert report["candidate_transition_count"] == 7
    # rejected：t2(next tick3 qpos 无效)、t3(state tick3 qpos 无效)、
    # t6(action tick6 qcmd 无效)；t5 的 next_state tick6 qpos 仍有效
    assert report["accepted_transition_count"] == 4
    assert report["rejected_transition_count"] == 3
    counts = report["invalid_reason_counts"]
    assert counts["STATE_QPOS_INVALID"] == 1
    assert counts["ACTION_QCMD_INVALID"] == 1
    assert counts["NEXT_STATE_QPOS_INVALID"] == 1
    saved = _load_json(result.episode_dir / "cleaning_report.json")
    assert saved["invalid_reason_counts"] == counts
    assert saved["training_blockers"]


def test_9_curated_rgb_only_without_depth_fields(
    tmp_path: Path, clean_raw_episode: Path
) -> None:
    result = build_curated_episode(clean_raw_episode, tmp_path / "curated")
    with np.load(result.episode_dir / "trajectory.npz") as trajectory:
        keys = set(trajectory.files)
    assert not any("depth" in key for key in keys)
    assert keys == set(TRAJECTORY_CORE_KEYS)
    media = result.episode_dir / "media"
    assert not (media / "head" / "depth").exists()
    assert not (media / "right_wrist" / "depth").exists()


def test_10_model_ready_contract_excludes_sdk_and_telemetry_diagnostics(
    tmp_path: Path, clean_raw_episode: Path
) -> None:
    result = build_curated_episode(clean_raw_episode, tmp_path / "curated")
    with np.load(result.episode_dir / "trajectory.npz") as trajectory:
        keys = set(trajectory.files)
    diagnostics = {
        "hand_qcmd_sdk_deg", "hand_feedback_sdk_rad", "hand_feedback_modes",
        "telemetry_publish_sequence", "telemetry_age_ns", "telemetry_valid",
        "telemetry_publish_timestamp_ns", "writer_backlog",
        "writer_drop_count", "sampler_wakeup_timestamp_ns",
        "sampler_jitter_ns", "arm_qcmd_sequence", "hand_feedback_sequence",
    }
    assert not keys & diagnostics
    assert not keys & FORBIDDEN_TRAJECTORY_KEYS


# ------------------------------------------------------------ tests 11-13


def test_11_readme_paths_match_code_defaults() -> None:
    text = README_PATH.read_text(encoding="utf-8")
    assert "recording/vla_raw/" in text
    assert "recording/vla_curated/" in text
    assert "datasets/pi05_ahfe/" in text
    assert str(DEFAULT_MODEL_READY_DIR) == "datasets/pi05_ahfe"
    assert str(DEFAULT_CURATED_DIR) == "recording/vla_curated"
    recorder_default = build_recorder_arg_parser().parse_args(
        ["--head-serial", "D455"]
    )
    assert recorder_default.output_dir == "recording/vla_raw"
    builder_default = build_builder_arg_parser().parse_args(
        ["recording/vla_raw/episode_000000.npz"]
    )
    assert builder_default.output_dir == str(DEFAULT_CURATED_DIR)


def test_12_readme_17d_ordering_matches_schema_constant() -> None:
    text = README_PATH.read_text(encoding="utf-8")
    pattern = re.compile(
        r"^\s*(\d{1,2})\s+"
        r"(rm65_joint\d+|thumb_j\d+|index_j\d+|middle_j\d+|little_j\d+)\s*$",
        re.MULTILINE,
    )
    found = {int(index): name for index, name in pattern.findall(text)}
    assert found == {
        index: name for index, name in enumerate(ROBOT_JOINT_NAMES)
    }


def test_13_readme_ahfe_slices_match_code() -> None:
    text = README_PATH.read_text(encoding="utf-8")
    arm = re.search(r"Arm slice:\s*\[(\d+):(\d+)\]", text)
    hand = re.search(r"Hand slice:\s*\[(\d+):(\d+)\]", text)
    assert arm is not None and hand is not None
    assert (int(arm.group(1)), int(arm.group(2))) == ARM_SLICE
    assert (int(hand.group(1)), int(hand.group(2))) == HAND_SLICE


# ------------------------------------------------------- builder behaviors


def test_synthetic_full_v3_episode_passes_raw_validator(tmp_path: Path) -> None:
    arrays = _full_v3_arrays(8, wrist=True)
    media = tmp_path / "media"
    _write_jpgs(media / "head" / "rgb", 8)
    _write_jpgs(media / "right_wrist" / "rgb", 8)
    report = validate_episode(
        arrays,
        media_root=media,
        writer_drop_count=0,
        writer_error_count=0,
        telemetry_sequence_gap_count=0,
    )
    assert report.quality == "PASS", report.errors


def test_builder_rejects_all_invalid_episode_by_default(
    tmp_path: Path,
) -> None:
    arrays = _full_v3_arrays(4, qpos_invalid=(0, 1, 2, 3))
    raw_path = _save_raw_episode(tmp_path, arrays, index=0)
    result = build_curated_episode(raw_path, tmp_path / "curated")
    assert not result.written and result.episode_dir is None
    assert result.report["training_blockers"].count("NO_ACCEPTED_TRANSITIONS")
    assert not (tmp_path / "curated").exists() or not any(
        (tmp_path / "curated").iterdir()
    )
    allow_empty = build_curated_episode(
        raw_path, tmp_path / "curated", allow_empty=True
    )
    assert allow_empty.written
    assert allow_empty.report["accepted_transition_count"] == 0
    assert _load_json(allow_empty.episode_dir / "metadata.json")[
        "training_ready"
    ] is False


def test_builder_refuses_overwrite(tmp_path: Path, clean_raw_episode: Path) -> None:
    build_curated_episode(clean_raw_episode, tmp_path / "curated")
    with pytest.raises(FileExistsError):
        build_curated_episode(clean_raw_episode, tmp_path / "curated")


def test_builder_missing_media_rejects_transition(tmp_path: Path) -> None:
    arrays = _full_v3_arrays(6)
    raw_path = _save_raw_episode(
        tmp_path, arrays, index=0, missing_head_media=(4,)
    )
    result = build_curated_episode(raw_path, tmp_path / "curated")
    counts = result.report["invalid_reason_counts"]
    assert counts.get("HEAD_RGB_MEDIA_MISSING") == 1
    assert result.accepted_count == 4


def test_builder_dual_camera_wrist_gate(tmp_path: Path) -> None:
    arrays = _full_v3_arrays(6, wrist=True, wrist_invalid=(2,))
    raw_path = _save_raw_episode(tmp_path, arrays, index=0, wrist=True)
    result = build_curated_episode(raw_path, tmp_path / "curated")
    assert result.report["invalid_reason_counts"].get("WRIST_RGB_INVALID") == 1
    assert result.accepted_count == 4
    assert result.report["accepted_transition_count"] == 4
    assert len(
        tuple((result.episode_dir / "media" / "right_wrist" / "rgb").glob("*.jpg"))
    ) == 4


def test_builder_curated_media_rebased_dense_indices(
    tmp_path: Path,
) -> None:
    arrays = _full_v3_arrays(6, qpos_invalid=(2,))
    raw_path = _save_raw_episode(tmp_path, arrays, index=0)
    result = build_curated_episode(raw_path, tmp_path / "curated")
    files = sorted(
        path.name
        for path in (result.episode_dir / "media" / "head" / "rgb").glob("*.jpg")
    )
    assert files == [f"{index:06d}.jpg" for index in range(3)]


# --------------------------------------------------------- export / replay


def test_export_replay_hand_slice_and_provenance(
    tmp_path: Path, clean_raw_episode: Path
) -> None:
    result = build_curated_episode(clean_raw_episode, tmp_path / "curated")
    trajectory = result.episode_dir / "trajectory.npz"
    commands, provenance = load_replay_commands(trajectory, target="hand")
    with np.load(trajectory) as archive:
        expected = archive["robot_qcmd_17d_rad"][:, 6:17]
    np.testing.assert_allclose(commands, expected)
    assert commands.shape == (7, 11)
    assert provenance["source"] == "effective"
    assert (
        provenance["command_semantics"]
        == "FINAL_EFFECTIVE_BACKEND_COMMAND"
    )
    assert provenance["slice"] == [6, 17]

    arm, arm_provenance = load_replay_commands(trajectory, target="arm")
    assert arm.shape == (7, 6)
    assert arm_provenance["slice"] == [0, 6]
    both, _ = load_replay_commands(trajectory, target="arm_hand")
    assert both.shape == (7, 17)


def test_export_replay_desired_source_distinguished(
    tmp_path: Path, clean_raw_episode: Path
) -> None:
    result = build_curated_episode(
        clean_raw_episode, tmp_path / "curated", include_desired=True
    )
    trajectory = result.episode_dir / "trajectory.npz"
    commands, provenance = load_replay_commands(
        trajectory, target="hand", source="desired"
    )
    with np.load(trajectory) as archive:
        expected = archive["robot_desired_qcmd_17d_rad"][:, 6:17]
    np.testing.assert_allclose(commands, expected)
    assert (
        provenance["command_semantics"]
        == "DESIRED_INTENT_NOT_HARDWARE_EXECUTED"
    )
    assert "desired" in provenance["note"]

    output, sidecar = export_replay_commands(
        trajectory, target="hand", source="desired", output=tmp_path / "out.npy"
    )
    assert output.name == "out.npy" and sidecar.name == "out.npy.json"
    saved = _load_json(sidecar)
    assert saved["source"] == "desired" and saved["target"] == "hand"
    default_output, _ = export_replay_commands(
        trajectory, target="arm", source="effective"
    )
    assert default_output.name == "trajectory_arm_qcmd_effective_rad.npy"


def test_export_replay_desired_nan_policy(tmp_path: Path) -> None:
    arrays = _full_v3_arrays(5, desired_invalid=(1, 3))
    raw_path = _save_raw_episode(tmp_path, arrays, index=0)
    with pytest.raises(ValueError, match="NaN"):
        load_replay_commands(
            raw_path, target="hand", source="desired", drop_nan=False
        )
    commands, provenance = load_replay_commands(
        raw_path, target="hand", source="desired", drop_nan=True
    )
    assert commands.shape == (3, 11)
    assert provenance["rows_dropped_nan"] == 2


# ------------------------------------------------------------- inspector


def test_inspect_raw_episode_derived_timing_metrics(
    tmp_path: Path,
) -> None:
    arrays = _full_v3_arrays(6)
    raw_path = _save_raw_episode(tmp_path, arrays, index=0)
    summary = inspect_vla_episode(raw_path)
    assert summary["layer"] == "RAW"
    assert summary["arm_hand_state_skew"]["mean_ms"] == pytest.approx(1.0)
    assert summary["arm_hand_action_skew"]["mean_ms"] == pytest.approx(1.0)
    # arm command sequence 每 tick 前进 3、tick 周期 1/30s → native ~90 Hz
    # （sequence-delta 证据估计；非 30Hz tick 冒充）
    assert summary["arm_command_native_hz"] == pytest.approx(90.0, rel=0.01)
    assert "arm_feedback" in str(summary["native_rate_evidence"])
    assert "desired_effective" in summary


def test_inspect_curated_episode_and_auto_dispatch(
    tmp_path: Path, clean_raw_episode: Path
) -> None:
    result = build_curated_episode(clean_raw_episode, tmp_path / "curated")
    summary = inspect_curated_episode(result.episode_dir)
    assert summary["layer"] == "CURATED"
    assert summary["transitions"] == 7
    assert summary["arm_hand_state_skew"]["mean_ms"] == pytest.approx(1.0)
    assert summary["state_action_dt"]["mean_ms"] == pytest.approx(4.0)
    assert summary["action_next_state_dt"]["mean_ms"] == pytest.approx(
        (PERIOD_NS - 4_000_000) / 1e6
    )
    assert summary["replay_slices"]["hand_qcmd_rad"] == [6, 17]
    assert summary["media_rgb_counts"] == {"head": 7, "right_wrist": 7}
    assert summary["dataset_status"] == "CURATED_NOT_TRAINING_READY"


def test_sequence_rate_hz_ignores_duplicate_sampling() -> None:
    sequence = np.array([0, 3, 3, 6, 6, 9])
    # CLOCK_MONOTONIC 永不为 0；ts<=0 是哨兵，必须被过滤
    timestamps = 1_000_000_000_000 + np.array(
        [0, 33, 66, 99, 132, 165]
    ) * 1_000_000
    assert sequence_rate_hz(sequence, timestamps) == pytest.approx(
        9 / 0.165, rel=0.01
    )


def test_curated_schema_constants_are_the_readme_contract() -> None:
    # README 数据流程图与状态表的代码侧 authority 检查
    text = README_PATH.read_text(encoding="utf-8")
    assert CURATED_SCHEMA_NAME == "rm65_sg100_dual_realsense_vla_curated"
    assert "ADAPTER" in text and "NOT IMPLEMENTED YET" in text
    assert "NOT GENERATED YET" in text
    assert "TRAINING READY" not in text.replace("NOT TRAINING READY", "")


# ------------------------------------------- v1 freeze: causality / segment


def test_component_causality_rejects_action_before_state(tmp_path: Path) -> None:
    arrays = _full_v3_arrays(6)
    # tick3 的 arm command 早于 arm state feedback → ARM component 因果破坏
    arrays["arm_qcmd_source_timestamp_ns"][3] -= 10_000_000
    raw_path = _save_raw_episode(tmp_path, arrays, index=0)
    result = build_curated_episode(raw_path, tmp_path / "curated")
    counts = result.report["invalid_reason_counts"]
    assert counts.get("ARM_CAUSALITY_INVALID") == 1
    assert result.accepted_count == 4
    with np.load(result.episode_dir / "trajectory.npz") as trajectory:
        assert 3 not in trajectory["source_tick_index"].tolist()


def test_component_causality_rejects_action_not_before_next_state(
    tmp_path: Path,
) -> None:
    arrays = _full_v3_arrays(6)
    # tick4 的 arm state feedback 回退到远古（仍 >0 且 <= tick）→ transition 3
    # 的 arm action 不再严格早于 next arm state
    arrays["arm_qpos_source_timestamp_ns"][4] = int(
        arrays["timestamp_ns"][0]
    ) + 1
    raw_path = _save_raw_episode(tmp_path, arrays, index=0)
    result = build_curated_episode(raw_path, tmp_path / "curated")
    counts = result.report["invalid_reason_counts"]
    assert counts.get("ARM_CAUSALITY_INVALID") == 1
    assert counts.get("UNIFIED_CAUSALITY_INVALID", 0) == 0
    assert result.accepted_count == 4


def test_hand_component_causality_gate(tmp_path: Path) -> None:
    arrays = _full_v3_arrays(6)
    arrays["hand_qcmd_source_timestamp_ns"][2] -= 10_000_000
    raw_path = _save_raw_episode(tmp_path, arrays, index=0)
    result = build_curated_episode(raw_path, tmp_path / "curated")
    assert result.report["invalid_reason_counts"].get(
        "HAND_CAUSALITY_INVALID"
    ) == 1


def test_unified_timestamps_are_component_max_not_tick(tmp_path: Path) -> None:
    arrays = _full_v3_arrays(6)
    raw_path = _save_raw_episode(tmp_path, arrays, index=0)
    result = build_curated_episode(raw_path, tmp_path / "curated")
    tick = arrays["timestamp_ns"]
    with np.load(result.episode_dir / "trajectory.npz") as trajectory:
        state_ts = trajectory["state_timestamp_ns"]
        action_ts = trajectory["action_timestamp_ns"]
        next_ts = trajectory["next_state_timestamp_ns"]
    np.testing.assert_array_equal(
        state_ts,
        np.maximum(
            arrays["arm_qpos_source_timestamp_ns"][:5],
            arrays["hand_feedback_source_timestamp_ns"][:5],
        ),
    )
    np.testing.assert_array_equal(
        action_ts,
        np.maximum(
            arrays["arm_qcmd_source_timestamp_ns"][:5],
            arrays["hand_qcmd_source_timestamp_ns"][:5],
        ),
    )
    np.testing.assert_array_equal(
        next_ts,
        np.maximum(
            arrays["arm_qpos_source_timestamp_ns"][1:6],
            arrays["hand_feedback_source_timestamp_ns"][1:6],
        ),
    )
    # unified ≠ dataset tick：state_ts 应早于 tick
    assert np.all(state_ts < tick[:5])


def test_estop_ends_segment_without_rejecting(tmp_path: Path) -> None:
    arrays = _full_v3_arrays(8, estop_ticks=(4,))
    raw_path = _save_raw_episode(tmp_path, arrays, index=0)
    result = build_curated_episode(raw_path, tmp_path / "curated")
    # E-stop 切 segment 而不拒绝有效数据
    assert result.accepted_count == 7
    assert result.report["segment_offsets"] == [0, 3, 4, 7]
    assert result.report["segment_event_split_count"] == 2
    metadata = _load_json(result.episode_dir / "metadata.json")
    assert metadata["segment_offsets"] == [0, 3, 4, 7]


def test_tracking_and_telemetry_loss_split_segment(tmp_path: Path) -> None:
    arrays = _full_v3_arrays(6, tracking_invalid_ticks=(2,))
    raw_path = _save_raw_episode(tmp_path, arrays, index=0)
    result = build_curated_episode(raw_path, tmp_path / "curated")
    # adverse tick2 覆盖 row1(k=1,k+1=2) 与 row2(k=2,k+1=3)
    assert result.report["segment_offsets"] == [0, 1, 2, 5]


def test_validator_rejects_discontinuity_without_segment_split(
    tmp_path: Path, clean_raw_episode: Path
) -> None:
    result = build_curated_episode(clean_raw_episode, tmp_path / "curated")
    metadata = _load_json(result.episode_dir / "metadata.json")
    with np.load(result.episode_dir / "trajectory.npz") as archive:
        trajectory = {key: archive[key] for key in archive.files}
    # 构造违规：不连续却留在同一 segment
    broken = dict(metadata)
    broken["segment_offsets"] = [0, 7]
    trajectory["source_tick_index"] = trajectory[
        "source_tick_index"
    ].copy()
    trajectory["source_tick_index"][3] += 1
    report = validate_curated_episode(
        broken, trajectory, allow_empty=False
    )
    assert report.quality == "FAIL"
    assert any("必须切 segment" in error for error in report.errors)


def _full_v3_save(tmp_path: Path) -> Path:
    arrays = _full_v3_arrays(6)
    return _save_raw_episode(tmp_path, arrays, index=0)


def test_validator_segment_offsets_boundary_contract(tmp_path: Path) -> None:
    raw_path = _full_v3_save(tmp_path)
    result = build_curated_episode(raw_path, tmp_path / "curated")
    metadata = _load_json(result.episode_dir / "metadata.json")
    with np.load(result.episode_dir / "trajectory.npz") as archive:
        trajectory = {key: archive[key] for key in archive.files}
    for bad_offsets in ([1, 5], [0, 6, 5], [0, 3], [0, 4, 4, 5]):
        broken = dict(metadata)
        broken["segment_offsets"] = bad_offsets
        report = validate_curated_episode(broken, trajectory)
        assert report.quality == "FAIL", bad_offsets
        assert any("segment_offsets" in error for error in report.errors)


def test_rgb_media_dedup_shares_curated_index(tmp_path: Path) -> None:
    arrays = _full_v3_arrays(7)
    # 多个 tick 引用同一 RAW frame（相机重采样）：
    # tick0/1→frame0, tick2→1, tick3→2, tick4/5→3, tick6→4
    arrays["head_rgb_frame_index"] = np.asarray(
        [0, 0, 1, 2, 3, 3, 4], dtype=np.int64
    )
    raw_path = _save_raw_episode(tmp_path, arrays, index=0)
    result = build_curated_episode(raw_path, tmp_path / "curated")
    assert result.accepted_count == 6
    with np.load(result.episode_dir / "trajectory.npz") as trajectory:
        head_index = trajectory["head_rgb_frame_index"].tolist()
    assert head_index == [0, 0, 1, 2, 3, 3]
    files = sorted(
        path.name
        for path in (result.episode_dir / "media" / "head" / "rgb").glob("*.jpg")
    )
    assert files == [f"{index:06d}.jpg" for index in range(4)]


def test_curated_native_rate_not_recoverable_is_stated(
    tmp_path: Path, clean_raw_episode: Path
) -> None:
    result = build_curated_episode(clean_raw_episode, tmp_path / "curated")
    summary = inspect_curated_episode(result.episode_dir)
    assert "NOT EXACTLY RECOVERABLE" in str(summary["native_rate_note"])
    assert summary["segment_count"] == 1
    assert summary["segment_offsets"] == [0, 7]


def test_readme_states_native_rate_limitation_and_sop() -> None:
    text = README_PATH.read_text(encoding="utf-8")
    assert "NOT EXACTLY RECOVERABLE" in text
    assert "CONFIGURED RATE" in text and "OBSERVED RATE" in text
    assert "Reserved" in text  # C/A/D/E 保留键章节
    assert "不是录制开始" in text
