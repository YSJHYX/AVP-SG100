"""30 Hz raw VLA recorder 纯软件测试；不得连接任何硬件。"""

from __future__ import annotations

from pathlib import Path
import socket
import threading
import time

import numpy as np
import pytest

from sg100_teleop.camera.realsense_camera import RealSenseFrame
from sg100_teleop.camera.diagnose_realsense_depth import (
    aggregate_valid_ratios,
    compare_validity,
    compute_depth_stats,
)
from sg100_teleop.data_collection.episode_controller import (
    EpisodeController,
    EpisodeState,
)
from sg100_teleop.data_collection.synchronized_sampler import SynchronizedSampler
from sg100_teleop.data_collection.vla_raw_buffer import VlaRawBuffer
from sg100_teleop.data_collection.vla_raw_schema import (
    DATASET_STATUS_QUARANTINED,
    DATASET_STATUS_READY,
    DEFAULT_CAMERA_MODALITIES,
    DEFAULT_REQUIRED_CAMERAS,
    FixedRateTimeline,
    FreshnessThresholds,
    SCHEMA_VERSION,
    scan_next_episode_index,
)
from sg100_teleop.data_collection.run_vla_raw_recorder import (
    build_arg_parser as build_recorder_arg_parser,
    create_runtime_sources,
)
from sg100_teleop.data_collection.vla_raw_writer import (
    BoundedMediaWriter,
    save_episode_atomic,
)
from sg100_teleop.data_collection.vla_raw_validator import (
    telemetry_rate_diagnostics,
    validate_episode,
)
from sg100_teleop.telemetry.teleop_telemetry import (
    TELEMETRY_SCHEMA_VERSION,
    TeleopTelemetryPublisher,
    TeleopTelemetrySubscriber,
    build_teleop_snapshot,
)
from sg100_teleop.robot.sg100_driver import (
    FEEDBACK_MAP_STATUS_UNVALIDATED,
    FEEDBACK_MAP_STATUS_VALIDATED,
)


def _packet(
    timestamp_ns: int,
    sequence: int = 1,
    *,
    arm_backend_kind: str = "sim",
    hand_backend_kind: str = "sim",
    feedback_mapping_status: str = FEEDBACK_MAP_STATUS_UNVALIDATED,
) -> dict:
    return {
        "schema_version": TELEMETRY_SCHEMA_VERSION,
        "publish_sequence": sequence,
        "publish_monotonic_ns": timestamp_ns,
        "arm_backend_kind": arm_backend_kind,
        "hand_backend_kind": hand_backend_kind,
        "telemetry_publish_hz_requested": 100.0,
        "tracking_valid": True,
        "avp_anchored": True,
        "motion_enabled": True,
        "estop_latched": False,
        "arm_q_goal_rad": np.arange(6, dtype=float).tolist(),
        "arm_qcmd_sent_rad": (10 + np.arange(6, dtype=float)).tolist(),
        "arm_qcmd_sent_timestamp_ns": timestamp_ns - 30,
        "arm_qcmd_sequence": 7,
        "arm_command_valid": True,
        "arm_qpos_rad": (20 + np.arange(6, dtype=float)).tolist(),
        "arm_qpos_timestamp_ns": timestamp_ns - 20,
        "arm_feedback_sequence": 8,
        "arm_feedback_valid": True,
        "controller_error": 0,
        "arm_status": 1,
        "feedback_channel": "TCP_POLL",
        "sender_hz": 100.0,
        "hand_qcmd_retargeted_rad": (30 + np.arange(11, dtype=float)).tolist(),
        "hand_qcmd_effective_canonical_rad": (40 + np.arange(11, dtype=float)).tolist(),
        "hand_qcmd_sdk_deg": (50 + np.arange(11, dtype=float)).tolist(),
        "hand_qcmd_timestamp_ns": timestamp_ns - 25,
        "hand_command_sequence": 9,
        "hand_command_valid": True,
        "hand_feedback_sdk_rad": (60 + np.arange(11, dtype=float)).tolist(),
        "hand_feedback_modes": [7] * 11,
        "hand_feedback_timestamp_ns": timestamp_ns - 15,
        "hand_feedback_sequence": 10,
        "hand_feedback_valid": True,
        "hand_qpos_canonical_rad": None,
        "hand_feedback_canonical_valid": False,
        "hand_feedback_mapping_status": feedback_mapping_status,
    }


def _free_udp_endpoint() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return f"127.0.0.1:{port}"


def test_udp_publisher_subscriber_integration_absent_and_stale() -> None:
    disabled = TeleopTelemetryPublisher("")
    assert disabled.publish({"not": "validated"}) is False
    assert disabled.publish_sequence == 0 and disabled.drop_count == 0

    endpoint = _free_udp_endpoint()
    subscriber = TeleopTelemetrySubscriber(endpoint, alive_timeout_ns=10_000_000)
    subscriber.start()
    publisher = TeleopTelemetryPublisher(endpoint)
    assert publisher.publish({"tracking_valid": True})
    deadline = time.monotonic() + 1.0
    while subscriber.latest() is None and time.monotonic() < deadline:
        time.sleep(0.005)
    packet = subscriber.latest()
    assert packet is not None and packet["tracking_valid"] is True
    assert subscriber.is_alive()
    assert not subscriber.is_alive(time.monotonic_ns() + 20_000_000)
    publisher.close()
    subscriber.stop()


def test_teleop_cli_keeps_external_telemetry_disabled_by_default() -> None:
    from sg100_teleop.sim.run_avp_arm_hand_sim import build_arg_parser

    args = build_arg_parser().parse_args(["--ip", "fake-avp"])
    assert args.telemetry_publish == ""
    assert args.telemetry_publish_hz == 100.0


def test_explicit_d_disarm_helper_never_enables_backends() -> None:
    from sg100_teleop.sim.run_avp_arm_hand_sim import (
        disarm_real_command_authority,
    )

    class Backend:
        def __init__(self, enabled: bool) -> None:
            self.enabled = enabled
            self.calls = []

        def set_enabled(self, value: bool) -> None:
            self.calls.append(("set_enabled", value))
            self.enabled = value

        def stop_smooth(self) -> None:
            self.calls.append(("stop_smooth",))

    arm = Backend(True)
    hand = Backend(True)
    assert disarm_real_command_authority(arm, hand) is True
    assert arm.calls == [("set_enabled", False), ("stop_smooth",)]
    assert hand.calls == [("set_enabled", False)]
    assert disarm_real_command_authority(arm, hand) is False
    assert arm.calls == [("set_enabled", False), ("stop_smooth",)]
    assert hand.calls == [("set_enabled", False)]


def test_udp_sequence_gap_is_counted() -> None:
    endpoint = _free_udp_endpoint()
    subscriber = TeleopTelemetrySubscriber(endpoint)
    subscriber.start()
    raw = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    host, port = endpoint.split(":")
    import json
    for sequence in (1, 4):
        raw.sendto(json.dumps({
            "schema_version": 1,
            "publish_sequence": sequence,
            "publish_monotonic_ns": time.monotonic_ns(),
        }).encode(), (host, int(port)))
        time.sleep(0.01)
    deadline = time.monotonic() + 1.0
    while subscriber.received_packets < 2 and time.monotonic() < deadline:
        time.sleep(0.005)
    assert subscriber.sequence_gap_count == 2
    raw.close()
    subscriber.stop()


def test_sim_snapshot_exposes_desired_and_requested_rate_without_effective() -> None:
    arm_goal = np.arange(6, dtype=np.float64)
    hand_goal = np.arange(11, dtype=np.float64)
    snapshot = build_teleop_snapshot(
        now_ns=2_000_000_000,
        tracking_received_ns=1_999_000_000,
        tracking_timeout_ns=100_000_000,
        avp_anchored=True,
        arm_backend=None,
        arm_q_goal_rad=arm_goal,
        arm_sender_hz=100.0,
        hand_driver=None,
        fallback_hand_feedback=None,
        restricted_hand_backend=None,
        hand_qcmd_retargeted_rad=hand_goal,
        latest_hand_command=None,
        arm_backend_kind="sim",
        hand_backend_kind="sim",
        telemetry_publish_hz_requested=100.0,
    )
    np.testing.assert_array_equal(snapshot["arm_q_goal_rad"], arm_goal)
    np.testing.assert_array_equal(
        snapshot["hand_qcmd_retargeted_rad"], hand_goal
    )
    assert snapshot["arm_qcmd_sent_rad"] is None
    assert snapshot["hand_qcmd_effective_canonical_rad"] is None
    assert snapshot["arm_backend_kind"] == "sim"
    assert snapshot["hand_backend_kind"] == "sim"
    assert snapshot["telemetry_publish_hz_requested"] == 100.0


def test_fixed_30hz_timeline_has_no_60s_accumulated_drift() -> None:
    timeline = FixedRateTimeline(30.0, 123)
    assert timeline.tick_ns(1800) == 60_000_000_123
    periods = np.diff([timeline.tick_ns(i) for i in range(1801)])
    assert set(periods.tolist()) == {33_333_333, 33_333_334}


class FakeTelemetry:
    def __init__(self, packets: list[dict] | None = None) -> None:
        self.packets = packets or []
        self.cleared = 0

    def latest_before(self, timestamp_ns: int):
        eligible = [p for p in self.packets if p["publish_monotonic_ns"] <= timestamp_ns]
        return dict(eligible[-1]) if eligible else None

    def latest(self):
        return dict(self.packets[-1]) if self.packets else None

    def is_alive(self):
        return bool(self.packets)

    def clear(self):
        self.cleared += 1

    @property
    def stats(self):
        return {"received_packets": len(self.packets), "sequence_gap_count": 0}


class FakeCamera:
    def __init__(self, role: str, frames: list[RealSenseFrame]) -> None:
        self.role = role
        self.frames = frames
        self.cleared = 0

    @property
    def ready(self):
        return bool(self.frames)

    def latest_before(self, timestamp_ns: int):
        eligible = [f for f in self.frames if f.host_receive_monotonic_ns <= timestamp_ns]
        return eligible[-1] if eligible else None

    def clear(self):
        self.cleared += 1

    def metadata(self):
        return {
            "model": "Intel RealSense D455" if self.role == "head" else "Intel RealSense D405",
            "serial": f"serial-{self.role}",
            "width": 2,
            "height": 2,
            "fps": 30,
            "depth_scale": 0.001,
            "color_intrinsics": np.eye(3),
            "depth_intrinsics": np.eye(3),
            "extrinsics_to_robot": "UNAVAILABLE / NOT_CALIBRATED",
        }


class MemoryWriter:
    def __init__(self):
        self.items = []
        self.drop_count = 0
        self.error_count = 0
        self.max_backlog = 0

    @property
    def backlog(self):
        return 0

    def enqueue(self, index, head, wrist):
        self.items.append((index, head, wrist))
        return True


def _frame(role: str, timestamp: int, number: int) -> RealSenseFrame:
    return RealSenseFrame(
        role=role,
        host_receive_monotonic_ns=timestamp,
        device_timestamp_ms=float(number) * 3.0,
        frame_number=number,
        color_rgb=np.full((2, 2, 3), number, dtype=np.uint8),
        depth_uint16=np.full((2, 2), number, dtype=np.uint16),
    )


def test_causal_alignment_staleness_17d_and_equal_n() -> None:
    tick = 1_000_000_000
    # episode start 早于所有 source timestamp：On 边界 floor 只允许
    # post-On source 成为 valid sample
    start = tick - 1_000
    telemetry = FakeTelemetry([_packet(tick - 10)])
    head = FakeCamera("head", [_frame("head", tick - 50, 1), _frame("head", tick + 1, 2)])
    wrist = FakeCamera("right_wrist", [_frame("right_wrist", tick - 40, 3)])
    buffer = VlaRawBuffer()
    writer = MemoryWriter()
    sampler = SynchronizedSampler(
        hz=30,
        start_monotonic_ns=start,
        telemetry=telemetry,
        head_camera=head,
        wrist_camera=wrist,
        buffer=buffer,
        media_writer=writer,
        thresholds=FreshnessThresholds(),
    )
    sampler.sample_tick(0, tick, tick + 5)
    arrays = buffer.arrays()
    assert arrays["head_camera_frame_number"].tolist() == [1]
    assert arrays["head_camera_host_timestamp_ns"][0] <= arrays["timestamp_ns"][0]
    np.testing.assert_allclose(
        arrays["robot_qcmd_17d_rad"][0],
        np.concatenate((10 + np.arange(6), 40 + np.arange(11))),
    )
    assert not arrays["robot_qpos_17d_valid"][0]
    assert np.isnan(arrays["robot_qpos_17d_rad"][0, 6:]).all()
    assert all(value.shape[0] == 1 for value in arrays.values())

    stale_buffer = VlaRawBuffer()
    stale_sampler = SynchronizedSampler(
        hz=30,
        start_monotonic_ns=tick + 1_000,
        telemetry=telemetry,
        head_camera=head,
        wrist_camera=wrist,
        buffer=stale_buffer,
        media_writer=MemoryWriter(),
        thresholds=FreshnessThresholds(
            max_arm_command_age_ns=1,
            max_arm_feedback_age_ns=1,
            max_hand_command_age_ns=1,
            max_hand_feedback_age_ns=1,
            max_camera_age_ns=1,
            max_telemetry_age_ns=1,
        ),
    )
    stale_sampler.sample_tick(0, tick + 1_000, tick + 1_001)
    assert not stale_buffer.arrays()["telemetry_valid"][0]
    assert not stale_buffer.arrays()["head_camera_valid"][0]


def _controller(tmp_path: Path, packet: dict | None = None):
    timestamp = time.monotonic_ns() - 1_000_000
    telemetry = FakeTelemetry([packet or _packet(timestamp)])
    head = FakeCamera("head", [_frame("head", timestamp, 1)])
    wrist = FakeCamera("right_wrist", [_frame("right_wrist", timestamp, 2)])
    return EpisodeController(
        output_dir=tmp_path,
        hz=30,
        language_instruction="Pick the object",
        telemetry=telemetry,
        head_camera=head,
        wrist_camera=wrist,
    ), telemetry, head, wrist


def _record_with_post_on_packet(controller, telemetry, seconds: float = 0.09):
    """On 后追加 post-On packet 再 Off。

    On 边界语义：tick 0 的 tick_ns == episode start，post-On source 尚不可
    能出现，因此 tick 0 只能保留 pre-On source 并标 invalid；post-On packet
    从 tick 1 起有效。
    """
    controller.handle_input("On")
    telemetry.packets.append(_packet(time.monotonic_ns()))
    time.sleep(seconds)
    return controller.handle_input("Off")


def test_state_machine_only_on_off_and_n_reset_do_not_consume_index(tmp_path: Path) -> None:
    controller, telemetry, head, wrist = _controller(tmp_path)
    assert controller.handle_input("C") is None
    assert controller.handle_input("A") is None
    assert controller.state is EpisodeState.READY
    assert controller.handle_input("On") == 0
    controller.language_instruction = "CHANGED MID EPISODE"
    assert controller.frozen_instruction == "Pick the object"
    assert controller.handle_input("A") is None
    assert controller.state is EpisodeState.RECORDING
    report = controller.handle_input("Off")
    assert report.metrics["frames"] >= 1
    assert controller.state is EpisodeState.PENDING_CONFIRMATION
    assert controller.candidate_arrays["language_instruction"].item() == "Pick the object"
    controller.handle_input("N")
    assert controller.state is EpisodeState.READY
    assert controller.next_episode_index == 0

    controller.language_instruction = "Second"
    controller.handle_input("On")
    controller.handle_input("Off")
    controller.handle_input("Reset")
    assert controller.next_episode_index == 0
    assert telemetry.cleared == head.cleared == wrist.cleared == 1


def test_y_saves_npz_media_and_increments_index(tmp_path: Path) -> None:
    controller, _, _, _ = _controller(tmp_path)
    controller.handle_input("On")
    controller.handle_input("Off")
    saved = controller.handle_input("Y")
    assert saved[0] == tmp_path / "episode_000000.npz"
    assert saved[1] == tmp_path / "episode_000000_media"
    assert controller.next_episode_index == 1
    with np.load(saved[0], allow_pickle=False) as archive:
        assert archive["dataset_status"].item() == DATASET_STATUS_QUARANTINED
        assert archive["training_ready"].item() is False
        assert archive["timestamp_ns"].shape[0] == archive["arm_qpos_rad"].shape[0]
        assert archive["hand_feedback_canonical_valid"].sum() == 0
        assert np.isnan(archive["hand_qpos_canonical_rad"]).all()
        assert archive["robot_joint_names"].shape == (17,)
    assert not (saved[1] / "head" / "depth").exists()


@pytest.mark.parametrize(
    (
        "case_name",
        "arm_backend_kind",
        "hand_backend_kind",
        "feedback_mapping_status",
        "expected_hardware",
        "expected_ready",
        "expected_status",
    ),
    (
        (
            "real_real_validated",
            "real",
            "real",
            FEEDBACK_MAP_STATUS_VALIDATED,
            True,
            True,
            DATASET_STATUS_READY,
        ),
        (
            "sim_sim",
            "sim",
            "sim",
            FEEDBACK_MAP_STATUS_VALIDATED,
            False,
            False,
            DATASET_STATUS_QUARANTINED,
        ),
        (
            "real_real_unvalidated",
            "real",
            "real",
            FEEDBACK_MAP_STATUS_UNVALIDATED,
            True,
            False,
            DATASET_STATUS_QUARANTINED,
        ),
    ),
)
def test_episode_readiness_is_derived_from_frozen_control_plane_provenance(
    tmp_path: Path,
    case_name: str,
    arm_backend_kind: str,
    hand_backend_kind: str,
    feedback_mapping_status: str,
    expected_hardware: bool,
    expected_ready: bool,
    expected_status: str,
) -> None:
    timestamp = time.monotonic_ns() - 1_000_000
    packet = _packet(
        timestamp,
        arm_backend_kind=arm_backend_kind,
        hand_backend_kind=hand_backend_kind,
        feedback_mapping_status=feedback_mapping_status,
    )
    controller, _, _, _ = _controller(tmp_path / case_name, packet)
    controller.handle_input("On")
    report = controller.handle_input("Off")
    assert report.quality == "PASS", report.errors
    arrays = controller.candidate_arrays
    assert arrays["arm_backend_kind"].item() == arm_backend_kind
    assert arrays["hand_backend_kind"].item() == hand_backend_kind
    assert arrays["hardware_execution"].item() is expected_hardware
    assert arrays["training_ready"].item() is expected_ready
    assert arrays["dataset_status"].item() == expected_status
    assert arrays["sg100_feedback_map_status"].item() == feedback_mapping_status
    controller.handle_input("N")


def test_missing_backend_identity_fails_closed(tmp_path: Path) -> None:
    timestamp = time.monotonic_ns() - 1_000_000
    packet = _packet(
        timestamp,
        arm_backend_kind="real",
        hand_backend_kind="real",
        feedback_mapping_status=FEEDBACK_MAP_STATUS_VALIDATED,
    )
    packet.pop("arm_backend_kind")
    packet.pop("hand_backend_kind")
    controller, _, _, _ = _controller(tmp_path, packet)
    controller.handle_input("On")
    report = controller.handle_input("Off")
    assert report.quality == "PASS", report.errors
    arrays = controller.candidate_arrays
    assert arrays["arm_backend_kind"].item() == "unknown"
    assert arrays["hand_backend_kind"].item() == "unknown"
    assert arrays["hardware_execution"].item() is False
    assert arrays["training_ready"].item() is False
    assert arrays["dataset_status"].item() == DATASET_STATUS_QUARANTINED
    controller.handle_input("N")


def test_legacy_v3_pending_status_without_backend_identity_remains_readable(
    tmp_path: Path,
) -> None:
    controller, _, _, _ = _controller(tmp_path)
    controller.handle_input("On")
    controller.handle_input("Off")
    legacy = dict(controller.candidate_arrays)
    legacy.pop("arm_backend_kind")
    legacy.pop("hand_backend_kind")
    legacy["sg100_feedback_map_status"] = np.asarray("F_FB_PENDING")
    report = validate_episode(
        legacy,
        media_root=controller.pending_dir,
        writer_drop_count=0,
        writer_error_count=0,
        telemetry_sequence_gap_count=0,
    )
    assert report.quality == "PASS", report.errors
    controller.handle_input("N")


def test_index_scan_uses_max_not_file_count(tmp_path: Path) -> None:
    for index in (0, 1, 4):
        (tmp_path / f"episode_{index:06d}.npz").touch()
    assert scan_next_episode_index(tmp_path) == 5


def test_atomic_npz_reopen_and_refuses_overwrite(tmp_path: Path) -> None:
    pending = tmp_path / ".pending_x"
    pending.mkdir()
    arrays = {
        "schema_name": np.asarray("test"),
        "episode_index": np.asarray(0),
        "timestamp_ns": np.asarray([1, 2], dtype=np.int64),
    }
    final, media = save_episode_atomic(tmp_path, 0, arrays, pending)
    assert final.exists() and media.exists()
    with np.load(final, allow_pickle=False) as archive:
        assert archive["timestamp_ns"].tolist() == [1, 2]
    pending2 = tmp_path / ".pending_y"
    pending2.mkdir()
    with pytest.raises(FileExistsError):
        save_episode_atomic(tmp_path, 0, arrays, pending2)
    assert not (tmp_path / "episode_000000.npz.tmp").exists()


def test_bounded_media_writer_reports_queue_drop(tmp_path: Path, monkeypatch) -> None:
    entered = threading.Event()
    release = threading.Event()

    def blocked_write(self, role, index, frame):
        del self, role, index, frame
        entered.set()
        release.wait(1.0)

    monkeypatch.setattr(BoundedMediaWriter, "_write_frame", blocked_write)
    writer = BoundedMediaWriter(tmp_path / "pending", queue_size=1)
    frame = _frame("head", 1, 1)
    assert writer.enqueue(0, frame, frame)
    assert entered.wait(1.0)
    assert writer.enqueue(1, frame, frame)
    assert writer.enqueue(2, frame, frame) is False
    assert writer.drop_count == 1
    release.set()
    writer.close()


def _head_only_controller(
    tmp_path: Path,
    *,
    telemetry: FakeTelemetry | None = None,
    telemetry_required: bool = False,
    camera_modalities: tuple[str, ...] = DEFAULT_CAMERA_MODALITIES,
    frame_has_depth: bool = True,
):
    timestamp = time.monotonic_ns() - 1_000_000
    source = telemetry if telemetry is not None else FakeTelemetry([])
    frame = _frame("head", timestamp, 11)
    if not frame_has_depth:
        frame = RealSenseFrame(
            role=frame.role,
            host_receive_monotonic_ns=frame.host_receive_monotonic_ns,
            device_timestamp_ms=frame.device_timestamp_ms,
            frame_number=frame.frame_number,
            color_rgb=frame.color_rgb,
            depth_uint16=None,
        )
    head = FakeCamera("head", [frame])
    controller = EpisodeController(
        output_dir=tmp_path,
        hz=30,
        language_instruction="D455 bring-up",
        telemetry=source,
        head_camera=head,
        wrist_camera=None,
        required_cameras=("head",),
        camera_modalities=camera_modalities,
        telemetry_required=telemetry_required,
        thresholds=FreshnessThresholds(
            max_arm_command_age_ns=1_000_000_000,
            max_arm_feedback_age_ns=1_000_000_000,
            max_hand_command_age_ns=1_000_000_000,
            max_hand_feedback_age_ns=1_000_000_000,
            max_camera_age_ns=1_000_000_000,
            max_telemetry_age_ns=1_000_000_000,
        ),
    )
    return controller, source, head


def test_head_only_camera_only_ready_has_no_fake_wrist_or_robot_data(
    tmp_path: Path,
) -> None:
    controller, _, _ = _head_only_controller(tmp_path)
    assert controller.handle_input("On") == 0
    assert controller.state is EpisodeState.RECORDING
    report = controller.handle_input("Off")
    assert report.quality == "PASS", report.errors
    arrays = controller.candidate_arrays
    assert arrays["head_camera_present"].item() is True
    assert arrays["wrist_camera_present"].item() is False
    assert arrays["hardware_execution"].item() is False
    assert arrays["robot_telemetry_present"].item() is False
    assert arrays["arm_backend_kind"].item() == "none"
    assert arrays["hand_backend_kind"].item() == "none"
    assert arrays["training_ready"].item() is False
    assert arrays["dataset_status"].item() == DATASET_STATUS_QUARANTINED
    n = arrays["timestamp_ns"].shape[0]
    assert n >= 1
    pending = controller.pending_dir
    assert len(tuple((pending / "head" / "rgb").glob("*.jpg"))) == n
    assert not (pending / "head" / "depth").exists()
    assert not (pending / "right_wrist").exists()
    assert not arrays["wrist_camera_valid"].any()
    for key in (
        "wrist_rgb_frame_index",
        "wrist_depth_frame_index",
        "wrist_camera_host_timestamp_ns",
        "wrist_camera_frame_number",
        "wrist_camera_age_ns",
    ):
        assert np.all(arrays[key] == -1), key
    assert np.isnan(arrays["wrist_camera_device_timestamp_ms"]).all()
    for key in (
        "arm_q_goal_rad",
        "arm_qcmd_sent_rad",
        "arm_qpos_rad",
        "hand_qcmd_retargeted_canonical_rad",
        "hand_qcmd_effective_canonical_rad",
        "hand_qcmd_sdk_deg",
        "hand_feedback_sdk_rad",
        "hand_qpos_canonical_rad",
        "robot_qcmd_17d_rad",
        "robot_desired_qcmd_17d_rad",
        "robot_qpos_17d_rad",
    ):
        assert np.isnan(arrays[key]).all(), key
    for key in (
        "arm_command_valid",
        "arm_feedback_valid",
        "hand_command_valid",
        "hand_feedback_valid",
        "hand_feedback_canonical_valid",
        "robot_qcmd_17d_valid",
        "robot_desired_qcmd_17d_valid",
        "robot_qpos_17d_valid",
        "telemetry_valid",
    ):
        assert not arrays[key].any(), key
    saved = controller.handle_input("Y")
    assert saved[0].exists()
    assert not (saved[1] / "right_wrist").exists()
    with np.load(saved[0], allow_pickle=False) as archive:
        assert archive["wrist_camera_present"].item() is False
        assert np.all(archive["wrist_rgb_frame_index"] == -1)
        assert np.isnan(archive["robot_qcmd_17d_rad"]).all()


def test_head_only_with_valid_telemetry_keeps_robot_data(tmp_path: Path) -> None:
    timestamp = time.monotonic_ns() - 1_000_000
    telemetry = FakeTelemetry([_packet(timestamp)])
    controller, _, _ = _head_only_controller(
        tmp_path,
        telemetry=telemetry,
        telemetry_required=True,
    )
    report = _record_with_post_on_packet(controller, telemetry)
    assert report.quality == "PASS", report.errors
    arrays = controller.candidate_arrays
    assert arrays["robot_telemetry_present"].item() is True
    # On 边界：tick 0 只有 pre-On packet 可见 → invalid，不静默使用
    assert not arrays["arm_command_valid"][0]
    assert arrays["arm_command_valid"][1:].all()
    assert arrays["arm_feedback_valid"][1:].all()
    assert arrays["hand_command_valid"][1:].all()
    assert arrays["robot_qcmd_17d_valid"][1:].all()
    np.testing.assert_allclose(
        arrays["robot_qcmd_17d_rad"][1],
        np.concatenate((10 + np.arange(6), 40 + np.arange(11))),
    )
    assert not arrays["wrist_camera_valid"].any()
    controller.handle_input("N")


def test_head_only_and_dual_camera_npz_schema_keys_are_compatible(
    tmp_path: Path,
) -> None:
    head_only, _, _ = _head_only_controller(tmp_path / "head")
    head_only.handle_input("On")
    head_only.handle_input("Off")

    dual, _, _, _ = _controller(tmp_path / "dual")
    dual.handle_input("On")
    dual.handle_input("Off")
    assert set(head_only.candidate_arrays) == set(dual.candidate_arrays)
    assert head_only.candidate_arrays["schema_name"].item() == dual.candidate_arrays[
        "schema_name"
    ].item()
    assert head_only.candidate_arrays["schema_version"].item() == dual.candidate_arrays[
        "schema_version"
    ].item()
    assert head_only.candidate_arrays["wrist_color_intrinsics"].shape == (
        3,
        3,
    )
    assert dual.candidate_arrays["wrist_color_intrinsics"].shape == (3, 3)
    head_only.handle_input("N")
    dual.handle_input("N")


def test_cli_head_only_instantiates_no_wrist_and_production_default_requires_it() -> None:
    parser = build_recorder_arg_parser()
    production = parser.parse_args(["--head-serial", "D455"])
    assert production.required_cameras == DEFAULT_REQUIRED_CAMERAS
    with pytest.raises(ValueError, match="wrist-serial"):
        create_runtime_sources(
            production,
            camera_factory=lambda **kwargs: kwargs,
            subscriber_factory=lambda endpoint: endpoint,
        )

    constructed = []

    def camera_factory(**kwargs):
        constructed.append(kwargs)
        return kwargs

    head_only = parser.parse_args([
        "--head-serial", "D455",
        "--required-cameras", "head",
        "--telemetry", "optional",
    ])
    telemetry, head, wrist, roles, telemetry_required = create_runtime_sources(
        head_only,
        camera_factory=camera_factory,
    )
    assert roles == ("head",)
    assert telemetry_required is False
    assert telemetry.is_alive() is False
    assert head["expected_model"] == "Intel RealSense D455"
    assert wrist is None
    assert [item["role"] for item in constructed] == ["head"]


def test_controller_production_default_still_blocks_missing_d405(
    tmp_path: Path,
) -> None:
    timestamp = time.monotonic_ns() - 1_000_000
    controller = EpisodeController(
        output_dir=tmp_path,
        hz=30,
        language_instruction="production",
        telemetry=FakeTelemetry([_packet(timestamp)]),
        head_camera=FakeCamera("head", [_frame("head", timestamp, 1)]),
        wrist_camera=None,
    )
    with pytest.raises(RuntimeError, match="D405"):
        controller.handle_input("On")
    assert controller.state is EpisodeState.READY


def test_camera_modality_cli_defaults_to_rgb_and_depth_only_rejected() -> None:
    parser = build_recorder_arg_parser()
    default = parser.parse_args(["--head-serial", "D455"])
    assert default.camera_modalities == ("rgb",)
    dual = parser.parse_args([
        "--head-serial", "D455",
        "--wrist-serial", "D405",
        "--telemetry", "optional",
    ])
    _, head, wrist, _, _ = create_runtime_sources(
        dual, camera_factory=lambda **kwargs: kwargs
    )
    assert head["enable_depth"] is False
    assert wrist["enable_depth"] is False
    rgb = parser.parse_args([
        "--head-serial", "D455",
        "--camera-modalities", "rgb",
    ])
    assert rgb.camera_modalities == ("rgb",)
    with pytest.raises(SystemExit):
        parser.parse_args([
            "--head-serial", "D455",
            "--camera-modalities", "depth",
        ])


def test_rgb_only_runtime_source_does_not_request_depth_pipeline() -> None:
    parser = build_recorder_arg_parser()
    args = parser.parse_args([
        "--head-serial", "D455",
        "--required-cameras", "head",
        "--camera-modalities", "rgb",
        "--telemetry", "optional",
    ])
    _, head, wrist, _, _ = create_runtime_sources(
        args, camera_factory=lambda **kwargs: kwargs
    )
    assert head["enable_depth"] is False
    assert wrist is None


def test_head_only_rgb_has_stable_depth_schema_and_no_fake_depth_media(
    tmp_path: Path,
) -> None:
    controller, _, _ = _head_only_controller(
        tmp_path,
        camera_modalities=("rgb",),
        frame_has_depth=False,
    )
    controller.handle_input("On")
    report = controller.handle_input("Off")
    assert report.quality == "PASS", report.errors
    arrays = controller.candidate_arrays
    n = arrays["timestamp_ns"].shape[0]
    assert arrays["camera_modalities"].item() == "rgb"
    assert arrays["record_depth"].item() is False
    assert arrays["head_depth_present"].item() is False
    assert arrays["wrist_depth_present"].item() is False
    assert len(tuple((controller.pending_dir / "head" / "rgb").glob("*.jpg"))) == n
    assert not (controller.pending_dir / "head" / "depth").exists()
    for prefix in ("head", "wrist"):
        assert not arrays[f"{prefix}_depth_valid"].any()
        for key in (
            f"{prefix}_depth_frame_index",
            f"{prefix}_depth_host_timestamp_ns",
            f"{prefix}_depth_frame_number",
            f"{prefix}_depth_age_ns",
        ):
            assert np.all(arrays[key] == -1), key
        assert np.isnan(arrays[f"{prefix}_depth_device_timestamp_ms"]).all()
    controller.handle_input("N")


def test_dual_camera_rgb_only_keeps_both_rgb_and_no_depth(tmp_path: Path) -> None:
    timestamp = time.monotonic_ns() - 1_000_000
    controller = EpisodeController(
        output_dir=tmp_path,
        hz=30,
        language_instruction="dual RGB",
        telemetry=FakeTelemetry([_packet(timestamp)]),
        head_camera=FakeCamera("head", [_frame("head", timestamp, 1)]),
        wrist_camera=FakeCamera(
            "right_wrist", [_frame("right_wrist", timestamp, 2)]
        ),
        camera_modalities=("rgb",),
        thresholds=FreshnessThresholds(max_camera_age_ns=1_000_000_000),
    )
    controller.handle_input("On")
    report = controller.handle_input("Off")
    assert report.quality == "PASS", report.errors
    n = controller.candidate_arrays["timestamp_ns"].shape[0]
    for role in ("head", "right_wrist"):
        assert len(tuple((controller.pending_dir / role / "rgb").glob("*.jpg"))) == n
        assert not (controller.pending_dir / role / "depth").exists()
    assert controller.candidate_arrays["wrist_camera_present"].item() is True
    assert controller.candidate_arrays["wrist_depth_present"].item() is False
    controller.handle_input("N")


def test_rgbd_default_preserves_uint16_depth_and_independent_depth_fields(
    tmp_path: Path,
) -> None:
    frame = _frame("head", 1, 7)
    depth = np.asarray([[0, 1], [1024, 65535]], dtype=np.uint16)
    frame = RealSenseFrame(
        role=frame.role,
        host_receive_monotonic_ns=frame.host_receive_monotonic_ns,
        device_timestamp_ms=frame.device_timestamp_ms,
        frame_number=frame.frame_number,
        color_rgb=frame.color_rgb,
        depth_uint16=depth,
        depth_host_receive_monotonic_ns=1,
        depth_device_timestamp_ms=22.5,
        depth_frame_number=8,
    )
    writer = BoundedMediaWriter(
        tmp_path / "media",
        camera_roles=("head",),
        camera_modalities=("rgb", "depth"),
    )
    assert writer.enqueue(0, frame)
    writer.close()
    import cv2
    decoded = cv2.imread(
        str(tmp_path / "media" / "head" / "depth" / "000000.png"),
        cv2.IMREAD_UNCHANGED,
    )
    assert decoded.dtype == np.uint16
    np.testing.assert_array_equal(decoded, depth)


def test_validator_rejects_unexpected_depth_in_rgb_episode(tmp_path: Path) -> None:
    controller, _, _ = _head_only_controller(
        tmp_path,
        camera_modalities=("rgb",),
        frame_has_depth=False,
    )
    controller.handle_input("On")
    controller.handle_input("Off")
    unexpected = controller.pending_dir / "head" / "depth"
    unexpected.mkdir()
    (unexpected / "000000.png").write_bytes(b"not a real depth frame")
    report = validate_episode(
        controller.candidate_arrays,
        media_root=controller.pending_dir,
        writer_drop_count=0,
        writer_error_count=0,
        telemetry_sequence_gap_count=0,
    )
    assert report.quality == "FAIL"
    assert any("head/depth" in error for error in report.errors)
    controller.handle_input("N")


def test_v1_h1_rgbd_episode_contract_remains_readable(tmp_path: Path) -> None:
    controller, _, _ = _head_only_controller(
        tmp_path, camera_modalities=("rgb", "depth")
    )
    controller.handle_input("On")
    controller.handle_input("Off")
    legacy = dict(controller.candidate_arrays)
    legacy["schema_version"] = np.asarray(1, dtype=np.int32)
    for key in (
        "camera_modalities", "record_depth", "head_depth_present",
        "wrist_depth_present", "head_depth_host_timestamp_ns",
        "head_depth_device_timestamp_ms", "head_depth_frame_number",
        "head_depth_age_ns", "head_depth_valid",
        "wrist_depth_host_timestamp_ns", "wrist_depth_device_timestamp_ms",
        "wrist_depth_frame_number", "wrist_depth_age_ns", "wrist_depth_valid",
    ):
        legacy.pop(key)
    report = validate_episode(
        legacy,
        media_root=controller.pending_dir,
        writer_drop_count=0,
        writer_error_count=0,
        telemetry_sequence_gap_count=0,
    )
    assert report.quality == "PASS", report.errors
    controller.handle_input("N")


def test_depth_diagnostic_synthetic_stats_and_ratio_classification() -> None:
    depth = np.asarray([
        [0, 0, 0, 0],
        [0, 1000, 2000, 0],
        [0, 3000, 4000, 0],
        [0, 0, 0, 0],
    ], dtype=np.uint16)
    stats = compute_depth_stats(depth, 0.001)
    assert stats.total_pixels == 16
    assert stats.valid_pixels == 4
    assert stats.zero_pixels == 12
    assert stats.valid_ratio == pytest.approx(0.25)
    assert stats.center_valid_ratio == pytest.approx(1.0)
    assert stats.median_nonzero_raw == pytest.approx(2500.0)
    assert stats.center_median_m == pytest.approx(2.5)
    aggregate = aggregate_valid_ratios([stats, stats])
    assert aggregate == {
        "mean": 0.25, "p5": 0.25, "p50": 0.25,
        "p95": 0.25, "min": 0.25, "max": 0.25,
    }
    delta, diagnosis = compare_validity(
        {"mean": 0.9}, {"mean": 0.6}
    )
    assert delta == pytest.approx(-0.3)
    assert diagnosis == "ALIGN / PROCESSING issue"
    _, low_diagnosis = compare_validity(
        {"mean": 0.07}, {"mean": 0.06}
    )
    assert low_diagnosis == "CAMERA / SCENE / DISTANCE / STEREO issue"


def test_rgb_and_rgbd_episode_npz_keys_remain_identical(tmp_path: Path) -> None:
    rgb, _, _ = _head_only_controller(
        tmp_path / "rgb", camera_modalities=("rgb",), frame_has_depth=False
    )
    rgbd, _, _ = _head_only_controller(tmp_path / "rgbd")
    for controller in (rgb, rgbd):
        controller.handle_input("On")
        controller.handle_input("Off")
    assert set(rgb.candidate_arrays) == set(rgbd.candidate_arrays)
    assert rgb.candidate_arrays["head_depth_frame_index"].shape == (
        rgb.candidate_arrays["timestamp_ns"].shape
    )
    rgb.handle_input("N")
    rgbd.handle_input("N")


def _sample_single_packet(packet: dict) -> dict[str, np.ndarray]:
    publish = int(packet["publish_monotonic_ns"])
    start = publish - 1_000  # packet 及其内部 source ts 都在 On 边界之后
    tick = publish + 1_000
    buffer = VlaRawBuffer()
    sampler = SynchronizedSampler(
        hz=30,
        start_monotonic_ns=start,
        telemetry=FakeTelemetry([packet]),
        head_camera=FakeCamera("head", [_frame("head", tick - 10, 1)]),
        wrist_camera=FakeCamera(
            "right_wrist", [_frame("right_wrist", tick - 10, 2)]
        ),
        buffer=buffer,
        media_writer=MemoryWriter(),
        thresholds=FreshnessThresholds(),
    )
    sampler.sample_tick(0, tick, tick)
    return buffer.arrays()


def test_desired17_valid_without_effective_backend_contract() -> None:
    timestamp = 2_000_000_000
    packet = _packet(timestamp)
    packet.update({
        "arm_qcmd_sent_rad": None,
        "arm_qcmd_sent_timestamp_ns": 0,
        "arm_command_valid": False,
        "hand_qcmd_effective_canonical_rad": None,
        "hand_qcmd_sdk_deg": None,
        "hand_qcmd_timestamp_ns": 0,
        "hand_command_valid": False,
    })
    arrays = _sample_single_packet(packet)
    assert arrays["robot_desired_qcmd_17d_valid"].tolist() == [True]
    np.testing.assert_allclose(
        arrays["robot_desired_qcmd_17d_rad"][0, :6],
        arrays["arm_q_goal_rad"][0],
    )
    np.testing.assert_allclose(
        arrays["robot_desired_qcmd_17d_rad"][0, 6:],
        arrays["hand_qcmd_retargeted_canonical_rad"][0],
    )
    assert arrays["robot_qcmd_17d_valid"].tolist() == [False]
    assert np.isnan(arrays["robot_qcmd_17d_rad"]).all()


@pytest.mark.parametrize(
    "missing_key",
    ["arm_q_goal_rad", "hand_qcmd_retargeted_rad"],
)
def test_desired17_invalid_if_either_desired_component_invalid(
    missing_key: str,
) -> None:
    packet = _packet(2_000_000_000)
    packet[missing_key] = None
    arrays = _sample_single_packet(packet)
    assert arrays["robot_desired_qcmd_17d_valid"].tolist() == [False]
    assert np.isnan(arrays["robot_desired_qcmd_17d_rad"]).all()


def test_desired_and_effective_are_allowed_to_differ(tmp_path: Path) -> None:
    controller, telemetry, _, _ = _controller(tmp_path)
    report = _record_with_post_on_packet(controller, telemetry)
    assert report.quality == "PASS", report.errors
    arrays = controller.candidate_arrays
    assert arrays["schema_version"].item() == SCHEMA_VERSION == 3
    assert arrays["robot_desired_qcmd_17d_valid"][1:].all()
    assert arrays["robot_qcmd_17d_valid"][1:].all()
    assert arrays["telemetry_publish_hz_requested"].item() == 100.0
    assert not np.array_equal(
        arrays["robot_desired_qcmd_17d_rad"][1:],
        arrays["robot_qcmd_17d_rad"][1:],
    )
    controller.handle_input("N")


def test_v3_validator_rejects_desired_provenance_and_ordering_corruption(
    tmp_path: Path,
) -> None:
    controller, telemetry, _, _ = _controller(tmp_path)
    _record_with_post_on_packet(controller, telemetry)
    corrupted = dict(controller.candidate_arrays)
    corrupted["robot_desired_qcmd_17d_rad"] = np.asarray(
        corrupted["robot_desired_qcmd_17d_rad"]
    ).copy()
    corrupted["robot_desired_qcmd_17d_rad"][1, 0] += 1.0
    corrupted["robot_joint_names"] = np.asarray(
        corrupted["robot_joint_names"]
    )[::-1]
    report = validate_episode(
        corrupted,
        media_root=controller.pending_dir,
        writer_drop_count=0,
        writer_error_count=0,
        telemetry_sequence_gap_count=0,
    )
    assert report.quality == "FAIL"
    assert any("ordering" in error for error in report.errors)
    assert any("前 6D" in error for error in report.errors)
    controller.handle_input("N")


def test_telemetry_effective_rate_uses_sequence_and_publish_timestamp() -> None:
    diagnostics = telemetry_rate_diagnostics({
        "timestamp_ns": np.asarray([
            1_010_000_000, 1_050_000_000, 1_070_000_000,
        ]),
        "telemetry_publish_sequence": np.asarray([10, 12, 13]),
        "telemetry_publish_timestamp_ns": np.asarray([
            1_000_000_000, 1_040_000_000, 1_060_000_000,
        ]),
        "telemetry_age_ns": np.asarray([
            10_000_000, 10_000_000, 10_000_000,
        ]),
    })
    assert diagnostics["telemetry_effective_hz"] == pytest.approx(50.0)
    assert diagnostics["telemetry_age_mean_ms"] == pytest.approx(10.0)
    assert diagnostics["telemetry_age_p95_ms"] == pytest.approx(10.0)
    assert diagnostics["telemetry_age_p99_ms"] == pytest.approx(10.0)
    assert diagnostics["telemetry_age_max_ms"] == pytest.approx(10.0)


def test_v2_episode_without_desired17_remains_readable(tmp_path: Path) -> None:
    controller, _, _ = _head_only_controller(
        tmp_path, camera_modalities=("rgb",), frame_has_depth=False
    )
    controller.handle_input("On")
    controller.handle_input("Off")
    v2 = dict(controller.candidate_arrays)
    v2["schema_version"] = np.asarray(2, dtype=np.int32)
    for key in (
        "robot_desired_qcmd_17d_rad", "robot_desired_qcmd_17d_valid",
        "robot_desired_qcmd_semantics", "robot_effective_qcmd_semantics",
        "robot_qpos_semantics", "telemetry_publish_timestamp_ns",
        "telemetry_publish_hz_requested", "telemetry_publish_hz_effective",
    ):
        v2.pop(key)
    report = validate_episode(
        v2,
        media_root=controller.pending_dir,
        writer_drop_count=0,
        writer_error_count=0,
        telemetry_sequence_gap_count=0,
    )
    assert report.quality == "PASS", report.errors
    controller.handle_input("N")
