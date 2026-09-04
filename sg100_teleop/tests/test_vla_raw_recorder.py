"""30 Hz raw VLA recorder 纯软件测试；不得连接任何硬件。"""

from __future__ import annotations

from pathlib import Path
import socket
import threading
import time

import numpy as np
import pytest

from sg100_teleop.camera.realsense_camera import RealSenseFrame
from sg100_teleop.data_collection.episode_controller import (
    EpisodeController,
    EpisodeState,
)
from sg100_teleop.data_collection.synchronized_sampler import SynchronizedSampler
from sg100_teleop.data_collection.vla_raw_buffer import VlaRawBuffer
from sg100_teleop.data_collection.vla_raw_schema import (
    DATASET_STATUS,
    FixedRateTimeline,
    FreshnessThresholds,
    scan_next_episode_index,
)
from sg100_teleop.data_collection.vla_raw_writer import (
    BoundedMediaWriter,
    save_episode_atomic,
)
from sg100_teleop.telemetry.teleop_telemetry import (
    TeleopTelemetryPublisher,
    TeleopTelemetrySubscriber,
)


def _packet(timestamp_ns: int, sequence: int = 1) -> dict:
    return {
        "schema_version": 1,
        "publish_sequence": sequence,
        "publish_monotonic_ns": timestamp_ns,
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
    telemetry = FakeTelemetry([_packet(tick - 10)])
    head = FakeCamera("head", [_frame("head", tick - 50, 1), _frame("head", tick + 1, 2)])
    wrist = FakeCamera("right_wrist", [_frame("right_wrist", tick - 40, 3)])
    buffer = VlaRawBuffer()
    writer = MemoryWriter()
    sampler = SynchronizedSampler(
        hz=30,
        start_monotonic_ns=tick,
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


def _controller(tmp_path: Path):
    timestamp = time.monotonic_ns() - 1_000_000
    telemetry = FakeTelemetry([_packet(timestamp)])
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
        assert archive["dataset_status"].item() == DATASET_STATUS
        assert archive["training_ready"].item() is False
        assert archive["timestamp_ns"].shape[0] == archive["arm_qpos_rad"].shape[0]
        assert archive["hand_feedback_canonical_valid"].sum() == 0
        assert np.isnan(archive["hand_qpos_canonical_rad"]).all()
        assert archive["robot_joint_names"].shape == (17,)
    assert len(tuple((saved[1] / "head" / "depth").glob("*.png"))) >= 1


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
