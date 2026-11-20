"""Offline regressions for long sessions, congested links and slow log disks."""

from __future__ import annotations

import json
import subprocess
import sys
from threading import Event, Lock
import time
import tracemalloc

import numpy as np
import pytest

from avp_stream.streamer import VisionProStreamer
from sg100_teleop.data_collection.collector import ArmHandDataCollector
from sg100_teleop.recording.real_hardware_error_recorder import RealHardwareErrorRecorder
from sg100_teleop.recording.session_spool import ArraySpool, SessionSpool
from sg100_teleop.robot.rm65_real import RM65RealBackend
from sg100_teleop.tests.test_robot_backend_units import FakeSdkArm


def pose_streamer():
    streamer = VisionProStreamer.__new__(VisionProStreamer)
    streamer._connection_shutdown = Event()
    streamer._pose_send_lock = Lock()
    streamer._pose_send_pending = False
    streamer._pose_send_dropped = 0
    streamer._pose_send_errors = 0
    streamer._pose_buffer_limit = 100

    class Loop:
        def __init__(self):
            self.callbacks = []

        def call_soon_threadsafe(self, fn):
            self.callbacks.append(fn)

    class Channel:
        readyState = "open"
        bufferedAmount = 0

        def __init__(self):
            self.sent = []

        def send(self, message):
            self.sent.append(message)

    streamer._webrtc_loop = Loop()
    streamer._webrtc_sim_channel = Channel()
    return streamer


def test_pose_callbacks_and_network_buffer_are_bounded():
    streamer = pose_streamer()
    for _ in range(1000):
        streamer._queue_pose_message(b"pose")
    assert len(streamer._webrtc_loop.callbacks) == 1
    streamer._webrtc_loop.callbacks.pop()()
    assert streamer._webrtc_sim_channel.sent == [b"pose"]
    assert streamer._pose_send_dropped == 999

    streamer._webrtc_sim_channel.bufferedAmount = 99
    streamer._queue_pose_message(b"pose")
    streamer._webrtc_loop.callbacks.pop()()
    assert streamer._webrtc_sim_channel.sent == [b"pose"]
    assert not streamer._pose_send_pending
    streamer._webrtc_sim_channel.bufferedAmount = 0
    streamer._queue_pose_message(b"newest")
    streamer._webrtc_loop.callbacks.pop()()
    assert streamer._webrtc_sim_channel.sent[-1] == b"newest"


def test_async_pose_send_exception_is_contained_and_next_send_recovers():
    streamer = pose_streamer()

    def failed_send(message):
        raise RuntimeError("channel closed during callback")

    streamer._webrtc_sim_channel.send = failed_send
    streamer._queue_pose_message(b"pose")
    streamer._webrtc_loop.callbacks.pop()()
    assert streamer._pose_send_errors == 1
    assert not streamer._pose_send_pending
    streamer._connection_shutdown.set()
    assert streamer._queue_pose_message(b"after cleanup") is False


def test_pending_pose_is_replaced_with_latest_snapshot():
    streamer = pose_streamer()
    streamer._queue_pose_message(b"old")
    streamer._queue_pose_message(b"newest")
    assert len(streamer._webrtc_loop.callbacks) == 1
    streamer._webrtc_loop.callbacks.pop()()
    assert streamer._webrtc_sim_channel.sent == [b"newest"]


def test_reconnect_never_blocks_get_latest_or_starts_duplicate_workers():
    streamer = VisionProStreamer.__new__(VisionProStreamer)
    streamer._connection_shutdown = Event()
    streamer._connection_request_lock = Lock()
    streamer._reconnect_thread = None
    streamer._cross_network_mode = True
    streamer._webrtc_connected = False
    streamer._latest_lock = Lock()
    streamer.latest = None
    entered, release = Event(), Event()
    calls = []

    def slow_connect():
        calls.append(1)
        entered.set()
        assert release.wait(2)

    streamer._ensure_cross_network_connected = slow_connect
    try:
        started = time.monotonic()
        for _ in range(1000):
            assert streamer.get_latest() is None
        assert time.monotonic() - started < 0.5
        assert entered.wait(1)
        assert len(calls) == 1
    finally:
        release.set()
        streamer._reconnect_thread.join(1)


def test_connection_wait_has_deadline_and_honors_shutdown():
    streamer = VisionProStreamer.__new__(VisionProStreamer)
    streamer._connection_timeout = 0.02
    streamer._connection_shutdown = Event()
    with pytest.raises(TimeoutError):
        streamer._wait_for_connection(lambda: False, "test")
    streamer._connection_shutdown.set()
    with pytest.raises(ConnectionError):
        streamer._wait_for_connection(lambda: False, "test")
    streamer._wait_for_connection(lambda: True, "already ready")


def test_connection_lock_allows_signal_cleanup_to_reenter(monkeypatch):
    # No sockets or receive threads: exercise constructor-owned synchronization.
    monkeypatch.setattr(VisionProStreamer, "_start_info_server", lambda self: None)
    monkeypatch.setattr(VisionProStreamer, "_start_hand_tracking", lambda self: None)
    streamer = VisionProStreamer(ip="127.0.0.1", record=False)
    try:
        with streamer._connection_request_lock:
            acquired = streamer._connection_request_lock.acquire(blocking=False)
            if acquired:
                streamer._connection_request_lock.release()
            assert acquired
            # SIGINT cleanup may run while get_latest owns this same lock.
            streamer.cleanup()
    finally:
        streamer.cleanup()


def test_repeated_signaling_timeouts_reuse_one_worker(monkeypatch):
    import avp_stream.streamer as module

    created = []

    class ThreadStub:
        def __init__(self, **kwargs):
            created.append(self)

        def start(self):
            pass

        def is_alive(self):
            return True

    streamer = VisionProStreamer.__new__(VisionProStreamer)
    streamer._connection_shutdown = Event()
    streamer._connection_request_lock = Lock()
    streamer._signaling_thread = None
    streamer._signaling_connected = False
    streamer._log = lambda *args, **kwargs: None

    def timed_out(*args):
        raise TimeoutError("offline")

    streamer._wait_for_connection = timed_out
    monkeypatch.setattr(module, "Thread", ThreadStub)
    for _ in range(4):
        with pytest.raises(TimeoutError):
            streamer._start_signaling_connection()
    assert len(created) == 1


@pytest.mark.parametrize("record_events", [False, True])
def test_command_archive_flag_keeps_live_commands_and_feedback(record_events):
    fake = FakeSdkArm(initial_deg=[0.0] * 6)
    backend = RM65RealBackend(
        "fake", sdk=fake, record_command_events=record_events,
        target_timeout_s=1, feedback_timeout_s=1,
        on_event=lambda *args: None,
    )
    backend.connect()
    backend.start()
    backend.set_enabled(True)
    try:
        deadline = time.monotonic() + 2
        while backend._send_count < 5 and time.monotonic() < deadline:
            backend.send_joint_target(np.full(6, 0.01))
            time.sleep(0.01)
        assert backend._send_count >= 5
        assert backend.latest_command_sent() is not None
        assert backend.joint_state() is not None
        events = backend.drain_command_events()
        assert bool(events) == record_events
        assert bool(fake.canfd_calls)
    finally:
        backend.set_enabled(False)
        backend.disconnect()


def test_disk_collections_preserve_order_lookup_and_all_records(tmp_path):
    store = SessionSpool(tmp_path)
    mapping, sequence, seen = store.mapping("map"), store.sequence("rows"), store.set("seen")
    for index in [300, 1, 256, 2]:
        mapping[index] = {"number": index}
        sequence.append(np.array([index], dtype=float))
        seen.add((index, 4))
    mapping[1] = {"number": -1}
    assert mapping[np.int64(1)] == {"number": -1}
    assert mapping[1.0] == {"number": -1}
    assert list(mapping) == [300, 1, 256, 2]
    assert mapping[1] == {"number": -1}
    np.testing.assert_array_equal(sequence[-1], [2])
    assert len(sequence[:2]) == 2
    assert (256, 4) in seen
    seen.discard((256, 4))
    assert (256, 4) not in seen
    assert list(mapping.items())[0] == (300, {"number": 300})
    store.close()


def test_long_session_spool_does_not_retain_all_numpy_arrays(tmp_path):
    store = SessionSpool(tmp_path)
    rows = store.sequence("rows")
    tracemalloc.start()
    try:
        for index in range(4000):
            rows.append(np.full(512, index, dtype=np.float64))
        _, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    # The complete payload is >15 MiB. Only a row/SQLite operation is in RAM.
    assert peak < 2 * 1024 * 1024
    assert len(rows) == 4000
    np.testing.assert_array_equal(rows[3999], np.full(512, 3999))
    store.close()


def test_array_spool_keeps_every_row_and_exposes_mmap(tmp_path):
    rows = ArraySpool(tmp_path, 32)
    for index in range(3000):
        row = np.full(32, index, dtype=float)
        rows.append(row)
        row[:] = -1
    view = rows.view()
    assert isinstance(view, np.memmap)
    assert view.shape == (3000, 32)
    np.testing.assert_array_equal(view[1234], np.full(32, 1234))
    view._mmap.close()
    rows.close()


def test_collector_finalizes_all_raw_events_without_one_large_json_string(tmp_path):
    collector = ArmHandDataCollector(tmp_path, started_timestamp_ns=1)
    for index in range(4000):
        collector._raw("test", index + 1, {"array": np.arange(17) + index})
    collector.finalize()
    with (tmp_path / "arm_hand_raw_events.jsonl").open() as handle:
        for index, line in enumerate(handle):
            record = json.loads(line)
            assert record["sequence_id"] == index + 1
            assert record["payload"]["array"][0] == index
    assert index == 3999
    assert collector.finalize() == tmp_path / "arm_hand_joint17_v1.npz"


def test_background_error_log_keeps_normal_records_and_order(tmp_path):
    recorder = RealHardwareErrorRecorder(tmp_path, background=True)
    for index in range(100):
        recorder.record(severity="INFO", component="test", event="TICK", message=str(index))
    assert recorder.finish(outcome="complete")
    records = [json.loads(line) for line in recorder.path.read_text().splitlines()]
    assert [row["sequence"] for row in records] == list(range(1, 103))
    assert [row["message"] for row in records[1:-1]] == [str(i) for i in range(100)]


def test_slow_log_disk_never_blocks_producer_and_overflow_is_reported(tmp_path, monkeypatch):
    entered, release = Event(), Event()
    original = RealHardwareErrorRecorder._write_payload

    def blocked_write(self, payload):
        entered.set()
        assert release.wait(3)
        original(self, payload)

    monkeypatch.setattr(RealHardwareErrorRecorder, "_write_payload", blocked_write)
    recorder = RealHardwareErrorRecorder(tmp_path, background=True, queue_size=2)
    try:
        assert entered.wait(1)
        started = time.monotonic()
        for index in range(1000):
            recorder.record(severity="ERROR", component="test", event="ERROR", message=str(index))
        assert time.monotonic() - started < 0.5
        assert recorder._queue.qsize() == 2
        assert recorder.dropped_records == 998
    finally:
        release.set()
        assert not recorder.finish(outcome="complete")
    records = [json.loads(line) for line in recorder.path.read_text().splitlines()]
    assert records[-1]["event"] == "DIAGNOSTIC_LOG_INCOMPLETE"


def test_exception_identity_cache_is_bounded(tmp_path):
    recorder = RealHardwareErrorRecorder(tmp_path)
    for index in range(100):
        recorder.record_exception(component="test", event="ERROR", exception=ValueError(str(index)))
    assert len(recorder._logged_exceptions) == 64


def test_background_log_is_flushed_at_process_exit_without_finish(tmp_path):
    source = """
import sys
from pathlib import Path
from sg100_teleop.recording.real_hardware_error_recorder import RealHardwareErrorRecorder
recorder = RealHardwareErrorRecorder(Path(sys.argv[1]), background=True)
recorder.record_exception(component='startup', event='ERROR', exception=RuntimeError('offline'))
"""
    subprocess.run([sys.executable, "-B", "-c", source, str(tmp_path)], check=True, timeout=10)
    records = [json.loads(line) for line in
               (tmp_path / "real_hardware_errors.jsonl").read_text().splitlines()]
    assert records[-1]["event"] == "ERROR"
    assert records[-1]["details"]["exception_message"] == "offline"
