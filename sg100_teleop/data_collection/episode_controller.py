"""显式 On/Off/Y/N/Reset raw episode state machine。"""

from __future__ import annotations

from enum import Enum
from pathlib import Path
import time
import uuid

import numpy as np

from sg100_teleop.data_collection.synchronized_sampler import SynchronizedSampler
from sg100_teleop.data_collection.vla_raw_buffer import VlaRawBuffer
from sg100_teleop.data_collection.vla_raw_schema import (
    DATASET_STATUS,
    ROBOT_JOINT_NAMES,
    SCHEMA_NAME,
    SCHEMA_VERSION,
    TRAINING_READY,
    FreshnessThresholds,
    scan_next_episode_index,
)
from sg100_teleop.data_collection.vla_raw_validator import (
    ValidationReport,
    validate_episode,
)
from sg100_teleop.data_collection.vla_raw_writer import (
    BoundedMediaWriter,
    discard_pending,
    save_episode_atomic,
)


class EpisodeState(str, Enum):
    READY = "READY"
    RECORDING = "RECORDING"
    PENDING_CONFIRMATION = "PENDING_CONFIRMATION"


class EpisodeController:
    def __init__(
        self,
        *,
        output_dir: str | Path,
        hz: float,
        language_instruction: str,
        telemetry,
        head_camera,
        wrist_camera,
        thresholds: FreshnessThresholds | None = None,
        writer_queue_size: int = 180,
    ) -> None:
        instruction = str(language_instruction).strip()
        if not instruction:
            raise ValueError("language instruction 不能为空")
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.hz = float(hz)
        if not np.isfinite(self.hz) or self.hz <= 0:
            raise ValueError("hz 必须 > 0")
        self.language_instruction = instruction
        self.telemetry = telemetry
        self.head_camera = head_camera
        self.wrist_camera = wrist_camera
        self.thresholds = thresholds or FreshnessThresholds()
        self.writer_queue_size = int(writer_queue_size)
        self.state = EpisodeState.READY
        self.next_episode_index = scan_next_episode_index(self.output_dir)
        self.frozen_instruction: str | None = None
        self.episode_start_monotonic_ns = 0
        self.episode_start_wall_ns = 0
        self.episode_stop_request_monotonic_ns = 0
        self.pending_dir: Path | None = None
        self.buffer: VlaRawBuffer | None = None
        self.media_writer: BoundedMediaWriter | None = None
        self.sampler: SynchronizedSampler | None = None
        self.candidate_arrays: dict[str, np.ndarray] | None = None
        self.validation_report: ValidationReport | None = None
        self._telemetry_stats_start: dict[str, int] = {}

    def turn_on(self) -> int:
        if self.state is not EpisodeState.READY:
            raise RuntimeError(f"On 仅在 READY 可用，当前 {self.state.value}")
        if not self.head_camera.ready:
            raise RuntimeError("HEAD D455 stream 未 READY")
        if not self.wrist_camera.ready:
            raise RuntimeError("WRIST D405 stream 未 READY")
        if not self.telemetry.is_alive():
            raise RuntimeError("Robot telemetry publisher 未 READY或已 stale")
        self.frozen_instruction = str(self.language_instruction)
        self.buffer = VlaRawBuffer()
        self.pending_dir = self.output_dir / f".pending_{uuid.uuid4().hex}"
        self.episode_start_monotonic_ns = time.monotonic_ns()
        self.episode_start_wall_ns = time.time_ns()
        self.episode_stop_request_monotonic_ns = 0
        self.candidate_arrays = None
        self.validation_report = None
        self._telemetry_stats_start = dict(self.telemetry.stats)
        try:
            self.media_writer = BoundedMediaWriter(
                self.pending_dir, queue_size=self.writer_queue_size
            )
            self.sampler = SynchronizedSampler(
                hz=self.hz,
                start_monotonic_ns=self.episode_start_monotonic_ns,
                telemetry=self.telemetry,
                head_camera=self.head_camera,
                wrist_camera=self.wrist_camera,
                buffer=self.buffer,
                media_writer=self.media_writer,
                thresholds=self.thresholds,
            )
            # Materialize tick 0 before returning RECORDING so even an
            # immediate operator Off has one coherent all-modality row.
            self.sampler.sample_tick(
                0,
                self.episode_start_monotonic_ns,
                time.monotonic_ns(),
            )
            self.sampler.start(start_index=1)
        except Exception:
            if self.media_writer is not None:
                self.media_writer.close()
            if self.pending_dir is not None:
                discard_pending(self.pending_dir)
            self._clear_candidate_state()
            raise
        self.state = EpisodeState.RECORDING
        return self.next_episode_index

    @staticmethod
    def _camera_metadata(prefix: str, camera) -> dict[str, np.ndarray]:
        metadata = camera.metadata()
        return {
            f"{prefix}_camera_model": np.asarray(str(metadata["model"])),
            f"{prefix}_camera_serial": np.asarray(str(metadata["serial"])),
            f"{prefix}_camera_width": np.asarray(int(metadata["width"]), dtype=np.int32),
            f"{prefix}_camera_height": np.asarray(int(metadata["height"]), dtype=np.int32),
            f"{prefix}_camera_fps": np.asarray(int(metadata["fps"]), dtype=np.int32),
            f"{prefix}_depth_scale": np.asarray(float(metadata["depth_scale"]), dtype=np.float64),
            f"{prefix}_color_intrinsics": np.asarray(metadata["color_intrinsics"], dtype=np.float64),
            f"{prefix}_depth_intrinsics": np.asarray(metadata["depth_intrinsics"], dtype=np.float64),
            f"{prefix}_color_format": np.asarray(str(metadata.get("color_format", "RGB8"))),
            f"{prefix}_depth_format": np.asarray(str(metadata.get("depth_format", "Z16_UINT16"))),
            f"{prefix}_color_stream_profile": np.asarray(str(metadata.get("color_stream_profile", ""))),
            f"{prefix}_depth_stream_profile": np.asarray(str(metadata.get("depth_stream_profile", ""))),
            f"{prefix}_depth_aligned_to": np.asarray(str(metadata.get("depth_aligned_to", "color"))),
            f"{prefix}_extrinsics_to_robot": np.asarray(str(metadata.get(
                "extrinsics_to_robot", "UNAVAILABLE / NOT_CALIBRATED"
            ))),
        }

    def turn_off(self) -> ValidationReport:
        if self.state is not EpisodeState.RECORDING:
            raise RuntimeError(f"Off 仅在 RECORDING 可用，当前 {self.state.value}")
        # Stop request timestamp is captured before stopping; no new tick may
        # be created once sampler.stop() returns.
        self.episode_stop_request_monotonic_ns = time.monotonic_ns()
        assert self.sampler is not None and self.media_writer is not None
        assert self.buffer is not None and self.pending_dir is not None
        self.sampler.stop()
        self.media_writer.close()
        arrays = self.buffer.arrays()
        arrays.update({
            "schema_name": np.asarray(SCHEMA_NAME),
            "schema_version": np.asarray(SCHEMA_VERSION, dtype=np.int32),
            "episode_index": np.asarray(self.next_episode_index, dtype=np.int64),
            "language_instruction": np.asarray(self.frozen_instruction),
            "dataset_hz": np.asarray(self.hz, dtype=np.float64),
            "episode_start_monotonic_ns": np.asarray(self.episode_start_monotonic_ns, dtype=np.int64),
            "episode_start_wall_ns": np.asarray(self.episode_start_wall_ns, dtype=np.int64),
            "episode_stop_request_monotonic_ns": np.asarray(
                self.episode_stop_request_monotonic_ns, dtype=np.int64
            ),
            "robot_joint_names": np.asarray(ROBOT_JOINT_NAMES),
            "dataset_status": np.asarray(DATASET_STATUS),
            "training_ready": np.asarray(TRAINING_READY, dtype=bool),
            "dual_camera_synchronization": np.asarray(
                "HOST MONOTONIC SOFTWARE ALIGNMENT"
            ),
            "camera_device_clock_domains_shared": np.asarray(False, dtype=bool),
            "sg100_feedback_map_status": np.asarray("F_FB_PENDING"),
            "media_writer_max_backlog": np.asarray(
                self.media_writer.max_backlog, dtype=np.int64
            ),
            "media_writer_drop_count": np.asarray(
                self.media_writer.drop_count, dtype=np.int64
            ),
            "media_writer_error_count": np.asarray(
                self.media_writer.error_count, dtype=np.int64
            ),
        })
        arrays.update(self.thresholds.as_metadata())
        arrays.update(self._camera_metadata("head", self.head_camera))
        arrays.update(self._camera_metadata("wrist", self.wrist_camera))
        stats = self.telemetry.stats
        episode_received = max(
            0,
            int(stats["received_packets"])
            - int(self._telemetry_stats_start.get("received_packets", 0)),
        )
        episode_gaps = max(
            0,
            int(stats["sequence_gap_count"])
            - int(self._telemetry_stats_start.get("sequence_gap_count", 0)),
        )
        arrays["telemetry_received_packets"] = np.asarray(
            episode_received, dtype=np.int64
        )
        arrays["telemetry_sequence_gap_count"] = np.asarray(
            episode_gaps, dtype=np.int64
        )
        report = validate_episode(
            arrays,
            media_root=self.pending_dir,
            writer_drop_count=self.media_writer.drop_count,
            writer_error_count=self.media_writer.error_count,
            telemetry_sequence_gap_count=episode_gaps,
        )
        arrays["validation_quality"] = np.asarray(report.quality)
        self.candidate_arrays = arrays
        self.validation_report = report
        self.state = EpisodeState.PENDING_CONFIRMATION
        return report

    def confirm(self, decision: str) -> tuple[Path, Path] | None:
        if self.state is not EpisodeState.PENDING_CONFIRMATION:
            raise RuntimeError("Y/N/Reset 仅在 PENDING_CONFIRMATION 可用")
        choice = str(decision).strip().lower()
        if choice == "y":
            assert self.candidate_arrays is not None and self.pending_dir is not None
            saved = save_episode_atomic(
                self.output_dir,
                self.next_episode_index,
                self.candidate_arrays,
                self.pending_dir,
            )
            self.next_episode_index += 1
            self._clear_candidate_state()
            self.state = EpisodeState.READY
            return saved
        if choice in ("n", "reset"):
            assert self.pending_dir is not None
            discard_pending(self.pending_dir)
            if choice == "reset":
                self.telemetry.clear()
                self.head_camera.clear()
                self.wrist_camera.clear()
            self._clear_candidate_state()
            self.state = EpisodeState.READY
            return None
        raise ValueError("PENDING_CONFIRMATION 只接受 Y、N 或 Reset")

    def handle_input(self, command: str):
        value = str(command).strip().lower()
        if self.state is EpisodeState.READY:
            if value == "on":
                return self.turn_on()
            return None
        if self.state is EpisodeState.RECORDING:
            if value == "off":
                return self.turn_off()
            return None
        if value in ("y", "n", "reset"):
            return self.confirm(value)
        return None

    def abort_pending(self) -> None:
        if self.sampler is not None:
            self.sampler.stop()
        if self.media_writer is not None:
            self.media_writer.close()
        if self.pending_dir is not None:
            discard_pending(self.pending_dir)
        self._clear_candidate_state()
        self.state = EpisodeState.READY

    def _clear_candidate_state(self) -> None:
        self.frozen_instruction = None
        self.episode_start_monotonic_ns = 0
        self.episode_start_wall_ns = 0
        self.episode_stop_request_monotonic_ns = 0
        self.pending_dir = None
        self.buffer = None
        self.media_writer = None
        self.sampler = None
        self.candidate_arrays = None
        self.validation_report = None
        self._telemetry_stats_start = {}
