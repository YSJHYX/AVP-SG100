"""Simulation-time sampler for lightweight MuJoCo visual replay state."""

from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from sg100_teleop.sim.camera_observations import (
    CAMERA_TO_OBSERVATION_KEY,
    validate_camera_observation_contract,
)
from sg100_teleop.sim.combined_robot_layout import (
    HAND_DOF,
    SG100_JOINT_ORDER,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CAMERA = "workstation_front"
DEFAULT_RECORD_FPS = 30.0

# Explicit provenance for the recorded hand command/state fields.  The
# command is the AUTHORITATIVE canonical hand command consumed by the
# backends (post-retarget, post-filter); the qpos slice is the MuJoCo
# simulation RESPONSE — they are different quantities even when close.
HAND_COMMAND_UNIT = "rad"
HAND_COMMAND_DOMAIN = "canonical"
HAND_COMMAND_SOURCE = "SG100Retargeter final backend input (post-filter)"
HAND_QPOS_SOURCE = "MuJoCo simulation state"


class MujocoStateRecorder:
    """Record visual state at a clock independent of physics and tracking.

    Sampling is driven only by ``data.time``. Runtime recording copies qpos,
    mocap_pos, and mocap_quat into memory; it never renders or encodes video.
    The complete session is written once during :meth:`stop`.

    Hand command recording (optional, enabled by ``hand_qpos_indices``):
    ``set_hand_command`` latches the authoritative canonical 11D hand
    command at physics rate; each sampled frame stores the latest latched
    value alongside the MuJoCo hand qpos slice, so the command and the
    simulated state are explicit and never inferred from each other.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        *,
        scene_path: str | Path,
        recording_dir: str | Path = "recording",
        record_fps: float = DEFAULT_RECORD_FPS,
        camera_name: str = DEFAULT_CAMERA,
        repository_root: str | Path = REPOSITORY_ROOT,
        hand_qpos_indices: np.ndarray | None = None,
        hand_joint_names: tuple[str, ...] = SG100_JOINT_ORDER,
    ) -> None:
        if not np.isfinite(record_fps) or record_fps <= 0.0:
            raise ValueError("record_fps must be finite and > 0")

        self.model = model
        validate_camera_observation_contract(model)
        self.record_fps = float(record_fps)
        self.period = 1.0 / self.record_fps
        self.camera_name = str(camera_name)
        self.repository_root = Path(repository_root).resolve()
        self.scene_path = Path(scene_path).resolve()
        self.scene_identity = self._scene_identity(self.scene_path)

        self.hand_qpos_indices = (
            None
            if hand_qpos_indices is None
            else np.asarray(hand_qpos_indices, dtype=np.int64).copy()
        )
        if self.hand_qpos_indices is not None:
            if self.hand_qpos_indices.shape != (HAND_DOF,):
                raise ValueError(
                    "hand_qpos_indices must have shape (11,), got "
                    f"{self.hand_qpos_indices.shape}"
                )
            if np.any(self.hand_qpos_indices < 0) or np.any(
                self.hand_qpos_indices >= model.nq
            ):
                raise ValueError("hand_qpos_indices outside qpos range")
        self.hand_joint_names = tuple(str(n) for n in hand_joint_names)
        if len(self.hand_joint_names) != HAND_DOF:
            raise ValueError("hand_joint_names must have 11 entries")

        root = Path(recording_dir)
        if not root.is_absolute():
            root = self.repository_root / root
        self.recording_root = root.resolve()

        self.session_dir: Path | None = None
        self.output_path: Path | None = None
        self.metadata_path: Path | None = None
        self._wall_start: datetime | None = None
        self._simulation_start: float | None = None
        self._next_capture_time: float | None = None
        self._timestamps: list[float] = []
        self._qpos: list[np.ndarray] = []
        self._mocap_pos: list[np.ndarray] = []
        self._mocap_quat: list[np.ndarray] = []
        self._hand_qcmd: list[np.ndarray] = []
        self._hand_qpos: list[np.ndarray] = []
        self._latest_hand_qcmd: np.ndarray | None = None
        self._started = False
        self._stopped = False

    def _scene_identity(self, path: Path) -> str:
        try:
            return path.relative_to(self.repository_root).as_posix()
        except ValueError:
            return str(path)

    def _create_session_dir(self, wall_start: datetime) -> Path:
        self.recording_root.mkdir(parents=True, exist_ok=True)
        stem = wall_start.strftime("%Y%m%d_%H%M%S")
        for suffix in range(1000):
            name = stem if suffix == 0 else f"{stem}_{suffix:02d}"
            candidate = self.recording_root / name
            try:
                candidate.mkdir()
            except FileExistsError:
                continue
            return candidate
        raise RuntimeError(f"Could not create a unique session below {self.recording_root}")

    def start(self, data: mujoco.MjData) -> Path:
        if self._started:
            raise RuntimeError("MuJoCo state recorder has already started")
        self._validate_data(data)

        self._wall_start = datetime.now().astimezone()
        self.session_dir = self._create_session_dir(self._wall_start)
        self.output_path = self.session_dir / "mujoco_recording.npz"
        self.metadata_path = self.session_dir / "metadata.json"
        self._simulation_start = float(data.time)
        self._next_capture_time = self._simulation_start
        self._started = True

        # Preserve the initial visual state at the session's simulation time.
        self.maybe_record(data)
        return self.session_dir

    def _validate_data(self, data: mujoco.MjData) -> None:
        if data.qpos.shape != (self.model.nq,):
            raise ValueError(
                f"qpos shape {data.qpos.shape} does not match model.nq={self.model.nq}"
            )
        if data.mocap_pos.shape != (self.model.nmocap, 3):
            raise ValueError("mocap_pos shape does not match model.nmocap")
        if data.mocap_quat.shape != (self.model.nmocap, 4):
            raise ValueError("mocap_quat shape does not match model.nmocap")

    def set_hand_command(
        self,
        canonical_qcmd_rad: np.ndarray,
    ) -> None:
        """Latch the authoritative canonical hand command (physics rate).

        The value stored here is the exact 11D vector consumed by the
        MuJoCo backend and the SG100 real backend (post-retarget,
        post-filter, pre-F_cmd).  It is copied immediately so later
        in-place mutation upstream can never rewrite history.
        """

        q = np.asarray(canonical_qcmd_rad, dtype=np.float64)
        if q.shape != (HAND_DOF,):
            raise ValueError(
                f"canonical hand command must have shape (11,), got {q.shape}"
            )
        self._latest_hand_qcmd = q.copy()

    def maybe_record(self, data: mujoco.MjData) -> bool:
        """Capture at most one state when simulation time reaches the deadline."""

        if not self._started or self._stopped:
            raise RuntimeError("MuJoCo state recorder is not active")
        self._validate_data(data)
        assert self._next_capture_time is not None

        simulation_time = float(data.time)
        if not np.isfinite(simulation_time):
            raise ValueError("data.time must be finite")
        if simulation_time + 1e-12 < self._next_capture_time:
            return False

        self._timestamps.append(simulation_time)
        self._qpos.append(data.qpos.copy())
        self._mocap_pos.append(data.mocap_pos.copy())
        self._mocap_quat.append(data.mocap_quat.copy())
        if self.hand_qpos_indices is not None:
            self._hand_qpos.append(
                data.qpos[self.hand_qpos_indices].copy()
            )
            self._hand_qcmd.append(
                self._latest_hand_qcmd.copy()
                if self._latest_hand_qcmd is not None
                else np.full(HAND_DOF, np.nan)
            )

        # Normally the 2 ms physics step crosses at most one 30 Hz deadline.
        # Advance through every missed deadline without duplicating a state if
        # a caller presents a larger time jump.
        while self._next_capture_time <= simulation_time + 1e-12:
            self._next_capture_time += self.period
        return True

    @property
    def frame_count(self) -> int:
        return len(self._timestamps)

    def _metadata(self) -> dict[str, Any]:
        timestamps = np.asarray(self._timestamps, dtype=np.float64)
        recorded_span = (
            float(timestamps[-1] - timestamps[0]) if timestamps.size > 1 else 0.0
        )
        return {
            "schema_version": 1,
            "scene_path": self.scene_identity,
            "camera_name": self.camera_name,
            "camera_observations": dict(CAMERA_TO_OBSERVATION_KEY),
            "record_fps": self.record_fps,
            "mujoco_timestep": float(self.model.opt.timestep),
            "frame_count": self.frame_count,
            "qpos_dimension": int(self.model.nq),
            "mocap_count": int(self.model.nmocap),
            "wall_start_time": (
                self._wall_start.isoformat() if self._wall_start is not None else None
            ),
            "simulation_start_time": self._simulation_start,
            "simulation_end_time": (
                float(timestamps[-1]) if timestamps.size else self._simulation_start
            ),
            "recorded_time_span": recorded_span,
            "video_duration": self.frame_count / self.record_fps,
            "state_fields": ["timestamps", "qpos", "mocap_pos", "mocap_quat"],
            "audio": False,
            **self._hand_metadata(),
        }

    def _hand_metadata(self) -> dict[str, Any]:
        """Hand command/state provenance (present only when enabled)."""

        if self.hand_qpos_indices is None:
            return {}

        missing_command_frames = sum(
            1
            for sample in self._hand_qcmd
            if not np.all(np.isfinite(sample))
        )
        return {
            "hand_dof": int(HAND_DOF),
            "hand_joint_names": list(self.hand_joint_names),
            "hand_qpos_slice": self.hand_qpos_indices.tolist(),
            "hand_command_unit": HAND_COMMAND_UNIT,
            "hand_command_domain": HAND_COMMAND_DOMAIN,
            "hand_command_source": HAND_COMMAND_SOURCE,
            "hand_qpos_source": HAND_QPOS_SOURCE,
            "hand_command_semantics": (
                "command != MuJoCo qpos: qpos is the simulation response "
                "to this command, not the command itself"
            ),
            "hand_command_missing_frames": missing_command_frames,
        }

    def stop(self) -> Path:
        """Finalize this session once and return ``mujoco_recording.npz``."""

        if not self._started:
            raise RuntimeError("MuJoCo state recorder was never started")
        if self._stopped:
            assert self.output_path is not None
            return self.output_path
        assert self.output_path is not None
        assert self.metadata_path is not None

        timestamps = np.asarray(self._timestamps, dtype=np.float64)
        qpos = np.stack(self._qpos).astype(np.float64, copy=False)
        mocap_pos = np.stack(self._mocap_pos).astype(np.float64, copy=False)
        mocap_quat = np.stack(self._mocap_quat).astype(np.float64, copy=False)
        metadata = self._metadata()

        hand_arrays: dict[str, np.ndarray] = {}
        if self.hand_qpos_indices is not None:
            hand_arrays["hand_qcmd_canonical_rad"] = (
                np.stack(self._hand_qcmd).astype(np.float64, copy=False)
            )
            hand_arrays["hand_qpos_mujoco_rad"] = (
                np.stack(self._hand_qpos).astype(np.float64, copy=False)
            )

        temporary_output = self.output_path.with_name(
            f".{self.output_path.name}.tmp"
        )
        with temporary_output.open("wb") as handle:
            np.savez_compressed(
                handle,
                timestamps=timestamps,
                qpos=qpos,
                mocap_pos=mocap_pos,
                mocap_quat=mocap_quat,
                hand_joint_names=np.asarray(self.hand_joint_names),
                hand_command_unit=np.asarray(HAND_COMMAND_UNIT),
                hand_command_domain=np.asarray(HAND_COMMAND_DOMAIN),
                scene_path=np.asarray(self.scene_identity),
                camera_name=np.asarray(self.camera_name),
                camera_names=np.asarray(tuple(CAMERA_TO_OBSERVATION_KEY)),
                camera_observation_keys=np.asarray(
                    tuple(CAMERA_TO_OBSERVATION_KEY.values())
                ),
                record_fps=np.asarray(self.record_fps, dtype=np.float64),
                mujoco_timestep=np.asarray(self.model.opt.timestep, dtype=np.float64),
                wall_start_time=np.asarray(metadata["wall_start_time"]),
                **hand_arrays,
            )
        temporary_output.replace(self.output_path)

        temporary_metadata = self.metadata_path.with_name(
            f".{self.metadata_path.name}.tmp"
        )
        temporary_metadata.write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary_metadata.replace(self.metadata_path)
        self._stopped = True
        return self.output_path
