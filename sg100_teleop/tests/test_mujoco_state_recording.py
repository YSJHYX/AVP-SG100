"""Camera, visual-state recorder, replay, and silent MP4 regression tests."""

from __future__ import annotations

import os
os.environ.setdefault("MUJOCO_GL", "egl")

from pathlib import Path
import json
import tempfile

import av
import mujoco
import numpy as np

from sg100_teleop.recording.mujoco_state_recorder import MujocoStateRecorder
from sg100_teleop.recording.render_mujoco_video import render_recording
from sg100_teleop.sim.combined_robot_layout import (
    initialize_robot_state,
    resolve_robot_layout,
)


ROOT = Path(__file__).resolve().parents[2]
SCENE = (
    ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/"
    "rm65_sg100_avp_grasp_scene.xml"
)
CAMERA = "workstation_front"
READY_Q = np.array(
    [0.32488, -0.76769, 0.97621, 0.22463, -1.80649, -0.12430],
    dtype=np.float64,
)


def make_ready_state() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    layout = resolve_robot_layout(model)
    initialize_robot_state(model, data, layout, READY_Q, np.zeros(11))
    mujoco.mj_forward(model, data)
    return model, data


def body_pose(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    assert body_id >= 0
    return np.concatenate((data.xpos[body_id], data.xmat[body_id]))


def test_camera_and_offscreen_render() -> None:
    model, data = make_ready_state()
    camera_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA
    )
    assert camera_id >= 0
    assert model.nlight >= 1

    target = np.array([0.4878925, -0.073461236, 0.16177864])
    expected = target - data.cam_xpos[camera_id]
    expected /= np.linalg.norm(expected)
    camera_rotation = data.cam_xmat[camera_id].reshape(3, 3)
    optical_axis = -camera_rotation[:, 2]
    assert float(np.dot(optical_axis, expected)) > 1.0 - 1e-8
    # Robot-side eye camera: behind the near x=-0.15 table edge, centred on
    # the table's Y midpoint and looking forward over the arm.
    assert abs(float(data.cam_xpos[camera_id, 1]) + 0.073461236) < 1e-12
    assert abs(float(data.cam_xpos[camera_id, 0]) + 0.05) < 1e-12
    assert abs(float(data.cam_xpos[camera_id, 2]) - 0.90) < 1e-12
    assert abs(float(np.dot(camera_rotation[:, 0], [0.0, 1.0, 0.0]))) > 0.99

    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        renderer.update_scene(data, camera=CAMERA)
        frame = renderer.render().copy()
    assert frame.shape == (720, 1280, 3)
    assert frame.dtype == np.uint8
    assert np.all(np.isfinite(frame))
    assert float(frame.mean()) > 5.0
    assert float(frame.var()) > 5.0


def test_30_fps_sampling_and_state_fidelity() -> None:
    model, data = make_ready_state()
    watched = ("sg100_base_link", "water_bottle", "obj_coke")
    expected_poses: dict[str, np.ndarray] = {}

    with tempfile.TemporaryDirectory() as temporary:
        recorder = MujocoStateRecorder(
            model,
            scene_path=SCENE,
            recording_dir=temporary,
            record_fps=30.0,
        )
        recorder.start(data)
        expected_poses = {
            name: body_pose(model, data, name).copy() for name in watched
        }
        for _ in range(500):
            mujoco.mj_step(model, data)
            if recorder.maybe_record(data):
                # mj_step may leave derived xpos/xmat at the pre-integration
                # stage while qpos is already advanced. Replay intentionally
                # uses qpos + mj_forward, so compare against the same state.
                mujoco.mj_forward(model, data)
                expected_poses = {
                    name: body_pose(model, data, name).copy() for name in watched
                }
        output = recorder.stop()

        assert abs(recorder.frame_count - 30) <= 1
        with np.load(output, allow_pickle=False) as archive:
            assert archive["timestamps"].shape == (recorder.frame_count,)
            assert archive["qpos"].shape == (recorder.frame_count, model.nq)
            assert archive["mocap_pos"].shape == (
                recorder.frame_count,
                model.nmocap,
                3,
            )
            assert tuple(archive["camera_names"].tolist()) == (
                "workstation_front",
                "wrist_camera",
            )
            assert tuple(archive["camera_observation_keys"].tolist()) == (
                "base_0_rgb",
                "right_wrist_0_rgb",
            )
            replay = mujoco.MjData(model)
            replay.qpos[:] = archive["qpos"][-1]
            replay.mocap_pos[:] = archive["mocap_pos"][-1]
            replay.mocap_quat[:] = archive["mocap_quat"][-1]
            replay.time = float(archive["timestamps"][-1])
            mujoco.mj_forward(model, replay)

        assert recorder.metadata_path is not None
        metadata = json.loads(recorder.metadata_path.read_text(encoding="utf-8"))
        assert metadata["camera_observations"] == {
            "workstation_front": "base_0_rgb",
            "wrist_camera": "right_wrist_0_rgb",
        }

        for name in watched:
            np.testing.assert_allclose(
                body_pose(model, replay, name),
                expected_poses[name],
                atol=1e-11,
                rtol=0.0,
            )


def test_mp4_smoke() -> None:
    model, data = make_ready_state()
    layout = resolve_robot_layout(model)

    with tempfile.TemporaryDirectory() as temporary:
        recorder = MujocoStateRecorder(
            model,
            scene_path=SCENE,
            recording_dir=temporary,
            record_fps=30.0,
        )
        recorder.start(data)
        for index in range(1, 31):
            data.time = index / 30.0
            data.qpos[layout.arm_qpos_indices[0]] = (
                READY_Q[0] + 0.20 * np.sin(2.0 * np.pi * index / 30.0)
            )
            mujoco.mj_forward(model, data)
            assert recorder.maybe_record(data)
        recording = recorder.stop()

        try:
            render_recording(recording, camera="camera_that_does_not_exist")
        except ValueError as exc:
            assert "Camera 'camera_that_does_not_exist' not found" in str(exc)
        else:
            raise AssertionError("Missing camera must fail instead of using free camera")

        summary = render_recording(recording, width=1280, height=720)
        output = Path(summary["output"])
        assert output.is_file() and output.stat().st_size > 0
        assert summary["frames"] == 31
        assert abs(float(summary["duration"]) - 31.0 / 30.0) < 1e-12

        with av.open(str(output)) as container:
            video_streams = [s for s in container.streams if s.type == "video"]
            audio_streams = [s for s in container.streams if s.type == "audio"]
            assert len(video_streams) == 1
            assert not audio_streams
            stream = video_streams[0]
            assert stream.codec_context.name == "h264"
            assert stream.width == 1280 and stream.height == 720
            assert abs(float(stream.average_rate) - 30.0) < 1e-6
            decoded = [frame.to_ndarray(format="rgb24") for frame in container.decode(stream)]

        assert len(decoded) == 31
        assert float(decoded[0].mean()) > 5.0
        assert float(decoded[0].var()) > 5.0
        frame_differences = [
            np.mean(np.abs(decoded[0].astype(int) - frame.astype(int)))
            for frame in decoded[1:]
        ]
        assert max(frame_differences) > 0.05


def main() -> None:
    tests = (
        test_camera_and_offscreen_render,
        test_30_fps_sampling_and_state_fidelity,
        test_mp4_smoke,
    )
    for test in tests:
        test()
        print(f"{test.__name__}: PASS")
    print("MUJOCO STATE RECORDING + OFFLINE VIDEO: PASS")


if __name__ == "__main__":
    main()
