"""Production-v8 workstation, camera, prop, and rendering contracts."""

from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np

from sg100_teleop.recording.refine_scene_v8 import (
    EXPECTED_REFERENCE_FRAMES,
    RECORDING,
    _demo_coverage,
    _restore_robot_state,
)
from sg100_teleop.sim.camera_observations import (
    CAMERA_TO_OBSERVATION_KEY,
    render_camera_observations,
    validate_camera_observation_contract,
)
from sg100_teleop.sim.combined_robot_layout import resolve_robot_layout


ROOT = Path(__file__).resolve().parents[2]
SCENE = ROOT / "sg100_teleop/assets/rm65_sg100/mjcf/rm65_sg100_avp_grasp_scene.xml"
HEAD, WRIST, DEMO, PALM = "workstation_front", "wrist_camera", "demo_camera", "sg100_base_link"
TASK_OBJECTS = {"obj_coke", "obj_apple", "obj_mug"}
REMOVED_OBJECTS = {"obj_redbull", "obj_pringles", "obj_orange", "water_bottle"}
WRIST_LOCAL_POSITION = np.array([0.016859, -0.160165, 0.0])
WRIST_LOCAL_ROTATION = np.array([
    [0.9999936402968342, -0.002427369520439596, -0.002612899366771533],
    [-0.0035664220010585635, -0.6806132539473645, -0.7326342055796293],
    [0.0, 0.7326388649453396, -0.6806175824731597],
])
HEAD_POSITION = np.array([-0.05, -0.073461236, 0.9])
HEAD_ROTATION = np.array([
    [0.0, 0.808212783, -0.588890565],
    [-1.0, 0.0, 0.0],
    [0.0, 0.588890565, 0.808212783],
])


def make_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(SCENE))


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    result = mujoco.mj_name2id(model, kind, name)
    assert result >= 0, f"{name!r} missing"
    return result


def render(model: mujoco.MjModel, data: mujoco.MjData, camera: str) -> np.ndarray:
    with mujoco.Renderer(model, height=360, width=640) as renderer:
        renderer.update_scene(data, camera=camera)
        return np.ascontiguousarray(renderer.render()).copy()


def assert_live(frame: np.ndarray) -> None:
    assert frame.shape == (360, 640, 3)
    assert frame.dtype == np.uint8
    assert np.all(np.isfinite(frame))
    assert float(frame.mean()) > 5.0 and float(frame.var()) > 5.0


def test_camera_names_and_frozen_optical_geometry() -> None:
    model = make_model()
    names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, index)
        for index in range(model.ncam)
    }
    assert names == {HEAD, WRIST, DEMO}
    head = object_id(model, mujoco.mjtObj.mjOBJ_CAMERA, HEAD)
    assert int(model.cam_bodyid[head]) == 0
    np.testing.assert_allclose(model.cam_pos[head], HEAD_POSITION, atol=1e-12, rtol=0)
    np.testing.assert_allclose(model.cam_mat0[head].reshape(3, 3), HEAD_ROTATION, atol=8e-10, rtol=0)
    assert float(model.cam_fovy[head]) == 50.0
    wrist = object_id(model, mujoco.mjtObj.mjOBJ_CAMERA, WRIST)
    palm = object_id(model, mujoco.mjtObj.mjOBJ_BODY, PALM)
    assert int(model.cam_bodyid[wrist]) == palm
    np.testing.assert_allclose(model.cam_pos[wrist], WRIST_LOCAL_POSITION, atol=1e-12, rtol=0)
    wrist_local_mat = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(wrist_local_mat, model.cam_quat[wrist])
    np.testing.assert_allclose(wrist_local_mat.reshape(3, 3), WRIST_LOCAL_ROTATION, atol=8e-10, rtol=0)
    assert float(model.cam_fovy[wrist]) == 75.0


def test_wrist_camera_is_optical_only_and_follows_palm() -> None:
    model = make_model()
    for name in (
        "wrist_camera_mount_base_visual", "wrist_camera_extension_visual",
        "wrist_camera_visual", "wrist_camera_collision",
    ):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) == -1
    for name in ("wrist_camera_mount_base", "wrist_camera_head"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) == -1
    for geom in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom) or ""
        assert "wrist_camera" not in name

    data = mujoco.MjData(model)
    wrist = object_id(model, mujoco.mjtObj.mjOBJ_CAMERA, WRIST)
    palm = object_id(model, mujoco.mjtObj.mjOBJ_BODY, PALM)
    for arm_q in (np.zeros(6), np.array([0.32488, -0.76769, 0.97621, 0.22463, -1.80649, -0.12430])):
        data.qpos[:] = model.qpos0
        data.qpos[:6] = arm_q
        mujoco.mj_forward(model, data)
        palm_rotation = data.xmat[palm].reshape(3, 3)
        relative_position = palm_rotation.T @ (data.cam_xpos[wrist] - data.xpos[palm])
        relative_rotation = palm_rotation.T @ data.cam_xmat[wrist].reshape(3, 3)
        np.testing.assert_allclose(relative_position, WRIST_LOCAL_POSITION, atol=2e-12, rtol=0)
        np.testing.assert_allclose(relative_rotation, WRIST_LOCAL_ROTATION, atol=8e-10, rtol=0)


def test_demo_camera_world_pose_zero_roll_and_full_trajectory_coverage() -> None:
    model = make_model()
    data = mujoco.MjData(model)
    camera = object_id(model, mujoco.mjtObj.mjOBJ_CAMERA, DEMO)
    assert int(model.cam_bodyid[camera]) == 0
    assert model.cam_pos[camera, 0] > 0.75
    assert 1.0 <= model.cam_pos[camera, 2] <= 1.4 + 1e-12
    assert 50.0 <= model.cam_fovy[camera] <= 60.0
    mujoco.mj_forward(model, data)
    rotation = data.cam_xmat[camera].reshape(3, 3)
    assert abs(float(rotation[2, 0])) < 1e-12
    expected_axis = np.array([0.35, 0.10, 0.35]) - data.cam_xpos[camera]
    expected_axis /= np.linalg.norm(expected_axis)
    np.testing.assert_allclose(-rotation[:, 2], expected_axis, atol=8e-10, rtol=0)
    if not RECORDING.is_file():
        return
    with np.load(RECORDING, allow_pickle=False) as archive:
        timestamps = np.asarray(archive["timestamps"], dtype=np.float64)
        qpos = np.asarray(archive["qpos"], dtype=np.float64)
    coverage = _demo_coverage(model, data, qpos, timestamps)
    assert coverage["scanned_frames"] == EXPECTED_REFERENCE_FRAMES
    assert coverage["coverage_fraction"] == 1.0
    assert coverage["minimum_conservative_margin"] >= 0.05


def test_only_three_task_objects_are_free_bodies() -> None:
    model = make_model()
    assert model.nsite == 0, "production RGB scene must not contain debug markers"
    free_bodies, free_joints = set(), set()
    for joint in range(model.njnt):
        if model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE:
            continue
        free_joints.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint))
        free_bodies.add(mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY, int(model.jnt_bodyid[joint])
        ))
    assert free_bodies == TASK_OBJECTS
    assert free_joints == {f"{name}_free" for name in TASK_OBJECTS}
    for name in REMOVED_OBJECTS:
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) == -1


def test_cup_is_upright_and_handle_faces_ready_palm() -> None:
    model = make_model()
    data = mujoco.MjData(model)
    mug = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "obj_mug")
    joint = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, "obj_mug_free")
    qadr = int(model.jnt_qposadr[joint])
    np.testing.assert_allclose(model.qpos0[qadr : qadr + 3], [0.55, 0.15, 0.04281], atol=1e-12, rtol=0)
    mujoco.mj_forward(model, data)
    rotation = data.xmat[mug].reshape(3, 3)
    assert float(rotation[2, 2]) > 1.0 - 1e-12
    mesh = object_id(model, mujoco.mjtObj.mjOBJ_MESH, "mug_vis")
    start = int(model.mesh_vertadr[mesh])
    vertices = model.mesh_vert[start : start + int(model.mesh_vertnum[mesh])]
    radial = np.linalg.norm(vertices[:, :2], axis=1)
    handle_local_xy = vertices[radial >= np.percentile(radial, 95), :2].mean(axis=0)
    assert handle_local_xy[1] > 0.03 and abs(handle_local_xy[0]) < 0.005
    handle_world = rotation[:2, 1]
    handle_world /= np.linalg.norm(handle_world)
    ready = np.array([0.25048434, 0.11727796]) - data.xpos[mug, :2]
    ready /= np.linalg.norm(ready)
    angle = np.rad2deg(np.arccos(np.clip(np.dot(handle_world, ready), -1.0, 1.0)))
    assert angle <= 5.0


def test_open_wooden_crate_has_bottom_and_four_low_collision_walls() -> None:
    model = make_model()
    crate = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "drop_bin")
    geoms = np.flatnonzero(model.geom_bodyid == crate)
    assert len(geoms) == 5
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom)) for geom in geoms}
    assert names == {
        "wood_crate_bottom", "wood_crate_wall_pX", "wood_crate_wall_nX",
        "wood_crate_wall_pY", "wood_crate_wall_nY",
    }
    cavity_probe = np.array([0.0, 0.0, 0.04])
    for geom in geoms:
        assert int(model.geom_type[geom]) == int(mujoco.mjtGeom.mjGEOM_BOX)
        assert (int(model.geom_contype[geom]), int(model.geom_conaffinity[geom])) == (4, 3)
        red, green, blue, alpha = model.geom_rgba[geom]
        assert red > green > blue and alpha == 1.0
        assert not np.all(np.abs(cavity_probe - model.geom_pos[geom]) <= model.geom_size[geom])
        assert model.geom_pos[geom, 2] + model.geom_size[geom, 2] <= 0.08 + 1e-12


def test_single_overhead_light_and_headlight_contract() -> None:
    model = make_model()
    assert model.nlight == 1
    assert mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_LIGHT, 0) == "workstation_overhead"
    np.testing.assert_allclose(model.light_dir[0], [0, 0, -1], atol=1e-12, rtol=0)
    assert not bool(model.light_castshadow[0])
    assert not bool(model.vis.headlight.active)


def test_rendering_and_vla_observation_mapping() -> None:
    model = make_model()
    data = mujoco.MjData(model)
    if RECORDING.is_file():
        with np.load(RECORDING, allow_pickle=False) as archive:
            _restore_robot_state(model, data, archive["qpos"][0], archive["timestamps"][0])
    else:
        mujoco.mj_forward(model, data)
    for camera in (HEAD, WRIST, DEMO):
        assert_live(render(model, data, camera))
    ids = validate_camera_observation_contract(model)
    assert set(ids) == set(CAMERA_TO_OBSERVATION_KEY)
    assert CAMERA_TO_OBSERVATION_KEY == {
        "workstation_front": "base_0_rgb", "wrist_camera": "right_wrist_0_rgb"
    }
    with mujoco.Renderer(model, height=360, width=640) as renderer:
        observations = render_camera_observations(renderer, data)
    assert tuple(observations) == ("base_0_rgb", "right_wrist_0_rgb")
    for frame in observations.values():
        assert_live(frame)


def test_robot_and_dynamic_scene_dimensions() -> None:
    model = make_model()
    layout = resolve_robot_layout(model)
    assert (model.nq, model.nv, model.nu) == (38, 35, 17)
    assert layout.qpos_indices.tolist() == list(range(17))
    assert layout.dof_indices.tolist() == list(range(17))
    assert len(layout.actuator_ids) == 17
    names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
        for index in range(model.nu)
    }
    assert len(names) == 17


def main() -> None:
    tests = (
        test_camera_names_and_frozen_optical_geometry,
        test_wrist_camera_is_optical_only_and_follows_palm,
        test_demo_camera_world_pose_zero_roll_and_full_trajectory_coverage,
        test_only_three_task_objects_are_free_bodies,
        test_cup_is_upright_and_handle_faces_ready_palm,
        test_open_wooden_crate_has_bottom_and_four_low_collision_walls,
        test_single_overhead_light_and_headlight_contract,
        test_rendering_and_vla_observation_mapping,
        test_robot_and_dynamic_scene_dimensions,
    )
    for test in tests:
        test()
        print(f"{test.__name__}: PASS")
    print("PRODUCTION V8 SCENE: PASS")


if __name__ == "__main__":
    main()
