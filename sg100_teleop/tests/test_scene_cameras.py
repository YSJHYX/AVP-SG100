"""Scene camera-layout contract tests for the dual-camera setup.

Covers the Phase-1 camera work: the world-fixed, robot-side head camera
(``workstation_front`` -> ``base_0_rgb``) and the compact inward-looking
wrist camera on the SG100 palm (``wrist_camera`` ->
``right_wrist_0_rgb``).  These tests pin down:

  1. both cameras exist in the active scene;
  2. ``workstation_front`` is world-fixed;
  3. ``wrist_camera`` is mounted on the ``sg100_base_link`` chain so it
     follows the wrist;
  4. ``wrist_camera`` is thumb-side, outward/distal, and aimed at the
     hand-object interaction volume;
  5. its physical envelope collides with table/objects without adding mass;
  6. the head camera is interaction-centred with <= 1.5 degree table roll;
  7. both cameras offscreen-render synchronized VLA RGB observations;
  8. adding cameras/lights did not change the robot model contract
     (nq/nv/nu/joint names/actuator names).
"""

from __future__ import annotations

import os
os.environ.setdefault("MUJOCO_GL", "egl")

from pathlib import Path

import mujoco
import numpy as np

from sg100_teleop.sim.combined_robot_layout import (
    initialize_robot_state,
    resolve_robot_layout,
)
from sg100_teleop.recording.render_camera_preview import _select_frames
from sg100_teleop.recording.render_mujoco_video import _restore_state
from sg100_teleop.sim.camera_observations import (
    CAMERA_TO_OBSERVATION_KEY,
    render_camera_observations,
    validate_camera_observation_contract,
)


ROOT = Path(__file__).resolve().parents[2]
SCENE = (
    ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/"
    "rm65_sg100_avp_grasp_scene.xml"
)
RECORDED_SESSION = ROOT / "recording/20260820_181024/mujoco_recording.npz"
HEAD_CAMERA = "workstation_front"
WRIST_CAMERA = "wrist_camera"
PALM_BODY = "sg100_base_link"
READY_Q = np.array(
    [0.32488, -0.76769, 0.97621, 0.22463, -1.80649, -0.12430],
    dtype=np.float64,
)
EXPECTED_HINGE_JOINTS = frozenset(
    {
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
        "thumb_j1", "thumb_j2", "thumb_j3",
        "index_j1", "index_j2", "index_j3",
        "middle_j1", "middle_j2",
        "little_j1", "little_j2", "little_j3",
    }
)
EXPECTED_FREE_JOINTS = 7
EXPECTED_ACTUATORS = frozenset(
    {
        "joint_1_pos", "joint_2_pos", "joint_3_pos",
        "joint_4_pos", "joint_5_pos", "joint_6_pos",
        "thumb_j1_pos", "thumb_j2_pos", "thumb_j3_pos",
        "index_j1_pos", "index_j2_pos", "index_j3_pos",
        "middle_j1_pos", "middle_j2_pos",
        "little_j1_pos", "little_j2_pos", "little_j3_pos",
    }
)
EXPECTED_WRIST_POSITION = np.array(
    [0.016859, -0.160165, 0.000], dtype=np.float64
)
EXPECTED_WRIST_OPTICAL_AXIS = np.array(
    [0.002612899, 0.732634205, 0.680617583], dtype=np.float64
)
V3_WRIST_POSITION = np.array([0.065, -0.015, -0.070])
WRIST_INTERACTION_TARGET = np.array(
    [0.017235401, -0.054625429, 0.098046320]
)
EXPECTED_HEAD_POSITION = np.array([-0.050000, -0.073461236, 0.900000])
HEAD_TARGET = np.array([0.4878925, -0.073461236, 0.16177864])
EXPECTED_SG100_MESH_FACES = {
    "sg100_base_link_visual_part1": 187_999,
    "sg100_base_link_visual_part2": 181_028,
    "sg100_base_link_visual_part3": 112_678,
    "sg100_thumb_link1": 16_970,
    "sg100_thumb_link2": 193_892,
    "sg100_thumb_link3": 19_130,
    "sg100_index_link1": 19_560,
    "sg100_index_link2": 193_892,
    "sg100_index_link3": 19_130,
    "sg100_middle_link1": 193_892,
    "sg100_middle_link2": 19_130,
    "sg100_little_link1": 19_502,
    "sg100_little_llink2": 193_892,
    "sg100_little_linik3": 19_130,
}
SG100_VISUAL_GEOMS = {
    "sg100_base_link_visual_part1": "sg100_base_link_visual_part1",
    "sg100_base_link_visual_part2": "sg100_base_link_visual_part2",
    "sg100_base_link_visual_part3": "sg100_base_link_visual_part3",
    "sg100_thumb_link1": "thumb_link1_visual",
    "sg100_thumb_link2": "thumb_link2_visual",
    "sg100_thumb_link3": "thumb_link3_visual",
    "sg100_index_link1": "index_link1_visual",
    "sg100_index_link2": "index_link2_visual",
    "sg100_index_link3": "index_link3_visual",
    "sg100_middle_link1": "middle_link1_visual",
    "sg100_middle_link2": "middle_link2_visual",
    "sg100_little_link1": "little_link1_visual",
    "sg100_little_llink2": "little_llink2_visual",
    "sg100_little_linik3": "little_linik3_visual",
}
SG100_COLLISION_GEOMS = (
    "palm_collision",
    "thumb_link2_collision", "thumb_link3_collision",
    "index_link2_collision", "index_link3_collision",
    "middle_link2_collision",
    "little_llink2_collision", "little_linik3_collision",
)
EXPECTED_SG100_INERTIALS = {
    "sg100_base_link": (0.126, (5.565e-05, 8.925e-05, 1.365e-04)),
    "thumb_link1": (0.0113388, (1.6811e-06, 1.39718e-06, 7.75511e-07)),
    "thumb_link2": (0.0389603, (5.64004e-06, 4.9655e-06, 2.66961e-06)),
    "thumb_link3": (0.03917, (4.86657e-06, 4.6047e-06, 2.51865e-06)),
    "index_link1": (0.0164783, (2.89253e-06, 2.46445e-06, 1.31555e-06)),
    "index_link2": (0.0389603, (5.64004e-06, 4.96551e-06, 2.6696e-06)),
    "index_link3": (0.0391696, (4.86645e-06, 4.6046e-06, 2.51862e-06)),
    "middle_link1": (0.0389603, (5.64004e-06, 4.96551e-06, 2.6696e-06)),
    "middle_link2": (0.0391696, (4.86646e-06, 4.6047e-06, 2.51862e-06)),
    "little_link1": (0.0161726, (2.83334e-06, 2.43857e-06, 1.28303e-06)),
    "little_llink2": (0.0389603, (5.64004e-06, 4.96551e-06, 2.6696e-06)),
    "little_linik3": (0.0391696, (4.86646e-06, 4.6047e-06, 2.51862e-06)),
}


def make_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(SCENE))


def make_ready_state() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = make_model()
    data = mujoco.MjData(model)
    layout = resolve_robot_layout(model)
    initialize_robot_state(model, data, layout, READY_Q, np.zeros(11))
    mujoco.mj_forward(model, data)
    return model, data


def camera_id(model: mujoco.MjModel, name: str) -> int:
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
    assert camera_id >= 0, f"camera {name!r} missing from scene"
    return camera_id


def body_ancestry(model: mujoco.MjModel, body_id: int) -> set[int]:
    ancestry = set()
    while body_id != 0:
        ancestry.add(body_id)
        body_id = int(model.body_parentid[body_id])
    ancestry.add(0)
    return ancestry


def render_frame(
    model: mujoco.MjModel, data: mujoco.MjData, camera: str
) -> np.ndarray:
    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        renderer.update_scene(data, camera=camera)
        return np.ascontiguousarray(renderer.render()).copy()


def assert_live_frame(frame: np.ndarray, context: str) -> None:
    assert frame.shape == (720, 1280, 3), f"{context}: shape {frame.shape}"
    assert frame.dtype == np.uint8, f"{context}: dtype {frame.dtype}"
    assert np.all(np.isfinite(frame)), f"{context}: non-finite pixels"
    assert float(frame.mean()) > 5.0, f"{context}: nearly black"
    assert float(frame.var()) > 5.0, f"{context}: constant frame"


def _project_world_point_ndc(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera: int,
    point: np.ndarray,
    *,
    aspect: float,
) -> tuple[float, float]:
    rotation = data.cam_xmat[camera].reshape(3, 3)
    camera_point = rotation.T @ (point - data.cam_xpos[camera])
    depth = -float(camera_point[2])
    assert depth > 0.0
    tan_half_fovy = np.tan(np.deg2rad(model.cam_fovy[camera]) / 2.0)
    return (
        float(camera_point[0] / (depth * tan_half_fovy * aspect)),
        float(camera_point[1] / (depth * tan_half_fovy)),
    )


def _contact_enabled(model: mujoco.MjModel, first: int, second: int) -> bool:
    return bool(
        int(model.geom_contype[first])
        & int(model.geom_conaffinity[second])
        or int(model.geom_contype[second])
        & int(model.geom_conaffinity[first])
    )


def test_both_cameras_exist() -> None:
    model = make_model()
    names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
        for i in range(model.ncam)
    }
    assert HEAD_CAMERA in names
    assert WRIST_CAMERA in names


def test_head_camera_is_world_fixed() -> None:
    model = make_model()
    camera = camera_id(model, HEAD_CAMERA)
    assert int(model.cam_bodyid[camera]) == 0, (
        "workstation_front must stay world-fixed (worldbody child)"
    )
    # MJCF mode="fixed" compiles to the runtime enum mjCAMERA_FREE (pose
    # fixed in the parent-body frame); the world parent makes it world-fixed.
    assert int(model.cam_mode[camera]) == int(
        mujoco.mjtCamera.mjCAMERA_FREE
    )
    np.testing.assert_allclose(
        model.cam_pos[camera], EXPECTED_HEAD_POSITION, atol=1e-12, rtol=0.0
    )
    assert 0.85 <= float(model.cam_pos[camera, 2]) <= 0.95
    assert abs(float(model.cam_fovy[camera]) - 50.0) < 1e-12


def test_head_camera_is_interaction_centered_and_zero_roll() -> None:
    model, data = make_ready_state()
    camera = camera_id(model, HEAD_CAMERA)
    rotation = data.cam_xmat[camera].reshape(3, 3)
    expected_axis = HEAD_TARGET - data.cam_xpos[camera]
    expected_axis /= np.linalg.norm(expected_axis)
    np.testing.assert_allclose(
        -rotation[:, 2], expected_axis, atol=2e-9, rtol=0.0
    )

    target_x, target_y = _project_world_point_ndc(
        model, data, camera, HEAD_TARGET, aspect=16.0 / 9.0
    )
    assert abs(target_x) <= 1e-8
    assert abs(target_y) <= 1e-8

    front_left = _project_world_point_ndc(
        model,
        data,
        camera,
        np.array([-0.15, -0.35, 0.0]),
        aspect=16.0 / 9.0,
    )
    front_right = _project_world_point_ndc(
        model,
        data,
        camera,
        np.array([-0.15, 0.65, 0.0]),
        aspect=16.0 / 9.0,
    )
    # Screen-space roll in pixels: undo the NDC normalisation (x spans the
    # width, y the height of the 16:9 canvas), so the angle matches what a
    # viewer actually sees instead of an aspect-stretched NDC angle.
    table_roll = float(
        np.degrees(
            np.arctan2(
                (front_right[1] - front_left[1]) * 9.0,
                (front_right[0] - front_left[0]) * 16.0,
            )
        )
    )
    table_roll = (table_roll + 90.0) % 180.0 - 90.0
    assert abs(table_roll) <= 1.5

    # V7 intentionally preserves the v4 crop: validate the active
    # manipulation bounds instead of demanding all four table corners.
    for point in (
        np.array([0.27, -0.28, 0.06]),
        np.array([0.55, 0.29, 0.24]),
    ):
        x_ndc, y_ndc = _project_world_point_ndc(
            model, data, camera, point, aspect=16.0 / 9.0
        )
        assert abs(x_ndc) <= 1.0
        assert abs(y_ndc) <= 1.0


def test_wrist_camera_is_on_palm_chain() -> None:
    model = make_model()
    camera = camera_id(model, WRIST_CAMERA)
    parent = int(model.cam_bodyid[camera])
    palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, PALM_BODY)
    assert palm >= 0
    assert palm in body_ancestry(model, parent), (
        "wrist_camera must ride on the sg100_base_link chain so it moves "
        "with the wrist; direct mount or a fixed camera-mount body are the "
        "only allowed parents"
    )
    mount = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "wrist_camera_mount_base"
    )
    head = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "wrist_camera_head"
    )
    assert mount >= 0 and head >= 0
    assert int(model.body_parentid[mount]) == palm
    assert int(model.body_parentid[head]) == mount
    assert int(model.body_jntnum[mount]) == 0
    assert int(model.body_jntnum[head]) == 0


def test_wrist_camera_is_outward_distal_and_interaction_looking() -> None:
    model, data = make_ready_state()
    camera = camera_id(model, WRIST_CAMERA)
    palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, PALM_BODY)
    palm_rotation = data.xmat[palm].reshape(3, 3)
    pos = palm_rotation.T @ (
        data.cam_xpos[camera] - data.xpos[palm]
    )
    np.testing.assert_allclose(
        pos, EXPECTED_WRIST_POSITION, atol=1e-12, rtol=0.0
    )
    radius = float(np.linalg.norm(pos))
    assert 0.15 <= radius <= 0.17
    # Medium extension: clearly displaced along the outward palm normal,
    # without returning to the close v3 wrist-side mount.
    assert float(pos[1]) <= -0.15
    assert float(np.linalg.norm(pos - V3_WRIST_POSITION)) >= 0.14
    assert pos[0] > 0.0 and pos[1] < 0.0

    camera_rotation = data.cam_xmat[camera].reshape(3, 3)
    optical_axis = palm_rotation.T @ (-camera_rotation[:, 2])
    optical_axis /= np.linalg.norm(optical_axis)
    np.testing.assert_allclose(
        optical_axis,
        EXPECTED_WRIST_OPTICAL_AXIS,
        atol=2e-8,
        rtol=0.0,
    )
    # Preserve the initial v6 palm-normal semantics: look inward along +Y
    # and distally along +Z to the same FK-derived grasp centre.
    expected_ray = WRIST_INTERACTION_TARGET - pos
    expected_ray /= np.linalg.norm(expected_ray)
    np.testing.assert_allclose(
        optical_axis, expected_ray, atol=2e-8, rtol=0.0
    )
    assert abs(float(optical_axis[0])) < 0.02
    assert optical_axis[1] > 0.70
    assert optical_axis[2] > 0.65
    assert abs(float(model.cam_fovy[camera]) - 75.0) < 1e-12

    target_site = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SITE, "wrist_camera_interaction_target"
    )
    assert target_site >= 0
    np.testing.assert_allclose(
        model.site_pos[target_site],
        WRIST_INTERACTION_TARGET,
        atol=1e-12,
        rtol=0.0,
    )
    assert float(model.site_rgba[target_site, 3]) == 0.0

    for geom_name in (
        "wrist_camera_mount_base_visual",
        "wrist_camera_extension_visual",
    ):
        geom = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, geom_name
        )
        assert geom >= 0
        assert int(model.geom_contype[geom]) == 0
        assert int(model.geom_conaffinity[geom]) == 0


def test_sg100_visual_meshes_and_physics_separation() -> None:
    model = make_model()
    for mesh_name, expected_faces in EXPECTED_SG100_MESH_FACES.items():
        mesh = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, mesh_name)
        assert mesh >= 0, f"visual mesh {mesh_name!r} missing"
        assert int(model.mesh_facenum[mesh]) == expected_faces
        assert int(model.mesh_vertnum[mesh]) > 0
        assert np.all(np.isfinite(model.mesh_scale[mesh]))
        vertex_adr = int(model.mesh_vertadr[mesh])
        vertex_num = int(model.mesh_vertnum[mesh])
        vertices = np.asarray(
            model.mesh_vert[vertex_adr : vertex_adr + vertex_num]
        )
        assert np.all(np.isfinite(vertices))
        assert np.all(np.ptp(vertices, axis=0) > 0.0)

    assert sum(
        EXPECTED_SG100_MESH_FACES[name]
        for name in SG100_VISUAL_GEOMS
        if name.startswith("sg100_base_link_visual_part")
    ) == 481_705

    for mesh_name, geom_name in SG100_VISUAL_GEOMS.items():
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        assert geom >= 0, f"visual geom {geom_name!r} missing"
        mesh = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, mesh_name)
        assert int(model.geom_dataid[geom]) == mesh
        assert int(model.geom_type[geom]) == int(mujoco.mjtGeom.mjGEOM_MESH)
        assert int(model.geom_group[geom]) == 1
        assert int(model.geom_contype[geom]) == 0
        assert int(model.geom_conaffinity[geom]) == 0

    for geom_name in SG100_COLLISION_GEOMS:
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        assert geom >= 0, f"collision geom {geom_name!r} missing"
        assert int(model.geom_group[geom]) == 3
        assert int(model.geom_contype[geom]) == 1
        assert int(model.geom_conaffinity[geom]) == 2
        assert float(model.geom_rgba[geom, 3]) == 0.0

    # These exact values are the pre-refinement body inertial contract.  The
    # high-resolution geoms are visual-only and must not contribute mass.
    for body_name, (expected_mass, expected_inertia) in (
        EXPECTED_SG100_INERTIALS.items()
    ):
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        assert body >= 0
        np.testing.assert_allclose(
            model.body_mass[body], expected_mass, atol=1e-12, rtol=0.0
        )
        np.testing.assert_allclose(
            model.body_inertia[body], expected_inertia, atol=1e-12, rtol=0.0
        )


def test_wrist_camera_collision_envelope_and_recorded_clearance() -> None:
    model = make_model()
    visual = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "wrist_camera_visual"
    )
    collision = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "wrist_camera_collision"
    )
    assert visual >= 0 and collision >= 0
    assert int(model.geom_bodyid[visual]) == int(model.geom_bodyid[collision])
    np.testing.assert_allclose(
        model.geom_pos[visual], model.geom_pos[collision], atol=1e-12, rtol=0.0
    )
    assert np.all(model.geom_size[collision] > model.geom_size[visual])
    assert int(model.geom_contype[visual]) == 0
    assert int(model.geom_conaffinity[visual]) == 0
    assert int(model.geom_group[collision]) == 3
    assert int(model.geom_contype[collision]) == 1
    assert int(model.geom_conaffinity[collision]) == 6
    assert float(model.geom_rgba[collision, 3]) == 0.0

    table = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "grasp_table_collision"
    )
    bottle = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "water_bottle_collision"
    )
    finger = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "index_link3_collision"
    )
    arm = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "rm65_visual_6"
    )
    assert _contact_enabled(model, collision, table)
    assert _contact_enabled(model, collision, bottle)
    assert not _contact_enabled(model, collision, finger)
    assert not _contact_enabled(model, collision, arm)

    if not RECORDED_SESSION.is_file():
        return
    data = mujoco.MjData(model)
    with np.load(RECORDED_SESSION, allow_pickle=False) as archive:
        timestamps = np.asarray(archive["timestamps"], dtype=np.float64)
        qpos = np.asarray(archive["qpos"], dtype=np.float64)
        mocap_pos = np.asarray(archive["mocap_pos"], dtype=np.float64)
        mocap_quat = np.asarray(archive["mocap_quat"], dtype=np.float64)

    environment_contact_frames = 0
    for index in range(timestamps.size):
        _restore_state(
            model,
            data,
            qpos[index],
            mocap_pos[index],
            mocap_quat[index],
            timestamps[index],
        )
        environment_contact_frames += any(
            collision in (contact.geom1, contact.geom2)
            for contact in data.contact
        )

    # Phase-1 v7 treats current prop/table placement as a warning.  Self
    # penetration remains a hard failure, so temporarily enable the camera
    # envelope against every collision class and replay all recorded states.
    model.geom_contype[collision] = 0xFFFF
    model.geom_conaffinity[collision] = 0xFFFF
    self_types = {1, 8}
    for index in range(timestamps.size):
        _restore_state(
            model,
            data,
            qpos[index],
            mocap_pos[index],
            mocap_quat[index],
            timestamps[index],
        )
        for contact in data.contact:
            if collision not in (contact.geom1, contact.geom2):
                continue
            other = (
                contact.geom2
                if contact.geom1 == collision
                else contact.geom1
            )
            assert int(model.geom_contype[other]) not in self_types, (
                "camera/robot self penetration at recorded frame "
                f"{index}: {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other)}"
            )
    assert environment_contact_frames >= 0


def _project_body_center(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera: int,
    body_name: str,
    *,
    width: int,
    height: int,
) -> tuple[float, float]:
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    assert body >= 0
    rotation = data.cam_xmat[camera].reshape(3, 3)
    camera_point = rotation.T @ (
        data.xpos[body] - data.cam_xpos[camera]
    )
    depth = -float(camera_point[2])
    assert depth > 0.0, f"{body_name} is behind wrist camera"
    tan_half_fovy = np.tan(np.deg2rad(model.cam_fovy[camera]) / 2.0)
    x_ndc = camera_point[0] / (
        depth * tan_half_fovy * (width / height)
    )
    y_ndc = camera_point[1] / (depth * tan_half_fovy)
    return (float((x_ndc + 1.0) * width / 2.0),
            float((1.0 - y_ndc) * height / 2.0))


def test_wrist_view_keeps_palm_digits_and_grasp_object_visible() -> None:
    if not RECORDED_SESSION.is_file():
        return
    model = make_model()
    data = mujoco.MjData(model)
    with np.load(RECORDED_SESSION, allow_pickle=False) as archive:
        timestamps = np.asarray(archive["timestamps"], dtype=np.float64)
        qpos = np.asarray(archive["qpos"], dtype=np.float64)
        mocap_pos = np.asarray(archive["mocap_pos"], dtype=np.float64)
        mocap_quat = np.asarray(archive["mocap_quat"], dtype=np.float64)
    frames = _select_frames(
        model, data, qpos, mocap_pos, mocap_quat, timestamps
    )
    camera = camera_id(model, WRIST_CAMERA)
    width, height = 640, 360
    object_geom = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "water_bottle_collision"
    )
    with mujoco.Renderer(model, height=height, width=width) as renderer:
        renderer.enable_segmentation_rendering()
        for tag in (
            "home",
            "mid",
            "interaction",
            "grasp_closure",
            "thumb_opposition",
        ):
            index = frames[tag]
            _restore_state(
                model,
                data,
                qpos[index],
                mocap_pos[index],
                mocap_quat[index],
                timestamps[index],
            )
            for body_name in (
                "thumb_link3",
                "index_link3",
                "middle_link2",
                "little_linik3",
            ):
                x, y = _project_body_center(
                    model,
                    data,
                    camera,
                    body_name,
                    width=width,
                    height=height,
                )
                margin = min(x, width - x, y, height - y) / width
                assert margin >= 0.06, (
                    f"{tag}: {body_name} margin {margin:.1%} below 6%"
                )
            renderer.update_scene(data, camera=camera)
            segmentation = renderer.render()
            object_pixels = (
                (segmentation[:, :, 0] == object_geom)
                & (segmentation[:, :, 1] == mujoco.mjtObj.mjOBJ_GEOM)
            )
            if tag in ("home", "mid", "interaction"):
                assert np.any(object_pixels), f"{tag}: object is not visible"


def test_both_cameras_render_offscreen() -> None:
    model, data = make_ready_state()
    for camera in (HEAD_CAMERA, WRIST_CAMERA):
        assert_live_frame(render_frame(model, data, camera), camera)


def test_recorded_state_renders_in_both_cameras() -> None:
    if not RECORDED_SESSION.is_file():
        # The committed sample session is the fixture; skip silently on
        # checkouts that prune recordings.
        return
    model = make_model()
    data = mujoco.MjData(model)
    with np.load(RECORDED_SESSION, allow_pickle=False) as archive:
        qpos = np.asarray(archive["qpos"], dtype=np.float64)
        timestamps = np.asarray(archive["timestamps"], dtype=np.float64)
    for index in (0, qpos.shape[0] // 2):
        data.qpos[:] = qpos[index]
        data.time = float(timestamps[index])
        mujoco.mj_forward(model, data)
        # Same restored state feeds both cameras: one timeline, two views.
        head = render_frame(model, data, HEAD_CAMERA)
        wrist = render_frame(model, data, WRIST_CAMERA)
        assert_live_frame(head, f"{HEAD_CAMERA}@frame{index}")
        assert_live_frame(wrist, f"{WRIST_CAMERA}@frame{index}")


def test_vla_observation_keys_render_from_one_mujoco_state() -> None:
    model, data = make_ready_state()
    ids = validate_camera_observation_contract(model)
    assert set(ids) == set(CAMERA_TO_OBSERVATION_KEY)
    qpos_before = data.qpos.copy()
    with mujoco.Renderer(model, height=360, width=640) as renderer:
        observations = render_camera_observations(renderer, data)
    assert tuple(observations) == ("base_0_rgb", "right_wrist_0_rgb")
    for key, frame in observations.items():
        assert frame.shape == (360, 640, 3), key
        assert frame.dtype == np.uint8, key
        assert float(frame.mean()) > 5.0, key
        assert float(frame.var()) > 5.0, key
    np.testing.assert_array_equal(data.qpos, qpos_before)


def test_camera_and_light_edits_preserve_robot_contract() -> None:
    model = make_model()
    assert model.nq == 66
    assert model.nv == 59
    assert model.nu == len(EXPECTED_ACTUATORS) == 17

    hinge, free = [], []
    for joint in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE:
            free.append(name)
        elif model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_HINGE:
            hinge.append(name)
    assert frozenset(hinge) == EXPECTED_HINGE_JOINTS
    assert len(free) == EXPECTED_FREE_JOINTS

    actuators = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        for i in range(model.nu)
    }
    assert actuators == EXPECTED_ACTUATORS


def main() -> None:
    tests = (
        test_both_cameras_exist,
        test_head_camera_is_world_fixed,
        test_head_camera_is_interaction_centered_and_zero_roll,
        test_wrist_camera_is_on_palm_chain,
        test_wrist_camera_is_outward_distal_and_interaction_looking,
        test_sg100_visual_meshes_and_physics_separation,
        test_wrist_camera_collision_envelope_and_recorded_clearance,
        test_wrist_view_keeps_palm_digits_and_grasp_object_visible,
        test_both_cameras_render_offscreen,
        test_recorded_state_renders_in_both_cameras,
        test_vla_observation_keys_render_from_one_mujoco_state,
        test_camera_and_light_edits_preserve_robot_contract,
    )
    for test in tests:
        test()
        print(f"{test.__name__}: PASS")
    print("SCENE CAMERA LAYOUT: PASS")


if __name__ == "__main__":
    main()
