"""Camera contract regression: frozen V8 cameras vs the demo scenes.

Every demo scene must preserve the V8 production cameras exactly:
same name set, same body attachment (world for the two fixed cameras,
sg100_base_link for the wrist sensor), same local position, same
orientation, same fovy.  This is the PART 10 contract test of the
demo scene suite spec.
"""

from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[2]
MJCF = ROOT / "sg100_teleop/assets/rm65_sg100/mjcf"

V8_SCENE = MJCF / "rm65_sg100_avp_grasp_scene.xml"
DEMO_SCENES = {
    "demo01_hammer_peg_assembly": (
        MJCF / "rm65_sg100_demo_hammer_peg_assembly_scene.xml"
    ),
    "demo02_drawer_retrieval": (
        MJCF
        / "rm65_sg100_demo_drawer_retrieval_and_loading_scene.xml"
    ),
    "demo03_maintenance_kit_packing": (
        MJCF / "rm65_sg100_demo_maintenance_kit_packing_scene.xml"
    ),
}

# Camera-name -> expected parent body ("world" for world-fixed).
EXPECTED_CAMERAS = {
    "workstation_front": "world",
    "wrist_camera": "sg100_base_link",
    "demo_camera": "world",
}

POSE_TOLERANCE = 0.0      # byte-identical camera definitions
FOVY_TOLERANCE = 0.0


def _camera_record(model: mujoco.MjModel) -> dict[str, dict]:
    records = {}
    for cam_id in range(model.ncam):
        name = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_CAMERA, cam_id
        )
        body_id = int(model.cam_bodyid[cam_id])
        body_name = (
            "world"
            if body_id == 0
            else mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY, body_id
            )
        )
        # cam_mat is stored row-major; reshape to 3x3 (columns are
        # the camera X and Y axes of the XML xyaxes attribute).
        rot = model.cam_mat0[cam_id].reshape(3, 3).copy()
        records[name] = {
            "body": body_name,
            "pos": model.cam_pos[cam_id].copy(),
            "rot": rot,
            "fovy": float(model.cam_fovy[cam_id]),
        }
    return records


@pytest.fixture(scope="module")
def v8_cameras() -> dict[str, dict]:
    model = mujoco.MjModel.from_xml_path(str(V8_SCENE))
    return _camera_record(model)


@pytest.mark.parametrize("demo_name", sorted(DEMO_SCENES))
def test_demo_scene_camera_count(demo_name, v8_cameras):
    model = mujoco.MjModel.from_xml_path(
        str(DEMO_SCENES[demo_name])
    )
    assert model.ncam == len(v8_cameras)
    assert model.ncam == len(EXPECTED_CAMERAS)


@pytest.mark.parametrize("demo_name", sorted(DEMO_SCENES))
def test_demo_scene_camera_names(demo_name, v8_cameras):
    model = mujoco.MjModel.from_xml_path(
        str(DEMO_SCENES[demo_name])
    )
    demo = _camera_record(model)
    assert set(demo) == set(v8_cameras) == set(EXPECTED_CAMERAS)


@pytest.mark.parametrize("demo_name", sorted(DEMO_SCENES))
def test_demo_scene_camera_contract(demo_name, v8_cameras):
    """Attachment, pose, orientation and fovy must match V8 exactly."""
    model = mujoco.MjModel.from_xml_path(
        str(DEMO_SCENES[demo_name])
    )
    demo = _camera_record(model)

    for name, expected_body in EXPECTED_CAMERAS.items():
        v8 = v8_cameras[name]
        got = demo[name]

        assert got["body"] == expected_body == v8["body"], (
            f"{demo_name}/{name}: attachment {got['body']} != "
            f"{expected_body}"
        )
        assert np.array_equal(got["pos"], v8["pos"]), (
            f"{demo_name}/{name}: local position changed"
        )
        assert np.array_equal(got["rot"], v8["rot"]), (
            f"{demo_name}/{name}: orientation changed"
        )
        assert got["fovy"] == pytest.approx(
            v8["fovy"], abs=FOVY_TOLERANCE
        ), f"{demo_name}/{name}: fovy changed"


def test_v8_camera_contract_selfcheck(v8_cameras):
    """The V8 reference itself must match the frozen expectation."""
    for name, expected_body in EXPECTED_CAMERAS.items():
        assert v8_cameras[name]["body"] == expected_body
    assert v8_cameras["workstation_front"]["fovy"] == 50.0
    assert v8_cameras["demo_camera"]["fovy"] == 60.0
    assert v8_cameras["wrist_camera"]["fovy"] == 75.0
