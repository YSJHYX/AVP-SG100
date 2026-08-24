"""Stable dual-camera observation contract for VLA training and deployment.

The MuJoCo camera names are scene implementation details.  VLA datasets and
policies consume the observation keys below.  Keeping the mapping in one
module prevents offline-rendered training data and online MuJoCo deployment
from silently swapping head and wrist images.
"""

from __future__ import annotations

from collections.abc import Mapping

import mujoco
import numpy as np


CAMERA_TO_OBSERVATION_KEY: Mapping[str, str] = {
    "workstation_front": "base_0_rgb",
    "wrist_camera": "right_wrist_0_rgb",
}


def validate_camera_observation_contract(
    model: mujoco.MjModel,
    camera_to_key: Mapping[str, str] = CAMERA_TO_OBSERVATION_KEY,
) -> dict[str, int]:
    """Return camera IDs after validating names and unique VLA keys."""

    if len(set(camera_to_key.values())) != len(camera_to_key):
        raise ValueError("Camera observation keys must be unique")
    camera_ids: dict[str, int] = {}
    for camera_name in camera_to_key:
        camera_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name
        )
        if camera_id < 0:
            raise ValueError(
                f"Required VLA camera {camera_name!r} not found in MuJoCo model"
            )
        camera_ids[camera_name] = camera_id
    return camera_ids


def render_camera_observations(
    renderer: mujoco.Renderer,
    data: mujoco.MjData,
    camera_to_key: Mapping[str, str] = CAMERA_TO_OBSERVATION_KEY,
) -> dict[str, np.ndarray]:
    """Render synchronized HWC uint8 RGB observations from one MuJoCo state.

    The caller advances or restores ``data`` exactly once.  This function
    only changes the renderer's active view, so both returned images describe
    the same robot/object state and can be used unchanged by an online policy
    or by an offline dataset conversion job.
    """

    observations: dict[str, np.ndarray] = {}
    for camera_name, observation_key in camera_to_key.items():
        renderer.update_scene(data, camera=camera_name)
        pixels = np.ascontiguousarray(renderer.render()).copy()
        if pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] != 3:
            raise RuntimeError(
                f"Unexpected RGB frame for {camera_name!r}: "
                f"shape={pixels.shape}, dtype={pixels.dtype}"
            )
        observations[observation_key] = pixels
    return observations
