"""Open the robot-only realworld scene at its grasp-ready keyframe."""

from __future__ import annotations

from pathlib import Path

import mujoco


REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_PATH = (
    REPO_ROOT
    / "sg100_teleop/assets/rm65_sg100/mjcf/"
    "rm65_sg100_realworld_teleop_scene.xml"
)
READY_KEYFRAME = "realworld_ready"


def load_ready_scene() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    keyframe_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_KEY, READY_KEYFRAME
    )
    if keyframe_id < 0:
        raise RuntimeError(f"Missing MuJoCo keyframe: {READY_KEYFRAME}")
    mujoco.mj_resetDataKeyframe(model, data, keyframe_id)
    mujoco.mj_forward(model, data)
    return model, data


def main() -> None:
    import mujoco.viewer

    model, data = load_ready_scene()
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
