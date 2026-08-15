"""Interactive viewer for the AVP grasp scene.

Run from the repo root:

    python -m sg100_teleop.sim.view_avp_grasp_scene

Loads ``rm65_sg100_avp_grasp_scene.xml``, puts the arm in the teleop-ready
pose with the hand open (the real AVP startup state), and runs a real-time
passive viewer. The arm is held with the same gravity-compensation
feedforward the AVP script will use, so the bottle rests on the table while
you look around.

In-viewer keys worth knowing:
    1 / 2   toggle geom group 1 (arm/hand visual meshes) / 2
    3       toggle geom group 3  -> the cyan finger + link-2 collision
            capsules and the green palm backstop box
    c       toggle contact points / forces
    Space   pause / resume physics
"""

from __future__ import annotations

import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from sg100_teleop.sim.combined_robot_layout import (
    ARM_DOF,
    HAND_DOF,
    initialize_robot_state,
    resolve_robot_layout,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_PATH = (
    REPO_ROOT
    / "sg100_teleop"
    / "assets"
    / "rm65_sg100"
    / "mjcf"
    / "rm65_sg100_avp_grasp_robot.xml"
)

# Mirrors RM65_TELEOP_READY_Q in run_avp_arm_hand_sim.py (inlined to avoid the
# AVP/hardware imports that module pulls in at load time).
READY_Q = np.array(
    [0.32488, -0.76769, 0.97621, 0.22463, -1.80649, -0.12430],
    dtype=np.float64,
)

RENDER_FPS = 60


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    layout = resolve_robot_layout(model)
    arm_dof = layout.dof_indices[:ARM_DOF]

    initialize_robot_state(model, data, layout, READY_Q, np.zeros(HAND_DOF))

    steps_per_frame = max(1, int(round(1.0 / (RENDER_FPS * model.opt.timestep))))
    frame_dt = 1.0 / RENDER_FPS

    with mujoco.viewer.launch_passive(model, data) as viewer:
        next_frame = time.time()
        while viewer.is_running():
            # Hold the arm against gravity; the free bottle keeps gravity.
            for _ in range(steps_per_frame):
                data.qfrc_applied[arm_dof] = data.qfrc_bias[arm_dof]
                mujoco.mj_step(model, data)
            viewer.sync()

            # Pace to real time.
            next_frame += frame_dt
            sleep_for = next_frame - time.time()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_frame = time.time()


if __name__ == "__main__":
    main()
