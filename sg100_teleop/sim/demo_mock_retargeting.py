import time

import mujoco
import mujoco.viewer
import numpy as np

from sg100_teleop.hand.canonical_hand import (
    CanonicalHandState,
)
from sg100_teleop.retargeting.sg100_retargeter import (
    SG100Retargeter,
)


MODEL_PATH = (
    "sg100_teleop/assets/sg100/mjcf/"
    "sg100_sim.xml"
)


def smoothstep(x: float) -> float:
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


POSES = {

    "neutral": CanonicalHandState(),

    "open": CanonicalHandState(
        thumb_abduction=0.45,
        thumb_distal=-0.25,

        index_distal=-0.20,
        middle_distal=-0.20,

        little_outward=0.20,
        little_distal=-0.20,
    ),

    "fist": CanonicalHandState(
        thumb_opposition=0.75,
        thumb_abduction=0.10,
        thumb_distal=0.80,

        index_adduction=0.15,
        index_flexion=0.90,
        index_distal=0.90,

        middle_flexion=0.90,
        middle_distal=0.90,

        little_outward=0.05,
        little_flexion=0.90,
        little_distal=0.90,
    ),

    "pinch": CanonicalHandState(
        thumb_opposition=0.70,
        thumb_abduction=0.20,
        thumb_distal=0.55,

        index_adduction=0.45,
        index_flexion=0.45,
        index_distal=0.55,

        middle_flexion=0.15,
        middle_distal=0.15,

        little_outward=0.05,
        little_flexion=0.15,
        little_distal=0.15,
    ),
}


def main():

    model = mujoco.MjModel.from_xml_path(
        MODEL_PATH
    )
    data = mujoco.MjData(model)

    retargeter = SG100Retargeter()

    if model.nu != 11:
        raise RuntimeError(
            f"Expected nu=11, got {model.nu}"
        )

    q_current_cmd = np.zeros(11)

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0

    mujoco.mj_forward(model, data)

    sequence = [
        "neutral",
        "open",
        "neutral",
        "fist",
        "neutral",
        "pinch",
        "neutral",
    ]

    transition_time = 2.0
    hold_time = 1.0

    dt = model.opt.timestep

    with mujoco.viewer.launch_passive(
        model,
        data,
    ) as viewer:

        while viewer.is_running():

            for pose_name in sequence:

                if not viewer.is_running():
                    break

                hand = POSES[pose_name]
                q_goal = retargeter.retarget(hand)

                print()
                print(
                    f"===== POSE: {pose_name.upper()} ====="
                )

                for joint, value in (
                    retargeter
                    .as_joint_dict(q_goal)
                    .items()
                ):
                    print(
                        f"{joint:12s}: "
                        f"{value:+.3f}"
                    )

                q_start = q_current_cmd.copy()

                steps = int(
                    transition_time / dt
                )

                for k in range(steps):

                    if not viewer.is_running():
                        break

                    alpha = smoothstep(
                        (k + 1) / steps
                    )

                    q_current_cmd = (
                        q_start
                        + alpha * (
                            q_goal - q_start
                        )
                    )

                    data.ctrl[:] = q_current_cmd

                    mujoco.mj_step(
                        model,
                        data,
                    )

                    viewer.sync()
                    time.sleep(dt)

                q_current_cmd = q_goal.copy()

                hold_steps = int(
                    hold_time / dt
                )

                for _ in range(hold_steps):

                    if not viewer.is_running():
                        break

                    data.ctrl[:] = q_current_cmd

                    mujoco.mj_step(
                        model,
                        data,
                    )

                    viewer.sync()
                    time.sleep(dt)


if __name__ == "__main__":
    main()
