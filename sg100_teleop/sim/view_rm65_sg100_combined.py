from __future__ import annotations

import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]

MODEL_PATH = (
    REPO_ROOT
    / "sg100_teleop"
    / "assets"
    / "rm65_sg100"
    / "mjcf"
    / "rm65_sg100_sim.xml"
)


def smoothstep(x: float) -> float:
    x = float(np.clip(x, 0.0, 1.0))
    return x * x * (3.0 - 2.0 * x)


class RealtimeStepper:
    def __init__(self, model, viewer, render_every=8):
        self.model = model
        self.viewer = viewer
        self.render_every = render_every
        self.step_count = 0

    def step(self, data):
        t0 = time.perf_counter()

        mujoco.mj_step(
            self.model,
            data,
        )

        self.step_count += 1

        if self.step_count % self.render_every == 0:
            self.viewer.sync()

        elapsed = time.perf_counter() - t0

        remain = (
            self.model.opt.timestep
            - elapsed
        )

        if remain > 0:
            time.sleep(remain)


def move_to(
    model,
    data,
    stepper,
    target,
    duration=2.0,
):
    start = data.ctrl.copy()

    steps = max(
        1,
        int(duration / model.opt.timestep),
    )

    for i in range(steps):
        s = smoothstep((i + 1) / steps)

        data.ctrl[:] = (
            start + s * (target - start)
        )

        stepper.step(data)


def hold(model, data, stepper, duration=1.5):
    steps = max(
        1,
        int(duration / model.opt.timestep),
    )

    for _ in range(steps):
        stepper.step(data)


def main():
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )

    data = mujoco.MjData(model)

    if model.nq != 17 or model.nu != 17:
        raise RuntimeError(
            f"Expected nq=17, nu=17; "
            f"got nq={model.nq}, nu={model.nu}"
        )

    zero = np.zeros(17)

    # Moderate arm pose for visual inspection.
    arm_pose = np.zeros(17)
    arm_pose[:6] = [
        0.35,
        -0.45,
        0.40,
        -0.25,
        0.30,
        0.25,
    ]

    # SG100 motion while RM65 remains zero.
    hand_pose = np.zeros(17)
    hand_pose[6:] = [
         0.55,   # thumb_j1
        -0.55,   # thumb_j2
         0.45,   # thumb_j3

         0.25,   # index_j1
         0.65,   # index_j2
         0.45,   # index_j3

        -0.65,   # middle_j1
         0.45,   # middle_j2

        -0.55,   # little_j1
         0.65,   # little_j2
         0.45,   # little_j3
    ]

    combined_pose = arm_pose.copy()
    combined_pose[6:] = hand_pose[6:]

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0

    mujoco.mj_forward(
        model,
        data,
    )

    print("===== RM65 + SG100 VIEWER =====")
    print("Model:", MODEL_PATH)
    print()
    print("Sequence:")
    print("  ZERO")
    print("  ARM ONLY")
    print("  ZERO")
    print("  HAND ONLY")
    print("  ZERO")
    print("  ARM + HAND")
    print("  ZERO")
    print()

    with mujoco.viewer.launch_passive(
        model,
        data,
    ) as viewer:

        stepper = RealtimeStepper(
            model,
            viewer,
            render_every=8,
        )

        viewer.cam.distance = 1.4
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        viewer.cam.lookat[:] = [
            0.0,
            0.0,
            0.45,
        ]

        print("ZERO")
        hold(
            model,
            data,
            stepper,
            2.0,
        )

        print("ARM ONLY")
        move_to(
            model,
            data,
            stepper,
            arm_pose,
        )
        hold(
            model,
            data,
            stepper,
        )

        print("ZERO")
        move_to(
            model,
            data,
            stepper,
            zero,
        )
        hold(
            model,
            data,
            stepper,
        )

        print("HAND ONLY")
        move_to(
            model,
            data,
            stepper,
            hand_pose,
        )
        hold(
            model,
            data,
            stepper,
        )

        print("ZERO")
        move_to(
            model,
            data,
            stepper,
            zero,
        )
        hold(
            model,
            data,
            stepper,
        )

        print("ARM + HAND")
        move_to(
            model,
            data,
            stepper,
            combined_pose,
        )
        hold(
            model,
            data,
            stepper,
            2.0,
        )

        print("ZERO")
        move_to(
            model,
            data,
            stepper,
            zero,
        )
        hold(
            model,
            data,
            stepper,
            2.0,
        )

        print("\nFinal qpos:")
        print(data.qpos)

        print(
            "\nSequence complete. "
            "Close viewer to exit."
        )

        while viewer.is_running():
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    main()
