from __future__ import annotations

import argparse
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
    / "RM65-B"
    / "mjcf"
    / "rm65_sim.xml"
)

JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]


def smoothstep(x: float) -> float:
    x = float(np.clip(x, 0.0, 1.0))
    return x * x * (3.0 - 2.0 * x)


def run_realtime_step(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    viewer,
) -> None:
    start = time.perf_counter()

    mujoco.mj_step(model, data)
    viewer.sync()

    elapsed = time.perf_counter() - start
    remaining = model.opt.timestep - elapsed

    if remaining > 0:
        time.sleep(remaining)


def move_to(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    viewer,
    target: np.ndarray,
    duration: float,
) -> None:
    start_ctrl = data.ctrl.copy()

    steps = max(
        1,
        int(round(duration / model.opt.timestep)),
    )

    for i in range(steps):
        ratio = (i + 1) / steps
        blend = smoothstep(ratio)

        data.ctrl[:] = (
            start_ctrl
            + blend * (target - start_ctrl)
        )

        run_realtime_step(model, data, viewer)


def hold(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    viewer,
    duration: float,
) -> None:
    steps = max(
        1,
        int(round(duration / model.opt.timestep)),
    )

    for _ in range(steps):
        run_realtime_step(model, data, viewer)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visually sweep one RM65 joint."
    )

    parser.add_argument(
        "--joint",
        required=True,
        choices=JOINT_NAMES,
        help="RM65 joint to sweep.",
    )

    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.5,
        help=(
            "Maximum sweep amplitude in radians. "
            "The value is clipped to the URDF joint range."
        ),
    )

    parser.add_argument(
        "--move-time",
        type=float,
        default=1.5,
        help="Seconds for each transition.",
    )

    parser.add_argument(
        "--hold-time",
        type=float,
        default=1.0,
        help="Seconds to hold each target.",
    )

    args = parser.parse_args()

    if args.amplitude <= 0:
        raise ValueError("--amplitude must be > 0")

    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )
    data = mujoco.MjData(model)

    if model.nq != 6 or model.nu != 6:
        raise RuntimeError(
            f"Expected RM65 nq=6, nu=6, "
            f"got nq={model.nq}, nu={model.nu}"
        )

    joint_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        args.joint,
    )

    actuator_name = f"{args.joint}_pos"

    actuator_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        actuator_name,
    )

    if joint_id < 0:
        raise RuntimeError(
            f"Joint not found: {args.joint}"
        )

    if actuator_id < 0:
        raise RuntimeError(
            f"Actuator not found: {actuator_name}"
        )

    lower, upper = model.jnt_range[joint_id]

    negative_target = max(
        float(lower),
        -args.amplitude,
    )

    positive_target = min(
        float(upper),
        args.amplitude,
    )

    zero = np.zeros(model.nu, dtype=np.float64)
    negative = zero.copy()
    positive = zero.copy()

    negative[actuator_id] = negative_target
    positive[actuator_id] = positive_target

    print("===== RM65 JOINT SWEEP =====")
    print("model:", MODEL_PATH)
    print("joint:", args.joint)
    print("joint_id:", joint_id)
    print("actuator:", actuator_name)
    print("actuator_id:", actuator_id)
    print(
        "URDF range:",
        [float(lower), float(upper)],
    )
    print("negative test:", negative_target)
    print("positive test:", positive_target)

    print(
        "\nSequence:\n"
        "ZERO\n"
        " -> NEGATIVE\n"
        " -> ZERO\n"
        " -> POSITIVE\n"
        " -> ZERO\n"
    )

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0

    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(
        model,
        data,
    ) as viewer:

        # Camera only affects visualization.
        viewer.cam.distance = 1.4
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        viewer.cam.lookat[:] = [0.0, 0.0, 0.4]

        print("ZERO")
        hold(
            model,
            data,
            viewer,
            args.hold_time,
        )

        print(
            f"NEGATIVE: "
            f"{args.joint} = "
            f"{negative_target:.4f} rad"
        )
        move_to(
            model,
            data,
            viewer,
            negative,
            args.move_time,
        )
        hold(
            model,
            data,
            viewer,
            args.hold_time,
        )

        print("ZERO")
        move_to(
            model,
            data,
            viewer,
            zero,
            args.move_time,
        )
        hold(
            model,
            data,
            viewer,
            args.hold_time,
        )

        print(
            f"POSITIVE: "
            f"{args.joint} = "
            f"{positive_target:.4f} rad"
        )
        move_to(
            model,
            data,
            viewer,
            positive,
            args.move_time,
        )
        hold(
            model,
            data,
            viewer,
            args.hold_time,
        )

        print("ZERO")
        move_to(
            model,
            data,
            viewer,
            zero,
            args.move_time,
        )
        hold(
            model,
            data,
            viewer,
            args.hold_time,
        )

        print("\nFINAL qpos:")
        print(data.qpos)

        print(
            "\nSweep finished. "
            "Close the viewer window to exit."
        )

        while viewer.is_running():
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    main()
