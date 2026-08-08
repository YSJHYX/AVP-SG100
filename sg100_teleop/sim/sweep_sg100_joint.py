import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np


MODEL_PATH = (
    "sg100_teleop/assets/sg100/mjcf/"
    "sg100_sim.xml"
)


def smoothstep(x):
    return x * x * (3.0 - 2.0 * x)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--joint",
        required=True,
        help="Joint name, e.g. index_j1",
    )
    parser.add_argument(
        "--segment-time",
        type=float,
        default=2.0,
        help="Seconds for each interpolation segment",
    )
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    joint_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        args.joint,
    )

    if joint_id < 0:
        raise ValueError(f"Unknown joint: {args.joint}")

    qpos_adr = model.jnt_qposadr[joint_id]
    lower, upper = model.jnt_range[joint_id]

    actuator_name = f"{args.joint}_pos"

    actuator_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        actuator_name,
    )

    if actuator_id < 0:
        raise ValueError(
            f"Actuator not found: {actuator_name}"
        )

    print()
    print("===== SG100 JOINT SWEEP =====")
    print("joint       :", args.joint)
    print("joint id    :", joint_id)
    print("qpos addr   :", qpos_adr)
    print("actuator id :", actuator_id)
    print("range       :", [lower, upper])
    print()

    # All other joints stay at mathematical zero.
    q_target = np.zeros(model.nu)

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = q_target

    mujoco.mj_forward(model, data)

    # Visit:
    # 0 -> lower -> 0 -> upper -> 0
    targets = [
        ("ZERO", 0.0),
        ("LOWER", lower),
        ("ZERO", 0.0),
        ("UPPER", upper),
        ("ZERO", 0.0),
    ]

    dt = model.opt.timestep
    steps_per_segment = int(args.segment_time / dt)

    with mujoco.viewer.launch_passive(model, data) as viewer:

        while viewer.is_running():

            for label, target in targets:

                if not viewer.is_running():
                    break

                start = float(data.ctrl[actuator_id])

                print(
                    f"{args.joint:12s} "
                    f"→ {label:5s} "
                    f"{target:+.4f} rad"
                )

                for k in range(steps_per_segment):

                    if not viewer.is_running():
                        break

                    alpha = smoothstep(
                        (k + 1) / steps_per_segment
                    )

                    cmd = (
                        start
                        + alpha * (target - start)
                    )

                    q_target[actuator_id] = cmd
                    data.ctrl[:] = q_target

                    mujoco.mj_step(model, data)

                    viewer.sync()

                    time.sleep(dt)

                # Hold briefly at endpoint
                hold_steps = int(0.8 / dt)

                for _ in range(hold_steps):

                    if not viewer.is_running():
                        break

                    q_target[actuator_id] = target
                    data.ctrl[:] = q_target

                    mujoco.mj_step(model, data)

                    viewer.sync()

                    time.sleep(dt)


if __name__ == "__main__":
    main()
