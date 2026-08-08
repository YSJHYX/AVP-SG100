import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from avp_stream import VisionProStreamer

from sg100_teleop.calibration.io import (
    load_calibration,
)
from sg100_teleop.control.joint_command_filter import (
    JointCommandFilter,
)
from sg100_teleop.retargeting.sg100_retargeter import (
    SG100Retargeter,
)
from sg100_teleop.tracking.avp_adapter import (
    AVPHandAdapter,
)
from sg100_teleop.tracking.avp_source import (
    get_hand_from_tracking,
)


DEFAULT_MODEL = (
    "sg100_teleop/assets/sg100/"
    "mjcf/sg100_sim.xml"
)

DEFAULT_CALIBRATION = (
    "sg100_teleop/config/"
    "avp_right_calibration.yaml"
)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--ip",
        required=True,
        help="Apple Vision Pro IP",
    )

    parser.add_argument(
        "--model",
        default=DEFAULT_MODEL,
    )

    parser.add_argument(
        "--calibration",
        default=DEFAULT_CALIBRATION,
    )

    parser.add_argument(
        "--port",
        type=int,
        default=50051,
    )

    parser.add_argument(
        "--tracking-hz",
        type=float,
        default=60.0,
    )

    parser.add_argument(
        "--stream-hz",
        type=float,
        default=60.0,
    )

    parser.add_argument(
        "--filter-tau",
        type=float,
        default=0.06,
    )

    parser.add_argument(
        "--max-velocity",
        type=float,
        default=4.0,
        help="Max SG100 command velocity rad/s",
    )

    parser.add_argument(
        "--tracking-timeout",
        type=float,
        default=0.5,
        help=(
            "Seconds before reporting "
            "tracking loss"
        ),
    )

    parser.add_argument(
        "--attach",
        nargs=4,
        type=float,
        default=[
            0.0,
            0.7,
            0.8,
            0.0,
        ],
        metavar=(
            "X",
            "Y",
            "Z",
            "YAW_DEG",
        ),
        help=(
            "AVP MuJoCo placement: "
            "x y z yaw_deg"
        ),
    )

    parser.add_argument(
        "--verbose",
        action="store_true",
    )

    args = parser.parse_args()

    model_path = Path(
        args.model
    )

    calibration_path = Path(
        args.calibration
    )

    if not model_path.exists():
        raise FileNotFoundError(
            model_path
        )

    if not calibration_path.exists():
        raise FileNotFoundError(
            "Calibration not found: "
            f"{calibration_path}\n"
            "Run calibrate_avp_hand first."
        )

    print(
        "===== AVP -> SG100 SIM ====="
    )

    print(
        "model       :",
        model_path,
    )

    print(
        "calibration :",
        calibration_path,
    )

    print(
        "Vision Pro  :",
        args.ip,
    )

    # ---------------------------------------------------------
    # MuJoCo
    # ---------------------------------------------------------

    model = mujoco.MjModel.from_xml_path(
        str(model_path)
    )

    data = mujoco.MjData(
        model
    )

    if (
        model.nq != 11
        or model.nu != 11
    ):
        raise RuntimeError(
            "Expected SG100 model with "
            f"nq=11, nu=11; "
            f"got nq={model.nq}, "
            f"nu={model.nu}"
        )

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0

    mujoco.mj_forward(
        model,
        data,
    )

    # ---------------------------------------------------------
    # Retargeting pipeline
    # ---------------------------------------------------------

    calibration = load_calibration(
        calibration_path
    )

    avp_adapter = AVPHandAdapter(
        calibration
    )

    retargeter = (
        SG100Retargeter()
    )

    command_filter = (
        JointCommandFilter(
            dof=11,
            tau=args.filter_tau,
            max_velocity=(
                args.max_velocity
            ),
        )
    )

    command_filter.reset(
        np.zeros(11)
    )

    q_goal = np.zeros(
        11,
        dtype=np.float64,
    )

    # ---------------------------------------------------------
    # Vision Pro
    # ---------------------------------------------------------

    streamer = VisionProStreamer(
        ip=args.ip,
        record=False,
        verbose=args.verbose,
    )

    streamer.configure_mujoco(
        xml_path=str(
            model_path
        ),
        model=model,
        data=data,
        relative_to=list(
            args.attach
        ),
        grpc_port=args.port,
    )

    streamer.start_webrtc()

    # ---------------------------------------------------------
    # Timing
    # ---------------------------------------------------------

    physics_dt = float(
        model.opt.timestep
    )

    tracking_period = (
        1.0
        / args.tracking_hz
    )

    stream_period = (
        1.0
        / args.stream_hz
    )

    last_wall = (
        time.perf_counter()
    )

    physics_accumulator = 0.0

    last_tracking_query = (
        -float("inf")
    )

    last_valid_tracking = (
        -float("inf")
    )

    last_stream = (
        -float("inf")
    )

    last_print = (
        -float("inf")
    )

    tracking_lost_printed = False

    print()
    print(
        "Waiting for right-hand "
        "tracking..."
    )

    try:
        with (
            mujoco.viewer.launch_passive(
                model,
                data,
            )
            as viewer
        ):

            while viewer.is_running():

                now = (
                    time.perf_counter()
                )

                wall_dt = (
                    now
                    - last_wall
                )

                last_wall = now

                # Prevent giant catch-up after
                # debugger/window pauses.
                wall_dt = min(
                    wall_dt,
                    0.05,
                )

                physics_accumulator += (
                    wall_dt
                )

                # ---------------------------------------------
                # AVP tracking update (~60 Hz)
                # ---------------------------------------------

                if (
                    now
                    - last_tracking_query
                    >= tracking_period
                ):
                    last_tracking_query = (
                        now
                    )

                    tracking = (
                        streamer.get_latest()
                    )

                    hand = (
                        get_hand_from_tracking(
                            tracking,
                            side="right",
                        )
                    )

                    if hand is not None:

                        try:
                            canonical = (
                                avp_adapter.convert(
                                    hand
                                )
                            )

                            q_goal = (
                                retargeter.retarget(
                                    canonical
                                )
                            )

                            last_valid_tracking = (
                                now
                            )

                            if (
                                tracking_lost_printed
                            ):
                                print(
                                    "Tracking recovered."
                                )

                                tracking_lost_printed = (
                                    False
                                )

                        except (
                            ValueError,
                            np.linalg.LinAlgError,
                        ) as exc:

                            if args.verbose:
                                print(
                                    "Tracking frame "
                                    "rejected:",
                                    exc,
                                )

                # ---------------------------------------------
                # Tracking-loss behavior
                #
                # HOLD the last goal.
                # Do not snap to q=0.
                # ---------------------------------------------

                tracking_age = (
                    now
                    - last_valid_tracking
                )

                if (
                    tracking_age
                    > args.tracking_timeout
                    and not tracking_lost_printed
                ):
                    print(
                        "WARNING: hand tracking "
                        "lost; holding last "
                        "SG100 target."
                    )

                    tracking_lost_printed = (
                        True
                    )

                # ---------------------------------------------
                # MuJoCo physics @ 500 Hz
                # ---------------------------------------------

                while (
                    physics_accumulator
                    >= physics_dt
                ):
                    q_cmd = (
                        command_filter.update(
                            q_goal,
                            physics_dt,
                        )
                    )

                    data.ctrl[:] = q_cmd

                    mujoco.mj_step(
                        model,
                        data,
                    )

                    physics_accumulator -= (
                        physics_dt
                    )

                # ---------------------------------------------
                # Local viewer
                # ---------------------------------------------

                viewer.sync()

                # ---------------------------------------------
                # Stream MuJoCo state back to AVP
                # ---------------------------------------------

                if (
                    now
                    - last_stream
                    >= stream_period
                ):
                    streamer.update_sim()

                    last_stream = now

                # ---------------------------------------------
                # Low-rate diagnostics
                # ---------------------------------------------

                if (
                    args.verbose
                    and (
                        now - last_print
                        >= 0.5
                    )
                ):
                    print()

                    print(
                        "q_goal =",
                        np.round(
                            q_goal,
                            3,
                        ),
                    )

                    print(
                        "q_cmd  =",
                        np.round(
                            command_filter.value,
                            3,
                        ),
                    )

                    print(
                        "qpos   =",
                        np.round(
                            data.qpos,
                            3,
                        ),
                    )

                    last_print = now

                time.sleep(0.001)

    except KeyboardInterrupt:
        pass

    print()
    print(
        "AVP -> SG100 simulation stopped."
    )


if __name__ == "__main__":
    main()
