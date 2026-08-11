from pathlib import Path
import math

import mujoco
import numpy as np

from sg100_teleop.kinematics.rm65_ik import RM65IKSolver
from sg100_teleop.sim.run_avp_arm_hand_sim import (
    RM65_TELEOP_READY_Q,
    TCP_BODY,
)
from sg100_teleop.tracking.avp_wrist_mapper import (
    AVPWristRelativeMapper,
    make_transform,
    rotation_angle,
)


ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = ROOT / "sg100_teleop/assets/rm65_sg100/mjcf/rm65_sg100_sim.xml"
A = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return (
        np.eye(3)
        + math.sin(angle) * skew
        + (1.0 - math.cos(angle)) * (skew @ skew)
    )


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    solver = RM65IKSolver(model, tcp_body_name=TCP_BODY)
    p_tcp0, R_tcp0 = solver.fk(RM65_TELEOP_READY_Q)
    T_tcp0 = make_transform(p_tcp0, R_tcp0)

    mapper = AVPWristRelativeMapper(
        translation_scale=0.5,
        R_tcp_from_avp_local=A,
    )
    T_avp0 = np.eye(4)
    mapper.calibrate(T_avp0, T_tcp0)

    cases = []
    for index, name in enumerate("XYZ"):
        delta = np.zeros(3)
        delta[index] = 0.004
        cases.append((f"T{name}", make_transform(delta, np.eye(3))))
    for index, name in enumerate("XYZ"):
        cases.append(
            (
                f"R{name}",
                make_transform(
                    np.zeros(3),
                    axis_rotation(np.eye(3)[index], math.radians(1.0)),
                ),
            )
        )

    for name, T_avp_relative in cases:
        target = mapper.map(T_avp0 @ T_avp_relative)
        q_arm, result = solver.solve(
            q_seed=RM65_TELEOP_READY_Q,
            target_position=target[:3, 3],
            target_rotation=target[:3, :3],
        )
        if not result.converged:
            raise RuntimeError(f"Mapped target {name} did not converge: {result}")

        p_fk, R_fk = solver.fk(q_arm)
        position_error = float(np.linalg.norm(p_fk - target[:3, 3]))
        rotation_error = rotation_angle(target[:3, :3] @ R_fk.T)
        if position_error > 1e-4 or rotation_error > 1e-3:
            raise RuntimeError(
                f"Mapped target {name} FK mismatch: "
                f"position={position_error}, rotation={rotation_error}"
            )
        print(
            f"{name}: position_error={position_error * 1000.0:.4f} mm, "
            f"rotation_error={math.degrees(rotation_error):.5f} deg"
        )

    print("CONFIGURED AVP BASIS -> RM65 IK 6D SMOKE: PASS")


if __name__ == "__main__":
    main()
