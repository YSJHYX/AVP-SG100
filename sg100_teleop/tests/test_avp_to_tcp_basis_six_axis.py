import math

import numpy as np

from sg100_teleop.tracking.avp_wrist_mapper import (
    AVPWristRelativeMapper,
    make_transform,
    rotation_angle,
)


A = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)
SCALE = 0.4


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


def calibrated_mapper():
    mapper = AVPWristRelativeMapper(
        translation_scale=SCALE,
        R_tcp_from_avp_local=A,
    )
    T_avp0 = make_transform(np.array([0.2, -0.1, 0.3]), np.eye(3))
    T_tcp0 = make_transform(
        np.array([0.4, 0.2, 0.6]),
        axis_rotation([0.0, 0.0, 1.0], 0.3),
    )
    mapper.calibrate(T_avp0, T_tcp0)
    return mapper, T_avp0, T_tcp0


def test_positive_and_negative_xyz_translation():
    mapper, T_avp0, T_tcp0 = calibrated_mapper()
    distance = 0.1
    for axis_index, axis_name in enumerate("XYZ"):
        for sign in (-1.0, 1.0):
            delta = np.zeros(3)
            delta[axis_index] = sign * distance
            T_avp = make_transform(
                T_avp0[:3, 3] + delta,
                T_avp0[:3, :3],
            )
            T_target = mapper.map(T_avp)
            delta_tcp_world = T_target[:3, 3] - T_tcp0[:3, 3]
            expected = SCALE * (A @ delta)
            np.testing.assert_allclose(delta_tcp_world, expected, atol=1e-12)
            print(
                f"AVP world {sign:+.0f}{axis_name} -> TCP world",
                np.round(delta_tcp_world, 6),
            )


def test_positive_and_negative_xyz_rotation():
    mapper, T_avp0, T_tcp0 = calibrated_mapper()
    angle = math.radians(5.0)
    axes = np.eye(3)
    for axis_index, axis_name in enumerate("XYZ"):
        for sign in (-1.0, 1.0):
            R_avp = axis_rotation(axes[axis_index], sign * angle)
            T_avp = make_transform(
                T_avp0[:3, 3],
                R_avp @ T_avp0[:3, :3],
            )
            T_target = mapper.map(T_avp)
            R_actual = T_target[:3, :3] @ T_tcp0[:3, :3].T
            R_expected = A @ R_avp @ A.T
            error = rotation_angle(R_expected @ R_actual.T)
            if error > 1e-10:
                raise RuntimeError(
                    f"World {axis_name} rotation basis error: {error} rad"
                )
            print(
                f"AVP world {sign:+.0f}R{axis_name} -> TCP world axis",
                np.round(A @ axes[axis_index] * sign, 3),
            )


def main():
    test_positive_and_negative_xyz_translation()
    test_positive_and_negative_xyz_rotation()
    print("AVP-TO-TCP NONIDENTITY SIX-AXIS MAPPING: PASS")


if __name__ == "__main__":
    main()
