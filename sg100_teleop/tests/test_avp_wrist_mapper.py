import math
import numpy as np

from sg100_teleop.tracking.avp_wrist_mapper import (
    AVPWristRelativeMapper,
    make_transform,
    rotation_angle,
)


def rot_x(theta):
    c = math.cos(theta)
    s = math.sin(theta)

    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c],
    ], dtype=np.float64)


def rot_y(theta):
    c = math.cos(theta)
    s = math.sin(theta)

    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c],
    ], dtype=np.float64)


def rot_z(theta):
    c = math.cos(theta)
    s = math.sin(theta)

    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ], dtype=np.float64)


def pose_error(
    T_actual,
    T_expected,
):
    p_error = np.linalg.norm(
        T_actual[:3, 3]
        - T_expected[:3, 3]
    )

    R_error = (
        T_expected[:3, :3]
        @ T_actual[:3, :3].T
    )

    r_error = rotation_angle(
        R_error
    )

    return p_error, r_error


def assert_pose_close(
    name,
    actual,
    expected,
    pos_tol=1e-10,
    rot_tol=1e-7,
):
    p_error, r_error = pose_error(
        actual,
        expected,
    )

    print(f"\n{name}")
    print("position error =", p_error)
    print(
        "rotation error [deg] =",
        math.degrees(r_error),
    )

    if p_error > pos_tol:
        raise RuntimeError(
            f"{name}: position mismatch"
        )

    if r_error > rot_tol:
        raise RuntimeError(
            f"{name}: rotation mismatch"
        )


def main():
    print(
        "===== AVP WRIST RELATIVE MAPPER TEST ====="
    )

    # --------------------------------------------------------
    # Non-trivial initial poses.
    #
    # This proves the mapper is relative, rather than relying
    # on either world frame being identity.
    # --------------------------------------------------------
    T_avp0 = make_transform(
        np.array([
            1.20,
            -0.40,
            0.85,
        ]),
        rot_y(
            math.radians(25.0)
        ),
    )

    T_tcp0 = make_transform(
        np.array([
            0.069,
            0.032,
            0.825,
        ]),
        rot_z(
            math.radians(160.0)
        )
        @ rot_y(
            math.radians(-12.0)
        ),
    )

    mapper = AVPWristRelativeMapper(
        translation_scale=1.0,
    )

    mapper.calibrate(
        T_avp0,
        T_tcp0,
    )

    # --------------------------------------------------------
    # Test 1:
    # Calibration pose maps exactly to initial TCP.
    # --------------------------------------------------------
    actual = mapper.map(
        T_avp0
    )

    assert_pose_close(
        "CALIBRATION_IDENTITY",
        actual,
        T_tcp0,
    )

    # --------------------------------------------------------
    # Test 2:
    # +5 cm along initial AVP-wrist local X.
    #
    # With identity axis alignment, this must become +5 cm
    # along initial TCP local X.
    # --------------------------------------------------------
    delta = make_transform(
        np.array([
            0.05,
            0.0,
            0.0,
        ]),
        np.eye(3),
    )

    T_avp = (
        T_avp0
        @ delta
    )

    expected = (
        T_tcp0
        @ delta
    )

    actual = mapper.map(
        T_avp
    )

    assert_pose_close(
        "LOCAL_X_TRANSLATION",
        actual,
        expected,
    )

    # --------------------------------------------------------
    # Test 3:
    # Relative local wrist rotation.
    # --------------------------------------------------------
    delta = make_transform(
        np.zeros(3),
        rot_z(
            math.radians(12.0)
        ),
    )

    T_avp = (
        T_avp0
        @ delta
    )

    expected = (
        T_tcp0
        @ delta
    )

    actual = mapper.map(
        T_avp
    )

    assert_pose_close(
        "LOCAL_Z_ROTATION",
        actual,
        expected,
    )

    # --------------------------------------------------------
    # Test 4:
    # Translation scaling.
    # --------------------------------------------------------
    half_mapper = AVPWristRelativeMapper(
        translation_scale=0.5,
    )

    half_mapper.calibrate(
        T_avp0,
        T_tcp0,
    )

    delta_avp = make_transform(
        np.array([
            0.10,
            -0.04,
            0.02,
        ]),
        np.eye(3),
    )

    T_avp = (
        T_avp0
        @ delta_avp
    )

    delta_expected = make_transform(
        np.array([
            0.05,
            -0.02,
            0.01,
        ]),
        np.eye(3),
    )

    expected = (
        T_tcp0
        @ delta_expected
    )

    actual = half_mapper.map(
        T_avp
    )

    assert_pose_close(
        "TRANSLATION_SCALE",
        actual,
        expected,
    )

    # --------------------------------------------------------
    # Test 5:
    # Explicit AVP->TCP local-axis alignment.
    #
    # A = +90 deg around local Z:
    #
    # AVP +X -> TCP +Y
    # AVP +Y -> TCP -X
    # AVP +Z -> TCP +Z
    # --------------------------------------------------------
    A = rot_z(
        math.radians(90.0)
    )

    aligned_mapper = AVPWristRelativeMapper(
        translation_scale=1.0,
        R_tcp_from_avp_local=A,
    )

    aligned_mapper.calibrate(
        T_avp0,
        T_tcp0,
    )

    delta_avp = make_transform(
        np.array([
            0.03,
            0.0,
            0.0,
        ]),
        rot_x(
            math.radians(10.0)
        ),
    )

    T_avp = (
        T_avp0
        @ delta_avp
    )

    mapped_translation = (
        A
        @ delta_avp[:3, 3]
    )

    mapped_rotation = (
        A
        @ delta_avp[:3, :3]
        @ A.T
    )

    delta_expected = make_transform(
        mapped_translation,
        mapped_rotation,
    )

    expected = (
        T_tcp0
        @ delta_expected
    )

    actual = aligned_mapper.map(
        T_avp
    )

    assert_pose_close(
        "AXIS_ALIGNMENT",
        actual,
        expected,
    )

    print(
        "\nAVP WRIST RELATIVE MAPPER: PASS"
    )


if __name__ == "__main__":
    main()
