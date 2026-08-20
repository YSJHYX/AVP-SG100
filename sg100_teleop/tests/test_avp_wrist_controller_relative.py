"""Controller-relative AVP wrist -> TCP pose mapping regressions."""

import math

import numpy as np

from sg100_teleop.sim.run_avp_arm_hand_sim import (
    build_arg_parser,
    select_full_pose_target_rotation,
)
from sg100_teleop.tracking.avp_wrist_mapper import (
    AVPWristRelativeMapper,
    make_transform,
    rotation_angle,
)


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=np.float64)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array(
        [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]],
        dtype=np.float64,
    )
    return (
        np.eye(3)
        + math.sin(angle) * skew
        + (1.0 - math.cos(angle)) * (skew @ skew)
    )


A = axis_rotation([0.3, -0.2, 0.9], math.radians(37.0))
R_TCP_0 = axis_rotation([0.2, 0.7, -0.1], math.radians(29.0))
P_AVP_0 = np.array([1.1, -0.4, 0.8])
P_TCP_0 = np.array([0.3, 0.15, 0.62])
SCALE = 0.6


def mapper_for(R_avp_anchor, T_tcp_anchor=None):
    mapper = AVPWristRelativeMapper(
        translation_scale=SCALE,
        R_tcp_from_avp_local=A,
    )
    T_avp_anchor = make_transform(P_AVP_0, R_avp_anchor)
    if T_tcp_anchor is None:
        T_tcp_anchor = make_transform(P_TCP_0, R_TCP_0)
    mapper.calibrate(T_avp_anchor, T_tcp_anchor)
    return mapper, T_avp_anchor, T_tcp_anchor


def test_a_translation_independent_of_calibration_rotation():
    R_anchor_a = axis_rotation([1.0, 0.0, 0.0], math.radians(20.0))
    R_anchor_b = axis_rotation([0.0, 1.0, 0.0], math.radians(-55.0))
    mapper_a, T_a, T_tcp = mapper_for(R_anchor_a)
    mapper_b, T_b, _ = mapper_for(R_anchor_b)
    delta_p = np.array([0.04, -0.03, 0.02])

    current_a = make_transform(T_a[:3, 3] + delta_p, R_anchor_a)
    current_b = make_transform(T_b[:3, 3] + delta_p, R_anchor_b)
    delta_tcp_a = mapper_a.map(current_a)[:3, 3] - T_tcp[:3, 3]
    delta_tcp_b = mapper_b.map(current_b)[:3, 3] - T_tcp[:3, 3]

    expected = SCALE * (A @ delta_p)
    np.testing.assert_allclose(delta_tcp_a, expected, atol=1e-12)
    np.testing.assert_allclose(delta_tcp_b, expected, atol=1e-12)


def test_b_rotation_zero_at_calibration():
    R_anchor = axis_rotation([1.0, 1.0, 0.0], math.radians(43.0))
    mapper, T_avp_anchor, T_tcp_anchor = mapper_for(R_anchor)
    target = mapper.map(T_avp_anchor)
    np.testing.assert_allclose(target, T_tcp_anchor, atol=1e-12)


def test_c_pure_translation():
    R_anchor = axis_rotation([0.0, 0.0, 1.0], math.radians(31.0))
    mapper, T_avp_anchor, T_tcp_anchor = mapper_for(R_anchor)
    delta_p = np.array([-0.02, 0.05, 0.01])
    current = make_transform(T_avp_anchor[:3, 3] + delta_p, R_anchor)
    target = mapper.map(current)

    np.testing.assert_allclose(
        target[:3, 3],
        T_tcp_anchor[:3, 3] + SCALE * (A @ delta_p),
        atol=1e-12,
    )
    np.testing.assert_allclose(
        target[:3, :3], T_tcp_anchor[:3, :3], atol=1e-12
    )


def test_d_pure_rotation():
    R_anchor = axis_rotation([0.2, 0.4, 0.8], math.radians(-24.0))
    mapper, T_avp_anchor, T_tcp_anchor = mapper_for(R_anchor)
    R_rel_avp_world = axis_rotation([0.4, -0.7, 0.2], math.radians(18.0))
    current = make_transform(
        T_avp_anchor[:3, 3],
        R_rel_avp_world @ R_anchor,
    )
    target = mapper.map(current)
    expected_rotation = (
        A @ R_rel_avp_world @ A.T @ T_tcp_anchor[:3, :3]
    )

    np.testing.assert_allclose(
        target[:3, 3], T_tcp_anchor[:3, 3], atol=1e-12
    )
    np.testing.assert_allclose(target[:3, :3], expected_rotation, atol=1e-12)
    assert rotation_angle(target[:3, :3] @ T_tcp_anchor[:3, :3].T) > 0.1


def test_e_recalibration_is_zero_delta():
    R_anchor = axis_rotation([1.0, 0.0, 1.0], math.radians(16.0))
    mapper, T_avp_anchor, _ = mapper_for(R_anchor)
    current_wrist = make_transform(
        T_avp_anchor[:3, 3] + np.array([0.03, 0.02, -0.01]),
        axis_rotation([0.0, 1.0, 0.0], math.radians(14.0)) @ R_anchor,
    )
    current_tcp = mapper.map(current_wrist)

    mapper.calibrate(current_wrist, current_tcp)
    recalibrated_target = mapper.map(current_wrist)
    np.testing.assert_allclose(recalibrated_target, current_tcp, atol=1e-12)


def test_f_default_path_does_not_lock_rotation():
    parser_args = build_arg_parser().parse_args(["--ip", "192.0.2.1"])
    assert not parser_args.lock_tcp_rotation
    assert not parser_args.position_only_ik
    assert not parser_args.forearm_axis_ik

    R_anchor = axis_rotation([1.0, 0.0, 0.0], math.radians(12.0))
    mapper, T_avp_anchor, T_tcp_anchor = mapper_for(R_anchor)
    current = make_transform(
        T_avp_anchor[:3, 3],
        axis_rotation([0.0, 0.0, 1.0], math.radians(20.0)) @ R_anchor,
    )
    target_anchor = mapper.map(T_avp_anchor)
    target_rotated = mapper.map(current)

    normal_anchor = select_full_pose_target_rotation(
        target_anchor,
        lock_tcp_rotation=False,
        R_tcp_rotation_lock=T_tcp_anchor[:3, :3],
    )
    normal_rotated = select_full_pose_target_rotation(
        target_rotated,
        lock_tcp_rotation=False,
        R_tcp_rotation_lock=T_tcp_anchor[:3, :3],
    )
    assert rotation_angle(normal_rotated @ normal_anchor.T) > 0.1

    locked = select_full_pose_target_rotation(
        target_rotated,
        lock_tcp_rotation=True,
        R_tcp_rotation_lock=T_tcp_anchor[:3, :3],
    )
    np.testing.assert_allclose(locked, T_tcp_anchor[:3, :3], atol=1e-12)


def main():
    tests = (
        test_a_translation_independent_of_calibration_rotation,
        test_b_rotation_zero_at_calibration,
        test_c_pure_translation,
        test_d_pure_rotation,
        test_e_recalibration_is_zero_delta,
        test_f_default_path_does_not_lock_rotation,
    )
    for test in tests:
        test()
        print(f"{test.__name__}: PASS")


if __name__ == "__main__":
    main()
