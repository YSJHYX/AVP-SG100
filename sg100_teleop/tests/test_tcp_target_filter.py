import math

import numpy as np
from scipy.spatial.transform import Rotation

from sg100_teleop.control.tcp_target_filter import (
    TCP_FILTER_MAX_DT,
    TcpTargetFilter,
    TcpTargetFilterConfig,
)


def make_pose(position=(0.0, 0.0, 0.0), rotation=None):
    pose = np.eye(4, dtype=np.float64)
    pose[:3, 3] = np.asarray(position, dtype=np.float64)
    if rotation is not None:
        pose[:3, :3] = np.asarray(rotation, dtype=np.float64)
    return pose


def rotation_vector(rotation):
    return Rotation.from_matrix(rotation).as_rotvec()


def expect_value_error(call, expected_text):
    try:
        call()
    except ValueError as exc:
        assert expected_text in str(exc), str(exc)
    else:
        raise AssertionError("Expected ValueError")


def unconstrained_config(**overrides):
    values = {
        "position_tau": 0.06,
        "rotation_tau": 0.06,
        "max_linear_velocity": 1e6,
        "max_angular_velocity": 1e6,
    }
    values.update(overrides)
    return TcpTargetFilterConfig(**values)


def test_position_low_pass_uses_time_constant_alpha():
    target_filter = TcpTargetFilter(unconstrained_config())
    target_filter.reset(make_pose(), timestamp=0.0)
    dt = 1.0 / 60.0

    output = target_filter.update(make_pose((1.0, 0.0, 0.0)), timestamp=dt)

    alpha = 1.0 - math.exp(-dt / 0.06)
    np.testing.assert_allclose(output[:3, 3], [alpha, 0.0, 0.0], atol=1e-12)
    assert not math.isclose(alpha, 0.05)


def test_time_constant_response_is_independent_of_update_rate():
    outputs = []
    for rate_hz in (60, 120):
        target_filter = TcpTargetFilter(unconstrained_config())
        target_filter.reset(make_pose(), timestamp=0.0)
        for index in range(1, rate_hz + 1):
            target_filter.update(
                make_pose((1.0, 0.0, 0.0)),
                timestamp=index / rate_hz,
            )
        outputs.append(target_filter.value[:3, 3])

    np.testing.assert_allclose(outputs[0], outputs[1], atol=1e-12)


def test_linear_velocity_limit_uses_vector_norm_and_preserves_direction():
    target_filter = TcpTargetFilter(
        unconstrained_config(position_tau=1e-6, max_linear_velocity=0.25)
    )
    target_filter.reset(make_pose(), timestamp=0.0)
    direction = np.array([1.0, 2.0, 2.0], dtype=np.float64)

    output = target_filter.update(make_pose(direction), timestamp=0.02)
    step = output[:3, 3]

    np.testing.assert_allclose(np.linalg.norm(step), 0.005, atol=1e-12)
    np.testing.assert_allclose(
        step / np.linalg.norm(step),
        direction / np.linalg.norm(direction),
        atol=1e-12,
    )
    assert np.count_nonzero(np.isclose(np.abs(step), 0.005)) == 0


def test_rotation_low_pass_uses_shortest_so3_path():
    target_filter = TcpTargetFilter(unconstrained_config())
    target_filter.reset(make_pose(), timestamp=0.0)
    dt = 1.0 / 60.0
    raw_rotation = Rotation.from_euler("z", 90.0, degrees=True).as_matrix()

    output = target_filter.update(
        make_pose(rotation=raw_rotation),
        timestamp=dt,
    )

    alpha = 1.0 - math.exp(-dt / 0.06)
    rotvec = rotation_vector(output[:3, :3])
    np.testing.assert_allclose(rotvec, [0.0, 0.0, alpha * math.pi / 2.0], atol=1e-12)


def test_rotation_low_pass_takes_short_path_across_angle_wrap():
    target_filter = TcpTargetFilter(unconstrained_config())
    target_filter.reset(make_pose(), timestamp=0.0)
    raw_rotation = Rotation.from_euler("z", 350.0, degrees=True).as_matrix()

    output = target_filter.update(
        make_pose(rotation=raw_rotation),
        timestamp=1.0 / 60.0,
    )

    assert rotation_vector(output[:3, :3])[2] < 0.0


def test_angular_velocity_limit_uses_geodesic_angle():
    target_filter = TcpTargetFilter(
        unconstrained_config(rotation_tau=1e-6, max_angular_velocity=1.5)
    )
    target_filter.reset(make_pose(), timestamp=0.0)
    raw_rotation = Rotation.from_euler("z", 90.0, degrees=True).as_matrix()

    output = target_filter.update(
        make_pose(rotation=raw_rotation),
        timestamp=0.02,
    )
    rotvec = rotation_vector(output[:3, :3])

    np.testing.assert_allclose(np.linalg.norm(rotvec), 0.03, atol=1e-12)
    np.testing.assert_allclose(rotvec / np.linalg.norm(rotvec), [0.0, 0.0, 1.0], atol=1e-12)


def test_rotation_remains_valid_over_many_updates():
    target_filter = TcpTargetFilter(TcpTargetFilterConfig())
    target_filter.reset(make_pose(), timestamp=0.0)

    for index in range(1, 301):
        raw_rotation = Rotation.from_rotvec(
            np.array([0.3, -0.2, 0.4]) * math.sin(index / 19.0)
        ).as_matrix()
        output = target_filter.update(
            make_pose(rotation=raw_rotation),
            timestamp=index / 60.0,
        )
        rotation = output[:3, :3]
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(np.linalg.det(rotation), 1.0, atol=1e-12)


def test_first_update_initializes_directly_to_raw_pose():
    target_filter = TcpTargetFilter(TcpTargetFilterConfig())
    raw_pose = make_pose(
        (0.2, -0.1, 0.4),
        Rotation.from_euler("xyz", [20.0, -10.0, 35.0], degrees=True).as_matrix(),
    )

    output = target_filter.update(raw_pose, timestamp=10.0)

    np.testing.assert_allclose(output, raw_pose, atol=1e-12)


def test_calibration_reset_discards_position_and_rotation_history():
    target_filter = TcpTargetFilter(TcpTargetFilterConfig())
    target_filter.reset(make_pose(), timestamp=0.0)
    old_target = make_pose(
        (1.0, 0.0, 0.0),
        Rotation.from_euler("y", 150.0, degrees=True).as_matrix(),
    )
    target_filter.update(old_target, timestamp=1.0 / 60.0)

    actual_pose = make_pose(
        (-0.1, 0.2, 0.5),
        Rotation.from_euler("x", -70.0, degrees=True).as_matrix(),
    )
    reset_output = target_filter.reset(actual_pose, timestamp=2.0)
    next_output = target_filter.update(actual_pose, timestamp=2.0 + 1.0 / 60.0)

    np.testing.assert_allclose(reset_output, actual_pose, atol=1e-12)
    np.testing.assert_allclose(next_output, actual_pose, atol=1e-12)


def test_micro_motion_is_not_removed_by_a_deadband():
    for distance in (0.001, 0.003, 0.005):
        target_filter = TcpTargetFilter(TcpTargetFilterConfig())
        target_filter.reset(make_pose(), timestamp=0.0)
        raw_pose = make_pose((distance, 0.0, 0.0))

        first_output = target_filter.update(raw_pose, timestamp=1.0 / 60.0)
        for index in range(2, 61):
            final_output = target_filter.update(raw_pose, timestamp=index / 60.0)

        assert 0.0 < first_output[0, 3] < distance
        assert final_output[0, 3] > 0.99 * distance


def test_invalid_pose_does_not_pollute_filter_state():
    for invalid_value in (np.nan, np.inf, -np.inf):
        target_filter = TcpTargetFilter(TcpTargetFilterConfig())
        target_filter.reset(make_pose((0.1, 0.2, 0.3)), timestamp=0.0)
        before = target_filter.value
        invalid_pose = make_pose()
        invalid_pose[0, 3] = invalid_value

        expect_value_error(
            lambda: target_filter.update(invalid_pose, timestamp=0.01),
            "finite",
        )
        np.testing.assert_allclose(target_filter.value, before, atol=0.0)


def test_long_update_gap_holds_once_and_rebases_time():
    target_filter = TcpTargetFilter(TcpTargetFilterConfig())
    target_filter.reset(make_pose(), timestamp=0.0)
    raw_pose = make_pose((1.0, 0.0, 0.0))

    held = target_filter.update(raw_pose, timestamp=TCP_FILTER_MAX_DT + 1.0)
    resumed = target_filter.update(
        raw_pose,
        timestamp=TCP_FILTER_MAX_DT + 1.0 + 1.0 / 60.0,
    )

    np.testing.assert_allclose(held, make_pose(), atol=0.0)
    assert 0.0 < resumed[0, 3] <= 0.25 / 60.0 + 1e-12


def test_clear_discards_session_state_before_rearm():
    target_filter = TcpTargetFilter(TcpTargetFilterConfig())
    target_filter.reset(make_pose(), timestamp=0.0)
    target_filter.update(make_pose((1.0, 0.0, 0.0)), timestamp=0.01)

    target_filter.clear()
    new_anchor = make_pose((-0.2, 0.3, 0.4))
    output = target_filter.update(new_anchor, timestamp=5.0)

    np.testing.assert_allclose(output, new_anchor, atol=0.0)


def test_config_requires_positive_finite_values():
    for field in (
        "position_tau",
        "rotation_tau",
        "max_linear_velocity",
        "max_angular_velocity",
    ):
        values = {
            "position_tau": 0.06,
            "rotation_tau": 0.06,
            "max_linear_velocity": 0.25,
            "max_angular_velocity": 1.5,
        }
        values[field] = 0.0
        expect_value_error(lambda: TcpTargetFilterConfig(**values), field)


def main():
    tests = [
        test_position_low_pass_uses_time_constant_alpha,
        test_time_constant_response_is_independent_of_update_rate,
        test_linear_velocity_limit_uses_vector_norm_and_preserves_direction,
        test_rotation_low_pass_uses_shortest_so3_path,
        test_rotation_low_pass_takes_short_path_across_angle_wrap,
        test_angular_velocity_limit_uses_geodesic_angle,
        test_rotation_remains_valid_over_many_updates,
        test_first_update_initializes_directly_to_raw_pose,
        test_calibration_reset_discards_position_and_rotation_history,
        test_micro_motion_is_not_removed_by_a_deadband,
        test_invalid_pose_does_not_pollute_filter_state,
        test_long_update_gap_holds_once_and_rebases_time,
        test_clear_discards_session_state_before_rearm,
        test_config_requires_positive_finite_values,
    ]
    for test in tests:
        test()
    print("TCP TARGET FILTER: PASS")


if __name__ == "__main__":
    main()
