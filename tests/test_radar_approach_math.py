import math
import unittest


def terminal_camera_position(target, target_course, distance, elevation_deg):
    elevation = math.radians(elevation_deg)
    direction = (
        math.cos(elevation) * math.cos(target_course),
        math.cos(elevation) * math.sin(target_course),
        math.sin(elevation),
    )
    return tuple(target[index] - distance * direction[index] for index in range(3))


def cv_kalman_step(state, covariance, measurement, dt,
                   measurement_std=5.773502692, accel_std=0.20):
    position, velocity = state
    p00, p01, p10, p11 = covariance
    position += velocity * dt
    accel_variance = accel_std ** 2
    predicted_covariance = (
        p00 + dt * (p01 + p10) + dt ** 2 * p11
        + 0.25 * accel_variance * dt ** 4,
        p01 + dt * p11 + 0.5 * accel_variance * dt ** 3,
        p10 + dt * p11 + 0.5 * accel_variance * dt ** 3,
        p11 + accel_variance * dt ** 2,
    )
    p00, p01, p10, p11 = predicted_covariance
    innovation_variance = p00 + measurement_std ** 2
    residual = measurement - position
    k0 = p00 / innovation_variance
    k1 = p10 / innovation_variance
    position += k0 * residual
    velocity += k1 * residual
    updated_covariance = (
        (1.0 - k0) * p00,
        (1.0 - k0) * p01,
        p10 - k1 * p00,
        p11 - k1 * p01,
    )
    return (position, velocity), updated_covariance


def horizontal_nis(residual_x, residual_y, p00_x, p00_y,
                   measurement_std=5.773502692):
    variance = measurement_std ** 2
    return (residual_x ** 2 / (p00_x + variance)
            + residual_y ** 2 / (p00_y + variance))


def bootstrap_linear_fit(samples):
    """Fit constant-velocity motion and return state at the latest sample."""
    mean_time = sum(sample[0] for sample in samples) / len(samples)
    mean_position = tuple(
        sum(sample[1][axis] for sample in samples) / len(samples)
        for axis in range(3)
    )
    denominator = sum((sample[0] - mean_time) ** 2 for sample in samples)
    velocity = tuple(
        sum(
            (sample[0] - mean_time) * sample[1][axis]
            for sample in samples
        ) / denominator
        for axis in range(3)
    )
    latest_offset = samples[-1][0] - mean_time
    position = tuple(
        mean_position[axis] + velocity[axis] * latest_offset
        for axis in range(3)
    )
    return position, velocity


def limit_target_velocity(velocity, total_limit,
                          horizontal_limit=-1.0, vertical_limit=-1.0):
    x, y, z = velocity
    horizontal_speed = math.hypot(x, y)
    if horizontal_limit > 0.0 and horizontal_speed > horizontal_limit:
        scale = horizontal_limit / horizontal_speed
        x *= scale
        y *= scale
    if vertical_limit > 0.0:
        z = max(-vertical_limit, min(z, vertical_limit))
    speed = math.sqrt(x * x + y * y + z * z)
    if speed > total_limit:
        scale = total_limit / speed
        x *= scale
        y *= scale
        z *= scale
    return x, y, z


def approach_axis_command(estimated_velocity, feedforward_gain,
                          position_gain, position_error):
    return feedforward_gain * estimated_velocity + position_gain * position_error


def apply_minimum_altitude(altitude, enabled, minimum_altitude):
    return max(altitude, minimum_altitude) if enabled else altitude


def braking_speed_limit(position_error, deceleration, buffer_distance,
                        maximum_speed):
    braking_distance = max(position_error - buffer_distance, 0.0)
    return min(maximum_speed,
               math.sqrt(2.0 * deceleration * braking_distance))


def handover_relative_speed_stable(closing_speed, maximum_closing_speed,
                                   maximum_opening_speed):
    return (-maximum_opening_speed <= closing_speed
            <= maximum_closing_speed)


def handover_region_half_extents(image_width, image_height, area_ratio):
    linear_fraction = math.sqrt(area_ratio)
    return (0.5 * image_width * linear_fraction,
            0.5 * image_height * linear_fraction)


def visual_stabilization_blend(target_range, tilt_deg,
                               stabilization_start_range=35.0,
                               handover_max_range=30.0,
                               attitude_start_deg=8.0,
                               attitude_full_deg=15.0):
    if target_range > stabilization_start_range:
        return 0.0
    range_blend = max(0.0, min(
        (stabilization_start_range - target_range) /
        (stabilization_start_range - handover_max_range),
        1.0,
    ))
    attitude_blend = max(0.0, min(
        (tilt_deg - attitude_start_deg) /
        (attitude_full_deg - attitude_start_deg),
        1.0,
    ))
    return max(range_blend, attitude_blend)


class RadarApproachMathTest(unittest.TestCase):
    def test_twenty_meter_terminal_geometry_centers_upward_camera(self):
        target = (100.0, 20.0, 30.0)
        camera = terminal_camera_position(target, 0.0, 20.0, 20.0)
        los = tuple(target[index] - camera[index] for index in range(3))
        slant_range = math.sqrt(sum(value * value for value in los))
        elevation = math.degrees(math.atan2(los[2], math.hypot(los[0], los[1])))

        self.assertAlmostEqual(slant_range, 20.0, places=10)
        self.assertAlmostEqual(elevation, 20.0, places=10)
        self.assertAlmostEqual(target[0] - camera[0], 18.793852, places=5)
        self.assertAlmostEqual(target[2] - camera[2], 6.840403, places=5)

    def test_three_second_covariance_filter_tracks_constant_velocity(self):
        state = (0.0, 5.0)
        covariance = (4.0, 0.0, 0.0, 1.0)
        state, covariance = cv_kalman_step(
            state, covariance, measurement=16.0, dt=3.0)

        self.assertGreater(state[0], 15.0)
        self.assertLess(state[0], 16.0)
        self.assertGreater(state[1], 5.0)
        self.assertGreater(covariance[0], 0.0)
        self.assertGreater(covariance[3], 0.0)

    def test_two_dimensional_nis_rejects_large_outlier(self):
        ordinary = horizontal_nis(6.0, -4.0, 10.0, 10.0)
        outlier = horizontal_nis(40.0, 35.0, 10.0, 10.0)

        self.assertLess(ordinary, 9.21)
        self.assertGreater(outlier, 25.0)

    def test_two_sample_bootstrap_is_backward_compatible(self):
        samples = [
            (10.0, (3.0, -2.0, 5.0)),
            (13.0, (15.0, 1.0, -1.0)),
        ]
        position, velocity = bootstrap_linear_fit(samples)

        self.assertEqual(position, samples[-1][1])
        self.assertEqual(velocity, (4.0, 1.0, -2.0))

    def test_four_sample_bootstrap_recovers_irregular_constant_velocity(self):
        initial = (12.0, -7.0, 3.0)
        velocity_expected = (2.5, -1.0, 0.4)
        times = (0.0, 2.7, 6.2, 9.4)
        samples = [
            (time, tuple(
                initial[axis] + velocity_expected[axis] * time
                for axis in range(3)
            ))
            for time in times
        ]
        position, velocity = bootstrap_linear_fit(samples)

        for axis in range(3):
            self.assertAlmostEqual(velocity[axis], velocity_expected[axis])
            self.assertAlmostEqual(
                position[axis],
                initial[axis] + velocity_expected[axis] * times[-1])

    def test_four_sample_bootstrap_suppresses_bad_second_frame_vertical_speed(self):
        # First four white-noise samples from the failing 20261001 radar seed,
        # with a 2 m/s constant target motion added on x.
        samples = [
            (0.0, (-5.60, 4.74, 7.32)),
            (3.0, (6.39, 8.44, -6.26)),
            (6.0, (19.95, -9.17, 5.28)),
            (9.0, (12.75, -2.14, 3.75)),
        ]
        _, two_sample_velocity = bootstrap_linear_fit(samples[:2])
        _, four_sample_velocity = bootstrap_linear_fit(samples)

        self.assertGreater(abs(two_sample_velocity[2]), 4.5)
        self.assertLess(abs(four_sample_velocity[2]), 0.1)

    def test_optional_axis_limits_and_legacy_total_limit(self):
        velocity = (30.0, 0.0, 10.0)
        legacy = limit_target_velocity(velocity, 25.0)
        legacy_scale = 25.0 / math.sqrt(30.0 ** 2 + 10.0 ** 2)
        self.assertAlmostEqual(legacy[0], 30.0 * legacy_scale)
        self.assertAlmostEqual(legacy[2], 10.0 * legacy_scale)

        protected = limit_target_velocity(
            velocity, 25.0, horizontal_limit=7.0, vertical_limit=1.2)
        self.assertAlmostEqual(math.hypot(protected[0], protected[1]), 7.0)
        self.assertAlmostEqual(protected[2], 1.2)

    def test_vertical_feedforward_gain_and_optional_altitude_floor(self):
        command = approach_axis_command(
            estimated_velocity=-1.2,
            feedforward_gain=0.25,
            position_gain=0.35,
            position_error=2.0,
        )
        self.assertAlmostEqual(command, 0.4)
        self.assertEqual(apply_minimum_altitude(1.5, False, 3.0), 1.5)
        self.assertEqual(apply_minimum_altitude(1.5, True, 3.0), 3.0)

    def test_handover_forward_speed_adds_closing_margin_and_limits(self):
        target_speed = math.hypot(6.0, 8.0)
        requested = target_speed + 2.0
        self.assertEqual(max(3.0, min(requested, 12.0)), 12.0)
        self.assertEqual(max(3.0, min(1.0 + 2.0, 12.0)), 3.0)

    def test_braking_profile_uses_handover_pose_error_not_target_range(self):
        self.assertEqual(braking_speed_limit(2.5, 2.0, 2.5, 8.0), 0.0)
        self.assertAlmostEqual(
            braking_speed_limit(6.5, 2.0, 2.5, 8.0), 4.0)
        self.assertEqual(braking_speed_limit(40.0, 2.0, 2.5, 8.0), 8.0)

    def test_handover_rejects_fast_opening_and_fast_closing(self):
        self.assertFalse(handover_relative_speed_stable(-2.35, 2.0, 0.5))
        self.assertTrue(handover_relative_speed_stable(-0.50, 2.0, 0.5))
        self.assertTrue(handover_relative_speed_stable(0.75, 2.0, 0.5))
        self.assertFalse(handover_relative_speed_stable(2.05, 2.0, 0.5))

    def test_central_handover_region_occupies_forty_percent_of_image(self):
        half_width, half_height = handover_region_half_extents(
            960.0, 540.0, 0.40)
        region_area = (2.0 * half_width) * (2.0 * half_height)

        self.assertAlmostEqual(region_area / (960.0 * 540.0), 0.40)
        self.assertAlmostEqual(half_width, 303.578655, places=5)
        self.assertAlmostEqual(half_height, 170.762993, places=5)

    def test_visual_stabilization_uses_range_or_measured_tilt(self):
        self.assertEqual(visual_stabilization_blend(40.0, 20.0), 0.0)
        self.assertAlmostEqual(visual_stabilization_blend(32.5, 5.0), 0.5)
        self.assertAlmostEqual(visual_stabilization_blend(34.0, 11.5), 0.5)
        self.assertEqual(visual_stabilization_blend(30.0, 5.0), 1.0)
        self.assertEqual(visual_stabilization_blend(34.0, 15.0), 1.0)

    def test_twenty_five_meter_terminal_geometry_preserves_slant_range(self):
        target = (80.0, -12.0, 9.0)
        camera = terminal_camera_position(target, math.radians(45.0), 25.0, 20.0)
        self.assertAlmostEqual(
            math.sqrt(sum((target[i] - camera[i]) ** 2 for i in range(3))),
            25.0,
        )


if __name__ == "__main__":
    unittest.main()
