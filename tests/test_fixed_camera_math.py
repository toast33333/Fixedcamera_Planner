import math
import unittest


def camera_to_body(ray):
    x_camera, y_camera, z_camera = ray
    body = (z_camera, -x_camera, -y_camera)
    length = math.sqrt(sum(value * value for value in body))
    return tuple(value / length for value in body)


def rotate_yaw(vector, yaw):
    x_value, y_value, z_value = vector
    return (
        math.cos(yaw) * x_value - math.sin(yaw) * y_value,
        math.sin(yaw) * x_value + math.cos(yaw) * y_value,
        z_value,
    )


def inverse_rotate_yaw(vector, yaw):
    return rotate_yaw(vector, -yaw)


def rotate_body_pitch(vector, pitch):
    x_value, y_value, z_value = vector
    return (
        math.cos(pitch) * x_value + math.sin(pitch) * z_value,
        y_value,
        -math.sin(pitch) * x_value + math.cos(pitch) * z_value,
    )


def fov_barrier_shape(pixel_error, view_half_extent, area_ratio=0.5,
                      maximum_shape=4.0):
    inactive_fraction = math.sqrt(area_ratio)
    normalized_position = abs(pixel_error) / view_half_extent
    if normalized_position <= inactive_fraction:
        return 0.0
    progress = max(0.0, min(
        (normalized_position - inactive_fraction) /
        (1.0 - inactive_fraction),
        0.999,
    ))
    shape = progress ** 2 / max(1.0 - progress ** 2, 1e-3)
    return min(shape, maximum_shape)


def angular_ratio(angle_deg, half_angle_limit_deg):
    return abs(angle_deg) / half_angle_limit_deg


def ramp_forward_speed(active, target, acceleration_limit, dt):
    target = max(target, 0.0)
    if acceleration_limit <= 0.0 or target <= active:
        return target
    return min(target, active + acceleration_limit * max(dt, 0.0))


class FixedCameraMathTest(unittest.TestCase):
    def test_optical_center_maps_to_body_forward(self):
        self.assertEqual(camera_to_body((0.0, 0.0, 1.0)), (1.0, 0.0, 0.0))

    def test_pixel_right_requires_negative_body_yaw(self):
        ray_body = camera_to_body((0.25, 0.0, 1.0))
        self.assertLess(math.atan2(ray_body[1], ray_body[0]), 0.0)

    def test_negative_twenty_degree_mount_pitch_raises_optical_axis(self):
        optical_center_body = camera_to_body((0.0, 0.0, 1.0))
        mounted_ray = rotate_body_pitch(optical_center_body, math.radians(-20.0))
        self.assertAlmostEqual(mounted_ray[0], math.cos(math.radians(20.0)), places=12)
        self.assertAlmostEqual(mounted_ray[1], 0.0, places=12)
        self.assertAlmostEqual(mounted_ray[2], math.sin(math.radians(20.0)), places=12)

    def test_attitude_compensation_keeps_inertial_los_fixed(self):
        target_world = (1.0, 0.0, 0.0)
        capture_yaw = math.radians(10.0)
        current_yaw = math.radians(25.0)

        captured_body_ray = inverse_rotate_yaw(target_world, capture_yaw)
        reconstructed_world_ray = rotate_yaw(captured_body_ray, capture_yaw)
        predicted_current_body_ray = inverse_rotate_yaw(
            reconstructed_world_ray, current_yaw
        )

        inertial_bearing = math.atan2(
            reconstructed_world_ray[1], reconstructed_world_ray[0]
        )
        current_body_bearing = math.atan2(
            predicted_current_body_ray[1], predicted_current_body_ray[0]
        )
        self.assertAlmostEqual(inertial_bearing, 0.0, places=12)
        self.assertAlmostEqual(current_body_bearing, -current_yaw, places=12)

    def test_fov_barrier_is_zero_in_central_half_area(self):
        half_view = 320.0
        inactive_half_width = half_view * math.sqrt(0.5)

        self.assertEqual(fov_barrier_shape(0.0, half_view), 0.0)
        self.assertEqual(
            fov_barrier_shape(inactive_half_width, half_view), 0.0)
        self.assertGreater(
            fov_barrier_shape(inactive_half_width + 1.0, half_view), 0.0)

    def test_additive_fov_barrier_is_monotonic_and_bounded(self):
        half_view = 320.0
        values = [fov_barrier_shape(half_view * ratio, half_view)
                  for ratio in (0.0, 0.70, 0.8, 0.95, 1.2)]
        self.assertEqual(values, sorted(values))
        self.assertEqual(values[0], 0.0)
        self.assertLessEqual(values[-1], 4.0)

    def test_central_guard_rectangle_has_half_the_view_area(self):
        half_width = 320.0
        half_height = 240.0
        fraction = math.sqrt(0.5)
        central_area = (2.0 * half_width * fraction) * (
            2.0 * half_height * fraction)
        full_area = (2.0 * half_width) * (2.0 * half_height)
        self.assertAlmostEqual(central_area / full_area, 0.5)

    def test_measured_half_angles_are_the_constraint_boundaries(self):
        self.assertAlmostEqual(angular_ratio(26.6, 26.6), 1.0)
        self.assertAlmostEqual(angular_ratio(-26.6, 26.6), 1.0)
        self.assertAlmostEqual(angular_ratio(20.6, 20.6), 1.0)
        self.assertAlmostEqual(angular_ratio(-20.6, 20.6), 1.0)

    def test_forward_speed_ramp_limits_positive_acceleration(self):
        speed = 0.0
        for _ in range(50):
            speed = ramp_forward_speed(speed, 8.0, 1.0, 0.02)
        self.assertAlmostEqual(speed, 1.0, places=12)

    def test_forward_speed_ramp_keeps_fov_deceleration_immediate(self):
        self.assertEqual(ramp_forward_speed(8.0, 3.5, 1.0, 0.02), 3.5)
        self.assertEqual(ramp_forward_speed(2.0, 8.0, 0.0, 0.02), 8.0)

if __name__ == "__main__":
    unittest.main()
