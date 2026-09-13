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


def fov_barrier_scale(ratio, gain=0.2, maximum=4.0):
    limited = max(0.0, min(ratio, 0.999))
    scale = 1.0 + gain * limited * limited / max(1.0 - limited * limited, 1e-3)
    return max(1.0, min(scale, maximum))


class FixedCameraMathTest(unittest.TestCase):
    def test_optical_center_maps_to_body_forward(self):
        self.assertEqual(camera_to_body((0.0, 0.0, 1.0)), (1.0, 0.0, 0.0))

    def test_pixel_right_requires_negative_body_yaw(self):
        ray_body = camera_to_body((0.25, 0.0, 1.0))
        self.assertLess(math.atan2(ray_body[1], ray_body[0]), 0.0)

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

    def test_fov_barrier_is_monotonic_and_bounded(self):
        values = [fov_barrier_scale(ratio) for ratio in (0.0, 0.5, 0.8, 0.95, 1.2)]
        self.assertEqual(values, sorted(values))
        self.assertEqual(values[0], 1.0)
        self.assertLessEqual(values[-1], 4.0)


if __name__ == "__main__":
    unittest.main()
