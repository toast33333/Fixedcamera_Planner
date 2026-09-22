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


def alpha_beta_update(position, velocity, measurement, dt, alpha, beta):
    prediction = tuple(position[index] + velocity[index] * dt for index in range(3))
    residual = tuple(measurement[index] - prediction[index] for index in range(3))
    updated_position = tuple(
        prediction[index] + alpha * residual[index] for index in range(3)
    )
    updated_velocity = tuple(
        velocity[index] + beta * residual[index] / dt for index in range(3)
    )
    return updated_position, updated_velocity


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

    def test_three_second_filter_extrapolates_constant_velocity(self):
        position = (0.0, 0.0, 0.0)
        velocity = (5.0, 1.0, 0.0)
        measurement = (15.0, 3.0, 0.0)
        position, velocity = alpha_beta_update(
            position, velocity, measurement, 3.0, 0.70, 0.20
        )
        predicted_two_seconds_later = tuple(
            position[index] + velocity[index] * 2.0 for index in range(3)
        )

        self.assertEqual(position, measurement)
        self.assertEqual(velocity, (5.0, 1.0, 0.0))
        self.assertEqual(predicted_two_seconds_later, (25.0, 5.0, 0.0))

    def test_handover_forward_speed_adds_closing_margin_and_limits(self):
        target_speed = math.hypot(6.0, 8.0)
        requested = target_speed + 2.0
        self.assertEqual(max(3.0, min(requested, 12.0)), 12.0)
        self.assertEqual(max(3.0, min(1.0 + 2.0, 12.0)), 3.0)


if __name__ == "__main__":
    unittest.main()
