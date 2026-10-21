import math
import unittest


def reference_state(start_radius, radial_rate, nominal_speed, elapsed, clockwise=False):
    radius = max(0.0, start_radius - radial_rate * elapsed)
    tangential_speed = math.sqrt(
        max(0.0, nominal_speed * nominal_speed - radial_rate * radial_rate))
    direction = -1.0 if clockwise else 1.0
    if radius <= 0.0:
        return radius, 0.0, 0.0, 0.0, 0.0
    theta = direction * tangential_speed / radial_rate * math.log(start_radius / radius)
    cosine = math.cos(theta)
    sine = math.sin(theta)
    tangent_x = direction * -sine
    tangent_y = direction * cosine
    vx = -radial_rate * cosine + tangential_speed * tangent_x
    vy = -radial_rate * sine + tangential_speed * tangent_y
    return radius, theta, radius * cosine, radius * sine, math.hypot(vx, vy)


class CircleTrajectoryMathTest(unittest.TestCase):
    def test_reference_begins_at_outer_radius_not_at_center(self):
        radius, theta, x, y, speed = reference_state(20.0, 0.5, 5.0, 0.0)
        self.assertAlmostEqual(radius, 20.0)
        self.assertAlmostEqual(theta, 0.0)
        self.assertAlmostEqual(x, 20.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(speed, 5.0)

    def test_feedforward_preserves_each_nominal_speed(self):
        for nominal_speed in (5.0, 7.0, 9.0, 11.0, 13.0, 15.0):
            for elapsed in (0.0, 10.0, 25.0, 35.0):
                *_, speed = reference_state(20.0, 0.5, nominal_speed, elapsed)
                self.assertAlmostEqual(speed, nominal_speed, places=9)

    def test_clockwise_only_reverses_turn_direction(self):
        ccw = reference_state(20.0, 0.5, 9.0, 10.0, clockwise=False)
        cw = reference_state(20.0, 0.5, 9.0, 10.0, clockwise=True)
        self.assertAlmostEqual(ccw[0], cw[0])
        self.assertAlmostEqual(ccw[1], -cw[1])
        self.assertAlmostEqual(ccw[2], cw[2])
        self.assertAlmostEqual(ccw[3], -cw[3])


if __name__ == "__main__":
    unittest.main()
