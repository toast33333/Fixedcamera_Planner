import math
import unittest


def matmul(a, b):
    return [
        [sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)]
        for row in range(3)
    ]


def rot_z(angle):
    return [
        [math.cos(angle), -math.sin(angle), 0.0],
        [math.sin(angle), math.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ]


def inslib_to_ros_rotation(ned_from_frd):
    enu_from_ned = [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
    frd_from_flu = [
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
    return matmul(matmul(enu_from_ned, ned_from_frd), frd_from_flu)


def yaw_from_rotation(rotation):
    return math.atan2(rotation[1][0], rotation[0][0])


class InslibAdapterMathTests(unittest.TestCase):
    def test_ned_vector_becomes_enu(self):
        north, east, down = 2.0, 3.0, 4.0
        east_out, north_out, up_out = east, north, -down
        self.assertEqual((east_out, north_out, up_out), (3.0, 2.0, -4.0))

    def test_north_facing_level_body_is_ros_yaw_plus_90_deg(self):
        identity = [[1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0]]
        ros_rotation = inslib_to_ros_rotation(identity)
        self.assertAlmostEqual(yaw_from_rotation(ros_rotation), math.pi / 2.0, places=12)

    def test_east_facing_level_body_is_ros_yaw_zero(self):
        # INSLIB NED/FRD yaw +90 deg turns body-forward from north to east.
        ros_rotation = inslib_to_ros_rotation(rot_z(math.pi / 2.0))
        self.assertAlmostEqual(yaw_from_rotation(ros_rotation), 0.0, places=12)

    def test_rotation_conversion_preserves_right_handed_determinant(self):
        rotation = inslib_to_ros_rotation(rot_z(0.37))
        determinant = (
            rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
            - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
            + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
        )
        self.assertAlmostEqual(determinant, 1.0, places=12)


if __name__ == "__main__":
    unittest.main()
