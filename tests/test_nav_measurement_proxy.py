import importlib.util
import pathlib
import sys
import unittest

import numpy as np


SCRIPTS_DIR = (
    pathlib.Path(__file__).resolve().parents[1] /
    "src" / "png_planner" / "scripts"
)
sys.path.insert(0, str(SCRIPTS_DIR))
MODULE_PATH = SCRIPTS_DIR / "nav_measurement_proxy.py"
SPEC = importlib.util.spec_from_file_location("nav_measurement_proxy", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ParameterAndQuaternionTest(unittest.TestCase):
    def test_vector_parameter_expands_scalar_and_rejects_wrong_shape(self):
        np.testing.assert_array_equal(
            MODULE.vector3_parameter(0.2, "test"), [0.2, 0.2, 0.2])
        np.testing.assert_array_equal(
            MODULE.vector3_parameter([0.1, 0.2, 0.3], "test"),
            [0.1, 0.2, 0.3])
        with self.assertRaises(ValueError):
            MODULE.vector3_parameter([0.1, 0.2], "test")

    def test_rotation_error_preserves_unit_quaternion(self):
        result = MODULE.apply_rotation_vector(
            [0.0, 0.0, 0.0, 1.0], [0.02, -0.03, 0.04])
        self.assertAlmostEqual(float(np.linalg.norm(result)), 1.0, places=12)
        self.assertGreater(float(np.linalg.norm(result[:3])), 0.0)


class VectorErrorModelTest(unittest.TestCase):
    def test_seed_is_reproducible_and_random_walk_is_bounded(self):
        left = MODULE.VectorErrorModel(0.0, 2.0, 0.3, 71)
        right = MODULE.VectorErrorModel(0.0, 2.0, 0.3, 71)
        samples = []
        for index in range(500):
            stamp = index * 0.05
            left_value = left.sample_error(stamp)
            right_value = right.sample_error(stamp)
            np.testing.assert_array_equal(left_value, right_value)
            samples.append(left_value)
        samples = np.asarray(samples)
        self.assertGreater(float(np.std(samples)), 0.03)
        self.assertLessEqual(float(np.max(np.abs(samples))), 0.3 + 1.0e-12)


class StreamTransportModelTest(unittest.TestCase):
    def test_delay_is_nonnegative_reproducible_and_does_not_reorder(self):
        left = MODULE.StreamTransportModel(0.08, 0.20, 0.0, 93)
        right = MODULE.StreamTransportModel(0.08, 0.20, 0.0, 93)
        left_due = [left.due_time(10.0 + 0.01 * i) for i in range(200)]
        right_due = [right.due_time(10.0 + 0.01 * i) for i in range(200)]
        np.testing.assert_array_equal(left_due, right_due)
        self.assertTrue(all(value >= 10.0 for value in left_due))
        self.assertTrue(all(
            newer > older for older, newer in zip(left_due, left_due[1:])))

    def test_dropout_probability_is_safely_clamped(self):
        always = MODULE.StreamTransportModel(0.0, 0.0, 2.0, 1)
        never = MODULE.StreamTransportModel(0.0, 0.0, -1.0, 1)
        self.assertTrue(all(always.should_drop() for _ in range(50)))
        self.assertFalse(any(never.should_drop() for _ in range(50)))


if __name__ == "__main__":
    unittest.main()
