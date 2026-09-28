import importlib.util
import pathlib
import unittest

import numpy as np


MODULE_PATH = (
    pathlib.Path(__file__).resolve().parents[1] /
    "src" / "png_planner" / "scripts" / "gauss_markov_disturbance.py"
)
SPEC = importlib.util.spec_from_file_location("gauss_markov_disturbance", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
FirstOrderGaussMarkov = MODULE.FirstOrderGaussMarkov


class FirstOrderGaussMarkovTest(unittest.TestCase):
    def test_same_seed_is_sample_for_sample_reproducible(self):
        left = FirstOrderGaussMarkov([0.2, 0.1], 2.0, 17)
        right = FirstOrderGaussMarkov([0.2, 0.1], 2.0, 17)
        for _ in range(200):
            np.testing.assert_array_equal(left.step(0.05), right.step(0.05))

    def test_zero_standard_deviation_stays_disabled(self):
        process = FirstOrderGaussMarkov([0.0, 0.0, 0.0], 2.0, 1)
        self.assertFalse(process.enabled)
        np.testing.assert_array_equal(process.step(1.0), np.zeros(3))

    def test_seed_is_normalized_to_uint32(self):
        wrapped = FirstOrderGaussMarkov([1.0], 2.0, 2 ** 32 + 7)
        direct = FirstOrderGaussMarkov([1.0], 2.0, 7)
        self.assertEqual(wrapped.seed, 7)
        for _ in range(20):
            np.testing.assert_array_equal(wrapped.step(0.05), direct.step(0.05))

    def test_stationary_rms_and_safety_clip(self):
        process = FirstOrderGaussMarkov([0.30], 1.5, 23, clip_sigma=3.0)
        samples = [float(process.step(0.05)[0]) for _ in range(24000)]
        settled = np.asarray(samples[4000:])
        self.assertAlmostEqual(float(np.mean(settled)), 0.0, delta=0.04)
        self.assertAlmostEqual(float(np.std(settled)), 0.30, delta=0.05)
        self.assertLessEqual(float(np.max(np.abs(settled))), 0.90 + 1.0e-12)


if __name__ == "__main__":
    unittest.main()
