import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import measure_mapping as mm  # noqa: E402


class TestMeasure(unittest.TestCase):
    def test_fit_linear(self):
        rows = [(p, 0.0015 * p - 2.25) for p in range(500, 2501, 50)]
        fit = mm.fit_gain(rows)
        self.assertAlmostEqual(fit["gain"], 0.0015, places=6)
        self.assertAlmostEqual(fit["offset"], -2.25, places=6)
        self.assertLess(fit["rms"], 1e-6)

    def test_analyze_wraps(self):
        rows = [(2500, 1.5), (500, -1.5), (1500, 0.0)]
        st = mm.analyze(rows)
        self.assertEqual(st["wraps"], 1)
        self.assertEqual(st["prad_min"], 500)
        self.assertEqual(st["prad_max"], 2500)

    def test_empty(self):
        self.assertEqual(mm.analyze([]), {"n": 0})


if __name__ == "__main__":
    unittest.main(verbosity=2)
