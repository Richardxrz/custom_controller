import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import calibrate  # noqa: E402


class TestCalibrate(unittest.TestCase):
    def test_analyze_endpoints(self):
        samples = list(range(500, 2501, 10))
        st = calibrate.analyze(samples)
        self.assertEqual(st["min"], 500)
        self.assertEqual(st["max"], 2500)
        self.assertGreaterEqual(st["suggest_prad_min"], 500)
        self.assertLessEqual(st["suggest_prad_max"], 2500)
        self.assertEqual(st["n"], len(samples))

    def test_outliers_rejected_by_percentile(self):
        samples = [1500] * 100 + [0, 4095]
        st = calibrate.analyze(samples)
        self.assertEqual(st["min"], 0)
        self.assertEqual(st["max"], 4095)
        self.assertGreater(st["suggest_prad_min"], 0)
        self.assertLess(st["suggest_prad_max"], 4095)

    def test_parse_csv(self):
        with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as f:
            f.write("t,prad\n0,500\n1,1500\n2,2500\n")
            path = f.name
        try:
            self.assertEqual(calibrate.parse_csv(path), [500, 1500, 2500])
        finally:
            os.unlink(path)

    def test_empty(self):
        with self.assertRaises(ValueError):
            calibrate.analyze([])


if __name__ == "__main__":
    unittest.main(verbosity=2)
