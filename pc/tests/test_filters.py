import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import filters  # noqa: E402


class TestFilters(unittest.TestCase):
    def test_hampel_removes_spike(self):
        h = filters.Hampel(window=5, min_threshold=3.0)
        for _ in range(5):
            self.assertEqual(h.update(1000), 1000)
        self.assertEqual(h.update(2000), 1000)

    def test_hampel_passthrough_until_full(self):
        h = filters.Hampel(window=5)
        self.assertEqual(h.update(1234), 1234)
        self.assertEqual(h.update(1235), 1235)

    def test_one_euro_constant(self):
        oe = filters.OneEuro()
        for _ in range(10):
            self.assertEqual(oe.update(1500, 0.02), 1500)

    def test_one_euro_step_converges(self):
        oe = filters.OneEuro(min_cutoff=5.0, beta=1.0)
        for _ in range(5):
            oe.update(1500, 0.02)
        out = 0.0
        for _ in range(15):
            out = oe.update(2000, 0.02)
        self.assertGreater(out, 1900)

    def test_chain_removes_spike(self):
        c = filters.FilterChain(min_cutoff=5.0, beta=1.0)
        for _ in range(5):
            c.update(1000, 0.02)
        out = c.update(2000, 0.02)
        self.assertLess(out, 1200)


if __name__ == "__main__":
    unittest.main(verbosity=2)
