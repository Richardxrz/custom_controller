import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import arm_sim  # noqa: E402


class TestFk(unittest.TestCase):
    def test_zero_pose(self):
        pts = arm_sim.fk([0.0] * 6)
        self.assertEqual(len(pts), 6)
        self.assertGreater(pts[5][0], 0.5)
        self.assertAlmostEqual(pts[0][2], 0.0)

    def test_base_yaw(self):
        pts = arm_sim.fk([math.pi / 2, 0, 0, 0, 0, 0])
        self.assertGreater(pts[5][1], 0.5)

    def test_shoulder_lift(self):
        level = arm_sim.fk([0.0] * 6)
        lifted = arm_sim.fk([0, math.pi / 2, 0, 0, 0, 0])
        self.assertGreater(lifted[2][2], level[2][2] + 0.2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
