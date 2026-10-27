import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from joint_map import JointMap  # noqa: E402


class TestJointMap(unittest.TestCase):
    def test_endpoints(self):
        m = JointMap()
        self.assertAlmostEqual(m.to_joint(500), -1.5)
        self.assertAlmostEqual(m.to_joint(1500), 0.0)
        self.assertAlmostEqual(m.to_joint(2500), 1.5)

    def test_360_mapping(self):
        import math

        m = JointMap(joint_min=-math.pi, joint_max=math.pi)
        self.assertAlmostEqual(m.to_joint(500), -math.pi)
        self.assertAlmostEqual(m.to_joint(1500), 0.0)
        self.assertAlmostEqual(m.to_joint(2500), math.pi)

    def test_direction(self):
        m = JointMap(direction=-1)
        self.assertAlmostEqual(m.to_joint(500), 1.5)
        self.assertAlmostEqual(m.to_joint(2500), -1.5)

    def test_clamp(self):
        m = JointMap()
        self.assertAlmostEqual(m.to_joint(0), -1.5)
        self.assertAlmostEqual(m.to_joint(9999), 1.5)

    def test_clutch_holds_then_relative(self):
        m = JointMap()
        m.engage(1500)
        self.assertAlmostEqual(m.to_joint(2500), 0.0)
        m.release(2500)
        self.assertAlmostEqual(m.to_joint(2500), 0.0)
        # 从 2500 往回 500 计数 = -0.75 rad
        self.assertAlmostEqual(m.to_joint(2000), -0.75)

    def test_wrap_continuous(self):
        m = JointMap(prad_min=0, prad_max=4095, joint_min=-100, joint_max=100, wrap=True)
        # 4090 -> 4095 -> 0 -> 5 共 +11 计数，应连续，不因回绕跳到另一头
        vals = [m.absolute(raw) for raw in (4090, 4095, 0, 5)]
        self.assertAlmostEqual(vals[-1] - vals[0], 11 / 4095 * 200, places=6)

    def test_invalid(self):
        with self.assertRaises(ValueError):
            JointMap(prad_min=1, prad_max=1)
        with self.assertRaises(ValueError):
            JointMap(direction=0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
