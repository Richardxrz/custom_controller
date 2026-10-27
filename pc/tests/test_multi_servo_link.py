import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import multi_servo_link  # noqa: E402
import payload  # noqa: E402
from terminal_app import parse_ports  # noqa: E402


class FakeLink:
    def __init__(self, ids, values, delay=0.001):
        self.ids, self.values, self.delay = ids, values, delay
        self.closed = False

    def read_angle(self, sid):
        time.sleep(self.delay)
        return self.values.get(sid)

    def close(self):
        self.closed = True


def wait(ml, want=None, timeout=2.0):
    end = time.time() + timeout
    while time.time() < end:
        frames = ml.poll()
        if frames:
            up = payload.decode_uplink(frames[0].data)
            if want is None or up.joints == want:
                return up
        time.sleep(0.002)
    raise AssertionError("没有等到合并帧")


class TestParsePorts(unittest.TestCase):
    def test_two_buses(self):
        self.assertEqual(parse_ports(["A:1,2,3", "B:4,5,6"]),
                         [("A", [1, 2, 3]), ("B", [4, 5, 6])])

    def test_bad_spec(self):
        with self.assertRaises(ValueError):
            parse_ports(["A"])


class TestMultiServoLink(unittest.TestCase):
    def test_merge_two_buses(self):
        a = FakeLink([1, 2, 3], {1: 600, 2: 1500, 3: 2400})
        b = FakeLink([4, 5, 6], {4: 700, 5: 1600, 6: 2300})
        ml = multi_servo_link.MultiServoLink(links=[a, b], publish_hz=200)
        ml.start()
        try:
            up = wait(ml, [600, 1500, 2400, 700, 1600, 2300])
            self.assertEqual(up.flags & 0x01, 0x01)
        finally:
            ml.close()
        self.assertTrue(a.closed and b.closed)

    def test_stale_clears_valid_bit(self):
        a = FakeLink([1, 2, 3], {1: 600, 2: 1500, 3: 2400})
        b = FakeLink([4, 5, 6], {})
        ml = multi_servo_link.MultiServoLink(links=[a, b], publish_hz=200, stale_s=0.05)
        ml.start()
        try:
            wait(ml)
            time.sleep(0.15)
            self.assertEqual(wait(ml).flags & 0x01, 0x00)
        finally:
            ml.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
