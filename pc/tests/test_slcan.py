import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import dm_can  # noqa: E402
import slcan  # noqa: E402


class FakeSer:
    def __init__(self):
        self.tx = bytearray()
        self.rx = bytearray()

    def write(self, data):
        self.tx += data
        return len(data)

    def read(self, n=1):
        d = bytes(self.rx[:n])
        del self.rx[:n]
        return d

    def close(self):
        pass


class TestSlcan(unittest.TestCase):
    def test_open_and_send(self):
        ser = FakeSer()
        bus = slcan.SlcanBus(ser=ser)
        bus.open(1_000_000)
        self.assertEqual(bytes(ser.tx), b"S8\rO\r")
        ser.tx.clear()
        bus.send(0x01, bytes.fromhex("7FFF7FF0000007FF"))
        self.assertEqual(bytes(ser.tx), b"t00187FFF7FF0000007FF\r")

    def test_recv(self):
        ser = FakeSer()
        ser.rx += b"t00187FFF7FF0000007FF\r"
        bus = slcan.SlcanBus(ser=ser)
        frames = bus.recv()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0][0], 1)
        self.assertEqual(len(frames[0][1]), 8)


class TestDm(unittest.TestCase):
    def test_zero_pack_golden(self):
        cfg = dm_can.MotorCfg(id=3, t_min=-10, t_max=10)
        self.assertEqual(dm_can.mit_pack(cfg, 0, 0, 0, 0, 0).hex(), "7fff7ff0000007ff")

    def test_feedback(self):
        cfg = dm_can.MotorCfg(id=3, t_min=-10, t_max=10)
        data = bytes([3, 0x21, 0x7F, 0xFF, 0x07, 0xFF, 0x07, 0xFF])
        fb = dm_can.feedback_decode(cfg, data)
        self.assertIsNotNone(fb)
        self.assertAlmostEqual(fb["pos"], 0.0, places=2)
        self.assertEqual(dm_can.feedback_decode(cfg, bytes([4] + list(data[1:]))), None)

    def test_special(self):
        self.assertEqual(dm_can.special("enable")[-1], 0xFC)
        self.assertEqual(dm_can.special("disable")[-1], 0xFD)


if __name__ == "__main__":
    unittest.main(verbosity=2)
