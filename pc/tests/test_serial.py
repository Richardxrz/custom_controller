import os
import pty
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import payload  # noqa: E402
import rmproto  # noqa: E402
import serial_link  # noqa: E402


class TestSerialLink(unittest.TestCase):
    def setUp(self):
        self.master, slave = pty.openpty()
        self.link = serial_link.SerialLink(os.ttyname(slave), baud=115200)

    def tearDown(self):
        self.link.close()
        os.close(self.master)

    def _read(self, timeout=1.0):
        frames = []
        t0 = time.time()
        while time.time() - t0 < timeout and not frames:
            frames += self.link.poll()
        return frames

    def test_read_one_frame(self):
        up = payload.Uplink(seq=5, joints=[1500] * 6)
        frame = rmproto.encode(0x0302, 5, payload.encode_uplink(up))
        os.write(self.master, frame)
        got = self._read()
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0].cmd_id, 0x0302)
        self.assertEqual(payload.decode_uplink(got[0].data).seq, 5)

    def test_garbage_then_frame(self):
        up = payload.Uplink(seq=6, joints=[1000, 1100, 1200, 1300, 1400, 1500])
        frame = rmproto.encode(0x0302, 6, payload.encode_uplink(up))
        os.write(self.master, b"garbage" + frame)
        got = self._read()
        self.assertEqual(len(got), 1)
        self.assertEqual(payload.decode_uplink(got[0].data).joints[3], 1300)

    def test_two_frames_in_one_read(self):
        f1 = rmproto.encode(0x0302, 1, payload.encode_uplink(payload.Uplink(seq=1)))
        f2 = rmproto.encode(0x0302, 2, payload.encode_uplink(payload.Uplink(seq=2)))
        os.write(self.master, f1 + f2)
        frames = []
        t0 = time.time()
        while time.time() - t0 < 1.0 and len(frames) < 2:
            frames += self.link.poll()
        self.assertGreaterEqual(len(frames), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
