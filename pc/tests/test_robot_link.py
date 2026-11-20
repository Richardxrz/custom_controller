import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rmproto  # noqa: E402
import robot_link  # noqa: E402
import serial_link  # noqa: E402


class TestRobotLink(unittest.TestCase):
    def test_control_and_telemetry(self):
        fd, path = serial_link.pty_pair()
        os.set_blocking(fd, False)
        got = []
        link = robot_link.SerialRobotLink(path, 115200, got.append)
        try:
            self.assertTrue(link.publish_control(bytes([1]) + b"\x00" * 29))
            time.sleep(0.1)
            raw = os.read(fd, 256)
            parser = rmproto.Parser()
            frame = None
            for b in raw:
                frame = parser.feed(b) or frame
            self.assertIsNotNone(frame)
            self.assertEqual(frame.cmd_id, 0x0311)
            self.assertEqual(len(frame.data), 30)

            telemetry = bytes([1, 0]) + b"\x00" * 298
            os.write(fd, rmproto.encode(0x0310, 0, telemetry))
            t0 = time.time()
            while not got and time.time() - t0 < 1:
                time.sleep(0.01)
            self.assertEqual(got, [telemetry])
        finally:
            link.close()
            os.close(fd)


if __name__ == "__main__":
    unittest.main(verbosity=2)
