import os
import re
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import payload  # noqa: E402
import servo_link  # noqa: E402


class FakeSer:
    """假串口：收到 #IDPRAD! 就排队一条应答，read 逐字节返回。"""

    def __init__(self, values):
        self.values = values
        self.buf = b""
        self.cmd = re.compile(rb"#(\d{3})PRAD!")

    def reset_input_buffer(self):
        self.buf = b""

    def write(self, data):
        m = self.cmd.search(data)
        if m:
            sid = int(m.group(1))
            if sid in self.values:
                self.buf += f"#{sid:03d}P{self.values[sid]:04d}!".encode()

    def read(self, n=1):
        if not self.buf:
            return b""
        b = self.buf[:1]
        self.buf = self.buf[1:]
        return b

    def close(self):
        pass


class TestServoLink(unittest.TestCase):
    def test_poll_round(self):
        ser = FakeSer({1: 1500, 2: 1600, 3: 1700})
        link = servo_link.ServoLink(ids=[1, 2, 3], ser=ser)
        frames = []
        for _ in range(3):
            frames += link.poll()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].cmd_id, 0x0302)
        up = payload.decode_uplink(frames[0].data)
        self.assertEqual(up.joints[:3], [1500, 1600, 1700])
        self.assertEqual(link.ok, 3)
        self.assertEqual(link.timeouts, 0)

    def test_missing_servo(self):
        ser = FakeSer({1: 1500})
        link = servo_link.ServoLink(ids=[1, 2], ser=ser)
        frames = []
        for _ in range(2):
            frames += link.poll()
        self.assertEqual(len(frames), 1)
        self.assertEqual(link.timeouts, 1)
        up = payload.decode_uplink(frames[0].data)
        self.assertEqual(up.joints[0], 1500)
        self.assertEqual(up.joints[1], 1500)


if __name__ == "__main__":
    unittest.main(verbosity=2)
