import os
import socket
import struct
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from video_link import VideoReceiver  # noqa: E402


class TestVideo(unittest.TestCase):
    def test_assemble_frame(self):
        rx = VideoReceiver(port=0)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            total = 100
            parts = [40, 40, 20]
            got = False
            for i, n in enumerate(parts):
                pkt = struct.pack("<HHI", 7, i, total) + bytes([i]) * n
                sender.sendto(pkt, ("127.0.0.1", rx.port))
                t0 = time.time()
                while time.time() - t0 < 0.5 and not got:
                    got = rx.poll() or got
            self.assertTrue(got)
            self.assertEqual(rx.frames, 1)
            self.assertEqual(rx.last_frame_size, total)
            self.assertEqual(rx.packets, 3)
        finally:
            sender.close()
            rx.close()

    def test_ignores_short_packet(self):
        rx = VideoReceiver(port=0)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sender.sendto(b"\x00\x01", ("127.0.0.1", rx.port))
            time.sleep(0.05)
            self.assertFalse(rx.poll())
        finally:
            sender.close()
            rx.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
