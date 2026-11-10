import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import client_app  # noqa: E402


class TestClientState(unittest.TestCase):
    def test_mapped_overrides_raw(self):
        st = client_app.State()
        st.active_channels = [0]
        st.channel_to_joint = [0] * 6
        st.on_mapped(b"\x01\x00" + b"\x00" * 12 +
                     b"".join(struct.pack("<h", v) for v in
                              [13500, -13500, 0, 100, 200, 300]))
        self.assertAlmostEqual(st.target[0], 135.0)
        self.assertAlmostEqual(st.target[1], -135.0)
        self.assertAlmostEqual(st.target[5], 3.0)

    def test_scan_report(self):
        st = client_app.State()
        data = bytes([1, 2, 1, 1, 2, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0])
        st.on_scan(data)
        self.assertEqual(st.scan_text, "A:[1, 2] B:[5]")

    def test_snapshot_shape(self):
        st = client_app.State()
        self.assertEqual(len(st.snapshot()), 10)


if __name__ == "__main__":
    unittest.main(verbosity=2)
