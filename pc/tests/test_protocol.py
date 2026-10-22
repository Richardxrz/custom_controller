import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import payload  # noqa: E402
import rmproto  # noqa: E402

GOLD = {
    "crc8_00": 0x35,
    "crc8_01": 0x6B,
    "crc8_a5": 0xC3,
    "crc16_a5": 0xD2F3,
    "uplink": "1109e8034c04b00414057805dc05d2040501000000000000000000002ee9",
    "frame0311": "a51e002a201103030a11181f262d343b424950575e656c737a81888f969da4abb2b9c0c7ce149a",
    "downlink": (
        "01a5c0bdf0ff508ef4ffe05ef8ff702ffcff0000000090d00300d4fed5fed6fed7fed8fed9fe00006400c8002c019001f4011e1f20212223000102030405c05d0300e74f00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"
    ),
}


class TestProtocol(unittest.TestCase):
    def test_crc_golden(self):
        self.assertEqual(rmproto.crc8(b"\x00"), GOLD["crc8_00"])
        self.assertEqual(rmproto.crc8(b"\x01"), GOLD["crc8_01"])
        self.assertEqual(rmproto.crc8(bytes([0xA5, 0, 0, 0])), GOLD["crc8_a5"])
        self.assertEqual(rmproto.crc16(bytes([0xA5, 0, 0, 0])), GOLD["crc16_a5"])
        self.assertEqual(rmproto.crc16(b"123456789"), 0x29B1)

    def test_uplink_golden(self):
        p = payload.Uplink(ver_mode=0x11, seq=9,
                           joints=[1000 + i * 100 for i in range(6)],
                           grip=1234, buttons=0x05, flags=0x01)
        got = payload.encode_uplink(p).hex()
        self.assertEqual(got, GOLD["uplink"])

    def test_uplink_roundtrip(self):
        p = payload.Uplink(seq=7, joints=[500, 600, 700, 800, 900, 2500],
                           grip=42, buttons=3, flags=1)
        q = payload.decode_uplink(payload.encode_uplink(p))
        self.assertIsNotNone(q)
        self.assertEqual(q.joints, p.joints)
        self.assertEqual(q.grip, 42)
        bad = bytearray(payload.encode_uplink(p))
        bad[5] ^= 1
        self.assertIsNone(payload.decode_uplink(bytes(bad)))

    def test_downlink_golden(self):
        d = payload.Downlink(ver=1, flags=0xA5, vbus_mv=24000, lost_frames=3)
        for i in range(6):
            d.joint[i] = payload.JointFb(pos_urad=-1000000 + i * 250000,
                                         vel_crad=-300 + i, torque_cnm=100 * i,
                                         temp_c=30 + i, err=i)
        got = payload.encode_downlink(d).hex()
        self.assertEqual(got, GOLD["downlink"])
        back = payload.decode_downlink(payload.encode_downlink(d))
        self.assertEqual(back.joint[5].pos_urad, d.joint[5].pos_urad)
        self.assertEqual(back.vbus_mv, 24000)

    def test_frame_golden(self):
        data = bytes((i * 7 + 3) & 0xFF for i in range(30))
        frame = rmproto.encode(0x0311, 42, data)
        self.assertEqual(frame.hex(), GOLD["frame0311"])

    def test_parser_resync(self):
        p = rmproto.Parser()
        frame = rmproto.encode(0x0311, 1, b"\x01\x02\x03")
        got = None
        for b in b"junk" + frame:
            got = p.feed(b) or got
        self.assertIsNotNone(got)
        self.assertEqual(got.cmd_id, 0x0311)
        self.assertEqual(got.data, b"\x01\x02\x03")

        # 坏帧之后能重新对齐
        p = rmproto.Parser()
        bad = bytearray(frame)
        bad[8] ^= 0xFF
        got = None
        for b in bytes(bad) + frame:
            got = p.feed(b) or got
        self.assertIsNotNone(got)
        self.assertEqual(got.seq, 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
