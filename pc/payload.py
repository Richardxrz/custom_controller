"""30 B 上行 / 300 B 下行载荷（与固件 Module/app_payload.c 逐字节一致）。"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

from rmproto import crc16

UPLINK_LEN = 30
DOWNLINK_LEN = 300
JOINTS = 6
UPLINK_VER = 1


@dataclass
class Uplink:
    ver_mode: int = 0x11
    seq: int = 0
    joints: list[int] = field(default_factory=lambda: [1500] * JOINTS)
    grip: int = 0
    buttons: int = 0
    flags: int = 0


@dataclass
class JointFb:
    pos_urad: int = 0
    vel_crad: int = 0
    torque_cnm: int = 0
    temp_c: int = 0
    err: int = 0


@dataclass
class Downlink:
    ver: int = 1
    flags: int = 0
    joint: list[JointFb] = field(default_factory=lambda: [JointFb() for _ in range(JOINTS)])
    vbus_mv: int = 0
    lost_frames: int = 0


def _clamp_pwm(v: int) -> int:
    return 500 if v < 500 else 2500 if v > 2500 else v


def encode_uplink(p: Uplink) -> bytes:
    out = bytearray(UPLINK_LEN)
    out[0] = p.ver_mode & 0xFF
    out[1] = p.seq & 0xFF
    out[2:14] = struct.pack("<6H", *[_clamp_pwm(int(x)) for x in p.joints])
    struct.pack_into("<H", out, 14, p.grip & 0xFFFF)
    out[16] = p.buttons & 0xFF
    out[17] = p.flags & 0xFF
    struct.pack_into("<H", out, 28, crc16(bytes(out[:28])))
    return bytes(out)


def decode_uplink(data: bytes) -> Uplink | None:
    if len(data) != UPLINK_LEN:
        return None
    if crc16(data[:28]) != struct.unpack_from("<H", data, 28)[0]:
        return None
    return Uplink(
        ver_mode=data[0],
        seq=data[1],
        joints=[_clamp_pwm(x) for x in struct.unpack_from("<6H", data, 2)],
        grip=struct.unpack_from("<H", data, 14)[0],
        buttons=data[16],
        flags=data[17],
    )


def encode_downlink(p: Downlink) -> bytes:
    out = bytearray(DOWNLINK_LEN)
    out[0] = p.ver & 0xFF
    out[1] = p.flags & 0xFF
    struct.pack_into("<6i", out, 2, *[int(j.pos_urad) for j in p.joint])
    struct.pack_into("<6h", out, 26, *[int(j.vel_crad) for j in p.joint])
    struct.pack_into("<6h", out, 38, *[int(j.torque_cnm) for j in p.joint])
    for i, j in enumerate(p.joint):
        out[50 + i] = j.temp_c & 0xFF
    for i, j in enumerate(p.joint):
        out[56 + i] = j.err & 0xFF
    struct.pack_into("<H", out, 62, p.vbus_mv & 0xFFFF)
    struct.pack_into("<H", out, 64, p.lost_frames & 0xFFFF)
    struct.pack_into("<H", out, 66, crc16(bytes(out[:66])))
    return bytes(out)


def decode_downlink(data: bytes) -> Downlink | None:
    if len(data) != DOWNLINK_LEN:
        return None
    if crc16(data[:66]) != struct.unpack_from("<H", data, 66)[0]:
        return None
    p = Downlink(ver=data[0], flags=data[1])
    pos = struct.unpack_from("<6i", data, 2)
    vel = struct.unpack_from("<6h", data, 26)
    tor = struct.unpack_from("<6h", data, 38)
    p.joint = [
        JointFb(pos_urad=pos[i], vel_crad=vel[i], torque_cnm=tor[i],
                temp_c=data[50 + i], err=data[56 + i])
        for i in range(JOINTS)
    ]
    p.vbus_mv = struct.unpack_from("<H", data, 62)[0]
    p.lost_frames = struct.unpack_from("<H", data, 64)[0]
    return p
