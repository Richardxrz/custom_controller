"""RM 串口帧编解码（与固件 Module/rm_frame.c 逐字节一致）。

帧：A5 | len(2,LE) | seq | CRC8 | cmd_id(2,LE) | data | CRC16(2,LE)
CRC8：poly 0x31 的反射形式（0x8C），init 0xFF
CRC16：CCITT，poly 0x1021，init 0xFFFF
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

SOF = 0xA5
DATA_MAX = 300


def crc8(data: bytes, init: int = 0xFF) -> int:
    c = init
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c >> 1) ^ 0x8C) if (c & 1) else (c >> 1)
    return c & 0xFF


def crc16(data: bytes, init: int = 0xFFFF) -> int:
    c = init
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if (c & 0x8000) else ((c << 1) & 0xFFFF)
    return c & 0xFFFF


def encode(cmd_id: int, seq: int, data: bytes) -> bytes:
    if len(data) > DATA_MAX:
        raise ValueError("data too long")
    header = bytes([SOF]) + struct.pack("<H", len(data)) + bytes([seq & 0xFF])
    head = header + bytes([crc8(header)])
    body = head + struct.pack("<H", cmd_id & 0xFFFF) + bytes(data)
    return body + struct.pack("<H", crc16(body))


@dataclass
class Frame:
    cmd_id: int
    seq: int
    data: bytes


class Parser:
    """逐字节喂，返回完整帧或 None；CRC/长度错时自动滑窗重同步。"""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, byte: int) -> Frame | None:
        self.buf.append(byte)
        while True:
            if len(self.buf) < 5:
                return None
            if self.buf[0] != SOF:
                del self.buf[0]
                continue
            if crc8(self.buf[:4]) != self.buf[4]:
                self.crc_errors += 1
                del self.buf[0]
                continue
            ln = self.buf[1] | (self.buf[2] << 8)
            if ln > DATA_MAX:
                del self.buf[0]
                continue
            total = 9 + ln
            if len(self.buf) < total:
                return None
            want = self.buf[7 + ln] | (self.buf[8 + ln] << 8)
            if crc16(self.buf[: 7 + ln]) != want:
                self.crc_errors += 1
                del self.buf[0]
                continue
            frame = Frame(
                cmd_id=self.buf[5] | (self.buf[6] << 8),
                seq=self.buf[3],
                data=bytes(self.buf[7 : 7 + ln]),
            )
            del self.buf[:total]
            return frame

    def reset(self) -> None:
        self.buf.clear()
