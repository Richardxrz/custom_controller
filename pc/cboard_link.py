"""C 板主端串口：读 0x0302/0x0303/0x0304，写配置/扫描帧。"""

from __future__ import annotations

import struct

import rmproto
import serial_link


class CBoardLink(serial_link.SerialLink):
    def __init__(self, port: str, baud: int = 921600):
        super().__init__(port, baud)
        self.seq = 0

    def _send(self, cmd: int, data: bytes) -> None:
        try:
            self.ser.write(rmproto.encode(cmd, self.seq, data))
            self.seq = (self.seq + 1) & 0xFF
        except Exception:
            pass

    def send_config(self, range_deg: int, dir_mask: int = 0) -> None:
        self._send(0x0301, struct.pack("<HBB", int(range_deg), int(dir_mask) & 0x3F, 0))

    def request_scan(self) -> None:
        self._send(0x0300, b"")
