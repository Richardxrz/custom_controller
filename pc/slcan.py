"""SLCAN（CANable / USB-CAN 常见固件）ASCII 协议：macOS 上不用装驱动，pyserial 即可。"""

from __future__ import annotations

import serial

BITRATE_CODE = {
    10_000: "S0",
    20_000: "S1",
    50_000: "S2",
    100_000: "S3",
    125_000: "S4",
    250_000: "S5",
    500_000: "S6",
    800_000: "S7",
    1_000_000: "S8",
}


class SlcanBus:
    def __init__(self, port: str | None = None, baud: int = 115200, ser=None):
        self.ser = ser if ser is not None else serial.Serial(port, baud, timeout=0)
        self.buf = ""

    def open(self, bitrate: int = 1_000_000) -> bool:
        code = BITRATE_CODE.get(bitrate)
        if code is None:
            raise ValueError(f"不支持的波特率 {bitrate}")
        self.ser.write((code + "\r").encode())
        self.ser.write(b"O\r")
        return True

    def close(self) -> None:
        try:
            self.ser.write(b"C\r")
        finally:
            self.ser.close()

    def send(self, can_id: int, data: bytes) -> None:
        if len(data) > 8:
            raise ValueError("经典 CAN 最多 8 字节")
        frame = "t%03X%X%s\r" % (can_id & 0x7FF, len(data), data.hex().upper())
        self.ser.write(frame.encode())

    def recv(self) -> list[tuple[int, bytes]]:
        raw = self.ser.read(512)
        if raw:
            self.buf += raw.decode("ascii", "ignore")
        out: list[tuple[int, bytes]] = []
        while "\r" in self.buf:
            line, self.buf = self.buf.split("\r", 1)
            line = line.strip()
            if len(line) < 5 or line[0] not in "tT":
                continue
            try:
                cid = int(line[1:4], 16)
                dlc = int(line[4], 16)
                payload = bytes.fromhex(line[5 : 5 + dlc * 2])
            except ValueError:
                continue
            if len(payload) == dlc:
                out.append((cid, payload))
        return out
