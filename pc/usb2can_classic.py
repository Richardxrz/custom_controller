"""达妙经典 USB2CAN（非 FD，CDC 串口，VID 2E88:4603）的 Python 驱动。

协议来自官方《USB 转 CAN 帧格式》+ 开源上位机 `can.cs`：

发送（30 字节，小端）：
    55 AA 1E 01 | sendTimes(u32) | sendInterval(u32, 100us) | idType(u8)
    | CANID(u32) | frameType(u8) | dataLen(u8) | idAcc(u8) | dataAcc(u8)
    | data[8] | CRC(u8，底层未解析，填 0)

接收（固定 16 字节）：
    AA | CMD | fmt(len:6|ide:1|rtr:1) | CANID(u32) | data[8] | 55

控制命令：
    握手：55 02 AA 55
    停止连续发送：55 03 AA 55
    设置 CAN 波特率：55 05 <index> AA 55（index 0 = 1 Mbps）
"""

from __future__ import annotations

import struct
import time

import serial

CAN_BAUD_INDEX = {
    1_000_000: 0,
    800_000: 1,
    666_666: 2,
    500_000: 3,
    400_000: 4,
    250_000: 5,
    200_000: 6,
    125_000: 7,
    100_000: 8,
    80_000: 9,
    50_000: 10,
    40_000: 11,
    20_000: 12,
    10_000: 13,
    5_000: 14,
}


def build_send(can_id: int, data: bytes, times: int = 1, interval: int = 0,
               id_type: int = 0, frame_type: int = 0,
               id_acc: int = 0, data_acc: int = 0) -> bytes:
    if len(data) > 8:
        raise ValueError("经典 CAN 最多 8 字节")
    buf = bytearray(30)
    buf[0], buf[1], buf[2], buf[3] = 0x55, 0xAA, 0x1E, 0x01
    struct.pack_into("<I", buf, 4, times & 0xFFFFFFFF)
    struct.pack_into("<I", buf, 8, interval & 0xFFFFFFFF)
    buf[12] = id_type & 0xFF
    struct.pack_into("<I", buf, 13, can_id & 0x1FFFFFFF)
    buf[17] = frame_type & 0xFF
    buf[18] = len(data)
    buf[19] = id_acc & 0xFF
    buf[20] = data_acc & 0xFF
    buf[21 : 21 + len(data)] = data
    buf[29] = 0  # CRC 底层不校验
    return bytes(buf)


class Usb2CanClassic:
    def __init__(self, port: str | None = None, baud: int = 921600, ser=None):
        self.ser = ser if ser is not None else serial.Serial(port, baud, timeout=0)
        self.buf = bytearray()
        self.frames = 0

    def open(self, can_bitrate: int = 1_000_000, handshake: bool = True) -> None:
        if handshake:
            self.ser.write(b"\x55\x02\xAA\x55")
            time.sleep(0.05)
            self.ser.read(64)
        idx = CAN_BAUD_INDEX.get(can_bitrate)
        if idx is None:
            raise ValueError(f"不支持的 CAN 波特率 {can_bitrate}")
        self.ser.write(bytes([0x55, 0x05, idx, 0xAA, 0x55]))
        time.sleep(0.05)
        self.ser.read(64)
        self.buf.clear()

    def send(self, can_id: int, data: bytes, times: int = 1, interval: int = 0) -> None:
        self.ser.write(build_send(can_id, data, times, interval))

    def recv(self) -> list[dict]:
        raw = self.ser.read(512)
        if raw:
            self.buf += raw
        out: list[dict] = []
        while len(self.buf) >= 16:
            if self.buf[0] != 0xAA or self.buf[15] != 0x55:
                del self.buf[0]
                continue
            cmd = self.buf[1]
            fmt = self.buf[2]
            can_id = struct.unpack_from("<I", self.buf, 3)[0]
            data = bytes(self.buf[7:15])
            out.append({"cmd": cmd, "fmt": fmt, "id": can_id & 0x1FFFFFFF, "data": data})
            self.frames += 1
            del self.buf[:16]
        return out

    def close(self) -> None:
        try:
            self.ser.write(b"\x55\x03\xAA\x55")
        finally:
            self.ser.close()
