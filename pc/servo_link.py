"""PC 直连舵机：通过 ZL 转接板的 USB 串口轮询 6 轴，合成 0x0302 帧。

这样操作端可以完全不要 C 板：
    PC ──USB── ZL 转接板 ──单总线── 6×ZP10S
PC 既是终端（MQTT/视频），也是舵机采集器。
"""

from __future__ import annotations

import re
import time

import serial

import payload
import rmproto


class ServoLink:
    def __init__(self, port: str | None = None, ids: list[int] | None = None,
                 baud: int = 115200, per_axis_timeout: float = 0.01, ser=None):
        if ser is None:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0)
        else:
            self.ser = ser
        self.ids = ids or [1, 2, 3, 4, 5, 6]
        self.joints = [1500] * payload.JOINTS
        self.i = 0
        self.seq = 0
        self.ok = 0
        self.timeouts = 0
        self.per_axis_timeout = per_axis_timeout
        self.pattern = re.compile(rb"#(\d{3})P(\d{4})!")

    def _query(self, sid: int) -> int | None:
        self.ser.reset_input_buffer()
        self.ser.write(f"#{sid:03d}PRAD!".encode())
        buf = b""
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < self.per_axis_timeout:
            n = self.ser.in_waiting
            if n:
                buf += self.ser.read(n)
                if b"!" in buf:
                    break
            else:
                time.sleep(0)
        m = self.pattern.search(buf)
        if m and int(m.group(1)) == sid:
            return int(m.group(2))
        return None

    def read_angle(self, sid: int) -> int | None:
        """单次读一个轴（标定用，不推进轮询下标）。"""
        return self._query(sid)

    def poll(self) -> list[rmproto.Frame]:
        """每调用一次只问一个轴；问完一圈返回一帧 0x0302。"""
        sid = self.ids[self.i]
        val = self._query(sid)
        if val is None:
            self.timeouts += 1
        else:
            self.joints[self.i] = val
            self.ok += 1
        self.i += 1
        if self.i < len(self.ids):
            return []
        self.i = 0
        self.seq = (self.seq + 1) & 0xFF
        up = payload.Uplink(seq=self.seq, joints=self.joints, flags=0x01)
        return [rmproto.Frame(cmd_id=0x0302, seq=self.seq, data=payload.encode_uplink(up))]

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass
