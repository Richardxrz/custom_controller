"""串口链路：读字节 → RM 帧。"""

from __future__ import annotations

import time

import serial

import rmproto


class SerialLink:
    def __init__(self, port: str, baud: int = 921600, read_timeout: float = 0.01):
        self.ser = serial.Serial(port, baudrate=baud, timeout=read_timeout)
        self.parser = rmproto.Parser()
        self.frames = 0
        self.bytes = 0

    def poll(self) -> list[rmproto.Frame]:
        data = self.ser.read(256)
        if not data:
            return []
        self.bytes += len(data)
        out = []
        for b in data:
            f = self.parser.feed(b)
            if f is not None:
                self.frames += 1
                out.append(f)
        return out

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass


def pty_pair():
    """测试用：返回 (master_fd, slave_path)。"""
    import os
    import pty

    master, slave = pty.openpty()
    return master, os.ttyname(slave)


def wait_frames(link: SerialLink, n: int, timeout: float = 1.0) -> list[rmproto.Frame]:
    out: list[rmproto.Frame] = []
    t0 = time.time()
    while len(out) < n and time.time() - t0 < timeout:
        out.extend(link.poll())
    return out
