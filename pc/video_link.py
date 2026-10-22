"""图传视频 UDP 接收（端口 3334）。

2026 通信协议：每包前 8 字节固定为
    帧编号(2) | 当前帧内分片序号(2) | 当前帧总字节数(4)
后面是 HEVC 码流。本模块只做拆包/组帧/统计，不解码（解码留给播放器或后续接入）。
"""

from __future__ import annotations

import socket
import struct
import time


class VideoReceiver:
    def __init__(self, host: str = "0.0.0.0", port: int = 3334, timeout: float = 0.2):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.host, self.port = self.sock.getsockname()[:2]
        self.sock.settimeout(timeout)
        self.packets = 0
        self.frames = 0
        self.bytes = 0
        self.last_frame_size = 0
        self._frame_no = None
        self._need = 0
        self._got = 0
        self._t0 = time.time()

    def poll(self) -> bool:
        """收一批包；组出完整帧时返回 True。"""
        try:
            pkt, _ = self.sock.recvfrom(65535)
        except socket.timeout:
            return False
        if len(pkt) < 8:
            return False
        self.packets += 1
        frame_no, _slice, total = struct.unpack_from("<HHI", pkt, 0)
        payload = pkt[8:]
        self.bytes += len(payload)
        if frame_no != self._frame_no:
            self._frame_no = frame_no
            self._need = total
            self._got = 0
        self._got += len(payload)
        if self._need and self._got >= self._need:
            self.frames += 1
            self.last_frame_size = self._got
            self._frame_no = None
            self._need = 0
            self._got = 0
            return True
        return False

    def fps(self) -> float:
        dt = time.time() - self._t0
        return self.frames / dt if dt > 0 else 0.0

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass
