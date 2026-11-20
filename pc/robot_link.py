"""PC ↔ 机器人 C 板的串口链路（图传的有线替身）。

帧：
    PC → 机器人   0x0311  30 B  CustomControl
    机器人 → PC   0x0310  300 B CustomByteBlock

用途：没有图传时，用 USB-TTL 直连机器人 C 板的图传 UART，协议完全不变。
"""

from __future__ import annotations

import threading
import time

import rmproto

UPLINK_CMD = 0x0311
DOWNLINK_CMD = 0x0310
UPLINK_LEN = 30
DOWNLINK_LEN = 300


class SerialRobotLink:
    def __init__(self, port: str, baud: int = 921600, on_telemetry=None):
        import serial

        self.ser = serial.Serial(port, baudrate=baud, timeout=0.01)
        self.on_telemetry = on_telemetry
        self.parser = rmproto.Parser()
        self.seq = 0
        self.last_tx = 0.0
        self.frames_tx = 0
        self.frames_rx = 0
        self.last_error = ""
        self.connected = True
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def publish_control(self, data30: bytes) -> bool:
        if len(data30) != UPLINK_LEN:
            raise ValueError("CustomControl 必须是 30 字节")
        now = time.monotonic()
        if now - self.last_tx < 1.0 / 75.0:
            return False
        self.last_tx = now
        try:
            self.ser.write(rmproto.encode(UPLINK_CMD, self.seq, data30))
            self.seq = (self.seq + 1) & 0xFF
            self.frames_tx += 1
            return True
        except Exception as exc:  # noqa: BLE001
            self.last_error = str(exc)
            self.connected = False
            return False

    def _reader(self) -> None:
        while not self._stop.is_set():
            try:
                data = self.ser.read(256)
            except Exception as exc:  # noqa: BLE001
                self.last_error = str(exc)
                self.connected = False
                return
            for byte in data:
                frame = self.parser.feed(byte)
                if frame is None:
                    continue
                if frame.cmd_id != DOWNLINK_CMD or len(frame.data) != DOWNLINK_LEN:
                    continue
                self.frames_rx += 1
                if self.on_telemetry is not None:
                    self.on_telemetry(frame.data)

    def close(self) -> None:
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass
        self.connected = False
        self._thread.join(timeout=0.5)
