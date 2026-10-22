"""没有图传时，用 USB-TTL 直接给机器人端 C 板喂 0x0311 帧（台架验证用）。

    python3 uart_0311_inject.py --port /dev/tty.usbserial-XXXX --hz 75

同时读回 0x0310（300 B），打印 6 个关节的角度/力矩/温度。
"""

from __future__ import annotations

import argparse
import math
import time

import serial

import payload
import rmproto


def make_uplink(t: float, seq: int) -> payload.Uplink:
    joints = [int(1500 + 400 * math.sin(t + i * 0.6)) for i in range(6)]
    return payload.Uplink(seq=seq & 0xFF, joints=joints, flags=1)


class Injector:
    def __init__(self, port: str, baud: int = 921600, hz: float = 75.0):
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.01)
        self.period = 1.0 / hz
        self.parser = rmproto.Parser()
        self.seq = 0
        self.last = 0.0
        self.downlink: payload.Downlink | None = None
        self.telemetry_frames = 0

    def step(self) -> None:
        now = time.time()
        if now - self.last < self.period:
            return
        self.last = now
        up = make_uplink(now, self.seq)
        self.seq += 1
        frame = rmproto.encode(0x0311, up.seq, payload.encode_uplink(up))
        self.ser.write(frame)
        for b in self.ser.read(512):
            f = self.parser.feed(b)
            if f is not None and f.cmd_id == 0x0310 and len(f.data) == payload.DOWNLINK_LEN:
                d = payload.decode_downlink(f.data)
                if d is not None:
                    self.downlink = d
                    self.telemetry_frames += 1

    def close(self) -> None:
        self.ser.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--hz", type=float, default=75.0)
    args = ap.parse_args()

    inj = Injector(args.port, args.baud, args.hz)
    print(f"inject 0x0311 @{args.hz} Hz → {args.port}")
    try:
        last = time.time()
        while True:
            inj.step()
            time.sleep(0.001)
            if time.time() - last >= 1.0:
                last = time.time()
                d = inj.downlink
                if d is None:
                    print("telemetry: 无")
                else:
                    deg = [j.pos_urad / 1e6 * 57.2958 for j in d.joint]
                    print("telemetry:", " ".join(f"{x:6.1f}°" for x in deg),
                          f"vbus={d.vbus_mv}mV frames={inj.telemetry_frames}")
    except KeyboardInterrupt:
        pass
    finally:
        inj.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
