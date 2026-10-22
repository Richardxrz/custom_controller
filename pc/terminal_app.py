"""终端应用：串口读主端 → 30 B 上行 → MQTT；MQTT 下行 → 300 B 遥测。

两种跑法：
    python3 terminal_app.py --port /dev/tty.usbserial-XXX --id 1
    python3 terminal_app.py --simulate --id 1     # 没有串口时用正弦波模拟
"""

from __future__ import annotations

import argparse
import math
import time

import mqtt_link
import payload
import rmproto
import serial_link


class SimLink:
    """没有硬件时的替身：100 Hz 产生 0x0302 帧，关节做正弦。"""

    def __init__(self, hz: float = 100.0):
        self.period = 1.0 / hz
        self.t0 = time.time()
        self.last = 0.0
        self.seq = 0

    def poll(self):
        now = time.time()
        if now - self.last < self.period:
            return []
        self.last = now
        t = now - self.t0
        joints = [int(1500 + 400 * math.sin(t * 1.0 + i * 0.7)) for i in range(6)]
        up = payload.Uplink(seq=self.seq & 0xFF, joints=joints, flags=1)
        self.seq += 1
        return [rmproto.Frame(cmd_id=0x0302, seq=up.seq, data=payload.encode_uplink(up))]

    def close(self):
        pass


class TerminalApp:
    def __init__(self, link, mq: mqtt_link.MqttLink, hz: float = 75.0):
        self.link = link
        self.mq = mq
        self.period = 1.0 / hz
        self.last_pub = 0.0
        self.latest: payload.Uplink | None = None
        self.downlink: payload.Downlink | None = None
        self.serial_frames = 0
        self.dropped = 0
        self.latency_ms = 0.0

    def _on_telemetry(self, data: bytes) -> None:
        d = payload.decode_downlink(data)
        if d is not None:
            self.downlink = d

    def step(self) -> None:
        now = time.time()
        for f in self.link.poll():
            if f.cmd_id != 0x0302 or len(f.data) != payload.UPLINK_LEN:
                continue
            up = payload.decode_uplink(f.data)
            if up is None:
                self.dropped += 1
                continue
            self.serial_frames += 1
            self.latest = up
        if self.latest is not None and now - self.last_pub >= self.period:
            self.last_pub = now
            self.mq.publish_control(payload.encode_uplink(self.latest))

    def run(self, seconds: float | None = None) -> None:
        t0 = time.time()
        while seconds is None or time.time() - t0 < seconds:
            self.step()
            time.sleep(0.001)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="串口（主端 C 板）")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--host", default=mqtt_link.default_config()["host"])
    ap.add_argument("--mqtt-port", type=int, default=mqtt_link.default_config()["port"])
    ap.add_argument("--id", default="1", help="clientID = 机器人 ID")
    ap.add_argument("--hz", type=float, default=75.0)
    ap.add_argument("--simulate", action="store_true")
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    link = SimLink() if args.simulate else serial_link.SerialLink(args.port, args.baud)
    mq = mqtt_link.MqttLink(args.host, args.mqtt_port, args.id)
    app = TerminalApp(link, mq, args.hz)
    mq.handlers[mqtt_link.TOPIC_BYTE_BLOCK] = app._on_telemetry

    if not mq.start():
        print(f"MQTT 连接失败: {mq.last_error}")
        return 1
    print(f"已连接 {args.host}:{args.mqtt_port}，clientID={args.id}，"
          f"{'模拟' if args.simulate else args.port}")
    try:
        t0 = time.time()
        last_report = t0
        while args.seconds is None or time.time() - t0 < args.seconds:
            app.step()
            time.sleep(0.001)
            if time.time() - last_report >= 1.0:
                last_report = time.time()
                line = (f"serial={app.serial_frames} pub={mq.published} "
                        f"down={mq.received} drop={app.dropped}")
                if args.verbose and app.latest is not None:
                    line += " joints=" + ",".join(str(x) for x in app.latest.joints)
                print(line)
    except KeyboardInterrupt:
        pass
    finally:
        mq.stop()
        link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
