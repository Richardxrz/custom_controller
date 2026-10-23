"""MQTT ↔ 0x0311 桥：没有图传时，用 USB-TTL 把 CustomControl 注入机器人 C 板。

链路：
    terminal_app（舵机） --MQTT--> 本桥 --UART 921600--> 臂控 C 板 --CAN--> 达妙
    臂控 C 板 --UART 0x0310--> 本桥 --MQTT CustomByteBlock--> 终端

用法：
    python3 uart_0311_bridge.py --mqtt-host 127.0.0.1 --mqtt-port 3333 \
        --id 1 --port /dev/tty.usbserial-XXXX --baud 921600
"""

from __future__ import annotations

import argparse
import queue
import sys
import time

import serial

import mqtt_link
import payload
import rmproto


class FakeSer:
    """测试替身：记录写出的字节，并从 rx 缓冲里吐字节。"""

    def __init__(self):
        self.tx = bytearray()
        self.rx = bytearray()

    def write(self, data):
        self.tx += data
        return len(data)

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, n=1):
        data = bytes(self.rx[:n])
        del self.rx[:n]
        return data

    def close(self):
        pass


class Bridge:
    def __init__(self, mqtt_host: str, mqtt_port: int, client_id: str,
                 port: str | None = None, baud: int = 921600, ser=None):
        self.mq = mqtt_link.MqttLink(mqtt_host, mqtt_port, client_id)
        self.mq.subscribe(mqtt_link.TOPIC_CONTROL, self._on_control)
        self.ser = ser if ser is not None else serial.Serial(port, baud, timeout=0)
        self.parser = rmproto.Parser()
        self.seq = 0
        self.pending: queue.Queue[bytes] = queue.Queue(maxsize=4)
        self.sent = 0
        self.telemetry = 0
        self.dropped = 0
        self.last_downlink: payload.Downlink | None = None

    def _on_control(self, data: bytes) -> None:
        if len(data) != payload.UPLINK_LEN:
            return
        try:
            self.pending.put_nowait(data)
        except queue.Full:
            self.dropped += 1

    def start(self) -> bool:
        return self.mq.start()

    def step(self) -> None:
        # 1) 把最新的 CustomControl 转成 0x0311 发给臂控
        while True:
            try:
                data = self.pending.get_nowait()
            except queue.Empty:
                break
            frame = rmproto.encode(0x0311, self.seq, data)
            self.seq = (self.seq + 1) & 0xFF
            self.ser.write(frame)
            self.sent += 1

        # 2) 收臂控的 0x0310，转成 CustomByteBlock 回终端
        n = getattr(self.ser, "in_waiting", 0)
        if n:
            for b in self.ser.read(n):
                f = self.parser.feed(b)
                if f is None or f.cmd_id != 0x0310 or len(f.data) != payload.DOWNLINK_LEN:
                    continue
                d = payload.decode_downlink(f.data)
                if d is None:
                    continue
                self.last_downlink = d
                self.telemetry += 1
                self.mq.publish_telemetry(f.data)

    def run(self, seconds: float | None = None) -> None:
        t0 = time.time()
        while seconds is None or time.time() - t0 < seconds:
            self.step()
            time.sleep(0.001)

    def close(self) -> None:
        self.mq.stop()
        self.ser.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mqtt-host", default="127.0.0.1")
    ap.add_argument("--mqtt-port", type=int, default=3333)
    ap.add_argument("--id", default="1")
    ap.add_argument("--port", required=True, help="接臂控 C 板 UART1 的串口")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=None)
    args = ap.parse_args()

    bridge = Bridge(args.mqtt_host, args.mqtt_port, args.id, args.port, args.baud)
    if not bridge.start():
        print("MQTT 连接失败", bridge.mq.last_error)
        return 1
    print(f"bridge: MQTT {args.mqtt_host}:{args.mqtt_port} ↔ {args.port} @{args.baud}")
    try:
        t0 = time.time()
        last = t0
        while args.seconds is None or time.time() - t0 < args.seconds:
            bridge.step()
            time.sleep(0.001)
            if time.time() - last >= 1.0:
                last = time.time()
                extra = ""
                if bridge.last_downlink is not None:
                    deg = [j.pos_urad / 1e6 * 57.2958 for j in bridge.last_downlink.joint]
                    extra = "  " + " ".join(f"{x:5.1f}°" for x in deg)
                print(f"sent={bridge.sent} telem={bridge.telemetry} "
                      f"drop={bridge.dropped}{extra}")
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
