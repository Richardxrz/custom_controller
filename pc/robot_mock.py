"""机器人端替身：订阅 CustomControl，回 CustomByteBlock 遥测。

用途：没有图传/机器人时，在本地把「终端 → 服务器 → 机器人 → 终端」整条链路跑通。
"""

from __future__ import annotations

import argparse
import math
import time

import mqtt_link
import payload


class RobotMock:
    def __init__(self, host: str, port: int, client_id: str):
        self.link = mqtt_link.MqttLink(host, port, client_id)
        self.link.subscribe(mqtt_link.TOPIC_CONTROL, self._on_control)
        self.uplink: payload.Uplink | None = None
        self.controls = 0
        self.t0 = time.time()

    def _on_control(self, data: bytes) -> None:
        up = payload.decode_uplink(data)
        if up is None:
            return
        self.uplink = up
        self.controls += 1
        self.link.publish_telemetry(payload.encode_downlink(self._telemetry()))

    def _telemetry(self) -> payload.Downlink:
        d = payload.Downlink(ver=1, flags=0x01, vbus_mv=24000)
        t = time.time() - self.t0
        for i in range(payload.JOINTS):
            pwm = self.uplink.joints[i] if self.uplink else 1500
            rad = (pwm - 1500) / 1000.0 * 1.5
            d.joint[i].pos_urad = int(rad * 1e6)
            d.joint[i].vel_crad = int(100 * math.sin(t + i))
            d.joint[i].torque_cnm = int(10 * (i + 1))
            d.joint[i].temp_c = 30 + i
        return d

    def start(self) -> bool:
        return self.link.start()

    def stop(self) -> None:
        self.link.stop()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=3333)
    ap.add_argument("--id", default="1")
    args = ap.parse_args()
    r = RobotMock(args.host, args.port, f"robot-{args.id}")
    if not r.start():
        print("连接失败", r.link.last_error)
        return 1
    print("robot mock 已连接，等待 CustomControl...")
    try:
        while True:
            time.sleep(1)
            print("controls", r.controls)
    except KeyboardInterrupt:
        pass
    finally:
        r.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
