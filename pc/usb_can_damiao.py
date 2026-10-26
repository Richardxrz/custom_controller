"""不经 C 板：PC + USB-CAN（SLCAN）直接驱动达妙电机。

两种模式：
    --test          正弦目标，验证 CAN 链路和电机
    --mqtt          订阅 CustomControl，用 J0 控制电机（舵机 → PC → CAN → 电机）

两种控制：
    --mode mit      直接发 MIT（Kp/Kd 是电机内部的位置环）
    --mode pid      外环 PID（PC 上跑）算力矩，再以 Kp=0/Kd=阻尼 发 MIT

用法：
    python3 usb_can_damiao.py --port /dev/tty.usbmodemXXXX --test --id 1 --kp 2 --kd 0.2
    python3 usb_can_damiao.py --port /dev/tty.usbmodemXXXX --mqtt --id 1 --mode pid
"""

from __future__ import annotations

import argparse
import math
import time

import dm_can
import payload
import slcan
import usb2can_classic


class Pid:
    def __init__(self, kp: float, ki: float, kd: float, limit: float):
        self.kp, self.ki, self.kd, self.limit = kp, ki, kd, limit
        self.integ = 0.0
        self.prev = None

    def step(self, err: float, dt: float) -> float:
        self.integ += err * dt
        # 积分限幅，防止上电/大误差时风up
        self.integ = max(-self.limit, min(self.limit, self.integ))
        d = 0.0 if self.prev is None else (err - self.prev) / max(dt, 1e-3)
        self.prev = err
        out = self.kp * err + self.ki * self.integ + self.kd * d
        return max(-self.limit, min(self.limit, out))


class Damiao:
    def __init__(self, bus: slcan.SlcanBus, cfg: dm_can.MotorCfg):
        self.bus = bus
        self.cfg = cfg
        self.fb = None

    def enable(self) -> None:
        self.bus.send(self.cfg.id, dm_can.special("enable"))

    def disable(self) -> None:
        self.bus.send(self.cfg.id, dm_can.special("disable"))

    def command(self, pos, vel=0.0, kp=0.0, kd=0.0, torque=0.0) -> None:
        self.bus.send(self.cfg.id, dm_can.mit_pack(self.cfg, pos, vel, kp, kd, torque))

    def poll(self) -> None:
        for item in self.bus.recv():
            if isinstance(item, dict):
                if item.get("cmd") not in (0x11, 0x12):
                    continue
                cid, data = item["id"], item["data"]
            else:
                cid, data = item
            fb = dm_can.feedback_decode(self.cfg, data)
            if fb is not None:
                self.fb = fb


def pwm_to_rad(pwm: int) -> float:
    return (pwm - 1500) / 1000.0 * 1.5


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--transport", choices=["slcan", "dm_classic"], default="slcan")
    ap.add_argument("--serial-baud", type=int, default=921600, help="CDC 串口波特率（dm_classic）")
    ap.add_argument("--baud", type=int, default=115200, help="SLCAN 串口波特率，不是 CAN 波特率")
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--mode", choices=["mit", "pid"], default="mit")
    ap.add_argument("--kp", type=float, default=2.0)
    ap.add_argument("--kd", type=float, default=0.2)
    ap.add_argument("--pid-kp", type=float, default=3.0)
    ap.add_argument("--pid-ki", type=float, default=0.5)
    ap.add_argument("--pid-kd", type=float, default=0.05)
    ap.add_argument("--torque-limit", type=float, default=2.0)
    ap.add_argument("--hz", type=float, default=200.0)
    ap.add_argument("--range", type=float, default=0.5, help="--test 的正弦幅度（rad）")
    ap.add_argument("--test", action="store_true")
    ap.add_argument("--mqtt", action="store_true")
    ap.add_argument("--mqtt-host", default="127.0.0.1")
    ap.add_argument("--mqtt-port", type=int, default=3333)
    ap.add_argument("--client-id", default="1")
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--dry-run", action="store_true", help="不接硬件，只打印 CAN 帧")
    args = ap.parse_args()

    if args.dry_run:
        class _Dry:
            def send(self, can_id, data):
                pass

            def recv(self):
                return []

            def close(self):
                pass

        bus = _Dry()
        print("dry-run：只跑控制循环，不接硬件")
    elif args.transport == "dm_classic":
        bus = usb2can_classic.Usb2CanClassic(args.port, args.serial_baud)
        bus.open(args.bitrate)
        print("transport: 达妙经典 USB2CAN")
    else:
        bus = slcan.SlcanBus(args.port, args.baud)
        bus.open(args.bitrate)
        print("transport: SLCAN")
    cfg = dm_can.MotorCfg(id=args.id)
    motor = Damiao(bus, cfg)
    motor.enable()
    time.sleep(0.02)

    latest = {"pwm": 1500}
    mq = None
    if args.mqtt:
        import mqtt_link

        mq = mqtt_link.MqttLink(args.mqtt_host, args.mqtt_port, args.client_id)
        mq.subscribe(mqtt_link.TOPIC_CONTROL, lambda d: latest.update(pwm=_up0(d)))
        if not mq.start():
            print("MQTT 连接失败", mq.last_error)
            motor.disable()
            bus.close()
            return 1
        print(f"MQTT {args.mqtt_host}:{args.mqtt_port} 已连接")

    pid = Pid(args.pid_kp, args.pid_ki, args.pid_kd, args.torque_limit)
    dt = 1.0 / args.hz
    t0 = time.time()
    next_t = t0
    last_print = t0
    last_telem = t0

    print("控制中，Ctrl-C 退出（退出时会失能电机）")
    try:
        while args.seconds is None or time.time() - t0 < args.seconds:
            now = time.time()
            if now < next_t:
                time.sleep(0.0005)
                continue
            next_t += dt
            motor.poll()

            if args.test:
                target = args.range * math.sin((now - t0) * 1.2)
            else:
                target = pwm_to_rad(latest["pwm"])

            if args.mode == "pid":
                pos = motor.fb["pos"] if motor.fb else 0.0
                torque = pid.step(target - pos, dt)
                motor.command(pos, 0.0, 0.0, args.kd, torque)
            else:
                motor.command(target, 0.0, args.kp, args.kd, 0.0)

            if mq is not None and now - last_telem >= 0.02:
                last_telem = now
                mq.publish_telemetry(_downlink(motor.fb))

            if now - last_print >= 0.5:
                last_print = now
                fb = motor.fb
                if fb is None:
                    print(f"target={target:+.2f}  无反馈")
                else:
                    print(f"target={target:+.2f}  pos={fb['pos']:+.3f}  "
                          f"vel={fb['vel']:+.2f}  tau={fb['torque']:+.2f}  T={fb['temp_mos']}/{fb['temp_rotor']}℃ state={fb['state']:#04x}")
    except KeyboardInterrupt:
        pass
    finally:
        motor.disable()
        bus.close()
        if mq is not None:
            mq.stop()
    return 0


def _up0(data: bytes) -> int:
    up = payload.decode_uplink(data)
    return up.joints[0] if up else 1500


def _downlink(fb) -> bytes:
    d = payload.Downlink(ver=1, flags=0x01 if fb else 0x00, vbus_mv=24000)
    if fb:
        d.joint[0].pos_urad = int(fb["pos"] * 1e6)
        d.joint[0].vel_crad = int(fb["vel"] * 100)
        d.joint[0].torque_cnm = int(fb["torque"] * 100)
        d.joint[0].err = fb["state"]
    return payload.encode_downlink(d)


if __name__ == "__main__":
    raise SystemExit(main())
