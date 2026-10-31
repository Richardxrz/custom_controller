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
import select
import sys
import termios
import time
import tty

import dm_can
import filters
import joint_map
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
    ap.add_argument("--map-json", help="calibrate.py 生成的 joint_map 配置")
    ap.add_argument("--prad-min", type=int, help="覆盖标定：PRAD 下限")
    ap.add_argument("--prad-max", type=int, help="覆盖标定：PRAD 上限")
    # 主端 270°（PRAD 500..2500）映射到关节 360°（-π..+π）
    ap.add_argument("--joint-min", type=float, default=-math.pi)
    ap.add_argument("--joint-max", type=float, default=math.pi)
    ap.add_argument("--dir", type=int, default=1, choices=[-1, 1])
    ap.add_argument("--wrap", action="store_true")
    ap.add_argument("--filter", action="store_true", help="对 PRAD 做 Hampel + One-Euro")
    ap.add_argument("--filter-cutoff", type=float, default=2.5)
    ap.add_argument("--filter-beta", type=float, default=0.7)
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

    jm = joint_map.JointMap(joint_min=args.joint_min, joint_max=args.joint_max,
                            direction=args.dir, wrap=args.wrap)
    if args.map_json:
        import json
        with open(args.map_json, encoding="utf-8") as f:
            cfg = json.load(f)
        cfg.pop("wrap", None) if not args.wrap else None
        jm = joint_map.JointMap(**{k: v for k, v in cfg.items() if k in
                                   {"prad_min", "prad_max", "joint_min", "joint_max", "direction", "wrap"}})
    if args.prad_min is not None:
        jm.prad_min = args.prad_min
    if args.prad_max is not None:
        jm.prad_max = args.prad_max

    latest = {"pwm": 1500}
    fc = filters.FilterChain(min_cutoff=args.filter_cutoff, beta=args.filter_beta) if args.filter else None
    flt = {"last_raw": None, "t": None, "out": 1500.0}

    def on_control(data: bytes) -> None:
        raw = _up0(data)
        if fc is None:
            latest["pwm"] = raw
            return
        if raw != flt["last_raw"]:
            now = time.monotonic()
            dt = 0.0 if flt["t"] is None else now - flt["t"]
            flt["t"] = now
            flt["last_raw"] = raw
            flt["out"] = fc.update(raw, max(dt, 1e-3))
        latest["pwm"] = flt["out"]

    mq = None
    if args.mqtt:
        import mqtt_link

        mq = mqtt_link.MqttLink(args.mqtt_host, args.mqtt_port, args.client_id)
        mq.subscribe(mqtt_link.TOPIC_CONTROL, on_control)
        if not mq.start():
            print("MQTT 连接失败", mq.last_error)
            motor.disable()
            bus.close()
            return 1
        print(f"MQTT {args.mqtt_host}:{args.mqtt_port} 已连接"
              + (f"；滤波 cutoff={args.filter_cutoff}Hz beta={args.filter_beta}" if fc else ""))

    pid = Pid(args.pid_kp, args.pid_ki, args.pid_kd, args.torque_limit)
    dt = 1.0 / args.hz
    t0 = time.time()
    next_t = t0
    last_print = t0
    last_telem = t0

    interactive = sys.stdin.isatty()
    old_term = None
    if interactive:
        old_term = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        print("Ctrl-C 退出；c = 离合开/关（保持/重对齐）")

    def read_key() -> str:
        if not interactive:
            return ""
        r, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if r else ""

    print("控制中，Ctrl-C 退出（退出时会失能电机）")
    try:
        while args.seconds is None or time.time() - t0 < args.seconds:
            now = time.time()
            if now < next_t:
                time.sleep(0.0005)
                continue
            next_t += dt
            motor.poll()

            key = read_key()
            if key in ("c", "C"):
                if jm.clutch:
                    jm.release(latest["pwm"])
                    print("离合：松开（以当前位置重对齐）")
                else:
                    jm.engage(latest["pwm"])
                    print("离合：保持")
            elif key in ("q", "Q"):
                break

            if args.test:
                target = args.range * math.sin((now - t0) * 1.2)
            else:
                target = jm.to_joint(latest["pwm"])

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
        if interactive and old_term is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
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
