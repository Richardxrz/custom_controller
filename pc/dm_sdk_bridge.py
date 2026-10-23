"""用达妙官方 USB2CANFD 模块 + 官方 Python SDK，把 CustomControl 直接变成电机 MIT 指令。

这个脚本必须在 **Python 3.10** 环境里跑（官方 macOS 的 usb_class 只提供 3.10 的 .so）：

    conda create -n dm310 python=3.10 -y
    conda run -n dm310 pip install pyusb paho-mqtt pyserial
    ~/miniconda3/envs/dm310/bin/python dm_sdk_bridge.py --sn <模块SN> --id 1 --motor 4340 ...

先查 SN：
    ~/miniconda3/envs/dm310/bin/python vendor/dm_sdk/dev_sn.py
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "vendor", "dm_sdk"))

import payload  # noqa: E402
import mqtt_link  # noqa: E402

from damiao import (  # noqa: E402
    Control_Mode,
    Control_Mode_Code,
    DM_Motor_Type,
    DmActData,
    Motor_Control,
)

MOTORS = {
    "4340": DM_Motor_Type.DM4340,
    "8009": DM_Motor_Type.DM8009,
    "4310": DM_Motor_Type.DM4310,
    "6006": DM_Motor_Type.DM6006,
}


def pwm_to_rad(pwm: int) -> float:
    return (pwm - 1500) / 1000.0 * 1.5


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sn", required=True, help="模块 USB 序列号（dev_sn.py 查）")
    ap.add_argument("--id", type=int, default=1, help="电机 CAN ID")
    ap.add_argument("--mst-id", type=int, default=0x00, help="电机反馈的 master ID")
    ap.add_argument("--motor", choices=sorted(MOTORS), default="4340")
    ap.add_argument("--nom-baud", type=int, default=1_000_000)
    ap.add_argument("--dat-baud", type=int, default=5_000_000)
    ap.add_argument("--kp", type=float, default=2.0)
    ap.add_argument("--kd", type=float, default=0.2)
    ap.add_argument("--hz", type=float, default=200.0)
    ap.add_argument("--test", action="store_true", help="不接 MQTT，正弦目标")
    ap.add_argument("--range", type=float, default=0.5)
    ap.add_argument("--mqtt-host", default="127.0.0.1")
    ap.add_argument("--mqtt-port", type=int, default=3333)
    ap.add_argument("--client-id", default="1")
    ap.add_argument("--seconds", type=float, default=None)
    args = ap.parse_args()

    act = DmActData(
        motorType=MOTORS[args.motor],
        mode=Control_Mode.MIT_MODE,
        can_id=args.id,
        mst_id=args.mst_id,
    )

    latest = {"pwm": 1500}
    mq = None
    if not args.test:
        mq = mqtt_link.MqttLink(args.mqtt_host, args.mqtt_port, args.client_id)
        mq.subscribe(mqtt_link.TOPIC_CONTROL, lambda d: latest.update(pwm=_up0(d)))
        if not mq.start():
            print("MQTT 连接失败", mq.last_error)
            return 1
        print(f"MQTT {args.mqtt_host}:{args.mqtt_port} 已连接")

    print("初始化 USB2CANFD ...")
    with Motor_Control(args.nom_baud, args.dat_baud, args.sn, [act]) as ctl:
        motor = ctl.getMotor(args.id)
        # 官方 enable_all 默认切到 VEL；这里显式切回 MIT 再使能
        ctl.switchControlMode(motor, Control_Mode_Code.MIT)
        time.sleep(0.02)
        ctl.control_cmd(motor.GetCanId() + Control_Mode.MIT_MODE, 0xFC)
        time.sleep(0.02)

        dt = 1.0 / args.hz
        t0 = time.perf_counter()
        next_t = t0
        last_print = t0
        last_telem = t0
        print("控制中，Ctrl-C 退出")
        try:
            while args.seconds is None or time.perf_counter() - t0 < args.seconds:
                now = time.perf_counter()
                if now < next_t:
                    time.sleep(0.0003)
                    continue
                next_t += dt
                if args.test:
                    target = args.range * math.sin((now - t0) * 1.2)
                else:
                    target = pwm_to_rad(latest["pwm"])
                ctl.control_mit(motor, args.kp, args.kd, target, 0.0, 0.0)

                if mq is not None and now - last_telem >= 0.02:
                    last_telem = now
                    mq.publish_telemetry(
                        payload.encode_downlink(_downlink(motor.Get_Position(),
                                                          motor.Get_Velocity(),
                                                          motor.Get_tau()))
                    )
                if now - last_print >= 0.5:
                    last_print = now
                    print(f"target={target:+.2f}  pos={motor.Get_Position():+.3f}  "
                          f"vel={motor.Get_Velocity():+.2f}  tau={motor.Get_tau():+.2f}")
        except KeyboardInterrupt:
            pass
        finally:
            ctl.disable_all()
    if mq is not None:
        mq.stop()
    return 0


def _up0(data: bytes) -> int:
    up = payload.decode_uplink(data)
    return up.joints[0] if up else 1500


def _downlink(pos: float, vel: float, tau: float) -> bytes:
    d = payload.Downlink(ver=1, flags=0x01, vbus_mv=24000)
    d.joint[0].pos_urad = int(pos * 1e6)
    d.joint[0].vel_crad = int(vel * 100)
    d.joint[0].torque_cnm = int(tau * 100)
    return payload.encode_downlink(d)


if __name__ == "__main__":
    raise SystemExit(main())
