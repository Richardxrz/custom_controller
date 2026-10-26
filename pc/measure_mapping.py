"""实测主端 PRAD ↔ 电机位置的关系：回答「过圈是 PRAD 范围不对，还是单圈回绕」。

同时读 ZL 转接板（PRAD）和达妙 USB2CAN（电机反馈），存 CSV 并做线性拟合：

    python3 measure_mapping.py --servo-port /dev/tty.usbserial-1110 --servo-id 5 \
        --can-port /dev/tty.usbmodem00000000050C1 --motor-id 1 --seconds 20 \
        --out mapping.csv

测试时手扳舵机从一头到另一头，尽量慢、走完整段。
"""

from __future__ import annotations

import argparse
import json
import time

import dm_can


def fit_gain(pairs: list[tuple[int, float]]) -> dict:
    """最小二乘 pos = a*prad + b。"""
    n = len(pairs)
    if n < 2:
        return {"n": n}
    mp = sum(p for p, _ in pairs) / n
    mq = sum(q for _, q in pairs) / n
    var = sum((p - mp) ** 2 for p, _ in pairs)
    if var == 0:
        return {"n": n, "gain": 0.0, "offset": mq, "rms": 0.0}
    a = sum((p - mp) * (q - mq) for p, q in pairs) / var
    b = mq - a * mp
    rms = (sum((q - (a * p + b)) ** 2 for p, q in pairs) / n) ** 0.5
    return {"n": n, "gain": a, "offset": b, "rms": rms}


def analyze(rows: list[tuple[int, float]]) -> dict:
    if not rows:
        return {"n": 0}
    prads = [p for p, _ in rows]
    poss = [q for _, q in rows]
    span = max(prads) - min(prads)
    wraps = sum(1 for a, b in zip(prads, prads[1:]) if abs(b - a) > max(50, span * 0.5))
    return {
        "n": len(rows),
        "prad_min": min(prads),
        "prad_max": max(prads),
        "prad_span": span,
        "pos_min": round(min(poss), 3),
        "pos_max": round(max(poss), 3),
        "pos_span": round(max(poss) - min(poss), 3),
        "wraps": wraps,
        "fit": fit_gain(rows),
    }


def run(servo_port: str, servo_id: int, can_port: str | None, motor_id: int,
        seconds: float) -> list[tuple[int, float]]:
    from servo_link import ServoLink
    from usb2can_classic import Usb2CanClassic

    link = ServoLink(servo_port, [servo_id])
    bus = None
    motor = None
    if can_port:
        bus = Usb2CanClassic(can_port, 921600)
        bus.open(1_000_000)
        motor = dm_can.MotorCfg(id=motor_id)
    rows: list[tuple[int, float]] = []
    pos = 0.0
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            raw = link.read_angle(servo_id)
            if bus is not None:
                for f in bus.recv():
                    if f["cmd"] == 0x11:
                        fb = dm_can.feedback_decode(motor, f["data"])
                        if fb:
                            pos = fb["pos"]
            if raw is not None:
                rows.append((raw, pos))
            time.sleep(0.002)
    finally:
        link.close()
        if bus is not None:
            bus.close()
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--servo-port", required=True)
    ap.add_argument("--servo-id", type=int, default=1)
    ap.add_argument("--can-port")
    ap.add_argument("--motor-id", type=int, default=1)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--out")
    args = ap.parse_args()

    rows = run(args.servo_port, args.servo_id, args.can_port, args.motor_id, args.seconds)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write("t,prad,pos\n")
            for i, (p, q) in enumerate(rows):
                f.write(f"{i},{p},{q:.6f}\n")
    print(json.dumps(analyze(rows), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
