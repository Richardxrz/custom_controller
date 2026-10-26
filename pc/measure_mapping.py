"""实测主端 PRAD ↔ 电机位置的关系：回答「过圈是 PRAD 范围不对，还是单圈回绕」。

用法（硬件接好后）：

    python3 measure_mapping.py --servo-port /dev/tty.usbserial-1110 --servo-id 5 \
        --can-port /dev/tty.usbmodem00000000050C1 --motor-id 1 --seconds 20 \
        --out mapping.csv

操作：运行后用手把舵机从一端慢慢扳到另一端（走完整段，别快），脚本同时记录
      「PRAD」和「电机实际位置」，结束后打印统计 + 最小二乘拟合。

只测 PRAD（不接电机）也可以：去掉 --can-port，此时 pos 恒为 0，只看 prad_min/max。
"""

from __future__ import annotations

import argparse
import json
import time
from itertools import pairwise

import dm_can


def fit_gain(pairs: list[tuple[int, float]]) -> dict:
    """最小二乘拟合一条直线：pos = a * prad + b。

    为什么用最小二乘：
        每个采样点 (p_i, q_i) 不会完美落在一条直线上（噪声、间隙、量化）。
        我们要找一条直线，使所有点到它的「竖直距离平方和」最小：
            S(a, b) = Σ (q_i - (a*p_i + b))²
        对 a、b 分别求偏导并令其为 0，得到：
            a = Σ(p_i - p̄)(q_i - q̄) / Σ(p_i - p̄)²
            b = q̄ - a * p̄
        这就是下面代码里 a、b 的来源；p̄、q̄ 是均值。

    返回：
        gain  —— 每 1 个 PRAD 计数对应多少 rad（例如 3 rad / 2000 计数 = 0.0015）
        offset—— 截距，主要用于诊断（正常应接近 -gain*中心值）
        rms   —— 残差均方根，越小说明「PRAD 与电机位置越接近线性」
    """
    n = len(pairs)
    if n < 2:  # 一个点连不成直线
        return {"n": n}

    mp = sum(p for p, _ in pairs) / n  # PRAD 均值 p̄
    mq = sum(q for _, q in pairs) / n  # 位置均值 q̄

    # 分母：Σ(p_i - p̄)²。这里没有除以 n，所以下面 a 的分子分母会约掉 n。
    var = sum((p - mp) ** 2 for p, _ in pairs)
    if var == 0:  # 所有 PRAD 相同，斜率无定义（舵机没动）
        return {"n": n, "gain": 0.0, "offset": mq, "rms": 0.0}

    # 分子：Σ(p_i - p̄)(q_i - q̄)。a = 协方差 / 方差。
    a = sum((p - mp) * (q - mq) for p, q in pairs) / var
    b = mq - a * mp  # 直线必过均值点 (p̄, q̄)

    # RMS：每个点与直线的偏差，单位是 rad。用来判断“线性假设”靠不靠谱。
    rms = (sum((q - (a * p + b)) ** 2 for p, q in pairs) / n) ** 0.5
    return {"n": n, "gain": a, "offset": b, "rms": rms}


def analyze(rows: list[tuple[int, float]]) -> dict:
    """对整段记录做统计：PRAD 范围、电机范围、回绕次数、线性拟合。"""
    if not rows:
        return {"n": 0}

    prads = [p for p, _ in rows]
    poss = [q for _, q in rows]
    span = max(prads) - min(prads)

    # 回绕检测：相邻两点 PRAD 跳变超过半程，就认为跨过了 0/最大值的边界。
    # pairwise(prads) 等价于 zip(prads, prads[1:])，但不会复制一份切片。
    wraps = sum(
        1
        for a, b in pairwise(prads)
        if abs(b - a) > max(50, span * 0.5)
    )

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


def run(
    servo_port: str, servo_id: int, can_port: str | None, motor_id: int, seconds: float
) -> list[tuple[int, float]]:
    """同时采样 PRAD 和电机位置，返回 (prad, pos) 列表。"""
    from servo_link import ServoLink
    from usb2can_classic import Usb2CanClassic

    link = ServoLink(servo_port, [servo_id])

    bus: Usb2CanClassic | None = None
    motor: dm_can.MotorCfg | None = None
    if can_port:
        bus = Usb2CanClassic(can_port, 921600)
        bus.open(1_000_000)
        motor = dm_can.MotorCfg(id=motor_id)

    rows: list[tuple[int, float]] = []
    pos = 0.0  # 没有 --can-port 时 pos 恒为 0
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            raw = link.read_angle(servo_id)

            if bus is not None:
                # Pyright 报错点：motor 的类型是 MotorCfg | None，这里断言非空，
                # 之后 feedback_decode 的参数类型才能通过检查（类型收窄）。
                assert motor is not None
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

    rows = run(
        args.servo_port, args.servo_id, args.can_port, args.motor_id, args.seconds
    )

    if args.out:
        # with 是「上下文管理器」：进入时打开文件，离开时（包括异常）自动 close。
        # 等价于 try/finally，但不会忘记关闭。encoding 明确写 utf-8，避免中文环境乱码。
        with open(args.out, "w", encoding="utf-8") as f:
            f.write("t,prad,pos\n")
            # enumerate 同时给「下标 i」和「元素 (p, q)」；i 用来当时间列。
            for i, (p, q) in enumerate(rows):
                f.write(f"{i},{p},{q:.6f}\n")

    print(json.dumps(analyze(rows), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
