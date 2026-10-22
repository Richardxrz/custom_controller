#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口回环诊断 —— 判断"上位机收到的数据"到底是谁发的

原理：
    下位机（usart6_it_txrx）只会在「收满 4 字节」后回一个「36 字节的 frame」，
    其中包含 28 个 0x00 和尾巴 00 00 80 7f。
    所以：
      * 发 3 字节却收到 3 字节原样  -> MCU 的 RX 中断没触发，绝不可能是 MCU
      * 收到 36 字节且含大量 0x00   -> 这才是 MCU 发的

用法：
    python loopback_diag.py
    python loopback_diag.py --port /dev/cu.usbserial-1140 --baud 115200

跑之前请先在 VOFA+ 里点 Disconnect（串口是独占资源）。
"""

import argparse
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial —— 请用项目自带的 venv 运行：\n"
             "  .venv/bin/python python/loopback_diag.py\n"
             "（没建过 venv 就先跑：\n"
             "  python3 -m venv .venv && .venv/bin/python -m pip install -r requirements.txt ）")

DEFAULT_PORT = "/dev/cu.usbserial-1120"
DEFAULT_BAUD = 115200
MCU_TAIL = bytes([0x00, 0x00, 0x80, 0x7F])


# ---------------------------------------------------------------- helpers

def hexs(b):
    return b.hex(" ") if b else "(空)"


def who_holds(port):
    """macOS: 用 lsof 看谁占着串口"""
    try:
        r = subprocess.run(["lsof", port], capture_output=True, text=True, timeout=5)
        lines = r.stdout.strip().splitlines()
        return lines[1:] if len(lines) > 1 else []
    except Exception:
        return []


def collect(ser, seconds, tag):
    """在 seconds 秒内收集并打印收到的所有字节"""
    total = bytearray()
    t0 = time.time()
    while time.time() - t0 < seconds:
        chunk = ser.read(4096)
        if chunk:
            total += chunk
            print(f"      {tag} [+{time.time() - t0:7.4f}s] {len(chunk):3d} B  {hexs(chunk)}")
    return bytes(total)


def open_port(port, baud):
    try:
        return serial.Serial(port, baud, timeout=0.05)
    except serial.SerialException as e:
        print(f"\n[!] 打不开 {port}: {e}")
        h = who_holds(port)
        if h:
            print("[!] 当前占用该端口的进程：")
            for line in h:
                print("      " + line)
            print("[!] 请在 VOFA+ 里点 Disconnect，然后重跑本脚本。")
        sys.exit(1)


def judge(sent, got):
    """给出判定"""
    if not got:
        print("      判定: 没收到任何数据")
        return "quiet"
    if got == sent:
        print("      判定: ★★ 原样回显 ★★")
        print("            收到的和你发的一模一样（长度、字节全同）")
        print("            -> 绝不可能是 MCU：MCU 只会回 36 字节的 frame")
        print("            -> 是串口芯片/驱动/接线把你发的字节回灌到了 RX")
        return "echo"
    zeros = got.count(0x00)
    if (MCU_TAIL in got) or zeros >= 20:
        print(f"      判定: 疑似 MCU 响应（含 {zeros} 个 0x00 字节）")
        return "mcu"
    print("      判定: 收到了内容，但既不是原样回显，也不像 MCU frame")
    print("            -> 请人工核对上面的 hex")
    return "other"


# ---------------------------------------------------------------- tests

def test1_passive(ser):
    """什么也不发，只听。看线路上是否本来就有数据流动。"""
    print("\n" + "-" * 62)
    print(" 测试 1  被动监听 3 秒（一个字节都不发）")
    print("-" * 62)
    ser.reset_input_buffer()
    got = collect(ser, 3.0, "[被动]")
    if got:
        print(f"      线路上有 {len(got)} 字节在流动 —— 且不是你发的")
        print("      -> MCU 主动上报？还是别的程序也在读这个口？")
    else:
        print("      完全安静。说明：不发数据时，线路上什么都没有。")
    return got


def test2_send(ser, payload, seconds, title):
    """发一串数据，然后只听。"""
    print("\n" + "-" * 62)
    print(f" {title}")
    print("-" * 62)
    ser.reset_input_buffer()
    time.sleep(0.1)                       # 让线路稳定
    print(f"      [TX] {len(payload):3d} B  {hexs(payload)}")
    ser.write(payload)
    ser.flush()
    got = collect(ser, seconds, "[RX]")
    print(f"      [RX] 共 {len(got)} B")
    return got


def main():
    ap = argparse.ArgumentParser(description="串口回环诊断")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = ap.parse_args()

    print("=" * 62)
    print("  串口回环诊断")
    print("=" * 62)
    print(f"  端口    : {args.port}")
    print(f"  波特率  : {args.baud}")

    holders = who_holds(args.port)
    if holders:
        print("  [!] 该端口正被占用（可能同时被两个程序读，会互相抢数据）：")
        for line in holders:
            print("      " + line)
    else:
        print("  占用    : 无")

    ser = open_port(args.port, args.baud)
    try:
        # ---- 测试 1：不发只听
        test1_passive(ser)

        # ---- 测试 2：发 8 字节
        p2 = bytes([0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44])
        g2 = test2_send(ser, p2, 2.0, "测试 2  发 8 字节唯一图案")
        v2 = judge(p2, g2)

        # ---- 测试 3：发 3 字节（决定性！MCU 收不满 4 字节，不会触发中断）
        p3 = bytes([0xDE, 0xAD, 0xBE])
        g3 = test2_send(ser, p3, 2.0,
                        "测试 3  发 3 字节 <- 决定性实验（MCU 需要 4 字节才响应）")
        v3 = judge(p3, g3)

        # ---- 测试 4：发 4 字节（这次 MCU 应该真的响应）
        p4 = bytes([0x3F, 0x80, 0x00, 0x00])   # 大端 float 1.0，随便挑的唯一图案
        g4 = test2_send(ser, p4, 2.0, "测试 4  发 4 字节（MCU 应在此响应）")
        v4 = judge(p4, g4)

    finally:
        ser.close()

    # ---- 汇总
    print("\n" + "=" * 62)
    print("  结论")
    print("=" * 62)
    print(f"  测试2 发8字节 ->      : {v2}")
    print(f"  测试3 发3字节 ->      : {v3}   <-- 看这一行")
    print(f"  测试4 发4字节 ->      : {v4}")

    print()
    if v3 == "echo":
        print("  >>> 定案：这 3 个字节被原样回显了。")
        print("      MCU 的 RX 中断需要 4 字节，不可能产生这 3 个字节。")
        print("      => 问题在【串口链路本身】，不在你的 MCU 代码。")
        print("      排查方向：")
        print("        1. USB-TTL 模块的 TX 和 RX 是否被短接（跳线帽/焊桥）")
        print("        2. 开发板上 USART6 的 TX/RX 之间是否插了回环跳线帽")
        print("        3. 换一个 USB-TTL 模块再试")
    elif v3 == "quiet":
        print("  >>> 发 3 字节没有任何回显 -> 线路是干净的。")
        print("      那么你在 VOFA 里看到的\"重复数据\"就是【上位机自己显示的发送记录】。")
        print("      在 VOFA 的 Raw Data 面板里点【↓ 接收】过滤掉发送方向即可。")
    else:
        print("  >>> 结果不是典型的原样回显，请把上面完整输出贴出来分析。")
    print()


if __name__ == "__main__":
    main()
