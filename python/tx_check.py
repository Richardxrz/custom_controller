#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tx_uart_it 模块验收 —— 「上位机收到的字节数对齐」

MCU 端（当前接线）：
    App_SetUp() → Test_TxIt_SetUp()   （main.c USER CODE 2）
    App_Loop()  → Test_TxIt_Run()     （main.c while 里）

测试意图（test_tx.c 里写的）：每次发 4 字节的固定模式 00 01 02 03

本脚本只干一件事：**把串口上收到的字节按块切开、数清楚、报事实**。
它不判断原因，也不给修法 —— 只输出「哪条满足 / 哪条不满足 + 原始数字」。

判据（4 条，全过才算过）：
    A 有数据    总字节 > 0
    B 无撕裂    总字节 % 块大小 == 0        （出现半个块 = 长度/时序错位）
    C 块内容    每一块都等于期望模式（默认 00 01 02 03）
    D 连续      块数 >= 2，且相邻两批数据之间最大空档 < --max-gap

（块数 = 0 时 B / C 记「不适用」，不当作通过。）

用法：
    .venv/bin/python python/tx_check.py                      # 采 3 秒
    .venv/bin/python python/tx_check.py --seconds 5
    .venv/bin/python python/tx_check.py --block 128 --expect 0001...   # 改块大小/期望模式
    .venv/bin/python python/tx_check.py --save /tmp/tx.raw   # 存原始字节，方便自己看

注意：
    · 跑之前先关掉 VOFA+（串口是独占的）
    · 插拔过 USB-TTL 后设备名会变，脚本会自动找 /dev/cu.usbserial-*
"""

import argparse
import glob
import time

import serial


def find_port():
    for p in sorted(glob.glob('/dev/cu.usbserial-*')):
        return p
    return None


def capture(ser, seconds):
    """连续读 seconds 秒；返回 (全部字节, [(累计字节数, 时刻), ...])"""
    t0 = time.time()
    buf = bytearray()
    marks = []
    while time.time() - t0 < seconds:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
            marks.append((len(buf), time.time() - t0))
    return bytes(buf), marks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default=None, help='默认自动找 /dev/cu.usbserial-*')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--seconds', type=float, default=3.0, help='采样秒数（默认 3）')
    ap.add_argument('--block', type=int, default=4, help='块大小（默认 4 = 测试里的模式长度）')
    ap.add_argument('--expect', default='00010203', help='期望的块内容（hex，默认 00010203）')
    ap.add_argument('--max-gap', type=float, default=0.5, dest='max_gap',
                    help='允许的最大空档秒数（默认 0.5）')
    ap.add_argument('--save', default=None, help='把原始字节存到这个文件')
    args = ap.parse_args()

    expect = bytes.fromhex(args.expect)
    if len(expect) != args.block:
        raise SystemExit(f'--expect 长度 {len(expect)} 与 --block {args.block} 不一致')

    port = args.port or find_port()
    if not port:
        raise SystemExit('找不到 /dev/cu.usbserial-*，USB-TTL 插好了吗？')

    print(f'端口   {port} @ {args.baud} 8N1')
    print(f'采样   {args.seconds} s ｜ 块大小 {args.block} ｜ 期望块 {args.expect}')
    print('-' * 62)

    ser = serial.Serial(port, args.baud, timeout=0.05)
    ser.reset_input_buffer()
    time.sleep(0.1)
    data, marks = capture(ser, args.seconds)
    ser.close()

    if args.save:
        with open(args.save, 'wb') as f:
            f.write(data)
        print(f'原始字节已存到 {args.save}（{len(data)} B）')

    # ---------------- 只报事实 ----------------
    total = len(data)
    blocks = total // args.block
    rest = total % args.block

    print(f'总字节 {total} ｜ 完整块 {blocks} ｜ 残余字节 {rest}')

    if total:
        first = data[:min(64, total)]
        print(f'开头 64 B: {first.hex(" ")}')
        if args.save is None:
            print(f'（想自己看全部：加 --save /tmp/tx.raw）')

    # A 有数据
    a = total > 0
    # B 无撕裂
    b = None if total == 0 else (total % args.block == 0)
    # C 块内容
    if blocks:
        bad = [i for i in range(blocks)
               if data[i * args.block:(i + 1) * args.block] != expect]
        c = (len(bad) == 0)
    else:
        bad = []
        c = None
    # D 连续：块数 >=2 且最大空档 < max_gap
    if len(marks) >= 2:
        gaps = [marks[i + 1][1] - marks[i][1] for i in range(len(marks) - 1)]
        max_gap = max(gaps)
        span = marks[-1][1] - marks[0][1]
        rate = (marks[-1][0] - marks[0][0]) / span if span > 0 else 0.0
        first_at = marks[0][1]
    else:
        max_gap = None
        rate = 0.0
        first_at = None
    d = (blocks >= 2) and (max_gap is not None) and (max_gap < args.max_gap)

    if rate:
        print(f'速率   ≈ {rate:,.0f} B/s ｜ ≈ {rate / args.block:,.0f} 块/s')
    if max_gap is not None:
        print(f'首字节 {first_at * 1000:.0f} ms ｜ 相邻两批之间最大空档 {max_gap * 1000:.1f} ms')
    if bad:
        print(f'内容不符的块：共 {len(bad)} 个，前几个下标 {bad[:8]}')
        for i in bad[:3]:
            seg = data[i * args.block:(i + 1) * args.block]
            print(f'  块 #{i}: {seg.hex(" ")}  ≠ {expect.hex(" ")}')

    print('-' * 62)

    def line(tag, name, ok, extra=''):
        mark = '—' if ok is None else ('✅' if ok else '❌')
        print(f'{mark} {tag} {name}{("  " + extra) if extra else ""}')

    line('A', '有数据', a, f'总字节 {total}')
    line('B', '无撕裂', b, f'总字节 % {args.block} = {rest}')
    line('C', '块内容', c, f'不符 {len(bad)} 块 / 共 {blocks} 块' if blocks else '块数 0，不适用')
    line('D', '连续', d,
         f'最大空档 {max_gap * 1000:.1f} ms' if max_gap is not None else '批次不足')

    passed = (a and b and (c is not False) and d)
    print('-' * 62)
    print('结论：' + ('全部满足' if passed else '有判据不满足'))
    print('（本脚本只报事实；原因和修法由你自己定位）')

    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
