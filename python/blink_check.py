#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A-1 验收：不丢帧计数器

MCU 端约定（JustFloat 4 通道 + 尾巴 00 00 80 7F）：

    ch0 = 累计合法帧
    ch1 = 上一帧长度      （一帧都没收到时 = 0）
    ch2 = 历史最长帧长度
    ch3 = 校验失败累计     （只算校验和失败）

帧协议：
    [0xAA][0x55][LEN][DATA x LEN][SUM]      LEN = 1..16
    SUM = (LEN + sum(DATA)) & 0xFF

这个脚本检查什么：
    1  发数据前先读一次基线（ch0..ch3）
    2  10ms 间隔连发 10 帧  →  ch0 应该 +10      ★核心：一帧都不能丢
    3  再连发 10 帧         →  ch0 应该累计 +20
    4  发一个坏校验帧       →  ch3 应该 +1，ch0 不变
    5  半帧 → 等 250ms（超阈值）→ 再发一个完整帧
                            →  ch3 应该 +1（残帧被超时清掉），ch0 应该 +1（新帧正常解析）
       注：超时是"懒检测"，只在回调里检查 —— 发完半帧什么都不发，是测不出超时的

    LED 是肉眼看，脚本会把「此刻应该显示什么」打出来。

用法：
    .venv/bin/python python/blink_check.py
    .venv/bin/python python/blink_check.py --port /dev/cu.usbserial-1120
"""

import argparse
import glob
import random
import struct
import time

import serial

TAIL = bytes([0x00, 0x00, 0x80, 0x7F])
FRAME_GAP = 0.010           # 10ms
REPORT_WAIT = 0.45          # 上报周期 200ms，等 0.45s 保证拿到新的一帧


def find_port():
    for p in sorted(glob.glob('/dev/cu.usbserial-*')):
        return p
    return None


def make_frame(data):
    ln = len(data)
    return bytes([0xAA, 0x55, ln]) + bytes(data) + bytes([(ln + sum(data)) & 0xFF])


def make_bad_frame(data):
    """合法帧，但把最后一个字节（SUM）改坏"""
    f = bytearray(make_frame(data))
    f[-1] ^= 0xFF
    return bytes(f)


def read_last(ser, seconds):
    """读 seconds 秒，返回最后一帧 JustFloat 的 4 个通道；一帧都没有则 None"""
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < seconds:
        buf += ser.read(8192)

    last = None
    i = 0
    while True:
        j = buf.find(TAIL, i)
        if j < 0:
            break
        if j >= 16:
            last = struct.unpack('<4f', bytes(buf[j - 16:j]))
        i = j + 4
    return last


def show(tag, ch):
    if ch is None:
        print(f'  {tag}: [!] 没收到任何上报帧')
        return
    c = [int(round(x)) for x in ch]
    led = c[0] & 0x7
    print(f'  {tag}: ch0={c[0]:<5} ch1={c[1]:<4} ch2={c[2]:<4} ch3={c[3]:<4}'
          f'   | LED 应显示 {led} ({led:03b})')


def send_frames(ser, lens, gap=FRAME_GAP):
    """按固定帧间隔连发；返回实际测到的间隔"""
    intervals = []
    prev = None
    for ln in lens:
        data = bytes(random.randint(0, 255) for _ in range(ln))
        f = make_frame(data)
        t_start = time.time()
        if prev is not None:
            intervals.append(t_start - prev)
        prev = t_start
        ser.write(f)
        ser.flush()
        rest = gap - (time.time() - t_start)
        if rest > 0:
            time.sleep(rest)
    return intervals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default=None)
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--seed', type=int, default=20260927)
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        raise SystemExit('找不到 /dev/cu.usbserial-*，USB-TTL 插好了吗？')

    random.seed(args.seed)

    print('=' * 66)
    print(f' A-1 验收：不丢帧计数器  ({port})')
    print('=' * 66)

    ser = serial.Serial(port, args.baud, timeout=0.1)
    results = []

    try:
        # ---------- 基线 ----------
        print('\n[基线] 还没发任何数据')
        ser.reset_input_buffer()
        time.sleep(0.3)
        base = read_last(ser, REPORT_WAIT)
        show('  基线', base)
        if base is None:
            raise SystemExit('  ❌ 收不到上报帧 —— 上报没实现？VOFA 还连着？')
        b0, b3 = int(round(base[0])), int(round(base[3]))

        # ---------- 测试 1：10ms 间隔连发 10 帧 ----------
        print('\n[测试1] 10ms 间隔连发 10 帧（长度随机 1~16）')
        ser.reset_input_buffer()
        lens = [random.randint(1, 16) for _ in range(10)]
        print(f'        帧长 = {lens}')
        iv = send_frames(ser, lens)
        print(f'        实际帧间隔 = {min(iv)*1000:.1f} ~ {max(iv)*1000:.1f} ms')
        time.sleep(REPORT_WAIT)
        ch = read_last(ser, REPORT_WAIT)
        show('  收', ch)
        d0 = int(round(ch[0])) - b0 if ch else 0
        ok = (d0 == 10)
        results.append(ok)
        print(f'    判据: ch0 增加 {d0}（期望 10）  {"✅" if ok else "❌"}')
        if not ok:
            print('        → 期望 10，实际不是 10')

        # ---------- 测试 2：再来 10 帧，累计 20 ----------
        print('\n[测试2] 再连发 10 帧（累计 20）')
        ser.reset_input_buffer()
        lens2 = [random.randint(1, 16) for _ in range(10)]
        print(f'        帧长 = {lens2}')
        send_frames(ser, lens2)
        time.sleep(REPORT_WAIT)
        ch2 = read_last(ser, REPORT_WAIT)
        show('  收', ch2)
        d0 = int(round(ch2[0])) - b0 if ch2 else 0
        ok = (d0 == 20)
        results.append(ok)
        print(f'    判据: ch0 累计增加 {d0}（期望 20）  {"✅" if ok else "❌"}')
        if ok:
            led = int(round(ch2[0])) & 0x7
            print(f'    肉眼看 LED：此刻应该显示 {led}（二进制 {led:03b}）')

        # ---------- 测试 3：坏校验帧 ----------
        print('\n[测试3] 发一个坏校验帧（ch3 应 +1，ch0 不变）')
        ser.reset_input_buffer()
        c0_before = int(round(ch2[0])) if ch2 else b0
        c3_before = int(round(ch2[3])) if ch2 else b3
        bad = make_bad_frame([0x11, 0x22, 0x33])
        print(f'        发: {bad.hex(" ")}')
        ser.write(bad); ser.flush()
        time.sleep(REPORT_WAIT)
        ch3 = read_last(ser, REPORT_WAIT)
        show('  收', ch3)
        dd0 = int(round(ch3[0])) - c0_before if ch3 else 0
        dd3 = int(round(ch3[3])) - c3_before if ch3 else 0
        ok = (dd3 == 1 and dd0 == 0)
        results.append(ok)
        print(f'    判据: ch3 增加 {dd3}（期望 1），ch0 增加 {dd0}（期望 0）'
              f'  {"✅" if ok else "❌"}')

        # ---------- 测试 4：半帧 → 等超阈值 → 再发一个完整帧 ----------
        print('\n[测试4] 发半帧 → 等 250ms（超阈值）→ 再发一个完整帧')
        ser.reset_input_buffer()
        c0_before = int(round(ch3[0])) if ch3 else c0_before
        c3_before = int(round(ch3[3])) if ch3 else c3_before
        half = bytes([0xAA, 0x55, 0x05, 0x11, 0x22])   # LEN=5，只给 2 个数据
        print(f'        发半帧  : {half.hex(" ")}')
        ser.write(half)
        ser.flush()
        time.sleep(0.25)                                # 超过 100ms 阈值
        nxt = make_frame([0x77])
        print(f'        再发完整: {nxt.hex(" ")}')
        ser.write(nxt)
        ser.flush()
        time.sleep(REPORT_WAIT)
        ch4 = read_last(ser, REPORT_WAIT)
        show('  收', ch4)
        dd0 = int(round(ch4[0])) - c0_before if ch4 else 0
        dd3 = int(round(ch4[3])) - c3_before if ch4 else 0
        ok = (dd0 == 1 and dd3 == 1)
        results.append(ok)
        print(f'    判据: ch0 增加 {dd0}（期望 1：新帧正常解析），'
              f'ch3 增加 {dd3}（期望 1：残帧被超时清掉）  {"✅" if ok else "❌"}')

    finally:
        ser.close()

    n_ok = sum(1 for r in results if r)
    print('\n' + '=' * 66)
    print(f' {n_ok}/{len(results)} 通过')
    print('=' * 66)
    for i, r in enumerate(results, 1):
        print(f'   测试{i}: {"✅" if r else "❌"}')


if __name__ == '__main__':
    main()
