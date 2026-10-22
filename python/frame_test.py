#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rx_dma 模块验收

协议： [0xAA][0x55][LEN][DATA x LEN][SUM]      SUM = (LEN + sum(DATA)) & 0xFF

MCU 端行为（main.c 里）：
    收到一帧 → 闪蓝灯 + 把 payload 原样回显

用法：
    python frame_test.py
    python frame_test.py --port /dev/cu.usbserial-1120
"""

import argparse
import glob
import time

import serial


def find_port():
    for p in sorted(glob.glob('/dev/cu.usbserial-*')):
        return p
    return None


def make_frame(data):
    """按协议拼一帧"""
    ln = len(data)
    s = (ln + sum(data)) & 0xFF
    return bytes([0xAA, 0x55, ln]) + bytes(data) + bytes([s])


def drain(ser, seconds):
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < seconds:
        buf += ser.read(4096)
    return bytes(buf)


def case(ser, tag, payload, expect_echo):
    f = make_frame(payload)
    ser.reset_input_buffer()
    time.sleep(0.05)
    ser.write(f)
    ser.flush()
    time.sleep(0.25)
    got = drain(ser, 0.25)

    ok = (got == bytes(payload)) if expect_echo else (len(got) == 0)
    mark = '✅' if ok else '❌'

    print(f'\n{mark} {tag}')
    print(f'    发  : {f.hex(" ")}')
    print(f'    收  : {got.hex(" ") if got else "(无)"}   (期望: {bytes(payload).hex(" ") if expect_echo else "无"})')
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default=None)
    ap.add_argument('--baud', type=int, default=115200)
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        raise SystemExit('找不到 /dev/cu.usbserial-*，USB-TTL 插好了吗？')

    print('=' * 60)
    print(f' rx_dma 验收  ({port})')
    print('=' * 60)

    ser = serial.Serial(port, args.baud, timeout=0.1)
    results = []
    try:
        ser.reset_input_buffer()
        time.sleep(0.3)

        results.append(case(ser, '测试1  3 字节 payload', [0x11, 0x22, 0x33], True))
        results.append(case(ser, '测试2  1 字节 payload', [0x01], True))
        results.append(case(ser, '测试3  16 字节 payload（LEN 上限）', list(range(0x10, 0x20)), True))

        # 坏校验：把 SUM 改掉
        bad = bytearray(make_frame([0xAA, 0xBB]))
        bad[-1] ^= 0xFF
        ser.reset_input_buffer()
        time.sleep(0.05)
        ser.write(bytes(bad))
        ser.flush()
        time.sleep(0.25)
        got = drain(ser, 0.25)
        ok = (len(got) == 0)
        results.append(ok)
        print(f'\n{"✅" if ok else "❌"} 测试4  坏校验（SUM 改掉）')
        print(f'    发  : {bytes(bad).hex(" ")}')
        print(f'    收  : {got.hex(" ") if got else "(无)"}   (期望: 无 —— 这一帧应该被丢弃)')

        # 连发两帧，应该回显两次
        f1, f2 = make_frame([0x0A, 0x0B]), make_frame([0x0C])
        ser.reset_input_buffer()
        time.sleep(0.05)
        ser.write(f1)
        ser.flush()
        time.sleep(0.3)
        ser.write(f2)
        ser.flush()
        time.sleep(0.3)
        got = drain(ser, 0.3)
        ok = (got == bytes([0x0A, 0x0B, 0x0C]))
        results.append(ok)
        print(f'\n{"✅" if ok else "❌"} 测试5  连发两帧')
        print(f'    发  : {f1.hex(" ")}  然后  {f2.hex(" ")}')
        print(f'    收  : {got.hex(" ") if got else "(无)"}   (期望: 0a 0b 0c)')

    finally:
        ser.close()

    n_ok = sum(1 for r in results if r)
    print('\n' + '=' * 60)
    print(f' {n_ok}/{len(results)} 通过')
    print('=' * 60)


if __name__ == '__main__':
    main()
