#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rx_dma 超时复位验收

协议： [0xAA][0x55][LEN][DATA x LEN][SUM]      SUM = (LEN + sum(DATA)) & 0xFF

测试设计：
  测试1  半截帧 → 等 150ms（超过超时）→ 发完整帧
         期望：完整帧正常回显
         如果超时没实现：半截帧残留在状态机里，新帧会被吃掉 → 收不到

  测试2  半截帧 → 等 30ms（未超时）→ 补完
         期望：分片发送也能正常收到（超时不能误伤）

  测试3  一帧拆成 6 片，每片间隔 45ms（每一片都远小于 100ms 超时阈值）
         期望：整帧完整收到
         这一条检查的是：连续多片、每片间隔都小于阈值时，一帧必须能拼完

  测试4  一次发 128 字节（一个有效帧 + 123 个 00），必然触发 HT(64) 和 TC(128)
         期望：那个帧只被回显「1 次」
         这一条检查的是：同一批数据不能被投喂两遍

  测试5  紧接着测试4，再发一个普通短帧
         期望：还能正常收到
         这一条检查的是：长突发之后接收链路是否还活着

用法：
    python timeout_test.py
    python timeout_test.py --port /dev/cu.usbserial-130
"""

import argparse
import glob
import time

import serial

SUM = lambda b: (b[0] + sum(b[1:])) & 0xFF


def make_frame(data):
    ln = len(data)
    return bytes([0xAA, 0x55, ln]) + bytes(data) + bytes([(ln + sum(data)) & 0xFF])


def find_port():
    for p in sorted(glob.glob('/dev/cu.usbserial-*')):
        return p
    return None


def drain(ser, seconds):
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < seconds:
        buf += ser.read(4096)
    return bytes(buf)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default=None)
    ap.add_argument('--baud', type=int, default=115200)
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        raise SystemExit('找不到 /dev/cu.usbserial-*，USB-TTL 插好了吗？')

    print('=' * 60)
    print(f' rx_dma 超时复位验收  ({port})')
    print('=' * 60)

    ser = serial.Serial(port, args.baud, timeout=0.1)
    results = []

    try:
        # ---------- 测试 1：超时后下一帧必须能收到 ----------
        print('\n[测试1] 半截帧 → 等 150ms → 完整帧')
        ser.reset_input_buffer()
        time.sleep(0.1)
        ser.write(bytes([0xAA, 0x55, 0x05, 0x11, 0x22]))   # 半截：LEN=5，只给 2 个数据
        ser.flush()
        print('        发半截: aa 55 05 11 22')
        time.sleep(0.15)                                    # 超过 100ms 超时
        f = make_frame([0x07, 0x08])
        ser.write(f)
        ser.flush()
        print(f'        发完整: {f.hex(" ")}')
        time.sleep(0.3)
        got = drain(ser, 0.3)
        ok = (got == bytes([0x07, 0x08]))
        results.append(ok)
        print(f'    收  : {got.hex(" ") if got else "(无)"}   (期望: 07 08)  {"✅" if ok else "❌"}')
        if not ok:
            print('        → 回显不是期望值' + ('（一个字节都没收到）' if not got else ''))

        # ---------- 测试 2：未超时的分片发送不能误伤 ----------
        print('\n[测试2] 半截帧 → 等 30ms（未超时）→ 补完')
        ser.reset_input_buffer()
        time.sleep(0.1)
        ser.write(bytes([0xAA, 0x55, 0x03, 0x11, 0x22]))
        ser.flush()
        print('        发前半: aa 55 03 11 22')
        time.sleep(0.03)                                    # 未超时
        ser.write(bytes([0x33, 0x69]))                      # 补完：33 + SUM(03+11+22+33=69)
        ser.flush()
        print('        发后半: 33 69')
        time.sleep(0.3)
        got = drain(ser, 0.3)
        ok = (got == bytes([0x11, 0x22, 0x33]))
        results.append(ok)
        print(f'    收  : {got.hex(" ") if got else "(无)"}   (期望: 11 22 33)  {"✅" if ok else "❌"}')
        if not ok:
            print('        → 回显不是期望值' + ('（一个字节都没收到）' if not got else ''))

        # ---------- 测试 3：连续多片，每片间隔都小于阈值 ----------
        print('\n[测试3] 一帧拆成 6 片，每片间隔 45ms（阈值 100ms）')
        ser.reset_input_buffer()
        time.sleep(0.1)
        pieces = [
            bytes([0xAA, 0x55]),   # 帧头
            bytes([0x08, 0x01]),   # LEN=8，数据1
            bytes([0x02, 0x03]),
            bytes([0x04, 0x05]),
            bytes([0x06, 0x07]),
            bytes([0x08, 0x2C]),   # 数据8，SUM
        ]
        for k, piece in enumerate(pieces):
            ser.write(piece)
            ser.flush()
            print(f'        片{k + 1}/6: {piece.hex(" ")}')
            if k < len(pieces) - 1:
                time.sleep(0.045)
        time.sleep(0.3)
        got = drain(ser, 0.3)
        ok = (got == bytes([1, 2, 3, 4, 5, 6, 7, 8]))
        results.append(ok)
        print(f'    收  : {got.hex(" ") if got else "(无)"}'
              f'   (期望: 01 02 03 04 05 06 07 08)  {"✅" if ok else "❌"}')
        if not ok:
            print('        → 回显不是期望值' + ('（一个字节都没收到）' if not got else ''))

        # ---------- 测试 4：一次发 128 字节（HT + TC 都会触发）----------
        print('\n[测试4] 一次发 128 字节（必然触发 HT=64 和 TC=128）')
        ser.reset_input_buffer()
        time.sleep(0.1)
        frame = bytes([0xAA, 0x55, 0x01, 0xA5, 0xA6])   # LEN=1，数据 0xA5，SUM=0xA6
        burst = frame + bytes(123)                       # 合计 128 字节，一次写完，中间不留空隙
        ser.write(burst)
        ser.flush()
        print(f'        发: {frame.hex(" ")} + 123 个 00（共 {len(burst)} 字节，中间无间隔）')
        time.sleep(0.4)
        got = drain(ser, 0.4)
        n = got.count(0xA5)
        ok = (n == 1)
        results.append(ok)
        print(f'    收  : {got.hex(" ") if got else "(无)"}'
              f'   (期望: 只有 1 个 a5)  {"✅" if ok else "❌"}')
        if not ok:
            print(f'        → 收到 {n} 个 a5（期望 1 个）')

        # ---------- 测试 5：探针 —— 经过上面的长突发之后，接收还活着吗 ----------
        print('\n[测试5] 长突发之后再发一个普通短帧（探针：接收链路还活着吗）')
        ser.reset_input_buffer()
        probe = bytes([0xAA, 0x55, 0x01, 0x11, 0x12])   # LEN=1，数据 0x11，SUM=0x12
        ser.write(probe)
        ser.flush()
        print(f'        发: {probe.hex(" ")}')
        time.sleep(0.3)
        got = drain(ser, 0.3)
        ok = (got == bytes([0x11]))
        results.append(ok)
        print(f'    收  : {got.hex(" ") if got else "(无)"}   (期望: 11)  {"✅" if ok else "❌"}')
        if not ok:
            print('        → 之后发什么都没反应了')

    finally:
        ser.close()

    n_ok = sum(1 for r in results if r)
    print('\n' + '=' * 60)
    print(f' {n_ok}/{len(results)} 通过')
    print('=' * 60)
    if n_ok == len(results):
        print('\n🎉 超时复位成功：既不残留，也不误伤，连续分片也能拼完。')


if __name__ == '__main__':
    main()
