#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
状态机验收脚本

MCU 端约定（JustFloat 4 通道 + tail）：
    ch0 = 当前状态编号 (0=S_HUNT 1=S_MAGIC2 2=S_LEN 3=S_DATA 4=S_SUM)
    ch1 = 有效帧数
    ch2 = 校验失败数
    ch3 = 最后一帧的 LEN

用法：
    python proto_test.py
    python proto_test.py --port /dev/cu.usbserial-1120
"""

import argparse
import glob
import struct
import time

import serial

TAIL = bytes([0x00, 0x00, 0x80, 0x7F])
FRAME_LEN = 20                      # 4 float + 4 tail
STATE_NAMES = ['S_HUNT', 'S_MAGIC2', 'S_LEN', 'S_DATA', 'S_SUM']


def find_port():
    for p in sorted(glob.glob('/dev/cu.usbserial-*')):
        return p
    return None


def drain(ser, seconds):
    """丢掉这段时间内积压的数据，返回最后一帧的 4 个通道值"""
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < seconds:
        buf += ser.read(8192)

    last = None
    i = 0
    seq = []
    while True:
        j = buf.find(TAIL, i)
        if j < 0:
            break
        if j >= 16:
            ch = struct.unpack('<4f', bytes(buf[j - 16:j]))
            seq.append(ch)
            last = ch
        i = j + 4

    # 把状态序列压成跳变序列（去掉连续重复）
    jumps = []
    for ch in seq:
        s = int(round(ch[0]))
        if not jumps or jumps[-1] != s:
            jumps.append(s)
    return last, jumps


def show(tag, last, jumps):
    if last is None:
        print(f'  {tag}: [!] 没收到任何帧')
        return
    st, ok, err, ln = int(round(last[0])), last[1], last[2], last[3]
    name = STATE_NAMES[st] if 0 <= st < len(STATE_NAMES) else f'?{st}'
    jn = ' -> '.join(STATE_NAMES[s] if 0 <= s < 5 else f'?{s}' for s in jumps)
    print(f'  {tag}:')
    print(f'      当前状态 = {st} ({name})')
    print(f'      有效帧 = {int(ok)}   校验失败 = {int(err)}   最后 LEN = {int(ln)}')
    print(f'      状态跳变 = {jn}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default=None)
    ap.add_argument('--baud', type=int, default=115200)
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        raise SystemExit('找不到 /dev/cu.usbserial-*，USB-TTL 插好了吗？')

    print('=' * 62)
    print(f' 状态机验收  ({port})')
    print('=' * 62)

    ser = serial.Serial(port, args.baud, timeout=0.1)
    try:
        ser.reset_input_buffer()
        time.sleep(0.3)
        last, jumps = drain(ser, 0.3)
        show('基线（还没发数据）', last, jumps)

        # ---- 测试 1：合法帧 ----
        # LEN=3, DATA=11 22 33, SUM = 03+11+22+33 = 0x69
        f1 = bytes([0xAA, 0x55, 0x03, 0x11, 0x22, 0x33, 0x69])
        print(f'\n[测试1] 发合法帧  {f1.hex(" ")}')
        ser.reset_input_buffer()
        ser.write(f1); ser.flush()
        time.sleep(0.4)
        show('  期望 有效帧 +1，跳变 0->1->2->3->3->3->4->0', *drain(ser, 0.3))

        # ---- 测试 2：坏校验 ----
        f2 = bytes([0xAA, 0x55, 0x03, 0x11, 0x22, 0x33, 0x00])
        print(f'\n[测试2] 发坏校验帧 {f2.hex(" ")}')
        ser.reset_input_buffer()
        ser.write(f2); ser.flush()
        time.sleep(0.4)
        show('  期望 校验失败 +1，有效帧不变', *drain(ser, 0.3))

        # ---- 测试 3：垃圾字节 ----
        f3 = bytes([0xFF, 0xFF, 0x00, 0x7E])
        print(f'\n[测试3] 发垃圾字节 {f3.hex(" ")}')
        ser.reset_input_buffer()
        ser.write(f3); ser.flush()
        time.sleep(0.4)
        show('  期望 停在 S_HUNT，两个计数都不变', *drain(ser, 0.3))

        # ---- 测试 4：假帧头（AA 后面不是 55）----
        f4 = bytes([0xAA, 0x00, 0xAA, 0x55, 0x01, 0x77, 0x78])
        print(f'\n[测试4] 发 AA 00 AA 55 01 77 78（第一个 AA 是假的）')
        print('        期望 只识别第二个 AA，LEN=1，SUM=01+77=0x78')
        ser.reset_input_buffer()
        ser.write(f4); ser.flush()
        time.sleep(0.4)
        show('  期望 有效帧 +1，最后 LEN = 1', *drain(ser, 0.3))

        # ---- 测试 5：半截帧 ★ 最能说明"状态机在记住进度" ----
        print('\n[测试5] 发半截帧 AA 55 05 11 22（后面【不发了】）')
        print('        期望 ch0 停在 S_DATA(3) 一动不动')
        ser.reset_input_buffer()
        ser.write(bytes([0xAA, 0x55, 0x05, 0x11, 0x22])); ser.flush()
        time.sleep(0.5)
        l1, j1 = drain(ser, 0.3)
        show('  半截帧之后', l1, j1)
        time.sleep(0.5)
        l2, j2 = drain(ser, 0.3)
        show('  再等 0.5 秒', l2, j2)
        if l1 is not None and l2 is not None and int(round(l1[0])) == int(round(l2[0])):
            print(f'      ✅ 状态卡在 {STATE_NAMES[int(round(l1[0]))]} 没动 —— 状态机记住了进度')
        else:
            print('      ❌ 状态变了，不符合预期')

        # 补完这一帧：还差 3 个数据字节 + SUM
        # SUM = LEN + sum(DATA) = 5 + 0x11+0x22+0x33+0x44+0x55 = 5 + 255 = 260 -> 0x04
        print('\n[测试6] 把这一帧补完：发 33 44 55 04')
        ser.reset_input_buffer()
        ser.write(bytes([0x33, 0x44, 0x55, 0x04])); ser.flush()
        time.sleep(0.4)
        show('  期望 有效帧+1，LEN=5，状态回到 S_HUNT(0)', *drain(ser, 0.3))

    finally:
        ser.close()

    print('\n' + '=' * 62)
    print(' 判据：')
    print('   测试1 -> 有效帧+1')
    print('   测试2 -> 校验失败+1，有效帧不变')
    print('   测试3 -> 两个计数都不变')
    print('   测试4 -> 有效帧+1 且 最后 LEN=1')
    print('   测试5 -> ch0 卡在 S_DATA(3) 不动  ★核心')
    print('   测试6 -> 补完后有效帧+1、LEN=5、回到 S_HUNT')
    print('=' * 62)


if __name__ == '__main__':
    main()
