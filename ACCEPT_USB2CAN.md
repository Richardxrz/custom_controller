# 无 C 板：舵机 → PC → 达妙经典 USB2CAN → 电机

> 用于先验证通信链路，C 板后加。
> 模块：**达妙经典 USB2CAN**（CDC 串口，VID:PID `2E88:4603`），不是 FD 版。

## 0. 为什么不用官方 FD SDK

经典 USB2CAN 是 **CDC 串口 + 自定义 30 字节帧**，协议来自达妙官方 Gitee
`kit-miao/usb-2-can`（`通讯协议/USB 转CAN 帧格式.xlsx` + 开源上位机 `can.cs`）。
本仓库已用纯 Python 实现：`pc/usb2can_classic.py`，**Python 3.14 直接跑，不需要 3.10 环境**。

帧格式（已用官方示例逐字节核对）：

```
发送 30 字节：
  55 AA 1E 01 | times(u32) | interval(u32,100us) | idType | CANID(u32)
  | frameType | dataLen | idAcc | dataAcc | data[8] | CRC(填0)

接收 16 字节：
  AA | CMD | fmt | CANID(u32) | data[8] | 55
  CMD: 0x11 接收成功 / 0x12 发送成功 / 0x02 发送失败
```

## 1. 接线

```
USB2CAN ── CAN_H ── 达妙 CAN_H
        └─ CAN_L ── 达妙 CAN_L
CAN_H/CAN_L 两端并 120 Ω
电机 24 V；USB2CAN 与电机共地
```

电机 ID=1、CAN 1 Mbps（先用达妙 Windows 上位机确认）。

## 2. 先确认模块在说话

```bash
cd ~/Embedded/custom_controller_ai/pc
python3 - <<'PY'
import time
from usb2can_classic import Usb2CanClassic
bus = Usb2CanClassic('/dev/tty.usbmodem00000000050C1', 921600)
bus.open(1_000_000)
bus.send(1, bytes([0xFF]*7+[0xFD]))   # 失能帧，安全
t0=time.time()
while time.time()-t0 < 1:
    for f in bus.recv():
        print(hex(f['cmd']), hex(f['id']), f['data'].hex())
    time.sleep(0.01)
bus.close()
PY
```

- 返回 `0x12` / `0x11`：链路通；
- 返回 `0x02`：**发送失败 = 总线上没有 ACK**，查电机供电、CAN_H/L、120 Ω。
  （我第一次测就是 0x02，因为电机没接。）

## 3. 单独验证电机（不走舵机/MQTT）

```bash
python3 usb_can_damiao.py --transport dm_classic \
    --port /dev/tty.usbmodem00000000050C1 \
    --test --id 1 --motor 4340 --kp 1 --kd 0.2 --range 0.3
```

期望：电机正弦摆动，终端每 0.5 s 打印 `target/pos/vel/tau`。

## 4. 接舵机：舵机 → PC → MQTT → USB2CAN → 电机

```bash
# 1) broker
python3 -c "from mock_broker import MockBroker; import time; b=MockBroker('127.0.0.1',3333); time.sleep(600)"

# 2) 舵机终端
python3 terminal_app.py --source servo --port /dev/tty.usbserial-YYYY \
    --ids 1 --host 127.0.0.1 --mqtt-port 3333 --id 1 --verbose

# 3) USB2CAN 桥
python3 usb_can_damiao.py --transport dm_classic \
    --port /dev/tty.usbmodem00000000050C1 \
    --mqtt --id 1 --motor 4340 --kp 1 --kd 0.2 \
    --mqtt-host 127.0.0.1 --mqtt-port 3333
```

期望：手扳舵机，电机跟着动；终端 `down` 递增（遥测回传）。

## 5. PID 说明

这里只发 MIT（Kp/Kd 是电机内部位置环），**不带 PID**。
按你的计划，PID 用现有框架在 C 板上验证；这一步只验证链路。

## 6. 常见问题

| 现象 | 查 |
|---|---|
| 一直 `0x02` 发送失败 | 电机没供电 / CAN_H、CAN_L 接反 / 没有 120 Ω |
| `0x12` 有但电机不动 | 电机 ID/波特率不对；MIT 模式未使能 |
| 手扳舵机电机不动 | 终端 `pub` 是否增长；桥的 `--mqtt-port` 是否一致 |
| 电机方向反 | 改桥里 `pwm_to_rad` 的符号 |
