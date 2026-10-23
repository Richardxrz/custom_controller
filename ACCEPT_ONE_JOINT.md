# 单舵机 → 单电机 全链路验收

> 目标：手扳 1 个 ZP10S，机器人端 1 个达妙电机跟着动。
> 链路：舵机 → ZL 板 USB → PC（MQTT）→ 桥 → USB-TTL → 臂控 C 板 → CAN → 达妙。

## 0. 需要的东西

| 件 | 数量 | 说明 |
|---|---|---|
| ZP10S（ID 1） | 1 | 已设开机释力 |
| ZL USB 转单总线板 | 2 | 1 块接舵机；1 块当 USB-TTL（或你有独立 USB-TTL） |
| 臂控 C 板 | 1 | 烧 `build/arm/stm32_module_lab.elf` |
| 达妙电机 | 1 | ID=1、CAN 1 Mbps、24 V |
| 120 Ω 电阻 | 1 | 并在 CAN H/L 两端 |
| 6 V / 24 V 电源 | 各 1 | 舵机 6 V；C 板 + 电机 24 V，**共地** |

## 1. 接线

```
舵机（ID1）──白色总线口── ZL板① ──USB── Mac

C 板 UART1（4-pin）          ZL板② 黄色排针
   pin2 TXD  ──────────────→  RX
   pin1 RXD  ←──────────────  TX
   pin3 GND  ───────────────  GND
   pin4 5V   ✗ 不接
                              USB ── Mac
（ZL板② 的白色总线口空着）

C 板 CAN1（2-pin）           达妙
   红 CANH  ────────────────  CAN_H
   黑 CANL  ────────────────  CAN_L
   两端并 120 Ω
C 板 24V 输入 ← 24 V；电机 24 V；两边 GND 连一起
```

> ⚠️ 达妙反馈帧的 ID/温度位域还没对官方手册核；先**空载**、小 Kp 试。

## 2. 烧固件

```bash
cd ~/Embedded/custom_controller_ai
pyocd flash -t stm32f407ighx build/arm/stm32_module_lab.elf
```

固件行为：**收到第一条有效 0x0311 才使能电机**；上电时电机是松的。

## 3. 开三个终端

```bash
# 终端 1：broker
cd ~/Embedded/custom_controller_ai/pc
python3 -c "from mock_broker import MockBroker; import time; b=MockBroker('127.0.0.1',3333); print('broker', b.port); time.sleep(600)"

# 终端 2：MQTT ↔ 0x0311 桥（ZL板② 的口）
python3 uart_0311_bridge.py --mqtt-host 127.0.0.1 --mqtt-port 3333 --id 1 \
    --port /dev/tty.usbserial-XXXX --baud 921600

# 终端 3：终端应用（ZL板① 的口）
python3 terminal_app.py --source servo --port /dev/tty.usbserial-YYYY \
    --ids 1 --host 127.0.0.1 --mqtt-port 3333 --id 1 --verbose
```

## 4. 期望现象

| 位置 | 现象 |
|---|---|
| `terminal_app` | `serial` 每秒 +27 左右；`pub` 每秒 +50；手扳舵机时 `joints` 变化 |
| `uart_0311_bridge` | `sent` 每秒 +27 左右；`telem` 每秒 +50；右侧 6 个角度 |
| 电机 | 手扳舵机，电机跟着转（PWM 1500 对应 0 rad，500/2500 对应 ±1.5 rad） |
| 第一条消息到达时 | 电机会使能并缓慢走到 0 rad（斜率 4 rad/s） |

## 5. 安全

- 电机空载、24 V 限流 2 A；
- 急停：拔 24 V，或 Ctrl-C 终端 → 300 ms 后臂控保持当前位置；
- 电机会在第一条指令后往 0 位走，确认周围没有夹手/限位再上电。

## 6. 不动时按顺序查

| 现象 | 查 |
|---|---|
| bridge `telem=0` | C 板 UART1 的 TX/RX 是否接反；波特率 921600；烧的是不是 arm 固件 |
| bridge `sent=0` | terminal_app 有没有在发布（`pub` 增长）；MQTT 端口是否一致 |
| 电机不动 | CAN H/L 是否接反；120 Ω 在不在；电机 ID 是否 1；电机是否 24 V；达妙是否 MIT 模式/1 Mbps |
| 电机一上电就冲 | 固件应等第一条 0x0311 才使能；若仍冲，检查是不是烧了旧固件 |
| 电机方向反 | 改 `App/arm_app.c` 的 `s_dir[]` |
