# 无 C 板：舵机 → PC → 达妙 USB2CAN → 电机

> 用于先验证通信链路，C 板后加。
> 前提：**达妙官方 USB2CANFD 模块**（USB VID:PID = `34b7:6877`），macOS。

## 0. 为什么需要 Python 3.10

达妙官方模块的底层协议**不开放**，只能用官方 SDK。官方 macOS 的 `usb_class` 只提供
**cpython-3.10** 的二进制（universal，x86_64 + arm64），所以：

```bash
conda create -n dm310 python=3.10 -y
~/miniconda3/envs/dm310/bin/pip install pyusb paho-mqtt pyserial
```

仓库里已经 vendored 官方 SDK：`pc/vendor/dm_sdk/`（`damiao.py` + `src/usb_class...so` + `dev_sn.py`）。

## 1. 查模块 SN

插上 USB2CANFD：

```bash
~/miniconda3/envs/dm310/bin/python pc/vendor/dm_sdk/dev_sn.py
```

期望输出 `VID: 0x34b7  PID: 0x6877  SN: xxxxx`。

- 没输出：模块不是 FD 版（可能是串口版），或没插好；
  用 `system_profiler SPUSBDataType | grep -i -A6 34b7` 确认。

## 2. 接线

```
USB2CANFD ── CAN_H ── 达妙 CAN_H
          └─ CAN_L ── 达妙 CAN_L
两端并 120 Ω；电机 24 V；CAN 与 C 板/PC 共地
```

电机 ID、CAN 波特率（1 Mbps）先用达妙上位机确认（Windows）。

## 3. 先单独验证电机（不走 MQTT）

```bash
cd ~/Embedded/custom_controller_ai/pc
~/miniconda3/envs/dm310/bin/python dm_sdk_bridge.py \
    --sn <SN> --id 1 --motor 4340 --test --kp 2 --kd 0.2 --range 0.5
```

期望：电机正弦摆动；终端每 0.5 s 打印 `target/pos/vel/tau`。

> SDK 的 `enable_all` 默认切到 VEL 模式；脚本随后显式切到 MIT 再使能。
> 如果电机不动，先看 SDK 打印的 `mode is ...` 是不是 1（MIT）。

## 4. 接舵机：舵机 → PC → MQTT → USB2CAN → 电机

三个终端：

```bash
cd ~/Embedded/custom_controller_ai/pc

# 1) broker
python3 -c "from mock_broker import MockBroker; import time; b=MockBroker('127.0.0.1',3333); time.sleep(600)"

# 2) 终端（舵机，27 Hz）
python3 terminal_app.py --source servo --port /dev/tty.usbserial-YYYY \
    --ids 1 --host 127.0.0.1 --mqtt-port 3333 --id 1 --verbose

# 3) 桥（Python 3.10）
~/miniconda3/envs/dm310/bin/python dm_sdk_bridge.py \
    --sn <SN> --id 1 --motor 4340 --kp 2 --kd 0.2 \
    --mqtt-host 127.0.0.1 --mqtt-port 3333 --client-id 1
```

期望：手扳舵机，电机跟着动；桥打印 `pos/vel/tau`；终端 `down` 递增（遥测回传）。

## 5. PID 说明

这个桥**不带 PID**：只用 MIT 的 Kp/Kd（电机内部位置环）。
按你的计划，PID 用现有框架在 C 板上验证；这个阶段只验证「舵机 → PC → CAN → 电机」链路通不通。

## 6. 常见问题

| 现象 | 查 |
|---|---|
| `dev_sn.py` 无输出 | 模块不是 USB2CANFD（34b7:6877）或没插好 |
| SDK import 报 `usb_class` 找不到 | 没用 dm310（3.10）环境 |
| 电机不动 | 电机 ID/波特率不对；120 Ω 缺失；SDK 模式不是 MIT |
| 手扳舵机电机不动 | 终端 `pub` 是否增长；桥的 `--mqtt-port` 是否一致 |
| 电机方向反 | `s_dir` 不在这里；桥里改 `pwm_to_rad` 的符号 |
