# 验收演示：舵机 → C 板 → 串口 → PC 上位机

> 目标：手扳舵机，PC 窗口里 6 根条实时动。
> 两条路线，先做 A（不需要 C 板，5 分钟出画面），再做 B（走固件全链路）。

---

## 0. 先看串口

```bash
cd ~/Embedded/custom_controller_ai/pc
python3 viewer.py --list
```

- ZL 转接板（CH340）通常显示 `usbserial` / `wchusbserial`；
- USB-TTL 也是 `usbserial`；
- `usbmodem` 一般是调试器/CMSIS-DAP，不是这两个。

---

## A. 最快：PC 直连舵机（不需要 C 板）

**接线**

1. ZL 转接板 **USB 插 Mac**（黄色排针不接）。
2. 舵机 3 根线插白色总线口（看好 `- / + / S` 顺序）。
3. 舵机电源 6 V 并在总线 `+ / -` 上；**不要用 USB 给舵机供电**。

**先设「开机释力」**（ZP10S 实测固件 `ZL-ZServo_AD_CBM V2.1.16STG` **不支持手册里的 `PULK`**，
但支持持久配置 `PCSM`）：

```bash
python3 ~/ai/50-机械臂控制器/tools/servo_cli.py -p /dev/tty.usbserial-XXXX free 1
# 然后给舵机断电、重新上电
```

上电后舵机应处于自由状态（手能扳动）；`PRAD` 轮询不会重新加力，只有发运动指令才会。

**先确认 ID**（出厂是 0）：

```bash
# 扫描 0~5
python3 ~/ai/50-机械臂控制器/tools/servo_cli.py -p /dev/tty.usbserial-XXXX scan --ids 0-5

# 如果是 0，改成 1（总线上只留这一个舵机！）
python3 ~/ai/50-机械臂控制器/tools/servo_cli.py -p /dev/tty.usbserial-XXXX set-id 0 1
```

**开画面**

```bash
python3 viewer.py --source servo --port /dev/tty.usbserial-XXXX --ids 1
```

**期望**：弹出「自定义控制器 · 6 轴角度」窗口；J0 那根条随手扳动实时变化，
右上角显示 `1500 → 1234` 之类的 PWM 和角度；底部 `帧/s` 稳定。

> 6 个舵机都接上、ID 改成 1–6 后，`--ids 1,2,3,4,5,6`，6 根条一起动。

---

## B. 全链路：舵机 → C 板固件 → UART1 → PC

**额外需要**：一块 USB-TTL（CH340/CP2102 都行，¥10 左右）+ 杜邦线。

**接线（主端 C 板）**

```
舵机总线（白口）── ZL 转接板 ──黄色排针── C 板 UART6（3-pin）
                                  RX  ←  PG14 (UART6_TX)
                                  TX  →  PG9  (UART6_RX)
                                  GND ↔  GND
                                  5V0 先不接（没反应再试）

C 板 UART1（4-pin）── USB-TTL ── Mac
        pin1 RXD  ←  USB-TTL TX（可选）
        pin2 TXD  →  USB-TTL RX
        pin3 GND  ↔  GND
        pin4 5V   ✗ 不接
```

**烧主端固件**

```bash
cd ~/Embedded/custom_controller_ai
pyocd flash -t stm32f407ighx build/master/stm32_module_lab.elf
```

**开画面**

```bash
cd pc
python3 viewer.py --source master --port /dev/tty.usbserial-XXXX
```

**期望**

- 窗口底部 `帧/s ≈ 100`、`丢帧 0`；
- 手扳 1 号舵机，J0 实时变化；
- 只接 1 个舵机时，其他 5 根条不动是正常的（固件轮询 ID 1–6，缺的会超时跳过）。

**如果不动，按顺序查**

| 现象 | 先查 |
|---|---|
| `帧/s = 0` | USB-TTL 的 TX/RX 是否接反；C 板 UART1 的 pin2 是否接到 USB-TTL 的 RX |
| 有帧但 J0 不动 | 舵机 ID 是不是 1；ZL 板 5V0 要不要接（有些板子总线侧要 5 V 供电） |
| 偶尔动一下 | 共地：舵机电源负极、ZL 板 GND、C 板 GND 必须连一起 |
| 窗口打不开 | 加 `--text` 先看终端输出：`python3 viewer.py --source master --port ... --text` |

---

## C. 没硬件先看效果（10 秒）

```bash
cd ~/Embedded/custom_controller_ai/pc
python3 viewer.py --source sim
```

窗口里 6 根条按正弦摆动——这是上位机本身的显示效果。

---

## D. 机器人端（等主端过了再做）

```bash
cd ~/Embedded/custom_controller_ai
pyocd flash -t stm32f407ighx build/arm/stm32_module_lab.elf
cd pc
python3 uart_0311_inject.py --port /dev/tty.usbserial-XXXX --hz 75
```

CAN1 接 1 台达妙（ID 1，24 V，两端 120 Ω），期望：每秒打印 6 个角度、第 1 轴正弦摆动，
电机跟随；拔掉 USB-TTL 后 300 ms 内保持位置。

> ⚠️ 达妙力矩量程/反馈位域还没对官方手册核，先空载、小 Kp。
