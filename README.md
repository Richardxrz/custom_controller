# custom_controller_ai

> **AI 实现区**：机械臂自定义控制器全链路（主端固件 + 机器人端固件 + PC 终端）。
> 用户的设计在 `~/ai/50-机械臂控制器/`，用户的复刻在 `~/Embedded/stm32_module_lab/`。
> 这里的代码是**参考实现**，允许用现成库和样板；复刻时按用户自己的框架风格重写。

## 操作端有两种形态

| 形态 | 链路 | 什么时候用 |
|---|---|---|
| **PC 直连（已采用）** | ZP10S×6 ─单总线×2（3+3）─ ZL 转接板×2 ─USB─ PC（轮询 + MQTT） | 操作端不加 C 板；单总线 27 Hz，双总线约 55 Hz（代码待支持） |
| C 板中转（不采用，仅备选） | 舵机 ─ C 板 UART6 ─ UART1 ─ USB-TTL ─ PC | 已验证无频率收益；只作为回退 |

PC 直连是当前方案：`terminal_app.py --source servo`；`--source master` 仅作 C 板备选。

## 数据流

```
主端（操作间）
  ZP10S×6（释力）──单总线×2（3+3）──ZL 转接板×2──USB──▶ 终端 PC（本仓库 pc/）
                                                     │ MQTT 192.168.12.1:3333
                                                     │  CustomControl(30 B) @75 Hz
                                               赛事引擎 / 图传
                                                     │ 图传链路 0x0311(30 B)
机器人（场上）                                        ▼
  C 板(臂控) ──CAN1 1 Mbps── 达妙 8009×3 + 4340×3（MIT）
       │
       └──UART1 921600── 图传发送端（0x0311 收 / 0x0310 回）
```

协议依据：《RoboMaster 2026 机甲大师高校系列赛通信协议 V1.3.1》。
关键命令码：`0x0311` 客户端→机器人（30 B，≤75 Hz）、`0x0310` 机器人→客户端（300 B，≤50 Hz）；
主端到 PC 这一段用 `0x0302`（30 B）承载同样的 30 B 载荷，PC 转成 `CustomControl`。

## 目录

| 路径 | 内容 |
|---|---|
| `Module/` | `ringbuf`、`servo_codec`、`bus_servo`、`bus_servo_group`、`rm_frame`、`app_payload`、`dm_can` |
| `App/` | 板级初始化、UART 接收、主端/机器人端应用、角色选择 |
| `tests/host/` | C 主机侧测试（不需要板子），`run_all.sh` |
| `pc/` | 终端：串口、MQTT、视频 UDP、机器人替身、mock broker、测试 |
| `scripts/` | `build_fw.sh`（双角色固件）、`test_all.sh`（全部测试） |

## 构建

```bash
./scripts/build_fw.sh          # 主端 + 机器人端，输出 build/{master,arm}/stm32_module_lab.elf
./scripts/test_all.sh          # 98 条 C 判据 + 36 个 Python 用例
```

烧录（示例）：

```bash
pyocd flash -t stm32f407ighx build/master/stm32_module_lab.elf   # 主端
pyocd flash -t stm32f407ighx build/arm/stm32_module_lab.elf      # 机器人端
```

> PC 直连不需要烧主端固件；`build/master` 只在 C 板备选方案使用。

## 接线

**主端（PC 直连）**

| 用途 | 接口 | 说明 |
|---|---|---|
| 舵机总线 | ZL 转接板×2（USB） | 6 轴拆 3+3；每块板 115200 |
| PC | USB-A / USB-C | 两块板各占一个 USB 口；建议带供电 Hub |
| 急停 | 软件（PC 键盘） | 无硬件急停；C 板备选方案才有 KEY（PA0） |

**机器人端 C 板**

| 用途 | 引脚/接口 | 说明 |
|---|---|---|
| 图传 | USART1（PA9/PB7） | 921600，3.3 V TTL，只接 RX/TX/GND |
| 达妙 | CAN1（PD0/PD1） | 1 Mbps，总线两端 120 Ω |
| 图传回传 | USART1 TX | `0x0310` 300 B @50 Hz |

## 跑终端（无硬件也能演示）

```bash
# 终端 1：本地 broker
python3 - <<'PY'
from pc.mock_broker import MockBroker
import time
b = MockBroker("127.0.0.1", 3333); print("broker", b.port); time.sleep(600)
PY

# 终端 2：机器人替身
cd pc && python3 robot_mock.py --host 127.0.0.1 --port 3333

# 终端 3：终端应用（模拟主端正弦输入）
cd pc && python3 terminal_app.py --simulate --host 127.0.0.1 --mqtt-port 3333 --seconds 10
```

真机：`python3 pc/terminal_app.py --port /dev/tty.usbserial-XXXX --host 192.168.12.1 --mqtt-port 3333 --id 1`

## 已知限制（明天验收重点）

1. **没有上过真机**：所有测试都是主机侧/回环；引脚、电平、CAN 时序需实测。
2. **达妙反馈帧的 ID 与温度位域未核**：`dm_can.h` 顶部列了三处不确定项；先单电机空载验证。
3. **UART1 是手写初始化**（CubeMX 里没有）：先量 PA9/PB7 波形再接图传。
4. **视频只做拆包统计**，没有 HEVC 解码。
5. **没有 USB CDC**：PC 链路走 UART1 + USB-TTL，避免在 CubeMX 之外手写 USB 栈。
6. **2027 版通信协议未发布**：当前按 2026 V1.3.1 实现；发布后核对命令码/MQTT 参数即可，协议层已隔离。
