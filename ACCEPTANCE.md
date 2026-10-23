# 验收报告（2026-10-23）

> 范围：机械臂自定义控制器全链路。**代码全部在主机侧/回环里测过；真机未验证**（见 §4）。

## 1. 交付物

| 层 | 文件 | 说明 |
|---|---|---|
| 驱动/数据结构 | `Module/ringbuf.c/h` | 单生产者/单消费者字节环，不关中断 |
| 驱动（板级） | `App/uart_rx.c/h` | UART 逐字节中断 → 环；`BusServoPort` 实现（写走 `tx_uart_it`） |
| 协议（舵机） | `Module/servo_codec.c/h` | ASCII 编解码、回显分类、半包/超长帧 |
| 指令（舵机） | `Module/bus_servo.c/h` | 发→等→超时；读角/释力/设角 |
| 调度（舵机） | `Module/bus_servo_group.c/h` | 6 轴轮询、周期、单轴超时跳过、逐个释力 |
| 协议（RM） | `Module/rm_frame.c/h` | A5 帧 + CRC8/CRC16（与官方表核对过） |
| 载荷 | `Module/app_payload.c/h` | 30 B 上行 / 300 B 下行 |
| 执行器 | `Module/dm_can.c/h` | 达妙 MIT 打包/反馈解包、使能/失能/设零 |
| 主端应用 | `App/master_app.c` | 释力→采样→`0x0302` @100 Hz |
| 机器人端应用 | `App/arm_app.c` | `0x0311`→6 电机 MIT @200 Hz→`0x0310` @50 Hz；断链/急停保持 |
| 板级 | `App/board_io.c` | UART1 921600、CAN1 1 Mbps（手写，CubeMX 未生成） |
| 中断 | `App/irq_handlers.c` | USART1 IRQ（CubeMX 只生成了 USART6） |
| PC 终端 | `pc/rmproto.py`、`payload.py`、`serial_link.py`、`mqtt_link.py`、`video_link.py`、`terminal_app.py` | 串口→MQTT、下行走 MQTT、视频拆包 |
| 台架工具 | `pc/robot_mock.py`、`pc/uart_0311_inject.py`、`pc/mock_broker.py` | 无图传/无机器人时跑通全链路 |

## 2. 测试证据

```bash
./scripts/test_all.sh
```

| 测试 | 判据数 | 内容 |
|---|---|---|
| `host_servo` | 41 | 环、构造/解析、回显、半包、超时、释力、设角 |
| `host_rm` | 25 | CRC 对照官方表、帧往返/容错/重同步、30 B/300 B |
| `host_group` | 10 | 6 轴轮询、单轴超时跳过、周期 20 ms、逐个释力 |
| `host_dm` | 16 | MIT 位域、满量程/夹紧、反馈、特殊帧 |
| Python | 15 用例 | C↔Python 逐字节金标、pty 串口、MQTT 回环、视频拆包、端到端 |
| 固件编译 | 2 角色 | `build/master`、`build/arm` 均链接通过（text ≈19 KB） |

C 与 Python 的 30 B / 300 B / 帧字节序列由 `tests/host/dump_vectors.c` 生成金标，两边必须完全一致（已加入 `test_protocol.py`）。

## 3. 关键决策

| 决策 | 理由 |
|---|---|
| PC 链路用 **UART1 + USB-TTL**，不做 USB CDC | CubeMX 没配 USB，手写 USB 栈风险远大于收益；UART 921600 足够 |
| 舵机接收用 **逐字节中断**，不用 DMA+IDLE | `rx_dma` 已占用 `HAL_UARTEx_RxEventCallback`；10 kB/s 的量中断完全够 |
| 机器人端解析 **0x0311**（不是 0x0302） | 2026 协议：`CustomControl` 由图传链路转成 0x0311；0x0302 需要选手端 |
| 30 B 载荷自定格式 + CRC16 | 官方帧本身有 CRC16；载荷再校验一次，PC 侧能拒错帧 |
| 达妙反馈按 **data[0]=电机 ID** 解 | 与常见实现一致；但反馈 CAN ID、状态字节位域仍需对达妙手册核对 |
| 断链/急停 **保持**（Kp+Kd）而不是断力 | 6 轴臂断力会砸下来；保持更安全 |

## 4. 没验证的部分（明天重点）

1. **所有引脚/电平**：UART1（PA9/PB7）是手写初始化，先用示波器/逻辑分析仪看波形再接图传。
2. **达妙反馈帧**：ID、温度/错误位域、P/V/T 量程（`dm_can.h` 顶部列了三处不确定项）。先单电机空载、小 Kp。
3. **CAN 终端电阻/拓扑**：两端 120 Ω；星形/长支线会翻车。
4. **舵机总线电平/回显**：取决于 ZL 转接板；`servo_codec` 已按“命令帧即回显”处理。
5. **ZP10S 固件是 V2（`ZL-ZServo_AD_CBM V2.1.16STG`），与手册不一致**：`PULK`/`PULR` 无响应；
   `PCSM`（开机释力）可用；`PRTV`/`PVER` 的应答以 `@` 开头。主端固件发的 `PULK` 在 V2 上会超时，
   所以**必须先 `servo_cli free` 设一次开机释力并重新上电**。
5. **图传链路**：没有设备，`0x0311` 只用本地 USB-TTL 注入验证；图传到货后只换传输层。
6. **视频**：只拆包统计，没有 HEVC 解码。

## 5. 明天的验收步骤

### A. 主端（30 分钟）

1. 烧 `build/master/stm32_module_lab.elf`。
2. UART6 → ZL 转接板 → 1 个舵机（ID 1），6 V 供电；UART1 → USB-TTL → Mac。
3. 起 broker + 机器人替身：
   ```bash
   python3 -c "from pc.mock_broker import MockBroker; import time; b=MockBroker('127.0.0.1',3333); print(b.port); time.sleep(600)" &
   (cd pc && python3 robot_mock.py --host 127.0.0.1 --port 3333) &
   ```
4. 起终端：
   ```bash
   cd pc && python3 terminal_app.py --port /dev/tty.usbserial-XXXX --host 127.0.0.1 --mqtt-port 3333 --id 1 --verbose
   ```
5. **期望**：`serial` 每秒约 +100；手扳舵机，`joints` 第 1 位变化；`robot_mock` 的 `controls` 递增；终端 `down` 递增。
6. 6 轴全接（ID 1–6），逐个扳动，确认对应位变化、无 `-`。

### B. 机器人端（30 分钟，先单电机）

1. 烧 `build/arm/stm32_module_lab.elf`。
2. CAN1 → 1 台达妙（ID 1），24 V，两端 120 Ω。
3. UART1 → USB-TTL → Mac，跑：
   ```bash
   cd pc && python3 uart_0311_inject.py --port /dev/tty.usbserial-XXXX --hz 75
   ```
4. **期望**：每秒打印 6 个角度（第 1 轴正弦摆动）、`vbus=24000mV`、`frames` 递增；电机跟随。
5. 拔掉 USB-TTL → 300 ms 后电机保持当前位置（不砸）。

### C. 通过标准

- 主端：6 轴 50 Hz 以上、扳动无跳变、PC 端计数稳定；
- 机器人端：单电机跟随后再上 6 台，断链保持、急停保持；
- 全程无复位、无 IWDG（本版未开看门狗，见风险）。

## 6. 风险 / 待办

| 项 | 说明 |
|---|---|
| 达妙参数 | 力矩量程、反馈 ID、温度位域必须对官方手册核；`arm_app.c` 里 8009/4340 的 T 量程是暂定值 |
| 关节映射 | `arm_app.c` 的 `s_scale`/`s_dir` 是占位（±1.5 rad、全 +1），装好后按实际关节标定 |
| 看门狗 | 未开；建议下一版加 IWDG + 上电安全位 |
| 标定 | PC 端还没有标定 UI；先用 `uart_0311_inject.py`/串口打印核对 |
| 2027 协议 | 发布后核对 0x0311/0x0310/MQTT 参数；协议层已隔离在 `pc/` 与 `Module/rm_frame` |
