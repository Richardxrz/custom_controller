"""自定义客户端上位机：舵机采集 → MQTT；机器人回传 → 6 轴显示。

一个窗口完成：
    - 连接/断开 MQTT
    - 单总线或双总线舵机采集
    - 发布 CustomControl
    - 订阅 CustomByteBlock，显示机器人 6 轴目标/实际

用法：
    python3 client_app.py
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
import urllib.request
import webbrowser

import dm_can
import joint_map
import mqtt_link
import payload
import serial_link
import serial.tools.list_ports as list_ports
import servo_link as servo_link_mod
import usb2can_classic
from multi_servo_link import MultiServoLink
from mock_broker import MockBroker
from usb_can_damiao import Damiao

HERE = os.path.dirname(os.path.abspath(__file__))


def _data_root() -> str:
    here = HERE
    candidates = [here]
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        candidates += [meipass,
                       os.path.join(os.path.dirname(meipass), "Resources"),
                       os.path.join(meipass, "..", "Resources")]
    for c in candidates:
        if os.path.isdir(os.path.join(c, "web_viewer")):
            return c
    return here


DATA_ROOT = _data_root()
_REBOT = os.path.join(DATA_ROOT, "web_viewer", "models", "rebot")
_DEMO = os.path.join(DATA_ROOT, "web_viewer", "models", "demo")
WEB_MODEL = _REBOT if os.path.isdir(_REBOT) else _DEMO


def find_cboard_port() -> str:
    """找 STM32 USB CDC（0483:5740），即 C 板主端。"""
    for p in list_ports.comports():
        if p.vid == 0x0483 and p.pid == 0x5740:
            return p.device
    return ""


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.servo = [1500] * 6
        self.target = [0.0] * 6
        self.actual = [0.0] * 6
        self.tau = [0.0] * 6
        self.temp = [0] * 6
        self.pub = 0
        self.down = 0
        self.game = 0
        self.direct = False
        self.servo_seen = False
        self.active_channels = list(range(6))
        self.channel_to_joint = list(range(6))
        self.joint_limit = math.radians(135)
        self.err = ""

    def on_control(self, data: bytes):
        up = payload.decode_uplink(data)
        if up is None:
            return
        with self.lock:
            self.servo = list(up.joints)
            targets = [0.0] * 6
            jm = joint_map.JointMap(joint_min=-self.joint_limit, joint_max=self.joint_limit)
            for i, raw in enumerate(up.joints):
                if i not in self.active_channels:
                    continue
                j = self.channel_to_joint[i] if i < 6 else i
                if 0 <= j < 6:
                    targets[j] = jm.to_joint(raw)
            self.target = targets
            self.pub += 1
            self.servo_seen = True

    def on_telemetry(self, data: bytes):
        if self.direct:
            return
        d = payload.decode_downlink(data)
        if d is None:
            return
        with self.lock:
            self.actual = [j.pos_urad / 1e6 for j in d.joint]
            self.tau = [j.torque_cnm / 100 for j in d.joint]
            self.temp = [j.temp_c for j in d.joint]
            self.down += 1

    def on_game(self, data: bytes):
        with self.lock:
            self.game += 1

    def on_motor(self, fb):
        with self.lock:
            self.actual[0] = fb["pos"]
            self.tau[0] = fb["torque"]
            self.temp[0] = fb["temp_mos"]

    def snapshot(self):
        with self.lock:
            return (list(self.servo), list(self.target), list(self.actual),
                    list(self.tau), list(self.temp), self.pub, self.down, self.game, self.err)


class ServoWorker(threading.Thread):
    def __init__(self, link, mq, state: State):
        super().__init__(daemon=True)
        self.link, self.mq, self.state = link, mq, state
        self.stop_flag = False

    def run(self):
        try:
            if hasattr(self.link, "start"):
                self.link.start()
            while not self.stop_flag:
                for f in self.link.poll():
                    if f.cmd_id != 0x0302:
                        continue
                    up = payload.decode_uplink(f.data)
                    if up is None:
                        continue
                    self.mq.publish_control(payload.encode_uplink(up))
                    self.state.on_control(f.data)
                time.sleep(0.001)
        finally:
            self.link.close()

    def stop(self):
        self.stop_flag = True
        self.join(timeout=1.0)


class MotorWorker(threading.Thread):
    """台架直连：把舵机 J0 的 360° 目标直接发给 USB2CAN 电机。"""

    def __init__(self, state, port, mid, kp, kd):
        super().__init__(daemon=True)
        self.state, self.port, self.mid = state, port, mid
        self.kp, self.kd = kp, kd
        self.stop_flag = False

    def run(self):
        bus = None
        motor = None
        try:
            bus = usb2can_classic.Usb2CanClassic(self.port, 921600)
            bus.open(1_000_000)
            motor = Damiao(bus, dm_can.MotorCfg(id=self.mid))
            motor.disable()
            time.sleep(0.05)
            motor.command(0, 0, 0, 0, 0)
            t0 = time.time()
            while motor.fb is None and time.time() - t0 < 1:
                motor.poll()
                time.sleep(0.01)
            motor.enable()
            while not self.stop_flag:
                if not self.state.servo_seen:
                    if motor.fb is not None:
                        motor.command(motor.fb["pos"], 0, 0, 0, 0)
                    motor.poll()
                    time.sleep(0.01)
                    continue
                with self.state.lock:
                    raw = self.state.servo[0]
                target = joint_map.JointMap(joint_min=-self.state.joint_limit, joint_max=self.state.joint_limit).to_joint(raw)
                motor.command(target, 0, self.kp, self.kd, 0)
                motor.poll()
                if motor.fb is not None:
                    self.state.on_motor(motor.fb)
                time.sleep(0.005)
        except Exception as exc:  # noqa: BLE001
            with self.state.lock:
                self.state.err = f"电机失败: {exc}"
        finally:
            if motor is not None:
                motor.disable()
            if bus is not None:
                bus.close()

    def stop(self):
        self.stop_flag = True
        self.join(timeout=1.0)


class ClientApp:
    def __init__(self, root):
        import tkinter as tk

        self.tk = tk
        self.root = root
        self.state = State()
        self.mq = None
        self.worker = None
        self.motor_worker = None
        self.broker = None
        self.sim = None
        self.viewer_server = None
        self.viewer_stop = threading.Event()
        self.viewer_poster = None
        root.title("自定义客户端 · 上位机")
        root.geometry("1200x860")
        root.minsize(1100, 760)
        root.configure(bg="#0f1318")

        bg, card_bg, line = "#0f1318", "#151b22", "#26333d"
        fg, muted, accent = "#dbe6ef", "#8ba0b2", "#3fd6a8"

        header = tk.Frame(root, bg=bg)
        header.pack(fill="x", padx=14, pady=(12, 6))
        tk.Label(header, text="自定义客户端", bg=bg, fg=fg,
                 font=("Helvetica", 18, "bold")).pack(side="left")
        self.status = tk.Label(header, text="未连接", bg=bg, fg=muted,
                               font=("Menlo", 12), anchor="e")
        self.status.pack(side="right", padx=(10, 0))
        self.btn = tk.Button(header, text="连接", command=self.toggle,
                             bg=accent, fg="#062018", relief="flat",
                             padx=18, pady=4, font=("Helvetica", 12, "bold"))
        self.btn.pack(side="right", padx=6)
        tk.Button(header, text="3D 仿真", command=self.open_web_sim,
                  bg="#1e2a33", fg=fg, relief="flat", padx=12, pady=4).pack(side="right", padx=6)
        tk.Button(header, text="2D 仿真", command=self.open_sim,
                  bg="#1e2a33", fg=fg, relief="flat", padx=12, pady=4).pack(side="right", padx=6)

        card = tk.Frame(root, bg=card_bg, highlightbackground=line, highlightthickness=1)
        card.pack(fill="x", padx=14, pady=6)
        left = tk.Frame(card, bg=card_bg)
        left.pack(side="left", fill="x", expand=True, padx=12, pady=10)
        right = tk.Frame(card, bg=card_bg)
        right.pack(side="right", fill="x", expand=True, padx=12, pady=10)

        def lab(parent, text, row, col=0):
            tk.Label(parent, text=text, bg=card_bg, fg=muted).grid(
                row=row, column=col, sticky="w", pady=3, padx=(0, 6))

        def entry(parent, row, col, width=22, value=""):
            e = tk.Entry(parent, width=width, bg="#0c1116", fg=fg,
                         insertbackground=fg, relief="flat", highlightthickness=1,
                         highlightbackground=line, highlightcolor=accent)
            e.insert(0, value)
            e.grid(row=row, column=col, sticky="w", pady=3, padx=(0, 10))
            return e

        lab(left, "MQTT", 0)
        self.host = entry(left, 0, 1, 16, "127.0.0.1")
        self.port = entry(left, 0, 2, 7, "3333")
        lab(left, "ID", 0, 3)
        self.cid = entry(left, 0, 4, 5, "1")
        self.broker_var = tk.BooleanVar(value=True)
        tk.Checkbutton(left, text="本地 broker", variable=self.broker_var,
                       bg=card_bg, fg=muted, selectcolor=card_bg,
                       activebackground=card_bg, activeforeground=fg).grid(
            row=0, column=5, sticky="w", padx=(0, 6))

        lab(left, "来源", 1)
        self.source_var = tk.StringVar(value="PC直连")
        tk.OptionMenu(left, self.source_var, "PC直连", "C板主端").grid(
            row=1, column=1, sticky="w", pady=3)
        lab(left, "C板端口", 1, 2)
        self.master_port = entry(left, 1, 3, 30, "")
        self.master_port.grid_configure(columnspan=3)
        detected = find_cboard_port()
        if detected:
            self.master_port.delete(0, "end")
            self.master_port.insert(0, detected)
            self.source_var.set("C板主端")

        lab(left, "舵机 A", 2)
        self.port_a = entry(left, 2, 1, 30, "/dev/tty.usbserial-1130:5")
        self.port_a.grid_configure(columnspan=3)
        lab(left, "舵机 B", 3)
        self.port_b = entry(left, 3, 1, 30, "")
        self.port_b.grid_configure(columnspan=3)

        self.direct_var = tk.BooleanVar(value=True)
        tk.Checkbutton(right, text="直连电机（台架）", variable=self.direct_var,
                       bg=card_bg, fg=muted, selectcolor=card_bg,
                       activebackground=card_bg, activeforeground=fg).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=3)
        lab(right, "CAN", 0, 2)
        self.can_port = entry(right, 0, 3, 28, "/dev/tty.usbmodem00000000050C1")
        lab(right, "ID", 1)
        self.motor_id = entry(right, 1, 1, 5, "8")
        lab(right, "Kp", 1, 2)
        self.kp = entry(right, 1, 3, 5, "5")
        lab(right, "Kd", 2)
        self.kd = entry(right, 2, 1, 5, "0.5")
        lab(right, "单舵机→关节", 2, 2)
        self.joint_var = tk.StringVar(value="J0")
        tk.OptionMenu(right, self.joint_var, *[f"J{i}" for i in range(6)],
                      command=self.set_joint_map).grid(row=2, column=3, sticky="w")
        lab(right, "关节范围", 3)
        self.range_var = tk.StringVar(value="270°")
        tk.OptionMenu(right, self.range_var, "270°", "360°",
                      command=self.set_joint_range).grid(row=3, column=1, sticky="w")
        lab(right, "3D 模型", 4)
        self.model_path = entry(right, 4, 1, 38, WEB_MODEL)
        self.model_path.grid_configure(columnspan=3)

        body = tk.Frame(root, bg=bg)
        body.pack(fill="both", expand=True, padx=14, pady=4)
        self.canvas = tk.Canvas(body, bg=bg, highlightthickness=0, height=390)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.create_text(20, 12, text="主端舵机（PRAD）", fill=muted, anchor="w")
        self.canvas.create_text(600, 12, text="机器人关节（目标/实际）", fill=muted, anchor="w")
        self.servo_bars, self.servo_text = [], []
        self.joint_bars, self.joint_lines, self.joint_text = [], [], []
        for i in range(6):
            y = 40 + i * 56
            self.canvas.create_text(20, y + 13, text=f"J{i}", fill=fg,
                                    font=("Menlo", 12, "bold"), anchor="w")
            self.canvas.create_rectangle(60, y, 460, y + 26, outline=line, fill="#101820")
            bar = self.canvas.create_rectangle(60, y, 60, y + 26, outline="", fill=accent)
            txt = self.canvas.create_text(470, y + 13, text="--", fill=fg,
                                          font=("Menlo", 10), anchor="w")
            self.servo_bars.append(bar)
            self.servo_text.append(txt)

            self.canvas.create_rectangle(640, y, 1040, y + 26, outline=line, fill="#101820")
            jb = self.canvas.create_rectangle(840, y, 840, y + 26, outline="", fill=accent)
            jl = self.canvas.create_line(840, y, 840, y + 26, fill="#f0b84a", width=2)
            jt = self.canvas.create_text(1050, y + 13, text="--", fill=fg,
                                         font=("Menlo", 10), anchor="w")
            self.joint_bars.append(jb)
            self.joint_lines.append(jl)
            self.joint_text.append(jt)

        self.log = tk.Text(root, height=6, bg="#0b0f14", fg="#b9c8d4",
                           insertbackground=fg, relief="flat", highlightthickness=1,
                           highlightbackground=line, font=("Menlo", 10))
        self.log.pack(fill="x", padx=14, pady=(4, 10))
        root.protocol("WM_DELETE_WINDOW", self.quit)
        root.after(50, self.tick)

    def _say(self, s: str):
        self.log.insert("end", s + "\n")
        self.log.see("end")

    def open_sim(self):
        if self.sim is not None:
            try:
                self.sim.win.deiconify()
                self.sim.win.lift()
                return
            except Exception:  # noqa: BLE001
                self.sim = None
        from arm_sim import ArmView

        self.sim = ArmView(self.root, self.state)

    def open_web_sim(self):
        model = self.model_path.get().strip() or WEB_MODEL
        if not os.path.isabs(model):
            model = os.path.join(HERE, model)
        if self.viewer_server is None:
            try:
                from web_viewer.server import ViewerServer
                self.viewer_server = ViewerServer(model, port=8765)
                self.viewer_server.start()
            except Exception as exc:  # noqa: BLE001
                self._say(f"3D 服务启动失败: {exc}")
                return
        webbrowser.open("http://127.0.0.1:8765/?follow=1")
        self._say(f"3D 仿真: {model}")
        if self.viewer_poster is None:
            self.viewer_stop.clear()
            self.viewer_poster = threading.Thread(target=self._post_angles, daemon=True)
            self.viewer_poster.start()

    def _post_angles(self):
        while not self.viewer_stop.is_set():
            with self.state.lock:
                vals = list(self.state.target)
            try:
                req = urllib.request.Request(
                    "http://127.0.0.1:8765/angles", data=json.dumps(vals).encode(),
                    headers={"Content-Type": "application/json"}, method="POST")
                urllib.request.urlopen(req, timeout=0.25).read()
            except Exception:  # noqa: BLE001
                pass
            self.viewer_stop.wait(0.1)

    def set_joint_range(self, value=None):
        deg = 270 if (value or self.range_var.get()).startswith("270") else 360
        self.state.joint_limit = math.radians(deg / 2)
        with self.state.lock:
            self.state.target = [0.0] * 6

    def set_joint_map(self, _=None):
        j = int(self.joint_var.get()[1:])
        with self.state.lock:
            self.state.channel_to_joint[0] = j
            self.state.target = [0.0] * 6

    def auto_cboard(self):
        port = find_cboard_port()
        if not port:
            self._say("未找到 C 板 USB CDC（0483:5740），请检查 USB 线/口")
            return
        self.master_port.delete(0, "end")
        self.master_port.insert(0, port)
        self.source_var.set("C板主端")
        self.direct_var.set(False)
        self.connect()

    def toggle(self):
        if self.mq is not None:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        host, port, cid = self.host.get().strip(), int(self.port.get()), self.cid.get().strip()
        if self.broker_var.get() and host in ("127.0.0.1", "localhost"):
            try:
                self.broker = MockBroker(host, port)
                self._say(f"本地 broker {host}:{port}")
            except OSError:
                self._say("broker 端口已占用，复用已有")
        self.mq = mqtt_link.MqttLink(host, port, f"client-{cid}")
        self.mq.handlers[mqtt_link.TOPIC_BYTE_BLOCK] = self.state.on_telemetry
        self.mq.handlers["GameStatus"] = self.state.on_game
        if not self.mq.start():
            self._say(f"MQTT 连接失败：{self.mq.last_error}")
            self.mq = None
            return
        link = None
        specs = []
        if self.source_var.get() == "C板主端":
            try:
                link = serial_link.SerialLink(self.master_port.get().strip(), 921600)
            except Exception as exc:  # noqa: BLE001
                self._say(f"C 板串口打开失败，仅 MQTT 继续：{exc}")
            with self.state.lock:
                self.state.active_channels = list(range(6))
                self.state.channel_to_joint = list(range(6))
                self.state.target = [0.0] * 6
        else:
            for e in (self.port_a, self.port_b):
                s = e.get().strip()
                if s:
                    p, _, ids = s.rpartition(":")
                    specs.append((p, [int(x) for x in ids.split(",")]))
            total_ids = sum(len(ids) for _, ids in specs)
            with self.state.lock:
                self.state.active_channels = list(range(total_ids))
                self.state.channel_to_joint = list(range(6))
                if total_ids == 1:
                    self.state.channel_to_joint[0] = int(self.joint_var.get()[1:])
                self.state.target = [0.0] * 6
            if not specs:
                self._say("未配置舵机：仅 MQTT 模式")
            else:
                try:
                    link = MultiServoLink(specs) if len(specs) > 1 else servo_link_mod.ServoLink(*specs[0])
                except Exception as exc:  # noqa: BLE001
                    self._say(f"舵机打开失败，仅 MQTT 继续：{exc}")
        if link is not None:
            self.worker = ServoWorker(link, self.mq, self.state)
            self.worker.start()
        self.state.direct = self.direct_var.get()
        if self.direct_var.get():
            self.motor_worker = MotorWorker(self.state, self.can_port.get().strip(),
                                            int(self.motor_id.get()), float(self.kp.get()),
                                            float(self.kd.get()))
            self.motor_worker.start()
        self.btn.configure(text="断开")
        self._say(f"已连接 {host}:{port}，来源 {self.source_var.get()}")

    def disconnect(self):
        if self.motor_worker is not None:
            self.motor_worker.stop()
            self.motor_worker = None
        self.state.direct = False
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
        if self.mq is not None:
            self.mq.stop()
            self.mq = None
        if self.broker is not None:
            self.broker.close()
            self.broker = None
        self.btn.configure(text="连接")
        self._say("已断开")

    def tick(self):
        servo, target, actual, tau, temp, pub, down, game, err = self.state.snapshot()
        for i in range(6):
            x = 60 + (servo[i] - 500) / 2000 * 400
            self.canvas.coords(self.servo_bars[i], 60, 40 + i * 56, x, 66 + i * 56)
            self.canvas.itemconfigure(self.servo_text[i],
                                      text=f"{servo[i]:4d}  {(servo[i]-500)/2000*270:5.1f}°")
            xt = 840 + max(-1, min(1, target[i] / math.pi)) * 190
            xa = 840 + max(-1, min(1, actual[i] / math.pi)) * 190
            self.canvas.coords(self.joint_bars[i], min(840, xa), 40 + i * 56,
                               max(840, xa), 66 + i * 56)
            self.canvas.coords(self.joint_lines[i], xt, 40 + i * 56, xt, 66 + i * 56)
            self.canvas.itemconfigure(self.joint_text[i],
                                      text=f"{actual[i]:+.2f}/{target[i]:+.2f} τ{tau[i]:+.1f} {temp[i]}℃")
        self.status.configure(text=f"发布 {pub}   回传 {down}   GameStatus {game}   {err}")
        self.root.after(50, self.tick)

    def quit(self):
        self.viewer_stop.set()
        if self.viewer_server is not None:
            self.viewer_server.stop()
            self.viewer_server = None
        self.disconnect()
        self.root.destroy()


def main() -> int:
    import argparse
    import tkinter as tk

    ap = argparse.ArgumentParser()
    ap.add_argument("--auto", action="store_true")
    ap.add_argument("--seconds", type=float)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--web-sim", action="store_true")
    ap.add_argument("--auto-cboard", action="store_true")
    args = ap.parse_args()
    root = tk.Tk()
    app = ClientApp(root)
    if args.auto:
        root.after(300, app.connect)
    if args.sim:
        root.after(200, app.open_sim)
    if args.web_sim:
        root.after(200, app.open_web_sim)
    if args.auto_cboard or getattr(sys, "frozen", False):
        root.after(400, app.auto_cboard)
    if args.seconds:
        root.after(int(args.seconds * 1000), app.quit)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
