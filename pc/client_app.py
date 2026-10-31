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

import math
import threading
import time

import joint_map
import mqtt_link
import payload
import servo_link as servo_link_mod
from multi_servo_link import MultiServoLink
from mock_broker import MockBroker


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
        self.err = ""

    def on_control(self, data: bytes):
        up = payload.decode_uplink(data)
        if up is None:
            return
        with self.lock:
            self.servo = list(up.joints)
            self.target = [joint_map.JointMap().to_joint(j) for j in up.joints]
            self.pub += 1

    def on_telemetry(self, data: bytes):
        d = payload.decode_downlink(data)
        if d is None:
            return
        with self.lock:
            self.actual = [j.pos_urad / 1e6 for j in d.joint]
            self.tau = [j.torque_cnm / 100 for j in d.joint]
            self.temp = [j.temp_c for j in d.joint]
            self.down += 1

    def snapshot(self):
        with self.lock:
            return (list(self.servo), list(self.target), list(self.actual),
                    list(self.tau), list(self.temp), self.pub, self.down, self.err)


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


class ClientApp:
    def __init__(self, root):
        import tkinter as tk

        self.tk = tk
        self.root = root
        self.state = State()
        self.mq = None
        self.worker = None
        self.broker = None
        root.title("自定义客户端 · 上位机")
        root.geometry("960x720")
        root.configure(bg="#141414")

        cfg = tk.Frame(root, bg="#141414")
        cfg.pack(fill="x", padx=10, pady=8)
        tk.Label(cfg, text="MQTT", fg="#bbb", bg="#141414").grid(row=0, column=0)
        self.host = tk.Entry(cfg, width=16)
        self.host.insert(0, "127.0.0.1")
        self.host.grid(row=0, column=1, padx=4)
        self.port = tk.Entry(cfg, width=6)
        self.port.insert(0, "3333")
        self.port.grid(row=0, column=2)
        tk.Label(cfg, text="ID", fg="#bbb", bg="#141414").grid(row=0, column=3)
        self.cid = tk.Entry(cfg, width=4)
        self.cid.insert(0, "1")
        self.cid.grid(row=0, column=4)
        self.broker_var = tk.BooleanVar(value=True)
        tk.Checkbutton(cfg, text="本地 broker", variable=self.broker_var,
                       fg="#bbb", bg="#141414", selectcolor="#333").grid(row=0, column=5, padx=8)

        tk.Label(cfg, text="舵机 A", fg="#bbb", bg="#141414").grid(row=1, column=0, pady=4)
        self.port_a = tk.Entry(cfg, width=34)
        self.port_a.insert(0, "/dev/tty.usbserial-1130:5")
        self.port_a.grid(row=1, column=1, columnspan=2, sticky="w")
        tk.Label(cfg, text="舵机 B（可空）", fg="#bbb", bg="#141414").grid(row=2, column=0)
        self.port_b = tk.Entry(cfg, width=34)
        self.port_b.grid(row=2, column=1, columnspan=2, sticky="w")

        self.btn = tk.Button(cfg, text="连接", command=self.toggle)
        self.btn.grid(row=1, column=2, padx=8)
        self.status = tk.Label(root, text="未连接", fg="#9ad", bg="#141414",
                               font=("Menlo", 12), anchor="w")
        self.status.pack(fill="x", padx=12)

        body = tk.Frame(root, bg="#141414")
        body.pack(fill="both", expand=True, padx=10, pady=6)
        self.canvas = tk.Canvas(body, width=930, height=470, bg="#141414",
                                highlightthickness=0)
        self.canvas.pack()
        self.canvas.create_text(180, 12, text="主端舵机（PRAD）", fill="#bbb")
        self.canvas.create_text(700, 12, text="机器人关节（目标/实际）", fill="#bbb")
        self.servo_bars, self.servo_text = [], []
        self.joint_bars, self.joint_lines, self.joint_text = [], [], []
        for i in range(6):
            y = 40 + i * 60
            self.canvas.create_text(20, y + 14, text=f"J{i}", fill="#eee",
                                    font=("Menlo", 12, "bold"))
            self.canvas.create_rectangle(50, y, 350, y + 28, outline="#333", fill="#1e1e1e")
            bar = self.canvas.create_rectangle(50, y, 50, y + 28, outline="", fill="#3fa")
            txt = self.canvas.create_text(360, y + 14, text="--", fill="#eee",
                                          font=("Menlo", 10), anchor="w")
            self.servo_bars.append(bar)
            self.servo_text.append(txt)

            self.canvas.create_rectangle(520, y, 900, y + 28, outline="#333", fill="#1e1e1e")
            jb = self.canvas.create_rectangle(710, y, 710, y + 28, outline="", fill="#3fa")
            jl = self.canvas.create_line(710, y, 710, y + 28, fill="#fa3", width=2)
            jt = self.canvas.create_text(905, y + 14, text="--", fill="#eee",
                                         font=("Menlo", 10), anchor="w")
            self.joint_bars.append(jb)
            self.joint_lines.append(jl)
            self.joint_text.append(jt)

        self.log = tk.Text(root, height=6, bg="#0b0b0b", fg="#ccc",
                           font=("Menlo", 10))
        self.log.pack(fill="x", padx=10, pady=8)
        root.protocol("WM_DELETE_WINDOW", self.quit)
        root.after(50, self.tick)

    def _say(self, s: str):
        self.log.insert("end", s + "\n")
        self.log.see("end")

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
        if not self.mq.start():
            self._say(f"MQTT 连接失败：{self.mq.last_error}")
            self.mq = None
            return
        specs = []
        for e in (self.port_a, self.port_b):
            s = e.get().strip()
            if s:
                p, _, ids = s.rpartition(":")
                specs.append((p, [int(x) for x in ids.split(",")]))
        link = MultiServoLink(specs) if len(specs) > 1 else servo_link_mod.ServoLink(*specs[0])
        self.worker = ServoWorker(link, self.mq, self.state)
        self.worker.start()
        self.btn.configure(text="断开")
        self._say(f"已连接 {host}:{port}，舵机 {specs}")

    def disconnect(self):
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
        servo, target, actual, tau, temp, pub, down, err = self.state.snapshot()
        for i in range(6):
            x = 50 + (servo[i] - 500) / 2000 * 300
            self.canvas.coords(self.servo_bars[i], 50, 40 + i * 60, x, 68 + i * 60)
            self.canvas.itemconfigure(self.servo_text[i],
                                      text=f"{servo[i]:4d}  {(servo[i]-500)/2000*270:5.1f}°")
            xt = 710 + max(-1, min(1, target[i] / math.pi)) * 190
            xa = 710 + max(-1, min(1, actual[i] / math.pi)) * 190
            self.canvas.coords(self.joint_bars[i], min(710, xa), 40 + i * 60,
                               max(710, xa), 68 + i * 60)
            self.canvas.coords(self.joint_lines[i], xt, 40 + i * 60, xt, 68 + i * 60)
            self.canvas.itemconfigure(self.joint_text[i],
                                      text=f"{actual[i]:+.2f}/{target[i]:+.2f} τ{tau[i]:+.1f} {temp[i]}℃")
        self.status.configure(text=f"发布 {pub}   回传 {down}   {err}")
        self.root.after(50, self.tick)

    def quit(self):
        self.disconnect()
        self.root.destroy()


def main() -> int:
    import argparse
    import tkinter as tk

    ap = argparse.ArgumentParser()
    ap.add_argument("--auto", action="store_true")
    ap.add_argument("--seconds", type=float)
    args = ap.parse_args()
    root = tk.Tk()
    app = ClientApp(root)
    if args.auto:
        root.after(300, app.connect)
    if args.seconds:
        root.after(int(args.seconds * 1000), app.quit)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
