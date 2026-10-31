"""一体化测试台：一个窗口启动 broker + 舵机终端 + 电机，并显示 J0。

用法：
    python3 app.py                # GUI，手动点“启动”
    python3 app.py --auto         # 打开后自动启动
    python3 app.py --headless --seconds 5   # 无界面自检
"""

from __future__ import annotations

import argparse
import os
import queue
import signal
import subprocess
import sys
import threading
import time

import joint_map
import mqtt_link
import payload
from mock_broker import MockBroker

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULTS = {
    "servo_port": "/dev/tty.usbserial-1130",
    "servo_ids": "5",
    "can_port": "/dev/tty.usbmodem00000000050C1",
    "motor_id": "8",
    "kp": "5",
    "kd": "0.5",
}


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.joints = [1500] * 6
        self.target = 0.0
        self.pos = 0.0
        self.tau = 0.0
        self.temp = 0
        self.ok = False

    def on_control(self, data: bytes):
        up = payload.decode_uplink(data)
        if up is None:
            return
        with self.lock:
            self.joints = list(up.joints)
            self.target = joint_map.JointMap().to_joint(up.joints[0])

    def on_telemetry(self, data: bytes):
        d = payload.decode_downlink(data)
        if d is None:
            return
        with self.lock:
            j = d.joint[0]
            self.pos = j.pos_urad / 1e6
            self.tau = j.torque_cnm / 100
            self.temp = j.temp_c
            self.ok = True

    def snapshot(self):
        with self.lock:
            return list(self.joints), self.target, self.pos, self.tau, self.temp, self.ok


class Bench:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.q: queue.Queue[str] = queue.Queue()
        self.procs: list[subprocess.Popen] = []
        self.broker = None
        self.state = State()
        self.mq = None

    def start(self):
        if self.broker is None:
            try:
                self.broker = MockBroker("127.0.0.1", 3333)
                self.q.put("[broker] 127.0.0.1:3333 已启动")
            except OSError:
                self.broker = None
                self.q.put("[broker] 3333 已有 broker，直接复用")
        if self.mq is None:
            self.mq = mqtt_link.MqttLink("127.0.0.1", 3333, "app-ui")
            self.mq.handlers[mqtt_link.TOPIC_CONTROL] = self.state.on_control
            self.mq.handlers[mqtt_link.TOPIC_BYTE_BLOCK] = self.state.on_telemetry
            self.mq.start()
        self._spawn([
            sys.executable, "-u", os.path.join(HERE, "terminal_app.py"),
            "--source", "servo", "--port", self.cfg["servo_port"],
            "--ids", self.cfg["servo_ids"], "--host", "127.0.0.1",
            "--mqtt-port", "3333", "--id", "1", "--verbose",
        ], "servo")
        self._spawn([
            sys.executable, "-u", os.path.join(HERE, "usb_can_damiao.py"),
            "--transport", "dm_classic", "--port", self.cfg["can_port"],
            "--mqtt", "--id", self.cfg["motor_id"], "--kp", self.cfg["kp"],
            "--kd", self.cfg["kd"], "--mqtt-host", "127.0.0.1",
            "--mqtt-port", "3333", "--client-id", "1",
        ], "motor")

    def _spawn(self, cmd: list[str], tag: str):
        p = subprocess.Popen(cmd, cwd=HERE, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, bufsize=1)
        self.procs.append(p)
        threading.Thread(target=self._read, args=(p, tag), daemon=True).start()

    def _read(self, p: subprocess.Popen, tag: str):
        for line in p.stdout:
            self.q.put(f"[{tag}] {line.rstrip()}")

    def stop(self):
        for p in self.procs:
            p.send_signal(signal.SIGINT)
        for p in self.procs:
            try:
                p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                p.kill()
        self.procs.clear()
        if self.mq is not None:
            self.mq.stop()
            self.mq = None
        self.q.put("[app] 已停止")


class App:
    def __init__(self, root, bench: Bench):
        import tkinter as tk

        self.tk, self.root, self.bench = tk, root, bench
        root.title("自定义控制器 · 测试台")
        root.geometry("900x600")
        root.configure(bg="#141414")
        self.entries = {}
        top = tk.Frame(root, bg="#141414")
        top.pack(fill="x", padx=10, pady=8)
        for i, key in enumerate(DEFAULTS):
            tk.Label(top, text=key, fg="#bbb", bg="#141414").grid(row=0, column=i * 2)
            e = tk.Entry(top, width=12 if key not in ("servo_port", "can_port") else 26)
            e.insert(0, DEFAULTS[key])
            e.grid(row=0, column=i * 2 + 1, padx=(2, 10))
            self.entries[key] = e
        tk.Button(top, text="启动", command=self.start).grid(row=0, column=12, padx=4)
        tk.Button(top, text="停止", command=self.stop).grid(row=0, column=13, padx=4)

        self.info = tk.Label(root, text="未启动", fg="#9ad", bg="#141414",
                             font=("Menlo", 14), anchor="w")
        self.info.pack(fill="x", padx=12, pady=4)
        self.joints = tk.Label(root, text="", fg="#8cf", bg="#141414",
                               font=("Menlo", 12), anchor="w")
        self.joints.pack(fill="x", padx=12)
        self.log = tk.Text(root, height=18, bg="#0b0b0b", fg="#ddd",
                           font=("Menlo", 10), insertbackground="#ddd")
        self.log.pack(fill="both", expand=True, padx=10, pady=8)
        root.protocol("WM_DELETE_WINDOW", self.quit)
        root.after(100, self.poll)

    def _cfg(self):
        return {k: e.get().strip() for k, e in self.entries.items()}

    def start(self):
        if self.bench.procs:
            self.bench.q.put("[app] 已在运行")
            return
        self.bench.cfg = self._cfg()
        self.bench.start()

    def stop(self):
        self.bench.stop()

    def quit(self):
        self.stop()
        self.root.destroy()

    def poll(self):
        while not self.bench.q.empty():
            self.log.insert("end", self.bench.q.get() + "\n")
            self.log.see("end")
        joints, target, pos, tau, temp, ok = self.bench.state.snapshot()
        if ok:
            self.info.configure(text=f"J0 target={target:+.2f}  pos={pos:+.3f}  "
                                      f"tau={tau:+.2f}  T={temp}℃")
        self.joints.configure(text="PRAD  " + "  ".join(str(x) for x in joints))
        self.root.after(100, self.poll)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--auto", action="store_true")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--seconds", type=float)
    args = ap.parse_args()

    bench = Bench(dict(DEFAULTS))
    if args.headless:
        bench.start()
        t0 = time.time()
        try:
            while args.seconds is None or time.time() - t0 < args.seconds:
                if not bench.q.empty():
                    print(bench.q.get())
                time.sleep(0.05)
        finally:
            bench.stop()
        return 0

    import tkinter as tk

    root = tk.Tk()
    app = App(root, bench)
    if args.auto:
        root.after(300, app.start)
    if args.seconds:
        root.after(int(args.seconds * 1000), app.quit)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
