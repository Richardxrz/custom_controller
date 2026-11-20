"""绝对角度监控：给舵盘对中用的大字号界面。

显示每个 ID 的：
    - PRAD
    - 舵机物理角（0–270°）
    - 对应关节角（相对 270° 中位）
    - 0/135/270 的位置条

单独运行：
    python3 angle_monitor.py            # 自动找 C 板 USB CDC
    python3 angle_monitor.py --port /dev/cu.usbmodemXXXX
"""

from __future__ import annotations

import argparse
import math
import struct
import threading
import time

import cboard_link
import payload
import serial.tools.list_ports as list_ports

ID_SLOT = {1: 0, 2: 1, 3: 2, 4: 4, 5: 3, 6: 5}
ID_JOINT = {1: "J0 base yaw", 2: "J1 shoulder pitch", 3: "J2 elbow pitch",
            4: "J4 pitch", 5: "J3 roll", 6: "J5 yaw"}
ID_TAG = {1: "J0", 2: "J1", 3: "J2", 4: "J4", 5: "J3", 6: "J5"}


def servo_deg(raw: int) -> float:
    return (max(500, min(2500, raw)) - 500) / 2000.0 * 270.0


class MonitorState:
    def __init__(self):
        self.lock = threading.Lock()
        self.servo = [1500] * 6
        self.target = [0.0] * 6
        self.seen = False

    def on_frame(self, f) -> None:
        if f.cmd_id == 0x0302:
            up = payload.decode_uplink(f.data)
            if up is None:
                return
            with self.lock:
                self.servo = list(up.joints)
                self.seen = True
        elif f.cmd_id == 0x0303 and len(f.data) >= 26:
            vals = [math.radians(struct.unpack_from("<h", f.data, 14 + i * 2)[0] / 100.0)
                    for i in range(6)]
            with self.lock:
                self.target = vals
                self.seen = True

    def snapshot(self):
        with self.lock:
            return list(self.servo), list(self.target), self.seen


class AngleMonitor:
    def __init__(self, master, state, standalone=False):
        import tkinter as tk

        self.tk = tk
        self.state = state
        self.win = tk.Toplevel(master) if master is not None else tk.Tk()
        self.win.title("绝对角度 · 舵盘对中")
        self.win.geometry("1040x720")
        self.win.minsize(960, 640)
        self.win.configure(bg="#0f1318")
        tk.Label(self.win, text="绝对角度监控", bg="#0f1318", fg="#dbe6ef",
                 font=("Helvetica", 20, "bold")).pack(anchor="w", padx=18, pady=(12, 0))
        tk.Label(self.win, text="ID4 → J4 pitch   ID5 → J3 roll   ID6 → J5 yaw；舵盘目标：PRAD 1500 / 舵机 135°",
                 bg="#0f1318", fg="#8ba0b2", font=("Helvetica", 12)).pack(anchor="w", padx=18)
        self.canvas = tk.Canvas(self.win, bg="#0f1318", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=18, pady=10)
        self.win.after(100, self.tick)

    def tick(self):
        try:
            servo, target, seen = self.state.snapshot()
        except Exception:
            return
        try:
            with self.state.lock:
                raw_values = list(self.state.raw)
        except Exception:
            raw_values = servo
        c = self.canvas
        c.delete("all")
        w = max(c.winfo_width(), 960)
        if not seen:
            c.create_text(w / 2, 80, text="等待 C 板数据…", fill="#8ba0b2",
                          font=("Helvetica", 18))
        for row, sid in enumerate(range(1, 7)):
            slot = ID_SLOT[sid]
            raw = raw_values[slot]
            sd = servo_deg(raw)
            jd = sd - 135.0
            hot = sid >= 4
            y = 55 + row * 92
            fg = "#dbe6ef" if hot else "#5f7385"
            c.create_rectangle(10, y - 18, w - 10, y + 58, fill="#151b22" if hot else "#11161c",
                               outline="#26333d")
            c.create_text(24, y, text=f"ID{sid}  →  {ID_JOINT[sid]}", anchor="w",
                          fill=fg, font=("Helvetica", 15, "bold"))
            c.create_text(300, y, text=f"{sd:5.1f}°", anchor="w", fill=fg,
                          font=("Helvetica", 28, "bold"))
            c.create_text(300, y + 30, text="舵机物理角", anchor="w", fill="#6f8496",
                          font=("Helvetica", 10))
            c.create_text(520, y, text=f"{jd:+6.1f}°", anchor="w", fill=fg,
                          font=("Helvetica", 28, "bold"))
            c.create_text(520, y + 30, text="关节角", anchor="w", fill="#6f8496",
                          font=("Helvetica", 10))
            x0, x1 = 700, w - 90
            c.create_rectangle(x0, y - 8, x1, y + 12, fill="#0b0f14", outline="#26333d")
            c.create_line((x0 + x1) / 2, y - 14, (x0 + x1) / 2, y + 18,
                          fill="#f0b84a", width=2)
            px = x0 + max(0, min(270, sd)) / 270.0 * (x1 - x0)
            c.create_line(px, y - 16, px, y + 20, fill="#3fd6a8", width=4)
            c.create_text(x0 - 8, y + 2, text="0", anchor="e", fill="#5f7385",
                          font=("Helvetica", 9))
            c.create_text(x1 + 8, y + 2, text="270", anchor="w", fill="#5f7385",
                          font=("Helvetica", 9))
            c.create_text(x1 + 8, y - 14, text=f"PRAD {raw}", anchor="w", fill=fg,
                          font=("Menlo", 10))
            if hot and abs(sd - 135.0) <= 8.0:
                c.create_text(24, y + 26, text="接近中位", anchor="w", fill="#3fd6a8",
                              font=("Helvetica", 10))
            elif hot and (sd < 15.0 or sd > 255.0):
                c.create_text(24, y + 26, text="接近端点，建议重新装舵盘", anchor="w",
                              fill="#f0b84a", font=("Helvetica", 10))
        self.win.after(100, self.tick)


def _reader(port: str, state: MonitorState, stop: threading.Event):
    try:
        link = cboard_link.CBoardLink(port, 921600)
    except Exception:
        return
    try:
        while not stop.is_set():
            for f in link.poll():
                state.on_frame(f)
            time.sleep(0.01)
    finally:
        link.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--seconds", type=float)
    args = ap.parse_args()
    port = args.port
    if not port:
        port = next((p.device for p in list_ports.comports()
                     if p.vid == 0x0483 and p.pid == 0x5740), "")
    state = MonitorState()
    stop = threading.Event()
    if port:
        threading.Thread(target=_reader, args=(port, state, stop), daemon=True).start()
    view = AngleMonitor(None, state, standalone=True)
    if args.seconds:
        view.win.after(int(args.seconds * 1000), view.win.destroy)
    try:
        view.win.mainloop()
    finally:
        stop.set()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
