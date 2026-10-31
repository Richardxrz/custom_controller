"""单电机直接控制面板：不经过 MQTT/舵机，直接 USB2CAN + MIT。

用法：
    python3 motor_app.py                  # GUI
    python3 motor_app.py --auto           # 打开即连 CAN
    python3 motor_app.py --headless --seconds 3   # 无界面自检
"""

from __future__ import annotations

import argparse
import math
import time

import dm_can
import usb2can_classic
from usb_can_damiao import Damiao

DEFAULT_PORT = "/dev/tty.usbmodem00000000050C1"
DEFAULT_ID = 8


class Panel:
    def __init__(self, root, auto=False):
        import tkinter as tk

        self.tk = tk
        self.root = root
        self.bus = None
        self.motor = None
        self.enabled = False
        self.closing = False
        root.title("单电机直接控制")
        root.geometry("760x520")
        root.configure(bg="#141414")

        top = tk.Frame(root, bg="#141414")
        top.pack(fill="x", padx=10, pady=8)
        tk.Label(top, text="CAN", fg="#bbb", bg="#141414").grid(row=0, column=0)
        self.port = tk.Entry(top, width=34)
        self.port.insert(0, DEFAULT_PORT)
        self.port.grid(row=0, column=1, padx=4)
        tk.Label(top, text="ID", fg="#bbb", bg="#141414").grid(row=0, column=2)
        self.mid = tk.Entry(top, width=4)
        self.mid.insert(0, str(DEFAULT_ID))
        self.mid.grid(row=0, column=3, padx=4)
        tk.Button(top, text="打开", command=self.open_can).grid(row=0, column=4, padx=4)
        tk.Button(top, text="关闭", command=self.close_can).grid(row=0, column=5, padx=4)

        self.state = tk.Label(root, text="未连接", fg="#9ad", bg="#141414",
                              font=("Menlo", 13), anchor="w")
        self.state.pack(fill="x", padx=12)

        mid = tk.Frame(root, bg="#141414")
        mid.pack(fill="x", padx=10, pady=8)
        tk.Label(mid, text="目标°", fg="#bbb", bg="#141414").grid(row=0, column=0)
        self.slider = tk.Scale(mid, from_=-180, to=180, resolution=0.5, orient="horizontal",
                               length=420, bg="#1e1e1e", fg="#eee", highlightthickness=0,
                               troughcolor="#333")
        self.slider.grid(row=0, column=1, padx=6)
        self.target_deg = tk.Label(mid, text="0.0", fg="#eee", bg="#141414",
                                   font=("Menlo", 12), width=7)
        self.target_deg.grid(row=0, column=2)
        tk.Button(mid, text="-5°", command=lambda: self.jog(-5)).grid(row=0, column=3)
        tk.Button(mid, text="+5°", command=lambda: self.jog(5)).grid(row=0, column=4)
        tk.Button(mid, text="设为当前", command=self.set_current).grid(row=0, column=5, padx=4)

        ctl = tk.Frame(root, bg="#141414")
        ctl.pack(fill="x", padx=10)
        tk.Label(ctl, text="Kp", fg="#bbb", bg="#141414").grid(row=0, column=0)
        self.kp = tk.Entry(ctl, width=6)
        self.kp.insert(0, "3")
        self.kp.grid(row=0, column=1, padx=4)
        tk.Label(ctl, text="Kd", fg="#bbb", bg="#141414").grid(row=0, column=2)
        self.kd = tk.Entry(ctl, width=6)
        self.kd.insert(0, "0.3")
        self.kd.grid(row=0, column=3, padx=4)
        tk.Button(ctl, text="使能并保持", command=self.enable).grid(row=0, column=4, padx=6)
        tk.Button(ctl, text="失能", command=self.disable).grid(row=0, column=5)
        tk.Button(ctl, text="急停", command=self.disable, bg="#822", fg="#fff").grid(row=0, column=6, padx=8)

        self.fb = tk.Label(root, text="", fg="#8cf", bg="#141414",
                           font=("Menlo", 13), anchor="w", justify="left")
        self.fb.pack(fill="x", padx=12, pady=8)
        self.log = tk.Text(root, height=8, bg="#0b0b0b", fg="#ccc",
                           font=("Menlo", 10))
        self.log.pack(fill="both", expand=True, padx=10, pady=8)
        root.protocol("WM_DELETE_WINDOW", self.quit)
        root.after(30, self.tick)
        if auto:
            self.open_can()

    def _say(self, s: str):
        self.log.insert("end", s + "\n")
        self.log.see("end")

    def open_can(self):
        if self.bus is not None:
            return
        try:
            self.bus = usb2can_classic.Usb2CanClassic(self.port.get().strip(), 921600)
            self.bus.open(1_000_000)
            self.motor = Damiao(self.bus, dm_can.MotorCfg(id=int(self.mid.get())))
            self.motor.disable()
            time.sleep(0.05)
            self.motor.command(0, 0, 0, 0, 0)
            t0 = time.time()
            while self.motor.fb is None and time.time() - t0 < 1:
                self.motor.poll()
                time.sleep(0.01)
            self._say(f"CAN 已打开，ID={self.mid.get()}，反馈={self.motor.fb}")
        except Exception as exc:  # noqa: BLE001
            self.bus = None
            self.motor = None
            self._say(f"打开失败：{exc}")

    def close_can(self):
        self.disable()
        if self.bus is not None:
            self.bus.close()
        self.bus = None
        self.motor = None
        self.state.configure(text="未连接")

    def enable(self):
        if self.motor is None:
            return
        fb = self.motor.fb
        if fb is not None:
            self.slider.set(max(-180, min(180, math.degrees(fb["pos"]))))
        self.motor.enable()
        self.enabled = True
        self._say("已使能，目标=当前位置")

    def disable(self):
        if self.motor is not None:
            self.motor.disable()
        self.enabled = False

    def set_current(self):
        if self.motor is not None and self.motor.fb is not None:
            self.slider.set(max(-180, min(180, math.degrees(self.motor.fb["pos"]))))

    def jog(self, d: float):
        self.slider.set(max(-180, min(180, self.slider.get() + d)))

    def tick(self):
        if self.motor is not None:
            self.motor.poll()
            fb = self.motor.fb
            if fb is not None:
                self.state.configure(
                    text=f"pos={fb['pos']:+.3f} rad  vel={fb['vel']:+.2f}  "
                         f"tau={fb['torque']:+.2f}  T={fb['temp_mos']}/{fb['temp_rotor']}℃  "
                         f"state={fb['state']:#04x} {dm_can.err_name(fb['err'])}")
                self.target_deg.configure(text=f"{self.slider.get():.1f}")
                if self.enabled:
                    try:
                        kp = float(self.kp.get())
                        kd = float(self.kd.get())
                    except ValueError:
                        kp, kd = 0.0, 0.0
                    target = math.radians(self.slider.get())
                    self.motor.command(target, 0.0, kp, kd, 0.0)
            else:
                self.state.configure(text="已连接，无反馈")
        if not self.closing:
            self.root.after(30, self.tick)

    def quit(self):
        self.closing = True
        self.close_can()
        self.root.destroy()


def headless(seconds: float) -> int:
    bus = usb2can_classic.Usb2CanClassic(DEFAULT_PORT, 921600)
    bus.open(1_000_000)
    motor = Damiao(bus, dm_can.MotorCfg(id=DEFAULT_ID))
    motor.disable()
    time.sleep(0.05)
    motor.command(0, 0, 0, 0, 0)
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            motor.poll()
            if motor.fb is not None:
                f = motor.fb
                print(f"pos={f['pos']:+.3f} vel={f['vel']:+.2f} tau={f['torque']:+.2f} "
                      f"T={f['temp_mos']}/{f['temp_rotor']} state={f['state']:#04x}")
            time.sleep(0.1)
    finally:
        motor.disable()
        bus.close()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--auto", action="store_true")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args()
    if args.headless:
        return headless(args.seconds)
    import tkinter as tk

    root = tk.Tk()
    Panel(root, auto=args.auto)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
