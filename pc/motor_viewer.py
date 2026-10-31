"""订阅 MQTT，显示 6 关节目标/实际；当前只有单电机 J0。"""

from __future__ import annotations

import argparse
import math
import threading
import time

import joint_map
import mqtt_link
import payload


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.target = [0.0] * 6
        self.actual = [0.0] * 6
        self.vel = [0.0] * 6
        self.torque = [0.0] * 6
        self.temp = [0] * 6
        self.err = [0] * 6
        self.seen = False

    def on_control(self, data: bytes) -> None:
        up = payload.decode_uplink(data)
        if up is None:
            return
        jm = joint_map.JointMap()
        with self.lock:
            self.target = [jm.to_joint(x) for x in up.joints]

    def on_telemetry(self, data: bytes) -> None:
        d = payload.decode_downlink(data)
        if d is None:
            return
        with self.lock:
            self.actual = [j.pos_urad / 1e6 for j in d.joint]
            self.vel = [j.vel_crad / 100 for j in d.joint]
            self.torque = [j.torque_cnm / 100 for j in d.joint]
            self.temp = [j.temp_c for j in d.joint]
            self.err = [j.err for j in d.joint]
            self.seen = True

    def snapshot(self):
        with self.lock:
            return (list(self.target), list(self.actual), list(self.vel),
                    list(self.torque), list(self.temp), self.seen)


class Viewer:
    def __init__(self, root, state: State):
        import tkinter as tk

        self.tk = tk
        self.root = root
        self.state = state
        root.title("电机 / 机械臂可视化")
        root.geometry("860x430")
        root.configure(bg="#141414")
        self.canvas = tk.Canvas(root, width=840, height=360, bg="#141414",
                                highlightthickness=0)
        self.canvas.pack(padx=10, pady=(10, 0))
        self.status = tk.Label(root, text="等待 MQTT...", fg="#9ad", bg="#141414",
                               font=("Menlo", 12), anchor="w")
        self.status.pack(fill="x", padx=12, pady=6)
        self.bars, self.lines, self.texts = [], [], []
        for i in range(6):
            y = 20 + i * 52
            self.canvas.create_text(20, y + 14, text=f"J{i}", fill="#eee",
                                    font=("Menlo", 13, "bold"))
            self.canvas.create_rectangle(60, y, 700, y + 28, outline="#333", fill="#1e1e1e")
            bar = self.canvas.create_rectangle(380, y, 380, y + 28, outline="", fill="#3fa")
            line = self.canvas.create_line(380, y, 380, y + 28, fill="#fa3", width=2)
            txt = self.canvas.create_text(830, y + 14, text="--", fill="#eee",
                                          font=("Menlo", 11), anchor="e")
            self.bars.append(bar)
            self.lines.append(line)
            self.texts.append(txt)
        root.after(50, self.tick)

    def tick(self):
        target, actual, vel, torque, temp, seen = self.state.snapshot()
        for i in range(6):
            xt = 380 + max(-1, min(1, target[i] / math.pi)) * 300
            xa = 380 + max(-1, min(1, actual[i] / math.pi)) * 300
            self.canvas.coords(self.bars[i], min(380, xa), 20 + i * 52,
                               max(380, xa), 48 + i * 52)
            self.canvas.coords(self.lines[i], xt, 20 + i * 52, xt, 48 + i * 52)
            if seen:
                self.canvas.itemconfigure(
                    self.texts[i],
                    text=f"{actual[i]:+.2f} / {target[i]:+.2f} rad  "
                         f"τ={torque[i]:+.2f}  {temp[i]}℃")
        self.status.configure(text="目标（黄线）/ 实际（绿条）；J0 为当前单电机" if seen
                              else "等待 CustomControl / CustomByteBlock ...")
        self.root.after(50, self.tick)


def text_loop(state: State, seconds: float | None) -> None:
    t0 = time.time()
    while seconds is None or time.time() - t0 < seconds:
        target, actual, vel, torque, temp, seen = state.snapshot()
        if seen:
            print(f"J0 target={target[0]:+.2f} pos={actual[0]:+.3f} "
                  f"vel={vel[0]:+.2f} tau={torque[0]:+.2f} T={temp[0]}℃")
        else:
            print("等待 MQTT ...")
        time.sleep(0.5)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--mqtt-port", type=int, default=3333)
    ap.add_argument("--id", default="view")
    ap.add_argument("--text", action="store_true")
    ap.add_argument("--seconds", type=float)
    args = ap.parse_args()

    state = State()
    mq = mqtt_link.MqttLink(args.host, args.mqtt_port, args.id)
    mq.handlers[mqtt_link.TOPIC_CONTROL] = state.on_control
    mq.handlers[mqtt_link.TOPIC_BYTE_BLOCK] = state.on_telemetry
    if not mq.start():
        print("MQTT 连接失败", mq.last_error)
        return 1
    try:
        if args.text:
            text_loop(state, args.seconds)
        else:
            import tkinter as tk

            root = tk.Tk()
            Viewer(root, state)
            if args.seconds:
                root.after(int(args.seconds * 1000), root.destroy)
            root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        mq.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
