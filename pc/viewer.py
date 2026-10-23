"""上位机显示：6 轴角度实时视图（Tkinter GUI，或 --text 纯终端）。

三种数据源：
    --source master   主端 C 板：UART1 921600，读 0x0302 帧（30 B 载荷）
    --source servo    直连舵机：ZL 转接板 USB，轮询 6 个 ID 的 #IDPRAD!
    --source sim      没硬件：正弦波

用法：
    python3 viewer.py --list
    python3 viewer.py --source servo --port /dev/tty.usbserial-XXXX --ids 1,2,3,4,5,6
    python3 viewer.py --source master --port /dev/tty.usbserial-XXXX
    python3 viewer.py --source sim
    python3 viewer.py --source sim --text --seconds 3
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("缺少 pyserial：pip install pyserial")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import payload  # noqa: E402
import rmproto  # noqa: E402

JOINTS = 6
PWM_MIN, PWM_MAX = 500, 2500


def pwm_to_deg(pwm: int) -> float:
    return (pwm - PWM_MIN) / (PWM_MAX - PWM_MIN) * 270.0


class State:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.joints = [1500] * JOINTS
        self.frames = 0
        self.drops = 0
        self.timeouts = 0
        self.err = ""
        self.t0 = time.time()

    def update(self, joints=None, frame=False, drop=False, timeout=False, err=None) -> None:
        with self.lock:
            if joints is not None:
                self.joints = list(joints)
            if frame:
                self.frames += 1
            if drop:
                self.drops += 1
            if timeout:
                self.timeouts += 1
            if err is not None:
                self.err = err

    def snapshot(self):
        with self.lock:
            return list(self.joints), self.frames, self.drops, self.timeouts, self.err


class Publisher:
    """把当前 6 轴打包成 30 B，按 50 Hz publish CustomControl。"""

    def __init__(self, host: str, port: int, client_id: str):
        import mqtt_link

        self.mqtt_link = mqtt_link
        self.link = mqtt_link.MqttLink(host, port, client_id)
        self.ok = False
        self.count = 0
        self.seq = 0
        self.last = 0.0

    def start(self) -> bool:
        self.ok = self.link.start()
        return self.ok

    def maybe_publish(self, joints: list[int]) -> None:
        if not self.ok:
            return
        now = time.time()
        if now - self.last < 0.02:
            return
        self.last = now
        self.seq = (self.seq + 1) & 0xFF
        import payload

        up = payload.Uplink(seq=self.seq, joints=joints, flags=0x01)
        if self.link.publish_control(payload.encode_uplink(up)):
            self.count += 1


class MasterSource(threading.Thread):
    def __init__(self, port: str, baud: int, state: State):
        super().__init__(daemon=True)
        self.port, self.baud, self.state = port, baud, state
        self.stop_flag = False

    def run(self) -> None:
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.02)
        except Exception as exc:  # noqa: BLE001
            self.state.update(err=f"串口打开失败: {exc}")
            return
        parser = rmproto.Parser()
        last_seq = None
        while not self.stop_flag:
            data = ser.read(256)
            for b in data:
                f = parser.feed(b)
                if f is None or f.cmd_id != 0x0302 or len(f.data) != payload.UPLINK_LEN:
                    continue
                up = payload.decode_uplink(f.data)
                if up is None:
                    self.state.update(err="0x0302 载荷 CRC 错")
                    continue
                lost = 0
                if last_seq is not None:
                    gap = (up.seq - last_seq) & 0xFF
                    if gap > 1:
                        lost = gap - 1
                last_seq = up.seq
                self.state.update(joints=up.joints, frame=True, drop=False)
                for _ in range(lost):
                    self.state.update(drop=True)
        ser.close()

    def stop(self) -> None:
        self.stop_flag = True


class ServoSource(threading.Thread):
    def __init__(self, port: str, baud: int, ids: list[int], state: State):
        super().__init__(daemon=True)
        self.port, self.baud, self.ids, self.state = port, baud, ids, state
        self.stop_flag = False
        self.pattern = re.compile(rb"#(\d{3})P(\d{4})!")

    def run(self) -> None:
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.01)
        except Exception as exc:  # noqa: BLE001
            self.state.update(err=f"串口打开失败: {exc}")
            return
        joints = [1500] * JOINTS
        while not self.stop_flag:
            for i, sid in enumerate(self.ids):
                ser.reset_input_buffer()
                ser.write(f"#{sid:03d}PRAD!".encode())
                buf = ser.read(32)
                matches = self.pattern.findall(buf)
                ok = False
                for mid, val in matches:
                    if int(mid) == sid:
                        joints[i] = int(val)
                        ok = True
                if ok:
                    self.state.update(joints=list(joints), frame=True)
                else:
                    self.state.update(timeout=True)
            time.sleep(0.005)
        ser.close()

    def stop(self) -> None:
        self.stop_flag = True


class SimSource(threading.Thread):
    def __init__(self, state: State):
        super().__init__(daemon=True)
        self.state = state
        self.stop_flag = False
        self.t0 = time.time()

    def run(self) -> None:
        while not self.stop_flag:
            t = time.time() - self.t0
            joints = [int(1500 + 400 * math.sin(t * 1.2 + i * 0.7)) for i in range(JOINTS)]
            self.state.update(joints=joints, frame=True)
            time.sleep(0.01)

    def stop(self) -> None:
        self.stop_flag = True


class Viewer:
    def __init__(self, root, state: State, publisher=None):
        import tkinter as tk

        self.tk = tk
        self.root = root
        self.state = state
        self.publisher = publisher
        root.title("自定义控制器 · 6 轴角度")
        root.geometry("820x420")
        root.configure(bg="#141414")

        self.canvas = tk.Canvas(root, width=800, height=330, bg="#141414",
                                highlightthickness=0)
        self.canvas.pack(padx=10, pady=(10, 0))
        self.status = tk.Label(root, text="", fg="#9ad", bg="#141414",
                               font=("Menlo", 12), anchor="w")
        self.status.pack(fill="x", padx=12, pady=6)

        self.bars = []
        self.values = []
        for i in range(JOINTS):
            y = 20 + i * 50
            self.canvas.create_text(20, y + 12, text=f"J{i}", fill="#eee",
                                    font=("Menlo", 13, "bold"))
            self.canvas.create_rectangle(60, y, 640, y + 24, outline="#333", fill="#1e1e1e")
            bar = self.canvas.create_rectangle(60, y, 60, y + 24, outline="", fill="#3fa")
            val = self.canvas.create_text(760, y + 12, text="--", fill="#eee",
                                          font=("Menlo", 13), anchor="e")
            self.bars.append(bar)
            self.values.append(val)

        self.t0 = time.time()
        self.last_frames = 0
        self.fps = 0.0
        root.after(33, self.tick)

    def tick(self) -> None:
        joints, frames, drops, timeouts, err = self.state.snapshot()
        if self.publisher is not None:
            self.publisher.maybe_publish(joints)
        now = time.time()
        if now - self.t0 >= 1.0:
            self.fps = (frames - self.last_frames) / (now - self.t0)
            self.last_frames = frames
            self.t0 = now
        for i, pwm in enumerate(joints):
            x = 60 + (pwm - PWM_MIN) / (PWM_MAX - PWM_MIN) * 580
            x = max(60, min(640, x))
            self.canvas.coords(self.bars[i], 60, 20 + i * 50, x, 44 + i * 50)
            self.canvas.itemconfigure(self.values[i],
                                      text=f"{pwm:4d}  {pwm_to_deg(pwm):6.1f}°")
        status = f"{self.fps:5.1f} 帧/s   总帧 {frames}   丢帧 {drops}   超时 {timeouts}"
        if self.publisher is not None:
            status += f"   MQTT {self.publisher.count}"
        if err:
            status += f"   [{err}]"
        self.status.configure(text=status)
        self.root.after(33, self.tick)


def text_loop(state: State, seconds: float | None, publisher=None) -> None:
    t0 = time.time()
    while seconds is None or time.time() - t0 < seconds:
        joints, frames, drops, timeouts, err = state.snapshot()
        if publisher is not None:
            publisher.maybe_publish(joints)
        line = "  ".join(f"J{i}:{pwm:4d}({pwm_to_deg(pwm):5.1f}°)" for i, pwm in enumerate(joints))
        sys.stdout.write("\r" + line + f"  frames={frames} drops={drops} to={timeouts} {err}")
        sys.stdout.flush()
        time.sleep(0.05)
    print()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", choices=["master", "servo", "sim"], default="master")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int)
    ap.add_argument("--ids", default="1,2,3,4,5,6")
    ap.add_argument("--text", action="store_true", help="不弹窗，终端打印")
    ap.add_argument("--seconds", type=float)
    ap.add_argument("--list", action="store_true", help="列出串口后退出")
    ap.add_argument("--mqtt-host", help="可选：同时发布 CustomControl 到 MQTT")
    ap.add_argument("--mqtt-port", type=int, default=3333)
    ap.add_argument("--mqtt-id", default="1")
    args = ap.parse_args()

    if args.list:
        for p in list_ports.comports():
            print(f"{p.device:30s} {p.description}  [{p.hwid}]")
        return 0

    state = State()
    if args.source == "sim":
        src = SimSource(state)
    elif args.source == "servo":
        if not args.port:
            print("--source servo 需要 --port（ZL 转接板的 USB 串口）")
            return 1
        ids = [int(x) for x in args.ids.split(",")]
        src = ServoSource(args.port, args.baud or 115200, ids, state)
    else:
        if not args.port:
            print("--source master 需要 --port（USB-TTL 的串口）")
            return 1
        src = MasterSource(args.port, args.baud or 921600, state)

    publisher = None
    if args.mqtt_host:
        publisher = Publisher(args.mqtt_host, args.mqtt_port, args.mqtt_id)
        if not publisher.start():
            print("MQTT 连接失败，继续只显示")
            publisher = None

    src.start()
    try:
        if args.text:
            text_loop(state, args.seconds, publisher)
        else:
            import tkinter as tk

            root = tk.Tk()
            Viewer(root, state, publisher)
            if args.seconds:
                root.after(int(args.seconds * 1000), root.destroy)
            root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        src.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
