"""6 自由度工业臂的正运动学渲染（只用 Tk，不依赖 ROS/物理引擎）。

构型：J0 底座偏航 → J1 肩俯仰 → J2 肘俯仰 → J3 腕滚转 → J4 腕俯仰 → J5 工具滚转。
只做视觉：输入 6 个关节角，画出连杆；不碰碰撞/动力学。

用法：
    python3 arm_sim.py                 # 独立窗口，手动拖 6 个滑条
    python3 arm_sim.py --demo          # 自动摆动
    python3 arm_sim.py --seconds 3     # 3 秒后自动关闭
"""

from __future__ import annotations

import argparse
import math
import time

L1, L2, L3, L4 = 0.36, 0.30, 0.10, 0.12
BASE_H = 0.16


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0), (s, c, 0), (0, 0, 1))


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0, -s), (0, 1, 0), (s, 0, c))


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return ((1, 0, 0), (0, c, -s), (0, s, c))


def _mm(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def _mv(m, v):
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def _add(p, q):
    return (p[0] + q[0], p[1] + q[1], p[2] + q[2])


def fk(q: list[float]) -> list[tuple[float, float, float]]:
    """返回 6 个关节点：底座、肩、肘、腕1、腕2、工具端点。"""
    m0 = _rz(q[0])
    p0 = (0.0, 0.0, 0.0)
    p1 = (0.0, 0.0, BASE_H)
    p2 = _add(p1, _mv(_mm(m0, _ry(q[1])), (L1, 0, 0)))
    p3 = _add(p2, _mv(_mm(m0, _ry(q[1] + q[2])), (L2, 0, 0)))
    mw = _mm(_mm(_mm(m0, _ry(q[1] + q[2])), _rx(q[3])), _ry(q[4]))
    p4 = _add(p3, _mv(mw, (L3, 0, 0)))
    p5 = _add(p4, _mv(_mm(mw, _rz(q[5])), (L4, 0, 0)))
    return [p0, p1, p2, p3, p4, p5]


def project(p, yaw=-0.6, pitch=0.45, scale=520, cx=330, cy=330):
    x, y, z = p
    x1 = x * math.cos(yaw) - y * math.sin(yaw)
    y1 = x * math.sin(yaw) + y * math.cos(yaw)
    sx = cx + x1 * scale
    sy = cy - (z * math.cos(pitch) - y1 * math.sin(pitch)) * scale
    return sx, sy


class ArmView:
    def __init__(self, master, state=None, demo=False):
        import tkinter as tk

        self.tk = tk
        self.state = state
        self.demo = demo
        self.win = tk.Toplevel(master) if master is not None else tk.Tk()
        self.win.title("6 轴机械臂 · 正运动学仿真")
        self.win.geometry("980x720")
        self.win.configure(bg="#101418")
        self.canvas = tk.Canvas(self.win, width=660, height=700, bg="#101418",
                                highlightthickness=0)
        self.canvas.pack(side="left")
        side = tk.Frame(self.win, bg="#101418")
        side.pack(side="right", fill="y", padx=8)
        self.sliders = []
        for i in range(6):
            tk.Label(side, text=f"J{i}", fg="#bbb", bg="#101418").pack(anchor="w")
            s = tk.Scale(side, from_=-180, to=180, orient="horizontal", length=260,
                         resolution=1, bg="#1e1e1e", fg="#eee", highlightthickness=0,
                         troughcolor="#333")
            s.set(0)
            s.pack(pady=(0, 4))
            self.sliders.append(s)
        self.follow = tk.BooleanVar(value=state is not None)
        if state is not None:
            tk.Checkbutton(side, text="跟随客户端数据", variable=self.follow,
                           fg="#bbb", bg="#101418", selectcolor="#333").pack(anchor="w", pady=6)
        self.t0 = time.time()
        self.win.after(40, self.tick)

    def _angles(self):
        if self.demo:
            t = time.time() - self.t0
            return [0.5 * math.sin(t * 0.8), 0.4 * math.sin(t * 1.1 + 1),
                    0.3 * math.sin(t * 0.9 + 2), 0.4 * math.sin(t * 1.3),
                    0.3 * math.sin(t * 1.0 + 1), 0.5 * math.sin(t * 1.2)]
        if self.state is not None and self.follow.get():
            with self.state.lock:
                return list(self.state.target)
        return [math.radians(s.get()) for s in self.sliders]

    def tick(self):
        q = self._angles()
        if self.state is not None and self.follow.get():
            for s, a in zip(self.sliders, q):
                s.set(round(math.degrees(a)))
        pts = fk(q)
        c = self.canvas
        c.delete("all")
        # 地面网格
        for i in range(-3, 4):
            x1, y1 = project((i * 0.2, -0.6, 0))
            x2, y2 = project((i * 0.2, 0.6, 0))
            c.create_line(x1, y1, x2, y2, fill="#1c2830")
        for i in range(-3, 4):
            x1, y1 = project((-0.6, i * 0.2, 0))
            x2, y2 = project((0.6, i * 0.2, 0))
            c.create_line(x1, y1, x2, y2, fill="#1c2830")
        # 底座 + 连杆
        c.create_oval(project((-0.16, -0.16, 0))[0], project((-0.16, 0.16, 0))[1],
                      project((0.16, 0.16, 0))[0], project((0.16, -0.16, 0))[1],
                      fill="#2a3a44", outline="#4a6a7a")
        colors = ["#3fa", "#5cf", "#5cf", "#fc6", "#fc6", "#f88"]
        for i in range(5):
            x1, y1 = project(pts[i])
            x2, y2 = project(pts[i + 1])
            c.create_line(x1, y1, x2, y2, fill=colors[i], width=12 if i < 3 else 7,
                          capstyle="round")
            c.create_oval(x1 - 7, y1 - 7, x1 + 7, y1 + 7, fill="#101418",
                          outline=colors[i], width=2)
        x, y = project(pts[5])
        c.create_oval(x - 8, y - 8, x + 8, y + 8, fill="#f88", outline="#fdd")
        c.create_text(10, 10, anchor="nw", fill="#889",
                      text="J0 yaw · J1/J2 pitch · J3/J4/J5 wrist")
        self.win.after(40, self.tick)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--seconds", type=float)
    args = ap.parse_args()
    view = ArmView(None, demo=args.demo)
    if args.seconds:
        view.win.after(int(args.seconds * 1000), view.win.destroy)
    view.win.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
