"""PRAD 滤波：Hampel 去野值 + One-Euro 自适应平滑。

用法：
    f = FilterChain()
    out = f.update(prad, dt_s)
"""

from __future__ import annotations

import math
from collections import deque


class Hampel:
    """中值绝对偏差（MAD）去单点野值；窗口未满时原样输出。"""

    def __init__(self, window: int = 5, n_sigma: float = 3.0, min_threshold: float = 3.0):
        self.win = deque(maxlen=window)
        self.n_sigma = n_sigma
        self.min_threshold = min_threshold

    def update(self, x: float) -> float:
        x = float(x)
        self.win.append(x)
        if len(self.win) < self.win.maxlen:
            return x
        vals = sorted(self.win)
        med = vals[len(vals) // 2]
        mad = sorted(abs(v - med) for v in vals)[len(vals) // 2]
        threshold = max(self.n_sigma * 1.4826 * mad, self.min_threshold)
        return med if abs(x - med) > threshold else x


class OneEuro:
    """速度自适应低通：慢速时截止低（平滑），快速时截止高（跟手）。"""

    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.7,
                 d_cutoff: float = 1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x: float | None = None
        self.dx = 0.0

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def update(self, x: float, dt: float) -> float:
        x = float(x)
        if self.x is None:
            self.x = x
            return x
        dt = max(dt, 1e-3)
        dx = (x - self.x) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        self.dx = a_d * dx + (1.0 - a_d) * self.dx
        cutoff = self.min_cutoff + self.beta * abs(self.dx)
        a = self._alpha(cutoff, dt)
        self.x = a * x + (1.0 - a) * self.x
        return self.x


class FilterChain:
    def __init__(self, window: int = 5, min_cutoff: float = 1.0,
                 beta: float = 0.7):
        self.hampel = Hampel(window=window)
        self.one_euro = OneEuro(min_cutoff=min_cutoff, beta=beta)

    def update(self, x: float, dt: float) -> float:
        return self.one_euro.update(self.hampel.update(x), dt)
