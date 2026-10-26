"""主端舵机 → 关节角的映射（混合方案 C）。

- 默认：把舵机整段行程线性缩放到关节安全范围；
- 可选 wrap：连续旋转模式下对 0/4095 回绕做 unwrap；
- 离合：engage 后输出冻结，release 时以当前位置重对齐（不跳变），之后按相对量移动。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class JointMap:
    prad_min: int = 500
    prad_max: int = 2500
    joint_min: float = -1.5
    joint_max: float = 1.5
    direction: int = 1
    wrap: bool = False

    clutch: bool = False
    _bias: float = 0.0
    _hold: float = 0.0
    _last_raw: int | None = None
    _cont: float = 0.0

    def __post_init__(self) -> None:
        if self.prad_max == self.prad_min:
            raise ValueError("prad_max 不能等于 prad_min")
        if self.joint_max == self.joint_min:
            raise ValueError("joint_max 不能等于 joint_min")
        if self.direction not in (-1, 1):
            raise ValueError("direction 只能是 ±1")

    # ---- 连续化（仅 wrap=True 时对回绕做展开） ----
    def _continuous(self, raw: int) -> float:
        if not self.wrap:
            return float(raw)
        if self._last_raw is None:
            self._last_raw = raw
            self._cont = float(raw)
            return self._cont
        span = self.prad_max - self.prad_min + 1
        d = raw - self._last_raw
        if d > span / 2:
            d -= span
        elif d < -span / 2:
            d += span
        self._cont += d
        self._last_raw = raw
        return self._cont

    def absolute(self, raw: int) -> float:
        """绝对映射：钳位到 [prad_min, prad_max]，再按方向缩放到关节范围。"""
        c = self._continuous(raw)
        u = (c - self.prad_min) / (self.prad_max - self.prad_min)
        if not self.wrap:
            u = max(0.0, min(1.0, u))
        if self.direction < 0:
            u = 1.0 - u
        return self.joint_min + u * (self.joint_max - self.joint_min)

    def to_joint(self, raw: int) -> float:
        if self.clutch:
            return self._hold
        j = self.absolute(raw) - self._bias
        return max(self.joint_min, min(self.joint_max, j))

    # ---- 离合 ----
    def engage(self, raw: int | None = None) -> None:
        if not self.clutch:
            if raw is not None:
                self._hold = self.absolute(raw) - self._bias
            self.clutch = True

    def release(self, raw: int) -> None:
        if not self.clutch:
            return
        self._bias = self.absolute(raw) - self._hold
        self.clutch = False

    def reset(self) -> None:
        self._bias = 0.0
        self._hold = 0.0
        self._last_raw = None
        self._cont = 0.0
        self.clutch = False
