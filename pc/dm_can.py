"""达妙（DM）MIT 帧的 Python 版，与固件 Module/dm_can.c 逐字节一致。"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class MotorCfg:
    id: int = 1
    p_min: float = -12.5
    p_max: float = 12.5
    v_min: float = -30.0
    v_max: float = 30.0
    t_min: float = -10.0
    t_max: float = 10.0


def float_to_uint(x: float, lo: float, hi: float, bits: int) -> int:
    x = max(lo, min(hi, x))
    span = hi - lo
    if span <= 0:
        return 0
    return int((x - lo) / span * ((1 << bits) - 1))


def uint_to_float(u: int, lo: float, hi: float, bits: int) -> float:
    return u / ((1 << bits) - 1) * (hi - lo) + lo


def mit_pack(cfg: MotorCfg, pos: float, vel: float, kp: float, kd: float, torque: float) -> bytes:
    p = float_to_uint(pos, cfg.p_min, cfg.p_max, 16)
    v = float_to_uint(vel, cfg.v_min, cfg.v_max, 12)
    kpi = float_to_uint(kp, 0.0, 500.0, 12)
    kdi = float_to_uint(kd, 0.0, 5.0, 12)
    t = float_to_uint(torque, cfg.t_min, cfg.t_max, 12)
    return bytes(
        [
            (p >> 8) & 0xFF,
            p & 0xFF,
            (v >> 4) & 0xFF,
            ((v & 0x0F) << 4) | ((kpi >> 8) & 0x0F),
            kpi & 0xFF,
            (kdi >> 4) & 0xFF,
            ((kdi & 0x0F) << 4) | ((t >> 8) & 0x0F),
            t & 0xFF,
        ]
    )


def feedback_decode(cfg: MotorCfg, data: bytes) -> dict | None:
    if len(data) != 8 or data[0] != cfg.id:
        return None
    p = (data[2] << 8) | data[3]
    v = (data[4] << 8) | data[5]
    t = (data[6] << 8) | data[7]
    return {
        "id": data[0],
        "state": data[1],
        "pos": uint_to_float(p, cfg.p_min, cfg.p_max, 16),
        "vel": uint_to_float(v, cfg.v_min, cfg.v_max, 12),
        "torque": uint_to_float(t, cfg.t_min, cfg.t_max, 12),
    }


def special(kind: str) -> bytes:
    tail = {"enable": 0xFC, "disable": 0xFD, "zero": 0xFE, "clear": 0xFB}.get(kind, 0xFD)
    return bytes([0xFF] * 7 + [tail])
