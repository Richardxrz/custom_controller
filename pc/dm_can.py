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
    t_min: float = -28.0
    t_max: float = 28.0


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
    if len(data) != 8:
        return None
    p = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    t = ((data[4] & 0x0F) << 8) | data[5]
    return {
        "id": cfg.id,
        "state": data[0],
        "ctrl_id": data[0] & 0x0F,
        "err": data[0] >> 4,
        "temp_mos": data[6],
        "temp_rotor": data[7],
        "pos": uint_to_float(p, cfg.p_min, cfg.p_max, 16),
        "vel": uint_to_float(v, cfg.v_min, cfg.v_max, 12),
        "torque": uint_to_float(t, cfg.t_min, cfg.t_max, 12),
    }


ERR_NAMES = {
    0x0: "disabled",
    0x1: "enabled",
    0x8: "overvoltage",
    0x9: "undervoltage",
    0xA: "overcurrent",
    0xB: "mos_overtemp",
    0xC: "coil_overtemp",
    0xD: "comm_lost",
    0xE: "overload",
}


def err_name(err: int) -> str:
    return ERR_NAMES.get(err, "unknown")


def special(kind: str) -> bytes:
    tail = {"enable": 0xFC, "disable": 0xFD, "zero": 0xFE, "clear": 0xFB}.get(kind, 0xFD)
    return bytes([0xFF] * 7 + [tail])
