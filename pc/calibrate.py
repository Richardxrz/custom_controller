"""标定主端舵机行程，生成 joint_map 配置。

用法：
    # 录一段：手扳舵机从一头到另一头
    python3 calibrate.py --record --port /dev/tty.usbserial-1110 --id 5 \
        --seconds 10 --out-csv calib.csv --out-json joint.json

    # 只分析已有 CSV（第一列可带时间，最后一列是 PRAD）
    python3 calibrate.py --csv calib.csv --joint-min -1.5 --joint-max 1.5
"""

from __future__ import annotations

import argparse
import json
import time


def analyze(samples: list[int], pct: float = 2.0) -> dict:
    if not samples:
        raise ValueError("没有采样")
    s = sorted(samples)
    n = len(s)
    lo = min(n - 1, max(0, int(n * pct / 100.0)))
    hi = max(0, n - 1 - lo)
    return {
        "n": n,
        "min": s[0],
        "max": s[-1],
        "p2": s[lo],
        "p98": s[hi],
        "span": s[hi] - s[lo],
        "suggest_prad_min": s[lo],
        "suggest_prad_max": s[hi],
    }


def parse_csv(path: str) -> list[int]:
    out: list[int] = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line[0].isalpha():
                continue
            parts = line.replace(",", " ").split()
            if not parts:
                continue
            try:
                out.append(int(float(parts[-1])))
            except ValueError:
                continue
    return out


def record(port: str, sid: int, seconds: float, baud: int = 115200) -> list[int]:
    from servo_link import ServoLink

    link = ServoLink(port, [sid], baud=baud)
    samples: list[int] = []
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            raw = link.read_angle(sid)
            if raw is not None:
                samples.append(raw)
            time.sleep(0.002)
    finally:
        link.close()
    return samples


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv")
    ap.add_argument("--record", action="store_true")
    ap.add_argument("--port")
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--out-csv")
    ap.add_argument("--out-json")
    ap.add_argument("--joint-min", type=float, default=-1.5)
    ap.add_argument("--joint-max", type=float, default=1.5)
    ap.add_argument("--direction", type=int, default=1, choices=[-1, 1])
    ap.add_argument("--wrap", action="store_true")
    args = ap.parse_args()

    if args.record:
        if not args.port:
            ap.error("--record 需要 --port")
        samples = record(args.port, args.id, args.seconds, args.baud)
        if args.out_csv:
            with open(args.out_csv, "w", encoding="utf-8") as f:
                f.write("t,prad\n")
                for i, v in enumerate(samples):
                    f.write(f"{i},{v}\n")
    elif args.csv:
        samples = parse_csv(args.csv)
    else:
        ap.error("需要 --record 或 --csv")

    st = analyze(samples)
    cfg = {
        "prad_min": st["suggest_prad_min"],
        "prad_max": st["suggest_prad_max"],
        "joint_min": args.joint_min,
        "joint_max": args.joint_max,
        "direction": args.direction,
        "wrap": bool(args.wrap),
    }
    print(json.dumps({"stats": st, "config": cfg}, ensure_ascii=False, indent=2))
    if args.out_json:
        with open(args.out_json, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
