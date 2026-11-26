# -*- coding: utf-8 -*-
"""把 MOTOR_PID 帧解成逐通道 CSV,并做统计。"""
import sys
from analyze_motor_pid_log import frames, decode, f32

KEYS = ["pos", "pos_tgt", "pos_err", "tgt_rpm", "fb_rpm", "err_rpm", "out",
        "m201_pos", "m201_w", "m201_t", "m203_pos", "m203_w", "m204_pos", "m204_w",
        "m204_t", "m204_tgt_rpm", "m204_pos_tgt", "m204_pos_err",
        "m6020_pos", "m6020_pos_tgt", "m6020_w", "m6020_w_cmd", "m6020_t", "m6020_online",
        "yaw", "gz", "pitch", "roll"]


def load(path):
    rows = []
    for t, b in frames(path):
        d = decode(b[8:-2])
        d["t"] = t
        rows.append(d)
    return rows


def stats(rows, path):
    print("=" * 90)
    print(path, "frames=%d" % len(rows))
    if not rows:
        return
    print("time", rows[0]["t"], "->", rows[-1]["t"])
    for k in KEYS:
        vs = [r[k] for r in rows]
        mn, mx = min(vs), max(vs)
        if abs(mn) + abs(mx) < 1e-9:
            continue
        nz = sum(1 for v in vs if abs(v) > 1e-9)
        print("  %-14s min=%9.4f max=%9.4f nz=%d" % (k, mn, mx, nz))
    print("  flags non-zero:", sum(1 for r in rows if r["flags"]))
    print("  m204_f non-zero:", sum(1 for r in rows if r["m204_f"]))


def dump(rows, out):
    with open(out, "w", encoding="utf-8") as f:
        f.write("t," + ",".join(KEYS) + "\n")
        for r in rows:
            f.write(r["t"] + "," + ",".join("%.6f" % r[k] for k in KEYS) + "\n")
    print("wrote", out)


if __name__ == "__main__":
    path = sys.argv[1]
    rows = load(path)
    stats(rows, path)
    if len(sys.argv) > 2:
        dump(rows, sys.argv[2])
