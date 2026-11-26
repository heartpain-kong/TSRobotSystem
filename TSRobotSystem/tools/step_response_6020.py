# -*- coding: utf-8 -*-
"""从 MOTOR_PID 日志里量 6020 位置阶跃:超调 / 峰值时间 / 建立时间。

用法:
    python tools/step_response_6020.py F:\\桌面\\mcu_raw_xxx.csv

前提:固件是 232B payload 版本(0x0102 帧 204 起带 6020 pos/target)。
"""
import math
import sys

from analyze_motor_pid_log import frames, decode


def load(path):
    rows = []
    for t, b in frames(path):
        d = decode(b[8:-2])
        if d.get("m6020_online", 0) & 0x02:
            rows.append((t, d["m6020_pos"], d["m6020_pos_tgt"], d["m6020_w"]))
    return rows


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def steps(rows, min_step=0.05):
    """按目标变化切段;返回每段的 (t0, target, 采样列表)。"""
    segs = []
    cur = None
    tgt_prev = rows[0][2]
    for t, pos, tgt, w in rows:
        if wrap(tgt - tgt_prev) != 0.0 and abs(wrap(tgt - tgt_prev)) > 1e-9:
            tgt_prev = tgt
        if cur is None or abs(wrap(tgt - cur["tgt"])) > 1e-6:
            # 目标变了 ⇒ 新段;但先把上一段的稳态角找出来
            if cur is not None:
                cur["end"] = pos
                segs.append(cur)
            cur = {"t0": t, "tgt": tgt, "start": pos, "samples": []}
        cur["samples"].append((t, pos))
    if cur is not None:
        cur["end"] = rows[-1][1]
        segs.append(cur)
    return [s for s in segs if abs(wrap(s["tgt"] - s["start"])) > min_step]


def report(seg):
    tgt, start = seg["tgt"], seg["start"]
    delta = wrap(tgt - start)          # 需要走的角度(带符号)
    sign = 1.0 if delta >= 0 else -1.0
    band = max(0.01, 0.05 * abs(delta))  # 5% 或 0.01rad
    peak_err = 0.0
    t_peak = None
    t_settle = None
    for t, pos in seg["samples"]:
        err = wrap(pos - tgt) * sign     # >0 = 冲过头
        if err > peak_err:
            peak_err = err
            t_peak = t
        if abs(wrap(tgt - pos)) > band:
            t_settle = t
    return {
        "t0": seg["t0"], "step_deg": math.degrees(delta),
        "overshoot_pct": (peak_err / abs(delta) * 100.0) if delta else 0.0,
        "peak_ms": (t_peak - seg["t0"]) if t_peak else None,
        "settle_ms": (t_settle - seg["t0"]) if t_settle else None,
    }


def main(path):
    rows = load(path)
    print("6020 在线样本: %d" % len(rows))
    if not rows:
        print("没有 6020 数据:抓的是旧 204B 帧,或 6020 没上线(can.id205=0)")
        return 2
    print("时间 %s -> %s" % (rows[0][0], rows[-1][0]))
    segs = steps(rows)
    if not segs:
        print("没检测到位置阶跃(目标没变化)")
        return 0
    print("%-12s %10s %12s %10s %11s" % ("t0", "step[deg]", "overshoot[%]", "peak[ms]", "settle[ms]"))
    for s in segs:
        r = report(s)
        print("%-12s %10.2f %12.1f %10s %11s" % (
            r["t0"], r["step_deg"], r["overshoot_pct"],
            "%.0f" % r["peak_ms"] if r["peak_ms"] is not None else "-",
            "%.0f" % r["settle_ms"] if r["settle_ms"] is not None else "-"))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
