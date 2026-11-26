# -*- coding: utf-8 -*-
"""6020 yaw 位置环模型:串级(位置 P + 反馈速度阻尼 → 速度 PI → 力矩)。

比第一版多了三件事,用来复现"抖"和"慢漂":
  1. 速度反馈按 CAN 分辨率量化(1 rpm);
  2. 位置反馈按编码器量化(8192/圈);
  3. 恒定扰动力矩(线缆/摩擦)—— 速度环只阻尼时会慢慢转走。

用法:
    python tools/sim_6020_pos.py            # 跑当前参数 + 对比几组
    python tools/sim_6020_pos.py sweep      # 扫参
"""
import math
import sys

DT = 0.001
RPM2RAD = 2.0 * math.pi / 60.0
QUANT_W = 1.0 * RPM2RAD            # 6020 反馈转速 1 rpm
QUANT_P = 2.0 * math.pi / 8192.0   # 编码器 1 count
TMAX = 2.2                         # 代码里 3 N·m 实际在 out=16384 处先削顶 ≈2.2
WMAX = 10.0
ALPHA = 0.05                       # 阻尼用转速低通


def run(Kp=10.0, Kv=1.0, skp=0.2, ski=0.0003, kiout=0.2,
        J=0.015, b=0.01, dist=0.03, step_deg=30.0, t_end=2.0, wmax=10.0):
    """先做一个 step_deg 阶跃,再在 t_end 时刻切到 hold 目标(rad)并加扰动。"""
    tgt = math.radians(step_deg)
    pos = w = iout = w_filt = 0.0
    w_q_prev = 0.0
    saturated = False
    rows = []
    n = int(t_end / DT)
    for k in range(n):
        t = k * DT
        # ---- 反馈量化(上一拍,模拟 CAN 1ms 延迟)----
        pos_q = round(pos / QUANT_P) * QUANT_P
        w_q = round(w_q_prev / QUANT_W) * QUANT_W
        # ---- 位置环 ----
        e = math.atan2(math.sin(tgt - pos_q), math.cos(tgt - pos_q))
        w_cmd = Kp * e - Kv * w_filt
        if w_cmd > wmax: w_cmd = wmax
        if w_cmd < -wmax: w_cmd = -wmax
        # ---- 速度环 ----
        werr = w_cmd - w_q
        if not saturated:
            iout += ski * werr
            if iout > kiout: iout = kiout
            if iout < -kiout: iout = -kiout
        tq_raw = skp * werr + iout
        tq = max(-TMAX, min(TMAX, tq_raw))
        saturated = (tq != tq_raw)
        # ---- 被控对象 ----
        w += (tq - b * w - dist) / J * DT
        pos += w * DT
        w_filt += ALPHA * (w_q - w_filt)
        w_q_prev = w
        rows.append((t, pos, w, tq, w_cmd))
    return rows


def step_metrics(rows, step_deg):
    tgt = math.radians(step_deg)
    peak = max(r[1] for r in rows)
    over = (peak - tgt) / tgt * 100.0 if tgt else 0.0
    band = 0.02 * abs(tgt)
    settle = 0.0
    for r in rows:
        if abs(r[1] - tgt) > band:
            settle = r[0]
    return over, settle, max(abs(r[2]) for r in rows)


def hold_metrics(rows, t_from):
    seg = [r for r in rows if r[0] >= t_from]
    if not seg:
        return 0.0, 0.0
    tend = seg[-1][1]
    err = [r[1] - tend for r in seg]
    ws = [r[2] for r in seg]
    mean = sum(ws) / len(ws)
    std = math.sqrt(sum((x - mean) ** 2 for x in ws) / len(ws))
    return max(abs(x) for x in err), std


def report(name, Kp, Kv, skp, ski, kiout, J, step_deg=30.0):
    rows = run(Kp, Kv, skp, ski, kiout, J=J, step_deg=step_deg, t_end=2.0)
    over, settle, peakw = step_metrics(rows, step_deg)
    # 0 目标保持 + 0.03Nm 扰动,看漂移/抖
    rows2 = run(Kp, Kv, skp, ski, kiout, J=J, step_deg=0.0, t_end=1.5)
    drift, stdw = hold_metrics(rows2, 0.8)
    print("%-34s J=%.3f  over=%5.1f%%  settle=%5.2fs  peakw=%5.1f  drift=%.3fdeg  jitter=%.3frad/s"
          % (name, J, over, settle, peakw, math.degrees(drift), stdw))


def sweep():
    best = []
    for Kp in (6.0, 8.0, 10.0, 12.0):
        for Kv in (0.6, 0.8, 1.0, 1.2):
            for skp in (0.2, 0.3):
                for ki in (0.0, 0.0002, 0.0004):
                    for kio in (0.15, 0.25):
                        scores = []
                        for J in (0.006, 0.015, 0.03):
                            rows = run(Kp, Kv, skp, ki, kio, J=J, t_end=2.0)
                            over, settle, peakw = step_metrics(rows, 30.0)
                            rows2 = run(Kp, Kv, skp, ki, kio, J=J, step_deg=0.0, t_end=1.2)
                            drift, stdw = hold_metrics(rows2, 0.7)
                            # 约束:超调别大、建立时间别太久、漂移别 >0.8°
                            bad = (over > 3.0) or (settle > 1.4) or (math.degrees(drift) > 0.8)
                            scores.append((bad, settle, over, math.degrees(drift), stdw))
                        bad = any(s[0] for s in scores)
                        score = (sum(s[1] for s in scores) / 3.0
                                 + 0.15 * sum(abs(s[2]) for s in scores) / 3.0
                                 + 8.0 * sum(s[4] for s in scores) / 3.0
                                 + 0.3 * sum(s[3] for s in scores) / 3.0)
                        best.append((bad, score, Kp, Kv, skp, ki, kio, scores))
    best.sort(key=lambda x: (x[0], x[1]))
    print("%-4s %-32s %s" % ("", "gains", "per-J: settle/over/drift/jitter"))
    for bad, score, Kp, Kv, skp, ki, kio, scores in best[:10]:
        detail = " | ".join("s=%.2f o=%+.1f%% d=%.2f° j=%.3f"
                            % (s[1], s[2], s[3], s[4]) for s in scores)
        print("%-4s %-32s score=%.2f  %s"
              % ("BAD" if bad else "", "Kp%g Kv%g skp%g ki%g io%g"
                 % (Kp, Kv, skp, ki, kio), score, detail))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "sweep":
        sweep()
        sys.exit(0)

    cases = [
        ("current: Kp10 Kv1.0 ki.0003 io.2", 10, 1.0, 0.2, 0.0003, 0.2),
        ("A: Kp8  Kv0.8 ki0     io.2", 8, 0.8, 0.2, 0.0, 0.2),
        ("B: Kp6  Kv0.5 ki0     io.2", 6, 0.5, 0.2, 0.0, 0.2),
        ("C: Kp10 Kv1.0 ki0     io.2", 10, 1.0, 0.2, 0.0, 0.2),
        ("D: Kp8  Kv1.0 ki.0002 io.3 skp.3", 8, 1.0, 0.3, 0.0002, 0.3),
        ("E: Kp12 Kv1.5 ki.0003 io.3 skp.3", 12, 1.5, 0.3, 0.0003, 0.3),
    ]
    for J in (0.006, 0.015, 0.03):
        print("--- J = %.3f kg.m^2 ---" % J)
        for c in cases:
            report(*c, J=J)
        print()
