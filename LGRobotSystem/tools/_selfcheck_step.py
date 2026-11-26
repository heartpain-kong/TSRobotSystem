# -*- coding: utf-8 -*-
"""自检 step_response_6020:合成一条 30° 阶跃(20% 超调)看指标对不对。"""
import math
import sys

sys.path.insert(0, "tools")
import step_response_6020 as S

tgt = math.radians(30.0)
rows = []
for k in range(1000):
    t = 0.001 * k
    # 2 阶欠阻尼响应,ζ=0.5,ωn=20 ⇒ 超调≈16%
    wn, z = 20.0, 0.5
    wd = wn * math.sqrt(1 - z * z)
    pos = tgt * (1 - math.exp(-z * wn * t) * (math.cos(wd * t) + (z * wn / wd) * math.sin(wd * t)))
    rows.append((t, pos, tgt, 0.0))

segs = S.steps(rows)
print("segments:", len(segs))
r = S.report(segs[0])
print({k: round(v, 3) if isinstance(v, float) else v for k, v in r.items()})
assert abs(r["overshoot_pct"] - 16.3) < 1.5, r
assert r["peak_ms"] is not None
print("ok")
