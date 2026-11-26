# -*- coding: utf-8 -*-
"""自检:新 232B payload 的 6020 块解析 + 旧 204B 日志兼容。"""
import struct
from analyze_motor_pid_log import decode

p = bytearray(232)
struct.pack_into("<f", p, 204, 0.5236)   # pos
struct.pack_into("<f", p, 208, 1.25)     # w
struct.pack_into("<f", p, 212, 0.6)      # target
struct.pack_into("<f", p, 216, 4.8)      # w_cmd
struct.pack_into("<f", p, 220, 0.9)      # T
p[224] = 3
d = decode(bytes(p))
assert abs(d["m6020_pos"] - 0.5236) < 1e-6, d
assert d["m6020_online"] == 3
print("232B ok:", {k: round(v, 3) for k, v in d.items() if k.startswith("m6020")})

d204 = decode(bytes(204))
assert d204["m6020_online"] == 0 and d204["m6020_pos"] == 0.0
print("204B ok")
