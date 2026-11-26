# -*- coding: utf-8 -*-
"""上板烧录后自检:抓 COM 口 5s,解码 0x0102(204B 版)最后一帧。"""
import struct
import sys
import time

import serial

from analyze_motor_pid_log import decode

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM28"
SEC = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

buf = bytearray()
ser = serial.Serial(PORT, 921600, timeout=0.2)
ser.reset_input_buffer()
t0 = time.time()
while time.time() - t0 < SEC:
    chunk = ser.read(ser.in_waiting or 1)
    if chunk:
        buf.extend(chunk)
ser.close()

frames = []
i = 0
while i + 10 <= len(buf):
    j = buf.find(b"\x5a", i)
    if j < 0 or j + 4 > len(buf):
        break
    ln = struct.unpack_from("<H", buf, j + 2)[0]
    if ln < 20 or ln > 256:
        i = j + 1
        continue
    if j + ln > len(buf):
        break
    f = bytes(buf[j:j + ln])
    if struct.unpack_from("<H", f, 4)[0] == 0x0102:
        frames.append(f)
    i = j + ln

print("bytes:", len(buf), "pid frames:", len(frames),
      "lens:", sorted({len(f) for f in frames}))
if frames:
    d = decode(frames[-1][8:-2])
    p = frames[-1][8:-2]
    for k in ("roll", "pitch", "yaw", "gx", "gy", "gz", "m6020_online"):
        if k in d:
            v = d[k]
            print("  %-14s %.4f" % (k, v) if isinstance(v, float) else "  %-14s %s" % (k, v))
    print("  err_level=%d err_code=%d fault=0x%08X payload=%dB"
          % (p[40], struct.unpack_from("<H", p, 42)[0],
             struct.unpack_from("<I", p, 44)[0], len(p)))
    if len(p) >= 144:
        print("  friction flags: %d %d" % (p[116], p[117]))
