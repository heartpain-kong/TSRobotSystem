# -*- coding: utf-8 -*-
"""烧录后自检:抓 COM 口 10s,统计 0x0102 帧长度/内容 + JSON 行。"""
import json
import struct
import sys
import time

import serial

from analyze_motor_pid_log import decode

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM30"
SEC = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

buf = bytearray()
ser = serial.Serial(PORT, 921600, timeout=0.2)
ser.reset_input_buffer()
t0 = time.time()
while time.time() - t0 < SEC:
    chunk = ser.read(ser.in_waiting or 1)
    if chunk:
        buf.extend(chunk)
ser.close()

print("bytes:", len(buf))

# JSON 文本行
texts = []
i = 0
while True:
    j = buf.find(b"[LOG] JSON ", i)
    if j < 0:
        break
    k = buf.find(b"\n", j)
    if k < 0:
        break
    texts.append(buf[j:k].decode("ascii", "replace"))
    i = k + 1
print("json lines:", len(texts))
for t in texts[-2:]:
    try:
        d = json.loads(t[len("[LOG] JSON "):])
        keys = ["m202.online", "m202.pos", "m204.online", "m204.pos",
                "m6020.online", "m6020.pos", "m6020.pos_target",
                "can.id201", "can.id202", "can.id203", "can.id204", "can.id205",
                "err.code", "err.level"]
        print({k: d.get(k) for k in keys})
    except Exception as e:
        print("json parse fail", e)

# 0x0102 帧
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

print("pid frames:", len(frames))
if frames:
    lens = sorted({len(f) for f in frames})
    print("frame lens:", lens)
    d = decode(frames[-1][8:-2])
    for k, v in d.items():
        if isinstance(v, float):
            print("  %-14s %.4f" % (k, v))
        else:
            print("  %-14s %s" % (k, v))
