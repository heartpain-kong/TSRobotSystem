# -*- coding: utf-8 -*-
"""capture_steer_full.py -- 上板 ch2 + 下板 seat/202/204/6020 + 航向诊断

帧 0x0104 (payload 46 起，旧固件 38):
    0/4/8/12  seat0 now/tgt, seat1 now/tgt
    16..33    flags, t201, a201, flags, t203, a203
    34        6020 pos
    38        heading error [rad]
    42        omega actually fed to kinematics [rad/s]
帧 0x0102 payload 204 = 上板(RC Ch[2] @96), 242 = 下板(202@26 204@180 6020@204)
"""
import math
import os
import struct
import sys
import time

import serial

SEC = float(sys.argv[1]) if len(sys.argv) > 1 else 14.0
CH_FULL, DZ, MAX_VX = 660.0, 0.05, 3.0


def crc(data, c=0):
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0x8408 if (c & 1) else (c >> 1)
    return c & 0xFFFF


def f32(b, o):
    return struct.unpack_from("<f", b, o)[0]


def i16(b, o):
    return struct.unpack_from("<h", b, o)[0]


def vx_of(ch):
    v = max(-1.0, min(1.0, ch / CH_FULL))
    return 0.0 if abs(v) < DZ else v * MAX_VX


def parse(buf, ev, now):
    i, n = 0, len(buf)
    while i + 10 <= n:
        j = buf.find(b"\x5a", i)
        if j < 0 or j + 10 > n:
            return i if j < 0 else j
        ln = struct.unpack_from("<H", buf, j + 2)[0]
        cmd = struct.unpack_from("<H", buf, j + 4)[0]
        if cmd == 0x0102 and 200 <= ln <= 300:
            tot = ln
        elif cmd == 0x0104 and 38 <= ln <= 80:
            tot = 8 + ln + 2
        else:
            i = j + 1
            continue
        if j + tot > n:
            return j
        fr = buf[j:j + tot]
        if crc(fr[:-2]) != struct.unpack_from("<H", fr, tot - 2)[0]:
            i = j + 1
            continue
        pl = fr[8:-2]
        i = j + tot
        if cmd == 0x0104:
            d = {"s0n": f32(pl, 0), "s0t": f32(pl, 4), "s1n": f32(pl, 8), "s1t": f32(pl, 12),
                 "t201": f32(pl, 17), "a201": f32(pl, 21),
                 "t203": f32(pl, 26), "a203": f32(pl, 30), "p6": f32(pl, 34)}
            if len(pl) >= 46:
                d["her"] = f32(pl, 38)
                d["wcmd"] = f32(pl, 42)
            ev.append((now, "steer", d))
        elif len(pl) == 204:
            ev.append((now, "up", {"ch2": i16(pl, 96), "rc": pl[84]}))
        elif len(pl) == 242:
            ev.append((now, "low", {"m202": f32(pl, 26), "m204": f32(pl, 180), "p6": f32(pl, 204)}))
    return i


ser = serial.Serial("COM37", 921600, timeout=0.1)
ser.reset_input_buffer()
buf, ev = bytearray(), []
t0 = time.time()
while time.time() - t0 < SEC:
    ch = ser.read(ser.in_waiting or 1)
    if not ch:
        continue
    buf.extend(ch)
    used = parse(buf, ev, time.time())
    if used:
        del buf[:used]
ser.close()

print("frames: up=%d steer=%d low=%d" % (
    sum(1 for e in ev if e[1] == "up"), sum(1 for e in ev if e[1] == "steer"),
    sum(1 for e in ev if e[1] == "low")))
if not ev:
    sys.exit(1)
t_start = ev[0][0]
cu, cl, cs, rows, nxt = {}, {}, {}, [], 0.0
for (t, kind, d) in ev:
    (cu if kind == "up" else cl if kind == "low" else cs).update(d)
    ts = t - t_start
    if ts >= nxt:
        nxt = ts + 0.3
        r = {"ts": ts}
        r.update(cu)
        r.update(cl)
        r.update(cs)
        rows.append(r)

p = os.path.join(os.environ.get("TEMP", "."), "steer_full2.csv")
with open(p, "w", encoding="ascii") as fh:
    fh.write("ts,ch2,vx,s0n,s0t,s1n,s1t,t201,a201,t203,a203,p6,her,wcmd,m202,m204\n")
    for r in rows:
        fh.write("%.3f,%d,%.2f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%.4f,%.4f\n" % (
            r["ts"], r.get("ch2", 0), vx_of(r.get("ch2", 0)),
            r.get("s0n", 0), r.get("s0t", 0), r.get("s1n", 0), r.get("s1t", 0),
            r.get("t201", 0), r.get("a201", 0), r.get("t203", 0), r.get("a203", 0),
            r.get("p6", 0), r.get("her", 0), r.get("wcmd", 0),
            r.get("m202", 0), r.get("m204", 0)))
print("csv:", p)
print("  t    ch2   vx | s0t    s1t   split |  her    wcmd  | rpm201 rpm203 | 6020")
for r in rows:
    sp = math.degrees(r.get("s0t", 0)) - math.degrees(r.get("s1t", 0))
    her = r.get("her", None)
    wc = r.get("wcmd", None)
    print("%5.2f %+5d %+5.2f |%+6.3f %+6.3f %+5.1f | %s %s | %+7.1f %+7.1f | %5.1f" % (
        r["ts"], r.get("ch2", 0), vx_of(r.get("ch2", 0)),
        r.get("s0t", 0), r.get("s1t", 0), sp,
        "%+6.3f" % her if her is not None else "  n/a ",
        "%+6.3f" % wc if wc is not None else "  n/a ",
        r.get("a201", 0), r.get("a203", 0), math.degrees(r.get("p6", 0))))
