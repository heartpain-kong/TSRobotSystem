# -*- coding: utf-8 -*-
"""capture_steer_x.py -- 同一路串口同时抓两块板:上板(遥控器 x) + 下板(202/204/6020)

为什么分两块板:
    下板不接遥控器(cmd_task.cpp: "下 MCU 不接遥控器, CAN3 指令是唯一驾驶来源"),
    摇杆在上板, 经 485 传给下板。COM37 上的 0x0102 有两路:
        payload 204B = 上板(有 RC, rx_count 很大, 电机全 0)
        payload 242B = 下板(真电机/6020, rc 恒 0)
    按 payload 长度分类, 同一条串口, 天生同一时间轴。

上板 0x0102:  RC Ch[i] 在 payload 92+2i  (i16, 中心 0, 满 660)
下板 0x0102:  202 pos@26 tgt@30 | 204 pos@180 tgt@196 | 6020 pos@204
下板 0x0104:  seat0 now@0 tgt@4 | seat1 now@8 tgt@12 | 6020 pos@34

注意:时间戳必须在【读串口的那一刻】打, 不能等抓完再解(否则全挤成一帧)。

用法: python tools/capture_steer_x.py [秒数]
"""
import math
import os
import struct
import sys
import time

import serial

PORTS = ["COM37", "COM30"]
SEC = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
CH_FULL = 660.0
DEAD_ZONE = 0.05
MAX_VX = 3.0


def crc_ccitt(data, crc=0):
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def f32(b, o):
    return struct.unpack_from("<f", b, o)[0]


def i16(b, o):
    return struct.unpack_from("<h", b, o)[0]


def stick_vx(ch):
    v = max(-1.0, min(1.0, ch / CH_FULL))
    if abs(v) < DEAD_ZONE:
        v = 0.0
    return v * MAX_VX


def parse_into(buf, ev, now):
    """吃掉 buf 里所有完整帧, 用 now 作时间戳; 返回消费掉的字节数"""
    i = 0
    n = len(buf)
    while i + 10 <= n:
        j = buf.find(b"\x5a", i)
        if j < 0 or j + 10 > n:
            return i if j < 0 else j
        ln = struct.unpack_from("<H", buf, j + 2)[0]
        cmd = struct.unpack_from("<H", buf, j + 4)[0]
        if cmd == 0x0102 and 200 <= ln <= 300:
            tot = ln
        elif cmd == 0x0104 and ln == 38:
            tot = 8 + ln + 2
        else:
            i = j + 1
            continue
        if j + tot > n:
            return j
        fr = buf[j:j + tot]
        if crc_ccitt(fr[:-2]) != struct.unpack_from("<H", fr, tot - 2)[0]:
            i = j + 1
            continue
        pl = fr[8:-2]
        i = j + tot
        if cmd == 0x0104:
            ev.append((now, "low", {"s0_now": f32(pl, 0), "s0_tgt": f32(pl, 4),
                                    "s1_now": f32(pl, 8), "s1_tgt": f32(pl, 12),
                                    "p6020": f32(pl, 34)}))
        elif len(pl) == 242:
            ev.append((now, "low", {"m202": f32(pl, 26), "m202t": f32(pl, 30),
                                    "m204": f32(pl, 180), "m204t": f32(pl, 196),
                                    "p6020": f32(pl, 204)}))
        elif len(pl) == 204:
            ev.append((now, "up", {"ch2": i16(pl, 96), "rc_on": pl[84],
                                   "s1": pl[85], "s2": pl[86]}))
        else:
            ev.append((now, "other", {"plen": len(pl)}))
    return i


def open_port():
    for p in PORTS:
        try:
            s = serial.Serial(p, 921600, timeout=0.1)
        except Exception as exc:
            print("[port] %s open fail: %s" % (p, exc))
            continue
        s.reset_input_buffer()
        probe = bytearray()
        ev = []
        t0 = time.time()
        while time.time() - t0 < 0.8:
            probe.extend(s.read(s.in_waiting or 1))
            used = parse_into(probe, ev, time.time())
            if used:
                del probe[:used]
        s.close()
        kinds = {e[1] for e in ev}
        print("[port] %s: frames=%d kinds=%s" % (p, len(ev), sorted(kinds)))
        if "low" in kinds and "up" in kinds:
            return p
    return None


def main():
    port = open_port()
    if port is None:
        print("no usable port")
        return 1

    print("capture %s %.0fs -- push x to both ends now" % (port, SEC))
    ser = serial.Serial(port, 921600, timeout=0.1)
    ser.reset_input_buffer()
    buf = bytearray()
    ev = []
    t0 = time.time()
    while time.time() - t0 < SEC:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf.extend(chunk)
        used = parse_into(buf, ev, time.time())
        if used:
            del buf[:used]
    ser.close()

    low = [e for e in ev if e[1] == "low"]
    up = [e for e in ev if e[1] == "up"]
    print("frames: low=%d up=%d" % (len(low), len(up)))
    if not low or not up:
        print("missing a stream")
        return 1

    t_start = ev[0][0]
    cur_up = {}
    cur_low = {}
    rows = []
    next_slot = 0.0
    for (t, kind, d) in ev:
        if kind == "up":
            cur_up.update(d)
        elif kind == "low":
            cur_low.update(d)
        else:
            continue
        ts = t - t_start
        if ts >= next_slot:
            next_slot = ts + 0.25
            r = {"ts": ts}
            r.update(cur_up)
            r.update(cur_low)
            rows.append(r)

    st = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(os.environ.get("TEMP", "."), "steer_x_%s.csv" % st)
    cols = ["ts", "ch2", "vx", "rc_on", "s1", "s2", "m202", "m202t",
            "m204", "m204t", "s0_now", "s0_tgt", "s1_now", "s1_tgt", "p6020"]
    with open(path, "w", encoding="ascii") as fh:
        fh.write(",".join(cols) + "\n")
        for r in rows:
            fh.write("%.3f,%d,%.3f,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n" % (
                r["ts"], r.get("ch2", 0), stick_vx(r.get("ch2", 0)), r.get("rc_on", -1),
                r.get("s1", -1), r.get("s2", -1),
                r.get("m202", 0), r.get("m202t", 0), r.get("m204", 0), r.get("m204t", 0),
                r.get("s0_now", 0), r.get("s0_tgt", 0), r.get("s1_now", 0),
                r.get("s1_tgt", 0), r.get("p6020", 0)))
    print("csv: %s" % path)

    ch = [r.get("ch2", 0) for r in rows]
    print("\nch2: min=%+d max=%+d  nonzero=%d/%d  rc_on(last)=%d s1=%d s2=%d" %
          (min(ch), max(ch), sum(1 for c in ch if c), len(ch),
           rows[-1].get("rc_on", -1), rows[-1].get("s1", -1), rows[-1].get("s2", -1)))
    p6 = rows[-1].get("p6020", 0.0)
    print("6020 pos: %.3f rad = %.1f deg = %+d counts (zero@550)" % (
        p6, math.degrees(p6), round(p6 / (2 * math.pi) * 8192)))
    print("seat0 tgt: min=%+.3f max=%+.3f | seat1 tgt: min=%+.3f max=%+.3f" % (
        min(r.get("s0_tgt", 0) for r in rows), max(r.get("s0_tgt", 0) for r in rows),
        min(r.get("s1_tgt", 0) for r in rows), max(r.get("s1_tgt", 0) for r in rows)))

    print("\n   t     ch2    vx  | s0_now  s0_tgt | s1_now  s1_tgt |  m202     m204   | 6020")
    for r in rows:
        print("%6.2f %+5d %+5.2f |%+8.3f %+7.3f |%+8.3f %+7.3f | %+7.3f %+7.3f | %+6.3f" % (
            r["ts"], r.get("ch2", 0), stick_vx(r.get("ch2", 0)),
            r.get("s0_now", 0), r.get("s0_tgt", 0),
            r.get("s1_now", 0), r.get("s1_tgt", 0),
            r.get("m202", 0), r.get("m204", 0), r.get("p6020", 0)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
