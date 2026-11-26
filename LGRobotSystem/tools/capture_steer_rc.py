# -*- coding: utf-8 -*-
"""capture_steer_rc.py -- 20s capture of 202/204 steering + RC x stick.

Frames on COM37 (lower board debug UART, 921600):
  0x0102 MOTOR_PID  (total len 252 with 242B payload / 214 with 204B payload)
      payload  0..  38 : 202 block   pos@26  pos_tgt@30  pos_err@34  (f32)
      payload 84..115  : RC block, Ch[i] as i16 at 92+2i  (Ch[2] = stick x)
      payload 144..159 : 201 drive
      payload 160..175 : 203 drive
      payload 176..203 : 204 steer  pos@180 pos_tgt@196 pos_err@200
      payload 204..227 : 6020 pos@204 w@208 tgt@212 w_cmd@216 torque@220
  0x0104 STEER_FAST (len field = payload 38, total 48)
      payload  0 : seat0 now, 4: seat0 tgt, 8: seat1 now, 12: seat1 tgt
      payload 34 : 6020 pos

stick x -> vx command (cmd_task.cpp): vx = +1 * deadzone(Ch[2]/660, 0.05) * 3.0
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


def decode(buf):
    """-> list of dicts (one per valid frame, both types merged in time order)"""
    out = []
    i = 0
    n = len(buf)
    while i + 10 <= n:
        j = buf.find(b"\x5a", i)
        if j < 0 or j + 10 > n:
            break
        ln = struct.unpack_from("<H", buf, j + 2)[0]
        cmd = struct.unpack_from("<H", buf, j + 4)[0]
        if cmd == 0x0102 and 200 <= ln <= 300:
            tot = ln                                     # len = total frame size
        elif cmd == 0x0104 and ln == 38:
            tot = 8 + ln + 2                             # len = payload size here
        else:
            i = j + 1
            continue
        if j + tot > n:
            break
        fr = buf[j:j + tot]
        if crc_ccitt(fr[:-2]) != struct.unpack_from("<H", fr, tot - 2)[0]:
            i = j + 1
            continue
        pl = fr[8:-2]
        i = j + tot
        rec = {"t": time.time(), "plen": len(pl), "seq": struct.unpack_from("<H", fr, 6)[0]}
        if cmd == 0x0102:
            rec["type"] = "pid"
            rec["m202_pos"] = f32(pl, 26)
            rec["m202_tgt"] = f32(pl, 30)
            rec["m202_err"] = f32(pl, 34)
            rec["rc_online"] = pl[84]
            rec["ch2"] = i16(pl, 96)
            rec["ch3"] = i16(pl, 98)
            rec["ch0"] = i16(pl, 92)
            rec["gz"] = f32(pl, 80)
            rec["m204_pos"] = f32(pl, 180) if len(pl) >= 200 else 0.0
            rec["m204_tgt"] = f32(pl, 196) if len(pl) >= 200 else 0.0
            rec["m204_err"] = f32(pl, 200) if len(pl) >= 204 else 0.0
            rec["p6020"] = f32(pl, 204) if len(pl) >= 208 else 0.0
        else:
            rec["type"] = "steer"
            rec["s0_now"] = f32(pl, 0)
            rec["s0_tgt"] = f32(pl, 4)
            rec["s1_now"] = f32(pl, 8)
            rec["s1_tgt"] = f32(pl, 12)
            rec["p6020"] = f32(pl, 34)
        out.append(rec)
    return out


def main():
    port = None
    for p in PORTS:
        try:
            s = serial.Serial(p, 921600, timeout=0.1)
        except Exception as exc:
            print("[port] %s open fail: %s" % (p, exc))
            continue
        s.reset_input_buffer()
        probe = bytearray()
        t0 = time.time()
        while time.time() - t0 < 1.0:
            probe.extend(s.read(s.in_waiting or 1))
        s.close()
        recs = decode(bytes(probe))
        print("[port] %s: %dB -> %d frames" % (p, len(probe), len(recs)))
        if recs:
            port = p
            break
    if port is None:
        print("no usable port")
        return 1

    print("capturing %s for %.0fs (push the x stick now)" % (port, SEC))
    ser = serial.Serial(port, 921600, timeout=0.1)
    ser.reset_input_buffer()
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < SEC:
        buf.extend(ser.read(ser.in_waiting or 1))
    ser.close()

    recs = decode(bytes(buf))
    print("bytes=%d frames=%d (pid=%d steer=%d)" %
          (len(buf), len(recs),
           sum(1 for r in recs if r["type"] == "pid"),
           sum(1 for r in recs if r["type"] == "steer")))
    if not recs:
        return 1

    t_start = recs[0]["t"]
    for r in recs:
        r["ts"] = r["t"] - t_start

    stamp = time.strftime("%Y%m%d_%H%M%S")
    # sandbox: 子进程不能在 workspace 里新建文件,先落到 %TEMP%,要归档再手工拷回 tools/
    path = os.path.join(os.environ.get("TEMP", "."), "steer_rc_%s.csv" % stamp)
    cols = ["ts", "type", "plen", "m202_pos", "m202_tgt", "s0_now", "s0_tgt",
            "m204_pos", "m204_tgt", "s1_now", "s1_tgt", "ch2", "vx", "gz", "p6020"]
    with open(path, "w", encoding="ascii") as fh:
        fh.write(",".join(cols) + "\n")
        for r in recs:
            fh.write("%.3f,%s,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.3f,%.3f,%.4f\n" % (
                r["ts"], r["type"], r["plen"],
                r.get("m202_pos", float("nan")), r.get("m202_tgt", float("nan")),
                r.get("s0_now", float("nan")), r.get("s0_tgt", float("nan")),
                r.get("m204_pos", float("nan")), r.get("m204_tgt", float("nan")),
                r.get("s1_now", float("nan")), r.get("s1_tgt", float("nan")),
                r.get("ch2", 0), stick_vx(r.get("ch2", 0)),
                r.get("gz", float("nan")), r.get("p6020", float("nan"))))
    print("csv: %s" % path)

    pid = [r for r in recs if r["type"] == "pid"]
    st = [r for r in recs if r["type"] == "steer"]

    def rng(vals, name, fmt="%8.3f"):
        if not vals:
            print("%-12s (none)" % name)
            return
        print("%-12s min=%s max=%s mean=%s  n=%d" %
              (name, fmt % min(vals), fmt % max(vals), fmt % (sum(vals) / len(vals)), len(vals)))

    print("\n--- summary ---")
    if pid:
        print("rc online=%d  last ch2=%+d  frames=%d" %
              (pid[-1]["rc_online"], pid[-1]["ch2"], len(pid)))
    rng([r["ch2"] for r in pid], "rc Ch[2]", "%8.0f")
    rng([stick_vx(r["ch2"]) for r in pid], "vx cmd [m/s]")
    rng([r["m202_pos"] for r in pid], "202 pos")
    rng([r["m204_pos"] for r in pid], "204 pos")
    rng([r["s0_now"] for r in st], "seat0 now")
    rng([r["s0_tgt"] for r in st], "seat0 tgt")
    rng([r["s1_now"] for r in st], "seat1 now")
    rng([r["s1_tgt"] for r in st], "seat1 tgt")
    rng([r["p6020"] for r in st if r.get("p6020")], "6020 pos")

    print("\nfirst/last 6 frames:")
    for r in recs[:6] + recs[-6:]:
        if r["type"] == "pid":
            print("  t=%6.2f pid  plen=%3d 202=%+7.3f/%+7.3f 204=%+7.3f/%+7.3f ch2=%+5d vx=%+.2f" % (
                r["ts"], r["plen"], r["m202_pos"], r["m202_tgt"],
                r["m204_pos"], r["m204_tgt"], r["ch2"], stick_vx(r["ch2"])))
        else:
            print("  t=%6.2f str  plen=%3d s0=%+7.3f/%+7.3f s1=%+7.3f/%+7.3f 6020=%+7.3f" % (
                r["ts"], r["plen"], r["s0_now"], r["s0_tgt"], r["s1_now"], r["s1_tgt"],
                r.get("p6020", 0.0)))

    # 0.25s 下采样时间线,便于肉眼看响应
    print("\ntimeline (0.25s step): t   ch2   vx   s0_now s0_tgt  s1_now s1_tgt  m202   m204")
    last_t = -1.0
    for r in recs:
        if r["ts"] - last_t < 0.25:
            continue
        last_t = r["ts"]
        if r["type"] == "pid":
            print("    %6.2f %+5d %+5.2f     -      -        -      -    %+7.3f %+7.3f" % (
                r["ts"], r["ch2"], stick_vx(r["ch2"]), r["m202_pos"], r["m204_pos"]))
        else:
            print("    %6.2f     -     - %+7.3f %+7.3f %+7.3f %+7.3f" % (
                r["ts"], r["s0_now"], r["s0_tgt"], r["s1_now"], r["s1_tgt"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
