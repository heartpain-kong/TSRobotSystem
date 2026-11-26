# -*- coding: utf-8 -*-
"""解析 mcu_raw_*.txt / *.csv 里的 MOTOR_PID(0x0102) 帧,提取 202/201/203/204 + IMU。"""
import re
import struct
import sys

SOF = 0x5A
ID_PID = 0x0102

FIELDS202 = [("j_rpm", 2), ("m_rpm", 6), ("tgt_rpm", 10), ("fb_rpm", 14),
             ("err_rpm", 18), ("out", 22), ("pos", 26), ("pos_tgt", 30), ("pos_err", 34)]


def f32(b, off):
    return struct.unpack_from("<f", b, off)[0]


def decode(p):
    out = {}
    out["id"] = p[0]
    out["flags"] = p[1]
    for name, off in FIELDS202:
        out[name] = f32(p, off)
    # IMU 60..83
    out["roll"], out["pitch"], out["yaw"] = f32(p, 60), f32(p, 64), f32(p, 68)
    out["gx"], out["gy"], out["gz"] = f32(p, 72), f32(p, 76), f32(p, 80)
    # 144: 0x201 drive
    out["m201_f"] = p[144]
    out["m201_pos"], out["m201_w"], out["m201_t"] = f32(p, 148), f32(p, 152), f32(p, 156)
    # 160: 0x203
    out["m203_f"] = p[160]
    out["m203_pos"], out["m203_w"], out["m203_t"] = f32(p, 164), f32(p, 168), f32(p, 172)
    # 176: 0x204
    out["m204_f"] = p[176]
    out["m204_pos"], out["m204_w"], out["m204_t"] = f32(p, 180), f32(p, 184), f32(p, 188)
    out["m204_tgt_rpm"], out["m204_pos_tgt"] = f32(p, 192), f32(p, 196)
    out["m204_pos_err"] = f32(p, 200)
    # 204: 0x205 6020 yaw(232B payload 才有)
    if len(p) >= 232:
        out["m6020_pos"] = f32(p, 204)
        out["m6020_w"] = f32(p, 208)
        out["m6020_pos_tgt"] = f32(p, 212)
        out["m6020_w_cmd"] = f32(p, 216)
        out["m6020_t"] = f32(p, 220)
        out["m6020_online"] = p[224]
    else:
        out["m6020_pos"] = out["m6020_w"] = 0.0
        out["m6020_pos_tgt"] = out["m6020_w_cmd"] = 0.0
        out["m6020_t"] = 0.0
        out["m6020_online"] = 0
    if len(p) >= 242:
        out["homing_state0"] = p[232]
        out["homing_state1"] = p[233]
        out["homing_scan0"] = f32(p, 234)
        out["homing_scan1"] = f32(p, 238)
    else:
        out["homing_state0"] = out["homing_state1"] = 0xFF
        out["homing_scan0"] = out["homing_scan1"] = 0.0
    return out


def parse_txt(path):
    rx = re.compile(r"^(\d\d:\d\d:\d\d\.\d+).*?len=(\d+)\s+((?:[0-9A-Fa-f]{2} )+[0-9A-Fa-f]{2})")
    for line in open(path, "r", errors="ignore"):
        m = rx.search(line)
        if not m:
            continue
        yield m.group(1), bytes.fromhex(m.group(3))


def parse_csv(path):
    import csv
    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            if row.get("proto") != "MOTOR_PID":
                continue
            yield row["time"], bytes.fromhex(row["hex"])


def frames(path):
    lines = parse_csv(path) if path.lower().endswith(".csv") else parse_txt(path)
    for t, b in lines:
        if len(b) < 20 or b[0] != SOF:
            continue
        if struct.unpack_from("<H", b, 4)[0] != ID_PID:
            continue
        ln = struct.unpack_from("<H", b, 2)[0]
        if ln != len(b):
            continue
        yield t, b


if __name__ == "__main__":
    for path in sys.argv[1:]:
        n = 0
        first = last = None
        cur = None
        for t, b in frames(path):
            n += 1
            if first is None:
                first = (t, b)
            last = (t, b)
        print(path)
        print("  frames:", n)
        if first:
            print("  first:", first[0], "last:", last[0])
            d = decode(first[1][8:-2])
            for k, v in d.items():
                print("   %-12s %s" % (k, v if not isinstance(v, float) else round(v, 4)))
            print()
            d2 = decode(last[1][8:-2])
            print("  last decoded:")
            for k, v in d2.items():
                print("   %-12s %s" % (k, v if not isinstance(v, float) else round(v, 4)))
