#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""校验 android/proto.py 与桌面版 调试助手.py 的解析结果完全一致。

    cd android && python tests/test_proto.py

桌面版是唯一真源:proto.py 由 tools/sync_proto.py 生成。这个测试用同一批
合成帧(0x5A 的 60/84/148 B 帧、0xA5 帧、夹杂噪声和半帧的字节流)分别喂给
两边的 parse_debug / try_parse_a5 / FrameDecoder / Recorder,逐字段比较。

桌面版导入需要 tkinter(只在开发机上跑,不进 APK)。
"""
import importlib.util
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ANDROID = os.path.dirname(HERE)
ROOT = os.path.dirname(ANDROID)
DESKTOP = os.path.join(ROOT, "调试助手.py")
PROTO = os.path.join(ANDROID, "proto.py")


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def mk_debug(payload_len, seq=1, mid=0x02):
    """合成 0x5A MOTOR_PID 帧(CRC 先占位,后面 fix_debug 补真值)。"""
    payload = bytearray(payload_len)
    payload[0] = mid
    payload[1] = 0x03
    struct.pack_into("<9f", payload, 2,
                     12.5, 100.0, 120.0, 118.0, -2.0, 0.42, 1.0, 1.2, 0.2)
    if payload_len >= 40:
        payload[38], payload[39] = 0, 1
    if payload_len >= 60:
        payload[40] = 2
        struct.pack_into("<H", payload, 42, 0x1234)
        struct.pack_into("<I", payload, 44, 3)
        struct.pack_into("<I", payload, 48, 0x04000000)
        struct.pack_into("<I", payload, 52, 0x08001234)
        struct.pack_into("<I", payload, 56, 0b101)
    if payload_len >= 84:
        struct.pack_into("<6f", payload, 60, 0.1, 0.2, -0.3, 1.0, 2.0, 3.0)
    if payload_len >= 116:
        payload[84] = 1
        payload[85], payload[86] = 1, 3
        payload[87], payload[88] = 1, 0
        struct.pack_into("<H", payload, 90, 0x0015)
        struct.pack_into("<5h", payload, 92, 0, 660, -660, 100, -100)
        struct.pack_into("<3h", payload, 102, 5, -5, 0)
        struct.pack_into("<H", payload, 108, 2)
        struct.pack_into("<I", payload, 112, 4321)
    if payload_len >= 148:
        payload[116] = 1
        payload[117] = 1
        struct.pack_into("<h", payload, 118, 500)
        struct.pack_into("<I", payload, 120, 0)
        struct.pack_into("<5f", payload, 124, 1.5, 2.5, 0.35, 41.5, 24.2)
        struct.pack_into("<H", payload, 144, 3)
        struct.pack_into("<H", payload, 146, 0)
    total = payload_len + 10
    frame = (bytes([0x5A, 0x00]) + struct.pack("<H", total)
             + struct.pack("<H", 0x0102) + struct.pack("<H", seq) + bytes(payload))
    return frame + struct.pack("<H", 0)


def fix_debug(frames, mod):
    """把 0x5A 帧末尾的占位 CRC 换成真值。"""
    return [f[:-2] + struct.pack("<H", mod.crc_ccitt(f[:-2])) for f in frames]


def mk_a5(mod, cmd_id, data, seq8=7):
    """A5 帧:SOF | u16 data_len(= data + cmd 2 + crc 2) | seq8 | crc8(head) | u16 cmd | data | u16 crc16"""
    data_len = len(data) + 4
    head = bytes([0xA5, data_len & 0xFF, (data_len >> 8) & 0xFF, seq8])
    frame = head + bytes([mod.crc8(head)]) + struct.pack("<H", cmd_id) + data
    return frame + struct.pack("<H", mod.crc_ccitt(frame[5:]))


def cmp_dict(a, b, where):
    if a.keys() != b.keys():
        raise AssertionError("%s 键不同:%s" % (where, set(a) ^ set(b)))
    for k in a:
        if a[k] != b[k]:
            raise AssertionError("%s 字段 %r 不同:桌面=%r proto=%r" % (where, k, a[k], b[k]))


def main():
    if not os.path.isfile(DESKTOP):
        raise SystemExit("找不到桌面版:" + DESKTOP)
    if not os.path.isfile(PROTO):
        raise SystemExit("找不到 proto.py,先运行 tools/sync_proto.py")
    with open(PROTO, encoding="utf-8") as fp:
        src = fp.read()
    if "tkinter" in src:
        raise SystemExit("proto.py 里不该出现 tkinter(生成范围错了)")

    desk = load(DESKTOP, "desktop_app")
    android = load(PROTO, "android_proto")

    # 1) 纯函数一致
    assert desk.crc8(b"\xA5\x10\x00\x07") == android.crc8(b"\xA5\x10\x00\x07")
    assert desk.crc_ccitt(b"\x01\x02\x03") == android.crc_ccitt(b"\x01\x02\x03")
    for raw in (-7.0, -0.1, 0.0, 0.1, 3.2, 6.5):
        assert desk.wrap_2pi(raw) == android.wrap_2pi(raw)
        assert desk.wrap_pi(raw) == android.wrap_pi(raw)

    # 2) 三种长度的 0x5A 帧(60 = 只有 PID,84 = 带 IMU,148 = 全量)
    debug_frames = fix_debug([mk_debug(n, seq=n) for n in (60, 84, 148)], android)
    for f in debug_frames:
        cmp_dict(desk.parse_debug(f), android.parse_debug(f), "parse_debug(len=%d)" % len(f))

    # 3) 0xA5 帧(minipc 的 vx/vy/w + 裁判系统)
    a5_data_mini = (bytes([1, 1, 100, 0]) + struct.pack("<3f", 1.5, -2.5, 0.5)
                    + struct.pack("<2f", 0.1, -0.2) + bytes(8))
    a5_frames = [mk_a5(android, 0x0001, a5_data_mini),
                 mk_a5(android, 0x0002, b"\x01\x02\x03\x04")]
    for f in a5_frames:
        da, ua = desk.try_parse_a5(f)
        db, ub = android.try_parse_a5(f)
        assert ua == ub == len(f), (ua, ub, len(f))
        cmp_dict(da, db, "try_parse_a5")

    # 4) 字节流:噪声 + 全帧 + A5 帧 + 半帧,按 1/3/7/999 字节切块喂
    stream = (b"\x00\x11\x22" + debug_frames[2] + a5_frames[0]
              + debug_frames[1][:40])
    for chunk in (1, 3, 7, 999):
        da, db = desk.FrameDecoder(), android.FrameDecoder()
        seq_a, seq_b = [], []
        for i in range(0, len(stream), chunk):
            for d, _f in da.feed(stream[i:i + chunk]):
                seq_a.append((d.get("proto"), d.get("seq")))
            for d, _f in db.feed(stream[i:i + chunk]):
                seq_b.append((d.get("proto"), d.get("seq")))
        assert seq_a == seq_b, "chunk=%d 解出的帧序列不同:%r != %r" % (chunk, seq_a, seq_b)
        assert da.stats == db.stats, (chunk, da.stats, db.stats)

    # 5) 坏 CRC:两边都只计 crc_err,不崩
    bad = bytearray(debug_frames[0])
    bad[-1] ^= 0xFF
    da, db = desk.FrameDecoder(), android.FrameDecoder()
    da.feed(bytes(bad))
    db.feed(bytes(bad))
    assert da.stats["crc_err"] == db.stats["crc_err"] == 1

    # 6) Recorder 记录的字段一致(时间戳除外)
    ra, rb = desk.Recorder(), android.Recorder()
    for f in debug_frames:
        ra.add("usb", "921600", desk.parse_debug(f), f)
        rb.add("usb", "921600", android.parse_debug(f), f)
    assert len(ra.rows) == len(rb.rows)
    for x, y in zip(ra.rows, rb.rows):
        x = dict(x, time=None)
        y = dict(y, time=None)
        assert x == y, "记录字段不同"

    print("OK: 协议解析 / 帧解码 / 记录 与桌面版一致")
    print("0x5A 帧 %d 个,0xA5 帧 %d 个,记录 %d 帧"
          % (len(debug_frames), len(a5_frames), len(ra.rows)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
