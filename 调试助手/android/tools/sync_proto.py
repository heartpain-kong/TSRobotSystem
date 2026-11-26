#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从桌面版 调试助手.py 抽取协议/记录代码,生成 android/proto.py。

用法(在 android/ 目录下):
    python tools/sync_proto.py            生成 proto.py
    python tools/sync_proto.py --check    只检查 proto.py 是否最新(提交前/CI 用)

为什么这样共享代码:
  · 桌面版是唯一真源,协议解析绝不能在两端各写一份然后慢慢跑偏;
  · 只抽取两端都能用的纯 Python 部分(不碰 tkinter):
      1) 协议常量 / CRC / 角度归一化 / 解析 / FrameDecoder
      2) Recorder(记录 + 导出 CSV / 原始 CSV / 原始 HEX / MD)
  · 抽取后自动编译 + 冒烟跑一遍(用合成帧),失败就不落盘。
"""
import argparse
import hashlib
import importlib.util
import os
import py_compile
import struct
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ANDROID_DIR = os.path.dirname(HERE)
DESKTOP = os.path.normpath(os.path.join(ANDROID_DIR, "..", "调试助手.py"))
OUT = os.path.join(ANDROID_DIR, "proto.py")

# 段落标题(桌面版里 "# ====...\n# 标题" 的形式)
SEC_PROTO = "协议常量"
SEC_SERIAL = "串口"
SEC_RECORD = "记录 + 导出"
SEC_GUI = "GUI"

HEADER = '''# -*- coding: utf-8 -*-
"""0x5A / 0xA5 协议解析 + 记录导出(Android 版共享层)。

★ 这个文件是自动生成的,不要手改 ★
源文件:{source}
生成命令:python tools/sync_proto.py
改协议请改桌面版 调试助手.py,然后重新生成(android/tests/test_proto.py 会校验两端一致)。
"""
import collections
import csv
import math
import os
import struct
import sys
import time
from datetime import datetime

# 桌面版里 CONFIG 由平台探测填充;这里只需要导出用到的两项
CONFIG = {{"csv_encoding": "utf-8-sig", "platform": "Android"}}

'''


def _section_start(text, title):
    """找到 "# 标题" 所在小节,返回它上面那行 "# ====" 的偏移量。"""
    marker = "# " + title
    idx = 0
    while True:
        idx = text.find(marker, idx)
        if idx < 0:
            raise SystemExit("桌面版里找不到小节标题:%r" % title)
        line_start = text.rfind("\n", 0, idx) + 1
        if text[line_start:idx].strip() == "" and text[line_start:idx].rstrip("\r").strip("#= \t") == "":
            # 上一行是 "# =====..." 分隔线
            prev = text.rfind("\n", 0, line_start - 1) + 1
            if text[prev:line_start].lstrip().startswith("# ===="):
                return prev
        idx += len(marker)


def _slice(text, start_title, end_title):
    i = _section_start(text, start_title)
    j = _section_start(text, end_title)
    if j <= i:
        raise SystemExit("小节顺序不对:%r -> %r" % (start_title, end_title))
    return text[i:j].rstrip() + "\n"


def build(text):
    proto = _slice(text, SEC_PROTO, SEC_SERIAL)
    # 记录+导出 小节里的 _export_dir() 依赖桌面平台,Android 自己决定导出目录 ⇒ 从 Recorder 起抽
    i = text.index("class Recorder:")
    j = _section_start(text, SEC_GUI)
    if j <= i:
        raise SystemExit("找不到 Recorder 或 GUI 小节")
    record = text[i:j].rstrip() + "\n"

    digest = hashlib.sha256(text.encode("utf-8")).hexdigest()
    footer = (
        "\n\n# ---- 生成信息(不要手改)----\n"
        "SOURCE = {\"file\": %r, \"sha256\": %r}\n"
        % ("调试助手.py", digest)
    )
    return HEADER.format(source="调试助手.py") + proto + "\n\n" + record + footer


def _smoke(path):
    """编译 + 用合成帧跑一遍解码和导出,确认抽出来的代码真能用。"""
    py_compile.compile(path, doraise=True)
    spec = importlib.util.spec_from_file_location("proto_smoke", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    # 合成一帧完整的 MOTOR_PID(158 B:8 头 + 148 payload + 2 CRC)
    payload = bytearray(148)
    payload[0] = 0x02
    payload[1] = 0x03
    struct.pack_into("<9f", payload, 2, 0.0, 100.0, 120.0, 118.0, -2.0, 0.42, 1.0, 1.2, 0.2)
    payload[38], payload[39] = 0, 1
    payload[40] = 2
    struct.pack_into("<H", payload, 42, 0x1234)
    struct.pack_into("<6f", payload, 60, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0)
    payload[84] = 1
    payload[85], payload[86] = 1, 3
    frame = (bytes([0x5A, 0x00]) + struct.pack("<H", 158)
             + struct.pack("<H", 0x0102) + struct.pack("<H", 7) + bytes(payload))
    frame += struct.pack("<H", mod.crc_ccitt(frame))

    dec = mod.FrameDecoder()
    out = dec.feed(frame)
    if len(out) != 1:
        raise SystemExit("冒烟失败:解出 %d 帧(应为 1)" % len(out))
    d = out[0][0]
    if d["proto"] != "MOTOR_PID" or abs(d["pid_feedback_rpm"] - 118.0) > 1e-6:
        raise SystemExit("冒烟失败:解出来的字段不对 %r" % (d.get("proto"),))

    rec = mod.Recorder()
    rec.add("usb", "921600", d, frame)
    with tempfile.TemporaryDirectory() as tmp:
        csv_path = os.path.join(tmp, "a.csv")
        md_path = os.path.join(tmp, "a.md")
        hex_path = os.path.join(tmp, "a.hex")
        rec.export_csv(csv_path)
        rec.export_md(md_path, dict(dec.stats))
        rec.export_raw_hex(hex_path)
        for p in (csv_path, md_path, hex_path):
            if not os.path.getsize(p):
                raise SystemExit("冒烟失败:导出文件是空的 " + os.path.basename(p))


def main(argv=None):
    ap = argparse.ArgumentParser(description="从桌面版生成 Android 共享协议层")
    ap.add_argument("--check", action="store_true", help="只检查是否最新,不写文件")
    args = ap.parse_args(argv)

    with open(DESKTOP, encoding="utf-8") as fp:
        text = fp.read()
    new = build(text)

    old = None
    if os.path.isfile(OUT):
        with open(OUT, encoding="utf-8") as fp:
            old = fp.read()

    if args.check:
        if old != new:
            print("proto.py 不是最新的,请运行: python tools/sync_proto.py")
            return 1
        print("proto.py 与桌面版一致")
        return 0

    if old == new:
        print("proto.py 没有变化")
    else:
        with open(OUT, "w", encoding="utf-8", newline="\n") as fp:
            fp.write(new)
        print("已生成 %s (%d 字节)" % (os.path.relpath(OUT, ANDROID_DIR), len(new.encode("utf-8"))))

    _smoke(OUT)
    print("冒烟通过:解码 + 导出 OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
