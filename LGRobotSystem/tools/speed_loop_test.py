#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""speed_loop_test.py — 通过 0x5C 给速度环一个阶跃,记录目标/反馈/力矩/位置。

用法:
    python tools/speed_loop_test.py COM23 30 2.0
"""

import importlib.util
import struct
import sys
import time

import serial

HOST = r"E:\rm\上位机接收器\Windows\mcu_receiver_Windows.py"


def load_host(path):
    spec = importlib.util.spec_from_file_location("mcu_receiver", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM23"
    target = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0

    mod = load_host(HOST)
    dec = mod.FrameDecoder()
    ser = serial.Serial(port, 921600, timeout=0.05)
    ser.reset_input_buffer()

    seq = 0

    def send(cmd, payload=b""):
        nonlocal seq
        seq = (seq + 1) & 0xFFFF
        ser.write(bytes([0x5C, cmd, (seq >> 8) & 0xFF, seq & 0xFF]) + payload)

    def read_for(duration, label):
        end = time.time() + duration
        last = None
        rows = []
        while time.time() < end:
            data = ser.read(ser.in_waiting or 1)
            if data:
                for d, _frame in dec.feed(data):
                    if d.get("proto") != "MOTOR_PID":
                        continue
                    last = d
                    rows.append(d)
        if last:
            print("%-8s flags=%d target=%+7.2f fb=%+7.2f err=%+7.2f "
                  "torque=%+8.4f pos=%+7.4f"
                  % (label, last.get("flags", 0), last.get("pid_target_rpm", 0),
                     last.get("pid_feedback_rpm", 0), last.get("pid_error_rpm", 0),
                     last.get("pid_torque", 0), last.get("pos", 0)), flush=True)
        else:
            print("%-8s no MOTOR_PID frame" % label, flush=True)
        return rows

    # 先看 0.5 s,确认在线
    rows = read_for(0.5, "check")
    if not rows or not (rows[-1].get("flags", 0) & 2):
        print("ABORT: motor not online (flags=%s)" % (rows[-1].get("flags") if rows else None))
        ser.close()
        return 2

    # 只发速度阶跃,不写 PID —— 避免把 APP 里刚调好的增益/位置环覆盖掉
    # (要写 PID 请用 APP 的“写入 PID”,或另发 0x03)
    time.sleep(0.1)

    # 目标 0 起步
    send(0x01, struct.pack("<h", 0))
    read_for(0.3, "zero")

    # 阶跃到 target
    send(0x01, struct.pack("<h", target))
    rows = read_for(seconds, "step")

    # 回 0,再等 0.5 s 看有没有残余力矩/过冲
    send(0x01, struct.pack("<h", 0))
    rows2 = read_for(0.8, "back0")

    # 停止(0x05),确保最后不残留
    send(0x05, b"\x00")
    time.sleep(0.2)
    ser.close()

    if rows:
        print("summary: pos_first=%+0.4f pos_last=%+0.4f delta=%+0.4f rad (%.3f rev)"
              % (rows[0].get("pos", 0), rows[-1].get("pos", 0),
                 rows[-1].get("pos", 0) - rows[0].get("pos", 0),
                 (rows[-1].get("pos", 0) - rows[0].get("pos", 0)) / 6.2831853))
        print("summary: torque min=%+0.4f max=%+0.4f"
              % (min(r.get("pid_torque", 0) for r in rows),
                 max(r.get("pid_torque", 0) for r in rows)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
