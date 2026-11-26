#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""motor_probe.py — 只读抓一段调试遥测,打印最后一帧的电机块(不杀进程/不写串口)。

用法:
    python tools/motor_probe.py [COM23] [2.5]
"""

import importlib.util
import json
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
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 2.5
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else 921600

    mod = load_host(HOST)
    dec = mod.FrameDecoder()
    last = None
    frames = 0
    raw = bytearray()

    ser = serial.Serial(port, baud, timeout=0.2)
    ser.reset_input_buffer()
    t0 = time.time()

    while (time.time() - t0) < seconds:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        raw.extend(chunk)
        for _data, frame in dec.feed(chunk):
            frames += 1
            last = mod.parse_debug(frame)

    ser.close()

    if last is None:
        print(json.dumps({"port": port, "baud": baud, "frames": 0, "raw_bytes": len(raw),
                          "raw_head_hex": bytes(raw[:64]).hex(" "),
                          "error": "no debug frame decoded (board silent / wrong port / baud)"},
                         ensure_ascii=False, indent=2))
        return 2

    out = {
        "port": port,
        "proto": last.get("proto"),
        "frames": frames,
        "crc_err": dec.stats.get("crc_err", 0),
        "seq": last.get("seq"),
        "err_level": last.get("err_level"),
        "err_code": last.get("err_code"),
        "fault_code": last.get("fault_code"),
        "motor_count": last.get("motor_count"),
        "flags": last.get("flags"),
        "jedec": hex(last.get("jedec", 0)),
        "acc_conf": last.get("acc_conf"),
        "imu_euler_deg": [round(last.get("roll", 0.0), 2),
                          round(last.get("pitch", 0.0), 2),
                          round(last.get("yaw", 0.0), 2)],
        "accel": [round(last.get("ax", 0.0), 3),
                  round(last.get("ay", 0.0), 3),
                  round(last.get("az", 0.0), 3)],
        "chassis_vx_vy_w": [round(last.get("vx", 0.0), 3),
                            round(last.get("vy", 0.0), 3),
                            round(last.get("w", 0.0), 3)],
        "photo": last.get("photo"),
        "pe2_level": last.get("pe2_level"),
        "motors": last.get("motors", []),
        "pid": {k: last.get(k) for k in (
            "encoder_joint_rpm", "encoder_motor_rpm", "pid_target_rpm",
            "pid_feedback_rpm", "pid_error_rpm", "pid_torque", "pos",
            "pos_target", "pos_error")} if last.get("proto") == "MOTOR_PID" else None,
        "ext_imu": {
            "gx": last.get("ext_gx", 0.0), "gy": last.get("ext_gy", 0.0),
            "gz": last.get("ext_gz", 0.0), "ax": last.get("ext_ax", 0.0),
            "ay": last.get("ext_ay", 0.0), "az": last.get("ext_az", 0.0),
        },
        "ticks": [last.get("tick%d" % i, 0) for i in range(8)],
    }
    print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
