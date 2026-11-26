#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_check.py —— 读一次调试串口,把关键字段打成 JSON,并按 --expect 判 PASS/FAIL

用法:
    python tools/serial_check.py                       # 读 2 秒,打印全部关键字段
    python tools/serial_check.py --seconds 4           # 读 4 秒
    python tools/serial_check.py --expect "jedec=0xEF4017"
    python tools/serial_check.py --expect "frames>10 and accel_z>9"
    python tools/serial_check.py --expect "selftest=4"   # Flash 自测四步全过

约定:
    * 字段名和 --expect 里用的一致(小写):frames/crc_err/err_code/jedec/selftest/
      flash_state/flash_err/flash_raw/accel_x/accel_y/accel_z/temp/ticks_ok/
      log_boot/log_cur/log_prev_start/log_prev_stop/log_wrap/hwm_min
    * 退出码 0 = PASS,1 = FAIL,2 = 串口/依赖问题

作者备注(为什么这么写):
    * 复用上位机的 FrameDecoder,保证"跟界面看到的是同一套解析"✓
    * 只做只读检查,不改任何固件状态 ✓
"""

import argparse
import importlib.util
import json
import os
import subprocess
import sys
import time

HOST_SCRIPT = r"E:\rm\上位机接收器\Windows\mcu_receiver_Windows.py"

FIELDS = [
    "err_code", "fault_code", "fault_pc", "roll", "pitch", "yaw",
    "gx", "gy", "gz", "ax", "ay", "az", "temp",
    "vx", "vy", "w", "gimbal_yaw", "gimbal_pitch",
    "jedec", "acc_conf", "bmi_err", "err_tick",
    "log_boot", "log_cur_start", "log_prev_start", "log_prev_stop", "log_wrap",
    "flash_state", "flash_err", "flash_cmd", "flash_rcv", "flash_raw",
]


def load_host(path):
    """把上位机脚本当模块加载,复用它的 FrameDecoder ✓"""
    spec = importlib.util.spec_from_file_location("mcu_receiver", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module




def kill_receiver():
    """关掉会占用串口的上位机(它自己也叫 MCU数据接收器.exe)✓
       用 DEVNULL 而不是 PIPE —— 本环境的沙箱不允许给子进程接管道 ✓"""
    try:
        subprocess.run(
            ["powershell", "-NoProfile", "-Command",
             "Get-CimInstance Win32_Process -ErrorAction SilentlyContinue | "
             "Where-Object { $_.Name -match 'MCU' } | "
             "ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
        time.sleep(0.8)
    except Exception:
        pass


def read_port(module, port, seconds):
    import serial

    ser = serial.Serial(port, 921600, timeout=0.2)
    dec = module.FrameDecoder()
    rec = module.Recorder()
    ser.reset_input_buffer()
    t0 = time.time()
    while (time.time() - t0) < seconds:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            for data, frame in dec.feed(chunk):
                rec.add(port, "921600", data, frame)
    ser.close()
    return dec, rec


def build_result(module, dec, rec, seconds):
    result = {
        "frames": len(rec.rows),
        "crc_err": dec.stats.get("crc_err", 0),
        "noise_bytes": dec.stats.get("reject", 0),
        "texts": len(dec.texts),
        "ok": False,
    }
    if not rec.rows:
        result["error"] = "no frame decoded (board silent / wrong port / wrong baud)"
        return result

    last = rec.rows[-1]
    result["frame_len"] = last.get("raw_len", 0)
    for key in FIELDS:
        result[key] = last.get(key, 0)

    # 便于 --expect 使用的归一化字段
    result["accel_x"] = last.get("ax", 0.0)
    result["accel_y"] = last.get("ay", 0.0)
    result["accel_z"] = last.get("az", 0.0)
    result["log_cur"] = last.get("log_cur_start", 0)
    result["selftest"] = last.get("flash_state", 0)
    result["flash_ok"] = 1 if (last.get("flags", 0) & 1) else 0
    result["log_ok"] = 1 if (last.get("flags", 0) & 0x20) else 0
    result["imu_online"] = 1 if (last.get("flags", 0) & 4) else 0
    result["mask"] = last.get("flags", 0)

    # 8 个任务心跳是否都在涨(前后帧比较)
    names = ["motor", "robot", "error", "cmd", "imu", "chassis", "gimbal", "highRise"]
    if len(rec.rows) >= 10:
        first, lastrow = rec.rows[0], rec.rows[-1]
        growing = []
        for i, name in enumerate(names):
            key = "tick%d" % i
            growing.append(1 if lastrow.get(key, 0) > first.get(key, 0) else 0)
        result["ticks_ok"] = sum(growing)
        result["ticks_stuck"] = [names[i] for i, ok in enumerate(growing) if not ok]
    else:
        result["ticks_ok"] = 0
        result["ticks_stuck"] = names

    hwm = [last.get("hwm_" + n, 0) for n in names]
    result["hwm_min"] = min(hwm) if hwm else 0
    result["hwm"] = hwm

    # 健康判定:有帧 + 无故障 + IMU 在线 + 8 个心跳都在涨
    result["ok"] = (
        result["frames"] > 0
        and result["fault_code"] == 0
        and result["imu_online"] == 1
        and result["ticks_ok"] == 8
    )
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM23")
    ap.add_argument("--seconds", type=float, default=2.0)
    ap.add_argument("--expect", default="")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    if not os.path.exists(HOST_SCRIPT):
        print(json.dumps({"error": "host script not found: " + HOST_SCRIPT}))
        return 2

    try:
        module = load_host(HOST_SCRIPT)
    except Exception as exc:  # noqa: BLE001
        print(json.dumps({"error": "load host failed: %r" % (exc,)}))
        return 2

    kill_receiver()
    try:
        dec, rec = read_port(module, args.port, args.seconds)
    except Exception as exc:  # noqa: BLE001
        print(json.dumps({"error": "open/read %s failed: %r" % (args.port, exc)}))
        return 2

    result = build_result(module, dec, rec, args.seconds)

    verdict = None
    if args.expect:
        try:
            verdict = bool(eval(args.expect, {"__builtins__": {}}, dict(result)))  # noqa: S307
        except Exception as exc:  # noqa: BLE001
            result["expect_error"] = repr(exc)
            verdict = False
        result["expect"] = args.expect
        result["pass"] = verdict

    if args.quiet:
        print(json.dumps({"pass": verdict, "frames": result.get("frames"),
                          "err_code": result.get("err_code")}))
    else:
        print(json.dumps(result, ensure_ascii=False, indent=2, default=str))

    if args.expect:
        return 0 if verdict else 1
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
