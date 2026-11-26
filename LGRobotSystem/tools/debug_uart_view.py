#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
debug_uart_view.py — 解析机器人调试遥测帧(debug_task.h 定义的协议)

用法:
  1) 抓串口存文件(任选其一):
       · 用 CLion / VSCode 的串口终端,打开 COM? 921600,把输出保存成 dump.bin
       · 或 python -m serial.tools.miniterm COM5 921600 > dump.bin   (需要 pip install pyserial)
  2) 解析:
       python tools/debug_uart_view.py dump.bin
       python tools/debug_uart_view.py            (默认从 stdin 读)

零依赖(只用标准库)。帧格式见 User/App/debug/debug_task.h。
"""

import struct
import sys

SOF = 0x5A
HEADER = 8
MOTOR_SIZE = 16
MAX_MOTORS = 8
PAYLOAD = 200
FRAME = HEADER + PAYLOAD + 2          # 210


def crc_ccitt(data: bytes, crc: int = 0x0000) -> int:
    """CRC-CCITT(★反射版:poly 0x8408,初值 0)—— 与固件 User/algorithm/crc/crc_ccitt.h 完全一致 ✓
       注意:不是标准 CCITT(0x1021/0xFFFF)✗ 两者结果完全不同 ✗"""
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def decode(frame: bytes):
    ver = frame[1]
    length = struct.unpack_from("<H", frame, 2)[0]
    cmd = struct.unpack_from("<H", frame, 4)[0]
    seq = struct.unpack_from("<H", frame, 6)[0]
    pl = frame[HEADER:]

    err_level = pl[0]
    n_motor = pl[1]
    err_code = struct.unpack_from("<H", pl, 2)[0]
    fault_code = struct.unpack_from("<I", pl, 4)[0]
    fault_cfsr = struct.unpack_from("<I", pl, 8)[0]
    fault_pc = struct.unpack_from("<I", pl, 12)[0]
    roll, pitch, yaw = struct.unpack_from("<3f", pl, 16)
    vx, vy, w = struct.unpack_from("<3f", pl, 28)
    yaw_mech, pitch_mech = struct.unpack_from("<2f", pl, 40)
    gx, gy, gz = struct.unpack_from("<3f", pl, 176)
    ax, ay, az = struct.unpack_from("<3f", pl, 188)

    motors = []
    for i in range(min(n_motor, MAX_MOTORS)):
        o = 48 + i * MOTOR_SIZE
        mid = pl[o]
        flags = pl[o + 1]
        pos, vel, tor = struct.unpack_from("<3f", pl, o + 4)
        motors.append((mid, flags, pos, vel, tor))

    return dict(ver=ver, length=length, cmd=cmd, seq=seq, err_level=err_level,
                err_code=err_code, fault_code=fault_code, fault_cfsr=fault_cfsr,
                fault_pc=fault_pc, roll=roll, pitch=pitch, yaw=yaw, vx=vx, vy=vy,
                w=w, yaw_mech=yaw_mech, pitch_mech=pitch_mech, gx=gx, gy=gy, gz=gz, ax=ax, ay=ay, az=az, motors=motors)


LEVEL = {0: "OK", 1: "WARN", 2: "ERROR", 3: "FATAL"}
FAULT = {0: "正常", 1: "堆分配失败", 2: "任务栈溢出", 3: "Error_Handler", 4: "HardFault"}


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else None
    data = open(path, "rb").read() if path else sys.stdin.buffer.read()
    print(f"输入 {len(data)} 字节;帧长 {FRAME}")

    buf = data
    i, ok, bad_crc = 0, 0, 0
    while i + FRAME <= len(buf):
        if buf[i] != SOF:
            i += 1
            continue
        frame = buf[i:i + FRAME]
        if crc_ccitt(frame[:-2]) != struct.unpack_from("<H", frame, FRAME - 2)[0]:
            bad_crc += 1
            i += 1
            continue
        i += FRAME
        ok += 1
        d = decode(frame)
        motors = "  ".join(
            f"[{m[0]}{'I' if m[1] & 1 else '-'}{'O' if m[1] & 2 else '-'}"
            f"{'E' if m[1] & 4 else '-'}] pos={m[2]:+.3f} w={m[3]:+.2f} t={m[4]:+.3f}"
            for m in d["motors"])
        print(f"#{d['seq']:5d} {LEVEL.get(d['err_level'], '?')} code={d['err_code']:#06x} "
              f"fault={FAULT.get(d['fault_code'], d['fault_code'])} "
              f"imu=({d['roll']:+.2f},{d['pitch']:+.2f},{d['yaw']:+.2f}) "
              f"chassis=({d['vx']:+.2f},{d['vy']:+.2f},{d['w']:+.2f}) "
              f"gimbal=({d['yaw_mech']:+.2f},{d['pitch_mech']:+.2f}) " 
              f"gyro=({d['gx']:+.3f},{d['gy']:+.3f},{d['gz']:+.3f})")
        if motors:
            print(f"        {motors}")

    print(f"\n解析完成:好帧 {ok} 个,CRC 错 {bad_crc} 次")


if __name__ == "__main__":
    main()
