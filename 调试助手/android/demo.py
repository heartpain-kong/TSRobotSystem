# -*- coding: utf-8 -*-
"""演示数据源:没有板子/没有 OTG 线时,用合成 0x5A 帧驱动界面。

手机上第一次装完可以直接点「演示」看界面长什么样;也方便截图和自测。
帧格式与固件一致:8 字节头 + 148 字节 payload + 2 字节 CRC = 158 字节。
"""
import math
import struct
import threading
import time

import proto


def build_frame(pos, rpm, torque, roll, seq):
    """按 158B 全量帧(含 IMU / 遥控 / IM6014)拼一帧。"""
    pl = bytearray(148)
    pl[0] = 0x02
    pl[1] = 0x03
    struct.pack_into("<9f", pl, 2,
                     0.0, rpm, 120.0, rpm, 120.0 - rpm, torque,
                     pos, pos + 0.05, 0.05)
    pl[38], pl[39] = 0, 1                      # 光电 PE2=0,PE3=1
    pl[40] = 0                                 # err_level = OK
    struct.pack_into("<H", pl, 42, 0)
    struct.pack_into("<I", pl, 44, 0)
    struct.pack_into("<I", pl, 48, 0)
    struct.pack_into("<I", pl, 52, 0)
    struct.pack_into("<I", pl, 56, 0)
    struct.pack_into("<6f", pl, 60, roll, roll * 0.5, roll * 2.0,
                     math.sin(roll) * 0.2, 0.05, -0.05)
    pl[84] = 1
    pl[85], pl[86] = 1, 3
    pl[87], pl[88] = 1, 0
    struct.pack_into("<H", pl, 90, 0x0015)
    struct.pack_into("<5h", pl, 92, int(660 * math.sin(pos)), 660, -660, 100, -100)
    struct.pack_into("<3h", pl, 102, 5, -5, 0)
    struct.pack_into("<H", pl, 108, 2)
    struct.pack_into("<I", pl, 112, seq)
    pl[116], pl[117] = 1, 1
    struct.pack_into("<h", pl, 118, 500)
    struct.pack_into("<I", pl, 120, 0)
    struct.pack_into("<5f", pl, 124, proto.wrap_2pi(pos), rpm * 0.1, torque * 0.8,
                     41.5 + 0.5 * math.sin(pos), 24.2)
    struct.pack_into("<H", pl, 144, 3)
    struct.pack_into("<H", pl, 146, 0)

    frame = (bytes([proto.SOF_DEBUG, 0x00]) + struct.pack("<H", 158)
             + struct.pack("<H", 0x0102) + struct.pack("<H", seq & 0xFFFF)
             + bytes(pl))
    return frame + struct.pack("<H", proto.crc_ccitt(frame))


class DemoSource(threading.Thread):
    """20 Hz 往 sink 里塞合成帧,接口与 serial_link.Reader 一致(data 消息)。"""

    def __init__(self, sink, hz=20.0):
        super().__init__(daemon=True)
        self.sink = sink
        self.dt = 1.0 / float(hz)
        self.stop_flag = threading.Event()
        self.seq = 0

    def run(self):
        t0 = time.time()
        while not self.stop_flag.is_set():
            t = time.time() - t0
            self.seq += 1
            pos = proto.wrap_2pi(t * 1.2)
            rpm = 60.0 + 55.0 * math.sin(t * 2.0)
            torque = 0.35 * math.sin(t * 2.0 + 0.7)
            roll = 0.25 * math.sin(t * 1.5)
            self.sink.put(("data", build_frame(pos, rpm, torque, roll, self.seq)))
            time.sleep(self.dt)

    def stop(self):
        self.stop_flag.set()
