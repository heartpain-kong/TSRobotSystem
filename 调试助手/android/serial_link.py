# -*- coding: utf-8 -*-
"""串口层:Android 走 USB-OTG(usb4a + usbserial4a),桌面预览走 pyserial。

两台机器用的是同一套上层接口,所以 main.py 不用关心底层是 COMx 还是
USB 设备节点。Android 上的设备名形如 /dev/bus/usb/001/002。
"""
import os
import sys
import threading
import time

# 只要装了 usb4a/usbserial4a 就走 USB-OTG(不管是不是 buildozer 打的包,
# Pydroid 里跑也一样);导入失败就会退到 pyserial。
HAS_USB4A = False
usb = None
serial4a = None
try:
    from usb4a import usb
    from usbserial4a import serial4a
    HAS_USB4A = True
except Exception:
    HAS_USB4A = False

# Kivy 的 Android 引导脚本会设置这些环境变量;Pydroid 没有,用 build.prop 兜底
IS_ANDROID = bool(os.environ.get("ANDROID_ARGUMENT")
                  or os.environ.get("P4A_BOOTSTRAP")
                  or sys.platform == "android"
                  or (HAS_USB4A and os.path.exists("/system/build.prop")))

try:
    import serial                  # 桌面预览:COMx / /dev/ttyUSB*
    from serial.tools import list_ports
    HAS_SERIAL = True
except Exception:
    HAS_SERIAL = False


class PortInfo:
    """一个可选串口:device 是给 open_port 用的名字,label 是界面显示。"""

    def __init__(self, device, label=""):
        self.device = device
        self.label = label or device

    def __str__(self):
        return self.label


def backend_name():
    if HAS_USB4A:
        return "USB-OTG (usbserial4a)"
    if HAS_SERIAL:
        return "pyserial"
    return "无串口后端"


def list_ports():
    """列出串口。Android 上没有权限也能列出来,打开时才申请权限。"""
    out = []
    if HAS_USB4A:
        try:
            for dev in usb.get_usb_device_list():
                try:
                    name = dev.getDeviceName()
                    label = " ".join(x for x in (dev.getProductName(),
                                                 dev.getManufacturerName()) if x)
                    vid = dev.getVendorId()
                    pid = dev.getProductId()
                    label = "%s [%04X:%04X]" % (label or "USB 串口", vid, pid)
                except Exception:
                    name, label = dev.getDeviceName(), "USB 设备"
                out.append(PortInfo(name, label))
        except Exception:
            pass
        return out
    if HAS_SERIAL:
        try:
            for p in list_ports.comports():
                desc = (p.description or "").strip()
                out.append(PortInfo(p.device, "%s  %s" % (p.device, desc) if desc else p.device))
        except Exception:
            pass
    return out


def _wait_permission(device, timeout=15.0):
    """首次插上会弹“允许访问 USB 设备”,等用户点允许。"""
    if not HAS_USB4A:
        return True
    try:
        if usb.has_usb_permission(device):
            return True
        usb.request_usb_permission(device)
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(0.1)
            if usb.has_usb_permission(device):
                return True
    except Exception:
        return False
    return False


def open_port(name, baud, timeout=0.05):
    """打开串口,返回一个 pyserial 风格的句柄(read/write/in_waiting/timeout/close)。"""
    if HAS_USB4A:
        device = usb.get_usb_device(name)
        if device is None:
            raise IOError("USB 设备不在了,重新插一下再刷新")
        if not _wait_permission(device):
            raise IOError("没拿到 USB 权限:请在弹窗里点“允许”(或重新插拔)")
        port = serial4a.get_serial_port(name, int(baud), 8, "N", 1, timeout=timeout)
        if not getattr(port, "is_open", True):
            raise IOError("串口没打开(USB 权限被拒绝?)")
        return port
    if HAS_SERIAL:
        return serial.Serial(name, int(baud), timeout=timeout)
    raise IOError("没有可用的串口后端:桌面装 pyserial,Android 装 usbserial4a")


def close_port(port):
    try:
        port.close()
    except Exception:
        pass


class Reader(threading.Thread):
    """后台读串口:把收到的字节块丢进队列,界面线程只做解析和画图。"""

    def __init__(self, name, baud, sink):
        super().__init__(daemon=True)
        self.name_ = name
        self.baud = int(baud)
        self.sink = sink
        self.stop_flag = threading.Event()
        self.error = None
        self.port = None
        self._lock = threading.Lock()

    def run(self):
        try:
            port = open_port(self.name_, self.baud)
        except Exception as exc:
            self.error = str(exc)
            return
        self.port = port
        try:
            while not self.stop_flag.is_set():
                try:
                    n = port.in_waiting
                except Exception:
                    n = 0
                if n:
                    try:
                        data = port.read(min(n, 8192))
                    except Exception as exc:
                        self.error = str(exc)
                        break
                    if data:
                        self.sink.put(("data", bytes(data)))
                else:
                    time.sleep(0.01)
        finally:
            self.port = None
            close_port(port)

    def write(self, data):
        with self._lock:
            if self.port is None:
                raise IOError("串口没打开")
            self.port.write(data)

    def stop(self):
        self.stop_flag.set()


def probe_baud(name, baud, seconds=1.2):
    """试一个波特率:这段时间里解出过有效帧就算匹配。"""
    from proto import FrameDecoder

    try:
        port = open_port(name, baud)
    except Exception:
        return False
    dec = FrameDecoder()
    ok = False
    try:
        deadline = time.time() + seconds
        while time.time() < deadline:
            try:
                n = port.in_waiting
            except Exception:
                n = 0
            if n:
                try:
                    dec.feed(port.read(min(n, 8192)) or b"")
                except Exception:
                    break
                if dec.stats["DEBUG"] or dec.stats["A5"]:
                    ok = True
                    break
            else:
                time.sleep(0.02)
    finally:
        close_port(port)
    return ok
