#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
调试助手 — STM32 机器人串口调试工具(Windows / Ubuntu 通用)

同时接收两路 MCU 数据,并导出"带时间和串口号"的故障排查文档:
  · 调试遥测   帧头 0x5A(兼容 30/38/40 B 小帧与旧 210 B 大帧)
  · 上位机协议 帧头 0xA5,变长(USB CDC 的 miniPC / 裁判系统)

特色:
  ★ 自动匹配端口:扫描系统串口 × 常见波特率,收到有效帧自动分配
  ★ 跨平台:Windows 用 COMx,Ubuntu 用 /dev/ttyUSB*、/dev/ttyACM*
  ★ 调试窗口:电机 / 舵轮 / 底盘 / 云台 / VOFA 波形,都是按需打开
  ★ 可换背景:深色 / 浅色 / 护眼绿 / 暖纸 + 自定义颜色
  ★ 背景图:GIF(动图)/PNG/JPG 铺满曲线画布,也能透到字段表和日志(自绘 Canvas)
  ★ 暂停显示但继续记录;HEX 发送;导出 CSV/Markdown

依赖: pip3 install pyserial
可选: pip3 install pillow(装了才能用 JPG/WebP 等格式,GIF 也更顺)
Ubuntu 串口权限: sudo usermod -aG dialout $USER(重新登录生效)
"""

import csv
import json
import math
import os
import queue
import collections
import struct
import sys
import threading
import time
import traceback
import tkinter as tk
from datetime import datetime
from tkinter import colorchooser, filedialog, messagebox, ttk
from tkinter import font as tkfont

# ============================================================================
# 平台配置(两份文件的唯一差异)
# ============================================================================

IS_WINDOWS = (os.name == "nt")
IS_LINUX = sys.platform.startswith("linux")
CONFIG = {
    "platform": "Windows" if IS_WINDOWS else ("Ubuntu" if IS_LINUX else sys.platform),
    "default_ports": [],   # 只用系统实际枚举到的串口,避免记住已失效的 COM23
    "baud_list": ["921600", "115200", "100000", "4000000"],
    "font": ("Microsoft YaHei UI", 10) if IS_WINDOWS else ("DejaVu Sans", 10),
    "font_big": ("Microsoft YaHei UI", 11, "bold") if IS_WINDOWS else ("DejaVu Sans", 11, "bold"),
    "csv_encoding": "utf-8-sig",
}

# ============================================================================
# 协议常量
# ============================================================================

SOF_DEBUG = 0x5A
SOF_A5 = 0xA5
DEBUG_FRAME = 230          # 8 头 + 200 payload + 2 CRC(含陀螺仪/加速度)
DEBUG_PAYLOAD = 220
DEBUG_MOTOR_SIZE = 16
DEBUG_PL_DIAG = 80     # 诊断块 12B(jedec/flags/acc_conf/bmi_err/tick)
DEBUG_PL_ACCEL = 92    # 3×f32 加速度 m/s^2
DEBUG_PL_HWM = 104      # 8×u16 各任务栈水位(单位:字=4B)
DEBUG_PL_LOG = 120      # 5×u32 日志上电链
DEBUG_PL_TEMP = 140
DEBUG_PL_TICKS = 144    # 8×u32 各任务心跳
DEBUG_PL_FLASH = 176
DEBUG_PL_EXT = 196      # 3x f32 gyro + 3x f32 accel (external IMU)    # 5×u32 Flash 诊断     # f32 BMI088 温度(判断加速度通路)
DEBUG_MAX_MOTORS = 8
A5_MIN_FRAME = 5

LEVEL_NAME = {0: "OK", 1: "WARN", 2: "ERROR", 3: "FATAL"}
LEVEL_COLOR = {0: "#2e9e5b", 1: "#d8a600", 2: "#e07b00", 3: "#d63031"}
FAULT_NAME = {0: "正常", 1: "RTOS堆失败", 2: "任务栈溢出", 3: "Error_Handler", 4: "HardFault"}
MOTOR_FLAG = [(0x01, "init"), (0x02, "online"), (0x04, "MError")]
RC_SWITCH_NAME = {1: "上", 2: "中", 3: "下", 0: "?"}
RC_KEYS = [(0x0001, "W"), (0x0002, "S"), (0x0004, "A"), (0x0008, "D"),
           (0x0010, "SHIFT"), (0x0020, "CTRL"), (0x0040, "Q"), (0x0080, "E"),
           (0x0100, "R"), (0x0200, "F"), (0x0400, "G"), (0x0800, "Z"),
           (0x1000, "X"), (0x2000, "C"), (0x4000, "V"), (0x8000, "B")]

# ============================================================================
# 主题(可换背景)
# ============================================================================

THEMES = {
    "深色":   {"bg": "#1b1f27", "panel": "#252b35", "fg": "#e8ecf1", "muted": "#9aa4b2",
               "accent": "#2f9e6e", "log_bg": "#12161c", "sel": "#38506b", "entry": "#2f3742"},
    "浅色":   {"bg": "#eef1f5", "panel": "#ffffff", "fg": "#1c2028", "muted": "#6b7280",
               "accent": "#1f7a5c", "log_bg": "#fbfcfd", "sel": "#cfe0f5", "entry": "#ffffff"},
    "护眼绿": {"bg": "#c9edcf", "panel": "#e9f8ec", "fg": "#12351c", "muted": "#3d6b48",
               "accent": "#2e7d32", "log_bg": "#e0f4e3", "sel": "#aedcbd", "entry": "#f4fdf5"},
    "暖纸":   {"bg": "#f2e8d5", "panel": "#fbf5e8", "fg": "#2b2418", "muted": "#7a6a4f",
               "accent": "#8a6d3b", "log_bg": "#f8f1e0", "sel": "#e6d8bb", "entry": "#fffdf7"},
}

CFG_FILE = os.path.join(os.path.expanduser("~"), ".debug_assistant.json")


def _custom_pal(color):
    pal = dict(THEMES["浅色"])
    pal["bg"] = color
    pal["panel"] = color
    pal["log_bg"] = color
    pal["entry"] = color
    return pal


# ============================================================================
# 背景图(静态图 / GIF 动图)
# ============================================================================
# 有 Pillow 就用它:格式全(JPG/WebP/BMP...),动图逐帧合成、缩放平滑;
# 没有 Pillow 退化到 Tk 自带解码:PNG + GIF(用 "gif -index N" 取每一帧)。
try:
    from PIL import Image as _PIL_Image
    from PIL import ImageTk as _PIL_ImageTk
    _HAS_PIL = True
except Exception:                          # 没装 Pillow 也能跑
    _PIL_Image = None
    _PIL_ImageTk = None
    _HAS_PIL = False

BG_MAX_SIDE = 1600                  # 单帧长边上限(像素)
BG_MAX_FRAMES = 600                 # 帧数上限,防止超大动图把内存吃光
BG_MEM_BUDGET = 96 * 1024 * 1024    # 所有源帧占用的上限(字节)
BG_CACHE_BUDGET = 32 * 1024 * 1024  # 缩放后画面缓存上限(字节)
BG_TAG = "bgimg"                    # 画布上背景图 item 的 tag
BG_DIM = 0.45                       # 默认压暗比例,保证曲线/文字看得清
# 没有 Pillow 时用点阵填充近似"半透明压暗"
BG_STIPPLE = ((0.10, None), (0.30, "gray12"), (0.50, "gray25"),
              (0.70, "gray50"), (1.01, "gray75"))


def image_filetypes():
    """文件对话框里的格式过滤。"""
    if _HAS_PIL:
        return [("图片/动图", "*.gif *.png *.jpg *.jpeg *.bmp *.webp *.tif *.tiff *.ppm *.pgm"),
                ("动图 GIF", "*.gif"),
                ("PNG", "*.png"),
                ("所有文件", "*.*")]
    return [("GIF / PNG(Tk 自带解码)", "*.gif *.png *.ppm *.pgm"),
            ("动图 GIF", "*.gif"),
            ("所有文件", "*.*")]


def _dim_stipple(dim):
    for limit, pattern in BG_STIPPLE:
        if dim < limit:
            return pattern
    return "gray75"


def _gif_delays(path):
    """从 GIF 文件里读出每帧延时(ms):0x21 0xF9 扩展块里是 1/100 秒。"""
    try:
        with open(path, "rb") as fp:
            blob = fp.read()
    except OSError:
        return []
    if len(blob) < 14 or not blob.startswith((b"GIF87a", b"GIF89a")):
        return []
    delays = []
    n = len(blob)
    i = 13
    gct = blob[10]
    if gct & 0x80:                                  # 全局调色板
        i += 3 * (1 << ((gct & 0x07) + 1))

    def skip_blocks(i):
        while i < n:
            size = blob[i]
            i += 1
            if size == 0:
                return i
            i += size
        return i

    try:
        while i < n:
            tag = blob[i]
            if tag == 0x3B:                         # 结束
                break
            if tag == 0x21:                         # 扩展块
                if blob[i + 1] == 0xF9 and i + 7 < n and blob[i + 2] == 4:
                    d = blob[i + 4] | (blob[i + 5] << 8)
                    delays.append(max(20, (d or 10) * 10))
                i = skip_blocks(i + 2)
            elif tag == 0x2C:                       # 图像描述符
                flags = blob[i + 9]
                i += 10
                if flags & 0x80:
                    i += 3 * (1 << ((flags & 0x07) + 1))
                i = skip_blocks(i + 1)              # +1 跳过 LZW 最小码长
            else:
                break
    except (IndexError, ValueError):
        pass
    return delays


class Wallpaper:
    """一张背景图:统一解码成若干帧,静态图只有 1 帧。"""

    def __init__(self, path):
        self.path = path
        self.frames = []        # list[PIL.Image] 或 list[tk.PhotoImage]
        self.delays = []        # 每帧显示时长 ms
        self.pillow = _HAS_PIL
        self.truncated = False
        if not os.path.isfile(path):
            raise ValueError("找不到文件:%s" % path)
        if self.pillow:
            self._load_pillow(path)
        else:
            self._load_tk(path)
        if not self.frames:
            raise ValueError("这个文件里没有能显示的图像帧")

    @property
    def count(self):
        return len(self.frames)

    @property
    def static(self):
        return len(self.frames) <= 1

    @property
    def size(self):
        f = self.frames[0]
        return f.size if self.pillow else (f.width(), f.height())

    def delay(self, i):
        if not self.delays:
            return 100
        return max(30, min(5000, self.delays[i % len(self.delays)]))

    # ---- 解码:Pillow ----
    def _load_pillow(self, path):
        im = _PIL_Image.open(path)
        try:
            n = int(getattr(im, "n_frames", 1) or 1)
        except Exception:
            n = 1
        if n > BG_MAX_FRAMES:
            n, self.truncated = BG_MAX_FRAMES, True
        iw, ih = im.size
        scale = min(1.0, BG_MAX_SIDE / float(max(iw, ih)))
        # 帧多/图大时继续缩,保证 n 帧总量不超过预算
        while n > 1 and scale > 0.12 and n * (iw * scale) * (ih * scale) * 3 > BG_MEM_BUDGET:
            scale *= 0.8
        tw = max(1, int(round(iw * scale)))
        th = max(1, int(round(ih * scale)))
        for i in range(n):
            try:
                im.seek(i)
            except EOFError:
                break
            except Exception:
                break
            if im.mode in ("RGB", "L"):
                fr = im.convert("RGB")
            else:                                   # 带透明通道的统一铺到黑底
                rgba = im.convert("RGBA")
                fr = _PIL_Image.new("RGB", rgba.size, (0, 0, 0))
                fr.paste(rgba, mask=rgba.split()[3])
            if fr.size != (tw, th):
                fr = fr.resize((tw, th), _PIL_Image.LANCZOS)
            self.frames.append(fr)
            self.delays.append(int(im.info.get("duration", 0) or 0) or 100)
        im.close()

    # ---- 解码:Tk 自带(只认 GIF/PNG/PPM/PGM) ----
    def _load_tk(self, path):
        ext = os.path.splitext(path)[1].lower()
        if ext == ".gif":
            for i in range(BG_MAX_FRAMES):
                try:
                    photo = tk.PhotoImage(file=path, format="gif -index %d" % i)
                except tk.TclError:
                    break
                except RuntimeError as exc:        # 还没有 Tk 窗口(正常流程不会走到)
                    raise ValueError("Tk 自带解码需要一个窗口:%s" % exc)
                self.frames.append(photo)
            self.delays = _gif_delays(path)
            self.truncated = len(self.frames) >= BG_MAX_FRAMES
            return
        try:
            self.frames.append(tk.PhotoImage(file=path))
        except RuntimeError as exc:
            raise ValueError("Tk 自带解码需要一个窗口:%s" % exc)
        except tk.TclError as exc:
            raise ValueError("Tk 只能直接读 GIF/PNG/PPM;JPG 等格式请先装 Pillow\n(pip install pillow) [%s]" % exc)


class BackgroundLayer:
    """管理背景图:给画布铺图、按窗口尺寸缩放、按帧延时播放动图。"""

    def __init__(self, app):
        self.app = app
        self.wp = None
        self.dim = BG_DIM
        self._idx = 0
        self._job = None
        self._canvases = []                 # 挂上的画布
        self._extra = {}                    # 画布 -> 额外压暗量
        self._align = {}                    # 画布 -> 按哪个窗口对齐(整窗一张图)
        self._shown = {}                    # 画布 -> 当前显示的 PhotoImage(防回收)
        self._state = {}                    # 画布 -> (w, h, 帧号, 压暗)
        self._cache = collections.OrderedDict()   # (画布, w, h, 帧号) -> PhotoImage
        self._cache_bytes = 0

    # ---- 状态 ----
    @property
    def active(self):
        return self.wp is not None

    @property
    def animated(self):
        return self.wp is not None and not self.wp.static

    def info(self):
        if self.wp is None:
            return "无背景图"
        kind = "动图 %d 帧" % self.wp.count if self.animated else "静态图"
        engine = "Pillow" if self.wp.pillow else "Tk 解码"
        return "%s · %s · %dx%d · %s" % (os.path.basename(self.wp.path), kind,
                                         self.wp.size[0], self.wp.size[1], engine)

    # ---- 装载/卸载 ----
    def load(self, path):
        wp = Wallpaper(path)
        self.stop()
        self._drop_all()
        self.wp = wp
        self._idx = 0
        self.paint_all()
        self.start()
        return wp

    def unload(self):
        self.stop()
        for cv in list(self._canvases):
            self._delete(cv)
        self.wp = None
        self._drop_all()
        self._idx = 0

    def set_dim(self, value):
        value = max(0.0, min(0.9, float(value)))
        if abs(value - self.dim) < 0.005:
            return False
        self.dim = value
        self._drop_cache()
        self.paint_all()
        return True

    # ---- 画布登记 ----
    def attach(self, cv, extra_dim=0.0, align_to=None):
        """挂一块画布。

        extra_dim : 这块画布额外再压暗多少(表格/日志要更暗才好读字)
        align_to  : 传窗口对象时,这块画布画"整窗那张图"里属于它自己的那一块,
                    这样几块画布拼起来才是一张完整的壁纸,而不是各自缩放一遍。
        """
        self._extra[cv] = float(extra_dim)
        if align_to is not None:
            self._align[cv] = align_to
        if cv in self._canvases:
            self._state.pop(cv, None)
            self.paint(cv)
            return
        self._canvases.append(cv)
        try:
            cv.bind("<Configure>", lambda _e, c=cv: self.paint(c), add="+")
        except Exception:
            pass
        self.paint(cv)

    def dim_for(self, cv):
        return min(0.92, self.dim + self._extra.get(cv, 0.0))

    def detach(self, cv):
        while cv in self._canvases:
            self._canvases.remove(cv)
        self._extra.pop(cv, None)
        self._align.pop(cv, None)
        self._shown.pop(cv, None)
        self._state.pop(cv, None)
        for key in [k for k in self._cache if k[0] is cv]:
            self._cache.pop(key, None)
        self._recount()

    # ---- 绘制 ----
    def paint(self, cv):
        """把当前帧铺到画布最底层;没背景图、画布还没布局时什么也不做。"""
        if not self._alive(cv):
            return
        try:
            w, h = cv.winfo_width(), cv.winfo_height()
        except tk.TclError:
            return
        if w <= 2 or h <= 2:
            return
        if self.wp is None or not self.wp.frames:
            self._delete(cv)
            return
        idx = self._idx % self.wp.count
        dim = self.dim_for(cv)

        target = self._align.get(cv)
        if target is not None and target is not cv:
            # 整窗一张图:取"窗口尺寸的那张图"里属于本画布的那一块
            try:
                W, H = target.winfo_width(), target.winfo_height()
                ox = cv.winfo_rootx() - target.winfo_rootx()
                oy = cv.winfo_rooty() - target.winfo_rooty()
            except tk.TclError:
                return
            if W <= 2 or H <= 2:
                return
            state = (w, h, W, H, ox, oy, idx, round(dim, 3))
            if self._state.get(cv) == state and cv.find_withtag(BG_TAG):
                return
            photo = self._prepared(target, W, H, idx, dim)
            if photo is None:
                return
            self._delete(cv)
            cv.create_image(-ox, -oy, anchor="nw", image=photo, tags=BG_TAG)
            self._shown[cv] = photo
            if not _HAS_PIL and dim >= 0.10:
                cv.create_rectangle(0, 0, w, h, fill="#000000", outline="",
                                    stipple=_dim_stipple(dim), tags=BG_TAG)
            cv.tag_lower(BG_TAG)
            self._state[cv] = state
            return

        state = (w, h, idx, round(dim, 3))
        if self._state.get(cv) == state and cv.find_withtag(BG_TAG):
            return                          # 尺寸/帧号都没变,别重复画
        photo = self._prepared(cv, w, h, idx, dim)
        if photo is None:
            return
        self._delete(cv)
        self._tile(cv, photo, w, h)
        if not _HAS_PIL and dim >= 0.10:        # 无 Pillow:点阵压暗
            cv.create_rectangle(0, 0, w, h, fill="#000000", outline="",
                                stipple=_dim_stipple(dim), tags=BG_TAG)
        cv.tag_lower(BG_TAG)
        self._state[cv] = state

    def paint_all(self):
        alive = []
        for cv in list(self._canvases):
            if self._alive(cv):
                alive.append(cv)
            else:
                self.detach(cv)
        self._canvases = alive
        for cv in alive:
            self.paint(cv)

    def _tile(self, cv, photo, w, h):
        pw, ph = photo.width(), photo.height()
        if pw <= 0 or ph <= 0:
            return
        if pw >= w and ph >= h:                       # 居中裁切,保留中间
            cv.create_image(-(pw - w) // 2, -(ph - h) // 2, anchor="nw",
                            image=photo, tags=BG_TAG)
            return
        nx, ny = (w + pw - 1) // pw, (h + ph - 1) // ph
        if nx * ny > 400:                             # 图太小就别铺满屏了
            cv.create_image(w // 2, h // 2, anchor="center", image=photo, tags=BG_TAG)
            return
        for y in range(0, h, ph):
            for x in range(0, w, pw):
                cv.create_image(x, y, anchor="nw", image=photo, tags=BG_TAG)

    def _delete(self, cv):
        try:
            cv.delete(BG_TAG)
        except tk.TclError:
            pass
        self._state.pop(cv, None)

    # ---- 一帧画面(按目标尺寸缩放 + 压暗),带缓存 ----
    def _prepared(self, cv, w, h, idx, dim=None):
        if dim is None:
            dim = self.dim_for(cv)
        key = (cv, w, h, idx, round(dim, 3))
        photo = self._cache.get(key)
        if photo is not None:
            self._cache.move_to_end(key)
            return photo
        photo = self._make(w, h, idx, dim)
        if photo is None:
            return None
        self._cache[key] = photo
        self._shown[cv] = photo
        self._cache_bytes += w * h * 4
        while len(self._cache) > 1 and self._cache_bytes > BG_CACHE_BUDGET:
            old, _p = self._cache.popitem(last=False)
            self._cache_bytes -= old[1] * old[2] * 4
            if self._cache_bytes < 0:
                self._cache_bytes = 0
        return photo

    def _make(self, w, h, idx, dim=None):
        if dim is None:
            dim = self.dim
        try:
            frame = self.wp.frames[idx]
        except IndexError:
            return None
        if self.wp.pillow:
            iw, ih = frame.size
            scale = max(w / float(iw), h / float(ih))
            tw = max(w, int(round(iw * scale)))
            th = max(h, int(round(ih * scale)))
            try:
                img = frame.resize((tw, th), _PIL_Image.BILINEAR)
                left, top = (tw - w) // 2, (th - h) // 2
                img = img.crop((left, top, left + w, top + h))
                if dim > 0.02:
                    img = _PIL_Image.blend(
                        img, _PIL_Image.new("RGB", img.size, (0, 0, 0)),
                        min(0.92, dim))
                return _PIL_ImageTk.PhotoImage(img)
            except Exception:
                return None
        # Tk 内置:只能整数倍缩放
        iw, ih = frame.width(), frame.height()
        photo = frame
        k = max(1, min(iw // max(w, 1), ih // max(h, 1)))
        if k > 1:
            try:
                photo = photo.subsample(k)
            except tk.TclError:
                return None
        if photo.width() < w and photo.height() < h:
            z = max(1, min(3, min(w // max(photo.width(), 1),
                                  h // max(photo.height(), 1))))
            if z > 1:
                try:
                    photo = photo.zoom(z)
                except tk.TclError:
                    return None
        return photo

    # ---- 动图播放 ----
    def start(self):
        self.stop()
        if self.animated:
            self._schedule(self.wp.delay(self._idx))

    def stop(self):
        if self._job is not None:
            try:
                self.app.after_cancel(self._job)
            except Exception:
                pass
            self._job = None

    def _schedule(self, ms):
        try:
            self._job = self.app.after(max(20, int(ms)), self._tick)
        except Exception:
            self._job = None

    def _tick(self):
        self._job = None
        if not self.animated:
            return
        visible = [cv for cv in list(self._canvases)
                   if self._alive(cv) and self._mapped(cv)]
        if not visible:                      # 窗口都关掉/最小化了,低频等着,别空转
            self._schedule(800)
            return
        self._idx = (self._idx + 1) % self.wp.count
        t0 = time.time()
        for cv in visible:
            self.paint(cv)
        delay = self.wp.delay(self._idx)
        spent = int((time.time() - t0) * 1000) + 5
        if spent > delay:                    # 机器跟不上就别排队,按实耗走
            delay = spent
        self._schedule(delay)

    # ---- 杂项 ----
    @staticmethod
    def _alive(cv):
        try:
            return bool(int(cv.winfo_exists()))
        except Exception:
            return False

    @staticmethod
    def _mapped(cv):
        try:
            return bool(cv.winfo_ismapped())
        except Exception:
            return False

    def _drop_cache(self):
        self._cache.clear()
        self._cache_bytes = 0
        self._state.clear()

    def _drop_all(self):
        self._drop_cache()
        self._shown.clear()

    def _recount(self):
        self._cache_bytes = sum(k[1] * k[2] * 4 for k in self._cache)


# ============================================================================
# CRC
# ============================================================================

def crc8(data: bytes, crc: int = 0xFF) -> int:
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def crc_ccitt(data: bytes, crc: int = 0x0000) -> int:
    """CRC-CCITT(★反射版:poly 0x8408,初值 0)—— 与固件 User/algorithm/crc/crc_ccitt.h 完全一致 ✓
       注意:不是标准 CCITT(0x1021/0xFFFF)✗ 两者结果完全不同 ✗"""
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


# ============================================================================
# 角度归一化
# ============================================================================

TWO_PI = 2.0 * math.pi


def wrap_2pi(rad: float) -> float:
    """把角度归一化到 [0, 2π);正好 2π 归 0。"""
    a = math.fmod(rad, TWO_PI)
    if a < 0.0:
        a += TWO_PI
    if a >= TWO_PI - 1e-12:
        a = 0.0
    return a


def wrap_pi(rad: float) -> float:
    """把误差归一化到 [-π, π],用于显示最短角误差。"""
    a = math.fmod(rad + math.pi, TWO_PI)
    if a < 0.0:
        a += TWO_PI
    return a - math.pi


# ============================================================================
# 解析
# ============================================================================

def parse_debug(frame: bytes) -> dict:
    """★ 按帧里真实的长度字段解:老固件 186 B(无陀螺仪)/ 新固件 210 B(带陀螺仪)都支持 ✓"""
    n = len(frame) - 10                      # payload 长度 = 整帧 - 头(8) - CRC(2)
    pl = frame[8:8 + n]
    cmd = struct.unpack_from("<H", frame, 4)[0]

    # 舵向实时角度帧:cmd=0x0104,payload 16B(4×f32)
    if cmd == 0x0104 and n >= 16:
        s0n, s0t, s1n, s1t = struct.unpack_from("<4f", pl, 0)
        return {
            "proto": "STEER",
            "seq": struct.unpack_from("<H", frame, 6)[0],
            "cmd_id": cmd,
            "steer0.now": s0n,
            "steer0.tgt": s0t,
            "steer1.now": s1n,
            "steer1.tgt": s1t,
        }

    # 极简电机 PID 帧:cmd=0x0102,payload 30B
    if cmd == 0x0102 and n >= 30:
        mid = pl[0]
        flags = pl[1]
        if n >= 38:
            (joint_rpm, motor_rpm, target_rpm, feedback_rpm, err_rpm,
             torque, pos_raw, pos_target_raw, pos_err_raw) = struct.unpack_from("<9f", pl, 2)
        else:
            # 兼容上一版 30B 小帧(还没有位置目标/误差)
            (joint_rpm, motor_rpm, target_rpm, feedback_rpm, err_rpm,
             torque, pos_raw) = struct.unpack_from("<7f", pl, 2)
            pos_target_raw, pos_err_raw = pos_raw, 0.0
        # 记录/显示统一归一化到 0~360°;误差用最短角 [-180°,180°]
        pos = wrap_2pi(pos_raw)
        pos_target = wrap_2pi(pos_target_raw)
        pos_err = wrap_pi(pos_target_raw - pos_raw)
        pe2_level = pl[38] if n >= 39 else 0
        pe3_level = pl[39] if n >= 40 else 0
        photo = 1 if pe2_level == 0 else 0   # 低有效
        if n >= 60:
            err_level = pl[40]
            err_code = struct.unpack_from("<H", pl, 42)[0]
            fault_code = struct.unpack_from("<I", pl, 44)[0]
            fault_cfsr = struct.unpack_from("<I", pl, 48)[0]
            fault_pc = struct.unpack_from("<I", pl, 52)[0]
            err_flags = struct.unpack_from("<I", pl, 56)[0]
        else:
            err_level = 0 if (flags & 2) else 2
            err_code = 0
            fault_code = 0
            fault_cfsr = 0
            fault_pc = 0
            err_flags = 0

        # 60 起:IMU 欧拉角(rad)+ 角速度(rad/s);旧 60B 帧没有这块
        if n >= 84:
            (imu_roll, imu_pitch, imu_yaw,
             imu_gx, imu_gy, imu_gz) = struct.unpack_from("<6f", pl, 60)
            has_imu = True
        else:
            imu_roll = imu_pitch = imu_yaw = 0.0
            imu_gx = imu_gy = imu_gz = 0.0
            has_imu = False

        # 84 起:遥控器/键鼠 32B
        if n >= 116:
            rc_online = pl[84]
            rc_sw = [pl[85], pl[86]]
            rc_mouse_l = pl[87]
            rc_mouse_r = pl[88]
            rc_key = struct.unpack_from("<H", pl, 90)[0]
            rc_ch = list(struct.unpack_from("<5h", pl, 92))
            rc_mouse = list(struct.unpack_from("<3h", pl, 102))
            rc_lost = struct.unpack_from("<H", pl, 108)[0]
            rc_rx = struct.unpack_from("<I", pl, 112)[0]
            has_rc = True
        else:
            rc_online = 0
            rc_sw = [0, 0]
            rc_mouse_l = rc_mouse_r = 0
            rc_key = 0
            rc_ch = [0, 0, 0, 0, 0]
            rc_mouse = [0, 0, 0]
            rc_lost = 0
            rc_rx = 0
            has_rc = False

        # 116 起:IM6014 单电机反馈 32B
        if n >= 148:
            ys_online = pl[116]
            ys_status = pl[117]
            ys_target = struct.unpack_from("<h", pl, 118)[0] / 1000.0
            ys_m_error = struct.unpack_from("<I", pl, 120)[0]
            ys_pos, ys_w, ys_t, ys_temp, ys_voltage = struct.unpack_from("<5f", pl, 124)
            ys_crc = struct.unpack_from("<H", pl, 144)[0]
            ys_warn = struct.unpack_from("<H", pl, 146)[0]
            has_ys = True
        else:
            ys_online = ys_status = 0
            ys_target = 0.0
            ys_m_error = 0
            ys_pos = ys_w = ys_t = ys_temp = ys_voltage = 0.0
            ys_crc = ys_warn = 0
            has_ys = False

        # 116 起:0x207/0x208 摩擦轮
        if n >= 144:
            fr_flags = [pl[116], pl[117]]
            fr_l = list(struct.unpack_from("<3f", pl, 120))
            fr_r = list(struct.unpack_from("<3f", pl, 132))
            has_friction = True
        else:
            fr_flags = [0, 0]
            fr_l = [0.0, 0.0, 0.0]
            fr_r = [0.0, 0.0, 0.0]
            has_friction = False

        # 144 起:0x201 drive fixed block
        m201 = None
        if n >= 160:
            f201 = pl[144]
            p201, w201, t201 = struct.unpack_from("<3f", pl, 148)
            m201 = {"id": 1, "flags": f201,
                    "pos": p201, "pos_raw": p201, "w": w201, "t": t201,
                    "init": bool(f201 & 1), "online": bool(f201 & 2),
                    "err": bool(f201 & 4)}
            if not (f201 & 1):
                m201 = None

        # 160 起:0x203 steer / 0x204 drive fixed blocks
        m203 = m204 = None
        if n >= 192:
            f203 = pl[160]
            p203, w203, t203 = struct.unpack_from("<3f", pl, 164)
            m203 = {"id": 3, "flags": f203,
                    "pos": p203, "pos_raw": p203, "w": w203, "t": t203,
                    "init": bool(f203 & 1), "online": bool(f203 & 2),
                    "err": bool(f203 & 4)}
            f204 = pl[176]
            p204, w204, t204 = struct.unpack_from("<3f", pl, 180)
            tgt204 = ptgt204 = perr204 = 0.0
            if n >= 204:
                tgt204, ptgt204, perr204 = struct.unpack_from("<3f", pl, 192)
            m204 = {"id": 2, "flags": f204,
                    "pos": p204, "pos_raw": p204, "w": w204, "t": t204,
                    "target": tgt204, "pos_target": ptgt204, "pos_error": perr204,
                    "init": bool(f204 & 1), "online": bool(f204 & 2),
                    "err": bool(f204 & 4)}
            if not (f203 & 1):
                m203 = None
            if not (f204 & 1):
                m204 = None

        return {
            "proto": "MOTOR_PID",
            "seq": struct.unpack_from("<H", frame, 6)[0],
            "cmd_id": cmd,
            "motor_id": mid,
            "flags": flags,
            "err_level": err_level,
            "err_code": err_code,
            "fault_code": fault_code,
            "fault_cfsr": fault_cfsr,
            "fault_pc": fault_pc,
            "err_flags": err_flags,
            "has_imu": has_imu,
            "imu_roll": imu_roll,
            "imu_pitch": imu_pitch,
            "imu_yaw": imu_yaw,
            "imu_gx": imu_gx,
            "imu_gy": imu_gy,
            "imu_gz": imu_gz,
            "roll": imu_roll,
            "pitch": imu_pitch,
            "yaw": imu_yaw,
            "gx": imu_gx,
            "gy": imu_gy,
            "gz": imu_gz,
            "has_rc": has_rc,
            "rc_online": rc_online,
            "rc_sw0": rc_sw[0],
            "rc_sw1": rc_sw[1],
            "rc_mouse_l": rc_mouse_l,
            "rc_mouse_r": rc_mouse_r,
            "rc_key": rc_key,
            "rc_ch": rc_ch,
            "rc_ch_norm": [max(-1.0, min(1.0, v / 660.0)) for v in rc_ch],
            "rc_mouse": rc_mouse,
            "rc_lost": rc_lost,
            "rc_rx": rc_rx,
            "has_ys": has_ys,
            "ys_online": ys_online,
            "ys_status": ys_status,
            "ys_target": ys_target,
            "ys_m_error": ys_m_error,
            "ys_pos": ys_pos,
            "ys_w": ys_w,
            "ys_t": ys_t,
            "ys_temp": ys_temp,
            "ys_voltage": ys_voltage,
            "ys_crc": ys_crc,
            "ys_warn": ys_warn,
            "has_friction": has_friction,
            "fr_flags": fr_flags,
            "fr_l": fr_l,
            "fr_r": fr_r,
            "encoder_joint_rpm": joint_rpm,
            "encoder_motor_rpm": motor_rpm,
            "pid_target_rpm": target_rpm,
            "pid_feedback_rpm": feedback_rpm,
            "pid_error_rpm": err_rpm,
            "pid_torque": torque,
            "pos": pos,
            "pos_target": pos_target,
            "pos_error": pos_err,
            "pos_raw": pos_raw,
            "pos_fb": pos_raw,          # 舵轮实测角(原始连续值,未归一化)
            "pos_target_raw": pos_target_raw,
            "pos_error_raw": pos_err_raw,
            "photo": photo,
            "pe2_level": pe2_level,
            "pe3_level": pe3_level,
            "motor_count": 4 if m204 is not None else (2 if m201 is not None else 1),
            "motors": ([{
                "id": mid,
                "flags": flags,
                "pos": pos,
                "pos_fb": pos_raw,          # 舵轮实测角(原始,未归一化)
                "pos_target": pos_target,   # 闭环目标角
                "pos_error": pos_err,       # 目标 − 实测(最短角)
                "w": feedback_rpm * 2.0 * math.pi / 60.0,
                "t": torque,
                "init": bool(flags & 1),
                "online": bool(flags & 2),
                "err": bool(flags & 4),
            }] + ([m201] if m201 is not None else [])
                + ([m203, m204] if (m203 is not None and m204 is not None) else [])),
        }

    d = {
        "proto": "DEBUG",
        "seq": struct.unpack_from("<H", frame, 6)[0],
        "err_level": pl[0],
        "motor_count": pl[1],
        "err_code": struct.unpack_from("<H", pl, 2)[0],
        "fault_code": struct.unpack_from("<I", pl, 4)[0],
        "fault_cfsr": struct.unpack_from("<I", pl, 8)[0],
        "fault_pc": struct.unpack_from("<I", pl, 12)[0],
    }
    d["roll"], d["pitch"], d["yaw"] = struct.unpack_from("<3f", pl, 16)
    d["vx"], d["vy"], d["w"] = struct.unpack_from("<3f", pl, 28)
    d["gimbal_yaw"], d["gimbal_pitch"] = struct.unpack_from("<2f", pl, 40)
    if n >= (DEBUG_PL_EXT + 24):
        (d["ext_gx"], d["ext_gy"], d["ext_gz"],
         d["ext_ax"], d["ext_ay"], d["ext_az"]) = struct.unpack_from("<6f", pl, DEBUG_PL_EXT)
    else:
        for k in ("ext_gx", "ext_gy", "ext_gz", "ext_ax", "ext_ay", "ext_az"):
            d[k] = 0.0

    if n >= (DEBUG_PL_FLASH + 20):
        (d["flash_state"], d["flash_err"], d["flash_cmd"],
         d["flash_rcv"], d["flash_raw"]) = struct.unpack_from("<5I", pl, DEBUG_PL_FLASH)
    else:
        d["flash_state"] = d["flash_err"] = d["flash_cmd"] = d["flash_rcv"] = d["flash_raw"] = 0

    d["has_ticks"] = n >= (DEBUG_PL_TICKS + 32)
    if d["has_ticks"]:
        tk = struct.unpack_from("<8I", pl, DEBUG_PL_TICKS)
    else:
        tk = (0,) * 8
    (d["tick0"], d["tick1"], d["tick2"], d["tick3"],
     d["tick4"], d["tick5"], d["tick6"], d["tick7"]) = tk

    d["has_hwm"] = n >= (DEBUG_PL_HWM + 16)            # 有栈水位块(226B 帧)
    if d["has_hwm"]:
        (d["hwm_motor"], d["hwm_robot"], d["hwm_error"], d["hwm_cmd"],
         d["hwm_imu"], d["hwm_chassis"], d["hwm_gimbal"], d["hwm_highRise"]) = \
            struct.unpack_from("<8H", pl, DEBUG_PL_HWM)
    else:
        for k in ("hwm_motor", "hwm_robot", "hwm_error", "hwm_cmd",
                  "hwm_imu", "hwm_chassis", "hwm_gimbal", "hwm_highRise"):
            d[k] = 0

    d["has_log"] = n >= (DEBUG_PL_LOG + 20)            # 有日志上电链(246B 帧)
    if d["has_log"]:
        (d["log_boot"], d["log_cur_start"], d["log_prev_start"],
         d["log_prev_stop"], d["log_wrap"]) = struct.unpack_from("<5I", pl, DEBUG_PL_LOG)
    else:
        for k in ("log_boot", "log_cur_start", "log_prev_start", "log_prev_stop", "log_wrap"):
            d[k] = 0

    d["temp"] = struct.unpack_from("<f", pl, DEBUG_PL_TEMP)[0] if n >= (DEBUG_PL_TEMP + 4) else 0.0

    d["has_gyro"] = n >= (DEBUG_PL_ACCEL + 12)          # 老固件没有这两块 ✓
    if d["has_gyro"]:
        d["jedec"] = struct.unpack_from("<I", pl, DEBUG_PL_DIAG)[0]
        d["flags"] = pl[DEBUG_PL_DIAG + 4]
        d["acc_conf"] = pl[DEBUG_PL_DIAG + 5]
        d["bmi_err"] = struct.unpack_from("<H", pl, DEBUG_PL_DIAG + 6)[0]
        d["err_tick"] = struct.unpack_from("<I", pl, DEBUG_PL_DIAG + 8)[0]
        d["gx"] = d["gy"] = d["gz"] = 0.0      # 陀螺仪这版不发了
        d["ax"], d["ay"], d["az"] = struct.unpack_from("<3f", pl, DEBUG_PL_ACCEL)
    else:
        d["gx"] = d["gy"] = d["gz"] = 0.0
        d["ax"] = d["ay"] = d["az"] = 0.0
    motors = []
    for i in range(min(d["motor_count"], DEBUG_MAX_MOTORS)):
        o = 48 + i * DEBUG_MOTOR_SIZE
        flags = pl[o + 1]
        pos_raw, vel, tor = struct.unpack_from("<3f", pl, o + 4)
        motors.append({"id": pl[o], "flags": flags, "pos": wrap_2pi(pos_raw), "pos_raw": pos_raw,
                       "w": vel, "t": tor,
                       "init": bool(flags & 1), "online": bool(flags & 2), "err": bool(flags & 4)})
    d["motors"] = motors
    return d


def try_parse_a5(buf: bytes):
    """0xA5 有两种帧:
    1) 新虚拟串口: A5 | control | len(8) | data | CRC16
       control bit7..6 = 0x0 RC / 0x1 RM, bit5..0 = mode
       帧尾 0x5A
    2) 旧 RM 标准: A5 | len(16) | seq | CRC8 | cmd_id | data | CRC16
    """
    if len(buf) < 5:
        return None, 0
    if buf[0] != SOF_A5:
        return None, 1

    # ---- 新虚拟串口帧 ----
    n = buf[2]
    total_new = n + 6
    if total_new <= 256 and len(buf) >= total_new:
        crc16_new = buf[total_new - 3] | (buf[total_new - 2] << 8)
        if (buf[total_new - 1] == 0x5A and
                crc_ccitt(buf[0:total_new - 3]) == crc16_new):
            control = buf[1]
            channel = (control >> 6) & 0x03
            mode = control & 0x3F
            data = buf[3:3 + n]
            d = {"proto": "MINIPC" if channel == 0 else "RM_TARGET",
                 "channel": channel, "mode": mode, "data_len": n,
                 "data_hex": data[:32].hex()}
            if channel == 0 and n >= 24:
                d["cmd_id"] = 0x0001
                d["mode_byte"], d["shoot_enable"], d["shoot_rate"] = data[0], data[1], data[2]
                d["vx"], d["vy"], d["w"] = struct.unpack_from("<3f", data, 4)
                d["gimbal_yaw"], d["gimbal_pitch"] = struct.unpack_from("<2f", data, 16)
            elif channel == 0:
                d["cmd_id"] = 0x0001
            else:
                d["cmd_id"] = 0x0002
            return d, total_new
        # 若长度看起来合理但 CRC 不对，继续按旧帧尝试；不立即丢弃

    # ---- 旧 RM 标准帧 ----
    data_len = buf[1] | (buf[2] << 8)
    total = data_len + 5
    if data_len < 4 or total > 136:
        return None, 1
    if len(buf) < total:
        return None, 0
    if crc8(buf[0:4]) != buf[4]:
        return None, 1
    cmd_id = buf[5] | (buf[6] << 8)
    crc16 = buf[total - 2] | (buf[total - 1] << 8)
    if crc_ccitt(buf[5:total - 2]) != crc16:
        return None, total
    data = buf[7:total - 2]
    d = {"proto": "MINIPC" if cmd_id in (0x0001, 0x0002) else "REFEREE",
         "cmd_id": cmd_id, "seq8": buf[3], "data_len": len(data), "data_hex": data[:32].hex()}
    if cmd_id == 0x0001 and len(data) >= 24:
        d["mode"], d["shoot_enable"], d["shoot_rate"] = data[0], data[1], data[2]
        d["vx"], d["vy"], d["w"] = struct.unpack_from("<3f", data, 4)
        d["gimbal_yaw"], d["gimbal_pitch"] = struct.unpack_from("<2f", data, 16)
    return d, total


class FrameDecoder:
    """一条字节流里可能混有两种帧,自动识别"""

    def __init__(self):
        self.buf = bytearray()
        self.texts = []           # 收到的人类可读文本行(如 [LOG] boot=...)
        self.stats = {"DEBUG": 0, "A5": 0, "crc_err": 0, "reject": 0}

    def feed(self, chunk: bytes):
        out = []
        self.buf += chunk

        # ★ 抽掉设备每秒发一次的 [LOG] 文本行(只出现在帧之间;不当成二进制垃圾 ✓)
        while True:
            i = self.buf.find(b"[LOG]")
            if i < 0:
                break
            j = self.buf.find(b"\n", i)
            if j < 0:
                break                       # 还没收全,等下一批
            self.texts.append(bytes(self.buf[i:j]).decode("ascii", "replace").strip())
            del self.buf[i:j + 1]

        # ★ [LOG] 行只收到一半时先别做二进制解析:
        #   Reader 可能是 1 字节 1 字节喂进来的,半行会被后面的二进制
        #   解析当“噪声”清掉,于是 PID set/now 这种行永远显示不出来。
        i_partial = self.buf.find(b"[LOG]")
        if i_partial >= 0 and self.buf.find(b"\n", i_partial) < 0:
            return out

        while self.buf:
            i5a = self.buf.find(bytes([SOF_DEBUG]))
            ia5 = self.buf.find(bytes([SOF_A5]))
            cands = [x for x in (i5a, ia5) if x >= 0]
            if not cands:
                self.stats["reject"] += len(self.buf)
                self.buf.clear()
                break
            pos = min(cands)
            if pos > 0:
                self.stats["reject"] += pos
                del self.buf[:pos]
            if self.buf[0] == SOF_DEBUG:
                if len(self.buf) < 4:
                    break
                # ★ 帧里的 [2..3] 就是长度字段 ⇒ 以它为准(186/210 都认 ✓)
                flen = self.buf[2] | (self.buf[3] << 8)
                if flen < 9 or flen > 256:
                    self.stats["reject"] += 1
                    del self.buf[:1]
                    continue
                if len(self.buf) < flen:
                    break
                frame = bytes(self.buf[:flen])
                if crc_ccitt(frame[:-2]) == (frame[-2] | (frame[-1] << 8)):
                    try:
                        out.append((parse_debug(frame), frame))
                        self.stats["DEBUG"] += 1
                    except struct.error:
                        self.stats["reject"] += 1
                    del self.buf[:flen]
                else:
                    self.stats["crc_err"] += 1
                    del self.buf[:1]
            else:
                d, used = try_parse_a5(bytes(self.buf))
                if used == 0:
                    break
                if d is None:
                    self.stats["crc_err"] += 1
                else:
                    out.append((d, bytes(self.buf[:used])))
                    self.stats["A5"] += 1
                del self.buf[:used]
        return out


# ============================================================================
# 串口
# ============================================================================

try:
    import serial
    from serial.tools import list_ports
    HAS_SERIAL = True
except Exception:
    HAS_SERIAL = False


def _linux_glob_ports():
    """Ubuntu 下 comports() 偶尔受 udev/权限影响,直接按设备节点兜底。"""
    import glob
    found = []
    for p in glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"):
        found.append(p)
    for p in glob.glob("/dev/serial/by-id/*"):
        try:
            found.append(os.path.realpath(p))
        except OSError:
            pass
    return sorted(set(found))


def list_serial_ports(timeout=1.0):
    """带超时的端口枚举。

    Windows: PyInstaller 单文件包里 comports() 偶尔会卡在 SetupAPI 上;
    Linux  : comports() 受 udev/权限影响时用 /dev/ttyUSB* 兜底。
    """
    if not HAS_SERIAL:
        return _linux_glob_ports() if IS_LINUX else []

    result = queue.Queue()

    def _worker():
        try:
            result.put(sorted(p.device for p in list_ports.comports()))
        except Exception:
            result.put([])

    threading.Thread(target=_worker, daemon=True).start()
    try:
        ports = result.get(timeout=timeout)
    except queue.Empty:
        ports = []
    if not ports and IS_LINUX:
        ports = _linux_glob_ports()
    return ports


class Reader(threading.Thread):
    def __init__(self, idx, port, baud, sink: queue.Queue):
        super().__init__(daemon=True)
        self.idx, self.port, self.baud, self.sink = idx, port, baud, sink
        self.stop_flag = threading.Event()
        self.error = None
        self.ser = None                 # 打开后由 run() 填入,供发送命令使用
        self._write_lock = threading.Lock()

    def run(self):
        try:
            with serial.Serial(self.port, self.baud, timeout=0.05) as ser:
                self.ser = ser
                ser.reset_input_buffer()
                while not self.stop_flag.is_set():
                    data = ser.read(ser.in_waiting or 1)
                    if data:
                        self.sink.put(("data", self.idx, data))
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.ser = None

    def write(self, data: bytes):
        """从 UI 线程发命令;读线程同时 read 是允许的,这里只锁 write。"""
        with self._write_lock:
            if self.ser is None:
                raise RuntimeError("端口未打开")
            self.ser.write(data)

    def stop(self):
        self.stop_flag.set()


def probe_port(port, baud, seconds=1.2):
    """试一个 (端口, 波特率):收到有效帧就返回 'DEBUG' / 'A5' / None"""
    if not HAS_SERIAL:
        return None
    dec = FrameDecoder()
    try:
        with serial.Serial(port, baud, timeout=0.05) as ser:
            ser.reset_input_buffer()
            t0 = time.time()
            while time.time() - t0 < seconds:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    dec.feed(data)
                if dec.stats["DEBUG"] >= 3:
                    return "DEBUG"
                if dec.stats["A5"] >= 2:
                    return "A5"
    except Exception:
        return None
    return None


# ============================================================================
# 记录 + 导出
# ============================================================================

def _export_dir():
    """导出目录:Ubuntu 用 ~/调试助手导出,Windows 优先程序旁的 导出/。"""
    if IS_LINUX:
        path = os.path.join(os.path.expanduser("~"), "调试助手导出")
        os.makedirs(path, exist_ok=True)
        return path
    if getattr(sys, "frozen", False):
        base = os.path.dirname(sys.executable)
    else:
        base = os.path.dirname(os.path.abspath(__file__))
    cand = os.path.join(os.path.dirname(base), "导出")
    if os.path.isdir(cand):
        return cand
    os.makedirs(os.path.join(base, "导出"), exist_ok=True)
    return os.path.join(base, "导出")


class Recorder:
    def __init__(self):
        self.rows = []
        self.raw_rows = []          # 原始帧(时间/端口/长度/HEX),给"导出原始数据"用
        self.started = datetime.now()
        self.ports = set()
        self.events = []
        self.last_level = None
        self.last_code = None
        self.max_rows = 300000
        self.max_raw_rows = 200000

    def add(self, port, baud, d: dict, frame: bytes):
        self.ports.add(f"{port}@{baud}")
        row = dict(d)
        row["time"] = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        row["port"] = port
        row["baud"] = baud
        row["raw_len"] = len(frame)
        self.rows.append(row)
        self.raw_rows.append({
            "time": row["time"],
            "port": port,
            "baud": baud,
            "proto": d.get("proto", "?"),
            "len": len(frame),
            "hex": frame.hex(" ").upper(),
        })
        if len(self.rows) > self.max_rows:
            del self.rows[:10000]
        if len(self.raw_rows) > self.max_raw_rows:
            del self.raw_rows[:10000]
        if d.get("proto") == "DEBUG":
            lv, code = d.get("err_level"), d.get("err_code")
            if lv != self.last_level or code != self.last_code:
                self.events.append(f"{row['time']} {port} 状态变化 → {LEVEL_NAME.get(lv, lv)}  code=0x{code:04X}")
                self.last_level, self.last_code = lv, code
            if d.get("fault_code"):
                self.events.append(f"{row['time']} {port} ★故障= {FAULT_NAME.get(d['fault_code'])}  "
                                   f"CFSR=0x{d['fault_cfsr']:08X} PC=0x{d['fault_pc']:08X}")

    CSV_FIELDS = (["time", "port", "baud", "proto", "seq", "seq8", "cmd_id", "data_len",
                   "err_level", "err_code", "fault_code", "fault_cfsr", "fault_pc",
                   "pos", "pos_target", "pos_error",
                   "pos_raw", "pos_target_raw", "pos_error_raw", "pos_fb",
                   "imu_roll", "imu_pitch", "imu_yaw",
                   "imu_gx", "imu_gy", "imu_gz",
                   "rc_online", "rc_sw0", "rc_sw1", "rc_mouse_l", "rc_mouse_r",
                   "rc_key", "rc_lost", "rc_rx",
                   "rc_ch0", "rc_ch1", "rc_ch2", "rc_ch3", "rc_ch4",
                   "rc_mouse_x", "rc_mouse_y", "rc_mouse_z",
                   "ys_online", "ys_status", "ys_target", "ys_m_error",
                   "ys_pos", "ys_w", "ys_t", "ys_temp", "ys_voltage",
                   "ys_crc", "ys_warn",
                   "roll", "pitch", "yaw", "gx", "gy", "gz", "ax", "ay", "az",
                   "hwm_motor", "hwm_robot", "hwm_error", "hwm_cmd",
                   "hwm_imu", "hwm_chassis", "hwm_gimbal", "hwm_highRise",
                   "log_boot", "log_cur_start", "log_prev_start", "log_prev_stop", "log_wrap",
                   "temp_c",
                   "vx", "vy", "w", "gimbal_yaw", "gimbal_pitch",
                   "mode", "shoot_enable", "shoot_rate", "data_hex"]
                  + [f"m{i}_{k}" for i in range(DEBUG_MAX_MOTORS) for k in ("pos", "w", "t", "online")])

    def export_csv(self, path):
        with open(path, "w", newline="", encoding=CONFIG["csv_encoding"]) as fp:
            w = csv.DictWriter(fp, fieldnames=self.CSV_FIELDS, extrasaction="ignore")
            w.writeheader()
            for r in self.rows:
                flat = dict(r)
                for i, m in enumerate(r.get("motors", [])):
                    flat[f"m{i}_pos"] = f"{m['pos']:.4f}"
                    flat[f"m{i}_w"] = f"{m['w']:.3f}"
                    flat[f"m{i}_t"] = f"{m['t']:.4f}"
                    flat[f"m{i}_online"] = int(m["online"])
                for i, v in enumerate(r.get("rc_ch", [])):
                    flat[f"rc_ch{i}"] = v
                for i, k in enumerate(("rc_mouse_x", "rc_mouse_y", "rc_mouse_z")):
                    vals = r.get("rc_mouse", [])
                    if i < len(vals):
                        flat[k] = vals[i]
                w.writerow(flat)

    def export_raw_csv(self, path):
        fields = ("time", "port", "baud", "proto", "len", "hex")
        with open(path, "w", newline="", encoding=CONFIG["csv_encoding"]) as fp:
            w = csv.DictWriter(fp, fieldnames=fields, extrasaction="ignore")
            w.writeheader()
            w.writerows(self.raw_rows)

    def export_raw_hex(self, path):
        """纯 HEX 行:时间 端口 协议 长度 HEX,便于 grep / 给厂商。"""
        with open(path, "w", encoding="utf-8") as fp:
            for r in self.raw_rows:
                fp.write("%s  %-10s %-8s len=%-3d  %s\n" %
                         (r["time"], r["port"], r["proto"], r["len"], r["hex"]))

    def export_md(self, path, stats: dict):
        now = datetime.now()
        lines = [
            "# MCU 数据记录 / 故障排查文档", "",
            f"- **生成时间**:{now.strftime('%Y-%m-%d %H:%M:%S')}",
            f"- **采集开始**:{self.started.strftime('%Y-%m-%d %H:%M:%S')}",
            f"- **采集时长**:{(now - self.started).total_seconds():.1f} s",
            f"- **平台**:{CONFIG['platform']} / Python {sys.version.split()[0]}",
            f"- **串口**:{', '.join(sorted(self.ports)) if self.ports else '(无)'}",
            f"- **总帧数**:{len(self.rows)}", "",
            "## 1. 协议统计", "", "| 项 | 值 |", "|---|---|",
            f"| 调试遥测帧 (0x5A) | {stats.get('DEBUG', 0)} |",
            f"| 上位机/裁判帧 (0xA5) | {stats.get('A5', 0)} |",
            f"| CRC 错误 | {stats.get('crc_err', 0)} |",
            f"| 丢弃字节(帧间噪声) | {stats.get('reject', 0)} |", "",
            "## 2. 关键事件(状态变化 / 故障)", "",
        ]
        lines += [f"- {e}" for e in self.events[-200:]] if self.events else ["- (无状态变化)"]

        dbg = [r for r in self.rows if r.get("proto") == "DEBUG"]
        if dbg:
            last = dbg[-1]
            lines += ["", "## 3. 最后一帧状态快照", "",
                      f"- 时间: {last['time']}  串口: {last['port']}",
                      f"- 错误级别: **{LEVEL_NAME.get(last['err_level'], last['err_level'])}**  "
                      f"错误码: 0x{last['err_code']:04X}",
                      f"- 故障码: {FAULT_NAME.get(last['fault_code'], last['fault_code'])} "
                      f"(CFSR=0x{last['fault_cfsr']:08X}, PC=0x{last['fault_pc']:08X})",
                      f"- IMU: roll={last['roll']:+.3f} pitch={last['pitch']:+.3f} yaw={last['yaw']:+.3f} rad",
                      (f"- 陀螺仪: gx={last['gx']:+.4f} gy={last['gy']:+.4f} gz={last['gz']:+.4f} rad/s" if last.get("has_gyro") else "- 陀螺仪: (旧固件无)"),
                      (f"- 加速度: ax={last['ax']:+.3f} ay={last['ay']:+.3f} az={last['az']:+.3f} m/s^2" if last.get("has_gyro") else "- 加速度: (旧固件无)"),
                      (f"- 栈水位(字):电机 {last['hwm_motor']} / robot {last['hwm_robot']} / error {last['hwm_error']} / cmd {last['hwm_cmd']} / "
                       f"imu {last['hwm_imu']} / chassis {last['hwm_chassis']} / gimbal {last['hwm_gimbal']} / highRise {last['hwm_highRise']}"
                       if last.get("has_hwm") else "- 栈水位: (旧固件无)"),
                      f"- 底盘: vx={last['vx']:+.3f} vy={last['vy']:+.3f} w={last['w']:+.3f}",
                      f"- 云台: yaw={last['gimbal_yaw']:+.3f} pitch={last['gimbal_pitch']:+.3f} rad", "",
                      "### 电机", "", "| # | 状态 | 位置 rad | 转速 rad/s | 力矩 N·m |", "|---|---|---|---|---|"]
            for m in last["motors"]:
                st = ",".join(n for bit, n in MOTOR_FLAG if m["flags"] & bit) or "-"
                lines.append(f"| {m['id']} | {st} | {m['pos']:+.4f} | {m['w']:+.3f} | {m['t']:+.4f} |")

        lines += ["", "## 4. 如何使用", "",
                  "1. 先看第 2 节:状态变化的时间点 + 第 3 节的快照",
                  "2. 若 `故障码 != 正常`,把快照里的 `PC=0x...` 拿去反查:",
                  "   `arm-none-eabi-addr2line -e LGRobotSystem.elf -f -C 0x<pc>`",
                  "3. 时间列来自上位机时钟(不是 MCU 时钟),用于对齐多路数据",
                  "4. 需要逐帧数据时看同名的 `.csv`", "", "---",
                  f"*由 调试助手 ({CONFIG['platform']}) 自动生成*"]
        with open(path, "w", encoding="utf-8") as fp:
            fp.write("\n".join(lines))


# ============================================================================
# GUI
# ============================================================================

# 底盘 / 云台调试命令(0x5C 调试协议里自定义的两个 cmd)。
# ★ 固件里 cmd 号或字段布局不一样,只改这里(或者在窗口底部直接发原始 HEX)★
CHASSIS_CMD = 0x07      # payload: <h vx(mm/s)> <h vy(mm/s)> <h wz(0.01 rad/s)>
GIMBAL_CMD = 0x08       # payload: <i yaw(0.001°)> <i pitch(0.001°)>


class MotorDebugWindow(tk.Toplevel):
    """单电机调试窗口:速度/角度给定 + PID 写入 + 两条实时曲线"""

    def __init__(self, app):
        super().__init__(app)
        self.app = app
        self.title("调试助手 — 电机调试")
        self.geometry("820x680")
        self.cmd_seq = 0
        self.t_speed = collections.deque(maxlen=300)
        self.a_speed = collections.deque(maxlen=300)
        self.t_pos = collections.deque(maxlen=300)
        self.a_pos = collections.deque(maxlen=300)
        self.v_tgt_speed = tk.StringVar(value="0")
        self.v_tgt_pos = tk.StringVar(value="0")
        self.v_motor = tk.StringVar(value="0x202")
        # speed Kp/Ki/Kd + position Kp/Ki/Kd
        self.pid_vars = [tk.StringVar(value=v) for v in ("0.4", "0.02", "0", "10", "0", "0.5")]
        self.pid_cache = {
            0x202: (0.4, 0.02, 0.0, 15.0, 0.0, 0.8),
            0x204: (0.6, 0.02, 0.0, 15.0, 0.0, 0.8),
            0x206: (0.1, 0.01, 0.0, 0.0, 0.0, 0.0),
            0x205: (0.05, 0.001, 0.0, 0.0, 0.0, 0.0),
        }
        self.after(250, self._query_pid)

        row = ttk.Frame(self); row.pack(fill="x", padx=8, pady=6)
        ttk.Label(row, text="选择电机").pack(side="left")
        self.cb_motor = ttk.Combobox(row, textvariable=self.v_motor,
                                     values=("0x201", "0x202", "0x203", "0x204", "0x206", "0x205"),
                                     state="readonly", width=8)
        self.cb_motor.pack(side="left", padx=4)
        self.cb_motor.bind("<<ComboboxSelected>>", self._on_motor_changed)
        ttk.Label(row, text="目标速度(rpm)").pack(side="left", padx=(12, 2))
        ttk.Entry(row, width=9, textvariable=self.v_tgt_speed).pack(side="left")
        ttk.Button(row, text="发送速度", command=self._send_speed).pack(side="left", padx=4)
        ttk.Label(row, text="目标角度(位置环)").pack(side="left", padx=(12, 2))
        ttk.Entry(row, width=9, textvariable=self.v_tgt_pos).pack(side="left")
        ttk.Button(row, text="发送角度", command=self._send_pos).pack(side="left", padx=4)

        status = ttk.LabelFrame(self, text="在线电机")
        status.pack(fill="x", padx=8, pady=4)
        self.lbl_online = ttk.Label(status, text="在线: 0x202 --")
        self.lbl_online.pack(anchor="w", padx=6, pady=2)

        self.pid_frame = ttk.LabelFrame(self, text="0x202 PID(先调速度环,位置环关闭)")
        pid = self.pid_frame
        pid.pack(fill="x", padx=8, pady=4)
        names = ["速 Kp", "速 Ki", "速 Kd", "位 Kp", "位 Ki", "位 Kd"]
        for i, n in enumerate(names):
            ttk.Label(pid, text=n).grid(row=0, column=i, padx=3, pady=3)
            ttk.Entry(pid, width=8, textvariable=self.pid_vars[i]).grid(row=1, column=i, padx=3)
        ttk.Button(pid, text="写入 PID", command=self._send_pid).grid(row=1, column=6, padx=8)
        ttk.Button(pid, text="使能").grid(row=1, column=7, padx=2)
        ttk.Button(pid, text="停止", command=self._send_stop).grid(row=1, column=8, padx=2)

        # 两个摩擦轮独立/同时调速(0x5C 0x06)
        fric = ttk.Frame(self)   # 摩擦轮已从本台架移除,不显示
        self.v_fr_l = tk.StringVar(value="0")
        self.v_fr_r = tk.StringVar(value="0")
        self.v_fr_shoot = tk.StringVar(value="100")
        ttk.Label(fric, text="左 rpm").grid(row=0, column=0, padx=3)
        ttk.Entry(fric, width=8, textvariable=self.v_fr_l).grid(row=0, column=1, padx=3)
        ttk.Button(fric, text="发送左", command=lambda: self._send_friction(0)).grid(row=0, column=2, padx=3)
        ttk.Label(fric, text="右 rpm").grid(row=0, column=3, padx=3)
        ttk.Entry(fric, width=8, textvariable=self.v_fr_r).grid(row=0, column=4, padx=3)
        ttk.Button(fric, text="发送右", command=lambda: self._send_friction(1)).grid(row=0, column=5, padx=3)
        ttk.Button(fric, text="同时启动", command=self._send_friction_both).grid(row=0, column=6, padx=8)
        ttk.Button(fric, text="同时停止", command=self._send_friction_stop).grid(row=0, column=7, padx=3)

        ttk.Label(fric, text="发射 rpm").grid(row=1, column=0, padx=3, pady=(6, 2))
        ttk.Entry(fric, width=8, textvariable=self.v_fr_shoot).grid(row=1, column=1, padx=3, pady=(6, 2))
        ttk.Button(fric, text="发射（左反 / 右正）",
                   command=self._send_friction_shoot).grid(row=1, column=2, columnspan=4,
                                                          sticky="w", padx=3, pady=(6, 2))
        ttk.Label(fric, text="左轮反转、右轮正转出弹",
                  style="Muted.TLabel").grid(row=1, column=6, columnspan=3,
                                             sticky="w", padx=8, pady=(6, 2))

        self.cv_speed = tk.Canvas(self, bg="#101418", height=230)
        self.cv_speed.pack(fill="both", expand=True, padx=8, pady=(6, 3))
        self.cv_pos = tk.Canvas(self, bg="#101418", height=230)
        self.cv_pos.pack(fill="both", expand=True, padx=8, pady=(3, 8))
        # 曲线画布允许铺背景图(动图会自己动);两块按窗口对齐 ⇒ 上下一张图,不重复
        self.app.bg.attach(self.cv_speed, extra_dim=0.25, align_to=self)
        self.app.bg.attach(self.cv_pos, extra_dim=0.25, align_to=self)

        # 目标值输入即生效(防抖),不必再点“发送速度/发送角度”
        self._target_after = None
        self.v_tgt_speed.trace_add("write", self._on_target_changed)
        self.v_tgt_pos.trace_add("write", self._on_angle_changed)

    def _on_target_changed(self, *_):
        if self._target_after is not None:
            try:
                self.after_cancel(self._target_after)
            except Exception:
                pass
        self._target_after = self.after(280, self._apply_target_live)

    def _on_angle_changed(self, *_):
        if self._target_after is not None:
            try:
                self.after_cancel(self._target_after)
            except Exception:
                pass
        self._target_after = self.after(280, self._apply_angle_live)

    def _apply_angle_live(self):
        self._target_after = None
        if self._sel() not in (0x202, 0x204):
            return
        try:
            float(self.v_tgt_pos.get() or 0)
            self._send_pos()
        except ValueError:
            pass

    def _apply_target_live(self):
        self._target_after = None
        try:
            int(float(self.v_tgt_speed.get() or 0))
            self._send_speed()
        except ValueError:
            pass

    # ---- 数据入口:主窗口每次刷新调用 ----
    def _sel(self):
        try:
            return int(str(self.v_motor.get()), 16)
        except Exception:
            return 0x202

    @staticmethod
    def _online_text(flags):
        return "在线" if (flags & 0x02) else "离线"

    def push(self, d):
        try:
            fields = getattr(self.app, "vofa_last", None) or {}
            def _on(mid):
                return float(fields.get("m%03X.online" % mid, 0.0)) > 0.5
            self.lbl_online.config(text="在线: " + "  ".join(
                "0x%03X %s" % (mid, "YES" if _on(mid) else "NO")
                for mid in (0x202, 0x201, 0x204, 0x203, 0x206, 0x205)))
            sel = self._sel()
            w = float(fields.get("m%03X.w" % sel, 0.0))
            pos = float(fields.get("m%03X.pos" % sel, 0.0))
            self.a_speed.append(w * 60.0 / (2.0 * math.pi))
            self.a_pos.append(pos)
            self.t_speed.append(float(self.v_tgt_speed.get() or 0))
            self.t_pos.append(float(fields.get("m%03X.pos_target" % sel, 0.0)))
            self._draw(self.cv_speed, self.t_speed, self.a_speed, f"速度 rpm (0x{sel:03X})")
            self._draw(self.cv_pos, self.t_pos, self.a_pos, f"角度 rad (0x{sel:03X})")
        except Exception:
            pass

    def _draw(self, cv, tgt, act, title):
        cv.delete("all")
        self.app.bg.paint(cv)                      # 先把背景图铺回最底层
        w = max(cv.winfo_width(), 200); h = max(cv.winfo_height(), 120)
        cv.create_text(8, 10, text=title, fill="#9ab", anchor="nw")
        vals = list(tgt) + list(act)
        if len(vals) < 2:
            cv.create_text(w / 2, h / 2, text="等待数据…", fill="#667")
            return
        lo, hi = min(vals), max(vals)
        if hi - lo < 1e-6:
            hi = lo + 1.0
        pad = 30
        def xy(i, v, n):
            x = pad + (w - 2 * pad) * (i / max(n - 1, 1))
            y = h - pad - (h - 2 * pad) * ((v - lo) / (hi - lo))
            return x, y
        for series, color in ((tgt, "#ffd24a"), (act, "#4ad0ff")):
            pts = []
            n = len(series)
            for i, v in enumerate(series):
                pts.extend(xy(i, v, n))
            if len(pts) >= 4:
                cv.create_line(*pts, fill=color, width=2)
        cv.create_text(w - 8, 10, text="max %.2f  min %.2f" % (hi, lo), fill="#789", anchor="ne")

    # ---- 发送(协议 0x5C:帧头 0x5C + cmd + seq + 参数) ----
    def _send_cmd(self, cmd, payload=b""):
        self.cmd_seq = (self.cmd_seq + 1) & 0xFFFF
        fr = bytes([0x5C, cmd, (self.cmd_seq >> 8) & 0xFF, self.cmd_seq & 0xFF]) + payload
        w = self.app.readers.get(0)
        if w is None:
            self.app._echo("[电机调试] 端口 1 未打开")
            return
        try:
            w.write(fr)
            self.app._echo("[电机调试] 发送 " + fr.hex(" "))
        except Exception as e:
            self.app._echo("[电机调试] 发送失败: %s" % e)

    def _apply_pid(self, motor_id):
        if hasattr(self, "pid_frame"):
            self.pid_frame.configure(
                text=f"0x{motor_id:03X} PID(速度环 Kp Ki Kd / 位置环 Kp Ki Kd)")
        vals = self.pid_cache.get(motor_id, (0.0,) * 6)
        if len(vals) < 6:
            vals = tuple(vals) + (0.0,) * (6 - len(vals))
        for i in range(6):
            self.pid_vars[i].set(f"{vals[i]:g}")

    def _query_pid(self):
        sel = self._sel()
        self._apply_pid(sel)
        if sel == 0x202:
            self._send_cmd(0x04, b"")

    def _on_motor_changed(self, _event=None):
        self._query_pid()

    def feed_pid_text(self, text):
        """解析固件回读的 [LOG] PID ... 文本,填充当前电机 PID。"""
        import re
        m = re.search(r"PID (?:set|now) speed\(Kp=([-\d.]+) Ki=([-\d.]+) Kd=([-\d.]+)\) pos\(Kp=([-\d.]+) Ki=([-\d.]+) Kd=([-\d.]+)\)", text)
        if m:
            self.pid_cache[0x202] = tuple(float(x) for x in m.groups())
        self._apply_pid(self._sel())

    def _send_speed(self):
        try:
            v = int(float(self.v_tgt_speed.get() or 0))
        except ValueError:
            return
        if self._sel() == 0x201:
            self._send_cmd(0x09, struct.pack("<h", v))
        elif self._sel() == 0x203:
            self._send_cmd(0x0D, struct.pack("<h", v))
        elif self._sel() == 0x202:
            self._send_cmd(0x01, struct.pack("<h", v))
        elif self._sel() == 0x204:
            self._send_cmd(0x0C, struct.pack("<h", v))
        elif self._sel() == 0x206:
            self._send_cmd(0x0F, struct.pack("<h", v))
        elif self._sel() == 0x205:
            self._send_cmd(0x10, struct.pack("<h", v))

    def _send_pos(self):
        sel = self._sel()
        if sel not in (0x202, 0x204, 0x205):
            self.app._echo("[电机调试] 只有 202/204/205 有角度命令")
            return
        v = int(float(self.v_tgt_pos.get() or 0) * 1000.0)
        cmd = 0x02 if sel == 0x202 else (0x0B if sel == 0x204 else 0x12)
        self._send_cmd(cmd, struct.pack("<i", v))

    def _send_friction(self, idx):
        try:
            rpm = int(float((self.v_fr_l if idx == 0 else self.v_fr_r).get() or 0))
        except ValueError:
            self.app._echo("[摩擦轮] rpm 只能填数字")
            return
        self._send_cmd(0x06, bytes([idx]) + struct.pack("<h", rpm))

    def _send_friction_both(self):
        self._send_friction(0)
        self._send_friction(1)

    def _send_friction_shoot(self):
        """发射:左轮正转、右轮反转;只填一个速度。"""
        try:
            rpm = int(float(self.v_fr_shoot.get() or 0))
        except ValueError:
            self.app._echo("[摩擦轮] 发射转速只能填数字")
            return
        self.v_fr_l.set(str(-rpm))
        self.v_fr_r.set(str(rpm))
        self._send_friction_both()
        self.app._echo(f"[摩擦轮] 发射:左={-rpm} 右={rpm} rpm")

    def _send_friction_stop(self):
        self.v_fr_l.set("0")
        self.v_fr_r.set("0")
        self._send_friction_both()

    def _send_pid(self):
        try:
            vals = [float(x.get() or 0) for x in self.pid_vars]
        except ValueError:
            self.app._echo("[电机调试] PID 只能填数字")
            return
        sel = self._sel()
        if sel in (0x201, 0x203):
            self._send_cmd(0x0A, struct.pack("<3f", vals[0], vals[1], vals[2]))
            self.pid_cache[0x201] = tuple(vals[:3]) + (0.0, 0.0, 0.0)
            self.pid_cache[0x203] = tuple(vals[:3]) + (0.0, 0.0, 0.0)
        elif sel == 0x204:
            self._send_cmd(0x0E, struct.pack("<6f", *vals))
            self.pid_cache[0x204] = tuple(vals)
        elif sel in (0x206, 0x205):
            idx = 0 if sel == 0x206 else 1
            self._send_cmd(0x11, bytes([idx]) + struct.pack("<3f", vals[0], vals[1], vals[2]))
            self.pid_cache[sel] = tuple(vals[:3]) + (0.0, 0.0, 0.0)
        else:
            self._send_cmd(0x03, struct.pack("<6f", *vals))
            self.pid_cache[0x202] = tuple(vals)

    def _send_stop(self):
        if self._sel() == 0x206:
            self._send_cmd(0x0F, struct.pack("<h", 0))
        elif self._sel() == 0x205:
            self._send_cmd(0x10, struct.pack("<h", 0))
        else:
            self._send_cmd(0x05, b"\x00")


class SteerDebugWindow(tk.Toplevel):
    """Single steering wheel debug: dial + angle target + position PID."""

    def __init__(self, app):
        super().__init__(app)
        self.app = app
        self.title("调试助手 — 舵轮调试 0x202")
        self.geometry("620x760")
        self.configure(bg="#0d131a")
        self.cmd_seq = 0
        self.pos = 0.0
        self.pos_target = 0.0
        self.online = False
        self.pe2 = 0
        self.pe3 = 0
        self.speed_target = 0.0
        self.pid_vars = [tk.StringVar(value=v) for v in ("0.4", "0.02", "0", "10", "0", "0.5")]
        self.v_angle = tk.StringVar(value="0")
        self.v_speed = tk.StringVar(value="0")

        top = ttk.Frame(self)
        top.pack(fill="x", padx=10, pady=8)
        ttk.Label(top, text="目标角度(°)").pack(side="left")
        ttk.Entry(top, width=10, textvariable=self.v_angle).pack(side="left", padx=6)
        ttk.Button(top, text="发送角度", command=self._send_angle).pack(side="left", padx=4)
        ttk.Label(top, text="目标速度(rpm)").pack(side="left", padx=(12, 2))
        ttk.Entry(top, width=10, textvariable=self.v_speed).pack(side="left", padx=6)
        ttk.Button(top, text="发送速度", command=self._send_speed).pack(side="left", padx=4)
        ttk.Button(top, text="停止", command=self._send_stop).pack(side="left", padx=4)

        self.cv = tk.Canvas(self, width=560, height=430, bg="#0d131a", highlightthickness=0)
        self.cv.pack(fill="both", expand=True, padx=8, pady=6)
        self.app.bg.attach(self.cv, extra_dim=0.25, align_to=self)  # 仪表盘也能铺背景图(按窗口对齐)

        pid = ttk.LabelFrame(self, text="0x202 PID(速度 Kp Ki Kd / 位置 Kp Ki Kd)")
        pid.pack(fill="x", padx=10, pady=6)
        names = ["速 Kp", "速 Ki", "速 Kd", "位 Kp", "位 Ki", "位 Kd"]
        for i, n in enumerate(names):
            ttk.Label(pid, text=n).grid(row=0, column=i, padx=4, pady=2)
            ttk.Entry(pid, width=8, textvariable=self.pid_vars[i]).grid(row=1, column=i, padx=4)
        ttk.Button(pid, text="写入 PID", command=self._send_pid).grid(row=1, column=6, padx=6)
        ttk.Button(pid, text="读取 PID", command=lambda: self._send_cmd(0x04)).grid(row=1, column=7, padx=4)

        self.info = tk.Label(self, text="等待 0x202 数据…", bg="#0d131a", fg="#9fb3c8",
                             justify="left", font=("Consolas", 11))
        self.info.pack(anchor="w", padx=16, pady=6)

    def _send_cmd(self, cmd, payload=b""):
        self.cmd_seq = (self.cmd_seq + 1) & 0xFFFF
        fr = bytes([0x5C, cmd, (self.cmd_seq >> 8) & 0xFF, self.cmd_seq & 0xFF]) + payload
        w = self.app.readers.get(0)
        if w is None:
            self.app._echo("[舵轮] 端口 1 未打开")
            return
        try:
            w.write(fr)
            self.app._echo("[舵轮] 发送 " + fr.hex(" "))
        except Exception as e:
            self.app._echo("[舵轮] 发送失败: %s" % e)

    def _send_angle(self):
        try:
            deg = float(self.v_angle.get() or 0)
        except ValueError:
            return
        self._send_cmd(0x02, struct.pack("<i", int(deg * 1000.0)))

    def _send_speed(self):
        try:
            rpm = int(float(self.v_speed.get() or 0))
        except ValueError:
            return
        self._send_cmd(0x01, struct.pack("<h", rpm))

    def _send_stop(self):
        self._send_cmd(0x05, b"")
        self.v_angle.set("0")

    def _send_pid(self):
        try:
            vals = [float(x.get() or 0) for x in self.pid_vars]
        except ValueError:
            return
        self._send_cmd(0x03, struct.pack("<6f", *vals))

    def feed_pid_text(self, text):
        import re
        m = re.search(r"PID (?:set|now) speed\(Kp=([-\d.]+) Ki=([-\d.]+) Kd=([-\d.]+)\) pos\(Kp=([-\d.]+) Ki=([-\d.]+) Kd=([-\d.]+)\)", text)
        if m:
            for i, v in enumerate(m.groups()):
                self.pid_vars[i].set(str(float(v)))

    def feed(self, data):
        try:
            self.pos = float(data.get("m202.pos", self.pos))
            self.pos_target = float(data.get("m202.pos_target", self.pos_target))
            self.online = bool(float(data.get("m202.online", 0.0)))
            self.pe2 = int(float(data.get("photo.pe2", self.pe2)))
            self.pe3 = int(float(data.get("photo.pe3", self.pe3)))
            self.speed_target = float(data.get("m202.target", 0.0))
        except Exception:
            return
        self._draw()

    def _draw(self):
        cv = self.cv
        cv.delete("all")
        self.app.bg.paint(cv)
        w = max(cv.winfo_width(), 300)
        h = max(cv.winfo_height(), 300)
        cx, cy, R = w / 2, h / 2, min(w, h) * 0.38

        cv.create_oval(cx - R, cy - R, cx + R, cy + R, outline="#34495e", width=3)
        for deg in range(0, 360, 15):
            a = math.radians(deg)
            r1 = R - (22 if deg % 45 == 0 else 12)
            x1, y1 = cx + r1 * math.sin(a), cy - r1 * math.cos(a)
            x2, y2 = cx + R * math.sin(a), cy - R * math.cos(a)
            cv.create_line(x1, y1, x2, y2, fill="#95a5a6" if deg % 45 == 0 else "#3d566e")
            if deg % 45 == 0:
                cv.create_text(cx + (R - 34) * math.sin(a), cy - (R - 34) * math.cos(a),
                               text=str(deg), fill="#9fb3c8", font=("Consolas", 9))

        def needle(angle, color, width, dash=None):
            x = cx + R * 0.86 * math.sin(angle)
            y = cy - R * 0.86 * math.cos(angle)
            cv.create_line(cx, cy, x, y, fill=color, width=width, dash=dash)
            cv.create_oval(x - 4, y - 4, x + 4, y + 4, fill=color, outline="")

        needle(self.pos_target, "#ffd24a", 2, (6, 4))
        needle(self.pos, "#4ad0ff", 4)
        cv.create_oval(cx - 7, cy - 7, cx + 7, cy + 7, fill="#ecf0f1", outline="")

        deg = math.degrees(self.pos) % 360.0
        tgt = math.degrees(self.pos_target) % 360.0
        err = math.degrees(self.pos_target - self.pos)
        self.info.config(text=(f"{'在线' if self.online else '离线'}    "
                               f"实际 {deg:7.2f}°   目标 {tgt:7.2f}°   误差 {err:+7.2f}°\n"
                               f"光电 PE2={self.pe2}   PE3={self.pe3}   速度目标={self.speed_target:.1f} rpm"))


def _hex_bytes(text):
    """'5C 05 00 01' / '5c,05,0001' → bytes;格式不对抛 ValueError。"""
    t = (text or "").replace(",", " ").replace("0x", " ").replace("0X", " ")
    out = bytearray()
    for part in t.split():
        if len(part) % 2:
            part = "0" + part
        out += bytes.fromhex(part)
    return bytes(out)


CHASSIS_TYPES = ("麦轮", "全向轮", "舵轮")      # 底盘类型(chassis 运动学不同)


class ChassisView(tk.Canvas):
    """底盘俯视示意图:车身+四轮,粗箭头=下发速度,绿弧=转动方向。

    按底盘类型自动解算出每个轮子的转速(舵轮还算转向角),不用自己手算:
      麦轮/全向轮: v_fl = vx - vy - (a+b)w …(X 型 45° 辊子)
      舵轮:        每轮 v_ix = vx - w*y_i, v_iy = vy + w*x_i, 转速=模长, 转向=atan2
    """

    MAX_MM = 1000.0        # 箭头满量程(mm/s)
    MAX_W = 400.0          # 转动弧满量程(wz,单位 0.01 rad/s)

    def __init__(self, master, height=300):
        super().__init__(master, bg="#0d131a", highlightthickness=0, height=height)
        self.cmd = (0.0, 0.0, 0.0)      # vx mm/s, vy mm/s, wz 0.01rad/s
        self.meas = None                # (vx, vy, wz) 或 None
        self.wheel = 0.0                # 0x201 轮速 rad/s
        self.gz = 0.0                   # IMU z 轴角速度 rad/s
        self.yaw = None                 # IMU 航向(°)
        self.kind = "麦轮"              # 底盘类型
        self.track = 200.0              # 半轮距 mm
        self.base = 200.0               # 半轴距 mm
        self.bgmanager = None           # 壁纸管理器(redraw 时要把底图补回来)
        self.bind("<Configure>", lambda _e: self.redraw())

    def set_cmd(self, vx, vy, wz):
        self.cmd = (float(vx), float(vy), float(wz))
        self.redraw()

    def set_kind(self, kind):
        if kind in CHASSIS_TYPES:
            self.kind = kind
            self.redraw()

    def set_geom(self, track=None, base=None):
        if track is not None:
            self.track = max(10.0, float(track))
        if base is not None:
            self.base = max(10.0, float(base))
        self.redraw()

    def set_meas(self, wheel=None, gz=None, vx=None, vy=None, wz=None, yaw=None):
        if wheel is not None:
            self.wheel = float(wheel)
        if gz is not None:
            self.gz = float(gz)
        if yaw is not None:
            self.yaw = float(yaw)
        if vx is not None or vy is not None or wz is not None:
            self.meas = (float(vx or 0.0), float(vy or 0.0), float(wz or 0.0))
        self.redraw()

    def wheel_solution(self):
        """按当前底盘类型把 (vx, vy, wz) 解算成每个轮子的转速 / 转向角。"""
        vx, vy, wz = self.cmd
        w = wz / 100.0                          # 0.01rad/s → rad/s
        speeds, angles = {}, {}
        if self.kind in ("麦轮", "全向轮"):
            k = self.track + self.base
            speeds = {"FL": vx - vy - k * w, "FR": vx + vy + k * w,
                      "RL": vx + vy - k * w, "RR": vx - vy + k * w}
        else:
            for name, xi, yi in (("FL", self.base, -self.track),
                                 ("FR", self.base, self.track),
                                 ("RL", -self.base, -self.track),
                                 ("RR", -self.base, self.track)):
                vix = vx - w * yi
                viy = vy + w * xi
                speeds[name] = math.hypot(vix, viy)
                angles[name] = math.degrees(math.atan2(viy, vix))
        return speeds, angles

    def _arrow(self, cx, cy, r0, vx, vy, color, width, dash=None):
        mag = math.hypot(vx, vy)
        if mag < 1e-6:
            return
        k = min(1.0, mag / self.MAX_MM)
        r1 = r0 + 34 + 74 * k
        ang = math.atan2(vy, vx)              # 屏幕坐标:右=+x,前=-y
        self.create_line(cx + math.cos(ang) * r0, cy - math.sin(ang) * r0,
                         cx + math.cos(ang) * r1, cy - math.sin(ang) * r1,
                         fill=color, width=width, arrow="last",
                         arrowshape=(14, 18, 6), dash=dash)

    def _spin(self, cx, cy, wz, color, radius=116):
        if abs(wz) < 1e-6:
            return
        span = max(-250.0, min(250.0, wz / self.MAX_W * 250.0))
        pts = []
        for i in range(25):
            a = -math.pi / 2 + math.radians(span) * (i / 24.0)
            pts.extend((cx + math.cos(a) * radius, cy + math.sin(a) * radius))
        self.create_line(*pts, fill=color, width=3, smooth=True,
                         arrow="last", arrowshape=(12, 16, 5))

    def redraw(self):
        self.delete("all")
        if self.bgmanager is not None:
            self.bgmanager.paint(self)      # delete("all") 会把底图删掉,补回来
        w = max(self.winfo_width(), 360)
        h = max(self.winfo_height(), 240)
        cx, cy = w / 2, h / 2 - 18
        bw = bh = 86
        speeds, angles = self.wheel_solution()

        # 车身 + 车头
        self.create_rectangle(cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2,
                              outline="#46525f", width=2, fill="#151b23")
        self.create_polygon(cx - 7, cy - bh / 2 + 6, cx + 7, cy - bh / 2 + 6,
                            cx, cy - bh / 2 - 8, fill="#7f8c99", outline="")

        # 四个轮子:舵轮按转向角画,麦轮/全向轮画辊子方向
        for name, sx, sy in (("FL", -1, -1), ("FR", 1, -1), ("RL", -1, 1), ("RR", 1, 1)):
            wx = cx + sx * (bw / 2 + 13)
            wy = cy + sy * 30
            spd = speeds.get(name, 0.0)
            if self.kind == "舵轮":
                a = math.radians(angles.get(name, 0.0))
                dx, dy = math.sin(a), -math.cos(a)
                self.create_line(wx - dx * 15, wy - dy * 15, wx + dx * 15, wy + dy * 15,
                                 fill="#7f8c99", width=7)
                self.create_line(wx - dx * 15, wy - dy * 15, wx + dx * 15, wy + dy * 15,
                                 fill="#26313d", width=3)
            else:
                self.create_rectangle(wx - 7, wy - 16, wx + 7, wy + 16,
                                      fill="#26313d", outline="#4a5866")
                if self.kind == "全向轮":     # 45° 辊子示意
                    self.create_line(wx - 6, wy + 12, wx + 6, wy - 12, fill="#4a5866")
                else:
                    self.create_line(wx, wy - 12, wx, wy + 12, fill="#3a4652")

            # 轮速箭头(向上=正转/向前)
            k = min(1.0, abs(spd) / self.MAX_MM)
            if k > 0.02:
                dy = -1 if spd >= 0 else 1
                self.create_line(wx + sx * 12, wy, wx + sx * 12, wy + dy * (10 + 26 * k),
                                 fill="#4ad0ff", width=3, arrow="last",
                                 arrowshape=(8, 10, 3))
            label = name if self.kind != "舵轮" else "%s %+.0f°" % (name, angles.get(name, 0.0))
            self.create_text(wx, wy + (34 if sy > 0 else -34), text=label,
                             fill="#9fb3c8", font=("Consolas", 8))

        # 下发速度(粗箭头) / 实测(细虚线) / 转动弧
        vx, vy, wz = self.cmd
        self._arrow(cx, cy, bw / 2 + 18, vx, vy, "#4ad0ff", 6)
        if self.meas:
            mx, my, _mwz = self.meas
            self._arrow(cx, cy, bw / 2 + 18, mx, my, "#ffd24a", 2, dash=(6, 4))
        self._spin(cx, cy, wz, "#7dff9b")

        # 文字:下发 / 轮速解算 / 实测
        mag = math.hypot(vx, vy)
        self.create_text(10, 10, anchor="nw", fill="#e6ecf3", font=("Consolas", 10),
                         text="%s   下发 vx %+.0f  vy %+.0f  |v| %.0f mm/s   wz %+.0f (%.2f rad/s)"
                              % (self.kind, vx, vy, mag, wz, wz / 100.0))
        self.create_text(10, 28, anchor="nw", fill="#7f8c99", font=("Consolas", 9),
                         text="▲=车头  粗箭头=下发  细虚线=实测  绿弧=转动  蓝色小箭头=该轮转速")
        wheel_txt = "   ".join("%s %+.0f" % (n, speeds[n]) for n in ("FL", "FR", "RL", "RR"))
        self.create_text(10, h - 30, anchor="sw", fill="#9fb3c8",
                         font=("Consolas", 9), text="轮速(mm/s): " + wheel_txt)
        meas = "实测  轮速 %.2f rad/s   gz %+.3f rad/s" % (self.wheel, self.gz)
        if self.yaw is not None:
            meas += "   yaw %.1f°" % self.yaw
        if self.meas:
            meas += "   vx %+.0f vy %+.0f" % (self.meas[0], self.meas[1])
        self.create_text(10, h - 10, anchor="sw", fill="#7f8c99",
                         font=("Consolas", 9), text=meas)


class GimbalView(tk.Canvas):
    """云台姿态:yaw 罗盘(目标/实际)+ 右侧 pitch 标尺。"""

    def __init__(self, master, height=300):
        super().__init__(master, bg="#0d131a", highlightthickness=0, height=height)
        self.yaw_act = None
        self.yaw_tgt = None
        self.pitch_act = None
        self.pitch_tgt = None
        self.rpm = 0.0
        self.bgmanager = None
        self.bind("<Configure>", lambda _e: self.redraw())

    def set_state(self, yaw_act=None, yaw_tgt=None, pitch_act=None,
                  pitch_tgt=None, rpm=None):
        if yaw_act is not None:
            self.yaw_act = float(yaw_act) % 360.0
        if yaw_tgt is not None:
            self.yaw_tgt = float(yaw_tgt) % 360.0
        if pitch_act is not None:
            self.pitch_act = float(pitch_act)
        if pitch_tgt is not None:
            self.pitch_tgt = float(pitch_tgt)
        if rpm is not None:
            self.rpm = float(rpm)
        self.redraw()

    def redraw(self):
        self.delete("all")
        if self.bgmanager is not None:
            self.bgmanager.paint(self)      # delete("all") 会把底图删掉,补回来
        w = max(self.winfo_width(), 360)
        h = max(self.winfo_height(), 220)
        cx, cy = w * 0.34, h * 0.44
        r = min(w * 0.24, h * 0.30)

        # 罗盘刻度(0 在正上方,顺时针)
        self.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#34495e", width=2)
        for deg in range(0, 360, 15):
            a = math.radians(deg)
            r1 = r - (14 if deg % 90 == 0 else 8)
            self.create_line(cx + r1 * math.sin(a), cy - r1 * math.cos(a),
                             cx + r * math.sin(a), cy - r * math.cos(a),
                             fill="#95a5a6" if deg % 90 == 0 else "#3d566e")
            if deg % 90 == 0:
                self.create_text(cx + (r + 14) * math.sin(a), cy - (r + 14) * math.cos(a),
                                 text="%d" % deg, fill="#9fb3c8", font=("Consolas", 9))

        def needle(deg, color, width, dash=None):
            a = math.radians(deg)
            self.create_line(cx, cy, cx + (r - 6) * math.sin(a), cy - (r - 6) * math.cos(a),
                             fill=color, width=width, dash=dash,
                             arrow="last", arrowshape=(10, 14, 4))

        if self.yaw_tgt is not None:
            needle(self.yaw_tgt, "#ffd24a", 2, dash=(6, 4))
        if self.yaw_act is not None:
            needle(self.yaw_act, "#4ad0ff", 4)
        self.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill="#ecf0f1", outline="")

        act = "-" if self.yaw_act is None else "%.1f°" % self.yaw_act
        tgt = "-" if self.yaw_tgt is None else "%.1f°" % self.yaw_tgt
        err = ""
        if self.yaw_act is not None and self.yaw_tgt is not None:
            e = ((self.yaw_tgt - self.yaw_act + 180) % 360) - 180
            err = "   误差 %+.1f°" % e
        self.create_text(cx, h - 32, fill="#e6ecf3", font=("Consolas", 10),
                         text="yaw 实际 %s   目标 %s%s" % (act, tgt, err))
        self.create_text(cx, h - 14, fill="#9fb3c8", font=("Consolas", 9),
                         text="反馈 %.1f rpm" % self.rpm)

        # pitch 竖标尺
        bx = w - 76
        top, bot = 40, h - 56
        self.create_line(bx, top, bx, bot, fill="#34495e", width=2)
        for deg in (-90, -45, 0, 45, 90):
            y = bot - (bot - top) * ((deg + 90) / 180.0)
            self.create_line(bx - 8, y, bx + 8, y, fill="#3d566e")
            self.create_text(bx + 16, y, text="%d" % deg, fill="#9fb3c8",
                             anchor="w", font=("Consolas", 8))

        def pm(deg, color, dash=None, width=3):
            deg = max(-90.0, min(90.0, deg))
            y = bot - (bot - top) * ((deg + 90) / 180.0)
            self.create_line(bx - 28, y, bx - 10, y, fill=color, width=width, dash=dash)

        if self.pitch_tgt is not None:
            pm(self.pitch_tgt, "#ffd24a", dash=(4, 3), width=2)
        if self.pitch_act is not None:
            pm(self.pitch_act, "#4ad0ff")
        self.create_text(bx, top - 14, text="pitch°", fill="#9fb3c8", font=("Consolas", 9))
        if self.pitch_act is None:
            self.create_text(bx - 52, (top + bot) / 2, text="无 pitch 数据",
                             fill="#667", font=("Consolas", 8), angle=90)


class _CmdWindow(tk.Toplevel):
    """底盘/云台窗口的公共部分:0x5C 发送、自定义 HEX、状态栏。"""

    TITLE = "调试助手 — 调试"
    TAG = "调试"

    def __init__(self, app, size):
        super().__init__(app)
        self.app = app
        self.title(self.TITLE)
        self.geometry(size)
        try:                      # 窗口底色跟主题走,否则 ttk 标签会浮在灰底上
            self.configure(bg=app.pal["bg"])
        except Exception:
            pass
        self.last = {}
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_close(self):
        """关窗前先把点动的 after 链停掉,免得回调打到已销毁的控件上。"""
        job = getattr(self, "_jog_job", None)
        if job is not None:
            try:
                self.after_cancel(job)
            except Exception:
                pass
            self._jog_job = None
        self.destroy()

    def _port_ready(self):
        if self.app.readers.get(0) is None:
            self.app._echo("[%s] 端口 1 没打开,先在主窗口打开串口" % self.TAG)
            return False
        return True

    def send(self, cmd, payload, note=""):
        self.app.send_debug_cmd(cmd, payload, tag=self.TAG)
        if note:
            self.app._echo("[%s] %s" % (self.TAG, note))

    def _hex_frame(self, parent):
        box = ttk.LabelFrame(parent, text="自定义 HEX(协议对不上时直接发原始帧)")
        box.pack(fill="x", padx=10, pady=(4, 8))
        row = ttk.Frame(box)
        row.pack(fill="x", padx=6, pady=6)
        var = tk.StringVar()
        ttk.Entry(row, textvariable=var, width=42).pack(side="left", padx=(0, 6))
        ttk.Button(row, text="发送", command=lambda: self._send_hex(var)).pack(side="left")
        ttk.Label(row, text="例:5C 05 00 01", style="Muted.TLabel").pack(side="left", padx=8)
        return box

    def _send_hex(self, var):
        try:
            data = _hex_bytes(var.get())
        except ValueError:
            self.app._echo("[%s] HEX 格式不对(只要 0-9 A-F)" % self.TAG)
            return
        if not data:
            return
        self.app.send_raw(data, tag=self.TAG)


class ChassisDebugWindow(_CmdWindow):
    """底盘调试:平移/旋转点动 + 目标速度下发 + 实时状态。

    命令走 0x5C 里自定义的 CHASSIS_CMD,payload = <h vx> <h vy> <h wz>:
        vx  mm/s       正=前进
        vy  mm/s       正=右移
        wz  0.01 rad/s 正=逆时针(左转)
    固件 cmd 号不同就改文件顶部的 CHASSIS_CMD(或在下面直接发 HEX)。
    """

    TITLE = "调试助手 — 底盘调试"
    TAG = "底盘"
    JOG_MS = 200          # 点动每 200ms 重发一次

    def __init__(self, app):
        super().__init__(app, "660x700")
        self.vx = tk.StringVar(value="0")
        self.vy = tk.StringVar(value="0")
        self.wz = tk.StringVar(value="0")
        self.jog_v = tk.StringVar(value="200")
        self.jog_w = tk.StringVar(value="80")
        self._jog_job = None
        self._jog_vec = (0, 0, 0)
        self._geom_job = None
        # 底盘类型 + 几何(决定轮速怎么解算)
        self.kind = tk.StringVar(value=self.app.cfg.get("chassis_type", CHASSIS_TYPES[0]))
        self.track = tk.StringVar(value=str(self.app.cfg.get("chassis_track", 200)))
        self.base = tk.StringVar(value=str(self.app.cfg.get("chassis_base", 200)))
        top = ttk.Frame(self)
        top.pack(fill="x", padx=10, pady=(8, 0))
        ttk.Label(top, text="底盘类型").pack(side="left")
        cb = ttk.Combobox(top, textvariable=self.kind, values=CHASSIS_TYPES,
                          state="readonly", width=8)
        cb.pack(side="left", padx=(4, 14))
        cb.bind("<<ComboboxSelected>>", lambda _e: self._on_kind())
        ttk.Label(top, text="半轮距 mm").pack(side="left")
        ttk.Entry(top, width=6, textvariable=self.track).pack(side="left", padx=(4, 12))
        ttk.Label(top, text="半轴距 mm").pack(side="left")
        ttk.Entry(top, width=6, textvariable=self.base).pack(side="left", padx=(4, 0))
        self.track.trace_add("write", lambda *_a: self._on_geom())
        self.base.trace_add("write", lambda *_a: self._on_geom())

        jog = ttk.LabelFrame(self, text="点动(按住走、松开停)")
        jog.pack(fill="x", padx=10, pady=(8, 4))
        pad = ttk.Frame(jog)
        pad.pack(pady=6)

        def mk(text, vx, vy, wz, r, c):
            b = ttk.Button(pad, text=text, width=8)
            b.grid(row=r, column=c, padx=3, pady=3)
            b.bind("<ButtonPress-1>", lambda _e: self._jog_start(vx, vy, wz))
            b.bind("<ButtonRelease-1>", lambda _e: self._jog_stop())
            return b

        mk("↑ 前", 1, 0, 0, 0, 1)
        mk("← 左", 0, -1, 0, 1, 0)
        mk("■ 停", 0, 0, 0, 1, 1)
        mk("→ 右", 0, 1, 0, 1, 2)
        mk("↓ 后", -1, 0, 0, 2, 1)
        mk("↺ 左转", 0, 0, -1, 2, 0)
        mk("↻ 右转", 0, 0, 1, 2, 2)

        row = ttk.Frame(jog)
        row.pack(fill="x", padx=8, pady=(0, 6))
        ttk.Label(row, text="点动 vx/vy (mm/s)").pack(side="left")
        ttk.Entry(row, width=7, textvariable=self.jog_v).pack(side="left", padx=4)
        ttk.Label(row, text="wz (0.01rad/s)").pack(side="left", padx=(10, 0))
        ttk.Entry(row, width=7, textvariable=self.jog_w).pack(side="left", padx=4)

        setf = ttk.LabelFrame(self, text="目标速度(持续,直到再发或点停止)")
        setf.pack(fill="x", padx=10, pady=4)
        row = ttk.Frame(setf)
        row.pack(fill="x", padx=8, pady=8)
        ttk.Label(row, text="vx mm/s").pack(side="left")
        ttk.Entry(row, width=8, textvariable=self.vx).pack(side="left", padx=(4, 12))
        ttk.Label(row, text="vy mm/s").pack(side="left")
        ttk.Entry(row, width=8, textvariable=self.vy).pack(side="left", padx=(4, 12))
        ttk.Label(row, text="wz 0.01rad/s").pack(side="left")
        ttk.Entry(row, width=8, textvariable=self.wz).pack(side="left", padx=(4, 12))
        ttk.Button(row, text="发送", command=self._send_speed).pack(side="left", padx=4)
        ttk.Button(row, text="停止", command=self._stop).pack(side="left", padx=4)

        self.lbl_state = ttk.Label(self, text="等待数据…", justify="left")
        self.lbl_state.pack(anchor="w", padx=14, pady=(6, 2))
        self.lbl_last = ttk.Label(self, text="", style="Muted.TLabel")
        self.lbl_last.pack(anchor="w", padx=14)
        # 俯视运动示意:比波形直观(车往哪走/多快/怎么转一眼看出)
        self.view = ChassisView(self, height=280)
        self.view.pack(fill="both", expand=True, padx=10, pady=(4, 2))
        self.view.bgmanager = self.app.bg
        self.app.bg.attach(self.view, extra_dim=0.35, align_to=self)   # 壁纸透到示意图后面
        self._hex_frame(self)
        self._on_kind()
        self._on_geom()

    # ---- 底盘类型 / 几何 ----
    def _on_kind(self):
        kind = self.kind.get()
        self.app._save_cfg(chassis_type=kind)
        view = getattr(self, "view", None)
        if view is not None:
            view.set_kind(kind)

    def _on_geom(self):
        view = getattr(self, "view", None)
        try:
            track = float(self.track.get() or 200)
            base = float(self.base.get() or 200)
        except ValueError:
            return
        if view is not None:
            view.set_geom(track=track, base=base)
        # 输入框每敲一个字都会回调,存配置防抖一下
        if self._geom_job is not None:
            try:
                self.after_cancel(self._geom_job)
            except Exception:
                pass
        self._geom_job = self.after(
            600, lambda: self.app._save_cfg(chassis_track=track, chassis_base=base))

    # ---- 发送 ----
    def _payload(self, vx, vy, wz):
        vx = max(-32768, min(32767, int(vx)))
        vy = max(-32768, min(32767, int(vy)))
        wz = max(-32768, min(32767, int(wz)))
        return struct.pack("<3h", vx, vy, wz)

    def send_speed(self, vx, vy, wz):
        self.send(CHASSIS_CMD, self._payload(vx, vy, wz),
                  "vx=%d vy=%d wz=%d" % (int(vx), int(vy), int(wz)))
        self.lbl_last.config(text="最近下发:vx=%d mm/s  vy=%d mm/s  wz=%d (0.01rad/s)"
                             % (int(vx), int(vy), int(wz)))
        self.view.set_cmd(vx, vy, wz)

    def _send_speed(self):
        try:
            self.send_speed(float(self.vx.get() or 0), float(self.vy.get() or 0),
                            float(self.wz.get() or 0))
        except ValueError:
            self.app._echo("[底盘] 速度只能填数字")

    def _stop(self):
        self.vx.set("0")
        self.vy.set("0")
        self.wz.set("0")
        self.send_speed(0, 0, 0)

    # ---- 点动:按住期间每 200ms 重发,松开立刻发 0 ----
    def _jog_start(self, vx, vy, wz):
        if not self._port_ready():
            return
        self._jog_vec = (vx, vy, wz)
        self._jog_stop(send=False)
        self._jog_tick()

    def _jog_tick(self):
        vx, vy, wz = self._jog_vec
        speed = float(self.jog_v.get() or 0)
        turn = float(self.jog_w.get() or 0)
        self.send_speed(vx * speed, vy * speed, wz * turn)
        self._jog_job = self.after(self.JOG_MS, self._jog_tick)

    def _jog_stop(self, send=True):
        if getattr(self, "_jog_job", None) is not None:
            try:
                self.after_cancel(self._jog_job)
            except Exception:
                pass
            self._jog_job = None
        if send and any(self._jog_vec):
            self._jog_vec = (0, 0, 0)
            self.send_speed(0, 0, 0)

    # ---- 状态 ----
    def feed(self, d):
        try:
            self.last = d
            if d.get("proto") == "MOTOR_PID":
                ms = d.get("motors") or []
                wheel = ms[1]["w"] if len(ms) > 1 else 0.0
                yaw = math.degrees(d.get("yaw", 0.0))
                gz = d.get("gz", 0.0)
                self.lbl_state.config(
                    text="轮速 0x201 %.2f rad/s    云台 0x202 %.1f°    IMU yaw %.1f°  gz %+.2f rad/s"
                         % (wheel, math.degrees(d.get("pos", 0.0)), yaw, gz))
                self.view.set_meas(wheel=wheel, gz=gz, yaw=yaw)
            else:
                self.lbl_state.config(
                    text="底盘 vx %+.3f m/s  vy %+.3f m/s  w %+.3f rad/s    云台 yaw %+.3f  pitch %+.3f rad"
                         % (d.get("vx", 0.0), d.get("vy", 0.0), d.get("w", 0.0),
                            d.get("gimbal_yaw", 0.0), d.get("gimbal_pitch", 0.0)))
                # 旧 210B 帧才带底盘实测速度(换算成 mm/s、0.01rad/s)
                self.view.set_meas(vx=float(d.get("vx", 0.0)) * 1000.0,
                                   vy=float(d.get("vy", 0.0)) * 1000.0,
                                   wz=float(d.get("w", 0.0)) * 100.0,
                                   gz=float(d.get("w", 0.0)))
        except Exception:
            pass


class GimbalDebugWindow(_CmdWindow):
    """云台调试:角度给定 + 点动微调 + 状态。

    命令走 0x5C 里自定义的 GIMBAL_CMD,payload = <i yaw> <i pitch>(单位 0.001°):
        正 yaw = 左转,正 pitch = 抬头(按你云台坐标系的约定改符号即可)
    """

    TITLE = "调试助手 — 云台调试"
    TAG = "云台"

    def __init__(self, app):
        super().__init__(app, "660x680")
        self.yaw = tk.StringVar(value="0")
        self.pitch = tk.StringVar(value="0")
        self.step = tk.StringVar(value="1.0")
        self._jog_job = None
        self._jog_dir = (0, 0)

        ang = ttk.LabelFrame(self, text="目标角度(绝对角,单位 °)")
        ang.pack(fill="x", padx=10, pady=(8, 4))
        row = ttk.Frame(ang)
        row.pack(fill="x", padx=8, pady=8)
        ttk.Label(row, text="yaw").pack(side="left")
        ttk.Entry(row, width=9, textvariable=self.yaw).pack(side="left", padx=(4, 12))
        ttk.Label(row, text="pitch").pack(side="left")
        ttk.Entry(row, width=9, textvariable=self.pitch).pack(side="left", padx=(4, 12))
        ttk.Button(row, text="发送角度", command=self._send_angle).pack(side="left", padx=4)
        ttk.Button(row, text="回中", command=self._center).pack(side="left", padx=4)

        jog = ttk.LabelFrame(self, text="点动微调(按住连续走,松开停)")
        jog.pack(fill="x", padx=10, pady=4)
        pad = ttk.Frame(jog)
        pad.pack(pady=6)
        for text, dy, dp, r, c in (("↑ pitch+", 0, 1, 0, 1), ("↓ pitch-", 0, -1, 2, 1),
                                   ("← yaw-", -1, 0, 1, 0), ("→ yaw+", 1, 0, 1, 2),
                                   ("■ 停", 0, 0, 1, 1)):
            b = ttk.Button(pad, text=text, width=9)
            b.grid(row=r, column=c, padx=3, pady=3)
            b.bind("<ButtonPress-1>", lambda _e, a=dy, b2=dp: self._jog_start(a, b2))
            b.bind("<ButtonRelease-1>", lambda _e: self._jog_stop())
        row = ttk.Frame(jog)
        row.pack(fill="x", padx=8, pady=(0, 6))
        ttk.Label(row, text="每步 °").pack(side="left")
        ttk.Entry(row, width=7, textvariable=self.step).pack(side="left", padx=4)
        ttk.Label(row, text="(按住后每 150ms 走一步)", style="Muted.TLabel").pack(side="left", padx=6)

        self.lbl_state = ttk.Label(self, text="等待数据…", justify="left")
        self.lbl_state.pack(anchor="w", padx=14, pady=(6, 2))
        self.lbl_last = ttk.Label(self, text="", style="Muted.TLabel")
        self.lbl_last.pack(anchor="w", padx=14)
        # 罗盘 + 俯仰标尺:目标/实际一眼对比
        self.view = GimbalView(self, height=270)
        self.view.pack(fill="both", expand=True, padx=10, pady=(4, 2))
        self.view.bgmanager = self.app.bg
        self.app.bg.attach(self.view, extra_dim=0.35, align_to=self)   # 壁纸透到罗盘后面
        self._hex_frame(self)

    # ---- 发送 ----
    def send_angle(self, yaw_deg, pitch_deg):
        yaw_i = max(-324000000, min(324000000, int(round(yaw_deg * 1000))))
        pit_i = max(-324000000, min(324000000, int(round(pitch_deg * 1000))))
        self.send(GIMBAL_CMD, struct.pack("<2i", yaw_i, pit_i),
                  "yaw=%.3f° pitch=%.3f°" % (yaw_deg, pitch_deg))
        self.lbl_last.config(text="最近下发:yaw %.3f°  pitch %.3f°" % (yaw_deg, pitch_deg))

    def _send_angle(self):
        try:
            self.send_angle(float(self.yaw.get() or 0), float(self.pitch.get() or 0))
        except ValueError:
            self.app._echo("[云台] 角度只能填数字")

    def _center(self):
        self.yaw.set("0")
        self.pitch.set("0")
        self.send_angle(0.0, 0.0)
        self.app._echo("[云台] 回中")

    # ---- 点动 ----
    def _jog_start(self, dy, dp):
        if not self._port_ready():
            return
        self._jog_dir = (dy, dp)
        self._jog_stop()
        self._jog_tick()

    def _jog_tick(self):
        dy, dp = self._jog_dir
        if not (dy or dp):
            return
        try:
            step = float(self.step.get() or 1.0)
        except ValueError:
            step = 1.0
        try:
            yaw = float(self.yaw.get() or 0) + dy * step
            pitch = float(self.pitch.get() or 0) + dp * step
        except ValueError:
            yaw = self.pitch = 0.0
        self.yaw.set("%.3f" % yaw)
        self.pitch.set("%.3f" % pitch)
        self.send_angle(yaw, pitch)
        self._jog_job = self.after(150, self._jog_tick)

    def _jog_stop(self):
        if getattr(self, "_jog_job", None) is not None:
            try:
                self.after_cancel(self._jog_job)
            except Exception:
                pass
            self._jog_job = None
        self._jog_dir = (0, 0)

    # ---- 状态 ----
    def feed(self, d):
        try:
            self.last = d
            if d.get("proto") == "MOTOR_PID":
                pos = math.degrees(d.get("pos", 0.0))
                tgt = math.degrees(d.get("pos_target", 0.0))
                rpm = d.get("pid_feedback_rpm", 0.0)
                err = ((tgt - pos + 180.0) % 360.0) - 180.0
                online = "在线" if (d.get("flags", 0) & 2) else "离线"
                self.lbl_state.config(
                    text="0x202 %s   实际 %.2f°   目标 %.2f°   误差 %+.2f°   反馈 %.1f rpm"
                         % (online, pos, tgt, err, rpm))
                self.view.set_state(yaw_act=pos, yaw_tgt=tgt, rpm=rpm)
            else:
                yaw = math.degrees(d.get("gimbal_yaw", 0.0))
                pitch = math.degrees(d.get("gimbal_pitch", 0.0))
                self.lbl_state.config(
                    text="云台 yaw %+.3f rad  pitch %+.3f rad   底盘 vx %+.2f vy %+.2f"
                         % (d.get("gimbal_yaw", 0.0), d.get("gimbal_pitch", 0.0),
                            d.get("vx", 0.0), d.get("vy", 0.0)))
                self.view.set_state(yaw_act=yaw, pitch_act=pitch)
        except Exception:
            pass


class VofaWindow(tk.Toplevel):
    """VOFA-style dynamic channels: channel names come from JSON telemetry."""

    COLORS = ("#4ad0ff", "#ffd24a", "#ff6b6b", "#7dff9b", "#c792ea", "#ff9f43", "#00d2d3", "#f368e0")

    def __init__(self, app):
        super().__init__(app)
        self.app = app
        self.title("调试助手 — VOFA+ 波形")
        try:
            self.state("zoomed")
        except Exception:
            self.geometry("1500x950")
        self.configure(bg="#0d131a")
        try:
            self.attributes("-topmost", True)
            self.after(1500, lambda: self.attributes("-topmost", False))
        except Exception:
            pass
        self.series = collections.OrderedDict()
        self.order = []
        self.paused = False

        bar = ttk.Frame(self)
        bar.pack(fill="x", padx=8, pady=6)
        ttk.Button(bar, text="暂停", command=self._toggle_pause).pack(side="left")
        ttk.Button(bar, text="清空", command=self._clear).pack(side="left", padx=6)
        ttk.Button(bar, text="选中前 4 路", command=self._select_first4).pack(side="left", padx=6)
        ttk.Label(bar, text="通道由固件 JSON 自动生成；可 Ctrl/Shift 多选").pack(side="left", padx=12)

        body = ttk.Panedwindow(self, orient="horizontal")
        body.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        left = ttk.Frame(body)
        style = ttk.Style(self)
        style.configure("Vofa.Treeview", background="#101820", fieldbackground="#101820",
                        foreground="#d7e2ec", rowheight=24)
        style.configure("Vofa.Treeview.Heading", background="#1b2633", foreground="#9fb3c8")
        self.tree = ttk.Treeview(left, columns=("value",), show="tree headings",
                                 selectmode="extended", height=30,
                                 style="Vofa.Treeview")
        self.tree.heading("#0", text="通道")
        self.tree.heading("value", text="最新值")
        self.tree.column("#0", width=210)
        self.tree.column("value", width=110, anchor="e")
        self.tree.pack(fill="both", expand=True)
        self.tree.insert("", "end", iid="__wait__", text="等待遥测数据…", values=("",))
        self.tree.bind("<<TreeviewSelect>>", lambda _e: self._redraw())
        body.add(left, weight=1)

        right = ttk.Frame(body)
        attf = tk.Frame(right, bg="#0d131a")
        attf.pack(fill="x")
        self.att = tk.Canvas(attf, width=230, height=190, bg="#0d131a", highlightthickness=0)
        self.att.pack(side="left", padx=4, pady=4)
        self.att_info = tk.Label(attf, text="等待 IMU…", bg="#0d131a", fg="#9fb3c8",
                                 justify="left", anchor="nw", font=("Consolas", 10))
        self.att_info.pack(side="left", padx=10, pady=8, anchor="n")
        self.canvas = tk.Canvas(right, bg="#101418", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        body.add(right, weight=4)
        self.app.bg.attach(self.canvas, extra_dim=0.25, align_to=self)   # 波形区铺背景图
        self.app.bg.attach(self.att, extra_dim=0.25, align_to=self)      # 姿态小窗也铺

        self.canvas.bind("<Configure>", lambda _e: self._redraw())
        self.after(100, self._redraw)

    def _toggle_pause(self):
        self.paused = not self.paused

    def _clear(self):
        for q in self.series.values():
            q.clear()
        self._redraw()

    def _select_first4(self):
        items = self.tree.get_children()
        self.tree.selection_set(items[:4])

    @staticmethod
    def _walk(prefix, value, out):
        if isinstance(value, dict):
            for k, v in value.items():
                VofaWindow._walk(f"{prefix}.{k}" if prefix else str(k), v, out)
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            out.append((prefix, float(value)))
        elif isinstance(value, bool):
            out.append((prefix, 1.0 if value else 0.0))

    def feed(self, data):
        if self.paused:
            return
        vals = []
        self._walk("", data, vals)

        if vals and self.tree.exists("__wait__"):
            self.tree.delete("__wait__")

        for name, value in vals:
            if name not in self.series:
                self.series[name] = collections.deque(maxlen=600)
                self.order.append(name)
                self.tree.insert("", "end", iid=name, text=name, values=("",))
            self.series[name].append(value)
            if self.tree.exists(name):
                self.tree.set(name, "value", f"{value:+.3f}")

        if not self.tree.selection() and self.order:
            self.tree.selection_set(self.order[:4])
        self._redraw()

    def _selected(self):
        sel = list(self.tree.selection())
        return [x for x in sel if x in self.series] or self.order[:4]

    def _draw_attitude(self):
        cv = self.att
        cv.delete("all")
        self.app.bg.paint(cv)
        w = max(cv.winfo_width(), 230)
        h = max(cv.winfo_height(), 190)
        cx, cy, R = w * 0.48, h * 0.52, min(w, h) * 0.32
        cv.create_oval(cx - R, cy - R, cx + R, cy + R, outline="#314151", width=1)
        cv.create_oval(cx - R, cy - R * 0.35, cx + R, cy + R * 0.35,
                       outline="#22303d", width=1)
        cv.create_line(cx - R, cy, cx + R, cy, fill="#22303d")
        cv.create_line(cx, cy - R, cx, cy + R, fill="#22303d")

        def val(name, default=0.0):
            q = self.series.get(name)
            return (q[-1] if q else default) if q else default

        roll = val("imu.roll")
        pitch = val("imu.pitch")
        yaw = val("imu.yaw")
        cyw, syw = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        # R = Rz(yaw) Ry(pitch) Rx(roll)
        Rm = (
            (cyw * cp, cyw * sp * sr - syw * cr, cyw * sp * cr + syw * sr),
            (syw * cp, syw * sp * sr + cyw * cr, syw * sp * cr - cyw * sr),
            (-sp, cp * sr, cp * cr),
        )
        axes = ((1, 0, 0, "#ff6b6b", "X"), (0, 1, 0, "#7dff9b", "Y"),
                (0, 0, 1, "#4ad0ff", "Z"))
        for ax, ay, az, color, label in axes:
            vx = Rm[0][0] * ax + Rm[0][1] * ay + Rm[0][2] * az
            vy = Rm[1][0] * ax + Rm[1][1] * ay + Rm[1][2] * az
            vz = Rm[2][0] * ax + Rm[2][1] * ay + Rm[2][2] * az
            ex, ey = cx + vx * R, cy - vy * R
            cv.create_line(cx, cy, ex, ey, fill=color, width=3)
            cv.create_text(ex, ey, text=label, fill=color, font=("Consolas", 9, "bold"))
        # quaternion from Euler
        qw = cr * cp * cyw + sr * sp * syw
        qx = sr * cp * cyw - cr * sp * syw
        qy = cr * sp * cyw + sr * cp * syw
        qz = cr * cp * syw - sr * sp * cyw
        self.att_info.config(
            text=f"roll  {roll:+.3f} rad\npitch {pitch:+.3f} rad\nyaw   {yaw:+.3f} rad\n\n"
                 f"q = {qw:+.3f}\n    {qx:+.3f}, {qy:+.3f}, {qz:+.3f}")

    def _redraw(self):
        self._draw_attitude()
        cv = self.canvas
        cv.delete("all")
        self.app.bg.paint(cv)
        w = max(cv.winfo_width(), 200)
        h = max(cv.winfo_height(), 120)
        if not self.app.bg.active:                 # 有背景图时别再盖一层实色
            cv.create_rectangle(0, 0, w, h, fill="#101418", outline="")
        cv.create_text(10, 8, text="VOFA", fill="#7f8c99", anchor="nw")

        names = self._selected()
        if not names:
            cv.create_text(w / 2, h / 2, text="等待 JSON 遥测…", fill="#667")
            return

        pad = 42
        plot_l, plot_r, plot_t, plot_b = pad, w - 118, 28, h - 28
        cv.create_rectangle(plot_l, plot_t, plot_r, plot_b, outline="#29323b")
        for k in range(1, 5):
            y = plot_t + (plot_b - plot_t) * k / 5
            cv.create_line(plot_l, y, plot_r, y, fill="#1d252d")

        all_vals = []
        for n in names:
            vals = list(self.series.get(n, []))
            if vals:
                all_vals.extend(vals[-600:])
        if not all_vals:
            return
        lo, hi = min(all_vals), max(all_vals)
        if hi - lo < 1e-6:
            hi = lo + 1.0
        span = hi - lo

        for idx, n in enumerate(names):
            color = self.COLORS[idx % len(self.COLORS)]
            vals = list(self.series.get(n, []))[-600:]
            if not vals:
                continue
            pts = []
            count = len(vals)
            for i, v in enumerate(vals):
                x = plot_l + (plot_r - plot_l) * (i / max(count - 1, 1))
                y = plot_b - (plot_b - plot_t) * ((v - lo) / span)
                pts.extend((x, y))
            if len(pts) >= 4:
                cv.create_line(*pts, fill=color, width=2)
            cv.create_text(plot_r + 8, plot_t + 14 * (idx + 1),
                           text=f"{n}  {vals[-1]:+.3f}", fill=color, anchor="w")

        cv.create_text(plot_l, plot_b + 8, text=f"{lo:+.3f}", fill="#667", anchor="nw")
        cv.create_text(plot_l, plot_t - 16, text=f"{hi:+.3f}", fill="#667", anchor="nw")


# ============================================================================
# 主窗口的数据区(壁纸透出)
# ============================================================================
# ttk.Treeview / tk.Text 都是实心底色,Tk 没有"控件透明"这种东西,
# 所以主窗口的字段表和日志改成自绘 Canvas:壁纸铺最底层(再压暗一档),
# 文字直接画在上面 —— 这样壁纸才真的透得出来。

class GlassTable(tk.Canvas):
    """字段表:两列(字段 / 最新值),壁纸垫底。"""

    ROW_H = 22
    HEAD_H = 24

    def __init__(self, master, bgmgr, font, name_w=230, **kw):
        super().__init__(master, highlightthickness=0, bd=0, **kw)
        self.bgmanager = bgmgr
        self.font = font
        self.name_w = name_w
        self.rows = []
        self.offset = 0
        self.pal = None
        self.sb = None
        self.bind("<Configure>", lambda _e: self.redraw())
        self.bind("<MouseWheel>", self._on_wheel)
        self.bind("<Button-4>", lambda _e: self._wheel(-3))   # Linux 上滚
        self.bind("<Button-5>", lambda _e: self._wheel(3))
        self.bind("<Control-c>", self._copy_all)
        self.bind("<Control-C>", self._copy_all)

    def _copy_all(self, _event=None):
        """整表复制到剪贴板(自绘控件没有选中功能,给个 Ctrl+C 兜底)。"""
        text = "\n".join("%s  %s" % (name, value) for name, value in self.rows)
        self.clipboard_clear()
        self.clipboard_append(text)
        return "break"

    # ---- 外部接口 ----
    def set_scrollbar(self, sb):
        self.sb = sb
        self._sync()

    def set_theme(self, pal):
        self.pal = pal
        try:
            self.configure(bg=pal["log_bg"])
        except Exception:
            pass
        self.redraw()

    def set_rows(self, rows):
        rows = list(rows)
        if rows == self.rows:
            return                      # 数据没变就别重画(每秒 16 次刷新)
        self.rows = rows
        self._clamp()
        self.redraw()

    # ---- 滚动 ----
    def visible_rows(self):
        h = max(self.winfo_height() - self.HEAD_H, self.ROW_H)
        return max(1, int(h // self.ROW_H))

    def _clamp(self):
        self.offset = max(0, min(self.offset,
                                 max(0, len(self.rows) - self.visible_rows())))

    def _sync(self):
        if self.sb is None:
            return
        total = max(1, len(self.rows))
        vis = self.visible_rows()
        self.sb.set(self.offset / total, min(1.0, (self.offset + vis) / total))

    def _on_wheel(self, event):
        self._wheel(-3 if getattr(event, "delta", 0) > 0 else 3)

    def _wheel(self, step):
        self.offset += step
        self._clamp()
        self.redraw()

    def yview(self, *args):
        if not args:
            return
        if args[0] == "moveto":
            frac = float(args[1])
            if frac > 0.97:
                self.offset = max(0, len(self.rows) - self.visible_rows())
            else:
                self.offset = int(frac * max(1, len(self.rows)))
        elif args[0] == "scroll":
            step = int(args[1])
            self.offset += step * (1 if args[2] == "units" else self.visible_rows())
        self._clamp()
        self.redraw()

    # ---- 绘制 ----
    def redraw(self):
        self.delete("cell")
        w = max(self.winfo_width(), 60)
        h = max(self.winfo_height(), 40)
        pal = self.pal or {}
        fg = pal.get("fg", "#e6ecf3")
        muted = pal.get("muted", "#8a97a8")
        accent = pal.get("accent", "#2f9e6e")
        panel = pal.get("panel", "#252b35")
        line = pal.get("entry", "#2f3742")

        self.create_rectangle(0, 0, w, self.HEAD_H, fill=panel, outline="", tags="cell")
        self.create_text(10, self.HEAD_H // 2, text="字段", anchor="w",
                         fill=accent, font=self.font, tags="cell")
        self.create_text(self.name_w + 10, self.HEAD_H // 2, text="最新值", anchor="w",
                         fill=accent, font=self.font, tags="cell")
        self.create_line(0, self.HEAD_H, w, self.HEAD_H, fill=line, tags="cell")

        if not self.rows:
            self.create_text(w // 2, (h + self.HEAD_H) // 2, text="等待数据…",
                             fill=muted, font=self.font, tags="cell")
            self._sync()
            return

        self._clamp()
        y = self.HEAD_H
        for name, value in self.rows[self.offset:self.offset + self.visible_rows()]:
            # 壁纸透出来时,文字加一层 1px 黑边,不靠压暗也能看清
            self.create_text(11, y + self.ROW_H // 2 + 1, text=name, anchor="w",
                             fill="#000000", font=self.font, tags="cell")
            self.create_text(10, y + self.ROW_H // 2, text=name, anchor="w",
                             fill=muted, font=self.font, tags="cell")
            self.create_text(self.name_w + 11, y + self.ROW_H // 2 + 1, text=value, anchor="w",
                             fill="#000000", font=self.font, tags="cell")
            self.create_text(self.name_w + 10, y + self.ROW_H // 2, text=value, anchor="w",
                             fill=fg, font=self.font, tags="cell")
            y += self.ROW_H
        self._sync()


class GlassLog(tk.Canvas):
    """日志:时间戳 + 文本,壁纸垫底。"""

    def __init__(self, master, bgmgr, font, **kw):
        super().__init__(master, highlightthickness=0, bd=0, **kw)
        self.bgmanager = bgmgr
        self.font = font
        self.pal = None
        self.sb = None
        self.lines = collections.deque(maxlen=800)
        self.offset = 0                      # 往回翻多少行
        try:
            self._lh = tkfont.Font(font=font).metrics("linespace") + 3
        except Exception:
            self._lh = 18
        self.bind("<Configure>", lambda _e: self.redraw())
        self.bind("<MouseWheel>", self._on_wheel)
        self.bind("<Button-4>", lambda _e: self._wheel(3))
        self.bind("<Button-5>", lambda _e: self._wheel(-3))
        self.bind("<Control-c>", self._copy_all)
        self.bind("<Control-C>", self._copy_all)

    def _copy_all(self, _event=None):
        """整个日志复制到剪贴板。"""
        self.clipboard_clear()
        self.clipboard_append("\n".join(self.lines))
        return "break"

    # ---- 外部接口 ----
    def set_scrollbar(self, sb):
        self.sb = sb
        self._sync()

    def set_theme(self, pal):
        self.pal = pal
        try:
            self.configure(bg=pal["log_bg"])
        except Exception:
            pass
        self.redraw()

    def add(self, text):
        self.lines.append(datetime.now().strftime("%H:%M:%S ") + str(text))
        self.offset = 0                      # 新消息来了就回到最新一行
        self.redraw()

    def clear(self):
        self.lines.clear()
        self.offset = 0
        self.redraw()

    # ---- 滚动 ----
    def _visible_lines(self):
        return max(1, int(max(self.winfo_height(), 20) // self._lh))

    def _clamp(self):
        self.offset = max(0, min(self.offset, max(0, len(self.lines) - self._visible_lines())))

    def _sync(self):
        if self.sb is None:
            return
        total = max(1, len(self.lines))
        vis = self._visible_lines()
        self.sb.set(self.offset / total, min(1.0, (self.offset + vis) / total))

    def _on_wheel(self, event):
        self._wheel(3 if getattr(event, "delta", 0) > 0 else -3)

    def _wheel(self, step):
        self.offset += step
        self._clamp()
        self.redraw()

    def yview(self, *args):
        if not args:
            return
        if args[0] == "moveto":
            frac = float(args[1])
            if frac > 0.97:
                self.offset = 0                       # 拖到底 = 最新
            else:
                self.offset = int((1.0 - frac) * max(1, len(self.lines)))
        elif args[0] == "scroll":
            step = int(args[1])
            self.offset -= step * (1 if args[2] == "units" else self._visible_lines())
        self._clamp()
        self.redraw()

    # ---- 绘制 ----
    def redraw(self):
        self.delete("cell")
        w = max(self.winfo_width(), 60)
        h = max(self.winfo_height(), 20)
        pal = self.pal or {}
        fg = pal.get("fg", "#e6ecf3")
        muted = pal.get("muted", "#8a97a8")

        n = self._visible_lines()
        total = len(self.lines)
        self.offset = max(0, min(self.offset, max(0, total - n)))
        start = max(0, total - n - self.offset)
        y = 3
        for line in list(self.lines)[start:start + n]:
            stamp, _sep, msg = line.partition(" ")
            self.create_text(9, y + 1, text=stamp, anchor="nw", fill="#000000",
                             font=self.font, tags="cell")
            self.create_text(8, y, text=stamp, anchor="nw", fill=muted,
                             font=self.font, tags="cell")
            self.create_text(71, y + 1, text=msg, anchor="nw", fill="#000000",
                             font=self.font, tags="cell")
            self.create_text(70, y, text=msg, anchor="nw", fill=fg,
                             font=self.font, tags="cell")
            y += self._lh
        self._sync()


class App(tk.Tk):
    _scanning = False      # 自动匹配是否在进行(防重入)
    _prev_tick = 0         # 上一帧 error 心跳(判断是否在涨)
    def __init__(self):
        super().__init__()
        self.title(f"调试助手 — {CONFIG['platform']}")
        self.geometry("1240x820")
        self.minsize(1000, 700)

        self.recorder = Recorder()
        self.decoders = {}
        self.readers = {}
        self.motor_win = None
        self.vofa_win = None
        self.steer_win = None
        self.chassis_win = None
        self.gimbal_win = None
        self.cmd_seq = 0
        self.vofa_last = None
        self.last_json = None
        # 最近解出的帧(电机调试窗口 push 用)。原代码只读 self.dbg 却从未初始化,
        # _tick 每轮在这里抛 AttributeError,after 链直接断掉 ⇒ 界面永远 0 帧。
        self.dbg = collections.deque(maxlen=300)
        self._scan_start = 0.0
        self.q = queue.Queue()
        self.report_callback_exception = self._report_cb_exception
        self.paused = tk.BooleanVar(value=False)
        self.recording = tk.BooleanVar(value=True)   # 是否记录收到的帧
        self.font = CONFIG["font"]
        self.font_big = CONFIG["font_big"]
        self.cfg = self._load_cfg()
        self.pal = THEMES["深色"]
        self.bg = BackgroundLayer(self)
        self._dim_busy = False
        self._glass_win = None          # "整窗半透明"时背后那层壁纸窗口
        self._glass_cv = None

        self._build()

        th = self.cfg.get("theme", "深色")
        if th == "自定义" and self.cfg.get("custom_bg"):
            self._apply_theme("自定义", _custom_pal(self.cfg["custom_bg"]))
        else:
            self._apply_theme(th if th in THEMES else "深色")

        self._restore_bg_image()
        self._refresh_ports(silent=True)
        self.after(50, self._tick)
        self.after(900, self._auto_open_first)
        # VOFA 窗口不再默认弹出,需要时点工具栏的「VOFA 波形」

    # ---------- 配置读写 ----------
    def _load_cfg(self):
        try:
            # utf-8-sig:记事本另存为 UTF-8 会带 BOM,带 BOM 时 json 会直接解析失败,
            # 之前的表现就是"配置里写了背景图却不生效"。
            with open(CFG_FILE, encoding="utf-8-sig") as fp:
                return json.load(fp)
        except Exception:
            return {}

    def _save_cfg(self, **kw):
        self.cfg.update(kw)
        try:
            with open(CFG_FILE, "w", encoding="utf-8") as fp:
                json.dump(self.cfg, fp, ensure_ascii=False, indent=2)
        except Exception:
            pass

    # ---------- 布局 ----------
    def _build(self):
        self.style = ttk.Style(self)
        try:
            self.style.theme_use("clam")
        except tk.TclError:
            pass

        # 最底层背景画布:垫在所有 ttk 控件下面;没有背景图时隐藏起来。
        # 表格/日志是实心底色,Tk 没法真透明,所以图片只从四周留白和各曲线画布露出来。
        self.bg_canvas = tk.Canvas(self, highlightthickness=0, bd=0)
        self.bg.attach(self.bg_canvas)      # 和曲线画布一样进动图播放队列
        self.bind("<Configure>", self._on_root_configure, add="+")
        # 最小化时把背后的壁纸窗口一起收起来
        self.bind("<Unmap>", lambda e: self._sync_glass() if e.widget is self else None, add="+")
        self.bind("<Map>", lambda e: self._sync_glass() if e.widget is self else None, add="+")

        head = ttk.Frame(self, style="Head.TFrame", padding=(12, 10))
        head.pack(fill="x")
        ttk.Label(head, text="MCU 数据接收器", style="Title.TLabel").pack(side="left")
        ttk.Label(head, text=f"{CONFIG['platform']} · 调试 0x5A + 上位机 0xA5",
                  style="Muted.TLabel").pack(side="left", padx=12)

        self.btn_bg = ttk.Menubutton(head, text="背景图 ▾")
        self.btn_bg.pack(side="right", padx=(6, 0))
        menu = tk.Menu(self.btn_bg, tearoff=False)
        menu.add_command(label="选择图片…(GIF 会动)", command=self._choose_bg_image)
        menu.add_separator()
        menu.add_command(label="变亮(-10%)", command=lambda: self._nudge_dim(-10))
        menu.add_command(label="变暗(+10%)", command=lambda: self._nudge_dim(+10))
        menu.add_separator()
        self.var_glass = tk.BooleanVar(value=bool(self.cfg.get("glass", False)))
        menu.add_checkbutton(label="整窗半透明(实验:图铺到顶部)",
                             variable=self.var_glass, command=self._toggle_glass)
        menu.add_separator()
        menu.add_command(label="去掉背景图", command=self._clear_bg_image)
        self.btn_bg["menu"] = menu

        self.var_theme = tk.StringVar(value=self.cfg.get("theme", "深色"))
        cb = ttk.Combobox(head, textvariable=self.var_theme, state="readonly", width=10,
                          values=list(THEMES.keys()) + ["自定义…"], font=self.font)
        cb.pack(side="right", padx=(6, 0))
        cb.bind("<<ComboboxSelected>>", lambda e: self._on_theme_choice())
        ttk.Label(head, text="背景", style="Muted.TLabel").pack(side="right")

        cards = ttk.Frame(self, padding=(12, 0))
        cards.pack(fill="x")
        self.port_vars, self.baud_vars, self.port_cbs, self.btn_vars, self.tag_lbls = [], [], [], [], []
        for i, (slot, hint) in enumerate(((1, "调试遥测 · 建议 921600"), (2, "USB CDC · miniPC/裁判"))):
            card = ttk.LabelFrame(cards, text=f"端口 {slot}   {hint}", padding=10)
            card.pack(side="left", fill="x", expand=True, padx=(0, 10) if i == 0 else 0)
            pv, bv = tk.StringVar(), tk.StringVar(value=CONFIG["baud_list"][0])
            cbx = ttk.Combobox(card, textvariable=pv, values=[], width=24, font=self.font)
            cbx.grid(row=0, column=0, columnspan=2, sticky="we", pady=(0, 6))
            ttk.Combobox(card, textvariable=bv, values=CONFIG["baud_list"], width=10,
                         font=self.font).grid(row=1, column=0, sticky="w")
            b = ttk.Button(card, text="打开", width=8, command=lambda k=i: self._toggle(k))
            b.grid(row=1, column=1, sticky="e")
            tag = ttk.Label(card, text="未连接", style="Muted.TLabel")
            tag.grid(row=2, column=0, columnspan=2, sticky="w", pady=(6, 0))
            self.port_vars.append(pv)
            self.baud_vars.append(bv)
            self.port_cbs.append(cbx)
            self.btn_vars.append(b)
            self.tag_lbls.append(tag)

        tools = ttk.Frame(self, padding=(12, 8))
        tools.pack(fill="x")
        # 一行放得下:连接 → 记录开关 → 调试窗口 → 导出/更多菜单
        ttk.Button(tools, text="⚡ 自动匹配", command=self._auto_match).pack(side="left")
        ttk.Button(tools, text="刷新端口", command=self._refresh_ports).pack(side="left", padx=(6, 0))
        ttk.Separator(tools, orient="vertical").pack(side="left", fill="y", padx=10)
        ttk.Checkbutton(tools, text="暂停显示", variable=self.paused).pack(side="left")
        ttk.Checkbutton(tools, text="记录", variable=self.recording).pack(side="left", padx=(10, 0))
        ttk.Separator(tools, orient="vertical").pack(side="left", fill="y", padx=10)
        for text, cb in (("电机", self._open_motor_debug),
                         ("舵轮", self._open_steer_debug),
                         ("底盘", self._open_chassis_debug),
                         ("云台", self._open_gimbal_debug),
                         ("VOFA 波形", self._open_vofa)):
            ttk.Button(tools, text=text, command=cb).pack(side="left", padx=(0, 4))
        ttk.Separator(tools, orient="vertical").pack(side="left", fill="y", padx=10)

        mb_exp = ttk.Menubutton(tools, text="导出 ▾")
        menu_exp = tk.Menu(mb_exp, tearoff=False)
        menu_exp.add_command(label="解码 CSV", command=lambda: self._export("csv"))
        menu_exp.add_command(label="原始 CSV", command=lambda: self._export_raw("csv"))
        menu_exp.add_command(label="原始 HEX", command=lambda: self._export_raw("hex"))
        menu_exp.add_command(label="文档 (MD)", command=lambda: self._export("md"))
        mb_exp["menu"] = menu_exp
        mb_exp.pack(side="left", padx=(0, 6))

        mb_more = ttk.Menubutton(tools, text="更多 ▾")
        menu_more = tk.Menu(mb_more, tearoff=False)
        menu_more.add_command(label="打开文件解析", command=self._open_file)
        menu_more.add_command(label="清空统计 / 记录", command=self._clear)
        mb_more["menu"] = menu_more
        mb_more.pack(side="left")

        bar = ttk.Frame(self, padding=(12, 4))
        bar.pack(fill="x")
        self.badge = tk.Label(bar, text="  --  ", font=self.font_big, padx=8, pady=2)
        self.badge.pack(side="left")
        self.var_status = tk.StringVar(value="就绪")
        ttk.Label(bar, textvariable=self.var_status, style="Muted.TLabel").pack(side="left", padx=10)

        send = ttk.Frame(self, padding=(12, 4))
        send.pack(fill="x")
        ttk.Label(send, text="发送 HEX", style="Muted.TLabel").pack(side="left")
        self.var_hex = tk.StringVar()
        e = ttk.Entry(send, textvariable=self.var_hex, width=46, font=self.font)
        e.pack(side="left", padx=6)
        e.bind("<Return>", lambda ev: self._send())
        self.var_slot = tk.StringVar(value="端口 1")
        ttk.Combobox(send, textvariable=self.var_slot, values=["端口 1", "端口 2"],
                     width=8, state="readonly", font=self.font).pack(side="left", padx=6)
        ttk.Button(send, text="发送", width=8, command=self._send).pack(side="left")

        paned = ttk.Panedwindow(self, orient="vertical")
        paned.pack(fill="both", expand=True, padx=12, pady=6)

        # 字段表:自绘 Canvas,壁纸垫底(表格再压暗一档,保证字看得清)
        self.table_wrap = ttk.Frame(paned)
        self.table = GlassTable(self.table_wrap, self.bg, self.font,
                                name_w=230, bg="#12161c")
        self.table_sb = ttk.Scrollbar(self.table_wrap, orient="vertical",
                                      command=self.table.yview)
        self.table.set_scrollbar(self.table_sb)
        self.table.pack(side="left", fill="both", expand=True)
        self.table_sb.pack(side="right", fill="y")
        paned.add(self.table_wrap, weight=3)
        self.bg.attach(self.table, extra_dim=0.20, align_to=self)

        # 日志:同样自绘,壁纸垫底
        self.log_wrap = ttk.Frame(paned)
        self.log = GlassLog(self.log_wrap, self.bg, self.font, bg="#12161c")
        self.log_sb = ttk.Scrollbar(self.log_wrap, orient="vertical", command=self.log.yview)
        self.log.set_scrollbar(self.log_sb)
        self.log.pack(side="left", fill="both", expand=True)
        self.log_sb.pack(side="right", fill="y")
        paned.add(self.log_wrap, weight=1)
        self.bg.attach(self.log, extra_dim=0.20, align_to=self)

        foot = ttk.Frame(self, padding=(12, 4, 12, 8))
        foot.pack(fill="x")
        self.var_rate = tk.StringVar()
        ttk.Label(foot, textvariable=self.var_rate, style="Muted.TLabel").pack(side="left")
        ttk.Button(foot, text="导出到 导出/ 目录", command=lambda: self._export("md")).pack(side="right")

        # 有背景图时才出现的"压暗"滑块:图片太亮会把曲线盖住
        self.dim_wrap = ttk.Frame(foot)
        ttk.Label(self.dim_wrap, text="背景压暗", style="Muted.TLabel").pack(side="left")
        self.var_dim = tk.DoubleVar(value=BG_DIM * 100.0)
        self.dim_scale = ttk.Scale(self.dim_wrap, from_=0, to=90, orient="horizontal",
                                   length=130, variable=self.var_dim, command=self._on_dim)
        self.dim_scale.pack(side="left", padx=(6, 0))
        self.dim_scale.bind("<ButtonRelease-1>", lambda e: self._save_cfg(bg_dim=self.bg.dim))

    # ---------- 背景图 ----------
    def _restore_bg_image(self):
        """启动时恢复上次用的背景图。"""
        path = str(self.cfg.get("bg_image") or "")
        if not path:
            return
        if not os.path.isfile(path):
            self._echo("[背景图] 上次的图片找不到了: " + path)
            return
        try:
            wp = self.bg.load(path)
        except Exception as exc:
            self._echo("[背景图] 恢复失败: %s" % exc)
            return
        self._after_bg_loaded(wp, restore=True)

    def _choose_bg_image(self):
        path = filedialog.askopenfilename(title="选一张背景图(GIF 会动)",
                                          filetypes=image_filetypes())
        if not path:
            return
        try:
            wp = self.bg.load(path)
        except Exception as exc:
            messagebox.showerror("背景图", "加载失败:\n%s" % exc)
            self._echo("[背景图] 加载失败: %s" % exc)
            return
        self._after_bg_loaded(wp)
        self._save_cfg(bg_image=path, bg_dim=round(self.bg.dim, 3))

    def _after_bg_loaded(self, wp, restore=False):
        dim = self.cfg.get("bg_dim")
        try:
            dim = float(dim)
        except (TypeError, ValueError):
            dim = self._auto_dim(wp)
        self._dim_busy = True
        try:
            self.var_dim.set(max(0.0, min(90.0, round(dim * 100.0))))
        finally:
            self._dim_busy = False
        self.bg.set_dim(dim)
        self._sync_bg_canvas()
        self._show_dim_control(True)
        note = "(已恢复)" if restore else ""
        self._echo("[背景图] %s %s" % (self.bg.info(), note))
        if not restore:
            self._echo("[背景图] 已铺到 电机调试 / 舵轮 / VOFA 的曲线画布,主窗口留白处也能看到")
        if wp.truncated:
            self._echo("[背景图] 帧数太多,只取了前 %d 帧" % BG_MAX_FRAMES)
        if self.var_glass.get():
            self._open_glass()

    def _clear_bg_image(self):
        self._close_glass()
        self.bg.unload()
        self._sync_bg_canvas()
        self._show_dim_control(False)
        self._save_cfg(bg_image="")
        self._echo("[背景图] 已去掉")

    # ---------- 整窗半透明(实验) ----------
    # Tk 的按钮/标签只有实心底色,图片没法从控件后面透出来。这里换个思路:
    # 在主窗口背后再放一个无边框窗口铺壁纸,再把主窗口整体调成 82% 不透明,
    # 于是壁纸就从包括顶部在内的所有面板底下透出来了(文字也会跟着淡一点)。
    def _toggle_glass(self):
        self._save_cfg(glass=bool(self.var_glass.get()))
        if self.var_glass.get():
            self._open_glass()
        else:
            self._close_glass()
            self._echo("[半透明] 已关闭")

    def _open_glass(self):
        if not self.bg.active:
            self.var_glass.set(False)
            self._echo("[半透明] 先选一张背景图")
            return
        if self._glass_win is not None:
            self._sync_glass()
            return
        win = tk.Toplevel(self)
        win.overrideredirect(True)
        try:
            win.attributes("-toolwindow", True)
        except tk.TclError:
            pass
        win.configure(bg=self.pal["bg"])
        cv = tk.Canvas(win, highlightthickness=0, bd=0, bg=self.pal["bg"])
        cv.pack(fill="both", expand=True)
        self._glass_win, self._glass_cv = win, cv
        self.bg.attach(cv)                    # 这层就是"窗口背后的壁纸"
        try:
            self.attributes("-alpha", 0.82)
        except tk.TclError:
            self._echo("[半透明] 这个系统不支持窗口透明,已跳过")
            self._close_glass()
            return
        self.var_glass.set(True)
        self._sync_glass()
        self._echo("[半透明] 开:整个窗口 82% 不透明,壁纸从所有面板底下透出来")

    def _close_glass(self):
        win, self._glass_win, self._glass_cv = self._glass_win, None, None
        if win is not None:
            try:
                win.destroy()
            except tk.TclError:
                pass
        try:
            self.attributes("-alpha", 1.0)
        except tk.TclError:
            pass
        try:
            self.var_glass.set(False)
        except Exception:
            pass

    def _sync_glass(self):
        """主窗口移动/改变大小时,把背后的壁纸窗口贴上去。"""
        win = self._glass_win
        if win is None:
            return
        try:
            if self.state() == "iconic":
                win.withdraw()
                return
            win.geometry("%dx%d+%d+%d" % (self.winfo_width(), self.winfo_height(),
                                          self.winfo_rootx(), self.winfo_rooty()))
            win.deiconify()
            self._stack_glass()
            if self._glass_cv is not None:
                self.bg.paint(self._glass_cv)
        except tk.TclError:
            pass

    def _stack_glass(self):
        """把壁纸窗口压在主窗口正下方。

        不能用 Toplevel.lower():那是 SetWindowPos(HWND_BOTTOM),在 Windows 上
        会掉到桌面下面,于是只看到桌面而不是壁纸。这里显式插到主窗口下面。
        """
        win = self._glass_win
        if win is None or not IS_WINDOWS:
            return
        try:
            import ctypes
            hwnd_glass = int(self.tk.call("wm", "frame", win._w), 16)
            hwnd_main = int(self.tk.call("wm", "frame", self._w), 16)
            ctypes.windll.user32.SetWindowPos(
                hwnd_glass, hwnd_main, 0, 0, 0, 0,
                0x0001 | 0x0002 | 0x0010)      # NOSIZE | NOMOVE | NOACTIVATE
        except Exception:
            pass

    def _nudge_dim(self, delta):
        if not self.bg.active:
            self._echo("[背景图] 先选一张背景图,再来调压暗")
            return
        value = max(0.0, min(90.0, self.var_dim.get() + delta))
        self._dim_busy = True
        try:
            self.var_dim.set(value)
            self.bg.set_dim(value / 100.0)
            self.var_dim.set(round(self.bg.dim * 100.0))   # 被夹到 0~90 后回显真实值
        finally:
            self._dim_busy = False
        self._save_cfg(bg_dim=round(self.bg.dim, 3))

    def _on_dim(self, value=None):
        if self._dim_busy or not self.bg.active:
            return
        try:
            v = float(self.var_dim.get() if value is None else value)
        except (TypeError, ValueError):
            return
        self.bg.set_dim(v / 100.0)

    @staticmethod
    def _auto_dim(wp):
        """按图片亮度自动挑压暗值,保证曲线还看得清(目标亮度约 96/255)。"""
        try:
            if wp.pillow:
                gray = wp.frames[0].convert("L")
                gray = gray.resize((32, 32))
                px = list(gray.getdata())
                mean = float(sum(px)) / max(1, len(px))
                if mean > 1.0:
                    return max(0.0, min(0.8, 1.0 - 96.0 / mean))
        except Exception:
            pass
        return BG_DIM

    def _show_dim_control(self, show):
        try:
            if show:
                self.dim_wrap.pack(side="left", padx=(20, 0))
            else:
                self.dim_wrap.pack_forget()
        except Exception:
            pass

    def _sync_bg_canvas(self):
        """把最底层画布铺满窗口(有图时),否则藏起来。"""
        if self.bg.active:
            self.bg_canvas.place(x=0, y=0, relwidth=1, relheight=1)
            try:      # Canvas.lower 被 Tk 当成 tag_lower 了,这里显式调窗口的 lower
                self.tk.call("lower", self.bg_canvas._w)
            except tk.TclError:
                pass
            self.after_idle(self._paint_root_bg)
        else:
            self.bg_canvas.delete(BG_TAG)
            self.bg_canvas.place_forget()

    def _paint_root_bg(self):
        if self.bg.active:
            self.bg.paint(self.bg_canvas)

    def _on_root_configure(self, event):
        if event.widget is self and self.bg.active:
            self.after_idle(self._paint_root_bg)
            self._sync_glass()

    # ---------- 主题 ----------
    def _on_theme_choice(self):
        name = self.var_theme.get()
        if name == "自定义…":
            color = colorchooser.askcolor(title="选一个背景色", color=self.pal.get("bg"))[1]
            if color:
                self._apply_theme("自定义", _custom_pal(color))
                self._save_cfg(theme="自定义", custom_bg=color)
            else:
                self.var_theme.set(self.cfg.get("theme", "深色"))
        else:
            self._apply_theme(name)

    def _apply_theme(self, name, pal=None):
        if pal is None:
            pal = THEMES.get(name, THEMES["深色"])
        self.pal = pal
        self.var_theme.set(name)
        if name != "自定义":
            self._save_cfg(theme=name)

        s = self.style
        self.configure(bg=pal["bg"])
        try:      # 背景画布垫在最底层,底色跟着主题走,免得铺图前一瞬间闪白
            self.bg_canvas.configure(bg=pal["bg"])
        except Exception:
            pass
        if self._glass_cv is not None:
            try:
                self._glass_cv.configure(bg=pal["bg"])
            except Exception:
                pass
        s.configure(".", background=pal["bg"], foreground=pal["fg"], font=self.font)
        s.configure("Head.TFrame", background=pal["panel"])
        s.configure("Title.TLabel", background=pal["panel"], foreground=pal["accent"], font=self.font_big)
        s.configure("Muted.TLabel", background=pal["bg"], foreground=pal["muted"], font=self.font)
        s.configure("TFrame", background=pal["bg"])
        s.configure("TLabelframe", background=pal["bg"], foreground=pal["fg"])
        s.configure("TLabelframe.Label", background=pal["bg"], foreground=pal["muted"])
        s.configure("TLabel", background=pal["bg"], foreground=pal["fg"])
        s.configure("TCheckbutton", background=pal["bg"], foreground=pal["fg"])
        s.configure("TButton", background=pal["panel"], foreground=pal["fg"], padding=(8, 4))
        s.map("TButton", background=[("active", pal["sel"])])
        # 「背景图 ▾」是 Menubutton,clam 里它不跟着 TButton 变色,要单独配,
        # 否则某些环境(尤其打包后的 exe)会变成白底白字,看着像没这个按钮。
        s.configure("TMenubutton", background=pal["panel"], foreground=pal["fg"],
                    arrowcolor=pal["fg"], padding=(8, 4))
        s.map("TMenubutton",
              background=[("active", pal["sel"]), ("pressed", pal["sel"])],
              foreground=[("active", pal["fg"]), ("pressed", pal["fg"])],
              arrowcolor=[("active", pal["fg"]), ("pressed", pal["fg"])])
        s.configure("TEntry", fieldbackground=pal["entry"], foreground=pal["fg"],
                    insertcolor=pal["fg"])
        s.map("TEntry",
              fieldbackground=[("!disabled", pal["entry"]), ("disabled", pal["panel"])],
              foreground=[("!disabled", pal["fg"]), ("disabled", pal["muted"])])
        s.configure("TCombobox", background=pal["entry"], fieldbackground=pal["entry"],
                    foreground=pal["fg"], arrowcolor=pal["fg"],
                    selectbackground=pal["entry"], selectforeground=pal["fg"])
        s.map("TCombobox",
              fieldbackground=[("readonly", pal["entry"]), ("!readonly", pal["entry"]),
                               ("active", pal["entry"]), ("focus", pal["entry"])],
              background=[("readonly", pal["entry"]), ("!readonly", pal["entry"]),
                          ("active", pal["entry"]), ("pressed", pal["sel"])],
              foreground=[("readonly", pal["fg"]), ("!readonly", pal["fg"])],
              selectbackground=[("readonly", pal["entry"])],
              selectforeground=[("readonly", pal["fg"])])
        s.map("TCombobox",
              fieldbackground=[("readonly", pal["entry"]), ("!readonly", pal["entry"]),
                               ("active", pal["entry"]), ("focus", pal["entry"])],
              foreground=[("readonly", pal["fg"]), ("!readonly", pal["fg"])],
              selectbackground=[("readonly", pal["entry"])],
              selectforeground=[("readonly", pal["fg"])])
        s.configure("TCombobox", fieldbackground=pal["entry"], background=pal["panel"],
                    foreground=pal["fg"], arrowcolor=pal["fg"])
        self.option_add("*TCombobox*Listbox.background", pal["entry"])
        self.option_add("*TCombobox*Listbox.foreground", pal["fg"])
        s.configure("Treeview", background=pal["panel"], fieldbackground=pal["panel"],
                    foreground=pal["fg"], rowheight=22)
        s.configure("Treeview.Heading", background=pal["panel"], foreground=pal["accent"])
        s.map("Treeview", background=[("selected", pal["sel"])])

        # 主窗口的数据区是自绘的(壁纸透出),颜色在这儿刷新
        self.table.set_theme(pal)
        self.log.set_theme(pal)
        self.badge.configure(background=pal["panel"], foreground=pal["muted"])
        self._echo(f"[主题] {name}")

    # ---------- 端口 ----------
    def _auto_open_first(self):
        """启动后自动打开第一个真实串口,避免用户以为“没数据”。"""
        try:
            if not getattr(self, "readers", None):
                return
            if self.readers[0] is not None:
                return
            port = self.port_vars[0].get().strip()
            if port:
                self._toggle(0)
        except Exception:
            self._log_exc("_auto_open_first")

    def _refresh_ports(self, silent=False):
        real = list_serial_ports()
        ports = real + [p for p in CONFIG["default_ports"] if p not in real]
        for i, cbx in enumerate(self.port_cbs):
            cur = self.port_vars[i].get()
            cbx.config(values=ports)
            if cur not in real:
                # 当前串口(如已拔掉的 COM23)不在系统里:优先切到第一个真实串口
                if real:
                    self.port_vars[i].set(real[0])
                elif cur not in ports:
                    self.port_vars[i].set(ports[i] if i < len(ports) else "")
        if not silent:
            self._echo(f"[刷新] 发现 {len(real)} 个串口:" + (", ".join(real) or "(无)"))

    def _auto_match(self):
        if type(self)._scanning:
            self._echo("[自动匹配] 正在扫描中,请等它出结果(别重复点)…")
            return
        if not HAS_SERIAL:
            messagebox.showwarning("缺少依赖", "未安装 pyserial,无法自动匹配。\n"
                                               "Ubuntu:  pip3 install pyserial\nWindows: pip install pyserial")
            return
        for idx in list(self.readers):
            self._toggle(idx)
        type(self)._scanning = True
        self._scan_start = time.time()
        self.var_status.set("自动匹配中(请稍候)…")
        threading.Thread(target=self._scan_worker, daemon=True).start()

    def _probe_one_port(self, p, found, lock):
        """(?^口) 一个端口的全部波特率 —— 每个端口一个线程,并行扫描 ✓"""
        for b in CONFIG["baud_list"]:
            self.q.put(("log", "[自动匹配] 试 %s @ %s …" % (p, b)))
            kind = probe_port(p, b)
            if kind:
                with lock:
                    found.setdefault(kind, []).append((p, b))
                label = "DEBUG(0x5A)" if kind == "DEBUG" else "A5"
                self.q.put(("log", "[自动匹配] %s @ %s -> %s" % (p, b, label)))
                return
        self.q.put(("log", "[自动匹配] %s 无有效帧" % p))

    def _scan_worker(self):
        """扫描**全部**串口(不只是当前选中的那个),w每个口并行试所有波特率 ✓"""
        found = {}
        lock = threading.Lock()
        try:
            ports = list_serial_ports()
        except Exception as exc:
            self.q.put(("log", "[自动匹配] 枚举串口异常: %r" % (exc,)))
            ports = []
        # 枚举失败/超时:Windows 退回 COM1..COM32;Ubuntu 用设备节点兜底
        if not ports:
            if IS_WINDOWS:
                ports = ["COM%d" % i for i in range(1, 33)]
            else:
                ports = _linux_glob_ports()
        self.q.put(("log", "[自动匹配] 扫描全部 %d 个串口: %s"
                    % (len(ports), ", ".join(ports) if ports else "(无)")))
        self.q.put(("log", "[自动匹配] 每个口并行试 %d 种波特率(最长约 %d 秒)"
                    % (len(CONFIG["baud_list"]), int(len(CONFIG["baud_list"]) * 1.3) + 2)))
        threads = [threading.Thread(target=self._probe_one_port, args=(p, found, lock), daemon=True)
                   for p in ports]
        for th in threads:
            th.start()
        for th in threads:
            th.join()
        self.q.put(("scan_done", found))

    def _apply_scan(self, found):
        type(self)._scanning = False
        self._scan_start = 0.0
        if not found:
            self._echo("[自动匹配] 没找到带有效帧的端口 —— 检查接线 / 波特率 / 板子是否在发")
            self.var_status.set("自动匹配失败")
            return
        slots = []
        if found.get("DEBUG"):
            slots.append(found["DEBUG"][0])
        if found.get("A5"):
            slots.append(found["A5"][0])
        if not slots:
            slots.append(list(found.values())[0][0])
        for i, (p, b) in enumerate(slots):
            self.port_vars[i].set(p)
            self.baud_vars[i].set(str(b))
            self.tag_lbls[i].config(text=f"已匹配:{p} @ {b}")
        for i in range(len(slots)):
            if i not in self.readers:
                self._toggle(i)
        self.var_status.set("自动匹配完成")
        self._echo("[自动匹配] 完成,已自动打开")

    def _toggle(self, idx):
        # ---- TOGGLE TRACE:任何异常都写进日志面板(窗口版 exe 看不到 stderr)----
        try:
            self._toggle_inner(idx)
        except Exception as exc:
            import traceback
            self._echo("[异常] _toggle 失败:" + repr(exc))
            for ln in traceback.format_exc().splitlines()[-4:]:
                self._echo("      " + ln)

    def _toggle_inner(self, idx):
        self._echo("[点击] 端口 %d 按钮,当前状态 = %s,扫描中 = %s"
                   % (idx + 1, "已连接" if idx in self.readers else "未连接",
                      "是" if type(self)._scanning else "否"))
        if type(self)._scanning:
            self._echo("[提示] 自动匹配正在进行,等它结束后再手动开关端口 ✓")
            return

        # ★ 端口在不在?(插拔/掉线最常见)⇒ 明确说出来,别静默失败 ✗
        port_now = self.port_vars[idx].get().strip()
        avail = list_serial_ports()
        # avail 为空时(打包版枚举失败)不要直接拒绝,允许手填 COMx 试开;
        # 真正的打开错误会在 Reader 线程里显示出来。
        if avail and port_now not in avail:
            self._echo("[错误] 端口 %s 不存在(现在系统里有:%s)⇒ 重插 USB-TTL,然后点「刷新端口」"
                       % (port_now or "(空)", ", ".join(avail) if avail else "一个都没有"))
            self.tag_lbls[idx].config(text="端口不存在,请重插后「刷新端口」")
            return
        if idx in self.readers:
            self.readers.pop(idx).stop()
            self.decoders.pop(idx, None)
            self.btn_vars[idx].config(text="打开")
            self.tag_lbls[idx].config(text="未连接")
            self._echo(f"[关闭] 端口 {idx + 1}")
            return
        port, baud = self.port_vars[idx].get().strip(), self.baud_vars[idx].get().strip()
        if not port:
            messagebox.showwarning("提示", "先选一个串口")
            return
        if not HAS_SERIAL:
            messagebox.showwarning("缺少依赖", "未安装 pyserial。\nUbuntu: pip3 install pyserial\n"
                                               "Windows: pip install pyserial")
            return
        self.decoders[idx] = FrameDecoder()
        r = Reader(idx, port, int(baud), self.q)
        self.readers[idx] = r
        r.start()
        self.btn_vars[idx].config(text="关闭")
        self.tag_lbls[idx].config(text=f"已连接 {port} @ {baud}")
        self._echo(f"[打开] {port} @ {baud}")

    # ---------- 主循环 ----------
    def _tick(self):
        # 绝不让一次 UI/解析异常把 after 链打断;异常写 app_error.log
        try:
            self._tick_once()
        except Exception:
            self._log_exc("_tick")
        finally:
            self.after(60, self._tick)

    def _tick_once(self):
        # watchdog:自动匹配线程如果卡在 comports()/某个端口上,最多锁 12 秒,
        # 超时强制解除 _scanning,否则手动“打开”永远被拒绝。
        if type(self)._scanning and self._scan_start and (time.time() - self._scan_start > 12.0):
            type(self)._scanning = False
            self._scan_start = 0.0
            self.var_status.set("自动匹配超时,可手动打开")
            self._echo("[自动匹配] 超时,已解除锁定;请直接选串口点「打开」")

        chunks = {}
        try:
            while True:
                msg = self.q.get_nowait()
                if msg[0] == "data":
                    _, idx, data = msg
                    # 同一拍同一端口先攒起来再解析,避免 1 字节 1 字节喂
                    # 把 [LOG] 文本行/二进制帧拆碎。
                    chunks[idx] = chunks.get(idx, b"") + data
                elif msg[0] == "log":
                    self._echo(msg[1])
                elif msg[0] == "scan_done":
                    self._apply_scan(msg[1])
        except queue.Empty:
            pass

        for idx, data in chunks.items():
            dec = self.decoders.get(idx)
            if dec:
                for d, frame in dec.feed(data):
                    if self.recording.get():
                        self.recorder.add(self.port_vars[idx].get(),
                                          self.baud_vars[idx].get(), d, frame)
                    self.dbg.append(d)
                # feed() 之后再取 texts:本次新解出的 [LOG] 行要立刻显示
                for txt in dec.texts:
                    self._echo(txt)
                dec.texts.clear()

        for idx, r in list(self.readers.items()):
            if r.error and not r.is_alive():
                self._echo(f"[错误] 端口 {idx + 1}:{r.error}")
                self.readers.pop(idx)
                self.btn_vars[idx].config(text="打开")
                self.tag_lbls[idx].config(text="连接失败")

        if not self.paused.get():
            self._refresh_view()
        if self.dbg:
            self._feed_vofa_from_frame(self.dbg[-1])
        # 电机 / 底盘 / 云台窗口开着就推一帧过去
        if self.dbg:
            for win in (self.motor_win, self.chassis_win, self.gimbal_win):
                if win is not None and win.winfo_exists():
                    try:
                        if win is self.motor_win:
                            win.push(self.dbg[-1])
                        else:
                            win.feed(self.dbg[-1])
                    except Exception:
                        pass

    _OSPI_STATE = {0: "RESET", 1: "READY", 2: "忙(写)", 4: "忙(读)", 8: "忙(轮询)",
                   0x10: "忙(内存映射)", 0x20: "忙(Abort)"}
    _HAL_RET = {0: "OK", 1: "ERROR", 2: "BUSY", 3: "TIMEOUT"}

    @staticmethod
    def _flash_line(row):
        """把 OSPI 状态/错误码/命令返回值翻译成人话,一眼看出卡在哪一层"""
        st = row.get("flash_state", 0)
        er = row.get("flash_err", 0)
        c = row.get("flash_cmd", 0xFFFFFFFF)
        r = row.get("flash_rcv", 0xFFFFFFFF)
        raw = row.get("flash_raw", 0)

        if er == 0 and c == 0 and r == 0 and raw not in (0, 0xFFFFFF):
            return "JEDEC=0x%06X  正常 ✓" % raw
        return ("State=%s  Error=0x%02X  Command=%s  Receive=%s  原始ID=0x%06X"
                % (App._OSPI_STATE.get(st, "0x%X" % st), er,
                   App._HAL_RET.get(c, "0x%X" % c), App._HAL_RET.get(r, "0x%X" % r), raw))

    _prev_ticks = [0] * 8

    @classmethod
    def _tick_line(cls, row, base, names):
        """任务心跳:和上次刷新比,不涨的那个就是卡住(⚠)"""
        parts = []
        for i, nm in enumerate(names):
            v = row.get("tick%d" % (base + i), 0)
            prev = cls._prev_ticks[base + i]
            cls._prev_ticks[base + i] = v
            parts.append("%s=%d%s" % (nm, v, "⚠没涨" if v == prev else ""))
        return "   ".join(parts)

    @staticmethod
    def _log_line(row):
        """日志上电链:boot 编号 / 本次首地址 / 上次首尾地址 / 回绕次数"""
        def a(v):
            return "(无)" if v == 0xFFFFFFFF or v == 0 else f"0x{v:06X}"
        return ("boot=%d   本次首=%s   上次=%s~%s   回绕=%d" %
                (row.get("log_boot", 0), a(row.get("log_cur_start", 0)),
                 a(row.get("log_prev_start", 0)), a(row.get("log_prev_stop", 0)),
                 row.get("log_wrap", 0)))

    @staticmethod
    def _hwm_line(row, keys, names):
        """把栈水位排成"名字=值",并对 <=8 的加警告(越小越危险)"""
        parts = []
        for k, nm in zip(keys, names):
            v = row.get(k, 0)
            parts.append(f"{nm}={v}" + ("⚠" if v <= 8 else ""))
        return "   ".join(parts)

    @staticmethod
    def _fmt_field_value(v):
        if isinstance(v, (int, float)):
            f = float(v)
            if abs(f - round(f)) < 1e-9:
                return str(int(round(f)))
            return f"{f:+.4f}"
        return str(v)

    def _group_field_rows(self, fields):
        labels = {
            "m202": "0x202 舵向",
            "m201": "0x201 驱动",
            "imu": "IMU",
            "rc": "遥控",
            "photo": "光电",
            "err": "错误",
            "debug": "调试",
            "can": "CAN 扫描",
            "test": "验证",
        }
        groups = collections.OrderedDict()
        for key, value in (fields or {}).items():
            if ". " in key:
                continue
            if "." in key:
                prefix, short = key.split(".", 1)
            else:
                prefix, short = key, key
            groups.setdefault(prefix, []).append((short, value))

        rows = []
        for prefix, vals in groups.items():
            label = labels.get(prefix, prefix)
            parts = []
            for short, value in vals:
                parts.append(f"{short}={self._fmt_field_value(value)}")
            rows.append((label, "   ".join(parts)))
        return rows

    def _refresh_view(self):
        pid = [r for r in self.recorder.rows if r.get("proto") == "MOTOR_PID"]
        last = pid[-1] if pid else None
        if last is None and self.dbg and self.dbg[-1].get("proto") == "MOTOR_PID":
            last = self.dbg[-1]
        if last is not None:
            flags = last.get("flags", 0)
            ok = bool(flags & 2)
            self.badge.configure(text="  OK  " if ok else "  MOTOR  ",
                                 background="#1d7a3a" if ok else "#8a4b12",
                                 foreground="#ffffff")
            state = ",".join(n for bit, n in MOTOR_FLAG if flags & bit) or "-"
            # 动态字段:按前缀合并成一组,例如 IMU 一行显示 roll/pitch/yaw/gz
            self._feed_vofa_from_frame(last)
            fields = self.vofa_last if isinstance(self.vofa_last, dict) else {}
            items = self._group_field_rows(fields)
            if not items:
                items = [("字段", "等待数据…")]
            self.table.set_rows(items)
            st = self._stats()
            self.var_status.set(f"帧: MOTOR_PID {st['DEBUG']} / 0xA5 {st['A5']}   "
                                f"CRC 错 {st['crc_err']}   记录 {len(self.recorder.rows)} 帧 / "
                                f"原始 {len(self.recorder.raw_rows)} 帧")
            return

        dbg = [r for r in self.recorder.rows if r.get("proto") == "DEBUG"]
        if dbg:
            last = dbg[-1]
            lv = last["err_level"]
            self.badge.configure(text=f"  {LEVEL_NAME.get(lv, lv)}  ",
                                 background=LEVEL_COLOR.get(lv, "#666"), foreground="#ffffff")
            items = [
                ("时间 / 串口", f"{last['time']}    {last['port']}"),
                ("错误码", f"0x{last['err_code']:04X}"),
                ("故障码", f"{FAULT_NAME.get(last['fault_code'], last['fault_code'])}   "
                          f"CFSR=0x{last['fault_cfsr']:08X}   PC=0x{last['fault_pc']:08X}"),
                ("IMU", f"roll={last['roll']:+.3f}  pitch={last['pitch']:+.3f}  yaw={last['yaw']:+.3f} rad   "
                          f"温度={last.get('temp', 0.0):.1f} ℃(走加速度 die ⇒ 判断加速度通路)"),
                ("诊断① Flash/IMU", (lambda f: f"Flash={('已禁用' if (f & 1) and last.get('jedec',0) == 0 else ('✓' if f & 1 else '✗'))} "
                                                  f"JEDEC=0x{last.get('jedec',0):06X}(0xEF4017=正常;0=已禁用或读失败)   "
                                                  f"IMU init={'✓' if f & 2 else '✗'} online={'✓' if f & 4 else '✗'}   "
                                                  f"ACC_CONF=0x{last.get('acc_conf',0):X}(应 0xA8)   BMI错误={last.get('bmi_err',0)}")(last.get("flags", 0))),
                ("诊断② 外设/心跳", (lambda f: f"WS2812={'✓' if f & 8 else '✗'} 蜂鸣器={'✓' if f & 16 else '✗'} 日志={'✓' if f & 32 else '✗'}   "
                                                  f"error心跳={last.get('err_tick',0)} "
                                                  f"{'✓ 在涨' if last.get('err_tick',0) != self._prev_tick else '✗ 没涨(任务卡住)'}"
                                                  f"(Δ={last.get('err_tick',0) - self._prev_tick})")(last.get("flags", 0))),
                ("加速度", (f"ax={last['ax']:+.3f}  ay={last['ay']:+.3f}  az={last['az']:+.3f} m/s²")
                          if last.get("has_gyro") else "—"),
                ("外接IMU 达妙DM-IMU-L1", (f"gyro={last.get('ext_gx',0):+.3f} {last.get('ext_gy',0):+.3f} {last.get('ext_gz',0):+.3f} rad/s   "
                                          f"accel={last.get('ext_ax',0):+.2f} {last.get('ext_ay',0):+.2f} {last.get('ext_az',0):+.2f} m/s²")),
                ("栈水位① 单位:字(4B)", self._hwm_line(last, ["hwm_motor", "hwm_robot", "hwm_error", "hwm_cmd"],
                                                      ["电机", "robot", "error", "cmd"])
                 if last.get("has_hwm") else "— 旧固件(210B)无栈水位"),
                ("栈水位② 越小越危险", self._hwm_line(last, ["hwm_imu", "hwm_chassis", "hwm_gimbal", "hwm_highRise"],
                                                      ["imu", "chassis", "gimbal", "highRise"])
                 if last.get("has_hwm") else "—"),
                ("日志 上电链", self._log_line(last) if last.get("has_log") else "— 旧固件无日志上电链"),
                ("任务心跳① motor/robot/error/cmd", self._tick_line(last, 0, ["motor", "robot", "error", "cmd"])),
                ("任务心跳② imu/chassis/gimbal/highRise", self._tick_line(last, 4, ["imu", "chassis", "gimbal", "highRise"])),
                ("底盘", f"vx={last['vx']:+.3f}  vy={last['vy']:+.3f}  w={last['w']:+.3f}"),
                ("云台", f"yaw={last['gimbal_yaw']:+.3f}  pitch={last['gimbal_pitch']:+.3f} rad"),
            ]
            for m in last["motors"]:
                st = ",".join(n for bit, n in MOTOR_FLAG if m["flags"] & bit) or "-"
                items.append((f"电机 {m['id']} [{st}]",
                              f"pos={m['pos']:+.4f}  w={m['w']:+.3f}  t={m['t']:+.4f}"))
            self.table.set_rows(items)

        st = self._stats()
        self.var_status.set(f"帧: 调试 {st['DEBUG']} / 0xA5 {st['A5']}    "
                            f"CRC 错 {st['crc_err']}    丢弃字节 {st['reject']}(帧间噪声,正常)    "
                            f"记录 {len(self.recorder.rows)} 帧 / "
                            f"原始 {len(self.recorder.raw_rows)} 帧")
        self.var_rate.set("暂停显示中(仍在记录)" if self.paused.get() else "")

    # ---------- 发送 ----------
    def _send(self):
        txt = self.var_hex.get().strip().replace(" ", "")
        if not txt:
            return
        try:
            payload = bytes.fromhex(txt)
        except ValueError:
            messagebox.showerror("格式错误", "HEX 格式不对,例:A5 1C 00 07 12 01 00 ...")
            return
        idx = 0 if self.var_slot.get() == "端口 1" else 1
        port = self.port_vars[idx].get()
        if not HAS_SERIAL or not port:
            messagebox.showwarning("提示", "未连接端口")
            return
        try:
            with serial.Serial(port, int(self.baud_vars[idx].get()), timeout=0.2) as ser:
                ser.write(payload)
            self._echo(f"[发送] {port} ← {len(payload)} 字节:{payload.hex(' ').upper()}")
        except Exception as exc:
            self._echo(f"[发送失败] {exc}")

    # ---------- 导出 / 文件 / 清空 / 日志 ----------
    def _stats(self):
        st = {"DEBUG": 0, "A5": 0, "crc_err": 0, "reject": 0}
        for dec in self.decoders.values():
            for k in st:
                st[k] += dec.stats.get(k, 0)
        return st

    def _export(self, kind):
        if not self.recorder.rows:
            messagebox.showinfo("提示", "还没有收到任何帧")
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if kind == "csv":
            path = filedialog.asksaveasfilename(initialdir=_export_dir(),
                                                defaultextension=".csv",
                                                initialfile=f"mcu_log_{stamp}.csv",
                                                filetypes=[("CSV", "*.csv")])
            if path:
                self.recorder.export_csv(path)
                self._echo(f"[导出] CSV → {path}")
        else:
            path = filedialog.asksaveasfilename(initialdir=_export_dir(),
                                                defaultextension=".md",
                                                initialfile=f"mcu_report_{stamp}.md",
                                                filetypes=[("Markdown", "*.md")])
            if path:
                self.recorder.export_md(path, self._stats())
                self._echo(f"[导出] 文档 → {path}")

    def _export_raw(self, kind):
        if not self.recorder.raw_rows:
            messagebox.showinfo("提示", "还没有记录到原始帧")
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if kind == "csv":
            path = filedialog.asksaveasfilename(initialdir=_export_dir(),
                                                defaultextension=".csv",
                                                initialfile=f"mcu_raw_{stamp}.csv",
                                                filetypes=[("CSV", "*.csv")])
            if path:
                self.recorder.export_raw_csv(path)
                self._echo(f"[导出] 原始 CSV → {path}")
        else:
            path = filedialog.asksaveasfilename(initialdir=_export_dir(),
                                                defaultextension=".txt",
                                                initialfile=f"mcu_raw_{stamp}.txt",
                                                filetypes=[("HEX 文本", "*.txt")])
            if path:
                self.recorder.export_raw_hex(path)
                self._echo(f"[导出] 原始 HEX → {path}")

    def _open_motor_debug(self):
        if self.motor_win is not None and self.motor_win.winfo_exists():
            self.motor_win.lift()
            return
        self.motor_win = MotorDebugWindow(self)

    def _feed_vofa_from_frame(self, d):
        if not isinstance(d, dict):
            return
        if d.get("proto") == "STEER":
            # 舵向实时角度帧(500Hz):并入持久字段表
            fields = getattr(self, "last_fields", {})
            for k in ("steer0.now", "steer0.tgt", "steer1.now", "steer1.tgt"):
                if k in d:
                    fields[k] = d[k]
            self.last_fields = fields
            return self._group_field_rows(fields)
        if d.get("proto") != "MOTOR_PID":
            return
        fields = {}
        motors = d.get("motors") or []
        if motors:
            m = motors[0]
            fields["m202.online"] = 1.0 if m.get("online") else 0.0
            fields["m202.pos"] = float(m.get("pos", 0.0))
            fields["m202.pos_fb"] = float(m.get("pos_fb", m.get("pos_raw", 0.0)))
            fields["m202.w"] = float(m.get("w", 0.0))
            fields["m202.t"] = float(m.get("t", 0.0))
            fields["m202.target"] = float(d.get("pid_target_rpm", 0.0))
            fields["m202.pos_target"] = float(d.get("pos_target", 0.0))
            fields["m202.pos_error"] = float(d.get("pos_error", 0.0))
        if len(motors) > 1:
            m = motors[1]
            fields["m201.online"] = 1.0 if m.get("online") else 0.0
            fields["m201.pos"] = float(m.get("pos", 0.0))
            fields["m201.w"] = float(m.get("w", 0.0))
            fields["m201.t"] = float(m.get("t", 0.0))

        if len(motors) > 2:
            m = motors[2]
            fields["m203.online"] = 1.0 if m.get("online") else 0.0
            fields["m203.pos"] = float(m.get("pos", 0.0))
            fields["m203.w"] = float(m.get("w", 0.0))
            fields["m203.t"] = float(m.get("t", 0.0))
        if len(motors) > 3:
            m = motors[3]
            fields["m204.online"] = 1.0 if m.get("online") else 0.0
            fields["m204.pos"] = float(m.get("pos", 0.0))
            fields["m204.pos_fb"] = float(m.get("pos_fb", m.get("pos_raw", 0.0)))
            fields["m204.w"] = float(m.get("w", 0.0))
            fields["m204.t"] = float(m.get("t", 0.0))
            fields["m204.target"] = float(m.get("target", 0.0))
            fields["m204.pos_target"] = float(m.get("pos_target", 0.0))
            fields["m204.pos_error"] = float(m.get("pos_error", 0.0))

        # 上 MCU 的 dm4310/207/208/can3/rc 只在 JSON 里,固定帧没有;
        # 合并最近一帧 [LOG] JSON,让字段表也能显示
        _last = getattr(self, "last_json", None)
        if isinstance(_last, dict):
            for _k, _v in _last.items():
                fields[_k] = _v
        # 下板([L])的字段:只补上板没有的键,避免互相覆盖
        _low = getattr(self, "last_json_l", None)
        if isinstance(_low, dict):
            for _k, _v in _low.items():
                fields.setdefault(_k, _v)
        if isinstance(_last, dict):
            for _k in ("m204.target", "m204.pos_target", "m204.pos_error",
                       "m206.online", "m206.w", "m206.target",
                       "m6020.online", "m6020.w", "m6020.target",
                       "m205.online", "m205.w", "m205.target"):
                if _k in _last:
                    fields[_k] = _last[_k]

        fields["imu.roll"] = float(d.get("roll", 0.0))
        fields["imu.pitch"] = float(d.get("pitch", 0.0))
        fields["imu.yaw"] = float(d.get("yaw", 0.0))
        fields["imu.gz"] = float(d.get("gz", 0.0))
        fields["rc.online"] = 1.0 if d.get("rc_online") else 0.0
        for i, v in enumerate(d.get("rc_ch") or []):
            fields[f"rc.ch{i}"] = float(v)
        fields["err.level"] = float(d.get("err_level", 0))
        fields["err.code"] = float(d.get("err_code", 0))
        fields["photo.pe2"] = float(d.get("pe2_level", 0))
        fields["photo.pe3"] = float(d.get("pe3_level", 0))
        fields["debug.rx_bytes"] = 0.0
        fields["debug.cmd_count"] = 0.0
        # 自描述验证字段:不依赖固件,帧到来就自增
        self._test_counter = getattr(self, "_test_counter", 0) + 1
        fields["test.counter"] = float(self._test_counter)
        self.vofa_last = fields
        vw = getattr(self, "vofa_win", None)
        # 不再自动弹出 VOFA 窗口;开着才喂数据
        if vw is not None and vw.winfo_exists():
            vw.feed(fields)
        sw = getattr(self, "steer_win", None)
        if sw is not None and sw.winfo_exists():
            sw.feed(fields)

    def _open_vofa(self):
        if self.vofa_win is not None and self.vofa_win.winfo_exists():
            self.vofa_win.lift()
            return
        self.vofa_win = VofaWindow(self)

    def _open_steer_debug(self):
        if self.steer_win is not None and self.steer_win.winfo_exists():
            self.steer_win.lift()
            return
        self.steer_win = SteerDebugWindow(self)

    def _open_chassis_debug(self):
        if self.chassis_win is not None and self.chassis_win.winfo_exists():
            self.chassis_win.lift()
            return
        self.chassis_win = ChassisDebugWindow(self)

    def _open_gimbal_debug(self):
        if self.gimbal_win is not None and self.gimbal_win.winfo_exists():
            self.gimbal_win.lift()
            return
        self.gimbal_win = GimbalDebugWindow(self)

    # ---------- 调试命令(0x5C):底盘/云台窗口共用 ----------
    def send_debug_cmd(self, cmd, payload=b"", tag="调试"):
        self.cmd_seq = (self.cmd_seq + 1) & 0xFFFF
        frame = bytes([0x5C, cmd, (self.cmd_seq >> 8) & 0xFF, self.cmd_seq & 0xFF]) + payload
        return self.send_raw(frame, tag=tag)

    def send_raw(self, data, tag="发送"):
        w = self.readers.get(0)
        if w is None:
            self._echo("[%s] 端口 1 没打开,发不出去" % tag)
            return False
        try:
            w.write(data)
            self._echo("[%s] 发送 %s" % (tag, data.hex(" ").upper()))
            return True
        except Exception as exc:
            self._echo("[%s] 发送失败: %s" % (tag, exc))
            return False

    def _open_file(self):
        path = filedialog.askopenfilename(filetypes=[("数据文件", "*.bin *.log *.raw *.txt"),
                                                     ("全部", "*.*")])
        if not path:
            return
        with open(path, "rb") as fp:
            data = fp.read()
        dec = FrameDecoder()
        self.decoders[-1] = dec
        n = 0
        for d, frame in dec.feed(data):
            self.recorder.add(os.path.basename(path), "-", d, frame)
            n += 1
        self._echo(f"[离线] {os.path.basename(path)}:{len(data)} 字节 → {n} 帧")
        self._refresh_view()

    def _clear(self):
        self.recorder = Recorder()
        for dec in self.decoders.values():
            dec.stats = {"DEBUG": 0, "A5": 0, "crc_err": 0, "reject": 0}
        self.table.set_rows([])
        self.badge.configure(text="  --  ", background=self.pal["panel"], foreground=self.pal["muted"])
        self._echo("[清空]")

    def _log_exc(self, where, exc=None, val=None, tb=None):
        """把异常写到 exe 同目录的 app_error.log(打包版看不到 stderr)。"""
        try:
            if exc is None:
                text = traceback.format_exc()
            else:
                text = "".join(traceback.format_exception(exc, val, tb))
            if IS_LINUX:
                path = os.path.join(os.path.expanduser("~"), "调试助手_error.log")
            else:
                if getattr(sys, "frozen", False):
                    base = os.path.dirname(sys.executable)
                else:
                    base = os.path.dirname(os.path.abspath(__file__))
                path = os.path.join(base, "app_error.log")
            with open(path, "a", encoding="utf-8") as fp:
                fp.write("\n=== %s %s ===\n" % (datetime.now().strftime("%Y-%m-%d %H:%M:%S"), where))
                fp.write(text)
            self._echo("[异常] %s: %s" % (where, text.strip().splitlines()[-1]))
        except Exception:
            pass

    def _report_cb_exception(self, exc, val, tb):
        self._log_exc("tk_callback", exc, val, tb)

    def _echo(self, text):
        # 上/下 MCU 的 JSON 前缀:[U]=上板自己,[L]=下板经 485 透传上来
        _src = "U"
        if text.startswith("[U] "):
            text = text[4:]
        elif text.startswith("[L] "):
            _src = "L"
            text = text[4:]
        if text.startswith("[LOG] JSON "):
            try:
                data = json.loads(text[11:])
                if _src == "L":
                    self.last_json_l = data
                else:
                    # 上板 JSON 里给下板留的电机键(m201~m206/m6020)全是 0,丢掉,
                    # 免得字段表里又冒出下板的空电机位
                    data = {k: v for k, v in data.items()
                            if not k.startswith(("m201.", "m202.", "m203.", "m204.",
                                                 "m205.", "m206.", "m6020."))}
                    self.last_json = data
                self.vofa_last = data
                # VOFA 窗口不再自动弹:只有点「VOFA 波形」或已在开着才喂数据
                vw = getattr(self, "vofa_win", None)
                if vw is not None and vw.winfo_exists():
                    vw.feed(data)
                sw = getattr(self, "steer_win", None)
                if sw is not None and sw.winfo_exists():
                    sw.feed(data)
            except Exception:
                pass
            return

        mw = getattr(self, "motor_win", None)
        if "[LOG] PID" in text and mw is not None and mw.winfo_exists():
            try:
                mw.feed_pid_text(text)
            except Exception:
                pass
        sw = getattr(self, "steer_win", None)
        if "[LOG] PID" in text and sw is not None and sw.winfo_exists():
            try:
                sw.feed_pid_text(text)
            except Exception:
                pass
        self.log.add(text)

    def on_close(self):
        for r in self.readers.values():
            r.stop()
        self.destroy()

    def destroy(self):
        try:
            self._close_glass()     # 先收掉背后的壁纸窗口
        except Exception:
            pass
        try:
            self.bg.stop()          # 先停动图,别让 after 回调在窗口没了以后还跑
        except Exception:
            pass
        super().destroy()


def main():
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()


if __name__ == "__main__":
    main()
