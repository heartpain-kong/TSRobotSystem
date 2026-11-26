# -*- coding: utf-8 -*-
"""0x5A / 0xA5 协议解析 + 记录导出(Android 版共享层)。

★ 这个文件是自动生成的,不要手改 ★
源文件:调试助手.py
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
CONFIG = {"csv_encoding": "utf-8-sig", "platform": "Android"}

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
                   "pos_raw", "pos_target_raw", "pos_error_raw",
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


# ---- 生成信息(不要手改)----
SOURCE = {"file": '调试助手.py', "sha256": '06bb7f8341d8fba183effb06219f84c73325aa09eb0be0e3eb1879ba4d95b466'}
