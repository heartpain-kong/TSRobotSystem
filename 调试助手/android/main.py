# -*- coding: utf-8 -*-
"""调试助手 · Android(Kivy)版 —— STM32 机器人串口调试

和桌面版 调试助手.py 用同一套协议(0x5A 遥测 / 0xA5 上位机 / 0x5C 命令),
协议解析直接在 proto.py 里,由 tools/sync_proto.py 从桌面版生成,不另写一份。

页面:
    字段    最后一帧解码出来的关键量(电机/IMU/遥控/IM6014/错误)
    曲线    速度、力矩、位置、光电 四张小图,和桌面版曲线对应
    日志    收到的 [LOG] 文本 + 操作提示,导出按钮也在这里
    命令    原始 HEX 发送 + 0x5C 速度/角度/PID/停止

用法:
    android/ 下 `python main.py`              桌面预览(用 pyserial 的 COMx)
    android/ 下 `python main.py --demo`       无硬件,合成帧演示
    Linux/WSL 里 `buildozer -v android debug`  打 APK(见 README)

真机要点:
    · 手机/平板要支持 USB Host(OTG),用 OTG 线接 CH340/CP2102 等;
    · 首次打开串口会弹「允许访问 USB 设备」,必须点允许;
    · CH340 最高 921600,4 Mbps 只有 CDC ACM 才可能跑得动。
"""
import collections
import math
import os
import queue
import struct
import sys
import threading
import time

from kivy.config import Config

try:                                  # 桌面预览时给个手机比例的窗口
    Config.set("graphics", "width", "480")
    Config.set("graphics", "height", "880")
    Config.set("kivy", "exit_on_escape", "1")
except Exception:
    pass

from kivy.app import App
from kivy.clock import Clock
from kivy.core.text import LabelBase
from kivy.core.window import Window
from kivy.graphics import Color, Line, Rectangle
from kivy.metrics import dp, sp
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.popup import Popup
from kivy.uix.scrollview import ScrollView
from kivy.uix.spinner import Spinner
from kivy.uix.textinput import TextInput
from kivy.uix.togglebutton import ToggleButton
from kivy.uix.widget import Widget
from kivy.utils import get_color_from_hex

import demo
import proto
import serial_link

# ---------------------------------------------------------------- 配色 / 字体
BG = get_color_from_hex("#12161d")
PANEL = get_color_from_hex("#1c2330")
PANEL2 = get_color_from_hex("#27303f")
FG = get_color_from_hex("#e6ecf3")
MUTED = get_color_from_hex("#8a97a8")
ACCENT = get_color_from_hex("#2f9e6e")
WARN = get_color_from_hex("#e0a030")
BAD = get_color_from_hex("#d64545")

CJK_FONTS = (
    # Android(系统自带 Noto/Droid 中文)
    "/system/fonts/NotoSansCJK-Regular.ttc",
    "/system/fonts/NotoSansSC-Regular.otf",
    "/system/fonts/NotoSansCJKsc-Regular.otf",
    "/system/fonts/DroidSansFallback.ttf",
    "/system/fonts/DroidSansFallbackFull.ttf",
    "/system/fonts/NotoSansCJK.ttc",
    # Windows 预览
    "C:/Windows/Fonts/msyh.ttc",
    "C:/Windows/Fonts/msyhbd.ttc",
    "C:/Windows/Fonts/simhei.ttf",
    "C:/Windows/Fonts/simsun.ttc",
    # Linux 预览
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)


def register_cjk_font():
    """把系统里的中文字体注册成默认字体,否则界面全是方块。"""
    for path in CJK_FONTS:
        if path and os.path.isfile(path):
            try:
                LabelBase.register(name="Roboto", fn_regular=path, fn_bold=path)
                return path
            except Exception:
                continue
    return None


FONT = register_cjk_font()


# ---------------------------------------------------------------- 小工具
def btn(text, on_press=None, color=PANEL2, height=dp(38), width=None, font_size=sp(14)):
    b = Button(text=text, font_size=font_size, color=FG,
               background_normal="", background_color=color,
               size_hint_y=None, height=height)
    if width is not None:
        b.size_hint_x = None
        b.width = width
    if on_press is not None:
        b.bind(on_press=on_press)
    return b


def lab(text="", color=FG, font_size=sp(13), align="left", **kw):
    """单行标签:文字超出宽度会被裁掉,行高固定,便于对齐。"""
    w = Label(text=text, color=color, font_size=font_size,
              halign=align, valign="middle", **kw)
    w.bind(size=lambda l, s: setattr(l, "text_size", (s[0], s[1])))
    return w


def cap(text="", color=MUTED, font_size=sp(11)):
    """固定高度的小标题(竖向布局里不加这个会抢高度,出现大片空白)。"""
    return lab(text, color=color, font_size=font_size,
               size_hint_y=None, height=dp(20))


def ti(text="", hint="", height=dp(38)):
    """深色输入框,和整体配色一致。"""
    return TextInput(text=text, hint_text=hint, multiline=False, font_size=sp(14),
                     size_hint_y=None, height=height,
                     background_normal="", background_active="",
                     background_color=PANEL2, foreground_color=FG,
                     cursor_color=FG, hint_text_color=MUTED,
                     padding=(dp(6), dp(9)))


class Panel(BoxLayout):
    """带底色的容器(canvas.before 画背景,不遮挡子控件)。"""

    def __init__(self, color=PANEL, **kw):
        super().__init__(**kw)
        with self.canvas.before:
            Color(*color)
            self._rect = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self._bg, size=self._bg)

    def _bg(self, *_):
        self._rect.pos = self.pos
        self._rect.size = self.size


def hbar(height=dp(40), padding=dp(4), spacing=dp(5)):
    return BoxLayout(orientation="horizontal", size_hint_y=None, height=height,
                     padding=(padding, 0), spacing=spacing)


def parse_hex(text):
    """'5C 05 00 01' / '5c,05,0001' / '0x5C 0x05' → bytes"""
    t = (text or "").replace(",", " ").replace("0x", " ").replace("0X", " ")
    out = bytearray()
    for part in t.split():
        part = part.strip()
        if not part:
            continue
        if len(part) % 2:
            part = "0" + part
        out += bytes.fromhex(part)
    return bytes(out)


# ---------------------------------------------------------------- 曲线
class Chart(Widget):
    """一条时间序列小图:Kivy 的 Line 只更新 points,不重建 canvas。"""

    def __init__(self, series, **kw):
        super().__init__(**kw)
        self.series = list(series)
        self.data = collections.OrderedDict(
            (name, collections.deque(maxlen=600)) for name, _c in self.series)
        self._lines = collections.OrderedDict()
        with self.canvas:
            for name, color in self.series:
                Color(*get_color_from_hex(color))
                self._lines[name] = Line(width=dp(1.4))

    def clear(self):
        for d in self.data.values():
            d.clear()
        for line in self._lines.values():
            line.points = []

    def refresh(self):
        w, h = self.size
        if w < 6 or h < 6:
            return
        every = [v for d in self.data.values() for v in d]
        if len(every) < 2:
            for line in self._lines.values():
                line.points = []
            return
        lo, hi = min(every), max(every)
        if hi - lo < 1e-6:
            lo, hi = lo - 1.0, lo + 1.0
        pad = dp(3)
        span = hi - lo
        # 注意:Kivy 的 canvas 坐标是相对父控件的,所以要加上自己的 pos
        ox, oy = self.pos
        for name, line in self._lines.items():
            d = self.data[name]
            n = len(d)
            if n < 2:
                line.points = []
                continue
            pts = []
            for i, v in enumerate(d):
                x = ox + pad + (w - 2 * pad) * (i / float(n - 1))
                y = oy + pad + (h - 2 * pad) * ((v - lo) / span)
                pts.extend((x, y))
            line.points = pts


class PlotPanel(BoxLayout):
    """四张图:速度 / 力矩 / 位置 / 光电,与桌面版曲线一一对应。"""

    SPEC = (
        ("速度 rpm", (("目标", "#ffd24a"), ("反馈", "#4ad0ff"))),
        ("力矩 N·m", (("力矩", "#ff9f43"),)),
        ("位置 °", (("目标", "#ffd24a"), ("实际", "#4ad0ff"))),
        ("光电电平", (("PE2", "#7dff9b"), ("PE3", "#c792ea"))),
    )

    def __init__(self, **kw):
        super().__init__(orientation="vertical", spacing=dp(3), padding=dp(4), **kw)
        self.charts = {}
        for title, series in self.SPEC:
            box = Panel(color=PANEL, orientation="vertical", padding=(dp(4), dp(1)))
            head = hbar(height=dp(16), padding=0)
            head.add_widget(lab(title, color=MUTED, font_size=sp(11)))
            for name, color in series:
                head.add_widget(lab(name, color=get_color_from_hex(color),
                                    font_size=sp(10), size_hint_x=None, width=dp(40)))
            box.add_widget(head)
            chart = Chart(series)
            box.add_widget(chart)
            self.add_widget(box)
            self.charts[title] = chart

    def clear(self):
        for chart in self.charts.values():
            chart.clear()

    def refresh(self):
        for chart in self.charts.values():
            chart.refresh()


# ---------------------------------------------------------------- 字段表
FIELD_NAMES = (
    "电机", "目标 / 反馈 rpm", "误差 rpm", "力矩 N·m",
    "位置 °", "目标 °", "误差 °", "光电 PE2/PE3",
    "IMU roll/pitch/yaw", "角速度 gx/gy/gz",
    "遥控", "杆量", "鼠标 / 按键",
    "IM6014", "IM6014 位置/速度/力矩", "IM6014 温度/电压",
    "错误", "故障",
)


class FieldsView(ScrollView):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.grid = GridLayout(cols=2, size_hint_y=None, spacing=dp(1), padding=dp(6))
        self.grid.bind(minimum_height=self.grid.setter("height"))
        self.add_widget(self.grid)
        self._keys = None
        self._vals = []
        self._build(FIELD_NAMES)

    def _build(self, names):
        self.grid.clear_widgets()
        self._vals = []
        for name in names:
            self.grid.add_widget(lab(name, color=MUTED, font_size=sp(12),
                                     size_hint_y=None, height=dp(24)))
            v = lab("-", font_size=sp(12), size_hint_y=None, height=dp(24))
            self.grid.add_widget(v)
            self._vals.append(v)
        self._keys = tuple(names)

    def update_rows(self, rows):
        """rows: [(名字, 值)];名字变了才重建,否则只改文字。"""
        keys = tuple(name for name, _v in rows)
        if keys != self._keys:
            self._build(keys)
        for w, (_name, value) in zip(self._vals, rows):
            if w.text != value:
                w.text = value


class LogView(TextInput):
    """日志:用只读 TextInput,自带滚动/换行,不用跟 ScrollView 的布局较劲。"""

    def __init__(self, **kw):
        super().__init__(text="", readonly=True, multiline=True, font_size=sp(11),
                         background_normal="", background_active="",
                         background_color=PANEL, foreground_color=FG,
                         cursor_color=FG, cursor_width=0, padding=(dp(6), dp(6)),
                         **kw)
        self.lines = collections.deque(maxlen=400)

    def add(self, text):
        self.lines.append("%s %s" % (time.strftime("%H:%M:%S"), text))
        self.text = "\n".join(self.lines)
        Clock.schedule_once(self._to_bottom, 0)   # 等排版完再滚到最新一行

    def _to_bottom(self, *_):
        self.scroll_y = 0

    def clear_text(self):
        self.lines.clear()
        self.text = ""


# ---------------------------------------------------------------- 命令页
class CmdView(BoxLayout):
    def __init__(self, app, **kw):
        super().__init__(orientation="vertical", spacing=dp(5), padding=dp(8), **kw)
        self.app = app

        self.add_widget(cap("发送 HEX(空格分隔,如 5C 05 00 01)"))
        row = hbar()
        self.hex_in = ti(hint="5C 05 00 01")
        row.add_widget(self.hex_in)
        row.add_widget(btn("发送", lambda _b: self.send_hex(), width=dp(64)))
        self.add_widget(row)

        self.add_widget(cap("0x5C 调试命令"))
        row = hbar()
        row.add_widget(Label(text="电机", color=FG, size_hint_x=None, width=dp(44)))
        self.motor = Spinner(text="0x202", values=("0x201", "0x202", "0x203", "0x204"),
                             size_hint_x=None, width=dp(100))
        row.add_widget(self.motor)
        self.add_widget(row)
        self.add_widget(self._num_row("速度 rpm", "120", self.send_speed))
        self.add_widget(self._num_row("角度 °(位置环)", "0", self.send_angle))
        self.add_widget(self._num_row("PID 六路(逗号分隔)", "0.002,0.0005,0,0,0,0", self.send_pid))
        row = hbar()
        row.add_widget(btn("回读 PID", lambda _b: self.app.send_cmd(0x04)))
        row.add_widget(btn("停止 / 失能", lambda _b: self.app.send_cmd(0x05), color=WARN))
        self.add_widget(row)
        self.add_widget(Widget())          # 占位,把内容顶到上面

    def _num_row(self, title, default, sender):
        box = BoxLayout(orientation="vertical", size_hint_y=None, height=dp(64), spacing=0)
        box.add_widget(cap(title))
        row = hbar()
        inp = ti(default)
        row.add_widget(inp)
        row.add_widget(btn("发送", lambda _b: sender(inp.text), width=dp(64)))
        box.add_widget(row)
        return box

    def send_hex(self):
        try:
            data = parse_hex(self.hex_in.text)
        except ValueError:
            self.app.toast("HEX 格式不对:只需要 0-9 A-F,用空格分开")
            return
        if not data:
            self.app.toast("没填内容")
            return
        self.app.write(data, "HEX")

    def _mid(self):
        try:
            return int(self.motor.text, 16)
        except Exception:
            return 0x202

    def send_speed(self, text):
        try:
            rpm = int(float(text or 0))
            mid = self._mid()
            cmd = {0x201: 0x09, 0x202: 0x01, 0x203: 0x0D, 0x204: 0x0C}.get(mid)
            if cmd is None:
                self.app.toast("该电机没有速度命令")
                return
            self.app.send_cmd(cmd, rpm.to_bytes(2, "little", signed=True))
        except Exception:
            self.app.toast("速度只能填整数 rpm")

    def send_angle(self, text):
        try:
            mid = self._mid()
            cmd = {0x202: 0x02, 0x204: 0x0B}.get(mid)
            if cmd is None:
                self.app.toast("只有 202/204 有角度命令")
                return
            self.app.send_cmd(cmd, int(float(text or 0) * 1000).to_bytes(4, "little", signed=True))
        except Exception:
            self.app.toast("角度只能填数字")

    def send_pid(self, text):
        try:
            vals = [float(x) for x in (text or "").replace(";", ",").split(",") if x.strip()]
        except ValueError:
            self.app.toast("PID 要 6 个数字,用逗号分开")
            return
        if len(vals) != 6:
            self.app.toast("PID 要 6 个数字(速 Kp/Ki/Kd,位 Kp/Ki/Kd)")
            return
        mid = self._mid()
        if mid in (0x201, 0x203):
            self.app.send_cmd(0x0A, struct.pack("<3f", vals[0], vals[1], vals[2]))
        elif mid == 0x204:
            self.app.send_cmd(0x0E, struct.pack("<6f", *vals))
        else:
            self.app.send_cmd(0x03, struct.pack("<6f", *vals))


# ---------------------------------------------------------------- App
class DebugApp(App):
    title = "调试助手 · Android"

    # ---- 生命周期 ----
    def build(self):
        Window.clearcolor = BG
        if not serial_link.IS_ANDROID:
            Window.size = (480, 880)

        self.q = queue.Queue()
        self.decoder = proto.FrameDecoder()
        self.recorder = self._new_recorder()
        self.recording = True
        self.paused = False
        self.reader = None
        self.demo_src = None
        self.port_map = {}
        self.baud = "921600"
        self.seq = 0
        self.last = None
        self.last_hex = ""
        self._page = "字段"

        root = Panel(color=BG, orientation="vertical")
        root.add_widget(self._top_bar())
        root.add_widget(self._conn_bar())
        root.add_widget(self._tab_bar())
        self.content = BoxLayout()
        root.add_widget(self.content)
        root.add_widget(self._status_bar())

        self.fields = FieldsView()
        self.plots = PlotPanel()
        self.log = LogView()
        self.cmd = CmdView(self)
        self.pages = {
            "字段": self.fields,
            "曲线": self.plots,
            "日志": self._log_page(),
            "命令": self.cmd,
        }
        self.show_page("字段")

        Clock.schedule_interval(self._pump, 1 / 30.0)
        Clock.schedule_interval(self._refresh_plots, 1 / 20.0)
        Clock.schedule_interval(self._refresh_fields, 0.2)
        Clock.schedule_interval(self._refresh_status, 0.5)
        Clock.schedule_once(lambda _dt: self.refresh_ports(), 0.2)
        if "--demo" in sys.argv:
            Clock.schedule_once(lambda _dt: self.start_demo(), 0.6)
        return root

    def on_stop(self):
        self._stop_link()

    @staticmethod
    def _new_recorder():
        """手机内存有限:记录上限比桌面版(30 万行)小很多。"""
        rec = proto.Recorder()
        rec.max_rows = 60000
        rec.max_raw_rows = 40000
        return rec

    def on_pause(self):
        return True          # 切后台继续读串口

    def on_resume(self):
        self.refresh_ports()

    # ---- 界面 ----
    def _top_bar(self):
        bar = hbar(height=dp(34), padding=dp(8))
        bar.add_widget(lab("调试助手", color=ACCENT, font_size=sp(16), bold=True))
        bar.add_widget(lab(serial_link.backend_name(), color=MUTED, font_size=sp(10)))
        self.conn_lbl = lab("未连接", color=MUTED, font_size=sp(12), align="right")
        bar.add_widget(self.conn_lbl)
        return bar

    def _conn_bar(self):
        box = Panel(color=PANEL, orientation="vertical", size_hint_y=None,
                    height=dp(96), spacing=dp(3), padding=(dp(6), dp(2)))
        row1 = hbar()
        self.port_spin = Spinner(text="选择串口", values=(), font_size=sp(12),
                                 size_hint_y=None, height=dp(38), background_normal="",
                                 background_color=PANEL2)
        row1.add_widget(self.port_spin)
        row1.add_widget(btn("刷新", lambda _b: self.refresh_ports(), width=dp(58)))
        self.conn_btn = btn("连接", self.toggle_link, color=ACCENT, width=dp(64))
        row1.add_widget(self.conn_btn)
        box.add_widget(row1)

        row2 = hbar()
        self.baud_spin = Spinner(text=self.baud, values=("921600", "115200", "100000", "4000000"),
                                 font_size=sp(12), size_hint_y=None, height=dp(38),
                                 background_normal="", background_color=PANEL2)
        self.baud_spin.bind(text=self._on_baud_pick)
        row2.add_widget(self.baud_spin)
        row2.add_widget(btn("自动匹配", lambda _b: self.auto_match(), width=dp(88)))
        self.demo_btn = btn("演示", self.toggle_demo)
        row2.add_widget(self.demo_btn)
        box.add_widget(row2)
        return box

    def _tab_bar(self):
        bar = BoxLayout(orientation="horizontal", size_hint_y=None, height=dp(38),
                        spacing=dp(2), padding=(dp(4), 0))
        self.tab_btns = {}
        for name in ("字段", "曲线", "日志", "命令"):
            b = ToggleButton(text=name, group="tabs", font_size=sp(13),
                             background_normal="", background_color=PANEL2, color=FG)
            b.bind(on_press=lambda _b, n=name: self.show_page(n))
            bar.add_widget(b)
            self.tab_btns[name] = b
        return bar

    def _log_page(self):
        page = BoxLayout(orientation="vertical", spacing=dp(3), padding=(dp(4), dp(2)))
        page.add_widget(self.log)
        bar = hbar()
        b_rec = ToggleButton(text="记录", state="down", font_size=sp(12),
                             background_normal="", background_color=PANEL2, color=FG)
        b_rec.bind(on_press=lambda b: self._toggle_record(b))
        b_pause = ToggleButton(text="暂停显示", font_size=sp(12),
                               background_normal="", background_color=PANEL2, color=FG)
        b_pause.bind(on_press=lambda b: self._toggle_pause(b))
        bar.add_widget(b_rec)
        bar.add_widget(b_pause)
        page.add_widget(bar)
        bar2 = hbar()
        bar2.add_widget(btn("导出 CSV", lambda _b: self.export("csv"), font_size=sp(12)))
        bar2.add_widget(btn("原始 CSV", lambda _b: self.export("raw"), font_size=sp(12)))
        bar2.add_widget(btn("文档 MD", lambda _b: self.export("md"), font_size=sp(12)))
        bar2.add_widget(btn("清空", lambda _b: self.clear_all(), font_size=sp(12)))
        page.add_widget(bar2)
        return page

    def _status_bar(self):
        bar = Panel(color=PANEL, orientation="horizontal", size_hint_y=None,
                    height=dp(24), padding=(dp(8), 0))
        self.status_lbl = lab("就绪", color=MUTED, font_size=sp(10))
        bar.add_widget(self.status_lbl)
        return bar

    def show_page(self, name):
        self._page = name
        for n, b in self.tab_btns.items():
            b.state = "down" if n == name else "normal"
        self.content.clear_widgets()
        self.content.add_widget(self.pages[name])

    # ---- 串口 ----
    def refresh_ports(self):
        ports = serial_link.list_ports()
        self.port_map = {}
        values = []
        for p in ports:
            label = p.label
            if label in self.port_map:
                label = "%s (%d)" % (label, len(values) + 1)
            self.port_map[label] = p.device
            values.append(label)
        self.port_spin.values = values
        if values:
            if self.port_spin.text not in values:
                self.port_spin.text = values[0]
            self.log.add("[串口] 发现 %d 个:%s" % (len(values), ", ".join(values)))
        else:
            self.port_spin.text = "没有串口"
            self.log.add("[串口] 没发现串口:检查 OTG 线 / 驱动(桌面看驱动,手机看是否弹权限窗)")
        return values

    def _on_baud_pick(self, _spin, text):
        self.baud = text

    def _pick_device(self):
        label = self.port_spin.text
        if label in self.port_map:
            return self.port_map[label]
        vals = list(self.port_map.values())
        return vals[0] if vals else None

    def toggle_link(self, *_):
        if self.reader is not None and self.reader.is_alive():
            self._stop_link()
            self.log.add("[串口] 已断开")
            return
        if self.demo_src is not None:
            self.stop_demo()
        name = self._pick_device()
        if not name:
            self.toast("没有可选串口:先插 OTG 线,再点「刷新」")
            return
        self._stop_link()
        self.decoder = proto.FrameDecoder()
        self.reader = serial_link.Reader(name, self.baud, self.q)
        self.reader.start()
        self.conn_lbl.text = "连接中…"
        self.conn_btn.text = "断开"
        self.log.add("[串口] 打开 %s @ %s" % (name, self.baud))

    def _stop_link(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
        if self.demo_src is not None:
            self.demo_src.stop()
            self.demo_src = None
        try:
            self.conn_btn.text = "连接"
            self.conn_lbl.text = "未连接"
        except Exception:
            pass

    def start_demo(self):
        self._stop_link()
        self.decoder = proto.FrameDecoder()
        self.demo_src = demo.DemoSource(self.q)
        self.demo_src.start()
        self.demo_btn.text = "停演示"
        self.conn_lbl.text = "演示数据"
        self.log.add("[演示] 合成 0x5A 帧(20 Hz),没有硬件也能看界面")

    def stop_demo(self):
        if self.demo_src is not None:
            self.demo_src.stop()
            self.demo_src = None
        self.demo_btn.text = "演示"
        self.conn_lbl.text = "未连接"

    def toggle_demo(self, *_):
        if self.demo_src is not None:
            self.stop_demo()
        else:
            self.start_demo()

    def auto_match(self):
        name = self._pick_device()
        if not name:
            self.toast("先选一个串口")
            return
        self._stop_link()                      # 扫描要独占端口,先把连接断掉
        self.log.add("[自动匹配] 开始试波特率…")

        def worker():
            for baud in ("921600", "115200", "100000", "4000000"):
                self.q.put(("log", "[自动匹配] 试 %s @ %s" % (name, baud)))
                if serial_link.probe_baud(name, baud, 1.2):
                    self.q.put(("match", name, baud))
                    return
            self.q.put(("log", "[自动匹配] 4 个波特率都没收到有效帧"))

        threading.Thread(target=worker, daemon=True).start()

    # ---- 发送 ----
    def send_cmd(self, cmd, payload=b""):
        self.seq = (self.seq + 1) & 0xFFFF
        frame = bytes([0x5C, cmd, (self.seq >> 8) & 0xFF, self.seq & 0xFF]) + payload
        self.write(frame, "0x5C cmd=0x%02X" % cmd)

    def write(self, data, note=""):
        if self.reader is None or not self.reader.is_alive():
            self.toast("串口没连接")
            return
        try:
            self.reader.write(data)
            self.log.add("[发送] %s %s" % (data.hex(" ").upper(), note))
        except Exception as exc:
            self.log.add("[发送失败] %s" % exc)

    # ---- 数据 ----
    def _pump(self, _dt):
        try:
            self._pump_once()
        except Exception:
            # 界面刷新出问题也不能让 Clock 链断掉
            import traceback
            self.log.add("[异常] " + traceback.format_exc().strip().splitlines()[-1])

    def _pump_once(self):
        chunks = []
        try:
            while True:
                msg = self.q.get_nowait()
                if msg[0] == "data":
                    chunks.append(msg[1])
                elif msg[0] == "log":
                    self.log.add(msg[1])
                elif msg[0] == "match":
                    _, name, baud = msg
                    self.baud_spin.text = baud
                    for label, dev in self.port_map.items():
                        if dev == name:
                            self.port_spin.text = label
                            break
                    self.log.add("[自动匹配] 命中 %s @ %s" % (name, baud))
                    self.toggle_link()
        except queue.Empty:
            pass

        if self.reader is not None and self.reader.error:
            self.log.add("[串口错误] %s" % self.reader.error)
            self._stop_link()

        for data in chunks:
            for d, frame in self.decoder.feed(data):
                if self.recording:
                    self.recorder.add(self.port_spin.text, self.baud, d, frame)
                self.last = d
                self.last_hex = frame.hex(" ").upper()
                self._push_series(d)
            for text in self.decoder.texts:
                self.log.add(text)
            self.decoder.texts.clear()

    def _push_series(self, d):
        """把一帧解码结果推到四张曲线上(按帧采样,和桌面版曲线一致)。"""
        ch = self.plots.charts
        try:
            if d.get("proto") == "MOTOR_PID":
                ch["速度 rpm"].data["目标"].append(float(d.get("pid_target_rpm", 0.0)))
                ch["速度 rpm"].data["反馈"].append(float(d.get("pid_feedback_rpm", 0.0)))
                ch["力矩 N·m"].data["力矩"].append(float(d.get("pid_torque", 0.0)))
                ch["位置 °"].data["目标"].append(math.degrees(float(d.get("pos_target", 0.0))))
                ch["位置 °"].data["实际"].append(math.degrees(float(d.get("pos", 0.0))))
                ch["光电电平"].data["PE2"].append(float(d.get("pe2_level", 0)))
                ch["光电电平"].data["PE3"].append(float(d.get("pe3_level", 0)))
            elif "vx" in d:                       # 上位机帧:画 vx 当作"速度"
                ch["速度 rpm"].data["目标"].append(float(d.get("vx", 0.0)))
                ch["速度 rpm"].data["反馈"].append(float(d.get("vy", 0.0)))
                ch["力矩 N·m"].data["力矩"].append(float(d.get("w", 0.0)))
        except Exception:
            pass

    def _refresh_plots(self, _dt):
        if self.paused or self._page != "曲线":
            return
        self.plots.refresh()

    def _refresh_fields(self, _dt):
        if self.paused:
            return
        if self._page == "字段":
            self.fields.update_rows(self._rows())

    def _refresh_status(self, _dt):
        st = self.decoder.stats
        self.status_lbl.text = ("0x5A %d | 0xA5 %d | CRC %d | 丢弃 %d | 记录 %d"
                                % (st.get("DEBUG", 0), st.get("A5", 0), st.get("crc_err", 0),
                                   st.get("reject", 0), len(self.recorder.rows)))
        if self.reader is not None and self.reader.is_alive():
            self.conn_lbl.text = "已连接"
            self.conn_btn.text = "断开"
        elif self.demo_src is None and self.conn_btn.text == "断开":
            self.conn_lbl.text = "未连接"
            self.conn_btn.text = "连接"

    # ---- 字段内容 ----
    def _rows(self):
        d = self.last
        if not d:
            return [(name, "-") for name in FIELD_NAMES]
        if d.get("proto") != "MOTOR_PID":
            rows = [(k, str(v)) for k, v in sorted(d.items())
                    if not isinstance(v, (list, dict))][:18]
            rows.append(("最后一帧 HEX", self.last_hex[:40] + ("…" if len(self.last_hex) > 40 else "")))
            return rows or [("等待数据", "-")]

        flags = d.get("flags", 0)
        state = " ".join(x for bit, x in proto.MOTOR_FLAG if flags & bit) or "正常"
        errmap = {0: "OK", 1: "WARN", 2: "ERROR", 3: "FATAL"}
        rows = [
            ("电机", "0x%03X  %s" % (d.get("motor_id", 0), state)),
            ("目标 / 反馈 rpm", "%.1f / %.1f" % (d.get("pid_target_rpm", 0), d.get("pid_feedback_rpm", 0))),
            ("误差 rpm", "%.1f" % d.get("pid_error_rpm", 0.0)),
            ("力矩 N·m", "%.3f" % d.get("pid_torque", 0.0)),
            ("位置 °", "%.2f" % math.degrees(d.get("pos", 0.0))),
            ("目标 °", "%.2f" % math.degrees(d.get("pos_target", 0.0))),
            ("误差 °", "%.2f" % math.degrees(d.get("pos_error", 0.0))),
            ("光电 PE2/PE3", "%s / %s" % (d.get("pe2_level", 0), d.get("pe3_level", 0))),
        ]
        if d.get("has_imu"):
            rows += [
                ("IMU roll/pitch/yaw",
                 "%.3f / %.3f / %.3f" % (d.get("imu_roll", 0), d.get("imu_pitch", 0),
                                         d.get("imu_yaw", 0))),
                ("角速度 gx/gy/gz",
                 "%.3f / %.3f / %.3f" % (d.get("imu_gx", 0), d.get("imu_gy", 0),
                                         d.get("imu_gz", 0))),
            ]
        else:
            rows += [("IMU roll/pitch/yaw", "旧固件无 IMU 块"), ("角速度 gx/gy/gz", "-")]
        if d.get("has_rc"):
            rows += [
                ("遥控", "%s  拨杆 %s/%s" % ("在线" if d.get("rc_online") else "离线",
                                            proto.RC_SWITCH_NAME.get(d.get("rc_sw0"), "?"),
                                            proto.RC_SWITCH_NAME.get(d.get("rc_sw1"), "?"))),
                ("杆量", " ".join(str(v) for v in d.get("rc_ch", []))),
                ("鼠标 / 按键", "%s,%s,%s  0x%04X" % (
                    (d.get("rc_mouse") or [0, 0, 0])[0], (d.get("rc_mouse") or [0, 0, 0])[1],
                    (d.get("rc_mouse") or [0, 0, 0])[2], d.get("rc_key", 0))),
            ]
        else:
            rows += [("遥控", "旧固件无遥控块"), ("杆量", "-"), ("鼠标 / 按键", "-")]
        if d.get("has_ys"):
            rows += [
                ("IM6014", "%s  %s" % ("在线" if d.get("ys_online") else "离线",
                                       "FOC" if d.get("ys_status") else "锁定")),
                ("IM6014 位置/速度/力矩", "%.3f rad  %.2f rad/s  %.3f N·m" % (
                    d.get("ys_pos", 0), d.get("ys_w", 0), d.get("ys_t", 0))),
                ("IM6014 温度/电压", "%.1f ℃  %.1f V" % (d.get("ys_temp", 0), d.get("ys_voltage", 0))),
            ]
        else:
            rows += [("IM6014", "旧固件无此块"), ("IM6014 位置/速度/力矩", "-"),
                     ("IM6014 温度/电压", "-")]
        rows += [
            ("错误", "%s  code=0x%04X" % (errmap.get(d.get("err_level", 0), "?"),
                                          d.get("err_code", 0))),
            ("故障", ("%s  CFSR=0x%08X  PC=0x%08X" % (
                proto.FAULT_NAME.get(d.get("fault_code", 0), "-"),
                d.get("fault_cfsr", 0), d.get("fault_pc", 0)))
             if d.get("fault_code") else
             ("正常  CFSR=0x%08X" % d.get("fault_cfsr", 0))),
        ]
        return rows

    # ---- 日志/导出 ----
    def _toggle_record(self, b):
        self.recording = b.state == "down"
        self.log.add("[记录] %s" % ("开" if self.recording else "关"))

    def _toggle_pause(self, b):
        self.paused = b.state == "down"
        self.log.add("[显示] %s" % ("暂停" if self.paused else "继续"))

    def clear_all(self):
        self.recorder = self._new_recorder()
        self.plots.clear()
        self.decoder = proto.FrameDecoder()
        self.last = None
        self.last_hex = ""
        self.log.add("[清空] 记录 / 曲线 / 统计 已清空")

    def export_dir(self):
        if not serial_link.IS_ANDROID:
            base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "导出")
            os.makedirs(base, exist_ok=True)
            return base
        for cand in ("/sdcard/Download/调试助手", "/storage/emulated/0/Download/调试助手",
                     os.path.join(self.user_data_dir, "导出")):
            try:
                os.makedirs(cand, exist_ok=True)
                probe = os.path.join(cand, ".w")
                with open(probe, "w") as fp:
                    fp.write("ok")
                os.remove(probe)
                return cand
            except Exception:
                continue
        return self.user_data_dir

    def export(self, kind):
        if kind != "md" and not self.recorder.rows:
            self.toast("还没记录到数据")
            return
        name = "mcu_%s" % time.strftime("%Y%m%d_%H%M%S")
        base = self.export_dir()
        try:
            if kind == "csv":
                path = os.path.join(base, name + ".csv")
                self.recorder.export_csv(path)
            elif kind == "raw":
                path = os.path.join(base, name + "_raw.csv")
                self.recorder.export_raw_csv(path)
            else:
                path = os.path.join(base, name + ".md")
                self.recorder.export_md(path, dict(self.decoder.stats))
        except Exception as exc:
            self.toast("导出失败:%s" % exc)
            return
        self.log.add("[导出] %s" % path)
        self.toast("已导出:\n%s" % path)

    def toast(self, text, title="提示"):
        self.log.add(text)
        box = Panel(color=PANEL, orientation="vertical", padding=dp(10), spacing=dp(8))
        msg = Label(text=text, font_size=sp(13), color=FG, halign="center", valign="middle")
        msg.bind(size=lambda l, s: setattr(l, "text_size", (s[0], s[1])))
        box.add_widget(msg)
        pop = Popup(title=title, content=box, size_hint=(0.86, 0.34),
                    background_color=PANEL2, title_color=FG, separator_color=ACCENT)
        b = btn("好", lambda _x: pop.dismiss(), color=ACCENT, width=dp(80))
        b.size_hint_x = 1
        box.add_widget(b)
        pop.open()


if __name__ == "__main__":
    try:
        DebugApp().run()
    except Exception:
        import traceback
        traceback.print_exc()
        raise
