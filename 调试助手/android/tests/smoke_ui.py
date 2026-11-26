# -*- coding: utf-8 -*-
"""界面冒烟:用演示数据跑 Kivy 界面,四个页面各截一张图,然后退出。

    cd android && python tests/smoke_ui.py

只在桌面开发机跑(需要图形环境),不进 APK。截图在 tests/shots/。
"""
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ANDROID = os.path.dirname(HERE)
OUT = os.path.join(ANDROID, "tests", "shots")
sys.path.insert(0, ANDROID)

from kivy.clock import Clock                                  # noqa: E402

import main                                                   # noqa: E402

PAGES = (("01_fields", "字段"), ("02_plots", "曲线"),
         ("03_log", "日志"), ("04_cmd", "命令"))


def grab(app, name):
    """用 Kivy 自己的离屏渲染截图,不受窗口前后台影响。"""
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name + ".png")
    app.root.export_to_png(path)
    print("shot ->", path)
    return path


def run():
    app = main.DebugApp()
    index = {"i": 0}

    def start(_dt):
        app.start_demo()

    def step(_dt):
        if index["i"] >= len(PAGES):
            return
        name, page = PAGES[index["i"]]
        app.show_page(page)
        Clock.schedule_once(lambda _dt: grab(app, name), 0.25)
        index["i"] += 1

    def finish(_dt):
        app.show_page("日志")
        # 导出到临时目录,别在仓库里堆文件
        tmp = tempfile.mkdtemp(prefix="dsh_android_export_")
        app.export_dir = lambda: tmp
        app.export("csv")
        app.export("raw")
        app.export("md")
        print("导出目录:", tmp)
        print("记录帧数:", len(app.recorder.rows))
        print("状态:", app.status_lbl.text)
        Clock.schedule_once(lambda _dt: app.stop(), 0.4)

    Clock.schedule_once(start, 0.8)
    Clock.schedule_once(step, 2.4)
    Clock.schedule_interval(step, 1.0)
    Clock.schedule_once(finish, 2.4 + 1.0 * len(PAGES) + 0.6)
    app.run()

    if not app.recorder.rows:
        raise SystemExit("演示模式没有记录到任何帧")
    for name, _page in PAGES:
        if not os.path.isfile(os.path.join(OUT, name + ".png")):
            raise SystemExit("缺少截图:" + name)
    print("UI SMOKE OK")


if __name__ == "__main__":
    run()
