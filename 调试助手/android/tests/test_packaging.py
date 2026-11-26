# -*- coding: utf-8 -*-
"""检查打包配置:buildozer.spec 能解析、manifest XML 合法、导出文件内容正常。"""
import configparser
import glob
import os
import re
import sys
import xml.etree.ElementTree as ET

ANDROID = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SPEC = os.path.join(ANDROID, "buildozer.spec")


def check_spec():
    cp = configparser.ConfigParser()
    cp.read(SPEC, encoding="utf-8")
    assert cp.has_section("app") and cp.has_section("buildozer"), "缺 [app]/[buildozer] 段"
    app = cp["app"]
    for key in ("title", "package.name", "package.domain", "source.dir", "requirements",
                "android.api", "android.minapi", "android.p4a_whitelist",
                "android.extra_manifest_xml", "android.manifest.intent_filters",
                "android.res_xml"):
        assert app.get(key), "spec 缺 %s" % key
    reqs = [r.strip() for r in app["requirements"].split(",")]
    for need in ("kivy", "pyjnius", "usb4a", "usbserial4a"):
        assert need in reqs, "requirements 缺 %s" % need
    for key in ("android.extra_manifest_xml", "android.manifest.intent_filters",
                "android.res_xml"):
        path = os.path.join(ANDROID, app[key])
        assert os.path.isfile(path), "%s 指向的文件不存在: %s" % (key, path)
    print("buildozer.spec OK · 版本", app.get("version"),
          "· API", app.get("android.api"), "· archs", app.get("android.archs"))


def check_xml():
    # 这三个是"片段",会被 buildozer 拼进 AndroidManifest.xml,
    # 所以套一层带 android 命名空间的根节点来校验 XML 是否合法。
    ns = 'xmlns:android="http://schemas.android.com/apk/res/android"'
    for path in sorted(glob.glob(os.path.join(ANDROID, "manifest", "*.xml"))):
        with open(path, encoding="utf-8") as fp:
            body = fp.read()
        body = re.sub(r"^\s*<\?xml[^>]*\?>", "", body)     # 片段里不能带 XML 声明
        ET.fromstring("<root %s>%s</root>" % (ns, body))
        print("xml OK:", os.path.basename(path))
    # device_filter 里的 <usb-device/> 为空表示匹配所有 USB 串口设备
    with open(os.path.join(ANDROID, "manifest", "device_filter.xml"), encoding="utf-8") as fp:
        assert "usb-device" in fp.read(), "device_filter 里没有 <usb-device/>"


def check_exports():
    dirs = sorted(glob.glob(os.path.join(os.environ.get("TEMP", "."),
                                        "dsh_android_export_*")))
    if not dirs:
        print("没有导出样本可检查(先跑 tests/smoke_ui.py)")
        return
    latest = max(dirs, key=os.path.getmtime)
    files = sorted(os.listdir(latest))
    assert len(files) == 3, "导出文件数量不对: %s" % files
    md = [f for f in files if f.endswith(".md")][0]
    raw = [f for f in files if "_raw" in f][0]
    with open(os.path.join(latest, raw), encoding="utf-8-sig") as fp:
        head = fp.readline().strip()
    with open(os.path.join(latest, md), encoding="utf-8") as fp:
        text = fp.read()
    assert head.startswith("time,port,baud,proto,len,hex"), head
    assert "MOTOR_PID" in text or "0x5A" in text, "MD 里没有协议内容"
    print("导出 OK:", latest)
    print("  raw header:", head)
    print("  md 行数:", len(text.splitlines()))


if __name__ == "__main__":
    check_spec()
    check_xml()
    check_exports()
    print("PACKAGING CHECK OK")
