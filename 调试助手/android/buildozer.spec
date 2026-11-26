[app]

# 应用信息
title = 调试助手
package.name = tiaoshizhushou
package.domain = org.dsh

# 源码目录(. 表示本文件所在目录)
source.dir = .
source.include_exts = py,png,jpg,kv,atlas,ttf,txt,md
# 只打包 App 需要的;测试截图、开发工具、导出目录不进 APK
source.exclude_dirs = tests, tools, 导出, __pycache__, bin, .buildozer

version = 0.1.0

# usbserial4a 需要 pyjnius + usb4a;pyserial 供 usbserial4a 复用接口
requirements = python3,kivy,pyjnius,usb4a,usbserial4a,pyserial

# 手机竖屏为主,平板/横屏也能用
orientation = portrait
fullscreen = 0

# Android 6.0 起(API 23),usb4a 0.3+ 要求 31 以上才对 PendingIntent 处理正确
android.api = 33
android.minapi = 21
android.ndk_api = 21
android.archs = arm64-v8a, armeabi-v7a
android.accept_sdk_license = True

# usbserial4a 依赖 termios
android.p4a_whitelist = lib-dynload/termios.so

# USB Host 声明 + OTG 插入时唤起的 intent-filter + 设备过滤
android.extra_manifest_xml = manifest/extra_manifest.xml
android.manifest.intent_filters = manifest/intent-filter.xml
android.res_xml = manifest/device_filter.xml

# 日志(排查真机问题时看 adb logcat 或 android.txt)
android.logcat_filters = *:S python:D

[buildozer]
log_level = 2
warn_on_root = 1
