# 调试助手

STM32 机器人串口调试工具，Windows / Ubuntu 22.04 通用，Tkinter 单文件 GUI；
另有一份 Kivy 写的 Android 版，手机用 USB-OTG 线直连串口。

## 目录

```text
调试助手.py                 唯一源码(Windows / Ubuntu 共用)
windows/                    Windows 打包脚本(--onefile 的 spec/build.ps1 引用 ../调试助手.py)
ubuntu/x64/                 Ubuntu 22.04 x86_64 运行/打包脚本(引用 ../调试助手.py)
ubuntu/arm/                 Ubuntu 22.04 arm64 运行/打包脚本(引用 ../调试助手.py)
android/                    Android(Kivy)版 + buildozer 打包,proto.py 由 tools/sync_proto.py 从 ../调试助手.py 抽取
```

> 各平台目录里**不再放 `调试助手.py` 副本**(以前放过,结果三份副本各自漂移、协议版本对不上)。
> 改协议只改根目录这一份;打包脚本已经全部指向 `../调试助手.py`。

## 功能

- **遥测**：0x5A 帧，兼容 30/38/40/70/94 B 小帧和旧 210 B 大帧；
- **IMU**：极简电机帧 60~83 B 新增 roll/pitch/yaw + gx/gy/gz；
- **上位机协议**：0xA5 变长帧；
- **自动匹配**：扫描系统串口 × 常见波特率，收到有效帧自动分配；
- **曲线**：目标/反馈 rpm、力矩、位置、光电电平；
- **调试命令**：0x5C 速度/角度/PID/停止，PID 六路 float；
- **底盘/云台调试**：底盘 vx/vy/wz、云台 yaw/pitch，带「按住走、松开停」的点动；
- **记录**：`记录数据` 开关；解码帧和原始帧都缓存；
- **导出**：解码 CSV / Markdown / **原始 CSV** / **原始 HEX**；
- **主题**：深色/浅色/护眼绿/暖纸，记住上次选择；
- **背景图**：GIF（动图）/PNG/JPG 铺在曲线、仪表、波形画布上，记住上次选择；
- **Android 版**：Kivy + USB-OTG 直连串口，协议解析与桌面版同源（见 [android/README.md](android/README.md)）。

## 背景图

标题栏右侧「背景图 ▾」：

```text
选择图片…(GIF 会动)   选一张图,动图会自己播
变亮 / 变暗           压暗程度 ±10%,底部滑块的快捷入口
整窗半透明(实验)      壁纸铺到顶部面板底下;整个窗口 82% 不透明
去掉背景图            恢复原来的纯色背景
```

- 铺图位置：电机调试的两条曲线、舵轮仪表盘、VOFA 波形 + 姿态小窗、底盘/云台示意图，以及主窗口的**字段表和日志**
  （这几块都按「整窗同一张图」取各自那一块，所以上下拼起来是一张连续的图，不是重复缩放）；
- 表格/日志只额外压暗 0.20，文字再带 1px 黑边，所以整窗看起来是一张图，不会中间嵌一块黑；
  其余面板（端口卡片、工具栏）仍是实心底色；
- 压暗：图片太亮会盖住曲线，载入时按图片亮度自动给一个值，底部「背景压暗」滑块可随时调，调完记住；
- 配置写在 `~/.debug_assistant.json`（`bg_image` / `bg_dim`），下次启动自动恢复；
- 动图只在窗口可见时播放，全部最小化后会停下来，不空转 CPU。

格式支持：

| 环境 | 支持 |
|---|---|
| 只装 pyserial | GIF（动图）、PNG、PPM/PGM（Tk 自带解码） |
| 再装 Pillow（推荐） | 以上 + JPG/JPEG、BMP、WebP、TIFF，动图逐帧合成、缩放更平滑 |

```bash
python -m pip install pillow     # 可选,但强烈建议装
```

大图/长动图会按内存上限自动缩小（单帧长边 ≤1600，全部帧 ≤96 MB，最多 600 帧），
缩放时铺满画布并居中裁切，不会变形。

### 字段表 / 日志为什么能透

Tk 的表格（`ttk.Treeview`）和文本框（`tk.Text`）只有实心底色，**没法做控件透明**。
为了让壁纸真的透出来，主窗口这两块换成了自绘 Canvas（`GlassTable` / `GlassLog`）：

- 壁纸按**整窗一张图**取自己那一块来画，所以表格和日志拼起来是一张完整的图，不会各缩一遍；
- 壁纸铺最底层（再多压暗一档），文字直接画在上面，动图也会跟着动；
- 滚轮滚动、右侧滚动条照常；`Ctrl+C` 复制整张表 / 整个日志；
- 代价：没有逐行选中高亮（数据本来就是每帧整表刷新），日志也不能拖选某一段。

### 整窗半透明（实验）

顶部的按钮、下拉框、标签都是 Tk 原生控件，只有实心底色，图片透不到它们后面。
勾上「背景图 ▾ → 整窗半透明(实验)」后换了个做法：在主窗口**背后**再放一个无边框窗口铺壁纸，
主窗口整体调成 82% 不透明 —— 于是壁纸从包括顶部在内的所有面板底下透出来。

```text
能：整窗都是壁纸，顶部面板也带图，不用改任何控件
不能：文字也跟着 82% 淡一点；拖窗口时背后那层可能稍微跟不上（松手就对齐）
```

该选项记在配置里（`glass`），关掉就是原来的实心面板。

### VOFA 窗口不再自动弹

启动时不会自动打开 VOFA+ 波形，收到数据时也不会自己弹；要看得点工具栏「VOFA 波形」。
电机调试、舵轮调试窗口同理，都是手动打开。


## 记录与导出

接收时默认记录，工具栏：

```text
☑ 记录数据          取消勾选后只显示、不写入记录
导出 CSV            解码后的字段表
导出原始 CSV        每帧一行:时间/端口/协议/长度/HEX
导出原始 HEX        纯文本,便于 grep 或发给厂商
导出文档(MD)        统计 + 事件 + 最后一帧快照
```

原始数据保存在内存，默认最多 20 万帧；导出后可用 Excel / 文本编辑器打开。

### 极简帧里的 IMU

固件 `Debug_Send_MotorPid()` 在 70 B 帧后追加 24 B：

```text
[60..63]  f32 roll   rad
[64..67]  f32 pitch  rad
[68..71]  f32 yaw    rad
[72..75]  f32 gx     rad/s
[76..79]  f32 gy     rad/s
[80..83]  f32 gz     rad/s
```

整帧：

```text
8 字节头 + 84 字节 payload + 2 字节 CRC = 94 字节
```

旧 70 B 帧仍兼容，界面显示“旧固件无 IMU 块”。

### 极简帧里的遥控器

在 IMU 后继续追加 32 B：

```text
[84]      online
[85]      左拨杆 1/2/3
[86]      右拨杆 1/2/3
[87]      鼠标左键
[88]      鼠标右键
[90..91]  u16 键盘位图
[92..101] 5 × i16 原始杆量(中心 0,约 ±660)
[102..107] 3 × i16 鼠标 x/y/z
[108..109] u16 掉线次数
[112..115] u32 累计收帧数
```

整帧：

```text
8 + 116 + 2 = 126 字节
```

界面显示在线状态、左右拨杆原始档位（1/2/3）、五路杆量原始值、鼠标、键盘按下的键。  
旧 94 B 帧仍兼容，显示“旧固件无遥控器块”。

### 极简帧里的 IM6014

在遥控器块后继续追加 32 B：

```text
[116]      online
[117]      status(0=锁定 1=FOC)
[120..123] u32 MError32
[124..127] f32 pos(rad)
[128..131] f32 w(rad/s)
[132..135] f32 t(N·m)
[136..139] f32 temp(℃)
[140..143] f32 voltage(V)
[144..145] u16 recv_crc_err
[146..147] u16 mwarn
```

整帧：

```text
8 + 148 + 2 = 158 字节
```

界面显示 IM6014 在线状态、位置、速度、反馈力矩、温度、电压、错误码和 CRC 错误数。  
当前固件按 `UART2 / 4 Mbps / ID=1 / 力矩模式 / 0.5 N·m` 发命令。

### 角度归一化

解码/记录/显示时：

```text
pos、pos_target  → 归一化到 [0, 2π)，即 0° ~ 360°
pos_error        → 最短角误差 [-π, π]，即 -180° ~ +180°
```

CSV 同时保留原始值：

```text
pos_raw / pos_target_raw / pos_error_raw
```

这样超过 360° 后不会一直累加，看闭环结果更直观；需要分析多圈时看 raw 字段。

## 跨平台修正

Windows 版原来的坑已处理：

- 不再硬编码 `COM1..COM32`；先在 daemon 线程里 `comports()`，超时才兜底；
- Ubuntu 用 `/dev/ttyUSB*`、`/dev/ttyACM*`、`/dev/serial/by-id/*` 兜底；
- 配置文件改为 `~/.debug_assistant.json`；
- 导出目录：Ubuntu `~/调试助手导出`，Windows 程序旁 `导出/`；
- 异常日志：Ubuntu `~/调试助手_error.log`；
- 字体按平台回退（Windows 微软雅黑，Ubuntu DejaVu Sans）；
- 去掉 Windows 专用 `CREATE_NO_WINDOW` / COM 默认值对 Linux 的影响。

## 快速运行

### Windows

```powershell
python -m pip install pyserial
python .\windows\调试助手.py
```

打包：

```powershell
cd windows
powershell -ExecutionPolicy Bypass -File .\build.ps1
```

生成 `windows\dist\调试助手.exe`。

### Ubuntu 22.04 x64

```bash
sudo apt install -y python3 python3-tk python3-pip
python3 -m pip install --user pyserial
sudo usermod -aG dialout $USER   # 重新登录
cd ubuntu/x64
python3 调试助手.py
```

### Ubuntu 22.04 arm64

```bash
cd ubuntu/arm
python3 调试助手.py
```

`x64/`、`arm/` 下都有 `run.sh`、`build.sh`、`README.md`；  
`ubuntu/` 下另有 `调试助手-ubuntu22.04-x64.tar.gz` 和 `...-arm64.tar.gz` 源码包。  
**PyInstaller 不能跨架构**：x64 包在 x86_64 上构建，arm 包在 arm64 上构建。

### Android

Kivy 版，手机用 USB-OTG 线直连 CH340/CP2102 等，不需要电脑：

```bash
cd android
python -m pip install -r requirements.txt
python main.py --demo      # 桌面先看界面(合成帧,不需要硬件)
buildozer -v android debug # 打包 APK(只能在 Linux/WSL 里跑)
```

细节、真机注意事项、协议同步和已知限制见 [android/README.md](android/README.md)。

## 串口接线

```text
CH340 TX  → 板子 UART7 RX (PE7)
CH340 RX  → 板子 UART7 TX (PE8)
GND       → GND
```

只接 CH340 RX 只能看遥测，发不了 0x5C 命令。

## 协议

### 0x5A 遥测

```text
5A | len? | cmd=0x0102 | seq | payload | crc16
```

极简电机帧 payload：

```text
u8 motor_id
u8 flags
f32 joint_rpm
f32 motor_rpm
f32 target_rpm
f32 feedback_rpm
f32 error_rpm
f32 torque
f32 pos
f32 pos_target
f32 pos_error
u8 pe2_level
u8 pe3_level
```

### 0x5C 命令

```text
5C | cmd | seq_hi | seq_lo | payload
```

| cmd | 功能 | payload |
|---|---|---|
| 0x01 | 速度 rpm | int16 LE |
| 0x02 | 角度 mdeg | int32 LE |
| 0x03 | PID 六路 | float×6 LE |
| 0x04 | 回读 PID | 无 |
| 0x05 | 停止/失能 | 无 |
| 0x06 | 摩擦轮 | u8 编号 + int16 rpm |
| 0x07 | 底盘速度（本工具定义） | int16 vx(mm/s) + int16 vy(mm/s) + int16 wz(0.01 rad/s) |
| 0x08 | 云台角度（本工具定义） | int32 yaw(0.001°) + int32 pitch(0.001°) |

## 底盘 / 云台调试

工具栏「底盘」「云台」两个按钮，各自一个窗口：

```text
底盘: ↑↓←→ 点动 / ↺↻ 左转右转(按住走、松开停)  +  目标 vx/vy/wz 持续下发  +  停止
      下方是俯视示意:车身 + 四轮 + 粗箭头(下发速度) + 细虚线箭头(实测) + 绿弧(转动方向)
      每个轮子旁边的小箭头是该轮转速,底部一行给出解算结果(mm/s)
云台: yaw/pitch 绝对角下发 + 回中  +  ↑↓←→ 点动(每 150ms 走一步)
      下方是罗盘(实际/目标指针 + 误差) + 右侧 pitch 标尺(-90°~+90°)
```

**底盘类型(麦轮 / 全向轮 / 舵轮)**:窗口顶上选型号,轮速会**自动解算**,不用手算:

```text
麦轮 / 全向轮 (X 型 45° 辊子,半轮距 tr、半轴距 wb,单位 mm)
  FL = vx - vy - (tr+wb)·w      FR = vx + vy + (tr+wb)·w
  RL = vx + vy - (tr+wb)·w      RR = vx - vy + (tr+wb)·w         (w 单位 rad/s)

舵轮 (每个轮子独立转向)
  v_ix = vx - w·y_i             v_iy = vy + w·x_i
  轮速 = hypot(v_ix, v_iy)      转向角 = atan2(v_iy, v_ix)
```

- 默认 `半轮距 200 mm / 半轴距 200 mm`,窗口里可改,记在配置里(`chassis_type/chassis_track/chassis_base`);
- 舵轮的轮子会按转向角画出来,并标 `FL +39°` 这样的角度;麦轮/全向轮画成直轮 + 45° 辊子示意;
- 解算只是**显示**,下发的还是 `vx/vy/wz`(0x07)——除非你固件要轮速,那就把窗口底部「自定义 HEX」里按 0x07 拼轮速发。

- **命令号是 0x07 / 0x08**（0x5C 协议里本工具自己定的两个），payload 见上表；
  固件不是这么实现的，改文件顶部两行就行：

  ```python
  CHASSIS_CMD = 0x07      # 底盘: <h vx> <h vy> <h wz>
  GIMBAL_CMD = 0x08       # 云台: <i yaw> <i pitch>   (0.001°)
  ```

- 两个窗口底部都有 **「自定义 HEX」**，协议对不上时直接发原始帧；
- 点动是「按住走、松开立刻发 0」，底盘每 200ms 重发一次，云台每步加 `step`；
- 状态栏显示的是收到的遥测：有 0x5A 极简帧就看 0x201/0x202 + IMU，
  有旧 210B 调试帧就看底盘 vx/vy/w、云台 yaw/pitch。

## 注意

- Ubuntu 串口权限用 `dialout` 组，不要 `sudo` 跑 GUI。
- 921600 波特率需要 CH340 支持；部分廉价模块不稳。
- 自动匹配会短暂占用所有串口；匹配完成后手动“打开”目标口。
- 导出文件默认在 `~/调试助手导出`（Ubuntu）或程序旁 `导出/`（Windows）。
