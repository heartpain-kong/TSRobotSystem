# 调试助手 — Windows

## 直接运行

```powershell
python -m pip install pyserial
python ..\调试助手.py
```

或双击 `调试助手.exe`（如果已打包）。

## 打包

```powershell
powershell -ExecutionPolicy Bypass -File .\build.ps1
```

输出：`dist\调试助手.exe`。

## 记录与导出

- `记录数据`：默认勾选；取消后只显示、不写入记录。
- `导出 CSV`：解码后的字段表。
- `导出原始 CSV`：每帧一行：时间 / 端口 / 协议 / 长度 / HEX。
- `导出原始 HEX`：纯文本，便于 grep 或发给厂商。
- `导出文档(MD)`：统计、状态变化、最后一帧快照。

角度列已归一化：

```text
pos / pos_target   [0, 2π)  (0°~360°)
pos_error          [-π, π]  (-180°~+180°)
pos_raw / pos_target_raw / pos_error_raw  原始值
```

## 注意

- 串口选 `COMx`（设备管理器里看）。
- CH340 驱动装好；TX/RX 交叉接。
- 只接 CH340 的 RX 只能看遥测，发不了 0x5C 命令。
