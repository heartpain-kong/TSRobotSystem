# User/模板 — 复制即用的骨架

> ⚠️ **`.tpl` 后缀是故意的** ✓ —— 这样这些模板**不会被编译进固件** ✗，只是给人看的原料。

## 怎么用

1. 复制到目标目录（按层次放）：
   | 模板 | 复制到 |
   |---|---|
   | `bsp_template.h.tpl` / `.cpp.tpl` | `User/Bsp/<模块>/bsp_<模块>.{h,cpp}` |
   | `device_template.h.tpl` / `.cpp.tpl` | `User/Device/<模块>/dev_<模块>.{h,cpp}` |
   | `algorithm_template.h.tpl` / `.cpp.tpl` | `User/algorithm/<模块>/alg_<模块>.{h,cpp}` |
   | `task_template.h.tpl` / `.cpp.tpl` | `User/App/<模块>/<模块>_task.{h,cpp}` |
   | `module_template.md.tpl` | 同目录的 `<同名>.md` |
2. **去掉 `.tpl` 后缀** ✓
3. 全文替换占位符：`xxx` → 你的文件名、`XXX` → 你的宏、`Xxx` → 你的函数/类型名
4. 按 `../代码架构框架.md` 第 5 节接线：**CMake include 列表** + **脚本 §5 清单** + **`Robot_Init()`** +（任务的话）**`OSTaskInit()` 与 `robot_def.h`**
5. 编译 + `tools\cubemx_post_export.ps1` 自检 + 按 md 的「上板自检」过一遍

## 每个模板里已经内置的约定

* `extern "C"`（C 侧能 include）+ 头文件保护宏
* 文件头 3 行注释 + "说明见同目录 md"
* `static_assert(sizeof(...) <= MSG_SLOT_SIZE)`（发话题的结构体必加 ✓）
* 消费者标准写法：`Msg_Is_New()` + `Msg_Latest()`（广播型，各自记 `seq`）
* 设备骨架：**中断只搬运 / 任务里组帧 / 有新数据才发布 / 记差错计数**
* 任务骨架：`Xxx_Init()` + `Xxx_Task()`（跑一拍）+ `Xxx_Get()`
* 异常上报：`Error_Set_Flag(ERROR_FLAG_MASK(...))`；关键量写 `Error_Get_Snapshot()`

> 详细规矩见 `../代码风格与模板.md`（命名/注释/单位/类写法）与 `../代码架构框架.md`（分层/任务/通信）。
