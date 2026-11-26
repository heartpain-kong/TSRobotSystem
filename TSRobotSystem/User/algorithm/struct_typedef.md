# struct_typedef — 全工程公用的基础类型别名

来源：`struct_typedef.h` 顶部的文件说明注释。文件内部那段"为什么不要自己 typedef 定宽整数"的注释仍留在源码里。

---

## 1. 定位

把 `fp32` / `fp64` / `bool_t` 这类短名字固定下来，所有模块统一使用，避免各处混用
`float` / `double` / `unsigned char`。

* 无依赖；被几乎所有 `User/` 下的文件包含。

## 2. 怎么用

```cpp
#include "struct_typedef.h"

fp32 x = 1.0f;   fp32 w = 3.14f;   bool_t ok = 1;
```

## 3. 定义内容

| 名字 | 实际类型 | 说明 |
|---|---|---|
| `bool_t` | `unsigned char` | ⚠️ **与 C++ 的 `bool` 不是一回事**（C++ 代码里直接用 `bool` 即可） |
| `fp32` | `float` | 单精度浮点 |
| `fp64` | `double` | 双精度浮点 |

定宽整数（`int8_t` / `uint16_t` / …）一律取自编译器自带的 `<stdint.h>`，
**不要在这里自己 typedef**。

## 4. 注意事项

* 定宽整数（`int8_t` / `uint16_t` / …）一律取自编译器自带的 `<stdint.h>`，
  **不要在这里自己 typedef** —— 会与 HAL 冲突：
  arm-none-eabi 的 newlib 中 `int32_t` 是 `long int`，若这里定义成 `signed int`，凡是同时包含
  HAL 头文件（链到 stdint）与本文件的编译单元都会报 `conflicting declaration` 错误。
* 本文件的 `bool_t` 是 `unsigned char`，与 C++ 的 `bool` 不是一回事（C++ 代码里直接用 `bool` 即可）。

## 5. 相关

* 无依赖；被几乎所有 `User/` 下的文件包含。
