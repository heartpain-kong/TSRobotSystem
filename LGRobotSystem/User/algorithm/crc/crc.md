# crc — 两套 CRC：CRC16-CCITT（宇树 8010）与 CRC32/MPEG-2（宇树 6014）

来源：`crc_ccitt.h`、`crc32.h`、`crc32.cpp` 顶部的文件说明注释。

---

## 1. 定位

宇树（UNITREE）两种电机协议的帧校验：`GO-M8010-6` 用 **CRC16-CCITT**，`IM6014` 用 **CRC32**。

> ⚠️ **两套 CRC 必须分清，别混用**：不同协议、不同多项式、不同覆盖范围、不同长度约束。

| 项 | CRC16-CCITT（8010） | CRC32/MPEG-2（6014） |
|---|---|---|
| 定义位置 | `crc_ccitt.h`（**纯头文件**，只有一个文件） | `crc32.h` 声明 + `crc32.cpp` 实现 |
| 多项式 | 反射多项式 `0x8408`（= x^16 + x^12 + x^5 + 1，隐式 x^16） | `0x04C11DB7` |
| 初值 | **0**（不是 `0xFFFF`，函数内部从 `tmp = 0` 开始） | **`0xFFFFFFFF`** |
| 末尾异或 | 无 | **无**（返回值直接与小端写在线上的 4 字节比较） |
| 位序 / 方向 | 表驱动（取自 Linux 内核 `lib/crc-ccitt.c`），逐字节右移查表 | **MSB-first**，即 CRC-32/MPEG-2（**不是**以太网那个反射的 CRC-32） |
| 发送帧覆盖范围 | 前 **15 B**，结果小端写在第 16、17 B | 前 **16 B（含包头）**，结果小端写在第 16–19 B |
| 反馈帧覆盖范围 | 前 **14 B**，结果小端写在第 15、16 B | **20 B（从 mode 字节起，即 Byte2..21，不含包头）**，结果小端写在第 22–25 B |
| 分组方式 | 逐字节，无分组约束 | **按 4 字节一组**（`word_count = len / 4u`），**不足 4 字节的尾巴不参与计算** |
| 长度约束 | 任意字节数 | **`len` 必须是 4 的倍数**（6014 用到的 16 / 20 都满足） |
| 唯一调用者 | `motor_YS.cpp`（8010 收发） | `motor_YS.cpp`（6014 收发） |

调用点（`User/Device/motor/motor_YS/motor_YS.cpp`，长度宏在 `motor_YS.h`）：

| 场景 | 调用 | `len` 来源 |
|---|---|---|
| 8010 组帧 | `crc_ccitt(motor_YS_send_buf, Crc_Send_Len)` | `MOTOR_YS_CRC_SEND_LEN = 15u` |
| 8010 解帧 | `crc_ccitt(_data, Crc_Recv_Len)` | `MOTOR_YS_CRC_RECV_LEN = 14u` |
| 6014 组帧 | `crc32_mpeg2(b, Crc_Send_Len)` | `MOTOR_6014_CRC_SEND_LEN = 16u` |
| 6014 解帧 | `crc32_mpeg2(&_data[2], Crc_Recv_Len)` | `MOTOR_6014_CRC_RECV_LEN = 20u` |

## 2. CRC16-CCITT（`crc_ccitt.h`）

### 怎么用

```cpp
#include "crc_ccitt.h"

uint16_t crc = crc_ccitt(buf, 15);        // 覆盖前 15 字节
buf[15] = (uint8_t)(crc & 0xFF);          // 小端写回
buf[16] = (uint8_t)(crc >> 8);
```

### 接口

| 函数 / 对象 | 作用 | 参数 | 返回值 | 注意 |
|---|---|---|---|---|
| `crc_ccitt_table[256]` | 每个可能字节的 CRC 查表（512 B） | — | — | `static uint16_t const`；多项式可在表项 128（`0x8408`）看到 |
| `crc_ccitt_byte(uint16_t crc, const uint8_t c)` | 单字节推进 | `crc` 前一状态；`c` 本字节 | `uint16_t` 新状态 | `(crc >> 8) ^ crc_ccitt_table[(crc ^ c) & 0xff]`；`static` |
| `crc_ccitt(uint8_t const *buffer, size_t len)` | 计算整段缓冲的 CRC16 | `buffer` 数据指针；`len` 字节数 | `uint16_t` | `static`；**初值 0**；参数顺序是 (指针, 长度)，别传错 |

### 注意

* 初值是 **0**（不是 `0xFFFF`）。
* 本文件是**纯头文件**（函数是 `static`）：谁 include 谁就得到一份 512 B 的表 + 一份函数体。
  目前只有 `motor_YS.cpp` 用它，无所谓；以后若多个文件要用，建议改成 `crc_ccitt.c` + `extern` 声明。
* 参数是 `(const uint8_t*, size_t)`，不是引用。

## 3. CRC32/MPEG-2（`crc32.h` + `crc32.cpp`）

### 接口

| 函数 | 作用 | 参数 | 返回值 | 注意 |
|---|---|---|---|---|
| `uint32_t crc32_mpeg2(const uint8_t *data, size_t len)` | 计算宇树 IM6014 协议的 CRC32 | `data` 数据指针；`len` 字节数 | `uint32_t` CRC32 值 | **`len` 必须是 4 的倍数**，否则尾部不足 4 字节不参与计算；**未做末尾异或**，可直接与小端写在线上的 4 字节比较；`data == nullptr` 时直接返回初值 `0xFFFFFFFF` |

### 实现说明（`crc32.cpp`）

* 这里是官方 `crc32_byte_by_byte()` 的**逐行等价实现**（不使用 1 KB 查表）。
* 官方同一个头文件里给了查表版和逐位版两个函数，数学上等价 —— 查表版按
  `(data[3], data[2], data[1], data[0])` 的顺序喂字节，逐位版把 4 字节拼成一个大端 word 后从 MSB 起折叠，结果相同。
* 用逐位版的好处：**省 1 KB 常量**、也**不会抄错表**。
* 参数照抄官方 `N6014B_HALDemo/App/crc_ccitt.h`：多项式 `0x04C11DB7`、初值 `0xFFFFFFFF`、
  无末尾异或、MSB-first。
* ⚠️ 官方就是按 `len / 4` 组处理，**不足 4 字节的尾巴不计入** —— 所以 `len` 必须是 4 的倍数，
  6014 用到的 16 / 20 都满足。

### 计算流程（对应源码）

```
crc = 0xFFFFFFFF
polynomial = 0x04C11DB7
word_count = len / 4
对每一组 4 字节：
    word = (data[i*4+3] << 24) | (data[i*4+2] << 16) | (data[i*4+1] << 8) | data[i*4+0]
    对 word 的 32 个 bit（从 MSB 到 LSB）：
        msb_crc = crc & 0x80000000
        crc <<= 1
        if (msb_crc)      crc ^= polynomial
        if (word & xbit)  crc ^= polynomial
        xbit >>= 1
返回 crc（不做末尾异或）
```

## 4. 相关

* `motor_YS.cpp` —— 两套 CRC 的唯一调用者（8010 与 6014 的组帧 / 解帧）
* `User/Device/motor/电机使用说明.md` —— 宇树两种协议的完整帧格式与单位换算
