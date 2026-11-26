# dev_log — 运行日志（内存环形 → 外部 Flash 追加落盘 + 可选实时控制台）

文件：`dev_log.h` / `dev_log.cpp`

## 1. 为什么这么设计

外部 Flash（W25Q64）**写一页要 ~0.4ms，而且是阻塞的** ⇒ 绝不能直接写进 1 kHz 控制任务。
所以拆成两段：

```
控制任务(快)                     低优先级/低频(慢)
Log_Printf / Log_Record   ──►  内存环形缓冲  ──►  Log_Task()  ──►  W25Q64 LOG 区
   （µs 级,只 memcpy）            2 KB               每 1ms 最多 1 条(限速)
```

环形满了、或者记录区满了 ⇒ **直接丢弃**并把量记在 `Log_Get_Dropped_Bytes()` 里
—— **绝不阻塞控制** ✓

## 2. 数据存在哪（用的是 `User/Bsp/flash` 划好的 LOG 区）

| 区域 | 地址 | 大小 | 内容 |
|---|---|---|---|
| 头部扇区 | `0x001000` | 4 KB | **256 个 16B 检查点槽**（磨损均衡：256 槽写满才擦一次）|
| 记录区 | `0x002000` | ≈1 MB | 记录；**一条固定 256B = 1 个 Flash 页** |

**记录格式**（小端）：

```
[0]'L' [1]'G'  |  [2..3] len  |  [4..5] seq  |  [6..7] crc16(payload)  |  [8..] payload(最多 248B)
```

**检查点槽**（16B）：

```
[0..3] magic 0x4C4F4731 | [4..7] data_offset | [8..11] seq | [12..13] crc16 | [14..15] 0xFFFF
```

* 每 `LOG_CHECKPOINT_RECORDS = 32` 条落一个检查点 ⇒ **掉电重启能接着记** ✓
* `Log_Init()` 只读头部（8 次 512B 读 ✓）+ 从检查点往后补扫 **≤32 条** ⇒ **启动很快** ✓
  （不会去啃 1 MB，那要十几秒 ✗）
* 记录区**按需擦扇区**：用到哪个扇区才擦哪个 ✓

## 3. API

| 接口 | 说明 |
|---|---|
| `Log_Init()` | 初始化（**必须在 `Flash_Init()` 成功之后**）|
| `Log_Task()` | 周期跑：把环形里攒的帧落盘（内部限速 1 条/ms）|
| `Log_Printf(level, fmt, ...)` | 文本日志，自动加 `[秒.毫秒] I:/W:/E:` 前缀 |
| `Log_Record(data, len)` | 一帧原始二进制（比如整帧 IMU 采样）|
| `Log_Set_Console(cb)` / `Log_Console_Enable(b)` | 注册/打开实时控制台（比如 USB CDC）|
| `Log_Is_Ready()` / `Log_Is_Full()` | 状态 |
| `Log_Get_Written_Bytes()` / `Log_Get_Seq()` | 已落盘字节数 / 条数 |
| `Log_Get_Dropped_Bytes()` | 丢了多少（环形满 / 盘满 / 写失败）|
| `Log_Read(offset, buf, len)` | 读回记录区（给上位机 dump）|
| `Log_Erase_All()` | 清空（擦头部 + 记录区首扇区，≈50ms）|
| `Log_Float2Str(str, v)` | float → 字符串（因为 **`%f` 用不了**，见下）|

## 4. ⚠️ 三条纪律

1. **不要用 `%f`** ✗：newlib-nano 默认没开浮点 printf，`%f` 会打印成空/乱码。
   浮点先用 `Log_Float2Str(buf, v)` 转成字符串再 `%s` —— 这正是参考工程
   `dev_log` 里 `Float2Str()` 存在的原因。
2. **`Log_Task()` 只能在同一个任务里调**：它和 `bsp_flash` 共用那颗 W25Q64 的状态机
   （和 `User/Bsp/flash/bsp_flash.md` 里那条纪律一样）。
3. **一帧最多 `LOG_FRAME_MAX`(240B)**，超了会被丢弃并计入 dropped ✗。

## 5. 用法（已接进 `Robot_Init()` / `Robot_Task()`）

```cpp
/* Robot_Init()：必须在 Flash_Init() 之后 */
log_init_ok = Log_Init();

/* 任意任务：写日志(不阻塞) */
Log_Printf(LOG_LEVEL_INFO, "boot: flash=%d mV", 24000);
Log_Printf(LOG_LEVEL_ERROR, "motor %d offline, err=%d", id, err);
Log_Record(&imu_frame, sizeof(imu_frame));        // 二进制整帧

/* Robot_Task()：低频落盘 */
Log_Task();

/* 实时看(USB CDC)：freertos.c 的 StartDefaultTask 里 USB 初始化之后打开 */
Log_Set_Console(My_Usb_Console);      // 内部调 CDC_Transmit_HS
Log_Console_Enable(true);
```

## 6. 把数据读出来

* 在线：`Log_Read(offset, buf, len)`（`offset` 从 0 到 `Log_Get_Written_Bytes()`）✓
* 离线（推荐）：把 `Log_Get_Written_Bytes()` 作为"有效长度"，从 `0x002000` 起
  按 256B 步进解析 ⇒ 校验 `'L''G'` + `crc16(payload)` ⇒ 收集 `payload` ✓
  遇到第一条校验不过的就停（后面是空白/旧数据）✓

## 7. 上板自检

```
① log_init_ok == true
② 写 10 条日志 ⇒ Log_Get_Seq() 应增加,Log_Get_Dropped_Bytes() == 0
③ 断电重启 ⇒ Log_Get_Seq() 从上次的值**接着涨**(不是从 0) ⇒ 说明检查点/续记 OK
④ 用 Log_Read(0, buf, 256) 读第一条:buf[0]=='L' && buf[1]=='G' ✓
⑤ 故意灌 3000 条(超过 2KB 环形) ⇒ dropped 应 > 0,但主循环**不能卡**
```


---

## ★ 存储布局与"上电链"(v2)

### 区划

| 区域 | 地址（相对 FLASH_LOG_ADDR=0x001000）| 大小 | 说明 |
|---|---|---|---|
**检查点区** | `0x000000` | 4 KB（1 扇区）| 128 个 **32 B** 槽，循环写（写满擦整扇区 ✓ 磨损均衡）|
**数据区** | `0x001000` | ≈1 MB − 4 KB | 256 B/条记录，**环形**写入 ✓ |

### 检查点槽（32 B，v2：magic `0x4C4F4732`）

| 偏移 | 类型 | 内容 |
|---|---|---|
0 | u32 | magic（v1 的 `...31` 不再被识别 ⇒ 会当"全新"重来一次 ✓）|
4 | u32 | **本次上电编号** boot_id（每次上电 +1）|
8 | u32 | **本次上电的当前写地址** cur_offset |
12 | u32 | **本次上电的首地址** cur_start（本次第一笔写在哪）|
16 | u32 | **上一次上电的停止位置** prev_offset |
20 | u32 | **上一次上电的首地址** prev_start |
24 | u16 | CRC16（覆盖 0~23，反射 0x8408/init 0 ✓）|
26~31 | — | 0xFF |

⇒ `prev_start ~ prev_offset` 就是**上一次上电写的那一整段** ✓ —— 查错误时直接圈出范围 ✓ 这就是"保险" ✓

### 上电流程（`Log_Init`）

1. 扫 128 个槽 ✓，取 **boot_id 最大、同 boot_id 取槽号最大** 的那条 ✓（= 最后一次写入 ✓）
2. 恢复：`cur = 检查点里的 cur_offset` ✓，`boot_id = 上次 + 1` ✓
3. 把"上一次"的链搬过来：`prev_offset = 上次 cur_offset` ✓，`prev_start = 上次 cur_start` ✓
4. 从检查点位置**往后补扫**最多 32 条记录 ✓（对齐到真正尾部 ✓）
5. 打一行日志（`LOG_DEBUG_BOOT_CHAIN=1`）：
   ```
   log: boot=7 start=0x1A200 prev_start=0x08200 prev_stop=0x1A000 wrap=1
   ```
   ⇒ 一眼看出**本次从哪开始写、上次写的是哪一段、覆盖过几次** ✓

### 写满之后（回绕）

`s_data_offset + 256 > 1MB` ⇒ **`s_data_offset = 0` + 擦掉首扇区 + `wrap_count++`** ✓ ⇒ 从头继续写 ✓（不再是"停住"✗）
⇒ `Log_Is_Full()` 现在表示"**已经开始覆盖旧数据**" ✓（`wrap_count > 0`）

### 查询接口

| 接口 | 含义 |
|---|---|
`Log_Get_Boot_Id()` | 本次上电编号 |
`Log_Get_Boot_Start()` | 本次首地址（相对 LOG_DATA_ADDR）|
`Log_Get_Prev_Offset()` | 上次停止位置 |
`Log_Get_Prev_Start()` | 上次首地址 |
`Log_Get_Wrap_Count()` | 回绕次数 |