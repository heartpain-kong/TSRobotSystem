# bsp_fdcan — FDCAN 总线 BSP：三路总线「一路一个回调」的收发接口（经典 CAN 帧）

把 CubeMX 生成的 `hfdcan1` / `hfdcan2` / `hfdcan3` 收成一套统一收发接口：上层电机驱动
（大疆 / 达妙 / 灵足）**每路总线只登记一个回调**，再按帧 id 自己分发，不用碰 HAL 的
FIFO 与过滤器。

| 文件 | 内容 |
|---|---|
| `bsp_fdcan.h` | 类型与声明（模块对外接口）|
| `bsp_fdcan.cpp` | 过滤器配置、收发实现与中断分发 |

## 干什么

* **`can_filter_init()`** —— 给 `hfdcan1/2/3` 各配一个标准帧「全通过」过滤器
  （`MASK` + `ID1`/`ID2` = 0），并把**不匹配帧也收进对应 FIFO**
  （FDCAN1 / FDCAN3 → RX FIFO0，FDCAN2 → RX FIFO1）。
* **`bsp_can_init()`** —— 登记 `CAN_Callback`、开接收中断、`Start` 总线
  （每路只用 `system_can[0..2]` 标志 Start 一次）。
* **`HAL_FDCAN_RxFifo0Callback` / `HAL_FDCAN_RxFifo1Callback`** —— 覆盖 HAL 弱回调，
  按 `Instance` 定位总线，`while` 循环取空 FIFO 并逐帧把 `Rx_Header` / `Rx_Buffer`
  交给该总线的回调。

## 怎么用

```c
static void CAN1_Callback(FDCAN_RxHeaderTypeDef Header, uint8_t *Buffer)
{
    const uint32_t id = Header.Identifier;   // 回调里按 id 找电机
}

can_filter_init();                       // 上电一次:配过滤器/全局过滤
bsp_can_init(&hfdcan1, CAN1_Callback);   // 每路各注册一次(内部会 Start)

fdcan_send_data_stand(&hfdcan1, 0x201, data, 8);       // 标准帧
fdcan_send_data_Exten(&hfdcan1, id, data, 8);          // 扩展帧,不等空位
fdcan_send_data_Exten_wait(&hfdcan1, id, data, 8, 10); // 扩展帧,等空位
```

## 接口一览

| 接口 | 作用 | 返回值 |
|---|---|---|
| `CAN_Callback` | 回调类型：`void (*)(FDCAN_RxHeaderTypeDef Header, uint8_t *Buffer)` | — |
| `struct CAN_Manage_Object` | 每路总线的句柄：`CAN_Handler`、`Callback_Function`、`Rx_Header`、`Rx_Buffer[8]` | — |
| `extern bool system_can[3]` | 三路总线的 Start 记录 | — |
| `void can_filter_init()` | 上电一次：配过滤器 + 全局过滤（不匹配帧进对应 FIFO）+ FIFO 水位 | void |
| `void bsp_can_init(FDCAN_HandleTypeDef *hfdcan, CAN_Callback Callback_Function)` | 登记回调、开接收中断、Start 总线 | void |
| `uint8_t fdcan_send_data_stand(hfdcan, id, data, len)` | 标准帧发送（**不等空位，失败即丢帧**）| 0 = 已进 TX FIFO；1 = 入队失败（TX FIFO 满，或该总线还没 Start）|
| `uint8_t fdcan_send_data_Exten(hfdcan, id, data, len)` | 扩展帧发送（**不等空位，失败即丢帧**）| 0 = 已进 TX FIFO；1 = 入队失败（TX FIFO 满，或该总线还没 Start）|
| `uint8_t fdcan_send_data_Exten_wait(hfdcan, id, data, len, timeout_ms)` | 扩展帧发送（**等 TX FIFO 空位，带超时**，失败必须上报）| 0 = 已进 TX FIFO；1 = 超时 / 参数错（这一帧没发出去）|

`len` 一律传**「字节数」**，不要传 DLC 编码；经典帧 ≤ 8。

## 注意

* **回调运行在 FDCAN 接收中断里**（`HAL_FDCAN_RxFifo0Callback` / `RxFifo1Callback`），
  必须短小，**不可阻塞或延时**。
* **回调的 `Buffer` 就是 `CANx_Manage_Object.Rx_Buffer`**：每路总线只有这一个 8 字节缓冲，
  回调返回后即被下一帧覆盖 —— **要留数据必须当场拷走**。
* **单帧最多 8 字节**（CubeMX 配的是 `FDCAN_FRAME_CLASSIC`，不是 CAN FD）。
* **总线与 FIFO 的绑定**：FDCAN1 / FDCAN3 走 RX FIFO0，FDCAN2 走 RX FIFO1。
  注册中断的 FIFO 必须和 `can_filter_init()` 里的过滤目标一致，否则收不到。
* **FDCAN2 的 `RxFifo0ElmtsNbr = 0`**，只能用 FIFO1（`hfdcan2` 已如此配置）。
* **TX FIFO 只有 8 槽**（见 `fdcan.c` 的 `TxFifoQueueElmtsNbr`）；不等空位的发送在 FIFO
  满时返回 1 并丢掉该帧，要保证送达请用 `fdcan_send_data_Exten_wait()`（见下节）。
* **`bsp_can_init()` 的回调登记是「首个非空生效」**：同一路再次传入不同回调**不会覆盖**，
  静默无效；重复调用只会补 Start / 通知标志。
* **两个 `RxFifo*Callback` 是 HAL 弱符号的工程内唯一实现**，别处不能再定义；它们只处理
  已 `ActivateNotification` 的 FIFO（FDCAN1/3 = FIFO0，FDCAN2 = FIFO1）。

## TX FIFO 与两种发送语义

TX FIFO 深度只有 **8**（CubeMX 的 `TxFifoQueueElmtsNbr = 8`），而灵足一次“置零”在一条
总线上要灌多帧（每台电机 lose / 置零 / 保存 / lose）——用**不等空位**的版本会把超出的帧
**静默丢掉**，真机现象是“每条总线只有前几个电机生效”。

所以需要送达的场合用 `fdcan_send_data_Exten_wait(hfdcan, id, data, len, timeout_ms)`：

* 参数校验：`hfdcan == nullptr || data == nullptr` 直接返回 1（参数错）。
* 空位判断**直接查 `TFQF` 位**（`hfdcan->Instance->TXFQS & FDCAN_TXFQS_TFQF`），
  不调 HAL 接口 —— 避免每次失败都往 `ErrorCode` 里塞 `FIFO_FULL`。
* 超时用 `HAL_GetTick()`。

> ⚠️ 这个自旋**依赖 `HAL_GetTick()`（SysTick）**：若在中断里、或把中断屏蔽到 SysTick
> 优先级之上再调用它，**超时永远不会到 ⇒ 死循环**。等待期间也不让出 CPU，所以**只在任务
> 上下文里用**（灵足的 zero / 参数操作就是任务里调的）。

## 实现要点

### 过滤器配置（`can_filter_init()`）

对 `hfdcan1`、`hfdcan2`、`hfdcan3` 各做一次同样的三连调用，只有目标 FIFO 不同：

| 调用 | hfdcan1 | hfdcan2 | hfdcan3 |
|---|---|---|---|
| `HAL_FDCAN_ConfigFilter` | `FDCAN_FILTER_TO_RXFIFO0` | `FDCAN_FILTER_TO_RXFIFO1` | `FDCAN_FILTER_TO_RXFIFO0` |
| `HAL_FDCAN_ConfigGlobalFilter` | `FDCAN_ACCEPT_IN_RX_FIFO0` | `FDCAN_ACCEPT_IN_RX_FIFO1` | `FDCAN_ACCEPT_IN_RX_FIFO0` |
| `HAL_FDCAN_ConfigFifoWatermark` | `FDCAN_CFG_RX_FIFO0`, 1 | `FDCAN_CFG_RX_FIFO1`, 1 | `FDCAN_CFG_RX_FIFO0`, 1 |

过滤器本身：`IdType = FDCAN_STANDARD_ID`、`FilterIndex = 0`、`FilterType = FDCAN_FILTER_MASK`、
`FilterID1 = 0x00`、`FilterID2 = 0x00`（全通过）；全局过滤的远端帧策略为
`FDCAN_REJECT_REMOTE`。

### 发送（可重入的模板拷贝）

每个发送接口都先把发送头**模板拷一份到局部变量**，再改写 `Identifier` / `DataLength`，
然后调 `HAL_FDCAN_AddMessageToTxFifoQ()`：

* 标准帧模板 `fdcan_tx_stand_header`：`IdType = FDCAN_STANDARD_ID`、
  `TxFrameType = FDCAN_DATA_FRAME`、`ErrorStateIndicator = FDCAN_ESI_ACTIVE`、
  `BitRateSwitch = FDCAN_BRS_OFF`、`FDFormat = FDCAN_CLASSIC_CAN`、
  `TxEventFifoControl = FDCAN_NO_TX_EVENTS`、`MessageMarker = 0`。
* 扩展帧模板 `fdcan_tx_Exten_header`：同上，只有 `IdType = FDCAN_EXTENDED_ID`。

### 中断分发

`HAL_FDCAN_RxFifo0Callback` 处理 FDCAN1 与 FDCAN3，`HAL_FDCAN_RxFifo1Callback` 处理
FDCAN2；各自 `while (HAL_FDCAN_GetRxMessage(...) == HAL_OK)` 取空 FIFO，回调函数指针非空
才调用（传入 `Rx_Header` 与 `Rx_Buffer`）。

## 历史记录（两处曾记录的不一致，现已修复）

1. **发送接口的可重入性 —— 已修**。早先两个发送头是 `static` 且被每次调用直接改写，
   多任务并发会互相踩 id/len。现在两个头改成了 **`static const` 只读模板**，
   `fdcan_send_data_stand()` / `fdcan_send_data_Exten()` / `fdcan_send_data_Exten_wait()`
   **都先把模板拷到局部变量**再改写 `Identifier` / `DataLength` ⇒ **三个发送接口现在都是可重入的**，
   而且 `Exten` 与 `Exten_wait` 之间也不再共用可变状态。
2. **`CAN1/2/3_Manage_Object` 的外部链接 —— 已修**。这三个对象只在 `bsp_fdcan.cpp` 内部使用，
   现已加 `static`（原来没有 `static`，属于"没有头文件声明但实际外部链接"的意外符号）。

> 另外：`bsp_can_init()` 的回调登记是"**首个非空生效**"（`if (Callback_Function == nullptr)`），
> 重复注册不会覆盖；`HAL_FDCAN_Start()` 由 `system_can[0..2]` 守着只做一次。

## 相关

* 依赖 `fdcan.h`（`hfdcan1/2/3` 的 extern 声明）与 `stm32h7xx_hal_fdcan.h`。
* 被 `motor_mit.h` / `motor_DJ.h` / `motor_DM.h` / `motor_LZ.h` 引入。
* 使用示例见 `User\Device\motor\电机使用说明.md` 第 2 节「接收」。
* 实现与回调分发见 `bsp_fdcan.cpp`；各函数的细节说明见其上方 `@brief`。
