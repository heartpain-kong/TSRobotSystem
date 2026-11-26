/* ============================================================================
 * dev_log.h — 运行日志(内存环形缓冲 → 外部 Flash 追加落盘 + 可选实时控制台)
 * 说明 / 用法 / 坑 见同目录 dev_log.md
 * ========================================================================== */

#ifndef DEV_LOG_H
#define DEV_LOG_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <stdint.h>

/* ---- 内部缓冲与记录规格(按需改) ---- */
#define LOG_RING_BYTES          2048u   /* 内存环形缓冲大小(必须 >= 一帧最大长度+2) */
#define LOG_FRAME_MAX           240u    /* 一帧 payload 上限(<= 记录 payload 上限) */
#define LOG_RECORD_SIZE         256u    /* 一条记录固定 256B = 1 个 Flash 页 */
#define LOG_RECORD_PAYLOAD_MAX  (LOG_RECORD_SIZE - 8u)      /* 248B */
/* 上电时是否把 本/上次 的首尾地址打一行日志(事后查错误很方便) */
#define LOG_DEBUG_BOOT_CHAIN    1

#define LOG_HEADER_SLOT_SIZE    32u
#define LOG_HEADER_SLOT_COUNT   128u    /* 4KB / 32B:一个扇区装 128 个检查点 */
#define LOG_CHECKPOINT_RECORDS  32u     /* 每 32 条记录落一个检查点 */

/* ---- 落盘节奏:每 LOG_TASK_INTERVAL_US 最多写 LOG_TASK_MAX_RECORDS 条 ---- */
#define LOG_TASK_INTERVAL_US    100000u  /* 100 ms 一条记录(降低写全频率,防止经常写满) ✓ */
#define LOG_TASK_MAX_RECORDS    1u

/* ---- 日志级别 ---- */
typedef enum
{
    LOG_LEVEL_INFO = 0,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_ERROR
} Enum_Log_Level;

/* ---- 控制台回调:把一段文本送出去(比如 USB CDC / UART) ---- */
typedef void (*Log_Console_Callback)(const char *__text, uint16_t __len);

/**
 * @brief 初始化:读头部检查点,对齐到上次的尾部(掉电重启可续记)
 * @return true = 就绪
 * @note 必须在 Flash_Init() 成功之后调用
 */
bool Log_Init(void);

/**
 * @brief 周期跑:把环形里的帧落盘(内部已限速)
 * @return void
 */
void Log_Task(void);

/**
 * @brief 写一条文本日志(自动加 [时间] 级别前缀)
 * @param __level 级别
 * @param __format 格式串(**别用 %f**)
 * @return void
 */
void Log_Printf(Enum_Log_Level __level, const char *__format, ...);

/**
 * @brief 写一帧原始二进制数据(比如整帧 IMU 采样)
 * @param __data 数据
 * @param __len  长度(<= LOG_FRAME_MAX)
 * @return void
 */
void Log_Record(const void *__data, uint16_t __len);

/**
 * @brief 注册实时控制台回调(可传 nullptr 注销)
 * @return void
 */
void Log_Set_Console(Log_Console_Callback __callback);

/**
 * @brief 打开/关闭实时控制台(USB 起来之后再打开)
 * @return void
 */
void Log_Console_Enable(bool __enable);

/**
 * @brief 是否已就绪
 * @return true = 可以记
 */
bool Log_Is_Ready(void);

/**
 * @brief 已落盘的字节数(相对记录区起点)
 * @return 字节数
 */
uint32_t Log_Get_Written_Bytes(void);

/**
 * @brief 已写记录条数(绝对序号)
 * @return 条数
 */
uint32_t Log_Get_Seq(void);

/**
 * @brief 被丢弃的字节数(环形满 / 记录区满 / 写失败)
 * @return 字节数
 */
uint32_t Log_Get_Dropped_Bytes(void);

/**
 * @brief 本次上电编号(第一次为 1,每次上电 +1)
 */
uint32_t Log_Get_Boot_Id(void);

/**
 * @brief 本次上电开始写的位置(相对 LOG_DATA_ADDR;0xFFFFFFFF = 无)
 */
uint32_t Log_Get_Boot_Start(void);

/**
 * @brief 上一次上电停止的位置(0xFFFFFFFF = 无)
 */
uint32_t Log_Get_Prev_Offset(void);

/**
 * @brief 上一次上电开始的位置(0xFFFFFFFF = 无)
 * @note start/stop 成对 ⇒ 能把上一次上电写的那一段整个圈出来 ✓
 */
uint32_t Log_Get_Prev_Start(void);

/**
 * @brief 回绕次数(写满一次 +1,之后从开头擦掉重写 ✓)
 */
uint32_t Log_Get_Wrap_Count(void);

/**
 * @brief 记录区是否已满
 * @return true = 满了(不再落盘,只丢弃)
 */
bool Log_Is_Full(void);

/**
 * @brief 读回记录区内容(给上位机 dump 用;也可以直接用 bsp_flash 读)
 * @param __offset 相对记录区起点的偏移
 * @param __buffer 输出
 * @param __len    长度
 * @return true = 成功
 */
bool Log_Read(uint32_t __offset, void *__buffer, uint32_t __len);

/**
 * @brief 清空日志(擦头部扇区 + 记录区第一个扇区,从头开始记)
 * @return true = 成功
 * @note ⚠️ 只擦头部和记录区首扇区(~50ms);旧数据在"已写长度"之外,不会被读到
 */
bool Log_Erase_All(void);

/**
 * @brief 把 float 转成字符串(替代 %f,参考工程 dev_log 的做法)
 * @param __str 输出缓冲(建议 >= 16 字节)
 * @param __value 数值
 * @return void
 */
void Log_Float2Str(char *__str, float __value);

#ifdef __cplusplus
}
#endif

#endif // DEV_LOG_H
