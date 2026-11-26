/* ============================================================================
 * dev_xxx.h — 一句话说清这是什么设备(照 User/模板/device_template.h.tpl 改)
 * 说明 / 用法 / 坑 见同目录 dev_xxx.md
 * ★ 用法:复制到 User/Device/<模块>/ → 去掉 .tpl → 全文替换 xxx/XXX/Xxx
 * ========================================================================== */

#ifndef DEV_XXX_H
#define DEV_XXX_H

#include "usart.h"              /* 只 include 你要用的 CubeMX 句柄头 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 协议常量 ---------------- */
#define XXX_FRAME_SIZE          18u         /* 单帧长度 */
#define XXX_LOST_TIMEOUT_MS     100u        /* 掉线超时(按对方帧率给) */

/**
 * @brief 解析结果(发布在 MSG_TOPIC_XXX)
 */
typedef struct
{
    float Value;                            /* 解出来的数据(单位写清) */
    bool Online;
    uint32_t Last_Rx_Ms;
    uint32_t Rx_Frames;                     /* 校验通过的帧数 */
    uint32_t Crc_Errors;
    uint32_t Reject_Count;
    uint32_t Lost_Count;
    uint32_t Seq;                           /* 收到一帧 +1 */
} Xxx_Data_t;

/**
 * @brief 初始化:注册 BSP 回调 + 清状态
 * @param __huart 句柄;传 NULL 用默认
 */
bool Xxx_Init(UART_HandleTypeDef *__huart);

/**
 * @brief 周期跑:组帧/校验/掉线判定 + 有新数据就发布到 msg_center
 */
void Xxx_Task(void);

/**
 * @brief 收数据入口(★ 在中断里调用:只搬运,不解析、不打印、不阻塞)
 */
void Xxx_Rx_Callback(uint8_t *__buf, uint16_t __len);

/**
 * @brief 读解析结果
 */
const Xxx_Data_t *Xxx_Get(void);

/**
 * @brief 是否在线
 */
bool Xxx_Is_Online(void);

#ifdef __cplusplus
}
#endif

#endif // DEV_XXX_H
