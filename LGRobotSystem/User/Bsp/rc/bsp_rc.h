/* ============================================================================
 * bsp_rc.h — DBUS 遥控帧接收(UART5:100000 8E1)—— 协议层,不含语义
 * 说明 / 用法 / 坑 见同目录 bsp_rc.md
 * ========================================================================== */

#ifndef BSP_RC_H
#define BSP_RC_H

#include "usart.h"      /* UART_HandleTypeDef / huart5 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 协议常量(DBUS 规定,改不了) ---------------- */
#define RC_CH_VALUE_OFFSET      1024        /* 杆量中位 */
#define RC_CH_VALUE_MIN         364         /* 1024 - 660 */
#define RC_CH_VALUE_MAX         1684        /* 1024 + 660 */
#define RC_CHANNEL_NUM          5u          /* 4 杆 + 1 拨轮 */
#define RC_FRAME_SIZE           18u         /* DBUS 单帧 18 字节 */

/* 键盘位图(RM 标准:遥控器把上位机的键鼠一起发过来) */
#define RC_KEY_W        (1u << 0)
#define RC_KEY_S        (1u << 1)
#define RC_KEY_A        (1u << 2)
#define RC_KEY_D        (1u << 3)
#define RC_KEY_SHIFT    (1u << 4)
#define RC_KEY_CTRL     (1u << 5)
#define RC_KEY_Q        (1u << 6)
#define RC_KEY_E        (1u << 7)
#define RC_KEY_R        (1u << 8)
#define RC_KEY_F        (1u << 9)
#define RC_KEY_G        (1u << 10)
#define RC_KEY_Z        (1u << 11)
#define RC_KEY_X        (1u << 12)
#define RC_KEY_C        (1u << 13)
#define RC_KEY_V        (1u << 14)
#define RC_KEY_B        (1u << 15)

/**
 * @brief 一帧 DBUS 的原始内容(★ 只有协议字段,没有"在线/掉线"这类语义)
 * @note 这个结构的字段顺序被 dev_remote 当"前缀"用 ⇒ 改字段要同步看 dev_remote.cpp
 */
typedef struct
{
    int16_t Ch[RC_CHANNEL_NUM];     /* 杆量,已减 1024(约 -660 ~ +660) */
    uint8_t Switch[2];              /* 左右拨杆:1=上 2=中 3=下 */
    int16_t Mouse_X;                /* 鼠标 x 增量 */
    int16_t Mouse_Y;
    int16_t Mouse_Z;                /* 滚轮 */
    uint8_t Mouse_Press_L;
    uint8_t Mouse_Press_R;
    uint16_t Key;                   /* 键盘位图(RC_KEY_xxx) */
    uint32_t Last_Rx_Ms;            /* 最近一帧的时刻(ms) */
    uint32_t Rx_Count;              /* 累计收到帧数 */
} Bsp_Rc_Frame_t;

/**
 * @brief 初始化:注册 UART 接收回调(DBM+IDLE 引擎)
 * @param __huart 串口句柄;传 NULL 用默认 &huart5
 * @return true = 成功
 */
bool BSP_RC_Init(UART_HandleTypeDef *__huart);

/**
 * @brief 读最近的原始帧(只读)
 */
const Bsp_Rc_Frame_t *BSP_RC_Get(void);

/**
 * @brief 累计收帧数(判断有没有在收)
 */
uint32_t BSP_RC_Get_Rx_Count(void);

#ifdef __cplusplus
}
#endif

#endif // BSP_RC_H
