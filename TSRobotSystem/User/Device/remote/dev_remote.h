/* ============================================================================
 * dev_remote.h — 遥控器/键鼠设备(在线判定 + 归一化杆量/拨杆/按键 + 发布话题)
 * 说明 / 用法 / 坑 见同目录 dev_remote.md
 * ========================================================================== */

#ifndef DEV_REMOTE_H
#define DEV_REMOTE_H

#include "bsp_rc.h"     /* ★ 协议层:帧结构、通道数、键盘位图宏都在这里 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define REMOTE_LOST_TIMEOUT_MS  100u        /* 100ms 没收到帧 = 掉线 */

/**
 * @brief 遥控器数据(发布在 MSG_TOPIC_RC)
 * @note **前面 9 个字段与 `Bsp_Rc_Frame_t` 完全一致**(顺序/类型都对得上),
 *       所以 dev_remote 可以一次 memcpy 把 BSP 层的原始帧搬进来 ⇒ 改这里要同步看 .cpp
 */
typedef struct
{
    /* ---- 以下与 Bsp_Rc_Frame_t 前缀一致(原始数据) ---- */
    int16_t Ch[RC_CHANNEL_NUM];     /* 杆量,已减 1024 */
    uint8_t Switch[2];              /* 拨杆 1/2/3 */
    int16_t Mouse_X;
    int16_t Mouse_Y;
    int16_t Mouse_Z;
    uint8_t Mouse_Press_L;
    uint8_t Mouse_Press_R;
    uint16_t Key;                   /* 键盘位图(RC_KEY_xxx) */
    uint32_t Last_Rx_Ms;
    uint32_t Rx_Count;
    /* ---- 以下为设备语义 ---- */
    bool Lost;                      /* 掉线(超时没帧) */
    uint32_t Lost_Count;            /* 发生了几次掉线 */
    uint32_t Seq;                   /* 每收到一帧 +1(消费者用它判断"有没有新数据") */
} Remote_Data_t;

/**
 * @brief 初始化:清状态 + 让 BSP 层开始收帧
 * @param __huart 串口句柄;传 NULL 用默认 &huart5
 */
bool Remote_Init(UART_HandleTypeDef *__huart);

/**
 * @brief 周期跑(建议 100Hz):掉线判定 + 收到新帧就发布到 MSG_TOPIC_RC
 */
void Remote_Task(void);

/**
 * @brief 读当前数据(只读)
 */
const Remote_Data_t *Remote_Get(void);

/**
 * @brief 是否在线
 */
bool Remote_Is_Online(void);

/**
 * @brief 累计收帧数
 */
uint32_t Remote_Get_Rx_Count(void);

/**
 * @brief 杆量归一化到 -1.0f ~ +1.0f(中位 = 0)
 * @param __index 0~4(0=右左右,1=右上下,2=左左右,3=左上下,4=拨轮)
 */
float Remote_Get_Ch_Norm(uint8_t __index);

/**
 * @brief 拨杆状态
 * @param __index 0 = 左,1 = 右
 * @return 1/2/3(未在线返回 0)
 */
uint8_t Remote_Get_Switch(uint8_t __index);

/**
 * @brief 键盘某键是否按下
 * @param __key_mask RC_KEY_xxx
 */
bool Remote_Key_Pressed(uint16_t __key_mask);

#ifdef __cplusplus
}
#endif

#endif // DEV_REMOTE_H
