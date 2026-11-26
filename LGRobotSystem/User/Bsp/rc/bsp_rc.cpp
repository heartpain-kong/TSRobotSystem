//
// bsp_rc.cpp — DBUS 遥控帧接收(UART5:100000 8E1)—— 协议层,不含语义
// 说明 / 用法 / 坑 见同目录 bsp_rc.md
//

#include "bsp_rc.h"

#include "bsp_timestamp.h"      /* 收帧时刻 */
#include "bsp_uart.h"           /* USART_init():DBM + IDLE 接收引擎 */

#include <string.h>

/* ============================================================================
 * 内部状态
 * ========================================================================== */

static Bsp_Rc_Frame_t s_frame;

/* ============================================================================
 * 接收回调(ISR 上下文:只解析这一帧,不阻塞、不发消息、不判定在线)
 * ========================================================================== */

static void BSP_RC_Uart_Callback(UART_HandleTypeDef *__huart, uint8_t *__buf, uint16_t __size)
{
    (void)__huart;

    /* 帧长不对:丢弃(DBUS 固定 18 字节) */
    if ((__buf == nullptr) || (__size < RC_FRAME_SIZE))
    {
        return;
    }

    /* 5 个通道:11 位打包,中心 1024 */
    s_frame.Ch[0] = (int16_t)((int16_t)((uint16_t)__buf[0] | ((uint16_t)__buf[1] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    s_frame.Ch[1] = (int16_t)((int16_t)(((uint16_t)__buf[1] >> 3) | ((uint16_t)__buf[2] << 5)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    s_frame.Ch[2] = (int16_t)((int16_t)(((uint16_t)__buf[2] >> 6) | ((uint16_t)__buf[3] << 2) | ((uint16_t)__buf[4] << 10)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    s_frame.Ch[3] = (int16_t)((int16_t)(((uint16_t)__buf[4] >> 1) | ((uint16_t)__buf[5] << 7)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    s_frame.Ch[4] = (int16_t)((int16_t)((uint16_t)__buf[16] | ((uint16_t)__buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;

    /* 拨杆 + 鼠标 + 键盘
       注意:第 16/17 字节属于 ch4(上面已读),**不是** SBUS 那种 failsafe 标志字节 */
    s_frame.Switch[0] = (uint8_t)((__buf[5] >> 4) & 0x03u);
    s_frame.Switch[1] = (uint8_t)((__buf[5] >> 6) & 0x03u);
    s_frame.Mouse_X = (int16_t)((uint16_t)__buf[6] | ((uint16_t)__buf[7] << 8));
    s_frame.Mouse_Y = (int16_t)((uint16_t)__buf[8] | ((uint16_t)__buf[9] << 8));
    s_frame.Mouse_Z = (int16_t)((uint16_t)__buf[10] | ((uint16_t)__buf[11] << 8));
    s_frame.Mouse_Press_L = __buf[12];
    s_frame.Mouse_Press_R = __buf[13];
    s_frame.Key = (uint16_t)((uint16_t)__buf[14] | ((uint16_t)__buf[15] << 8));

    s_frame.Last_Rx_Ms = (uint32_t)Timestamp_Get_Ms();
    s_frame.Rx_Count++;
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

bool BSP_RC_Init(UART_HandleTypeDef *__huart)
{
    memset(&s_frame, 0, sizeof(s_frame));
    USART_init((__huart != nullptr) ? __huart : &huart5, BSP_RC_Uart_Callback);
    return true;
}

const Bsp_Rc_Frame_t *BSP_RC_Get(void)
{
    return &s_frame;
}

uint32_t BSP_RC_Get_Rx_Count(void)
{
    return s_frame.Rx_Count;
}
