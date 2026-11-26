//
// dev_remote.cpp — 遥控器/键鼠设备(基于 BSP 层的 DBUS 原始帧 + 在线判定与对外接口)
// 说明 / 用法 / 坑 见同目录 dev_remote.md
//

#include "dev_remote.h"

#include "bsp_timestamp.h"      /* 掉线超时 */
#include "msg_center.h"         /* 发布到 MSG_TOPIC_RC */

#include <string.h>

static_assert(sizeof(Remote_Data_t) <= MSG_SLOT_SIZE, "Remote_Data_t 超过 MSG_SLOT_SIZE");
static_assert(sizeof(Remote_Data_t) >= sizeof(Bsp_Rc_Frame_t),
              "Remote_Data_t 必须以 Bsp_Rc_Frame_t 为前缀(见 .h 注释)");

/* ============================================================================
 * 内部状态
 * ========================================================================== */

static Remote_Data_t s_remote;
static uint32_t s_published_seq = 0u;

/* ============================================================================
 * 对外接口
 * ========================================================================== */

bool Remote_Init(UART_HandleTypeDef *__huart)
{
    memset(&s_remote, 0, sizeof(s_remote));
    s_published_seq = 0u;
    s_remote.Lost = true;               /* 收到第一帧之前都算掉线 */

    /* 设备初始化 = 让协议层开始收帧 */
    return BSP_RC_Init(__huart);
}

void Remote_Task(void)
{
    const Bsp_Rc_Frame_t *raw = BSP_RC_Get();
    uint32_t now_ms = (uint32_t)Timestamp_Get_Ms();

    /* ① 把 BSP 层的原始帧搬进来(前缀一致 ⇒ 一次 memcpy;后面的语义字段不受影响) */
    memcpy(&s_remote, raw, sizeof(Bsp_Rc_Frame_t));

    /* ② 掉线判定:超时没帧(遥控器关机时本来就不会来帧) */
    if ((s_remote.Rx_Count == 0u) ||
        ((uint32_t)(now_ms - s_remote.Last_Rx_Ms) > REMOTE_LOST_TIMEOUT_MS))
    {
        if (!s_remote.Lost)
        {
            s_remote.Lost_Count++;
        }
        s_remote.Lost = true;
    }
    else
    {
        s_remote.Lost = false;
    }

    /* ③ 只有收到过新帧才发布一次(不然会把环形数组刷满无意义的重复数据) */
    if (s_remote.Rx_Count != s_published_seq)
    {
        s_published_seq = s_remote.Rx_Count;
        s_remote.Seq++;
        (void)Msg_Publish(MSG_TOPIC_RC, &s_remote, (uint16_t)sizeof(s_remote));
    }
}

const Remote_Data_t *Remote_Get(void)
{
    return &s_remote;
}

bool Remote_Is_Online(void)
{
    return (!s_remote.Lost);
}

uint32_t Remote_Get_Rx_Count(void)
{
    return s_remote.Rx_Count;
}

float Remote_Get_Ch_Norm(uint8_t __index)
{
    float v;

    if (__index >= RC_CHANNEL_NUM)
    {
        return 0.0f;
    }

    v = (float)s_remote.Ch[__index] / 660.0f;

    if (v > 1.0f)
    {
        v = 1.0f;
    }
    else if (v < -1.0f)
    {
        v = -1.0f;
    }

    return v;
}

uint8_t Remote_Get_Switch(uint8_t __index)
{
    if ((__index > 1u) || (!Remote_Is_Online()))
    {
        return 0u;
    }
    return s_remote.Switch[__index];
}

bool Remote_Key_Pressed(uint16_t __key_mask)
{
    if (!Remote_Is_Online())
    {
        return false;
    }
    return ((s_remote.Key & __key_mask) != 0u);
}
