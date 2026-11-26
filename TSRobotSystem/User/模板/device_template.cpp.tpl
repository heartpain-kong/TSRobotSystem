//
// dev_xxx.cpp — 一句话说清这是什么设备(照 User/模板/device_template.cpp.tpl 改)
// 说明 / 用法 / 坑 见同目录 dev_xxx.md
// ❌ 本层(Device)禁止:直接写电机目标/调 motor_task_*、读遥控业务语义
//

#include "dev_xxx.h"

#include "bsp_timestamp.h"
#include "bsp_uart.h"           /* USART_init():注册接收回调 */
#include "crc_ccitt.h"          /* 协议要 CRC 的话:crc8.h / crc_ccitt.h / crc32.h */
#include "msg_center.h"         /* 发布到自己的话题 */

#include "main.h"               /* __get_PRIMASK(需要临界区时) */

#include <string.h>

static_assert(sizeof(Xxx_Data_t) <= MSG_SLOT_SIZE, "Xxx_Data_t 超过 MSG_SLOT_SIZE");

/* ============================================================================
 * 内部状态
 * ========================================================================== */

static Xxx_Data_t s_xxx;
static uint32_t s_published_seq = 0u;

/* ============================================================================
 * ① 收:中断里只搬运(不解析、不打印、不阻塞)
 * ========================================================================== */

void Xxx_Rx_Callback(uint8_t *__buf, uint16_t __len)
{
    (void)__buf;
    (void)__len;
    /* 多数设备由 BSP 引擎"一帧一次回调" ⇒ 这里可以直接把整帧拷进自己的缓冲、
       置一个"有新帧"标志,真正的解析放到 Xxx_Task() 里做。
       如果是字节流(如 USB CDC),就在这里往环形缓冲里追加字节。 */
}

/* ============================================================================
 * ② 解:组帧 → 校验 → 解包(放任务里做)
 * ========================================================================== */

static void Xxx_Parse(const uint8_t *__frame, uint16_t __len)
{
    if ((__frame == nullptr) || (__len < XXX_FRAME_SIZE))
    {
        s_xxx.Reject_Count++;
        return;
    }

    /* 校验(长度/帧头/CRC)…… 不通过就 Crc_Errors++ / Reject_Count++ 并 return */

    s_xxx.Value = 0.0f;                     /* 解出来的物理量,单位写清 */
    s_xxx.Rx_Frames++;
    s_xxx.Last_Rx_Ms = (uint32_t)Timestamp_Get_Ms();
    s_xxx.Online = true;
    s_xxx.Seq++;
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

bool Xxx_Init(UART_HandleTypeDef *__huart)
{
    memset(&s_xxx, 0, sizeof(s_xxx));
    s_published_seq = 0u;
    s_xxx.Online = false;

    USART_init((__huart != nullptr) ? __huart : &huart5, Xxx_Rx_Callback);
    return true;
}

void Xxx_Task(void)
{
    uint32_t now_ms = (uint32_t)Timestamp_Get_Ms();

    /* 掉线判定 */
    if ((s_xxx.Rx_Frames == 0u) || ((uint32_t)(now_ms - s_xxx.Last_Rx_Ms) > XXX_LOST_TIMEOUT_MS))
    {
        if (s_xxx.Online)
        {
            s_xxx.Lost_Count++;
        }
        s_xxx.Online = false;
    }

    /* 有新数据才发布(避免刷满环形数组) */
    if (s_xxx.Seq != s_published_seq)
    {
        s_published_seq = s_xxx.Seq;
        (void)Msg_Publish(MSG_TOPIC_XXX, &s_xxx, (uint16_t)sizeof(s_xxx));
    }
}

const Xxx_Data_t *Xxx_Get(void)
{
    return &s_xxx;
}

bool Xxx_Is_Online(void)
{
    return s_xxx.Online;
}
