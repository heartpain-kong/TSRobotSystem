//
// msg_center.cpp — 任务间通信:环形数组(静态分配,无 malloc,短临界区保护)
// 说明 / 用法 / 坑 见同目录 msg_center.md
//

#include "msg_center.h"

#include "main.h"       /* __get_PRIMASK / __disable_irq */

#include <string.h>

/* ============================================================================
 * 每个话题一个环形数组
 * ========================================================================== */

typedef struct
{
    uint8_t Buf[MSG_RING_DEPTH][MSG_SLOT_SIZE];
    uint16_t Len[MSG_RING_DEPTH];
    uint32_t Seq[MSG_RING_DEPTH];
    volatile uint16_t Head;             /* 生产者写指针(下一个要写的位置) */
    volatile uint16_t Tail;             /* 消费者弹出指针(Msg_Pop 用) */
    volatile uint32_t Next_Seq;         /* 下一条消息的序号 */
    volatile uint32_t Published;
    volatile uint32_t Overwritten;
} Msg_Ring_t;

static Msg_Ring_t s_rings[MSG_TOPIC_COUNT];

static const char *const s_topic_name[MSG_TOPIC_COUNT] = {
    "rc", "minipc", "referee", "cmd", "imu_chassis", "imu_gimbal",
    "chassis", "gimbal", "highrise", "motor", "robot"
};

/* ============================================================================
 * 小工具
 * ========================================================================== */

static bool Topic_Is_Valid(Enum_Msg_Topic __topic)
{
    return ((uint32_t)__topic < (uint32_t)MSG_TOPIC_COUNT);
}

static uint16_t Ring_Next(uint16_t __index)
{
    return (uint16_t)((__index + 1u) % MSG_RING_DEPTH);
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 清空所有话题
 */
void Msg_Init(void)
{
    memset(s_rings, 0, sizeof(s_rings));
}

/**
 * @brief 发布(写环形数组;满了覆盖最旧的)
 */
bool Msg_Publish(Enum_Msg_Topic __topic, const void *__data, uint16_t __len)
{
    Msg_Ring_t *ring;
    uint32_t primask;
    uint16_t head;

    if ((!Topic_Is_Valid(__topic)) || (__data == nullptr) || (__len == 0u))
    {
        return false;
    }

    ring = &s_rings[(uint32_t)__topic];

    if (__len > MSG_SLOT_SIZE)
    {
        ring->Overwritten++;            /* 超长:拒收(也记一笔,便于发现协议写错) */
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    head = ring->Head;
    memcpy(ring->Buf[head], __data, __len);
    ring->Len[head] = __len;
    ring->Seq[head] = ring->Next_Seq;
    ring->Next_Seq++;
    ring->Published++;
    ring->Head = Ring_Next(head);

    /* 写指针追上弹出指针 ⇒ 环形满了,丢掉最旧的一条 */
    if (ring->Head == ring->Tail)
    {
        ring->Tail = Ring_Next(ring->Tail);
        ring->Overwritten++;
    }

    __set_PRIMASK(primask);
    return true;
}

/**
 * @brief 取最新一条(不弹出)
 */
bool Msg_Latest(Enum_Msg_Topic __topic, void *__out, uint16_t __out_size, uint32_t *__seq)
{
    Msg_Ring_t *ring;
    uint32_t primask;
    uint16_t head;
    uint16_t idx;
    uint16_t len;

    if ((!Topic_Is_Valid(__topic)) || (__out == nullptr))
    {
        return false;
    }

    ring = &s_rings[(uint32_t)__topic];

    primask = __get_PRIMASK();
    __disable_irq();

    head = ring->Head;
    if (ring->Published == 0u)
    {
        __set_PRIMASK(primask);
        return false;                   /* 还没人发过 */
    }

    idx = (head == 0u) ? (uint16_t)(MSG_RING_DEPTH - 1u) : (uint16_t)(head - 1u);
    len = ring->Len[idx];
    if ((len == 0u) || (len > __out_size))
    {
        __set_PRIMASK(primask);
        return false;
    }

    memcpy(__out, ring->Buf[idx], len);
    if (__seq != nullptr)
    {
        *__seq = ring->Seq[idx];
    }

    __set_PRIMASK(primask);
    return true;
}

/**
 * @brief 有没有比 __last_seq 更新的数据
 */
bool Msg_Is_New(Enum_Msg_Topic __topic, uint32_t __last_seq)
{
    if (!Topic_Is_Valid(__topic))
    {
        return false;
    }
    return (s_rings[(uint32_t)__topic].Next_Seq != __last_seq) &&
           (s_rings[(uint32_t)__topic].Published != 0u);
}

/**
 * @brief 取最旧一条并弹出
 */
bool Msg_Pop(Enum_Msg_Topic __topic, void *__out, uint16_t __out_size)
{
    Msg_Ring_t *ring;
    uint32_t primask;
    uint16_t tail;
    uint16_t len;

    if ((!Topic_Is_Valid(__topic)) || (__out == nullptr))
    {
        return false;
    }

    ring = &s_rings[(uint32_t)__topic];

    primask = __get_PRIMASK();
    __disable_irq();

    if (ring->Tail == ring->Head)
    {
        __set_PRIMASK(primask);
        return false;                   /* 空 */
    }

    tail = ring->Tail;
    len = ring->Len[tail];
    if ((len == 0u) || (len > __out_size))
    {
        ring->Tail = Ring_Next(tail);   /* 脏数据:丢掉继续 */
        __set_PRIMASK(primask);
        return false;
    }

    memcpy(__out, ring->Buf[tail], len);
    ring->Tail = Ring_Next(tail);

    __set_PRIMASK(primask);
    return true;
}

/**
 * @brief 最新消息序号
 */
uint32_t Msg_Get_Seq(Enum_Msg_Topic __topic)
{
    if (!Topic_Is_Valid(__topic))
    {
        return 0u;
    }
    return s_rings[(uint32_t)__topic].Next_Seq;
}

/**
 * @brief 累计发布条数
 */
uint32_t Msg_Get_Published(Enum_Msg_Topic __topic)
{
    if (!Topic_Is_Valid(__topic))
    {
        return 0u;
    }
    return s_rings[(uint32_t)__topic].Published;
}

/**
 * @brief 被覆盖掉的历史条数
 */
uint32_t Msg_Get_Overwritten(Enum_Msg_Topic __topic)
{
    if (!Topic_Is_Valid(__topic))
    {
        return 0u;
    }
    return s_rings[(uint32_t)__topic].Overwritten;
}

/**
 * @brief 话题名
 */
const char *Msg_Get_Topic_Name(Enum_Msg_Topic __topic)
{
    if (!Topic_Is_Valid(__topic))
    {
        return "N/A";
    }
    return s_topic_name[(uint32_t)__topic];
}
