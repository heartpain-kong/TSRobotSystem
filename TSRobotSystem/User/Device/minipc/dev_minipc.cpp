//
// dev_minipc.cpp — miniPC / 虚拟串口通信
//
// 帧格式:
//   [0]      0xA5
//   [1]      control: bit7..6 = 通道(0=RC,1=RM), bit5..0 = mode
//   [2]      payload 长度 n (8bit)
//   [3..]    payload n 字节
//   [n+3..]  CRC16 小端
//
// CRC16: 0x8408 反射, init 0, 覆盖 [0 .. n+3-1], 即整帧除末尾 CRC
//

#include "dev_minipc.h"

#include "bsp_timestamp.h"
#include "crc_ccitt.h"
#include "msg_center.h"
#include "usbd_cdc_if.h"

#include "main.h"

#include <string.h>

static_assert(sizeof(MiniPC_Data_t) <= MSG_SLOT_SIZE, "MiniPC_Data_t too large");

#define MINIPC_RX_BUF_SIZE    256u

static uint8_t s_rx[MINIPC_RX_BUF_SIZE];
static volatile uint16_t s_rx_len = 0u;
static volatile uint32_t s_rx_overflow = 0u;

static MiniPC_Data_t s_host;
static uint32_t s_published_seq = 0u;
static bool     s_published_online = false;

extern "C" void MiniPC_Rx_Callback(uint8_t *__buf, uint16_t __len)
{
    uint32_t primask;
    uint16_t i;

    if ((__buf == nullptr) || (__len == 0u)) return;

    primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0u; i < __len; i++)
    {
        if (s_rx_len < MINIPC_RX_BUF_SIZE)
        {
            s_rx[s_rx_len++] = __buf[i];
        }
        else
        {
            s_rx_overflow++;
            s_rx_len = 0u;
            break;
        }
    }
    __set_PRIMASK(primask);
}

static void MiniPC_Drop(uint16_t __n)
{
    uint32_t primask;
    uint16_t len;

    primask = __get_PRIMASK();
    __disable_irq();
    len = s_rx_len;
    if (__n >= len)
    {
        s_rx_len = 0u;
    }
    else
    {
        memmove(s_rx, &s_rx[__n], (size_t)(len - __n));
        s_rx_len = (uint16_t)(len - __n);
    }
    __set_PRIMASK(primask);
}

static bool MiniPC_Dispatch(uint8_t __channel, uint8_t __mode,
                            const uint8_t *__data, uint16_t __len)
{
    s_host.Channel = __channel;
    s_host.Mode = __mode;

    if (__channel == MINIPC_CH_RC)
    {
        if (__len == sizeof(MiniPC_Cmd_Payload_t))
        {
            memcpy(&s_host.Cmd, __data, sizeof(MiniPC_Cmd_Payload_t));
            return true;
        }
        s_host.Reject_Count++;
        return false;
    }

    if (__channel == MINIPC_CH_RM)
    {
        /* RM 自瞄通道:载荷预留,先接受不消费 */
        return true;
    }

    s_host.Reject_Count++;
    return false;
}

bool MiniPC_Init(void)
{
    memset(&s_host, 0, sizeof(s_host));
    memset(s_rx, 0, sizeof(s_rx));
    s_rx_len = 0u;
    s_rx_overflow = 0u;
    s_published_seq = 0u;
    s_published_online = false;
    s_host.Online = false;
    return true;
}

void MiniPC_Task(void)
{
    uint32_t now_ms = (uint32_t)Timestamp_Get_Ms();
    uint16_t guard = 0u;

    while (guard++ < 8u)
    {
        uint8_t data_len;
        uint16_t total;
        uint16_t crc16;

        if (s_rx_len < MINIPC_MIN_FRAME) break;

        if (s_rx[0] != MINIPC_SOF)
        {
            s_host.Reject_Count++;
            MiniPC_Drop(1u);
            continue;
        }

        data_len = s_rx[2];
        total = (uint16_t)data_len + 6u;
        if (total > MINIPC_RX_BUF_SIZE)
        {
            s_host.Reject_Count++;
            MiniPC_Drop(1u);
            continue;
        }
        if (s_rx_len < total) break;
        if (s_rx[total - 1u] != 0x5Au)
        {
            s_host.Reject_Count++;
            MiniPC_Drop(1u);
            continue;
        }

        crc16 = (uint16_t)((uint16_t)s_rx[total - 3u] |
                           ((uint16_t)s_rx[total - 2u] << 8));
        if (crc_ccitt(s_rx, (size_t)(total - 3u)) != crc16)
        {
            s_host.Crc16_Errors++;
            MiniPC_Drop(total);
            continue;
        }

        {
            const uint8_t control = s_rx[1];
            const uint8_t channel = (uint8_t)((control >> 6) & 0x03u);
            const uint8_t mode = (uint8_t)(control & 0x3Fu);

            if (MiniPC_Dispatch(channel, mode, &s_rx[3], data_len))
            {
                s_host.Rx_Frames++;
                s_host.Last_Rx_Ms = now_ms;
                s_host.Online = true;
                s_host.Seq++;
            }
        }
        MiniPC_Drop(total);
    }

    if ((s_host.Rx_Frames == 0u) ||
        ((uint32_t)(now_ms - s_host.Last_Rx_Ms) > MINIPC_LOST_TIMEOUT_MS))
    {
        if (s_host.Online) s_host.Lost_Count++;
        s_host.Online = false;
    }

    if ((s_host.Seq != s_published_seq) ||
        (s_host.Online != s_published_online))
    {
        s_published_seq = s_host.Seq;
        s_published_online = s_host.Online;
        (void)Msg_Publish(MSG_TOPIC_MINIPC, &s_host, (uint16_t)sizeof(s_host));
    }
}

const MiniPC_Data_t *MiniPC_Get(void) { return &s_host; }
bool MiniPC_Is_Online(void) { return s_host.Online; }

bool MiniPC_Send(const void *__data, uint16_t __len)
{
    if ((__data == nullptr) || (__len == 0u)) return false;
    return (CDC_Transmit_HS((uint8_t *)__data, __len) == USBD_OK);
}
