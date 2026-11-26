//
// cmd_485.cpp — 上/下 MCU 链路(USART2 RS485,500Hz,虚拟串口帧格式)
// 说明见同目录 cmd_485.md
//
// 固定 54B 帧:命令/反馈(24B) + 调试分片(24B) 装在同一帧里,
// 所以“一发一回 500Hz”和双向调试透传可以同时满速。
//

#include "cmd_485.h"

#include "bsp_timestamp.h"      /* Timestamp_Get_Ms */
#include "bsp_uart.h"           /* huart2 / USART_RX485_init_Ex / UART_Transmit_Data */
#include "chassis_task.h"       /* 下板回传实测里程计 */
#include "cmd_task.h"           /* Cmd_Get */
#include "crc_ccitt.h"          /* CRC16(reflected 0x8408, init 0) */
#include "dev_minipc.h"
#include "imu_task.h"
#include "math_support.h"         /* MiniPC_Cmd_Payload_t:复用虚拟串口的命令载荷 */
#include "robot_def.h"          /* ROBOT_UPPER_ONLY */

#include <string.h>

static_assert(sizeof(MiniPC_Cmd_Payload_t) == CMD485_HALF_LEN, "命令半帧必须是 24B");
static_assert(sizeof(Cmd485_Feedback_t) == CMD485_HALF_LEN, "反馈半帧必须是 24B");

/* 帧内偏移 */
#define CMD485_OFF_A       3u          /* 半帧 A:命令/反馈 */
#define CMD485_OFF_B       27u         /* 半帧 B:调试分片 */
#define CMD485_OFF_CRC     51u
#define CMD485_OFF_TAIL    53u

/* ============================================================================
 * 内部状态
 * ========================================================================== */

/* 上 -> 下:底盘指令 */
static volatile float    s_cmd_vx = 0.0f;
static volatile float    s_cmd_vy = 0.0f;
static volatile float    s_cmd_w  = 0.0f;
static volatile uint8_t  s_cmd_mode = 0u;      /* payload.Mode */
static volatile uint8_t  s_cmd_shoot_en = 0u;  /* payload.Shoot_Enable(上板拨杆) */
static volatile uint8_t  s_cmd_shoot_rate = 0u;/* payload.Shoot_Rate */
static volatile uint8_t  s_cmd_flags = 0u;     /* mode 低 2 位 */
static volatile uint32_t s_cmd_ms = 0u;
static volatile bool     s_cmd_seen = false;

/* 下 -> 上:反馈 */
static volatile float    s_fb_vx = 0.0f;
static volatile float    s_fb_vy = 0.0f;
static volatile float    s_fb_w  = 0.0f;
static volatile float    s_fb_yaw = 0.0f;
static volatile float    s_fb_power = 0.0f;
static volatile uint8_t  s_fb_status = 0u;
static volatile uint32_t s_fb_ms = 0u;
static volatile bool     s_fb_seen = false;

#if !ROBOT_UPPER_ONLY
/* 下板:反馈快照(CMD 任务每 2ms 更新;发送在接收中断里做) */
static Cmd485_Feedback_t s_fb_cache;
#endif

/* 调试计数 */
static volatile uint32_t s_rx_count = 0u;
static volatile uint32_t s_bad_count = 0u;
static volatile uint32_t s_last_ch = 0u;
static volatile uint32_t s_last_ms = 0u;
/* 上板 IMU yaw/角速度:随命令帧下发,下板小陀螺用它做世界航向闭环 */
static volatile float    s_upper_yaw = 0.0f;
static volatile float    s_upper_gyro_z = 0.0f;
static volatile uint32_t s_upper_yaw_ms = 0u;
static volatile bool     s_link_seen = false;

/* ============================================================================
 * 调试数据透传缓冲
 *   线性缓冲 + memmove:数据量小(KB/s),比环形省事,且能按 '\n' 找整行
 * ========================================================================== */

static uint8_t           s_relay[CMD485_RELAY_BUF];
static volatile uint16_t s_relay_len = 0u;
static volatile uint32_t s_relay_drop = 0u;

#if !ROBOT_UPPER_ONLY
static uint16_t Relay_Take(uint8_t *__dst, uint16_t __max)
{
    uint16_t n;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    n = (s_relay_len < __max) ? s_relay_len : __max;
    if (n > 0u)
    {
        memcpy(__dst, s_relay, n);
        s_relay_len = (uint16_t)(s_relay_len - n);
        if (s_relay_len > 0u)
        {
            memmove(s_relay, &s_relay[n], s_relay_len);
        }
    }

    __set_PRIMASK(primask);
    return n;
}
#endif

void Cmd_485_Relay_Push(const void *__data, uint16_t __len)
{
    uint32_t primask;

    if ((__data == nullptr) || (__len == 0u))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (((uint32_t)s_relay_len + (uint32_t)__len) <= (uint32_t)sizeof(s_relay))
    {
        memcpy(&s_relay[s_relay_len], __data, __len);
        s_relay_len = (uint16_t)(s_relay_len + __len);
    }
    else
    {
        s_relay_drop++;     /* 空间不够:整行丢弃,避免半行 JSON */
    }

    __set_PRIMASK(primask);
}

uint16_t Cmd_485_Relay_Pop_Line(uint8_t *__dst, uint16_t __max)
{
    uint16_t i;
    uint16_t n = 0u;
    uint32_t primask;

    if (__dst == nullptr)
    {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0u; i < s_relay_len; i++)
    {
        if (s_relay[i] == (uint8_t)'\n')
        {
            n = (uint16_t)(i + 1u);
            break;
        }
    }

    if ((n > 0u) && (n <= __max))
    {
        memcpy(__dst, s_relay, n);
        s_relay_len = (uint16_t)(s_relay_len - n);
        if (s_relay_len > 0u)
        {
            memmove(s_relay, &s_relay[n], s_relay_len);
        }
    }
    else
    {
        n = 0u;             /* 还没有完整行,或单行超过 __max */
    }

    __set_PRIMASK(primask);
    return n;
}

uint16_t Cmd_485_Relay_Used(void) { return s_relay_len; }
uint32_t Cmd_485_Relay_Drop(void) { return s_relay_drop; }

/* ---- 上 -> 下 调试文本(上板串口收到的字节,随命令帧下发) ---- */
static uint8_t           s_relay_dn[CMD485_RELAY_BUF];
static volatile uint16_t s_relay_dn_len = 0u;

void Cmd_485_RelayDown_Push(const void *__data, uint16_t __len)
{
    uint32_t primask;

    if ((__data == nullptr) || (__len == 0u))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (((uint32_t)s_relay_dn_len + (uint32_t)__len) <= (uint32_t)sizeof(s_relay_dn))
    {
        memcpy(&s_relay_dn[s_relay_dn_len], __data, __len);
        s_relay_dn_len = (uint16_t)(s_relay_dn_len + __len);
    }
    else
    {
        s_relay_drop++;
    }

    __set_PRIMASK(primask);
}

uint16_t Cmd_485_RelayDown_Take(uint8_t *__dst, uint16_t __max)
{
    uint16_t n;
    uint32_t primask;

    if (__dst == nullptr)
    {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    n = (s_relay_dn_len < __max) ? s_relay_dn_len : __max;
    if (n > 0u)
    {
        memcpy(__dst, s_relay_dn, n);
        s_relay_dn_len = (uint16_t)(s_relay_dn_len - n);
        if (s_relay_dn_len > 0u)
        {
            memmove(s_relay_dn, &s_relay_dn[n], s_relay_dn_len);
        }
    }

    __set_PRIMASK(primask);
    return n;
}

uint16_t Cmd_485_RelayDown_Used(void) { return s_relay_dn_len; }

/* ============================================================================
 * 组帧 / 解帧
 * ========================================================================== */

/**
 * @brief 组一帧:SOF + control + len + data + CRC16 + tail(固定 54B)
 */
static void Cmd_485_Build(uint8_t __ch, uint8_t __mode,
                          const void *__data, uint8_t *__out)
{
    uint16_t crc;

    __out[0] = CMD485_SOF;
    __out[1] = (uint8_t)(((__ch & 0x03u) << 6) | (__mode & 0x3Fu));
    __out[2] = CMD485_DATA_LEN;
    if (__data != nullptr)
    {
        memcpy(&__out[CMD485_OFF_A], __data, CMD485_DATA_LEN);
    }

    crc = crc_ccitt(__out, (size_t)CMD485_OFF_CRC);
    __out[CMD485_OFF_CRC]      = (uint8_t)(crc & 0xFFu);
    __out[CMD485_OFF_CRC + 1u] = (uint8_t)((crc >> 8) & 0xFFu);
    __out[CMD485_OFF_TAIL]     = CMD485_TAIL;
}

/**
 * @brief USART2 的 RS485 接收回调(ISR 上下文)
 */
static void Cmd_485_Rx(UART_HandleTypeDef *__huart, uint8_t *__buf, uint16_t __size)
{
    uint8_t frame[CMD485_FRAME_LEN];
    uint16_t crc;
    uint8_t ch;
    uint8_t mode;

    (void)__huart;

    if ((__buf == nullptr) || (__size != CMD485_FRAME_LEN))
    {
        s_bad_count++;
        return;
    }

    memcpy(frame, __buf, sizeof(frame));

    if ((frame[0] != CMD485_SOF) || (frame[CMD485_OFF_TAIL] != CMD485_TAIL))
    {
        s_bad_count++;
        return;
    }

    crc = (uint16_t)((uint16_t)frame[CMD485_OFF_CRC] |
                     ((uint16_t)frame[CMD485_OFF_CRC + 1u] << 8));
    if (crc_ccitt(frame, (size_t)CMD485_OFF_CRC) != crc)
    {
        s_bad_count++;
        return;
    }

    ch   = (uint8_t)((frame[1] >> 6) & 0x03u);
    mode = (uint8_t)(frame[1] & 0x3Fu);

    s_last_ch   = ch;
    s_last_ms   = (uint32_t)Timestamp_Get_Ms();
    s_link_seen = true;
    s_rx_count++;

    if (ch == CMD485_CH_CMD)
    {
        MiniPC_Cmd_Payload_t p;
        uint8_t dn_len = (uint8_t)((mode >> CMD485_MODE_DN_SHIFT) & 0x1Fu);

        memcpy(&p, &frame[CMD485_OFF_A], sizeof(p));

        s_cmd_vx    = p.Chassis_Vx;
        s_cmd_vy    = p.Chassis_Vy;
        s_cmd_w     = p.Chassis_W;
        s_cmd_shoot_en   = p.Shoot_Enable;   /* 上板 S1=3 发射 */
        s_cmd_shoot_rate = p.Shoot_Rate;
        s_upper_yaw    = p.Gimbal_Yaw;      /* 上板 IMU yaw */
        s_upper_gyro_z = p.Gimbal_Pitch;    /* 上板 yaw 角速度 */
        s_upper_yaw_ms = s_last_ms;

        s_cmd_mode  = p.Mode;
        s_cmd_flags = (uint8_t)(mode & 0x03u);
        s_cmd_ms    = s_last_ms;
        s_cmd_seen  = true;

#if !ROBOT_UPPER_ONLY
        /* 下行的调试字节:送给下板的调试解析 */
        if (dn_len > CMD485_HALF_LEN)
        {
            dn_len = CMD485_HALF_LEN;
        }
        if (dn_len > 0u)
        {
            Cmd_485_RelayDown_Push(&frame[CMD485_OFF_B], dn_len);
        }

        /* 一发一回:每收到一帧命令,立刻回一帧(反馈 + 上行调试分片) */
        {
            uint8_t payload[CMD485_DATA_LEN];
            uint8_t out[CMD485_FRAME_LEN];
            uint8_t chunk[CMD485_HALF_LEN];
            uint16_t n;

            memcpy(payload, &s_fb_cache, CMD485_HALF_LEN);

            n = Relay_Take(chunk, CMD485_HALF_LEN);
            memset(&payload[CMD485_HALF_LEN], 0, CMD485_HALF_LEN);
            if (n > 0u)
            {
                memcpy(&payload[CMD485_HALF_LEN], chunk, n);
            }

            Cmd_485_Build(CMD485_CH_FB, (uint8_t)n, payload, out);
            (void)UART_Transmit_Data(&huart2, out, (uint8_t)CMD485_FRAME_LEN);
        }
#endif
    }
    else if (ch == CMD485_CH_FB)
    {
        Cmd485_Feedback_t f;
        uint8_t up_len = (uint8_t)(mode & 0x3Fu);

        memcpy(&f, &frame[CMD485_OFF_A], sizeof(f));

        s_fb_vx     = f.Vx;
        s_fb_vy     = f.Vy;
        s_fb_w      = f.W;
        s_fb_yaw    = f.Yaw;
        s_fb_power  = f.Power_W;
        s_fb_status = f.Status;
        s_fb_ms     = s_last_ms;
        s_fb_seen   = true;

#if ROBOT_UPPER_ONLY
        if (up_len > CMD485_HALF_LEN)
        {
            up_len = CMD485_HALF_LEN;
        }
        if (up_len > 0u)
        {
            Cmd_485_Relay_Push(&frame[CMD485_OFF_B], up_len);
        }
#else
        (void)up_len;
#endif
    }
    else
    {
        /* 其它通道:协议预留 */
    }
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

void Cmd_485_Init(void)
{
    s_cmd_seen  = false;
    s_fb_seen   = false;
    s_link_seen = false;
    s_rx_count  = 0u;
    s_bad_count = 0u;
    s_last_ch   = 0u;
    s_last_ms   = 0u;

    /* USART2:4Mbps,8N1,RS485(DE=PD4 由 HAL 硬件管理),接收按定长 54 字节 */
    USART_RX485_init_Ex(&huart2, CMD485_FRAME_LEN, Cmd_485_Rx);
}

void Cmd_485_Task(void)
{
#if ROBOT_UPPER_ONLY
    /* 上 MCU(master):每 2ms 发一帧命令,同时捎带 24B 下行调试数据 */
    {
        uint8_t payload[CMD485_DATA_LEN];
        uint8_t dn_chunk[CMD485_HALF_LEN];
        uint8_t frame[CMD485_FRAME_LEN];
        const Cmd_Data_t *cmd = Cmd_Get();
        MiniPC_Cmd_Payload_t p;
        uint8_t flags = CMD485_MODE_ENABLE;
        uint8_t dn_len;
        uint16_t n;

        memset(&p, 0, sizeof(p));

        if (cmd != nullptr)
        {
            p.Mode          = cmd->Mode;
            p.Shoot_Enable  = cmd->Shoot_Enable ? 1u : 0u;
            p.Shoot_Rate    = cmd->Shoot_Rate;
            p.Chassis_Vx    = cmd->Chassis_Vx;
            p.Chassis_Vy    = cmd->Chassis_Vy;
            p.Chassis_W     = cmd->Chassis_W;
            p.Gimbal_Yaw    = cmd->Gimbal_Yaw;
            p.Gimbal_Pitch  = cmd->Gimbal_Pitch;

            if (cmd->Chassis_Spin)
            {
                flags |= CMD485_MODE_SPIN;
            }
        }

        /* 半帧 A 的 Gimbal_Yaw/Pitch 复用为“上板 IMU yaw / yaw 角速度”,
           放在 cmd 赋值之后,避免被鼠标云台目标覆盖 */
        {
            /* Yaw_Total 是 ±π 归一化的,这里展开成连续角,避免跨 ±π 时小陀螺甩 2π */
            static float s_yaw_last = 0.0f;
            static float s_yaw_cont = 0.0f;
            static bool  s_yaw_init = false;
            const Imu_Data_t *uimu = Imu_Get(IMU_CHANNEL_CHASSIS);

            if (uimu != nullptr)
            {
                if (!s_yaw_init)
                {
                    s_yaw_cont = uimu->Yaw_Total;
                    s_yaw_last = uimu->Yaw_Total;
                    s_yaw_init = true;
                }
                else
                {
                    s_yaw_cont += Math_Angle_Diff_Pi(uimu->Yaw_Total, s_yaw_last);
                    s_yaw_last  = uimu->Yaw_Total;
                }
                p.Gimbal_Yaw   = s_yaw_cont;
                p.Gimbal_Pitch = uimu->Gyro[2];
            }
        }

        memcpy(payload, &p, CMD485_HALF_LEN);

        n = Cmd_485_RelayDown_Take(dn_chunk, CMD485_HALF_LEN);
        dn_len = (uint8_t)n;
        memset(&payload[CMD485_HALF_LEN], 0, CMD485_HALF_LEN);
        if (dn_len > 0u)
        {
            memcpy(&payload[CMD485_HALF_LEN], dn_chunk, dn_len);
        }

        Cmd_485_Build(CMD485_CH_CMD,
                      (uint8_t)(flags | (uint8_t)(dn_len << CMD485_MODE_DN_SHIFT)),
                      payload, frame);
        (void)UART_Transmit_Data(&huart2, frame, (uint8_t)CMD485_FRAME_LEN);
    }
#else
    /* 下 MCU(slave):只更新反馈快照;真正的发送在 Cmd_485_Rx 里(收到命令立刻回) */
    {
        const Chassis_Data_t *ch;
        Cmd485_Feedback_t f;

        memset(&f, 0, sizeof(f));
        ch = Chassis_Get();
        if (ch != nullptr)
        {
            f.Vx      = ch->Vx;
            f.Vy      = ch->Vy;
            f.W       = ch->W;
            f.Yaw     = ch->Yaw_Total;
            f.Power_W = ch->Power_Est;
            if (ch->Online)
            {
                f.Status |= 0x05u;      /* bit0 在线 + bit2 IMU */
            }
        }
        s_fb_cache = f;
    }
#endif
}

bool Cmd_485_Get_Chassis(float *__vx, float *__vy, float *__w, uint8_t *__mode)
{
#ifndef ROBOT_UPPER_ONLY
    if (!s_cmd_seen)
    {
        return false;
    }
    if ((uint32_t)(Timestamp_Get_Ms() - s_cmd_ms) > 100u)
    {
        return false;
    }
    if (__vx != nullptr) *__vx = s_cmd_vx;
    if (__vy != nullptr) *__vy = s_cmd_vy;
    if (__w  != nullptr) *__w  = s_cmd_w;
    if (__mode != nullptr) *__mode = s_cmd_flags;
    return true;
#else
    (void)__vx; (void)__vy; (void)__w; (void)__mode;
    return false;
#endif
}

bool Cmd_485_Get_Feedback(float *__vx, float *__vy, float *__w, uint8_t *__status)
{
#if ROBOT_UPPER_ONLY
    if (!s_fb_seen)
    {
        return false;
    }
    if ((uint32_t)(Timestamp_Get_Ms() - s_fb_ms) > 100u)
    {
        return false;
    }
    if (__vx != nullptr) *__vx = s_fb_vx;
    if (__vy != nullptr) *__vy = s_fb_vy;
    if (__w  != nullptr) *__w  = s_fb_w;
    if (__status != nullptr) *__status = s_fb_status;
    return true;
#else
    (void)__vx; (void)__vy; (void)__w; (void)__status;
    return false;
#endif
}

bool Cmd_485_Is_Link_Alive(void)
{
    return s_link_seen && ((uint32_t)(Timestamp_Get_Ms() - s_last_ms) <= 100u);
}

uint32_t Cmd_485_Get_Rx_Count(void)    { return s_rx_count; }
uint32_t Cmd_485_Get_Bad_Count(void)   { return s_bad_count; }
uint32_t Cmd_485_Get_Last_Channel(void){ return s_last_ch; }

int16_t Cmd_485_Get_Fb_Vx(void)        { return (int16_t)(s_fb_vx * 1000.0f); }
int16_t Cmd_485_Get_Fb_Vy(void)        { return (int16_t)(s_fb_vy * 1000.0f); }
int16_t Cmd_485_Get_Fb_W(void)         { return (int16_t)(s_fb_w * 1000.0f); }
uint8_t Cmd_485_Get_Fb_Status(void)    { return s_fb_status; }

float Cmd_485_Get_Upper_Yaw(void)      { return s_upper_yaw; }
float Cmd_485_Get_Gimbal_Yaw_Cmd(void) { return s_upper_gyro_z; }   /* 上板复用该字段发云台 yaw */
bool  Cmd_485_Get_Shoot(uint8_t *__en, uint8_t *__rate)
{
    /* 不做 100ms 新鲜度判定:上板 500Hz 一直在发,偶尔抖动不该把发射打成 0 */
    if (!s_cmd_seen)
    {
        if (__en != nullptr)   { *__en = 0u; }
        if (__rate != nullptr) { *__rate = 0u; }
        return false;
    }
    if (__en != nullptr)   { *__en = s_cmd_shoot_en; }
    if (__rate != nullptr) { *__rate = s_cmd_shoot_rate; }
    return true;
}
float Cmd_485_Get_Upper_Gyro_Z(void)   { return s_upper_gyro_z; }
uint32_t Cmd_485_Get_Upper_Yaw_Ms(void){ return s_upper_yaw_ms; }
