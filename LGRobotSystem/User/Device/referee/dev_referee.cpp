//
// dev_referee.cpp — 裁判系统(帧解析 + 校验 + 发布到 msg_center)
// 说明 / 用法 / 坑 见同目录 dev_referee.md
//

#include "dev_referee.h"

#include "bsp_timestamp.h"
#include "crc8.h"             /* CRC8(poly 0x31, init 0xFF):与上位机协议共用 */
#include "bsp_uart.h"
#include "crc_ccitt.h"          /* CRC16(协议用的就是 CRC-CCITT) */
#include "msg_center.h"

#include <string.h>

static_assert(sizeof(Referee_Data_t) <= MSG_SLOT_SIZE, "Referee_Data_t 超过 MSG_SLOT_SIZE,会被环形数组静默拒收");

/* ============================================================================
 * 内部状态
 * ========================================================================== */

static Referee_Data_t s_ref;
static uint32_t s_published_frames = 0u;
static bool s_published_online = false;

/* 裁判系统 CRC8:poly 0x31 的反射形式(0x8C),init 0xFF。
   注意不能直接用 minipc 的 crc8() —— 那个是逐位 MSB 版本。 */
static uint8_t Referee_Crc8(const uint8_t *__p, uint16_t __len)
{
    uint8_t crc = 0xFFu;
    for (uint16_t i = 0u; i < __len; i++)
    {
        crc ^= __p[i];
        for (uint8_t b = 0u; b < 8u; b++)
        {
            crc = ((crc & 0x01u) != 0u) ? (uint8_t)((crc >> 1) ^ 0x8Cu)
                                        : (uint8_t)(crc >> 1);
        }
    }
    return crc;
}

/* 裁判系统 CRC16:CCITT,init 0xFFFF */
static uint16_t Referee_Crc16(const uint8_t *__p, uint16_t __len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0u; i < __len; i++)
    {
        crc = crc_ccitt_byte(crc, __p[i]);
    }
    return crc;
}

/* ============================================================================
 * 解析
 *   帧:SOF(1) | data_length(2) | seq(1) | CRC8(1) | cmd_id(2) | data(n) | CRC16(2)
 *   data_length = 从 seq 到 data 结束的字节数 = n + 4
 *   帧总长 = data_length + 5
 * ========================================================================== */

static void Referee_Dispatch(uint16_t __cmd_id, const uint8_t *__data, uint16_t __len)
{
    switch (__cmd_id)
    {
    case REFEREE_ID_GAME_STATUS:
        if (__len >= 3u)
        {
            /* 2027:game_type/progress 压在一个字节的低/高 4 位 */
            s_ref.Game_Type = (uint8_t)(__data[0] & 0x0Fu);
            s_ref.Game_Progress = (uint8_t)((__data[0] >> 4) & 0x0Fu);
            s_ref.Stage_Remain_S = (uint16_t)((uint16_t)__data[1] |
                                              ((uint16_t)__data[2] << 8));
        }
        break;

    case REFEREE_ID_ROBOT_STATUS:
        if (__len >= 13u)
        {
            s_ref.Robot_Id = __data[0];
            s_ref.Robot_Level = __data[1];
            s_ref.Hp = (uint16_t)((uint16_t)__data[2] | ((uint16_t)__data[3] << 8));
            s_ref.Max_Hp = (uint16_t)((uint16_t)__data[4] | ((uint16_t)__data[5] << 8));
            s_ref.Barrel_Cooling = (uint16_t)((uint16_t)__data[6] | ((uint16_t)__data[7] << 8));
            s_ref.Heat_Limit = (uint16_t)((uint16_t)__data[8] | ((uint16_t)__data[9] << 8));
            s_ref.Chassis_Power_Limit = (uint16_t)((uint16_t)__data[10] | ((uint16_t)__data[11] << 8));
            if (__len >= 16u)
            {
                memcpy(&s_ref.Shooter_Speed_Limit, &__data[12], sizeof(float));
            }
            if (__len >= 17u)
            {
                s_ref.Power_Management = __data[16];
                s_ref.Power_Gimbal_Out = (uint8_t)((__data[16] >> 0) & 1u);
                s_ref.Power_Chassis_Out = (uint8_t)((__data[16] >> 1) & 1u);
                s_ref.Power_Shooter_Out = (uint8_t)((__data[16] >> 2) & 1u);
            }
        }
        break;

    case REFEREE_ID_POWER_HEAT:
        if (__len >= 4u)
        {
            if (__len >= 12u)
            {
                s_ref.Chassis_Power_Buffer = (uint16_t)((uint16_t)__data[8] |
                                                        ((uint16_t)__data[9] << 8));
                s_ref.Heat = (uint16_t)((uint16_t)__data[10] |
                                        ((uint16_t)__data[11] << 8));
                s_ref.Heat_Update_Ms = (uint32_t)Timestamp_Get_Ms();
            }
            if (__len >= 8u)
            {
                memcpy(&s_ref.Chassis_Power, &__data[4], sizeof(float));
                s_ref.Power_Update_Ms = s_ref.Heat_Update_Ms;
            }
        }
        break;

    case REFEREE_ID_HURT:
        if (__len >= 4u)
        {
            s_ref.Armor_Id = (uint8_t)(__data[0] & 0x0Fu);
            s_ref.Hurt_Reason = (uint8_t)((__data[0] >> 4) & 0x0Fu);
            s_ref.Hurt_Seq = (uint16_t)((uint16_t)__data[1] |
                                        ((uint16_t)__data[2] << 8));
        }
        break;

    case REFEREE_ID_SHOOT:
        if (__len >= 7u)
        {
            memcpy(&s_ref.Initial_Speed, &__data[3], sizeof(float));
        }
        break;

    case REFEREE_ID_ALLOWANCE:
        if (__len >= 2u)
        {
            s_ref.Projectile_Allowance = (uint16_t)((uint16_t)__data[0] |
                                                    ((uint16_t)__data[1] << 8));
        }
        break;

    case REFEREE_ID_ROBOT_HP:
        /* 0x0003 第 1 字节是机器人编号,不能直接当血量 */
        if (__len >= 4u)
        {
            s_ref.Hp = (uint16_t)((uint16_t)__data[2] | ((uint16_t)__data[3] << 8));
        }
        break;

    default:
        break;      /* 其它 ID 先不收:要哪个就在这里加一个 case */
    }
}

static void Referee_Uart_Callback(UART_HandleTypeDef *__huart, uint8_t *__buf, uint16_t __size)
{
    uint16_t off = 0u;

    (void)__huart;

    if ((__buf == nullptr) || (__size < REFEREE_MIN_FRAME))
    {
        s_ref.Reject_Count++;
        return;
    }

    /* 一个 IDLE 块里可能粘了多帧,逐帧切 */
    while ((uint16_t)(off + REFEREE_MIN_FRAME) <= __size)
    {
        const uint8_t *f = &__buf[off];
        uint16_t data_len;
        uint16_t frame_len;
        uint16_t cmd_id;
        uint16_t crc16;

        if (f[0] != REFEREE_SOF)
        {
            off++;
            s_ref.Reject_Count++;
            continue;
        }

        data_len = (uint16_t)((uint16_t)f[1] | ((uint16_t)f[2] << 8));
        if (data_len > REFEREE_MAX_DATA)
        {
            off++;
            s_ref.Reject_Count++;
            continue;
        }
        frame_len = (uint16_t)(data_len + 9u);
        if ((uint16_t)(off + frame_len) > __size)
        {
            break;                      /* 半帧:等下一块 */
        }

        if (Referee_Crc8(f, 4u) != f[4])
        {
            off++;
            s_ref.Crc8_Errors++;
            continue;
        }

        crc16 = (uint16_t)((uint16_t)f[frame_len - 2u] |
                           ((uint16_t)f[frame_len - 1u] << 8));
        if (Referee_Crc16(f, (uint16_t)(frame_len - 2u)) != crc16)
        {
            off++;
            s_ref.Crc16_Errors++;
            continue;
        }

        cmd_id = (uint16_t)((uint16_t)f[5] | ((uint16_t)f[6] << 8));
        Referee_Dispatch(cmd_id, &f[7], data_len);

        s_ref.Rx_Frames++;
        s_ref.Last_Rx_Ms = (uint32_t)Timestamp_Get_Ms();
        s_ref.Online = true;
        off = (uint16_t)(off + frame_len);
    }
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 初始化
 */
bool Referee_Init(UART_HandleTypeDef *__huart)
{
    memset(&s_ref, 0, sizeof(s_ref));
    s_published_frames = 0u;
    s_published_online = false;
    s_ref.Online = false;

    USART_init((__huart != nullptr) ? __huart : &huart1, Referee_Uart_Callback);
    return true;
}

/**
 * @brief 周期跑:掉线检测 + 有新数据就发布
 */
void Referee_Task(void)
{
    uint32_t now_ms = (uint32_t)Timestamp_Get_Ms();

    if ((s_ref.Rx_Frames == 0u) || ((uint32_t)(now_ms - s_ref.Last_Rx_Ms) > REFEREE_LOST_TIMEOUT_MS))
    {
        if (s_ref.Online)
        {
            s_ref.Lost_Count++;
        }
        s_ref.Online = false;
    }

    if ((s_ref.Rx_Frames != s_published_frames) ||
        (s_ref.Online != s_published_online))
    {
        s_published_frames = s_ref.Rx_Frames;
        s_published_online = s_ref.Online;
        (void)Msg_Publish(MSG_TOPIC_REFEREE, &s_ref, (uint16_t)sizeof(s_ref));
    }
}

/**
 * @brief 读当前数据
 */
const Referee_Data_t *Referee_Get(void)
{
    return &s_ref;
}

/**
 * @brief 是否在线
 */
bool Referee_Is_Online(void)
{
    return s_ref.Online;
}

/**
 * @brief 比赛是否进行中
 */
bool Referee_Is_In_Match(void)
{
    return (s_ref.Game_Progress == 4u);
}

/**
 * @brief 当前阶段剩余时间
 */
uint16_t Referee_Get_Stage_Remain(void)
{
    return s_ref.Stage_Remain_S;
}
