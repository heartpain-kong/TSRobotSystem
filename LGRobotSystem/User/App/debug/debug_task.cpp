//
// debug_task.cpp ?(USART3 / UART7 ?? + ?+ )
//  /  / ? debug_task.md
//

#include "debug_task.h"

#include "bsp_timestamp.h"      /*  TIM5 */
#include "bsp_uart.h"           /* USART_init: 0x5C  */
#include "chassis_task.h"       /* Chassis_Data_t */
#include "crc_ccitt.h"          /* CRC16 */
#include "error_task.h"         /*  / ?*/
#include "gimbal_task.h"        /* Gimbal_Data_t */
#include "imu_task.h"           /* Imu_Data_t */
#include "motor_task.h"         /* ?*/
#include "cmd_485.h"           /* 上/下 MCU 链路(USART2 RS485)计数 */
#include "dev_log.h"            /* ?Log_Get_*) */
#include "dev_bmi088.h"        /* BMI088_Is_Online / Get_Error_Count / acc_conf */
#include "dev_electric.h"      /* PE2/PE3  */
#include "dev_remote.h"        /* ?: + /// */
#include "dev_bmi088.h"
#include "bsp_flash.h"          /* FLASH_LOG_ADDR */
#include "math_support.h"       /* MATH_RPM_TO_RADPS */

#include "main.h"               /* g_fault_* + HAL ?*/

#include <stdio.h>   /* snprintf:?*/
#include <string.h>

/* 下 MCU:自己的调试串口 TX 全关,数据只走 485(上板 [L] 转发) */
#define DEBUG_UART_TX_ENABLE    ROBOT_UPPER_ONLY

/* RTOS ??User/App/robot/robot.cpp ?RTOS_Update_Diagnostics )?*/
extern volatile uint32_t hwm_motor;
extern volatile uint32_t hwm_robot;
extern volatile uint32_t hwm_error;
extern volatile uint32_t hwm_cmd;
extern volatile uint32_t hwm_imu;
extern volatile uint32_t hwm_chassis;
extern volatile uint32_t hwm_gimbal;
extern volatile uint32_t hwm_highRise;
extern volatile uint32_t flash_jedec_id;
extern volatile bool flash_init_ok;
extern volatile bool bmi088_init_ok;
extern volatile bool ws2812_init_ok;
extern volatile bool buzzer_init_ok;
extern volatile uint32_t g_error_tick;
extern volatile uint32_t g_task_tick[DEBUG_TICK_NUM];
extern volatile uint32_t flash_diag_state;
extern volatile uint32_t flash_diag_err;
extern volatile uint32_t flash_diag_cmd;
extern volatile uint32_t flash_diag_rcv;
extern volatile uint32_t flash_diag_raw;   /* 8 (robot_task.cpp ) */

/* ============================================================================
 * 
 * ========================================================================== */

/* :8 ?+ payload(48 + 816 + 24 /) + 2  CRC = 210 */
#define DEBUG_PAYLOAD_SIZE      (DEBUG_PL_EXT + DEBUG_EXT_SIZE)   /* 220: external IMU at the end */
#define DEBUG_FRAME_SIZE        (8u + DEBUG_PAYLOAD_SIZE + 2u)

static_assert(DEBUG_FRAME_SIZE <= 256u, "debug frame too large for DMA buffer");

/* ============================================================================
 * ?
 * ========================================================================== */

/* ??DMA ? .AXI_SRAM:DMA ?+  D-Cache )
   ?bsp_uart ?UART_Transmit_Data() ? 64B ,??*/
static uint8_t s_frame[256];        /* ?) */
static uint8_t s_tx7[256] __attribute__((section(".AXI_SRAM"), aligned(32)));   /* UART7  DMA  */
static uint8_t s_json_tx7[1024] __attribute__((section(".AXI_SRAM"), aligned(32)));
static uint8_t s_json_tx10[1024] __attribute__((section(".AXI_SRAM"), aligned(32)));
static uint8_t s_json_buf[1024];
static uint16_t s_json_len = 0u;
static uint32_t s_next_json_us = 0u;
static uint8_t s_tx10[256] __attribute__((section(".AXI_SRAM"), aligned(32)));  /* USART10 ?DMA  */
static uint8_t s_txt7[160] __attribute__((section(".AXI_SRAM"), aligned(32)));   /* ?UART7) */
static uint8_t s_txt10[160] __attribute__((section(".AXI_SRAM"), aligned(32)));  /* ?USART10) */
static char s_txt[128];

/* PID (0x5C 0x03/0x04)??boot ?*/
static uint8_t s_pid_txt[192];
static uint8_t s_pid_txt7[192] __attribute__((section(".AXI_SRAM"), aligned(32)));
static uint8_t s_pid_txt10[192] __attribute__((section(".AXI_SRAM"), aligned(32)));

static uint32_t s_next_txt_ms = 0u;

static uint16_t s_seq = 0u;
static uint32_t s_tx_frames = 0u;
static uint32_t s_tx_skip = 0u;
static uint32_t s_next_us = 0u;
static bool s_inited = false;

/* ---- 0x5C :ISR ?Debug_Task ?----
   ??USB-TTL/IDLE  28 B ?PID ?
      (?8+20),?0x03 ?*/
#define DEBUG_CMD_RING_SIZE 128u
#define DEBUG_CMD_BUF_SIZE  32u
static volatile uint8_t s_cmd_ring[DEBUG_CMD_RING_SIZE];
static volatile uint16_t s_cmd_ring_head = 0u;
static volatile uint16_t s_cmd_ring_tail = 0u;
static uint8_t s_cmd_parse[DEBUG_CMD_BUF_SIZE];
static uint16_t s_cmd_parse_len = 0u;

/* : 0x5C ? RX/? */
volatile uint32_t g_debug_cmd_count = 0u;
/* :?0 ?TX/RX ?? */
volatile uint32_t g_debug_rx_bytes = 0u;

/* ============================================================================
 * ???memcpy,/)
 * ========================================================================== */

static void Put_U16(uint8_t *__p, uint16_t __v)
{
    __p[0] = (uint8_t)(__v & 0xFFu);
    __p[1] = (uint8_t)((__v >> 8) & 0xFFu);
}

static void Put_U32(uint8_t *__p, uint32_t __v)
{
    __p[0] = (uint8_t)(__v & 0xFFu);
    __p[1] = (uint8_t)((__v >> 8) & 0xFFu);
    __p[2] = (uint8_t)((__v >> 16) & 0xFFu);
    __p[3] = (uint8_t)((__v >> 24) & 0xFFu);
}

static void Put_F32(uint8_t *__p, float __v)
{
    memcpy(__p, &__v, sizeof(float));
}

/* ---- ? snprintf:?newlib)?---- */

static uint16_t Text_Put_Str(uint8_t *__p, const char *__s)
{
    uint16_t n = 0u;

    while ((__s != nullptr) && (__s[n] != '\0'))
    {
        __p[n] = (uint8_t)__s[n];
        n++;
    }
    return n;
}

static uint16_t Text_Put_U32(uint8_t *__p, uint32_t __v)
{
    uint8_t tmp[10];
    uint16_t n = 0u;
    uint16_t i;

    if (__v == 0u)
    {
        __p[0] = (uint8_t)'0';
        return 1u;
    }
    while ((__v > 0u) && (n < sizeof(tmp)))
    {
        tmp[n++] = (uint8_t)('0' + (__v % 10u));
        __v /= 10u;
    }
    for (i = 0u; i < n; i++)
    {
        __p[i] = tmp[n - 1u - i];
    }
    return n;
}

static uint16_t Text_Put_Hex6(uint8_t *__p, uint32_t __v)
{
    static const char hex[] = "0123456789ABCDEF";
    uint16_t i;

    __p[0] = (uint8_t)'0';
    __p[1] = (uint8_t)'x';
    for (i = 0u; i < 6u; i++)
    {
        __p[2u + i] = (uint8_t)hex[(__v >> ((5u - i) * 4u)) & 0x0Fu];
    }
    return 8u;
}

/*  3 ??newlib ?printf(/?flash) */
static uint16_t Text_Put_F32_3(uint8_t *__p, float __v)
{
    uint16_t n = 0u;

    if (__v < 0.0f)
    {
        __p[n++] = (uint8_t)'-';
        __v = -__v;
    }

    const uint32_t m = (uint32_t)(__v * 1000.0f + 0.5f);
    const uint32_t frac = m % 1000u;

    n += Text_Put_U32(&__p[n], m / 1000u);
    __p[n++] = (uint8_t)'.';
    __p[n++] = (uint8_t)('0' + (frac / 100u));
    __p[n++] = (uint8_t)('0' + ((frac / 10u) % 10u));
    __p[n++] = (uint8_t)('0' + (frac % 10u));

    return n;
}

/*  6 ?PID ?0.0001 ?3 ?*/
static uint16_t Text_Put_F32_6(uint8_t *__p, float __v)
{
    uint16_t n = 0u;

    if (__v < 0.0f)
    {
        __p[n++] = (uint8_t)'-';
        __v = -__v;
    }

    const uint32_t m = (uint32_t)(__v * 1000000.0f + 0.5f);
    const uint32_t frac = m % 1000000u;

    n += Text_Put_U32(&__p[n], m / 1000000u);
    __p[n++] = (uint8_t)'.';
    __p[n++] = (uint8_t)('0' + ((frac / 100000u) % 10u));
    __p[n++] = (uint8_t)('0' + ((frac / 10000u) % 10u));
    __p[n++] = (uint8_t)('0' + ((frac / 1000u) % 10u));
    __p[n++] = (uint8_t)('0' + ((frac / 100u) % 10u));
    __p[n++] = (uint8_t)('0' + ((frac / 10u) % 10u));
    __p[n++] = (uint8_t)('0' + (frac % 10u));

    return n;
}

/* ?***(;?Debug_Send_One ?s_frame) */
/* ---- self-describing JSON channels ---- */
static uint16_t Json_Key(uint8_t *__p, const char *__key)
{
    uint16_t n = 0u;
    __p[n++] = (uint8_t)'"';
    n += Text_Put_Str(&__p[n], __key);
    __p[n++] = (uint8_t)'"';
    __p[n++] = (uint8_t)':';
    return n;
}

static uint16_t Json_F32(uint8_t *__p, const char *__key, float __v)
{
    uint16_t n = Json_Key(__p, __key);
    n += Text_Put_F32_3(&__p[n], __v);
    return n;
}

static uint16_t Json_U32(uint8_t *__p, const char *__key, uint32_t __v)
{
    uint16_t n = Json_Key(__p, __key);
    n += Text_Put_U32(&__p[n], __v);
    return n;
}

static bool Debug_Send_Raw(UART_HandleTypeDef *__huart, uint8_t *__buf, const void *__src, uint16_t __len)
{
#if !DEBUG_UART_TX_ENABLE
    /* 下板:调试串口不输出,数据只走 485 透传 */
    (void)__huart; (void)__buf; (void)__src; (void)__len;
    return false;
#else
    if ((__huart == nullptr) || (__buf == nullptr) || (__src == nullptr))
    {
        return false;
    }
    if (HAL_UART_GetState(__huart) != HAL_UART_STATE_READY)
    {
        return false;                      /*  */
    }
    memcpy(__buf, __src, __len);
    return (HAL_UART_Transmit_DMA(__huart, __buf, __len) == HAL_OK);
#endif
}

/* 0x5C PID :?APP ?
   [LOG] PID set speed(Kp=1.500 Ki=0.200 Kd=0.010) pos(Kp=2.500 Ki=0.100 Kd=0.050)
   ?[LOG],APP ?FrameDecoder ?*/
static void Debug_Send_Pid_Line(const char *__tag)
{
    float pid[6];
    uint16_t n = 0u;

    motor_task_debug_get_pid(pid);

    n += Text_Put_Str(&s_pid_txt[n], "[LOG] PID ");
    n += Text_Put_Str(&s_pid_txt[n], __tag);
    n += Text_Put_Str(&s_pid_txt[n], " speed(Kp=");
    n += Text_Put_F32_6(&s_pid_txt[n], pid[0]);
    n += Text_Put_Str(&s_pid_txt[n], " Ki=");
    n += Text_Put_F32_6(&s_pid_txt[n], pid[1]);
    n += Text_Put_Str(&s_pid_txt[n], " Kd=");
    n += Text_Put_F32_6(&s_pid_txt[n], pid[2]);
    n += Text_Put_Str(&s_pid_txt[n], ") pos(Kp=");
    n += Text_Put_F32_6(&s_pid_txt[n], pid[3]);
    n += Text_Put_Str(&s_pid_txt[n], " Ki=");
    n += Text_Put_F32_6(&s_pid_txt[n], pid[4]);
    n += Text_Put_Str(&s_pid_txt[n], " Kd=");
    n += Text_Put_F32_6(&s_pid_txt[n], pid[5]);
    n += Text_Put_Str(&s_pid_txt[n], ")\r\n");

    (void)Debug_Send_Raw(&huart7, s_pid_txt7, s_pid_txt, n);
#if DEBUG_USE_USART10
    (void)Debug_Send_Raw(&huart10, s_pid_txt10, s_pid_txt, n);
#endif
}

/* ?PID :[LOG] PID FR0 Kp=... Ki=... Kd=... */
static void Debug_Send_Friction_Pid_Line(uint8_t __idx)
{
    float pid[3];
    uint16_t n = 0u;

    motor_task_friction_get_pid(__idx, pid);

    n += Text_Put_Str(&s_pid_txt[n], "[LOG] PID FR");
    s_pid_txt[n++] = (uint8_t)('0' + (__idx & 1u));
    n += Text_Put_Str(&s_pid_txt[n], " Kp=");
    n += Text_Put_F32_6(&s_pid_txt[n], pid[0]);
    n += Text_Put_Str(&s_pid_txt[n], " Ki=");
    n += Text_Put_F32_6(&s_pid_txt[n], pid[1]);
    n += Text_Put_Str(&s_pid_txt[n], " Kd=");
    n += Text_Put_F32_6(&s_pid_txt[n], pid[2]);
    n += Text_Put_Str(&s_pid_txt[n], "\r\n");

    (void)Debug_Send_Raw(&huart7, s_pid_txt7, s_pid_txt, n);
#if DEBUG_USE_USART10
    (void)Debug_Send_Raw(&huart10, s_pid_txt10, s_pid_txt, n);
#endif
}

/* ?(?), */
static bool Debug_Send_One(UART_HandleTypeDef *__huart, uint8_t *__buf, uint16_t __len)
{
#if !DEBUG_UART_TX_ENABLE
    (void)__huart; (void)__buf; (void)__len;
    return false;
#else
    if (__huart == nullptr)
    {
        return false;
    }
    if (HAL_UART_GetState(__huart) != HAL_UART_STATE_READY)
    {
        s_tx_skip++;
        return false;
    }
    memcpy(__buf, s_frame, __len);
    if (HAL_UART_Transmit_DMA(__huart, __buf, __len) != HAL_OK)
    {
        s_tx_skip++;
        return false;
    }
    return true;
#endif
}

/* ============================================================================
 * 0x5C 
 *   ?[0x5C][cmd][seq_h][seq_l][payload]
 *   0x01: int16  [rpm]
 *   0x02: int32  [0.001]
 *   0x03: 6f32 PID( Kp/Ki/Kd, Kp/Ki/Kd)
 *   0x05: 
 * ========================================================================== */
static void Debug_Rx_Callback(UART_HandleTypeDef *__huart, uint8_t *__buf, uint16_t __size)
{
    (void)__huart;

    if ((__buf == nullptr) || (__size == 0u))
    {
        return;
    }

    g_debug_rx_bytes += __size;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint16_t i = 0u; i < __size; i++)
    {
        const uint16_t next = (uint16_t)((s_cmd_ring_head + 1u) % DEBUG_CMD_RING_SIZE);
        if (next != s_cmd_ring_tail)
        {
            s_cmd_ring[s_cmd_ring_head] = __buf[i];
            s_cmd_ring_head = next;
        }
        /* , */
    }
    __set_PRIMASK(primask);
}

#if !ROBOT_UPPER_ONLY
/* 把上板透传下来的串口字节(通道 2)塞进本板的调试命令环形缓冲,
   这样上位机仍用 0x5C 帧就能给下板调参 */
static void Debug_RelayDown_Poll(void)
{
    uint8_t buf[64];
    uint16_t n;
    uint32_t primask;

    while ((n = Cmd_485_RelayDown_Take(buf, (uint16_t)sizeof(buf))) > 0u)
    {
        primask = __get_PRIMASK();
        __disable_irq();
        for (uint16_t i = 0u; i < n; i++)
        {
            const uint16_t next = (uint16_t)((s_cmd_ring_head + 1u) % DEBUG_CMD_RING_SIZE);
            if (next != s_cmd_ring_tail)
            {
                s_cmd_ring[s_cmd_ring_head] = buf[i];
                s_cmd_ring_head = next;
            }
        }
        __set_PRIMASK(primask);
    }
}
#endif

static void Debug_Handle_Cmd(const uint8_t *__buf, uint16_t __len)
{
    uint16_t off = 0u;

    while ((uint16_t)(off + 4u) <= __len)
    {
        if (__buf[off] != 0x5Cu)
        {
            off++;                      /* ?*/
            continue;
        }

        switch (__buf[off + 1u])
        {
        case 0x01:  /*  [rpm] */
        {
            if ((uint16_t)(off + 6u) > __len) { return; }
            int16_t rpm = 0;
            memcpy(&rpm, &__buf[off + 4u], sizeof(rpm));
            motor_task_debug_set_speed_rpm(rpm);
            g_debug_cmd_count++;
            off = (uint16_t)(off + 6u);
            break;
        }
        case 0x02:  /*  [0.001] */
        {
            if ((uint16_t)(off + 8u) > __len) { return; }
            int32_t mdeg = 0;
            memcpy(&mdeg, &__buf[off + 4u], sizeof(mdeg));
            motor_task_debug_set_pos_mdeg(mdeg);
            g_debug_cmd_count++;
            off = (uint16_t)(off + 8u);
            break;
        }
        case 0x03:  /* PID:  Kp/Ki/Kd +  Kp/Ki/Kd */
        {
            if ((uint16_t)(off + 28u) > __len) { return; }
            float p[6];
            memcpy(p, &__buf[off + 4u], sizeof(p));
            motor_task_debug_set_pid(p[0], p[1], p[2], p[3], p[4], p[5]);
            Debug_Send_Pid_Line("set");
            g_debug_cmd_count++;
            off = (uint16_t)(off + 28u);
            break;
        }
        case 0x04:  /* PID :5C 04 00 01 */
        {
            if ((uint16_t)(off + 4u) > __len) { return; }
            Debug_Send_Pid_Line("now");
            g_debug_cmd_count++;
            off = (uint16_t)(off + 4u);
            break;
        }
        case 0x05:  /*  */
        {
            if ((uint16_t)(off + 5u) > __len) { return; }
            motor_task_debug_stop();
            g_debug_cmd_count++;
            off = (uint16_t)(off + 5u);
            break;
        }
        case 0x06:  /* :[idx u8][rpm i16],idx 0=207,1=208 */
        {
            if ((uint16_t)(off + 7u) > __len) { return; }
            uint8_t idx = __buf[off + 4u];
            int16_t rpm = 0;
            memcpy(&rpm, &__buf[off + 5u], sizeof(rpm));
            if (idx <= 1u)
            {
                motor_task_friction_set_rpm(idx, (float)rpm);
                g_debug_cmd_count++;
            }
            off = (uint16_t)(off + 7u);
            break;
        }
        case 0x07:  /* ?PID:[idx][kp][ki][kd] */
        {
            if ((uint16_t)(off + 17u) > __len) { return; }
            uint8_t idx = __buf[off + 4u];
            float kp, ki, kd;
            memcpy(&kp, &__buf[off + 5u], 4);
            memcpy(&ki, &__buf[off + 9u], 4);
            memcpy(&kd, &__buf[off + 13u], 4);
            if (idx <= 1u)
            {
                motor_task_friction_set_pid(idx, kp, ki, kd);
                Debug_Send_Friction_Pid_Line(idx);
                g_debug_cmd_count++;
            }
            off = (uint16_t)(off + 17u);
            break;
        }
        case 0x09:  /* 0x201 drive speed [rpm] */
        {
            if ((uint16_t)(off + 6u) > __len) { return; }
            int16_t rpm = 0;
            memcpy(&rpm, &__buf[off + 4u], sizeof(rpm));
            motor_task_debug_set_drive_rpm(rpm);
            g_debug_cmd_count++;
            off = (uint16_t)(off + 6u);
            break;
        }
        case 0x0F:  /* 0x206 trigger speed [rpm] */
        {
            if ((uint16_t)(off + 6u) > __len) { return; }
            int16_t rpm = 0;
            memcpy(&rpm, &__buf[off + 4u], sizeof(rpm));
            motor_task_aux_set_rpm(0u, rpm);
            g_debug_cmd_count++;
            off = (uint16_t)(off + 6u);
            break;
        }
        case 0x10:  /* 0x205 6020 speed [rpm] */
        {
            if ((uint16_t)(off + 6u) > __len) { return; }
            int16_t rpm = 0;
            memcpy(&rpm, &__buf[off + 4u], sizeof(rpm));
            motor_task_aux_set_rpm(1u, rpm);
            g_debug_cmd_count++;
            off = (uint16_t)(off + 6u);
            break;
        }
        case 0x12:  /* 6020 angle [mdeg] */
        {
            if ((uint16_t)(off + 8u) > __len) { return; }
            int32_t mdeg = 0;
            memcpy(&mdeg, &__buf[off + 4u], sizeof(mdeg));
            motor_task_aux_set_pos(1u, (float)mdeg * 0.001f * (PI / 180.0f));
            g_debug_cmd_count++;
            off = (uint16_t)(off + 8u);
            break;
        }
        case 0x20:  /* 1kHz 电机紧凑帧开关:[on u8] */
        {
            if ((uint16_t)(off + 5u) > __len) { return; }
            motor_task_set_fast_stream(__buf[off + 4u]);
            g_debug_cmd_count++;
            off = (uint16_t)(off + 5u);
            break;
        }
        case 0x13:  /* 6020 位置环:[idx u8][pos_kp][vel_damp][max_w rad/s] */
        {
            if ((uint16_t)(off + 17u) > __len) { return; }
            uint8_t idx = __buf[off + 4u];
            float kp, damp, wmax;
            memcpy(&kp,    &__buf[off + 5u], 4);
            memcpy(&damp,  &__buf[off + 9u], 4);
            memcpy(&wmax,  &__buf[off + 13u], 4);
            if (idx == 1u)
            {
                motor_task_aux_set_pos_pid(idx, kp, damp, wmax);
                g_debug_cmd_count++;
            }
            off = (uint16_t)(off + 17u);
            break;
        }
        case 0x21:  /* 调试模式:拨盘转速 [rpm i16] */
        {
            if ((uint16_t)(off + 6u) > __len) { return; }
            int16_t rpm = (int16_t)((uint16_t)__buf[off + 4u]
                                    | ((uint16_t)__buf[off + 5u] << 8));
            motor_task_trigger_set_rpm((float)rpm);
            g_debug_cmd_count++;
            off = (uint16_t)(off + 6u);
            break;
        }
        case 0x23:  /* 6020 调试开关:[on u8] 1=底盘不再控制 6020 */
        {
            if ((uint16_t)(off + 5u) > __len) { return; }
            motor_task_set_6020_debug(__buf[off + 4u]);
            g_debug_cmd_count++;
            off = (uint16_t)(off + 5u);
            break;
        }
        case 0x22:  /* 舵轮调试开关:[on u8] 1=暂停底盘舵向下发 */
        {
            if ((uint16_t)(off + 5u) > __len) { return; }
            motor_task_set_steer_debug(__buf[off + 4u]);
            g_debug_cmd_count++;
            off = (uint16_t)(off + 5u);
            break;
        }
        case 0x11:  /* aux PID: [idx u8][kp][ki][kd] */
        {
            if ((uint16_t)(off + 17u) > __len) { return; }
            uint8_t idx = __buf[off + 4u];
            float kp, ki, kd;
            memcpy(&kp, &__buf[off + 5u], 4);
            memcpy(&ki, &__buf[off + 9u], 4);
            memcpy(&kd, &__buf[off + 13u], 4);
            motor_task_aux_set_pid(idx, kp, ki, kd);
            g_debug_cmd_count++;
            off = (uint16_t)(off + 17u);
            break;
        }
        case 0x0E:  /* 0x204 steer PID [6f] */
        {
            if ((uint16_t)(off + 28u) > __len) { return; }
            float skp, ski, skd, pkp, pki, pkd;
            memcpy(&skp, &__buf[off + 4u], 4);
            memcpy(&ski, &__buf[off + 8u], 4);
            memcpy(&skd, &__buf[off + 12u], 4);
            memcpy(&pkp, &__buf[off + 16u], 4);
            memcpy(&pki, &__buf[off + 20u], 4);
            memcpy(&pkd, &__buf[off + 24u], 4);
            motor_task_debug_set_steer2_pid(skp, ski, skd, pkp, pki, pkd);
            g_debug_cmd_count++;
            off = (uint16_t)(off + 28u);
            break;
        }
        case 0x0B:  /* 0x204 steer angle [mdeg] */
        {
            if ((uint16_t)(off + 8u) > __len) { return; }
            int32_t mdeg = 0;
            memcpy(&mdeg, &__buf[off + 4u], sizeof(mdeg));
            motor_task_debug_set_steer2_mdeg(mdeg);
            g_debug_cmd_count++;
            off = (uint16_t)(off + 8u);
            break;
        }
        case 0x0C:  /* 0x204 steer speed [rpm] */
        {
            if ((uint16_t)(off + 6u) > __len) { return; }
            int16_t rpm = 0;
            memcpy(&rpm, &__buf[off + 4u], sizeof(rpm));
            motor_task_debug_set_steer2_rpm(rpm);
            g_debug_cmd_count++;
            off = (uint16_t)(off + 6u);
            break;
        }
        case 0x0D:  /* 0x203 drive speed [rpm] */
        {
            if ((uint16_t)(off + 6u) > __len) { return; }
            int16_t rpm = 0;
            memcpy(&rpm, &__buf[off + 4u], sizeof(rpm));
            motor_task_debug_set_drive2_rpm(rpm);
            g_debug_cmd_count++;
            off = (uint16_t)(off + 6u);
            break;
        }
        case 0x0A:  /* 0x201 drive PID [kp][ki][kd] */
        {
            if ((uint16_t)(off + 16u) > __len) { return; }
            float kp, ki, kd;
            memcpy(&kp, &__buf[off + 4u], 4);
            memcpy(&ki, &__buf[off + 8u], 4);
            memcpy(&kd, &__buf[off + 12u], 4);
            motor_task_debug_set_drive_pid(kp, ki, kd);
            g_debug_cmd_count++;
            off = (uint16_t)(off + 16u);
            break;
        }
        case 0x08:  /* ?PID :[idx] */
        {
            if ((uint16_t)(off + 5u) > __len) { return; }
            uint8_t idx = __buf[off + 4u];
            if (idx <= 1u)
            {
                Debug_Send_Friction_Pid_Line(idx);
                g_debug_cmd_count++;
            }
            off = (uint16_t)(off + 5u);
            break;
        }
        default:
            off = (uint16_t)(off + 4u);
            break;
        }
    }
}

static uint16_t Debug_Cmd_Need(uint8_t __cmd)
{
    switch (__cmd)
    {
    case 0x01u: return 6u;
    case 0x02u: return 8u;
    case 0x03u: return 28u;
    case 0x04u: return 4u;
    case 0x05u: return 5u;
    case 0x06u: return 7u;
    case 0x07u: return 17u;
    case 0x08u: return 5u;
    case 0x09u: return 6u;
    case 0x0Au: return 16u;
    case 0x0Bu: return 8u;
    case 0x0Cu: return 6u;
    case 0x0Du: return 6u;
    case 0x0Eu: return 28u;
    case 0x0Fu: return 6u;
    case 0x10u: return 6u;
    case 0x11u: return 17u;
    case 0x12u: return 8u;
    case 0x13u: return 17u;
    case 0x20u: return 5u;
    case 0x21u: return 6u;
    case 0x22u: return 5u;
    case 0x23u: return 5u;
    default:    return 4u;
    }
}

/* ?0x5C ??Debug_Handle_Cmd?
    PID ?*/
static void Debug_Process_Rx(void)
{
    while (s_cmd_ring_tail != s_cmd_ring_head)
    {
        const uint8_t b = s_cmd_ring[s_cmd_ring_tail];
        s_cmd_ring_tail = (uint16_t)((s_cmd_ring_tail + 1u) % DEBUG_CMD_RING_SIZE);

        if ((s_cmd_parse_len == 0u) && (b != 0x5Cu))
        {
            continue;   /* ?*/
        }

        if (s_cmd_parse_len >= sizeof(s_cmd_parse))
        {
            s_cmd_parse_len = 0u;
        }
        s_cmd_parse[s_cmd_parse_len++] = b;

        if (s_cmd_parse_len < 2u)
        {
            continue;
        }

        const uint16_t need = Debug_Cmd_Need(s_cmd_parse[1]);
        if (s_cmd_parse_len >= need)
        {
            Debug_Handle_Cmd(s_cmd_parse, s_cmd_parse_len);
            s_cmd_parse_len = 0u;
        }
    }
}

/* ============================================================================
 * 
 * ========================================================================== */

bool Debug_Init(void)
{
    memset(s_frame, 0, sizeof(s_frame));
    s_seq = 0u;
    s_tx_frames = 0u;
    s_tx_skip = 0u;
    s_next_us = Timestamp_Get_Us32();
    s_next_json_us = Timestamp_Get_Us32();

    s_cmd_ring_head = 0u;
    s_cmd_ring_tail = 0u;
    s_cmd_parse_len = 0u;

    /*  0x5C (RX DMA  CubeMX  Circular) */
#if DEBUG_USE_UART7
    USART_init(&huart7, Debug_Rx_Callback);
#endif
#if DEBUG_USE_USART10
    USART_init(&huart10, Debug_Rx_Callback);
#endif

    s_inited = true;
    return true;
}

#if DEBUG_MOTOR_PID_ONLY
/* ? + ??
   payload(30B):
     [0] u8  id
     [1] u8  flags(bit0 init / bit1 online / bit2 err)
     [2] f32 ?rpm
     [6] f32 ?rpm(=4)
     [10] f32 PID  rpm(?
     [14] f32 PID  rpm(?
     [18] f32 PID  rpm
     [22] f32 PID (Nm)
     [26] f32 (rad,?
*/
static void Debug_Send_MotorPid(void)
{
    uint8_t *pl = &s_frame[8];
    const uint16_t len = DEBUG_MOTOR_PID_FRAME;
    uint8_t flags = 0u;

    if (motor_task_is_init_ok(0u)) { flags |= 0x01u; }
    if (motor_task_is_online(0u))  { flags |= 0x02u; }
    if (motor_task_get_m_error(0u) != 0u) { flags |= 0x04u; }

    const float joint_rpm = motor_task_get_now_w(0u) / MATH_RPM_TO_RADPS;
    const float motor_rpm = joint_rpm * DEBUG_MOTOR_GEAR;
    /* := rpm;=?*/
    const float target_rpm = motor_task_get_pid_target_rpm();
    const float feedback_rpm = joint_rpm;
    const float err_rpm = target_rpm - feedback_rpm;
    const float pos_fb = motor_task_get_now_pos(0u);
    const float pos_target = motor_task_get_target_rad();
    const float pos_err = pos_target - pos_fb;

    s_frame[0] = (uint8_t)DEBUG_SOF;
    s_frame[1] = (uint8_t)DEBUG_VERSION_PID;
    Put_U16(&s_frame[2], len);
    Put_U16(&s_frame[4], (uint16_t)DEBUG_ID_MOTOR_PID);
    Put_U16(&s_frame[6], s_seq);

    pl[0] = 0u;
    pl[1] = flags;
    Put_F32(&pl[2],  joint_rpm);
    Put_F32(&pl[6],  motor_rpm);
    Put_F32(&pl[10], target_rpm);
    Put_F32(&pl[14], feedback_rpm);
    Put_F32(&pl[18], err_rpm);
    Put_F32(&pl[22], motor_task_get_pid_out(0u));
    Put_F32(&pl[26], pos_fb);
    Put_F32(&pl[30], pos_target);
    Put_F32(&pl[34], pos_err);
#if CHASSIS_PHOTO_ENABLE
    pl[38] = Electric_Input_Get_Raw(ELECTRIC_IN_PHOTO_1);   /* PE2  */
    pl[39] = Electric_Input_Get_Raw(ELECTRIC_IN_PHOTO_2);   /* PE3  */
#else
    pl[38] = 0u;
    pl[39] = 0u;
#endif

    /* ?20B):/??CFSR/PC/ */
    pl[40] = (uint8_t)Error_Get_Level();
    pl[41] = 0u;
    Put_U16(&pl[42], Error_Get_Code());
    Put_U32(&pl[44], g_fault_code);
    Put_U32(&pl[48], g_fault_cfsr);
    Put_U32(&pl[52], g_fault_pc_psp);
    Put_U32(&pl[56], Error_Get_Flags());

    /* IMU:?rad)+ (rad/s),??*/
    {
        const Imu_Data_t *imu = Imu_Get(IMU_CHANNEL_CHASSIS);
        Put_F32(&pl[DEBUG_PID_IMU_OFFSET + 0u],  imu->Euler[0]);   /* roll  */
        Put_F32(&pl[DEBUG_PID_IMU_OFFSET + 4u],  imu->Euler[1]);   /* pitch */
        Put_F32(&pl[DEBUG_PID_IMU_OFFSET + 8u],  imu->Euler[2]);   /* yaw   */
        Put_F32(&pl[DEBUG_PID_IMU_OFFSET + 12u], imu->Gyro[0]);    /* gx    */
        Put_F32(&pl[DEBUG_PID_IMU_OFFSET + 16u], imu->Gyro[1]);    /* gy    */
        Put_F32(&pl[DEBUG_PID_IMU_OFFSET + 20u], imu->Gyro[2]);    /* gz    */
    }

    /* ?////5 ??32B) */
    {
        const Remote_Data_t *rc = Remote_Get();
        uint8_t i;

        pl[DEBUG_PID_RC_OFFSET + 0u] = Remote_Is_Online() ? 1u : 0u;
        pl[DEBUG_PID_RC_OFFSET + 1u] = rc->Switch[0];
        pl[DEBUG_PID_RC_OFFSET + 2u] = rc->Switch[1];
        pl[DEBUG_PID_RC_OFFSET + 3u] = rc->Mouse_Press_L;
        pl[DEBUG_PID_RC_OFFSET + 4u] = rc->Mouse_Press_R;
        pl[DEBUG_PID_RC_OFFSET + 5u] = 0u;
        Put_U16(&pl[DEBUG_PID_RC_OFFSET + 6u], rc->Key);
        for (i = 0u; i < RC_CHANNEL_NUM; i++)
        {
            Put_U16(&pl[DEBUG_PID_RC_OFFSET + 8u + (i * 2u)], (uint16_t)rc->Ch[i]);
        }
        Put_U16(&pl[DEBUG_PID_RC_OFFSET + 18u], (uint16_t)rc->Mouse_X);
        Put_U16(&pl[DEBUG_PID_RC_OFFSET + 20u], (uint16_t)rc->Mouse_Y);
        Put_U16(&pl[DEBUG_PID_RC_OFFSET + 22u], (uint16_t)rc->Mouse_Z);
        Put_U16(&pl[DEBUG_PID_RC_OFFSET + 24u], (uint16_t)rc->Lost_Count);
        Put_U16(&pl[DEBUG_PID_RC_OFFSET + 26u], 0u);
        Put_U32(&pl[DEBUG_PID_RC_OFFSET + 28u], rc->Rx_Count);
    }

    /* 0x207/0x208 ?flags + pos/w/t */
    {
        pl[DEBUG_PID_FRICTION_OFFSET + 0u] = motor_task_friction_flags(0u);
        pl[DEBUG_PID_FRICTION_OFFSET + 1u] = motor_task_friction_flags(1u);
        pl[DEBUG_PID_FRICTION_OFFSET + 2u] = 0u;
        pl[DEBUG_PID_FRICTION_OFFSET + 3u] = 0u;
        Put_F32(&pl[DEBUG_PID_FRICTION_OFFSET + 4u],  motor_task_friction_pos(0u));
        Put_F32(&pl[DEBUG_PID_FRICTION_OFFSET + 8u],  motor_task_friction_w(0u));
        Put_F32(&pl[DEBUG_PID_FRICTION_OFFSET + 12u], motor_task_friction_t(0u));
        Put_F32(&pl[DEBUG_PID_FRICTION_OFFSET + 16u], motor_task_friction_pos(1u));
        Put_F32(&pl[DEBUG_PID_FRICTION_OFFSET + 20u], motor_task_friction_w(1u));
        Put_F32(&pl[DEBUG_PID_FRICTION_OFFSET + 24u], motor_task_friction_t(1u));
    }

    /* 0x201 drive motor: fixed block so CSV can record m1 */
    {
        uint8_t flags = 0u;
        if (motor_task_is_init_ok(1u)) flags |= 0x01u;
        if (motor_task_is_online(1u))  flags |= 0x02u;
        pl[DEBUG_PID_M201_OFFSET + 0u] = flags;
        pl[DEBUG_PID_M201_OFFSET + 1u] = 0x01u;
        pl[DEBUG_PID_M201_OFFSET + 2u] = 0u;
        pl[DEBUG_PID_M201_OFFSET + 3u] = 0u;
        Put_F32(&pl[DEBUG_PID_M201_OFFSET + 4u],  motor_task_get_now_pos(1u));
        Put_F32(&pl[DEBUG_PID_M201_OFFSET + 8u],  motor_task_get_now_w(1u));
        Put_F32(&pl[DEBUG_PID_M201_OFFSET + 12u], motor_task_get_now_torque(1u));
    }

    /* 0x203 / 0x204 fixed blocks for CSV m2/m3 */
    {
        uint8_t flags = 0u;
        /* m2 = 0x203 drive */
        if (motor_task_is_init_ok(3u)) flags |= 0x01u;
        if (motor_task_is_online(3u))  flags |= 0x02u;
        pl[DEBUG_PID_M203_OFFSET + 0u] = flags;
        pl[DEBUG_PID_M203_OFFSET + 1u] = 0x03u;
        Put_F32(&pl[DEBUG_PID_M203_OFFSET + 4u],  motor_task_get_now_pos(3u));
        Put_F32(&pl[DEBUG_PID_M203_OFFSET + 8u],  motor_task_get_now_w(3u));
        Put_F32(&pl[DEBUG_PID_M203_OFFSET + 12u], motor_task_get_now_torque(3u));

        /* m3 = 0x204 steer */
        flags = 0u;
        if (motor_task_is_init_ok(2u)) flags |= 0x01u;
        if (motor_task_is_online(2u))  flags |= 0x02u;
        pl[DEBUG_PID_M204_OFFSET + 0u] = flags;
        pl[DEBUG_PID_M204_OFFSET + 1u] = 0x02u;
        Put_F32(&pl[DEBUG_PID_M204_OFFSET + 4u],  motor_task_get_now_pos(2u));
        Put_F32(&pl[DEBUG_PID_M204_OFFSET + 8u],  motor_task_get_now_w(2u));
        Put_F32(&pl[DEBUG_PID_M204_OFFSET + 12u], motor_task_get_now_torque(2u));
        Put_F32(&pl[DEBUG_PID_M204_OFFSET + 16u], motor_task_get_pid_target_rpm2());
        Put_F32(&pl[DEBUG_PID_M204_OFFSET + 20u], motor_task_get_target_rad2());
        Put_F32(&pl[DEBUG_PID_M204_OFFSET + 24u],
               motor_task_get_target_rad2() - motor_task_get_now_pos(2u));
    }

    /* 0x205 6020 yaw 固定块(204 起):位置环超调就看 pos/target/w_cmd 这三条 */
    {
        Put_F32(&pl[DEBUG_PID_6020_OFFSET + 0u],  motor_task_aux_get_pos(1u));
        Put_F32(&pl[DEBUG_PID_6020_OFFSET + 4u],  motor_task_aux_get_w(1u));
        Put_F32(&pl[DEBUG_PID_6020_OFFSET + 8u],  motor_task_aux_get_target_rad(1u));
        Put_F32(&pl[DEBUG_PID_6020_OFFSET + 12u], motor_task_aux_get_w_cmd(1u));
        Put_F32(&pl[DEBUG_PID_6020_OFFSET + 16u], motor_task_aux_get_torque(1u));
        pl[DEBUG_PID_6020_OFFSET + 20u] = motor_task_aux_online(1u);
        pl[DEBUG_PID_6020_OFFSET + 21u] = 0u;
        pl[DEBUG_PID_6020_OFFSET + 22u] = 0u;
        pl[DEBUG_PID_6020_OFFSET + 23u] = 0u;
    }

    constexpr uint8_t steer_indices[] = {0u, 2u};  /* 202, 204 */
    uint16_t age_offset = DEBUG_PID_STEER_AGE_OFFSET;
    for (uint8_t index : steer_indices)
    {
        Motor_Steer_Feedback_t feedback;
        (void)motor_task_get_steer_feedback(index, &feedback);
        const uint32_t age = feedback.age_us;
        const uint16_t age_wire = (age == UINT32_MAX) ? UINT16_MAX :
                                  (age < UINT16_MAX ? (uint16_t)age : UINT16_MAX - 1u);
        Put_U16(&pl[age_offset], age_wire);
        age_offset += 2u;
    }

    /* 232 起:202/204 找零状态(state u8×2 + 当前扫描角 f32×2) */
    {
        pl[DEBUG_PID_HOMING_OFFSET + 0u] = Chassis_Homing_State(0u);
        pl[DEBUG_PID_HOMING_OFFSET + 1u] = Chassis_Homing_State(1u);
        Put_F32(&pl[DEBUG_PID_HOMING_OFFSET + 2u], Chassis_Homing_Scan(0u));
        Put_F32(&pl[DEBUG_PID_HOMING_OFFSET + 6u], Chassis_Homing_Scan(1u));
    }

    {
        const uint16_t crc = crc_ccitt(s_frame, (size_t)(len - 2u));
        Put_U16(&s_frame[len - 2u], crc);
    }

    {
        bool sent = false;
#if DEBUG_USE_UART7
        /* 同步推一份到 485 转发缓冲:上板原样转发到串口,PC 才能看到 6020 块 */

        Cmd_485_Relay_Push(s_frame, len);

        sent = Debug_Send_One(&huart7, s_tx7, len) || sent;
#endif
#if DEBUG_USE_USART10
        sent = Debug_Send_One(&huart10, s_tx10, len) || sent;
#endif
        if (sent)
        {
            s_tx_frames++;
            s_seq++;
        }
    }
}
#endif

static void Debug_Send_Json(void)
{
    uint8_t *p = s_json_buf;
    uint16_t n = 0u;

    n += Text_Put_Str(&p[n], "[LOG] JSON ");
    p[n++] = (uint8_t)'{';

    n += Json_U32(&p[n], "m202.online", motor_task_is_online(0u) ? 1u : 0u);
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m202.pos", motor_task_get_now_pos(0u));
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m202.w", motor_task_get_now_w(0u));
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m202.t", motor_task_get_now_torque(0u));
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m202.target", motor_task_get_pid_target_rpm());
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m202.pos_target", motor_task_get_target_rad());
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m202.pos_error", motor_task_get_target_rad() - motor_task_get_now_pos(0u));
    p[n++] = (uint8_t)',';

    n += Json_U32(&p[n], "m201.online", motor_task_is_online(1u) ? 1u : 0u);
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m201.pos", motor_task_get_now_pos(1u));
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m201.w", motor_task_get_now_w(1u));
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m201.t", motor_task_get_now_torque(1u));
    p[n++] = (uint8_t)',';

    n += Json_U32(&p[n], "m204.online", motor_task_is_online(2u) ? 1u : 0u);
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m204.pos", motor_task_get_now_pos(2u));
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m204.w", motor_task_get_now_w(2u));
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m204.t", motor_task_get_now_torque(2u));
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m204.target", motor_task_get_pid_target_rpm2());
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m204.pos_target", motor_task_get_target_rad2());
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m204.pos_error", motor_task_get_target_rad2() - motor_task_get_now_pos(2u));
    p[n++] = (uint8_t)',';

    n += Json_U32(&p[n], "m206.online", motor_task_aux_online(0u));
    p[n++] = (uint8_t)','; 
    n += Json_F32(&p[n], "m206.w", motor_task_aux_get_w(0u));
    p[n++] = (uint8_t)','; 
    n += Json_F32(&p[n], "m206.target", motor_task_aux_get_target_rpm(0u));
    p[n++] = (uint8_t)','; 

    n += Json_U32(&p[n], "m205.online", motor_task_aux_online(1u));
    p[n++] = (uint8_t)','; 
    n += Json_F32(&p[n], "m205.w", motor_task_aux_get_w(1u));
    p[n++] = (uint8_t)','; 
    n += Json_F32(&p[n], "m205.target", motor_task_aux_get_target_rpm(1u));
    p[n++] = (uint8_t)','; 

    n += Json_U32(&p[n], "m6020.online", motor_task_aux_online(1u));
    p[n++] = (uint8_t)','; 
    n += Json_F32(&p[n], "m6020.w", motor_task_aux_get_w(1u));
    p[n++] = (uint8_t)','; 
    n += Json_F32(&p[n], "m6020.target", motor_task_aux_get_target_rpm(1u));
    p[n++] = (uint8_t)','; 
    n += Json_U32(&p[n], "m6020.enc", motor_task_aux_get_encoder(1u));
    p[n++] = (uint8_t)','; 
    n += Json_F32(&p[n], "m6020.pos", motor_task_aux_get_pos(1u));
    p[n++] = (uint8_t)','; 
    n += Json_F32(&p[n], "m6020.pos_target", motor_task_aux_get_target_rad(1u));
    p[n++] = (uint8_t)','; 

    n += Json_U32(&p[n], "m203.online", motor_task_is_online(3u) ? 1u : 0u);
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m203.pos", motor_task_get_now_pos(3u));
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m203.w", motor_task_get_now_w(3u));
    p[n++] = (uint8_t)',';
    n += Json_F32(&p[n], "m203.t", motor_task_get_now_torque(3u));
    p[n++] = (uint8_t)',';

    {
        const Imu_Data_t *imu = Imu_Get(IMU_CHANNEL_CHASSIS);
        n += Json_F32(&p[n], "imu.roll", imu->Euler[0]);
        p[n++] = (uint8_t)',';
        n += Json_F32(&p[n], "imu.pitch", imu->Euler[1]);
        p[n++] = (uint8_t)',';
        n += Json_F32(&p[n], "imu.yaw", imu->Euler[2]);
        p[n++] = (uint8_t)',';
        n += Json_F32(&p[n], "imu.gz", imu->Gyro[2]);
        p[n++] = (uint8_t)',';
    }

    {
        /* 舵向实时角度:now=实测(电机反馈经找零映射),tgt=闭环目标 */
        n += Json_F32(&p[n], "steer0.now", Chassis_Get_Seat_Now(0u));
        p[n++] = (uint8_t)',';
        n += Json_F32(&p[n], "steer0.tgt", Chassis_Get_Seat_Target(0u));
        p[n++] = (uint8_t)',';
        n += Json_F32(&p[n], "steer1.now", Chassis_Get_Seat_Now(1u));
        p[n++] = (uint8_t)',';
        n += Json_F32(&p[n], "steer1.tgt", Chassis_Get_Seat_Target(1u));
        p[n++] = (uint8_t)',';
    }

    {
        const Remote_Data_t *rc = Remote_Get();
        n += Json_U32(&p[n], "rc.online", Remote_Is_Online() ? 1u : 0u);
        p[n++] = (uint8_t)',';
        n += Json_U32(&p[n], "rc.ch0", (uint32_t)(uint16_t)rc->Ch[0]);
        p[n++] = (uint8_t)',';
        n += Json_U32(&p[n], "rc.ch1", (uint32_t)(uint16_t)rc->Ch[1]);
        p[n++] = (uint8_t)',';
        n += Json_U32(&p[n], "rc.ch2", (uint32_t)(uint16_t)rc->Ch[2]);
        p[n++] = (uint8_t)',';
        n += Json_U32(&p[n], "rc.s1", (uint32_t)rc->Switch[0]);
        p[n++] = (uint8_t)',';
        n += Json_U32(&p[n], "rc.s2", (uint32_t)rc->Switch[1]);
        p[n++] = (uint8_t)',';
    }

    n += Json_U32(&p[n], "link.rx", Cmd_485_Get_Rx_Count());
    p[n++] = (uint8_t)','; 
    n += Json_U32(&p[n], "link.ch", Cmd_485_Get_Last_Channel());
    p[n++] = (uint8_t)','; 

    n += Json_U32(&p[n], "err.level", (uint32_t)Error_Get_Level());
    p[n++] = (uint8_t)',';
    n += Json_U32(&p[n], "err.code", (uint32_t)Error_Get_Code());
    p[n++] = (uint8_t)',';
    n += Json_U32(&p[n], "debug.rx_bytes", g_debug_rx_bytes);
    p[n++] = (uint8_t)',';
    n += Json_U32(&p[n], "debug.cmd_count", g_debug_cmd_count);
    p[n++] = (uint8_t)',';
    {
        static const char *const can_names[11] = {
            "can.id201", "can.id202", "can.id203", "can.id204", "can.id205",
            "can.id206", "can.id207", "can.id208", "can.id209", "can.id20A",
            "can.id20B"
        };
        for (uint8_t i = 0u; i < 11u; i++)
        {
            n += Json_U32(&p[n], can_names[i],
                          (uint32_t)motor_task_can_online((uint16_t)(0x201u + i)));
            p[n++] = (uint8_t)',';
        }
    }
#if CHASSIS_PHOTO_ENABLE
    n += Json_U32(&p[n], "photo.pe2", (uint32_t)Electric_Input_Get_Raw(ELECTRIC_IN_PHOTO_1));
    p[n++] = (uint8_t)',';
    n += Json_U32(&p[n], "photo.pe3", (uint32_t)Electric_Input_Get_Raw(ELECTRIC_IN_PHOTO_2));
#endif
    p[n++] = (uint8_t)',';
    {
        static uint32_t s_json_test = 0u;
        n += Json_U32(&p[n], "test.counter", s_json_test++);
    }

    p[n++] = (uint8_t)'}';
    p[n++] = (uint8_t)'\r';
    p[n++] = (uint8_t)'\n';
    s_json_len = n;

#if !ROBOT_UPPER_ONLY
    /* 调试数据透传:每 2 帧 JSON(10Hz)往 485 透传缓冲塞一份,
       由 cmd_485 按 24B 分片发给上板,上板加 [L] 前缀从它的串口转出 */
    {
        static uint8_t s_relay_div = 0u;

        if (++s_relay_div >= 2u)
        {
            s_relay_div = 0u;
            {
                static uint32_t s_json_relay_ms = 0u;   /* JSON 只 5Hz 推转发缓冲,省带宽 */
                if (0) /* JSON 不再占转发带宽 */
                {
                    s_json_relay_ms = (uint32_t)Timestamp_Get_Ms();
                    Cmd_485_Relay_Push(s_json_buf, n);
                }
            }
        }
    }
#endif

    (void)Debug_Send_Raw(&huart7, s_json_tx7, s_json_buf, n);
#if DEBUG_USE_USART10
    (void)Debug_Send_Raw(&huart10, s_json_tx10, s_json_buf, n);
#endif
}

void Debug_Send_Homing(uint8_t __idx, float __pos, uint8_t __pe2, uint8_t __pe3, uint8_t __state)
{
    char buf[96];
    int n = snprintf(buf, sizeof(buf),
                     "[LOG] HOME %u pos=%.3f pe2=%u pe3=%u state=%u\r\n",
                     (unsigned int)__idx, (double)__pos,
                     (unsigned int)__pe2, (unsigned int)__pe3,
                     (unsigned int)__state);
    if (n <= 0) return;
    if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;

    (void)HAL_UART_Transmit(&huart7, (uint8_t *)buf, (uint16_t)n, 10u);
#if DEBUG_USE_USART10
    (void)HAL_UART_Transmit(&huart10, (uint8_t *)buf, (uint16_t)n, 10u);
#endif
}

void Debug_Send_Steer_Fast(void)
{
    static uint16_t s_steer_seq = 0u;
    static uint32_t s_last_ms = 0u;
    const uint32_t now_ms = (uint32_t)Timestamp_Get_Ms();

    if ((uint32_t)(now_ms - s_last_ms) < 20u) { return; }   /* 50Hz,给 0x0102/JSON 让带宽 */
    s_last_ms = now_ms;
    constexpr uint16_t payload_len = 16u + 18u + 4u + 8u;  /* 舵角、驱动状态、6020 角度、航向诊断 */
    uint8_t f[8u + payload_len + 2u];                   /* 帧头、载荷、CRC */
    uint16_t n = 0u;
    uint16_t crc;

    f[n++] = (uint8_t)DEBUG_SOF;          /* 0x5A */
    f[n++] = 0x01u;                       /* version */
    Put_U16(&f[n], payload_len); n = (uint16_t)(n + 2u);
    Put_U16(&f[n], (uint16_t)DEBUG_ID_STEER); n = (uint16_t)(n + 2u);
    Put_U16(&f[n], s_steer_seq++); n = (uint16_t)(n + 2u);

    Put_F32(&f[n], Chassis_Get_Seat_Now(0u));    n = (uint16_t)(n + 4u);
    Put_F32(&f[n], Chassis_Get_Seat_Target(0u)); n = (uint16_t)(n + 4u);
    Put_F32(&f[n], Chassis_Get_Seat_Now(1u));    n = (uint16_t)(n + 4u);
    Put_F32(&f[n], Chassis_Get_Seat_Target(1u)); n = (uint16_t)(n + 4u);

    /* 驱动状态(201/203):flags + 目标 rpm + 实际 rpm */
    {
        const float t201 = motor_task_get_drive_target_rpm(0u);
        const float a201 = motor_task_get_now_w(1u) * 9.5492966f;
        const float t203 = motor_task_get_drive_target_rpm(2u);
        const float a203 = motor_task_get_now_w(3u) * 9.5492966f;

        f[n++] = (uint8_t)(motor_task_is_init_ok(1u) ? 1u : 0u);
        Put_F32(&f[n], t201); n = (uint16_t)(n + 4u);
        Put_F32(&f[n], a201); n = (uint16_t)(n + 4u);
        f[n++] = (uint8_t)(motor_task_is_init_ok(3u) ? 1u : 0u);
        Put_F32(&f[n], t203); n = (uint16_t)(n + 4u);
        Put_F32(&f[n], a203); n = (uint16_t)(n + 4u);
        Put_F32(&f[n], motor_task_aux_get_pos(1u)); n = (uint16_t)(n + 4u);  /* 6020 pos */
        /* 38/42:航向环诊断 —— 误差(>8° 会被重锁成 0)+ 本拍实际送进运动学的 ω */
        Put_F32(&f[n], Chassis_Get_Heading_Error()); n = (uint16_t)(n + 4u);
        Put_F32(&f[n], Chassis_Get_Yaw_Rate_Cmd());  n = (uint16_t)(n + 4u);
    }

    crc = (uint16_t)crc_ccitt(f, n);
    f[n++] = (uint8_t)(crc & 0xFFu);
    f[n++] = (uint8_t)(crc >> 8);

    Cmd_485_Relay_Push(f, n);             /* 485 -> 上板 -> 串口 */
}

void Debug_Task(void)
{
    uint8_t *pl;
    uint16_t len = DEBUG_FRAME_SIZE;
    uint16_t crc;
    uint8_t i;
    uint8_t n;

    if (!s_inited)
    {
        return;
    }

    /* ? 0x5C (ISR ?? */
#if !ROBOT_UPPER_ONLY
    Debug_RelayDown_Poll();     /* 上板经 485 转发下来的调参字节 */
#endif
    Debug_Process_Rx();

#if DEBUG_MOTOR_PID_ONLY
    /*  PID ?/ */
    if ((int32_t)(Timestamp_Get_Us32() - s_next_us) < 0)
    {
        return;
    }
    /* 20Hz self-describing JSON: send it instead of one fixed frame,
       otherwise the fixed frame's DMA is still busy and JSON is dropped. */
    if ((int32_t)(Timestamp_Get_Us32() - s_next_json_us) >= 0)
    {
        s_next_json_us = Timestamp_Get_Us32() + 50000u;
        Debug_Send_Json();
        s_next_us = Timestamp_Get_Us32() + (DEBUG_PERIOD_MS * 1000u);
        return;
    }

    s_next_us = Timestamp_Get_Us32() + (DEBUG_PERIOD_MS * 1000u);
    Debug_Send_MotorPid();
    return;
#endif

    /* ??*/
    if ((int32_t)(Timestamp_Get_Us32() - s_next_us) < 0)
    {
        return;
    }
    /* ?(??,?" ?*/
    if ((uint32_t)(Timestamp_Get_Us32() - s_next_us) > (DEBUG_PERIOD_MS * 2000u))
    {
        s_next_us = Timestamp_Get_Us32();
    }
    s_next_us += (DEBUG_PERIOD_MS * 1000u);


#if DEBUG_TEXT_ENABLE
    /* ?1 Hz (?[LOG] ??,??*/
    {
        uint32_t now_ms = (uint32_t)Timestamp_Get_Ms();

        if ((uint32_t)(now_ms - s_next_txt_ms) >= DEBUG_TEXT_PERIOD_MS)
        {
            int n;
            uint32_t boot_start = Log_Get_Boot_Start();
            uint32_t prev_start = Log_Get_Prev_Start();
            uint32_t prev_stop = Log_Get_Prev_Offset();

            s_next_txt_ms = now_ms;

            /* :[LOG] boot=?cur=0x?prev=0x?stop=0x?wrap=?*/
            {
                uint8_t *p = (uint8_t *)s_txt;
                uint16_t n = 0u;

                n += Text_Put_Str(&p[n], "[LOG] boot=");
                n += Text_Put_U32(&p[n], Log_Get_Boot_Id());
                n += Text_Put_Str(&p[n], " cur=");
                n += Text_Put_Hex6(&p[n], (FLASH_LOG_ADDR + 4096u + Log_Get_Boot_Start()) & 0xFFFFFFu);
                n += Text_Put_Str(&p[n], " prev=");
                n += Text_Put_Hex6(&p[n], (prev_start == 0xFFFFFFFFu) ? 0xFFFFFFu
                                                               : (FLASH_LOG_ADDR + 4096u + prev_start) & 0xFFFFFFu);
                n += Text_Put_Str(&p[n], " stop=");
                n += Text_Put_Hex6(&p[n], (prev_stop == 0xFFFFFFFFu) ? 0xFFFFFFu
                                                              : (FLASH_LOG_ADDR + 4096u + prev_stop) & 0xFFFFFFu);
                n += Text_Put_Str(&p[n], " wrap=");
                n += Text_Put_U32(&p[n], Log_Get_Wrap_Count());
                p[n++] = (uint8_t)'\r';
                p[n++] = (uint8_t)'\n';

                (void)Debug_Send_Raw(&huart7, s_txt7, s_txt, n);
#if DEBUG_USE_USART10
                (void)Debug_Send_Raw(&huart10, s_txt10, s_txt, n);
#endif
            }
        }
    }
#endif

    /* ??*/
    s_frame[0] = (uint8_t)DEBUG_SOF;
    s_frame[1] = (uint8_t)DEBUG_VERSION;
    Put_U16(&s_frame[2], len);
    Put_U16(&s_frame[4], (uint16_t)DEBUG_ID_TELEMETRY);
    Put_U16(&s_frame[6], s_seq);

    /* ?payload */
    pl = &s_frame[8];

    pl[DEBUG_PL_ERR_LEVEL] = (uint8_t)Error_Get_Level();
    n = motor_task_get_count();
    if (n > DEBUG_MAX_MOTORS)
    {
        n = (uint8_t)DEBUG_MAX_MOTORS;
    }
    pl[DEBUG_PL_MOTOR_COUNT] = n;
    Put_U16(&pl[DEBUG_PL_ERR_CODE], Error_Get_Code());
    Put_U32(&pl[DEBUG_PL_FAULT_CODE], g_fault_code);
    Put_U32(&pl[DEBUG_PL_FAULT_CFSR], g_fault_cfsr);
    Put_U32(&pl[DEBUG_PL_FAULT_PC], g_fault_pc_psp);

    {
        const Imu_Data_t *imu = Imu_Get(IMU_CHANNEL_CHASSIS);
        Put_F32(&pl[DEBUG_PL_IMU + 0u], imu->Euler[0]);      /* roll  */
        Put_F32(&pl[DEBUG_PL_IMU + 4u], imu->Euler[1]);      /* pitch */
        Put_F32(&pl[DEBUG_PL_IMU + 8u], imu->Yaw_Total);     /*  yaw */

        /* ??12B):?"? ?
             176: flash JEDEC ID      180: flags()      181: ACC_CONF 
             182: BMI088       184:  */
        Put_U32(&pl[DEBUG_PL_DIAG + 0u], flash_jedec_id);
        {
            uint8_t flags = 0u;
            if (flash_init_ok)   { flags |= 0x01u; }
            if (bmi088_init_ok)  { flags |= 0x02u; }
            if (BMI088_Is_Online()) { flags |= 0x04u; }
            if (ws2812_init_ok)  { flags |= 0x08u; }
            if (buzzer_init_ok)  { flags |= 0x10u; }
            if (Log_Is_Ready())  { flags |= 0x20u; }
            if (Imu_Ext_Is_Online()) { flags |= 0x40u; }   /* bit6: external IMU online */
            pl[DEBUG_PL_DIAG + 4u] = flags;
        }
        pl[DEBUG_PL_DIAG + 5u] = bmi088_acc_conf;
        Put_U16(&pl[DEBUG_PL_DIAG + 6u], (uint16_t)(BMI088_Get_Error_Count() & 0xFFFFu));
        Put_U32(&pl[DEBUG_PL_DIAG + 8u], g_error_tick);   /* ?error (=? */
        Put_F32(&pl[DEBUG_PL_ACCEL + 0u], imu->Accel[0]);
        Put_F32(&pl[DEBUG_PL_ACCEL + 4u], imu->Accel[1]);
        Put_F32(&pl[DEBUG_PL_ACCEL + 8u], imu->Accel[2]);

        /* ?(:?4B,?<=1 ??
           ?motor/robot/error/cmd/imu/chassis/gimbal/highRise  ?*/
        {
            const uint32_t hwm[DEBUG_HWM_NUM] = {
                hwm_motor, hwm_robot, hwm_error, hwm_cmd,
                hwm_imu, hwm_chassis, hwm_gimbal, hwm_highRise
            };
            uint8_t k;

            for (k = 0u; k < DEBUG_HWM_NUM; k++)
            {
                uint16_t v = (hwm[k] > 0xFFFFu) ? 0xFFFFu : (uint16_t)hwm[k];
                Put_U16(&pl[DEBUG_PL_HWM + (k * 2u)], v);
            }
        }

        /* ?? Flash ;0xFFFFFFFF = ) */
        {
            uint32_t prev_stop = Log_Get_Prev_Offset();
            uint32_t prev_start = Log_Get_Prev_Start();

            Put_U32(&pl[DEBUG_PL_LOG + 0u],  Log_Get_Boot_Id());
            Put_U32(&pl[DEBUG_PL_LOG + 4u],  FLASH_LOG_ADDR + 4096u + Log_Get_Boot_Start());
            Put_U32(&pl[DEBUG_PL_LOG + 8u],  (prev_start == 0xFFFFFFFFu) ? 0xFFFFFFFFu
                                                                        : (FLASH_LOG_ADDR + 4096u + prev_start));
            Put_U32(&pl[DEBUG_PL_LOG + 12u], (prev_stop == 0xFFFFFFFFu) ? 0xFFFFFFFFu
                                                                       : (FLASH_LOG_ADDR + 4096u + prev_stop));
            Put_U32(&pl[DEBUG_PL_LOG + 16u], Log_Get_Wrap_Count());
        }

        /* BMI088 ( die ?) */
        Put_F32(&pl[DEBUG_PL_TEMP], imu->Temp_C);

        /* ?8 : ?*/
        {
            uint8_t k;
            for (k = 0u; k < DEBUG_TICK_NUM; k++)
            {
                Put_U32(&pl[DEBUG_PL_TICKS + (k * 4u)], g_task_tick[k]);
            }
        }

        /* ?Flash (?:State / ErrorCode / Command / Receive /  ID */
        Put_U32(&pl[DEBUG_PL_FLASH + 0u],  flash_diag_state);
        Put_U32(&pl[DEBUG_PL_FLASH + 4u],  flash_diag_err);
        Put_U32(&pl[DEBUG_PL_FLASH + 8u],  flash_diag_cmd);
        Put_U32(&pl[DEBUG_PL_FLASH + 12u], flash_diag_rcv);
        Put_U32(&pl[DEBUG_PL_FLASH + 16u], flash_diag_raw);
    }
    {
        const Chassis_Data_t *ch = Chassis_Get();
        Put_F32(&pl[DEBUG_PL_CHASSIS + 0u], ch->Vx);
        Put_F32(&pl[DEBUG_PL_CHASSIS + 4u], ch->Vy);
        Put_F32(&pl[DEBUG_PL_CHASSIS + 8u], ch->W);
    }
    {
        const Gimbal_Data_t *gi = Gimbal_Get();
        Put_F32(&pl[DEBUG_PL_GIMBAL + 0u], gi->Now[GIMBAL_AXIS_YAW]);
        Put_F32(&pl[DEBUG_PL_GIMBAL + 4u], gi->Now[GIMBAL_AXIS_PITCH]);
    }

    /*  16B: /  /  / ?/ (? */
    for (i = 0u; i < n; i++)
    {
        uint8_t *m = &pl[DEBUG_PL_MOTORS + (i * DEBUG_MOTOR_SIZE)];
        uint8_t flags = 0u;

        if (motor_task_is_init_ok(i))
        {
            flags |= 0x01u;
        }
        if (motor_task_is_online(i))
        {
            flags |= 0x02u;
        }
        if (motor_task_get_m_error(i) != 0u)
        {
            flags |= 0x04u;
        }

        m[DEBUG_M_ID] = i;
        m[DEBUG_M_FLAGS] = flags;
        Put_F32(&m[DEBUG_M_POS], motor_task_get_now_pos(i));
        Put_F32(&m[DEBUG_M_W], motor_task_get_now_w(i));       /* ?rad/s(motor_task ) */
        Put_F32(&m[DEBUG_M_T], motor_task_get_now_torque(i));
    }

    /* ?CRC16( [0..len-3]) + ?*/
    crc = crc_ccitt(s_frame, (size_t)(len - 2u));
    Put_U16(&s_frame[len - 2u], crc);

    /* ???) */
    {
        bool sent = false;

#if DEBUG_USE_UART7
        sent = Debug_Send_One(&huart7, s_tx7, len) || sent;
#endif
#if DEBUG_USE_USART10
        sent = Debug_Send_One(&huart10, s_tx10, len) || sent;
#endif
        if (sent)
        {
            s_tx_frames++;
            s_seq++;
        }
    }
}

uint32_t Debug_Get_Tx_Frames(void)
{
    return s_tx_frames;
}

uint32_t Debug_Get_Tx_Skip(void)
{
    return s_tx_skip;
}

uint16_t Debug_Get_Frame_Size(void)
{
#if DEBUG_MOTOR_PID_ONLY
    return (uint16_t)DEBUG_MOTOR_PID_FRAME;
#else
    return (uint16_t)DEBUG_FRAME_SIZE;
#endif
}





