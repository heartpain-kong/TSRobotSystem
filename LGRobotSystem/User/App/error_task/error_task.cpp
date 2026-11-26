//
// error_task.cpp — 错误处理(蜂鸣器 + RGB 逐位量化) + 关键数据记录(→ dev_log)
// 说明 / 用法 / 坑 见同目录 error_task.md

#include "error_task.h"

#include "dev_buzzer.h"
#include "dev_log.h"
#include "dev_ws2812.h"
#include "bsp_power.h"          /* 欠/过压 */
#include "bsp_timestamp.h"      /* 图案计时 */

#include "motor_task.h"          /* 逐台电机状态(20+/40+/60+ 的码就来自它) */

#include <stdio.h>
#include <string.h>

/* ============================================================================
 * 外部状态(main.c 的是 C 链接,robot.cpp 的是 C++ 链接 —— 别搞混)
 * ========================================================================== */

extern "C"
{
    extern volatile uint32_t g_fault_code;          /* 1 堆 2 栈 3 Error_Handler 4 HardFault */
    extern volatile const char *g_fault_task;
}

extern volatile bool bmi088_init_ok;
extern volatile bool buzzer_init_ok;
extern volatile bool ws2812_init_ok;

/* ★ 心跳:每跑一拍 +1;遥测里看它是否在涨 ⇒ 判断 error 任务是否还活着 */
volatile uint32_t g_error_tick = 0u;
extern volatile bool ws2812_clock_ok;
extern volatile bool adc1_init_ok;
extern volatile bool flash_init_ok;
extern volatile bool log_init_ok;
extern volatile bool electric_init_ok;
extern volatile bool power_voltage_ok;

/* ============================================================================
 * 内部状态
 * ========================================================================== */

#define ERROR_REPORT_SLOTS      16u     /* 自定义码最多同时记 16 个 */
#define ERROR_LED_NUM           8u      /* WS2812 灯数(必须与实际一致) */

/* 正常状态的"绿色心跳"参数(本任务 100Hz ⇒ 时间分辨率 10ms) */
#define ERROR_OK_PERIOD_MS      2000u   /* breath period = 2 s */
#define ERROR_OK_G_MIN             0u   /* breath starts from fully OFF */
#define ERROR_OK_G_MAX             6u   /* peak 6/255 (2.4%), single LED, 2 s breath */
#define ERROR_OK_G_LIMIT          0x10u   /* hard cap: never exceed 0x10 even if someone puts 0xFF */

#define ERROR_CODE_BITS         7u      /* LED1~7 能表示 7 位(0~127) */

static Error_Snapshot_t s_snapshot;

/* 自定义错误位 + 直接报上来的码 */
static uint32_t s_flags = 0u;
static uint16_t s_report_codes[ERROR_REPORT_SLOTS];

/* 当前/上次的码与级别(变化时写一条文本日志) */
static uint16_t s_code = ERROR_CODE_NONE;
static uint16_t s_code_last = 0xFFFFu;      /* 故意与初值不同 ⇒ 首拍必写一条 */
static Enum_Error_Level s_level = ERROR_LEVEL_OK;

static uint32_t s_next_log_ms = 0u;
static uint32_t s_phase_ms = 0u;

/* LED 画像(变了才发 DMA) */
static uint8_t s_led[ERROR_LED_NUM][3];
static uint8_t s_led_last[ERROR_LED_NUM][3];
static bool s_led_sent = false;

/* ============================================================================
 * 对外:快照 / 标志位 / 查询
 * ========================================================================== */

Error_Snapshot_t *Error_Get_Snapshot(void)
{
    return &s_snapshot;
}

uint32_t Error_Get_Flags(void)
{
    return s_flags;
}

void Error_Set_Flag(uint32_t __mask, bool __set)
{
    if (__set)
    {
        s_flags |= __mask;
    }
    else
    {
        s_flags &= ~__mask;
    }
}

void Error_Clear_Flags(uint32_t __mask)
{
    s_flags &= ~__mask;
}

void Error_Report(uint16_t __code, bool __active)
{
    uint8_t i;
    uint8_t free_slot = ERROR_REPORT_SLOTS;

    if (__code == ERROR_CODE_NONE)
    {
        return;
    }

    /* 已经在表里? */
    for (i = 0u; i < ERROR_REPORT_SLOTS; i++)
    {
        if (s_report_codes[i] == __code)
        {
            if (!__active)
            {
                s_report_codes[i] = ERROR_CODE_NONE;
            }
            return;
        }
        if ((free_slot == ERROR_REPORT_SLOTS) && (s_report_codes[i] == ERROR_CODE_NONE))
        {
            free_slot = i;
        }
    }

    /* 新码:置位时找个空位放进去 */
    if (__active && (free_slot < ERROR_REPORT_SLOTS))
    {
        s_report_codes[free_slot] = __code;
    }
}

uint16_t Error_Get_Code(void)
{
    return s_code;
}

Enum_Error_Level Error_Get_Level(void)
{
    return s_level;
}

const char *Error_Get_Level_Name(Enum_Error_Level __level)
{
    switch (__level)
    {
    case ERROR_LEVEL_FATAL:
        return "FATAL";
    case ERROR_LEVEL_ERROR:
        return "ERROR";
    case ERROR_LEVEL_WARNING:
        return "WARN";
    default:
        return "OK";
    }
}

const char *Error_Get_Code_Name(uint16_t __code)
{
    static char buf[48];
    uint16_t n;

    /* 电机相关:带上电机名,直接告诉你"是哪一台" */
    if ((__code >= ERROR_CODE_MOTOR_INIT_BASE) &&
        (__code < (ERROR_CODE_MOTOR_INIT_BASE + ERROR_CODE_MOTOR_MAX)))
    {
        n = (uint16_t)(__code - ERROR_CODE_MOTOR_INIT_BASE);
        (void)snprintf(buf, sizeof(buf), "motor init fail: %s", motor_task_get_name((uint8_t)n));
        return buf;
    }
    if ((__code >= ERROR_CODE_MOTOR_OFFLINE_BASE) &&
        (__code < (ERROR_CODE_MOTOR_OFFLINE_BASE + ERROR_CODE_MOTOR_MAX)))
    {
        n = (uint16_t)(__code - ERROR_CODE_MOTOR_OFFLINE_BASE);
        (void)snprintf(buf, sizeof(buf), "motor offline: %s", motor_task_get_name((uint8_t)n));
        return buf;
    }
    if ((__code >= ERROR_CODE_MOTOR_FAULT_BASE) &&
        (__code < (ERROR_CODE_MOTOR_FAULT_BASE + ERROR_CODE_MOTOR_MAX)))
    {
        n = (uint16_t)(__code - ERROR_CODE_MOTOR_FAULT_BASE);
        (void)snprintf(buf, sizeof(buf), "motor fault(err=%u): %s",
                       (unsigned int)motor_task_get_m_error((uint8_t)n),
                       motor_task_get_name((uint8_t)n));
        return buf;
    }

    if ((__code >= ERROR_CODE_FLAG_BASE) && (__code < (ERROR_CODE_FLAG_BASE + 32u)))
    {
        (void)snprintf(buf, sizeof(buf), "flag bit%u", (unsigned int)(__code - ERROR_CODE_FLAG_BASE));
        return buf;
    }

    switch (__code)
    {
    case ERROR_CODE_RTOS_MALLOC:     return "RTOS: malloc failed";
    case ERROR_CODE_RTOS_STACK:      return "RTOS: task stack overflow";
    case ERROR_CODE_ERROR_HANDLER:   return "Error_Handler";
    case ERROR_CODE_HARD_FAULT:      return "HardFault";
    case ERROR_CODE_FLASH:           return "init: external flash";
    case ERROR_CODE_LOG:             return "init: log";
    case ERROR_CODE_IMU:             return "init: IMU(BMI088)";
    case ERROR_CODE_ADC:             return "init: ADC";
    case ERROR_CODE_WS2812:          return "init: WS2812";
    case ERROR_CODE_WS2812_CLOCK:    return "init: WS2812 clock(SPI6 prescaler)";
    case ERROR_CODE_BUZZER:          return "init: buzzer";
    case ERROR_CODE_ELECTRIC:        return "init: electric(relay)";
    case ERROR_CODE_VOLTAGE:         return "power: voltage out of range";
    case ERROR_CODE_LOG_FULL:        return "log: region full";
    case ERROR_CODE_LOG_DROPPED:     return "log: dropped data";
    default:                         return "?";
    }
}

bool Error_Is_Beeping(void)
{
    return Buzzer_Is_Sounding();
}

/* ============================================================================
 * 内部:算出当前码 + 级别
 *   优先级(取第一个命中的):RTOS → 设备 → 电机 → 自定义码 → 日志(WARNING)
 * ========================================================================== */

static uint16_t Evaluate_Code(Enum_Error_Level *__level)
{
    uint8_t i;
    uint8_t count = motor_task_get_count();

    /* ① RTOS 级致命(1~4 与 g_fault_code 一一对应) */
    if (g_fault_code != 0u)
    {
        *__level = ERROR_LEVEL_FATAL;
        switch (g_fault_code)
        {
        case 1u:  return ERROR_CODE_RTOS_MALLOC;
        case 2u:  return ERROR_CODE_RTOS_STACK;
        case 3u:  return ERROR_CODE_ERROR_HANDLER;
        case 4u:  return ERROR_CODE_HARD_FAULT;
        default: return ERROR_CODE_HARD_FAULT;
        }
    }

    /* ② 设备初始化失败(固定顺序,保证同一个故障每次报同一个码) */
    if (!flash_init_ok)          { *__level = ERROR_LEVEL_ERROR; return ERROR_CODE_FLASH; }
    if (!log_init_ok)            { *__level = ERROR_LEVEL_ERROR; return ERROR_CODE_LOG; }
    if (!bmi088_init_ok)         { *__level = ERROR_LEVEL_ERROR; return ERROR_CODE_IMU; }
    if (!adc1_init_ok)           { *__level = ERROR_LEVEL_ERROR; return ERROR_CODE_ADC; }
    if (!ws2812_init_ok)         { *__level = ERROR_LEVEL_ERROR; return ERROR_CODE_WS2812; }
    if (!ws2812_clock_ok)        { *__level = ERROR_LEVEL_ERROR; return ERROR_CODE_WS2812_CLOCK; }
    if (!buzzer_init_ok)         { *__level = ERROR_LEVEL_ERROR; return ERROR_CODE_BUZZER; }
    if (!electric_init_ok)       { *__level = ERROR_LEVEL_ERROR; return ERROR_CODE_ELECTRIC; }
#if ERROR_CHECK_VOLTAGE
    if (!power_voltage_ok)       { *__level = ERROR_LEVEL_ERROR; return ERROR_CODE_VOLTAGE; }
#endif

    /* ③ 电机:初始化失败 → 离线 → 报错(逐台扫,顺序固定) */
    for (i = 0u; i < count; i++)
    {
        if (!motor_task_is_init_ok(i))
        {
            *__level = ERROR_LEVEL_ERROR;
            return (uint16_t)(ERROR_CODE_MOTOR_INIT_BASE + i);
        }
    }
    for (i = 0u; i < count; i++)
    {
        if (!motor_task_is_online(i))
        {
            *__level = ERROR_LEVEL_ERROR;
            return (uint16_t)(ERROR_CODE_MOTOR_OFFLINE_BASE + i);
        }
    }
    for (i = 0u; i < count; i++)
    {
        if (motor_task_get_m_error(i) != 0u)
        {
            *__level = ERROR_LEVEL_ERROR;
            return (uint16_t)(ERROR_CODE_MOTOR_FAULT_BASE + i);
        }
    }

    /* ④ 自定义标志位(码 = 80 + 位号) */
    for (i = 0u; i < 32u; i++)
    {
        if ((s_flags & (1u << i)) != 0u)
        {
            *__level = ERROR_LEVEL_ERROR;
            return (uint16_t)(ERROR_CODE_FLAG_BASE + i);
        }
    }

    /* ⑤ 直接报上来的自定义码 */
    for (i = 0u; i < ERROR_REPORT_SLOTS; i++)
    {
        if (s_report_codes[i] != ERROR_CODE_NONE)
        {
            *__level = ERROR_LEVEL_ERROR;
            return s_report_codes[i];
        }
    }

    /* ⑥ 电源异常 / 日志问题(WARNING) */
#if ERROR_CHECK_VOLTAGE
    if (Power_Is_Under_Voltage() || Power_Is_Over_Voltage())
    {
        *__level = ERROR_LEVEL_WARNING;
        return ERROR_CODE_VOLTAGE;
    }
#endif
    /* ★ 不再报 LOG_FULL:环形日志写满后从头覆盖是正常行为,不是故障 ✓ */
    if (Log_Get_Dropped_Bytes() != 0u)
    {
        *__level = ERROR_LEVEL_WARNING;
        return ERROR_CODE_LOG_DROPPED;
    }

    /* ⑦ 正常 */
    *__level = ERROR_LEVEL_OK;
    return ERROR_CODE_NONE;
}

/* ============================================================================
 * 内部:类别色 / 音高
 * ========================================================================== */

static void Category_Color(Enum_Error_Level __level, uint8_t *__r, uint8_t *__g, uint8_t *__b)
{
    /* ★ 配色必须"饱和":原来 12~40/255 只有 5%~16% 亮度,
       而且橙(40,12) 与 黄(30,24) 在低亮度下几乎分不开 ✗;
       WS2812 在极低亮度下绿色通道视觉上最"跳",所以看起来"都偏绿" ✗
       ⇒ 现在给满色:红 / 橙(偏红) / 黄(偏黄绿) / 绿,三者差异明显 ✓ */
    switch (__level)
    {
    case ERROR_LEVEL_FATAL:     *__r = 255u; *__g =   0u; *__b = 0u; break;  /* 红 */
    case ERROR_LEVEL_ERROR:     *__r = 255u; *__g =  50u; *__b = 0u; break;  /* 橙(黄少 ⇒ 明显偏红) */
    case ERROR_LEVEL_WARNING:   *__r = 255u; *__g = 220u; *__b = 0u; break;  /* 黄(接近满黄) */
    default:                    *__r =   0u; *__g = 200u; *__b = 0u; break;  /* 绿 */
    }
}

static float Category_Freq(Enum_Error_Level __level)
{
    switch (__level)
    {
    case ERROR_LEVEL_FATAL:     return 3000.0f;
    case ERROR_LEVEL_ERROR:     return 2500.0f;
    case ERROR_LEVEL_WARNING:   return 1500.0f;
    default:                    return 0.0f;
    }
}

/**
 * @brief 类别色该亮还是该灭(闪烁相位)
 */
static bool Category_Visible(Enum_Error_Level __level)
{
    uint32_t t = s_phase_ms;

    switch (__level)
    {
    case ERROR_LEVEL_FATAL:     return ((t % 200u) < 100u);     /* 5Hz */
    case ERROR_LEVEL_ERROR:     return ((t % 400u) < 200u);     /* 2.5Hz */
    case ERROR_LEVEL_WARNING:   return ((t % 1000u) < 500u);    /* 1Hz */
    default:                    return true;                    /* OK 常显(呼吸由亮度做) */
    }
}

/* ============================================================================
 * 内部:RGB 显示(LED0 = 类别,LED1~7 = 码的二进制)
 * ========================================================================== */

static void Update_Led(Enum_Error_Level __level, uint16_t __code)
{
    uint8_t r, g, b;
    uint8_t i, bit;
    bool changed = false;

    Category_Color(__level, &r, &g, &b);

    if (__level == ERROR_LEVEL_OK)
    {
        /* 正常:绿色**呼吸**(暗 → 更暗 → 暗,不闪;峰值仅 14/255,不刺眼)
           —— 之前是"暗底 24 + 闪到 255",太刺眼 ✗ */
        uint32_t t_ok = s_phase_ms % ERROR_OK_PERIOD_MS;
        uint32_t half = ERROR_OK_PERIOD_MS / 2u;
        uint32_t k    = (t_ok < half) ? t_ok : (ERROR_OK_PERIOD_MS - t_ok);   /* 0..half 三角波 */
        uint32_t span = (uint32_t)(ERROR_OK_G_MAX - ERROR_OK_G_MIN);
        uint8_t  g_ok = (uint8_t)((uint32_t)ERROR_OK_G_MIN + (span * k) / half);
        if (g_ok > ERROR_OK_G_LIMIT) { g_ok = (uint8_t)ERROR_OK_G_LIMIT; }   /* hard cap 0x10 */

        /* ★ 只用 LED0 做暗呼吸(全余 7 颗灭),
           8 颗一起亮 ≈ 一颗 8 倍亮度 ✗ */
        for (i = 0u; i < ERROR_LED_NUM; i++)
        {
            s_led[i][0] = 0u;
            s_led[i][1] = (i == 0u) ? g_ok : 0u;
            s_led[i][2] = 0u;
        }
    }
    else
    {
        /* LED0 = 类别色(按类别闪) */
        bool on = Category_Visible(__level);
        s_led[0][0] = on ? r : 0u;
        s_led[0][1] = on ? g : 0u;
        s_led[0][2] = on ? b : 0u;

        /* LED1~7 = 码的 bit0~bit6(常亮,亮度减半便于与类别色区分) */
        for (bit = 0u; bit < ERROR_CODE_BITS; bit++)
        {
            uint8_t idx = (uint8_t)(1u + bit);
            bool set = ((__code & (1u << bit)) != 0u);
            s_led[idx][0] = set ? (uint8_t)(r / 2u) : 0u;
            s_led[idx][1] = set ? (uint8_t)(g / 2u) : 0u;
            s_led[idx][2] = set ? (uint8_t)(b / 2u) : 0u;
        }
    }

    /* 画像变了才发一次 DMA */
    if ((!s_led_sent) || (memcmp(s_led, s_led_last, sizeof(s_led)) != 0))
    {
        for (i = 0u; i < ERROR_LED_NUM; i++)
        {
            (void)WS2812_Set_RGB(i, s_led[i][0], s_led[i][1], s_led[i][2]);
        }
        (void)WS2812_Show();
        memcpy(s_led_last, s_led, sizeof(s_led));
        s_led_sent = true;
        changed = true;
    }
    (void)changed;
}

/* ============================================================================
 * 内部:蜂鸣器(N 声 = 码,音高 = 类别)
 * ========================================================================== */

static void Update_Buzzer(Enum_Error_Level __level, uint16_t __code)
{
    uint32_t n, step, total, pos;
    float freq = Category_Freq(__level);

    if ((freq <= 0.0f) || (__code == ERROR_CODE_NONE))
    {
        return;                     /* OK:静音 */
    }

    n = __code;
    if (n > ERROR_BEEP_MAX)
    {
        n = ERROR_BEEP_MAX;         /* 码太大只响 10 声,精确码看 LED/日志 */
    }

    step = ERROR_BEEP_ON_MS + ERROR_BEEP_GAP_MS;
    total = n * step + ERROR_BEEP_CYCLE_GAP_MS;
    pos = s_phase_ms % total;

    /* 第几声里、且在该声的"响"区间内 ⇒ 补一声非阻塞的响 */
    if ((pos < (n * step)) && ((pos % step) < ERROR_BEEP_ON_MS) && (!Buzzer_Is_Sounding()))
    {
        (void)Buzzer_Beep(freq, 0.6f, ERROR_BEEP_ON_MS);
    }
}

/* ============================================================================
 * 内部:关键数据落盘(10Hz)+ 码变化时写一条文本
 * ========================================================================== */

static void Update_Log(Enum_Error_Level __level, uint16_t __code, uint32_t __now_ms)
{
    Error_Frame_t frame;

    /* 级别或码变了 ⇒ 立刻写一条文本(带可读名,能看出是哪台电机) */
    if (__code != s_code_last)
    {
        Log_Printf((__level >= ERROR_LEVEL_ERROR) ? LOG_LEVEL_ERROR :
                   ((__level == ERROR_LEVEL_WARNING) ? LOG_LEVEL_WARNING : LOG_LEVEL_INFO),
                   "code %u(%s) level=%s fault=%u task=%s flags=%08X",
                   (unsigned int)__code,
                   Error_Get_Code_Name(__code),
                   Error_Get_Level_Name(__level),
                   (unsigned int)g_fault_code,
                   (const char *)g_fault_task,
                   (unsigned int)s_flags);
        s_code_last = __code;
    }

    /* 按 10Hz 记一帧关键数据 */
    if ((int32_t)(__now_ms - s_next_log_ms) < 0)
    {
        return;
    }
    s_next_log_ms = __now_ms + ERROR_LOG_PERIOD_MS;

    memset(&frame, 0, sizeof(frame));
    frame.Magic = ERROR_FRAME_MAGIC;
    frame.Type = ERROR_FRAME_TYPE_SNAPSHOT;
    frame.Level = (uint8_t)__level;
    frame.Code = __code;
    frame.Ms = __now_ms;
    frame.Flags = s_flags;
    frame.Snapshot = s_snapshot;

    Log_Record(&frame, (uint16_t)sizeof(frame));
}

/* ============================================================================
 * 跑一拍
 * ========================================================================== */

void Error_Task(void)
{
    g_error_tick++;    /* ★ 心跳(遥测里能看出它是否在跑) */
    uint32_t now_ms = (uint32_t)Timestamp_Get_Ms();

    s_phase_ms = now_ms;

    s_code = Evaluate_Code(&s_level);

    Update_Buzzer(s_level, s_code);
    Update_Led(s_level, s_code);
    Update_Log(s_level, s_code, now_ms);
}
