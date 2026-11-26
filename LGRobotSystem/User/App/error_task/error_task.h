/* ============================================================================
 * error_task.h — 错误处理(蜂鸣器 + RGB 逐位量化) + 关键数据记录
 * 说明 / 用法 / 坑 见同目录 error_task.md
 * ========================================================================== */

#ifndef ERROR_TASK_H
#define ERROR_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 周期 ---------------- */
#define ERROR_TASK_PERIOD_MS        10u     /* 指示刷新(蜂鸣器/RGB 图案) */
#define ERROR_LOG_PERIOD_MS         100u    /* 关键数据落盘:**10Hz** */

/* ---------------- 蜂鸣器图案参数 ---------------- */
#define ERROR_BEEP_ON_MS            80u     /* 一声响多久 */
#define ERROR_BEEP_GAP_MS           120u    /* 声与声之间静多久 */
#define ERROR_BEEP_CYCLE_GAP_MS     800u    /* 一轮结束的长静音(分隔用) */
#define ERROR_BEEP_MAX              10u     /* 最多响几声(码更大时按 10 声) */

/* ---------------- 错误级别(数值越大越严重) ---------------- */
typedef enum
{
    ERROR_LEVEL_OK = 0,
    ERROR_LEVEL_WARNING,
    ERROR_LEVEL_ERROR,
    ERROR_LEVEL_FATAL
} Enum_Error_Level;

/* ---------------- 错误码 ---------------- */
#define ERROR_CODE_NONE                 0u

/* RTOS 级致命(与 g_fault_code 一一对应) */
#define ERROR_CODE_RTOS_MALLOC          1u
#define ERROR_CODE_RTOS_STACK           2u
#define ERROR_CODE_ERROR_HANDLER        3u
#define ERROR_CODE_HARD_FAULT           4u

/* 设备初始化失败 */
#define ERROR_CODE_FLASH                5u
#define ERROR_CODE_LOG                  6u
#define ERROR_CODE_IMU                  7u
#define ERROR_CODE_ADC                  8u
#define ERROR_CODE_WS2812               9u
#define ERROR_CODE_WS2812_CLOCK         10u
#define ERROR_CODE_BUZZER               11u
#define ERROR_CODE_ELECTRIC             12u
#define ERROR_CODE_VOLTAGE              13u

/* 日志问题(WARNING) */
#define ERROR_CODE_LOG_FULL             14u
#define ERROR_CODE_LOG_DROPPED          15u

/* 电机:20/40/60 + 下标(最多量化 8 台) */
#define ERROR_CODE_MOTOR_MAX            8u
/* enable voltage supervision: 0 = bench/no battery, do not report under/over voltage */
#define ERROR_CHECK_VOLTAGE             0

#define ERROR_CODE_MOTOR_INIT_BASE      20u
#define ERROR_CODE_MOTOR_OFFLINE_BASE   40u
#define ERROR_CODE_MOTOR_FAULT_BASE     60u

/* 自定义标志位:80 + 位号 ⇒ 码 80~111(7 位能表示) */
#define ERROR_CODE_FLAG_BASE            80u

/* ---------------- 自定义错误位(ERROR_FLAG_xxx) ----------------
   设备初始化 + 电机状态本任务自己查;这里放"运行期"的错 */
#define ERROR_FLAG_MOTOR        0u      /* bit0 → 码 80 */
#define ERROR_FLAG_IMU          1u      /* bit1 → 码 81 */
#define ERROR_FLAG_REMOTE       2u
#define ERROR_FLAG_FLASH        3u
#define ERROR_FLAG_LOG          4u
#define ERROR_FLAG_LED          5u
#define ERROR_FLAG_ADC          6u
#define ERROR_FLAG_VOLTAGE      7u
#define ERROR_FLAG_SENSOR       8u
#define ERROR_FLAG_TASK         9u
#define ERROR_FLAG_CUSTOM       10u

#define ERROR_FLAG_MASK(__bit)  (1u << (__bit))

/* ---------------- 关键数据快照(各 App 层直接写) ---------------- */
typedef struct
{
    float Chassis_Cmd[3];       /* 底盘指令:vx, vy, w */
    float Chassis_Now[3];       /* 底盘实际(里程计):vx, vy, w */
    float Imu_Euler[3];         /* 欧拉角:roll, pitch, yaw(连续,rad) */
    float Gimbal_Cmd[2];        /* 云台指令:yaw, pitch(rad) */
    float Gimbal_Now[2];        /* 云台实际:yaw, pitch(rad) */
    uint8_t Mode;               /* 当前模式(自己定义) */
    uint8_t Reserved[3];        /* 对齐用 */
} Error_Snapshot_t;

/* ---------------- 落盘帧格式(给上位机 dump 用) ---------------- */
#define ERROR_FRAME_MAGIC       0x4552u     /* 'R''E' */
#define ERROR_FRAME_TYPE_SNAPSHOT   0u      /* 关键数据快照 */

typedef struct
{
    uint16_t Magic;             /* ERROR_FRAME_MAGIC */
    uint8_t Type;               /* ERROR_FRAME_TYPE_SNAPSHOT */
    uint8_t Level;              /* Enum_Error_Level */
    uint16_t Code;              /* ★ 错误码(0 = 无错) */
    uint16_t Reserved;
    uint32_t Ms;                /* 时间戳(ms,低 32 位) */
    uint32_t Flags;             /* 自定义错误位 */
    Error_Snapshot_t Snapshot;  /* 关键数据 */
} Error_Frame_t;

/* ---------------- 接口 ---------------- */

/**
 * @brief 跑一拍:算码 → 蜂鸣器/RGB 显示 → 按周期落盘关键数据
 * @return void
 * @note 由 ERROR 任务按 ERROR_TASK_PERIOD_MS 周期调用
 */
void Error_Task(void);

/**
 * @brief 取关键数据快照(各 App 层直接往里写,零拷贝)
 * @return 快照指针
 */
Error_Snapshot_t *Error_Get_Snapshot(void);

/**
 * @brief 当前错误码(0 = 无错)
 * @return 码
 */
uint16_t Error_Get_Code(void);

/**
 * @brief 当前错误级别
 * @return 级别
 */
Enum_Error_Level Error_Get_Level(void);

/**
 * @brief 报一个错(可按码报,也可以只用标志位)
 * @param __code   错误码(见上表;0 = 清掉这个码)
 * @param __active true = 出错,false = 恢复
 * @return void
 * @note 自定义码建议从 120 起;"亮着的码"里优先级最高的那个会被显示
 */
void Error_Report(uint16_t __code, bool __active);

/**
 * @brief 置/清一个自定义错误位(会自动折算成码 80 + 位号)
 * @param __mask ERROR_FLAG_MASK(ERROR_FLAG_xxx)
 * @param __set  true = 置位
 * @return void
 */
void Error_Set_Flag(uint32_t __mask, bool __set);
void Error_Clear_Flags(uint32_t __mask);

/**
 * @brief 当前自定义错误位
 * @return 位掩码
 */
uint32_t Error_Get_Flags(void);

/**
 * @brief 级别名(调试/日志用)
 * @return "OK" / "WARN" / "ERROR" / "FATAL"
 */
const char *Error_Get_Level_Name(Enum_Error_Level __level);

/**
 * @brief 错误码的可读名(含"哪台电机")
 * @param __code 错误码
 * @return 静态缓冲里的字符串(单任务用;不要保存指针)
 */
const char *Error_Get_Code_Name(uint16_t __code);

/**
 * @brief 当前蜂鸣器是否在响(调试用)
 * @return true = 在响
 */
bool Error_Is_Beeping(void);

#ifdef __cplusplus
}
#endif

#endif // ERROR_TASK_H
