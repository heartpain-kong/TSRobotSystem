/* ============================================================================
 * robot.h — 机器人层入口 + 决策层门控(给 C 文件用的纯声明头)
 * 说明 / 用法 / 坑 见同目录 robot.md
 * ========================================================================== */

#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 机器人初始化 —— **唯一需要放进 main.c 的函数**
 *
 * 顺序:各 BSP → 各子系统 Init(只初始化,不建任务)。
 * 必须在 osKernelStart() 之前调用(main.c 的 USER CODE BEGIN 2 里)。
 */
void Robot_Init(void);

/**
 * @brief 创建所有持续运行的 RTOS 任务
 *
 * ⚠️ CMSIS-RTOS2 要求**先 osKernelInitialize() 才能建线程**,所以本函数不能在
 *    main.c 的 USER CODE 2 里调,而由 Core/Src/freertos.c 的 MX_FREERTOS_Init()
 *    在 `USER CODE BEGIN RTOS_THREADS` 段里调用。
 */
void OSTaskInit(void);

/**
 * @brief 机器人高层逻辑,跑**一拍**(不含循环)
 *
 * 由 ROBOT 任务按 ROBOT_TASK_PERIOD_MS 周期调用:子系统服务(日志落盘/电压检测)
 * + **决策层**(裁判系统 → 门控 → 发 MSG_TOPIC_ROBOT)。
 */
void Robot_Task(void);

/* ============================================================================
 * 决策层(流程图:裁判系统 → robot → chassis/gimbal/highRise)
 *   robot 把"能不能动/能不能打"统一成一个门控,发到 MSG_TOPIC_ROBOT
 *   ★ 裁判系统离线 ⇒ 一律允许(练习/调试)
 * ========================================================================== */

typedef struct
{
    bool Motion_Allowed;        /* 允许底盘运动 */
    bool Shoot_Allowed;         /* 允许发弹 */
    uint8_t Mode_Limit;         /* 0 无限制 / 2 全锁 */
    uint8_t Referee_Progress;   /* 裁判系统的比赛阶段(转发,便于调试) */
    bool Referee_Online;
    uint32_t Seq;
} Robot_Data_t;

/**
 * @brief 读决策层数据
 */
const Robot_Data_t *Robot_Get(void);

/**
 * @brief 现在允许底盘运动吗(机内默认 true ⇒ fail-open,不会因为没数据就把车锁死)
 */
bool Robot_Is_Motion_Allowed(void);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_H
