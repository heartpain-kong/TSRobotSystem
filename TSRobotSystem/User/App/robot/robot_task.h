/* ============================================================================
 * robot_task.h — 任务创建入口
 *
 * ⚠️ 仿 basic_framework 的 application/robot_task.h:**本文件只用于"创建任务",
 *    只应被 robot.cpp 包含**,别在别处 include(它会把各子系统的头都拉进来)。
 *
 * 分工:
 *   · 本文件 + robot_task.cpp:建任务、跑任务循环、监测任务有没有超时
 *   · robot.cpp              :Robot_Init() / Robot_Task()
 *   · robot_def.h            :周期/优先级/栈/子系统开关
 * ========================================================================== */

#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include "cmsis_os2.h"
#include "robot_def.h"

/* ---- 子系统任务体(跑一拍,不含循环;由各自的模块提供)---- */
#if ROBOT_HAVE_MOTOR
#include "motor_task.h"      // motor_task_init() / motor_task()
#endif

/* ---- 任务句柄:调试时看它们确认任务真的建起来了 ---- */
extern osThreadId_t motor_task_handle;
extern osThreadId_t robot_task_handle;

/* ---- 任务超时(执行时间 > 周期)计数:调试时看这两个变量,一直涨说明该拍算不完 ---- */
extern volatile uint32_t motor_task_overrun;
extern volatile uint32_t robot_task_overrun;

/* osThreadNew 失败次数:>0 ⇒ FreeRTOS 堆不够 */
extern volatile uint32_t g_thread_create_failed;

/* 注:OSTaskInit() 的声明在 robot.h 里(它要能被 C 文件调用,必须是 C 链接),
   本文件不再重复声明,避免链接方式冲突。 */

#endif // ROBOT_TASK_H
