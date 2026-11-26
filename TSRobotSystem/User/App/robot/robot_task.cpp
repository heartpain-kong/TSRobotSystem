//
// robot_task.cpp — 创建并运行机器人各任务(仿 basic_framework 的 application/robot_task.h)
//
// 每个任务一个无限循环:
//    记起始时刻 → 调子系统"跑一拍"的函数 → 用 DWT 量本拍耗时 → 超时计数 → 等下一拍
// ⚠️ 只被 robot.cpp 通过 robot_task.h 使用。
//

#include "robot_task.h"

#include "bsp_dwt.h"        //本拍耗时用 DWT 量(1us 分辨率)
#include "robot.h"          //Robot_Task()
#include "error_task.h"     //Error_Task():错误显示(蜂鸣器+RGB) + 关键数据记录
#include "cmd_task.h"       //Cmd_Task():遥控/裁判/上位机 → 统一控制指令
#include "imu_task.h"       //Imu_Task():BMI088 + EKF → IMU 话题
#include "chassis_task.h"   //Chassis_Task():底盘(麦轮)
#include "gimbal_task.h"    //Gimbal_Task():云台(yaw/pitch)
#include "highRise_task.h"  //HighRise_Task():末端机构(发射)
#include "debug_task.h"
#include "dev_bmi088.h"

/* ★ 每个任务的心跳:遥测里看它是否在涨 ⇒ 判断这个任务是否卡住 ✓ */
volatile uint32_t g_task_tick[8] = { 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u };      //Debug_Task():串口遥测(电机数据/姿态/故障)

/* ---- 任务句柄 ---- */
osThreadId_t motor_task_handle = nullptr;
osThreadId_t robot_task_handle = nullptr;
osThreadId_t error_task_handle = nullptr;
osThreadId_t cmd_task_handle = nullptr;
osThreadId_t imu_task_handle = nullptr;
osThreadId_t chassis_task_handle = nullptr;
osThreadId_t gimbal_task_handle = nullptr;
osThreadId_t highRise_task_handle = nullptr;
osThreadId_t debug_task_handle = nullptr;

/* ---- 超时计数(执行时间超过周期就 +1;调试时看它,一直涨说明这拍根本算不完)---- */
volatile uint32_t motor_task_overrun = 0u;
volatile uint32_t robot_task_overrun = 0u;
volatile uint32_t imu_task_overrun = 0u;   /* 1kHz 跑 EKF 撑不住时这里会一直涨 */

/* osThreadNew 失败次数:>0 说明 FreeRTOS 堆不够(configTOTAL_HEAP_SIZE 太小) */
volatile uint32_t g_thread_create_failed = 0u;

/* ============================================================================
 * 电机控制任务:1kHz
 *   写目标 → 算需要主机闭环的电机 → 按各自频率发帧
 *   注意:发不发由电机自己的 Is_Send_Due() 决定,所以本任务即使跑到 1kHz,
 *         也不会把 500Hz 的电机发成 1kHz
 * ========================================================================== */
#if ROBOT_HAVE_MOTOR
static void start_motor_task(void *argument)
{
    (void)argument;

    motor_task_handle = osThreadGetId();

    uint32_t next = osKernelGetTickCount();

    for (;;)
    {
        uint64_t t0 = DWT_GetTimeline_us();

        motor_task();                                   //跑一拍

        uint32_t dt_us = (uint32_t)(DWT_GetTimeline_us() - t0);
        if (dt_us > (MOTOR_TASK_PERIOD_MS * 1000u))
        {
            motor_task_overrun++;                       //本拍超时
        }

        next += MOTOR_TASK_PERIOD_MS;
        g_task_tick[0]++;   /* motor 心跳 */
        osDelayUntil(next);                             //按绝对时刻等,不会累积漂移
    }
}
#endif

/* ============================================================================
 * 机器人高层任务:500Hz
 *   读遥控/视觉 → 算参考值 → 交给各子系统(决策,不发帧)
 * ========================================================================== */
static void start_robot_task(void *argument)
{
    (void)argument;

    robot_task_handle = osThreadGetId();

    uint32_t next = osKernelGetTickCount();

    for (;;)
    {
        uint64_t t0 = DWT_GetTimeline_us();

        Robot_Task();                                   //跑一拍

        uint32_t dt_us = (uint32_t)(DWT_GetTimeline_us() - t0);
        if (dt_us > (ROBOT_TASK_PERIOD_MS * 1000u))
        {
            robot_task_overrun++;
        }

        next += ROBOT_TASK_PERIOD_MS;
        g_task_tick[1]++;   /* robot 心跳 */
        osDelayUntil(next);
    }
}

/* ============================================================================
 * 错误处理 / 关键数据记录任务:100Hz(优先级最低)
 *   评估错误级别 → 蜂鸣器图案 + WS2812 颜色 → 每 20ms 把关键数据打成二进制帧落盘
 * ========================================================================== */
static void start_error_task(void *argument)
{
    (void)argument;

    error_task_handle = osThreadGetId();

    uint32_t next = osKernelGetTickCount();

    for (;;)
    {
        Error_Task();                                   //跑一拍

        next += ERROR_TASK_PERIOD_MS;
        g_task_tick[2]++;   /* error 心跳 */
        osDelayUntil(next);
    }
}

/* ============================================================================
 * 指令任务:500Hz —— 遥控器/键鼠 + 裁判系统 + 上位机 → 统一控制指令
 *   它同时负责推进遥控器与裁判系统两个设备(掉线检测 / 发布到环形数组)
 *   500Hz 也是上/下 MCU 链路的发送节拍(见 cmd_485)
 * ========================================================================== */
static void start_cmd_task(void *argument)
{
    (void)argument;

    cmd_task_handle = osThreadGetId();

    uint32_t next = osKernelGetTickCount();

    for (;;)
    {
        Cmd_Task();

        next += CMD_TASK_PERIOD_MS;
        g_task_tick[3]++;   /* cmd 心跳 */
        osDelayUntil(next);
    }
}

/* ============================================================================
 * 姿态解算任务:1kHz —— BMI088 读数 → 四元数 EKF → 发布 IMU 话题
 *   优先级与电机任务同高:姿态是所有控制环的输入,不能晚
 * ========================================================================== */
static void start_imu_task(void *argument)
{
    (void)argument;

    imu_task_handle = osThreadGetId();

    uint32_t next = osKernelGetTickCount();

    for (;;)
    {
        uint64_t t0 = DWT_GetTimeline_us();

        Imu_Task();

        if ((uint32_t)(DWT_GetTimeline_us() - t0) > (IMU_TASK_PERIOD_MS * 1000u))
        {
            imu_task_overrun++;
        }

        next += IMU_TASK_PERIOD_MS;
        g_task_tick[4]++;   /* imu 心跳 */
        osDelayUntil(next);
    }
}

/* ============================================================================
 * 底盘任务:500Hz —— 收 CMD + IMU_CHASSIS → 麦轮解算 → 写电机目标 → 正解里程计
 * ========================================================================== */
static void start_chassis_task(void *argument)
{
    (void)argument;

    chassis_task_handle = osThreadGetId();

    uint32_t next = osKernelGetTickCount();

    for (;;)
    {
        Chassis_Task();

        next += CHASSIS_TASK_PERIOD_MS;
        g_task_tick[5]++;   /* chassis 心跳 */
        osDelayUntil(next);
    }
}

/* ============================================================================
 * 云台任务:500Hz —— 收 CMD + IMU_GIMBAL + CHASSIS → yaw/pitch 关节角 MIT 控制
 * ========================================================================== */
static void start_gimbal_task(void *argument)
{
    (void)argument;

    gimbal_task_handle = osThreadGetId();

    uint32_t next = osKernelGetTickCount();

    for (;;)
    {
        Gimbal_Task();

        next += GIMBAL_TASK_PERIOD_MS;
        g_task_tick[6]++;   /* gimbal 心跳 */
        osDelayUntil(next);
    }
}

/* ============================================================================
 * 末端机构任务:500Hz —— 收 CMD(+CHASSIS/GIMBAL 联锁)→ 摩擦轮/拨弹(或升降)
 * ========================================================================== */
static void start_highRise_task(void *argument)
{
    (void)argument;

    highRise_task_handle = osThreadGetId();

    uint32_t next = osKernelGetTickCount();

    for (;;)
    {
        HighRise_Task();

        next += HIGHRISE_TASK_PERIOD_MS;
        g_task_tick[7]++;   /* highRise 心跳 */
        osDelayUntil(next);
    }
}

/* ============================================================================
 * 调试遥测任务:100Hz —— 组帧 + DMA 发送(USART3/UART7 → 上位机)
 * ========================================================================== */
static void start_debug_task(void *argument)
{
    (void)argument;

    debug_task_handle = osThreadGetId();

    uint32_t next = osKernelGetTickCount();

    for (;;)
    {
        Debug_Task();

        next += DEBUG_TASK_PERIOD_MS;
        osDelayUntil(next);
    }
}

/* ============================================================================
 * 建任务(仿 basic_framework 的 OSTaskInit:所有持续运行的任务都在这里创建)
 *   osThreadNew 在 osKernelStart() 之前调用完全可以 —— 任务会在调度器跑起来后才执行
 * ========================================================================== */
extern "C" void OSTaskInit(void)
{
    /* ⚠️ 一定检查 osThreadNew 的返回值:堆不够时它只会返回 NULL(不报错),
       任务就"静默地不存在" —— 现象是某个功能完全不跑,极难查。
       失败个数记在 g_thread_create_failed(调试器直接看,>0 就说明要加大
       FreeRTOSConfig.h 的 configTOTAL_HEAP_SIZE)。 */
#if ROBOT_HAVE_MOTOR
    {
        osThreadAttr_t attr = {};
        attr.name       = "motor";
        attr.stack_size = MOTOR_TASK_STACK;
        attr.priority   = MOTOR_TASK_PRIORITY;
        if (osThreadNew(start_motor_task, nullptr, &attr) == nullptr) { g_thread_create_failed++; }
    }
#endif

    {
        osThreadAttr_t attr = {};
        attr.name       = "robot";
        attr.stack_size = ROBOT_TASK_STACK;
        attr.priority   = ROBOT_TASK_PRIORITY;
        if (osThreadNew(start_robot_task, nullptr, &attr) == nullptr) { g_thread_create_failed++; }
    }

    {
        osThreadAttr_t attr = {};
        attr.name       = "error";
        attr.stack_size = ERROR_TASK_STACK;
        attr.priority   = ERROR_TASK_PRIORITY;
        if (osThreadNew(start_error_task, nullptr, &attr) == nullptr) { g_thread_create_failed++; }
    }

    {
        osThreadAttr_t attr = {};
        attr.name       = "cmd";
        attr.stack_size = CMD_TASK_STACK;
        attr.priority   = CMD_TASK_PRIORITY;
        if (osThreadNew(start_cmd_task, nullptr, &attr) == nullptr) { g_thread_create_failed++; }
    }

    {
        osThreadAttr_t attr = {};
        attr.name       = "imu";
        attr.stack_size = IMU_TASK_STACK;
        attr.priority   = IMU_TASK_PRIORITY;
        if (osThreadNew(start_imu_task, nullptr, &attr) == nullptr) { g_thread_create_failed++; }
    }

    {
        osThreadAttr_t attr = {};
        attr.name       = "chassis";
        attr.stack_size = CHASSIS_TASK_STACK;
        attr.priority   = CHASSIS_TASK_PRIORITY;
        if (osThreadNew(start_chassis_task, nullptr, &attr) == nullptr) { g_thread_create_failed++; }
    }

    {
        osThreadAttr_t attr = {};
        attr.name       = "gimbal";
        attr.stack_size = GIMBAL_TASK_STACK;
        attr.priority   = GIMBAL_TASK_PRIORITY;
        if (osThreadNew(start_gimbal_task, nullptr, &attr) == nullptr) { g_thread_create_failed++; }
    }

    {
        osThreadAttr_t attr = {};
        attr.name       = "highRise";
        attr.stack_size = HIGHRISE_TASK_STACK;
        attr.priority   = HIGHRISE_TASK_PRIORITY;
        if (osThreadNew(start_highRise_task, nullptr, &attr) == nullptr) { g_thread_create_failed++; }
    }

    {
        osThreadAttr_t attr = {};
        attr.name       = "debug";
        attr.stack_size = DEBUG_TASK_STACK;
        attr.priority   = DEBUG_TASK_PRIORITY;
        if (osThreadNew(start_debug_task, nullptr, &attr) == nullptr) { g_thread_create_failed++; }
    }
}
