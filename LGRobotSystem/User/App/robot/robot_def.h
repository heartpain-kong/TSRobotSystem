/* ============================================================================
 * robot_def.h — 这台机器人的配置(子系统开关 + 任务周期/优先级/栈)
 *
 * 仿 basic_framework 的 application/robot_def.h:
 *   换机器人、换任务频率、去掉没装的子系统,都只改这一个文件。
 *
 * ⚠️ 加任务前先看这里:栈单位是**字节**,优先级 osPriorityXxx(数值越大越高)。
 *    优先级别都设成一样的最高:1kHz 的电机任务要能抢占,否则频率会抖。
 * ========================================================================== */

#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "cmsis_os2.h"

/* ============================================================================
 * ★★ 本机配置 —— 换车 / 换板 / 调参,只改这一段 ★★
 *
 *   改数值只改这里,不要再去各模块里翻:
 *     · 这块板是哪块          ROBOT_BOARD_SEL
 *     · 底盘几何(轴距/轮距/轮径)CFG_CHASSIS_*
 *     · 舵向 / 云台标定        CFG_STICK_* / CFG_SEAT_TRIM_* / CFG_6020_ZERO_ENC
 *     · 速度 / 功率上限        CFG_MAX_* / CFG_POWER_LIMIT
 *     · 发射(射速)            CFG_FRICTION_RPM / CFG_TRIGGER_RPM / CFG_TRIGGER_SLOTS
 *
 *   各模块用**自己的老名字**引用这些值(例如 chassis_task.cpp 的 CHASSIS_HALF_A
 *   就是 CFG_CHASSIS_HALF_A),所以模块内部代码不用动 ✓
 * ========================================================================== */

/* ---------------- 0. 这块板是哪块 ----------------
   同一份代码烧两块板,只有这一个开关不同(cmd_485.md 第 97 行就是按这个约好的):
     下位机 = 底盘/舵向/驱动/云台 6020/底盘 IMU/找零/日志
     上板   = 遥控接收 + 485 中继 + 调试分片转发(UART 直发也只在上板开) */
#define ROBOT_BOARD_LOWER       0
#define ROBOT_BOARD_UPPER       1
#define ROBOT_BOARD_SEL         ROBOT_BOARD_LOWER   /* ★ 烧上板时改成 ROBOT_BOARD_UPPER */
#define ROBOT_UPPER_ONLY        (ROBOT_BOARD_SEL == ROBOT_BOARD_UPPER)

/* ---------------- 1. 底盘几何 [m] ---------------- */
#define CFG_CHASSIS_HALF_A      0.1625f     /* 舵轮(驱动轮)前后半轴距 */
#define CFG_CHASSIS_HALF_B      0.1625f     /* 舵轮(驱动轮)左右半轴距 */
#define CFG_CHASSIS_HALF_OMNI   0.183345f   /* 被动全向轮半距(只记录,不参与驱动解算) */
#define CFG_CHASSIS_WHEEL_R     0.05f       /* 驱动轮半径 */
/* 舵轮中心到车心 = sqrt(A² + B²);改 A/B 后这个数要跟着改(航向修正按它归一化) */
#define CFG_CHASSIS_HALF_DIAG   0.2298f

/* ---------------- 2. 舵向 / 云台标定 ---------------- */
#define CFG_STICK_TO_BODY_YAW_DEG   303.0f  /* 摇杆 y+ 在车体系的角度(实测标定值) */
#define CFG_GIMBAL_POS_SIGN         (+1.0f) /* 云台与舵向同向:+1;装反了才改 -1 */
#define CFG_SEAT_TRIM_DEG_202       (-10.0f)/* 202 舵向微调:叠加在光电找零算出的零位上 */
#define CFG_SEAT_TRIM_DEG_204       (0.0f)  /* 204 同上 */
#define CFG_6020_ZERO_ENC           550u    /* 6020 零点:raw 550 = 云台指向 +Y(车头正前) */

/* ---------------- 3. 速度 / 功率上限 ---------------- */
#define CFG_MAX_VX              3.0f        /* 前后最大速度 [m/s] */
#define CFG_MAX_VY              3.0f        /* 左右最大速度 [m/s] */
#define CFG_MAX_W               8.0f        /* 最大旋转角速度 [rad/s] */
#define CFG_POWER_LIMIT         120u        /* 底盘功率上限 [W] */

/* ---------------- 4. 发射 ---------------- */
#define CFG_FRICTION_RPM        3000.0f     /* 摩擦轮目标转速 [rpm] */
#define CFG_TRIGGER_RPM         100.0f      /* 连发时拨弹盘转速 [rpm] */
#define CFG_TRIGGER_SLOTS       6u          /* 拨弹盘一圈几格(单发走一格) */
/* 射速 = CFG_TRIGGER_RPM / 60 × CFG_TRIGGER_SLOTS
   现在 = 100/60×6 ≈ 10.0 发/s(改这两个数就改射速) */

/* ---------------- 运行模式 / 功率 ----------------
   PRACTICE: 练习模式,裁判离线也允许调试,用 PRACTICE_POWER_LIMIT 保守限功率
   COMPETITION: 比赛模式,功率和发弹完全听裁判系统 */
#define ROBOT_RUN_COMPETITION   0
#define ROBOT_RUN_PRACTICE      1
#define ROBOT_RUN_MODE          ROBOT_RUN_PRACTICE
#define PRACTICE_POWER_LIMIT    CFG_POWER_LIMIT

/* ---------------- 舵轮上电找零 ----------------
   1 = Chassis_Init() 阻塞等待 202/204 找零完成
   0 = 跳过找零 */
#define CHASSIS_SERVO_D_INIT_ENABLE 1

/* ---------------- 这台机器人装了哪些子系统 ----------------
   没装的注释掉:省 RAM(每个任务一份栈)也省 CPU */
#define ROBOT_HAVE_MOTOR        1

/* 1 = 把 USART10 的两个脚(PE2/PE3)改成 GPIO 输入,读两路光电找零;
   0 = PE2/PE3 保持 USART10,原串口功能不变。
   注意:两者不能同时用。 */
#define CHASSIS_PHOTO_ENABLE    1
// #define ROBOT_HAVE_INS       1       // 姿态解算(以后加)
// #define ROBOT_HAVE_REFEREE   1       // 裁判系统(以后加)

/* ---------------- 任务参数 ----------------
   周期:osDelayUntil 用它换算成 tick(RTOS tick = 1ms)
   栈  :字节。对象都在全局,栈上只有局部变量,2KB 一般够;有浮点大数组再加 */

//电机控制任务:1kHz —— 写目标、算闭环、按各自频率发帧
#define MOTOR_TASK_PERIOD_MS    1u
#define MOTOR_TASK_PRIORITY     osPriorityHigh
#define MOTOR_TASK_STACK        2048u

//机器人高层任务:500Hz —— 读遥控/视觉,给各子系统发参考值
#define ROBOT_TASK_PERIOD_MS    2u
#define ROBOT_TASK_PRIORITY     osPriorityNormal
#define ROBOT_TASK_STACK        2048u

//错误处理/记录任务:100Hz —— 最低优先级,评估错误级别(蜂鸣器+RGB)并按 50Hz 记关键数据
//  优先级刻意最低:它被别的任务挤掉也不影响控制;里面只往日志内存环形里塞,不阻塞
#define ERROR_TASK_PERIOD_MS    10u
#define ERROR_TASK_PRIORITY     osPriorityLow
#define ERROR_TASK_STACK        4096u       //snprintf/vsnprintf 很吃栈(3072 实测偏紧)⇒ 给到 4KB

//指令任务:500Hz —— 遥控器/键鼠 + 裁判系统 + 上位机 → 统一控制指令(给 chassis/gimbal/highRise)
//  ⚠️ 500Hz 也是上/下 MCU 链路(USART2 RS485)的命令/反馈发送频率(见 cmd_485)
#define CMD_TASK_PERIOD_MS      2u
#define CMD_TASK_PRIORITY       osPriorityNormal
#define CMD_TASK_STACK          3072u

//姿态解算任务:1kHz —— BMI088 + 四元数 EKF,产出 IMU 话题(底盘/云台)
//  ⚠️ 1kHz 跑 EKF 偏重:若 imu_task_overrun 一直涨,就把周期改成 2(500Hz)
#define IMU_TASK_PERIOD_MS      1u
#define IMU_TASK_PRIORITY       osPriorityHigh
#define IMU_TASK_STACK          2048u

//底盘任务:500Hz —— 收 CMD + IMU_CHASSIS → 麦轮解算 → 写电机目标 → 正解里程计 → 发 CHASSIS
#define CHASSIS_TASK_PERIOD_MS  2u
#define CHASSIS_TASK_PRIORITY   osPriorityNormal
#define CHASSIS_TASK_STACK      2048u

/* ---------------- 电机分配(注册表下标 → 用途;★ 按实车接线改) ----------------
   ⚠️ 同一路电机只能归一个任务。现在注册表只有 2 台(CAN1 两台达妙):
      底盘占 0/1(四轮只接上前两路),云台默认**未分配**(0xFF);
      接上云台电机后,把 GIMBAL_MOTOR_YAW/PITCH 改成实际下标即可 ✓ */
#define CHASSIS_MOTOR_0         0u      /* 1 号轮(右前) */
#define CHASSIS_MOTOR_1         1u      /* 2 号轮(左前) */
#define CHASSIS_MOTOR_2         2u      /* 3 号轮(左后) */
#define CHASSIS_MOTOR_3         3u      /* 4 号轮(右后) */
#define GIMBAL_MOTOR_YAW        0xFFu   /* 云台 yaw:0xFF = 未分配 */
#define GIMBAL_MOTOR_PITCH      0xFFu   /* 云台 pitch:0xFF = 未分配 */

//云台任务:500Hz —— 收 CMD + IMU_GIMBAL + CHASSIS → yaw/pitch 关节角 MIT 控制 + 小陀螺前馈
#define GIMBAL_TASK_PERIOD_MS   2u
#define GIMBAL_TASK_PRIORITY    osPriorityNormal
#define GIMBAL_TASK_STACK       2048u

/* ---------------- 末端机构(highRise;★ 这是 RC(ROBOCON) 与 RM 的"app 层差异点" ----------------
   本系统两个比赛共用:RM 用发射(默认),RC 只改这一个宏 + 对应分支(升降)即可 ✓ */
#define HIGHRISE_MECH_SHOOT     0u      /* 发射:2 个摩擦轮 + 1 个拨弹 */
#define HIGHRISE_MECH_LIFT      1u      /* 升降/抬升(接口已预留) */
#define HIGHRISE_MECH           HIGHRISE_MECH_SHOOT

#define HIGHRISE_MOTOR_FRICTION_L   0xFFu   /* 摩擦轮左:0xFF = 未分配 */
#define HIGHRISE_MOTOR_FRICTION_R   0xFFu   /* 摩擦轮右 */
#define HIGHRISE_MOTOR_TRIGGER      0xFFu   /* 拨弹盘 */
#define HIGHRISE_MOTOR_LIFT         0xFFu   /* 升降电机(预留) */

//末端机构任务:500Hz —— 收 CMD(+CHASSIS/GIMBAL 联锁)→ 摩擦轮/拨弹(或升降)→ 发 HIGHRISE
#define HIGHRISE_TASK_PERIOD_MS 2u
#define HIGHRISE_TASK_PRIORITY  osPriorityNormal
#define HIGHRISE_TASK_STACK     2048u


/* ---------------- 调试台开关(★ 上板前确认) ----------------
   裸板/调试时只靠 ST-LINK 供电:WS2812 与蜂鸣器可能挂在 5V 上 ⇒ 5V 不开就没声音没灯 ✗
   想让它上电就打开 5V ⇒ 把下面改成 1(正式上电仍然默认关 ✓) */
#define POWER_5V_ON_AT_BOOT     1

//调试遥测任务:100Hz —— USART3(或 UART7)向上位机发电机数据/姿态/故障现场
//  ★ 优先级可给最低:它是诊断用途,被别的任务挤掉没关系(丢帧只会计到 Debug_Get_Tx_Skip)
#define DEBUG_TASK_PERIOD_MS    10u
#define DEBUG_TASK_PRIORITY     osPriorityLow
#define DEBUG_TASK_STACK        4096u   //snprintf/newlib 很吃栈,给足(踩过)✗


/* ---------------- RS485 电机(宇树 YS 系列)----------------
   USART3(PD8/PD9, DE=PB14 硬件驱动, 4Mbps)/ USART2(DE=PD4) 都可用于 485 电机 ✓
   配置表在 User/Device/motor/dev_motor_485.cpp 里(串口/ID/型号/模式/频率)✓
   MOTOR_485_AUTO_ENABLE = 1 时上电就使能(台架调试用;上实车请保持 0)✗ */
#define MOTOR_485_ENABLE        0       /* 实车只有大疆舵向, 无 485 电机 ✓ */
#define MOTOR_485_AUTO_ENABLE   0

#endif // ROBOT_DEF_H