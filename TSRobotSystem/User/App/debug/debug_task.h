/* ============================================================================
 * debug_task.h — 调试遥测(USART3 或 UART7 → 上位机:电机数据 + 姿态 + 故障现场)
 * 说明 / 用法 / 坑 见同目录 debug_task.md
 * ========================================================================== */

#ifndef DEBUG_TASK_H
#define DEBUG_TASK_H

#include "usart.h"
#include "robot_def.h"      /* CHASSIS_PHOTO_ENABLE */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 配置 ---------------- */
#define DEBUG_USE_UART7         1           /* 1 = 在 UART7(PE7/PE8, 921600)上发调试遥测 */
#if CHASSIS_PHOTO_ENABLE
#define DEBUG_USE_USART10       0           /* 光电占用 PE2/PE3,USART10 关闭 */
#else
#define DEBUG_USE_USART10       1
#endif
#define DEBUG_PERIOD_MS         10u         /* 回传周期:10ms = 100Hz(≈18KB/s,921600 够用) */
/* ★ USART3(PD8/PD9, DE=PB14)保留给 RS485 通信,不再用于调试 ✓ */
#define DEBUG_MAX_MOTORS        2u          /* 一帧最多带几台电机(每台 16B) */

/* ---------------- 帧格式(和裁判/miniPC 的 0xA5 区分开,避免混淆) ----------------
   [0]     0x5A              帧头
   [1]     版本(1)
   [2..3]  帧总长(2,LE)
   [4..5]  cmd_id(2,LE)
   [6..7]  序号(2,LE,每帧 +1)
   [8..]   payload
   [len-2..len-1] CRC16(CRC-CCITT,覆盖 [0..len-3])
   ------------------------------------------------------------------------- */
#define DEBUG_SOF               0x5Au
#define DEBUG_VERSION           1u
#define DEBUG_ID_TELEMETRY      0x0101u     /* 机器人 → 上位机:旧的大遥测帧 */
#define DEBUG_ID_MOTOR_PID      0x0102u     /* 极简帧:编码器速度 + 速度环目标/反馈 */
#define DEBUG_ID_STEER          0x0104u     /* 舵向实时角度:now0/tgt0/now1/tgt1 (4×f32) */
#define DEBUG_VERSION_PID       2u
/* 40:电机/光电;40 起故障诊断;60 起 IMU;84 起遥控器;116 起摩擦轮 */
#define DEBUG_MOTOR_PID_PAYLOAD 242u
#define DEBUG_MOTOR_PID_FRAME   (8u + DEBUG_MOTOR_PID_PAYLOAD + 2u)
#define DEBUG_PID_IMU_OFFSET     60u   /* 6 × f32: roll/pitch/yaw + gx/gy/gz */
#define DEBUG_PID_RC_OFFSET      84u   /* 32B: 在线/拨杆/鼠标/键盘/杆量/收帧数 */
#define DEBUG_PID_FRICTION_OFFSET 116u /* 28B: 0x207/0x208 flags + pos/w/t */
#define DEBUG_PID_M201_OFFSET  144u /* 16B: 0x201 drive */
#define DEBUG_PID_M203_OFFSET  160u /* 16B: 0x203 drive */
#define DEBUG_PID_M204_OFFSET  176u /* 28B: 0x204 steer */
#define DEBUG_PID_6020_OFFSET  204u /* 24B: 0x205 6020 yaw */
#define DEBUG_PID_STEER_AGE_OFFSET 228u  /* 4B: 202/204 舵向 CAN 帧年龄 [µs]，0xFFFF 表示无有效帧 */
#define DEBUG_PID_HOMING_OFFSET 232u /* 10B: 202/204 找零 state(u8×2) + scan_angle(f32×2) */
#define DEBUG_MOTOR_GEAR        8.0f   /* 与 motor_task.cpp 的 DJ_GEAR_RATIO 保持一致 */

/* 1 = 只发极简电机 PID 帧,其他外设遥测/文本全部停发 */
#define DEBUG_MOTOR_PID_ONLY    1

/* payload 布局(定长,小端;偏移量就是协议的一部分 ⇒ 改这里要同步改上位机) */
#define DEBUG_PL_ERR_LEVEL      0u          /* u8  错误级别 0=OK 1=WARN 2=ERROR 3=FATAL */
#define DEBUG_PL_MOTOR_COUNT    1u          /* u8  本帧带了几台电机 */
#define DEBUG_PL_ERR_CODE       2u          /* u16 错误码(见 error_task.h) */
#define DEBUG_PL_FAULT_CODE     4u          /* u32 g_fault_code(4 = HardFault) */
#define DEBUG_PL_FAULT_CFSR     8u          /* u32 SCB->CFSR:MMFSR/BFSR/UFSR */
#define DEBUG_PL_FAULT_PC       12u         /* u32 出错时的 PC(任务里跑飞时有效) */
#define DEBUG_PL_IMU            16u         /* 3 × f32:roll / pitch / yaw_total */
#define DEBUG_PL_CHASSIS        28u         /* 3 × f32:vx / vy / w */
#define DEBUG_PL_GIMBAL         40u         /* 2 × f32:yaw / pitch(实测关节角) */
#define DEBUG_PL_MOTORS         48u         /* 每台 16B */
#define DEBUG_MOTOR_SIZE        16u
#define DEBUG_PL_DIAG           (DEBUG_PL_MOTORS + (DEBUG_MAX_MOTORS * DEBUG_MOTOR_SIZE))  /* 176:诊断块(12B) */
#define DEBUG_PL_ACCEL          (DEBUG_PL_DIAG + 12u)                                       /* 188:加速度 m/s^2 */
#define DEBUG_PL_HWM            (DEBUG_PL_ACCEL + 12u)   /* 8 × u16:各任务栈水位(单位 4B) */
#define DEBUG_HWM_NUM           8u
#define DEBUG_PL_LOG            (DEBUG_PL_HWM + (DEBUG_HWM_NUM * 2u))  /* 5 × u32:日志上电链 */
#define DEBUG_LOG_NUM           5u
/* 每秒在调试口发一行**可读文本**(给人看,例:[LOG] boot=1 cur=0x002000 ...)✓
   上位机会把它单独显示;二进制帧不受影响 ✓ */
#define DEBUG_TEXT_ENABLE       1
#define DEBUG_TEXT_PERIOD_MS    1000u

#define DEBUG_PL_TEMP           (DEBUG_PL_LOG + 20u)

/* ★ 8 个任务的心跳计数(每跑一拍 +1;不涨 ⇒ 该任务卡住) */
#define DEBUG_PL_TICKS          (DEBUG_PL_TEMP + 4u)     /* 8 × u32 */
#define DEBUG_TICK_NUM          8u
#define DEBUG_PL_FLASH          (DEBUG_PL_TICKS + (DEBUG_TICK_NUM * 4u))  /* 5 × u32:Flash 诊断 */
#define DEBUG_PL_EXT            (DEBUG_PL_FLASH + 20u)   /* external IMU: gyro[3] + accel[3] (24B) */
#define DEBUG_EXT_SIZE          24u
/* 顺序与 robot_task 里的创建顺序一致:motor/robot/error/cmd/imu/chassis/gimbal/highRise */        /* f32:BMI088 温度(走加速度那颗 die ⇒ 判断 accel 通路是否活着) */

/* 每台电机 16B 的布局 */
#define DEBUG_M_ID              0u          /* u8  注册表下标(0 起) */
#define DEBUG_M_FLAGS           1u          /* u8  bit0 已初始化 / bit1 在线 / bit2 有 MError */
#define DEBUG_M_POS             4u          /* f32 位置(rad,关节侧) */
#define DEBUG_M_W               8u          /* f32 转速(rad/s,关节侧) */
#define DEBUG_M_T               12u         /* f32 力矩(N·m,关节侧) */

/**
 * @brief 初始化(只是清状态;UART 本身由 CubeMX 的 MX_USARTx_UART_Init 初始化)
 * @return true
 */
bool Debug_Init(void);

/**
 * @brief 跑一拍(由 DEBUG 任务按 DEBUG_PERIOD_MS 调用):组帧 + DMA 发送(不阻塞)
 */
void Debug_Task(void);

/**
 * @brief 已发帧数
 */
uint32_t Debug_Get_Tx_Frames(void);

/**
 * @brief 因"上一帧还没发完"而丢的帧数(串口太慢/上位机没接时会涨,属正常)
 */
uint32_t Debug_Get_Tx_Skip(void);

/**
 * @brief 一帧的总字节数(固定值,方便上位机校验)
 */
uint16_t Debug_Get_Frame_Size(void);

/** @brief 初始化期间直接发一行找零状态(不依赖 Debug_Task/RTOS) */
void Debug_Send_Homing(uint8_t __idx, float __pos, uint8_t __pe2, uint8_t __pe3, uint8_t __state);
/** @brief 舵向实时角度帧(0x0104,48B)，函数内限频为 50Hz */
void Debug_Send_Steer_Fast(void);

#ifdef __cplusplus
}
#endif

#endif // DEBUG_TASK_H
