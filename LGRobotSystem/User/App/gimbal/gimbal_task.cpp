//
// gimbal_task.cpp — 云台任务(yaw/pitch 关节角 MIT 控制 + 小陀螺前馈补偿)
// 说明 / 用法 / 坑 见同目录 gimbal_task.md
//

#include "gimbal_task.h"

#include "bsp_timestamp.h"      /* dt(小陀螺前馈用) */
#include "chassis_task.h"       /* 流程图里的"chassis → gimbal":拿底盘自转角速度 */
#include "cmd_task.h"           /* 控制指令 */
#include "imu_task.h"           /* 云台 imu 角度 */
#include "math_support.h"       /* 限幅 */
#include "motor_task.h"         /* MIT 位置目标 */
#include "msg_center.h"
#include "error_task.h"         /* 关键数据快照 */
#include "robot_def.h"          /* GIMBAL_MOTOR_YAW/PITCH(电机分配集中在这里) */

#include <string.h>

static_assert(sizeof(Gimbal_Data_t) <= MSG_SLOT_SIZE, "Gimbal_Data_t 超过 MSG_SLOT_SIZE");

/* ============================================================================
 * 参数(★ 按实车改)
 * ========================================================================== */

#define GIMBAL_YAW_MIN          (-2.967f)   /* yaw 关节限位 ±170° */
#define GIMBAL_YAW_MAX          (2.967f)
#define GIMBAL_PITCH_MIN        (-0.60f)    /* pitch 关节限位 */
#define GIMBAL_PITCH_MAX        (0.50f)
#define GIMBAL_KP               30.0f       /* MIT 刚度(位置误差 → 力矩) */
#define GIMBAL_KD               1.0f        /* MIT 阻尼 */
#define GIMBAL_SPIN_COMP_GAIN   1.0f        /* 小陀螺前馈增益:1 = 完全反向补偿,0 = 关 */

/* ============================================================================
 * 内部状态
 * ========================================================================== */

static Gimbal_Data_t s_data;
static Cmd_Data_t s_cmd;
static uint32_t s_cmd_seq = 0u;
static uint32_t s_imu_seq = 0u;
static uint32_t s_chassis_seq = 0u;
static uint32_t s_last_us = 0u;
static bool s_inited = false;

/* ============================================================================
 * 小工具
 * ========================================================================== */

static float Clamp_F(float __v, float __min, float __max)
{
    if (__v > __max)
    {
        return __max;
    }
    if (__v < __min)
    {
        return __min;
    }
    return __v;
}

/**
 * @brief 这一路电机能不能用(未分配 0xFF / 越界 / 没初始化成功 都不行)
 */
static bool Axis_Motor_Ok(uint8_t __motor_index)
{
    if (__motor_index == 0xFFu)
    {
        return false;
    }
    if (__motor_index >= motor_task_get_count())
    {
        return false;
    }
    return motor_task_is_init_ok(__motor_index);
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

void Gimbal_Init(void)
{
    memset(&s_data, 0, sizeof(s_data));
    memset(&s_cmd, 0, sizeof(s_cmd));
    s_cmd_seq = 0u;
    s_imu_seq = 0u;
    s_chassis_seq = 0u;
    s_last_us = Timestamp_Get_Us32();
    s_inited = true;
}

void Gimbal_Task(void)
{
    Cmd_Data_t cmd;
    Imu_Data_t imu;
    Chassis_Data_t chas;
    Error_Snapshot_t *snap;
    uint32_t now_us;
    uint32_t dt_us;
    float dt;
    float target[GIMBAL_AXIS_N];
    uint8_t i;
    uint8_t axis_count = 0u;

    if (!s_inited)
    {
        return;
    }

    s_data.Seq++;

    /* ① 收三路输入(广播型:取最新,不弹出) */
    if (Msg_Is_New(MSG_TOPIC_CMD, s_cmd_seq) &&
        Msg_Latest(MSG_TOPIC_CMD, &cmd, (uint16_t)sizeof(cmd), &s_cmd_seq))
    {
        s_cmd = cmd;
    }
    if (Msg_Is_New(MSG_TOPIC_IMU_GIMBAL, s_imu_seq))
    {
        (void)Msg_Latest(MSG_TOPIC_IMU_GIMBAL, &imu, (uint16_t)sizeof(imu), &s_imu_seq);
    }
    else
    {
        imu = *Imu_Get(IMU_CHANNEL_GIMBAL);
    }
    if (Msg_Is_New(MSG_TOPIC_CHASSIS, s_chassis_seq))
    {
        (void)Msg_Latest(MSG_TOPIC_CHASSIS, &chas, (uint16_t)sizeof(chas), &s_chassis_seq);
    }
    else
    {
        chas = *Chassis_Get();
    }

    /* ② dt */
    now_us = Timestamp_Get_Us32();
    dt_us = (uint32_t)(now_us - s_last_us);
    s_last_us = now_us;
    if (dt_us > 100000u)
    {
        dt_us = 100000u;
    }
    dt = (float)dt_us * 1.0e-6f;

    /* ③ 小陀螺前馈:底盘自转带着云台转 ⇒ 关节目标反向补偿,尽量保持世界朝向
          (用底盘里程计的 W,而不是云台 imu —— 否则会把操作手自己的转动也算进去) */
    if (chas.Spin && (GIMBAL_SPIN_COMP_GAIN > 0.0f))
    {
        s_data.Spin_Comp -= GIMBAL_SPIN_COMP_GAIN * chas.W * dt;
        s_data.Spin_Comp = Clamp_F(s_data.Spin_Comp, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
    }
    else
    {
        s_data.Spin_Comp = 0.0f;
    }

    /* ④ 目标角 = 指令 + 前馈,再限位 */
    target[GIMBAL_AXIS_YAW] = Clamp_F(s_cmd.Gimbal_Yaw + s_data.Spin_Comp, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
    target[GIMBAL_AXIS_PITCH] = Clamp_F(s_cmd.Gimbal_Pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);

    /* ⑤ 写电机(MIT **位置**控制:关节侧 rad,和电机接口同单位 ⇒ 不需要换算) */
    for (i = 0u; i < GIMBAL_AXIS_N; i++)
    {
        uint8_t idx = (i == GIMBAL_AXIS_YAW) ? GIMBAL_MOTOR_YAW : GIMBAL_MOTOR_PITCH;

        if (!Axis_Motor_Ok(idx))
        {
            s_data.Target[i] = 0.0f;
            s_data.Now[i] = 0.0f;
            continue;
        }

        axis_count++;
        s_data.Now[i] = motor_task_get_now_pos(idx);

        if (s_cmd.Gimbal_Enable)
        {
            s_data.Target[i] = target[i];
        }
        else
        {
            s_data.Target[i] = s_data.Now[i];       /* 不使能:锁在当前角(别让它自由倒) */
        }

        (void)motor_task_set_mit(idx, s_data.Target[i], 0.0f, 0.0f, GIMBAL_KP, GIMBAL_KD);
    }

    /* ⑥ 发布 */
    s_data.Imu_Yaw_Total = imu.Yaw_Total;
    s_data.Imu_Pitch = imu.Euler[1];
    s_data.Kp = GIMBAL_KP;
    s_data.Kd = GIMBAL_KD;
    s_data.Axis_Count = axis_count;
    s_data.Enable = s_cmd.Gimbal_Enable;
    s_data.Online = imu.Online;
    (void)Msg_Publish(MSG_TOPIC_GIMBAL, &s_data, (uint16_t)sizeof(s_data));

    /* ⑦ 写进 error_task 的关键数据快照(它按 10Hz 落盘) */
    snap = Error_Get_Snapshot();
    snap->Gimbal_Cmd[0] = s_cmd.Gimbal_Yaw;
    snap->Gimbal_Cmd[1] = s_cmd.Gimbal_Pitch;
    snap->Gimbal_Now[0] = s_data.Now[GIMBAL_AXIS_YAW];
    snap->Gimbal_Now[1] = s_data.Now[GIMBAL_AXIS_PITCH];
}

const Gimbal_Data_t *Gimbal_Get(void)
{
    return &s_data;
}

bool Gimbal_Is_Online(void)
{
    return s_data.Online;
}

float Gimbal_Get_Target(uint8_t __axis)
{
    if (__axis >= GIMBAL_AXIS_N)
    {
        return 0.0f;
    }
    return s_data.Target[__axis];
}
