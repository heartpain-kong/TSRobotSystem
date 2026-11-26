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
#include <math.h>

static_assert(sizeof(Gimbal_Data_t) <= MSG_SLOT_SIZE, "Gimbal_Data_t 超过 MSG_SLOT_SIZE");

/* ============================================================================
 * 参数(★ 按实车改)
 * ========================================================================== */

#define GIMBAL_YAW_MIN          (-2.967f)   /* yaw 关节限位 ±170° */
#define GIMBAL_YAW_MAX          (2.967f)
#define GIMBAL_PITCH_MIN        (-0.53f)    /* pitch 关节限位 */
#define GIMBAL_PITCH_MAX        (0.53f)
#define GIMBAL_KP               25.0f       /* MIT 刚度(位置误差 → 力矩) */
#define GIMBAL_KD               1.0f        /* MIT 阻尼 */
#define GIMBAL_SPIN_COMP_GAIN   1.0f
/* pitch 4310 前馈: T_ff = Tmax*cos(theta) - B*omega
   Tmax = m*g*L, 先标定 Tmax; 默认 0 不影响位置环 */
#define GIMBAL_PITCH_TMAX       (-3.95f)   /* m*g*L: 自动标定 2026-11-25(上电自标, 多次实测 -3.91~-4.18, 取中值) */
#define GIMBAL_PITCH_DAMP       0.0f        /* 小陀螺前馈增益:1 = 完全反向补偿,0 = 关 */

/* ============================================================================
 * 内部状态
 * ========================================================================== */

static float s_pitch_tmax = GIMBAL_PITCH_TMAX;
/* 标定采样缓存(供读回分析): [k]=pos/rad, t/N·m */
static float g_cal_pos[8] = {0};
static float g_cal_t[8]   = {0};
static bool  s_cal_force = false;
/* 前馈模型: T = c0 + c1*cos + c2*sin + c3*cos2 + c4*sin2 (重力+弹簧, 非线性) */
static float g_ff_c[5] = {3.771f, -5.977f, -21.09f, 2.112f, 9.728f};   /* 外部采集+最小二乘重新拟合(rms 0.13), 固定 */
static bool  g_ff_ok = true;
static float s_pitch_off = 0.0f;      /* 上电锁存的 pitch 零偏(joint = pos - off) */
static bool  s_pitch_off_ok = false;

static Gimbal_Data_t s_data;
static Cmd_Data_t s_cmd;
static uint32_t s_cmd_seq = 0u;
static uint32_t s_imu_seq = 0u;
static uint32_t s_chassis_seq = 0u;
static uint32_t s_last_us = 0u;
static bool s_inited = false;
static bool s_pitch_zero_done = false;

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
    s_pitch_zero_done = false;
    s_inited = true;
}

void Gimbal_Set_Pitch_Tmax(float __tmax)
{
    s_pitch_tmax = __tmax;
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

    /* 4310 在机械 0 时,把上 MCU IMU pitch 对齐到 0 */
    /* 上电锁存 pitch 零偏: 云台 IMU 收敛后, 把当前 4310 位置当作关节 0 */
    if (!s_pitch_off_ok && motor_task_4310_online() &&
        Imu_Is_Converged(IMU_CHANNEL_GIMBAL))
    {
        s_pitch_off = 0.0f;   /* 双编码器:零点固定,不做动态零偏锁存 */
        s_pitch_off_ok = true;
        Imu_Zero_Pitch_Now();
        s_pitch_zero_done = true;
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
    /* ---- pitch 重力前馈自动标定 ----
       上电 2s 后自动执行一次:
         8 个角度各停留 1.2s, 每点采样 5 次取平均(取 KP*(pos-目标) 作为保持力矩),
         最小二乘拟合 t = Tmax*cos(theta), 结果写入 s_pitch_tmax */
    {
        static uint8_t  s_cal_state = 0u;   /* 0=等启动 1=扫角 2=完成 */
        static uint8_t  s_cal_idx = 0u;
        static uint8_t  s_cal_sub = 0u;
        static uint32_t s_cal_ms = 0u;
        static uint32_t s_cal_boot = 0u;
        static float    s_cal_set[8];
        static float    s_cal_pos[8];
        static float    s_cal_t[8];
        static float    s_cal_sum = 0.0f;

        const uint32_t now_ms = (uint32_t)Timestamp_Get_Ms();
        const float now_pos = motor_task_4310_pos() - s_pitch_off;


        if (s_cal_boot == 0u) { s_cal_boot = now_ms; }

        if (0)   /* 已固定拟合结果, 不再上电标定 */
        {
            s_cal_set[0] = 0.0f;   s_cal_set[1] =  0.2f;  s_cal_set[2] =  0.4f;  s_cal_set[3] =  0.53f;
            s_cal_set[4] = 0.0f;   s_cal_set[5] = -0.2f;  s_cal_set[6] = -0.4f;  s_cal_set[7] = -0.53f;
            s_cal_idx = 0u; s_cal_sub = 0u; s_cal_sum = 0.0f;
            s_cal_ms = now_ms;
            s_cal_state = 1u;
            s_cal_force = true;
        }

        if (s_cal_state == 1u)
        {
            target[GIMBAL_AXIS_PITCH] = s_cal_set[s_cal_idx];   /* 下面发送时统一加 off */

            /* 每点: 先静置 1200ms, 再 5 次采样(40ms 间隔)平均 */
            if ((uint32_t)(now_ms - s_cal_ms) > 1200u)
            {
                s_cal_sum += GIMBAL_KP * (now_pos - s_cal_set[s_cal_idx]);
                s_cal_sub++;

                if (s_cal_sub >= 5u)
                {
                    s_cal_pos[s_cal_idx] = now_pos;
                    g_cal_pos[s_cal_idx]  = now_pos;
                    s_cal_t[s_cal_idx]   = s_cal_sum / 5.0f;
                    g_cal_t[s_cal_idx]   = s_cal_t[s_cal_idx];
                    s_cal_idx++;
                    s_cal_sub = 0u;
                    s_cal_sum = 0.0f;
                    s_cal_ms = now_ms;

                    if (s_cal_idx >= 8u)
                    {
                        /* 5 项傅里叶基最小二乘: ATA*C = ATy */
                        float A[5][6] = {{0}};

                        for (uint8_t k = 0u; k < 8u; k++)
                        {
                            const float th = s_cal_pos[k];
                            const float c1 = cosf(th), s1 = sinf(th);
                            const float phi[5] = {1.0f, c1, s1,
                                                  2.0f * c1 * c1 - 1.0f,
                                                  2.0f * s1 * c1};

                            for (uint8_t r = 0u; r < 5u; r++)
                            {
                                for (uint8_t q = 0u; q < 5u; q++)
                                {
                                    A[r][q] += phi[r] * phi[q];
                                }
                                A[r][5] += phi[r] * s_cal_t[k];
                            }
                        }

                        /* 高斯消元(5x5, 带部分主元) */
                        for (uint8_t col = 0u; col < 5u; col++)
                        {
                            uint8_t piv = col;
                            for (uint8_t r = (uint8_t)(col + 1u); r < 5u; r++)
                            {
                                if (fabsf(A[r][col]) > fabsf(A[piv][col])) { piv = r; }
                            }
                            if (piv != col)
                            {
                                for (uint8_t q = 0u; q < 6u; q++)
                                {
                                    const float tmp = A[col][q];
                                    A[col][q] = A[piv][q];
                                    A[piv][q] = tmp;
                                }
                            }
                            if (fabsf(A[col][col]) < 1.0e-9f) { continue; }
                            for (uint8_t r = (uint8_t)(col + 1u); r < 5u; r++)
                            {
                                const float f = A[r][col] / A[col][col];
                                for (uint8_t q = col; q < 6u; q++)
                                {
                                    A[r][q] -= f * A[col][q];
                                }
                            }
                        }
                        for (int8_t r = 4; r >= 0; r--)
                        {
                            float v = A[r][5];
                            for (uint8_t q = (uint8_t)(r + 1); q < 5u; q++) { v -= A[r][q] * g_ff_c[q]; }
                            g_ff_c[r] = (fabsf(A[r][r]) > 1.0e-9f) ? (v / A[r][r]) : 0.0f;
                        }
                        g_ff_ok = true;
                        s_pitch_tmax = g_ff_c[1];      /* 兼容旧字段 */
                        s_cal_state = 2u;
                        s_cal_force = false;
                    }
                }
            }
        }
    }



    /* ⑤ 写电机(MIT **位置**控制:关节侧 rad,和电机接口同单位 ⇒ 不需要换算) */
    for (i = 0u; i < GIMBAL_AXIS_N; i++)
    {
        uint8_t idx = (i == GIMBAL_AXIS_YAW) ? GIMBAL_MOTOR_YAW : GIMBAL_MOTOR_PITCH;

        if (idx == 0xFEu)
        {
            /* DM4310 pitch: 位置 + 重力/阻尼前馈 */
            float now = motor_task_4310_pos() - s_pitch_off;
            float tgt = (s_cmd.Gimbal_Enable || s_cal_force) ? (target[i] + s_pitch_off) : now;
            float t_ff;
            if (s_cal_force)
            {
                t_ff = 0.0f;                      /* 标定期间前馈置 0, 测纯重力/弹簧 */
            }
            else if (g_ff_ok)
            {
                const float c1 = cosf(now), s1 = sinf(now);
                t_ff = g_ff_c[0] + g_ff_c[1] * c1 + g_ff_c[2] * s1
                     + g_ff_c[3] * (2.0f * c1 * c1 - 1.0f) + g_ff_c[4] * (2.0f * s1 * c1);
            }
            else
            {
                t_ff = s_pitch_tmax * cosf(now);
            }
            t_ff -= GIMBAL_PITCH_DAMP * imu.Gyro[1];
            axis_count++;
            s_data.Now[i] = now;
            s_data.Target[i] = tgt;
            motor_task_4310_set_pitch(tgt, GIMBAL_KP, GIMBAL_KD, t_ff);
            continue;
        }

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
