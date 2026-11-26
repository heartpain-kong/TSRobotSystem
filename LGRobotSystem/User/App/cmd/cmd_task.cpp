//
// cmd_task.cpp — 指令任务:遥控器/键鼠 + 裁判系统 + 上位机 → 统一控制指令
// 说明 / 用法 / 坑 见同目录 cmd_task.md
//

#include "cmd_task.h"
#include "cmd_485.h"

#include "dev_remote.h"             /* 遥控器:杆量/拨杆/键鼠 */
#include "dev_minipc.h"          /* miniPC(USB CDC):上位机/视觉经它转发 */
#include "dev_referee.h"        /* 裁判系统:比赛状态/功率上限 */
#include "msg_center.h"         /* 环形数组:收 RC/REFEREE/HOST,发 CMD */
#include "robot_def.h"          /* ROBOT_RUN_MODE:比赛/练习 */

#include <string.h>
#include <math.h>

static_assert(sizeof(Cmd_Data_t) <= MSG_SLOT_SIZE, "Cmd_Data_t 超过 MSG_SLOT_SIZE,会被环形数组静默拒收");

/* ============================================================================
 * 可调参数 —— 数值全部在 robot_def.h 的「本机配置」段
 * ========================================================================== */

#define CMD_MAX_VX          CFG_MAX_VX     /* 底盘前后最大速度(m/s) */
#define CMD_MAX_VY          CFG_MAX_VY     /* 底盘左右最大速度(m/s) */
#define CMD_MAX_W           CFG_MAX_W      /* 底盘最大旋转角速度(rad/s) */
#define CMD_SPIN_XY_SCALE   0.20f       /* 小陀螺时 xy 只给 20% */
#define CMD_SPIN_W_SCALE    0.75f       /* 小陀螺时 w 给 75% */
#define CMD_SPIN_XY_MAX     0.60f       /* 小陀螺 xy 合成速度上限(m/s),防解算发散 */
#define CMD_POWER_LIMIT     CFG_POWER_LIMIT   /* 底盘功率上限(W) */

/* 遥控器方向规定(只改这三个符号即可反方向) */
#define RC_SIGN_VX          (+1.0f)     /* 左杆右推 = +x(右移) */
#define RC_SIGN_VY          (-1.0f)     /* 左杆上推 = +y(前进),实测反向 */
#define RC_SIGN_W           (-1.0f)     /* 右杆右推 = 顺时针 */
#define CMD_DEAD_ZONE       0.05f       /* 摇杆死区(归一化) */
#define CMD_GIMBAL_SENS     0.0016f     /* 鼠标灵敏度:rad / 每个鼠标计数 */
#define CMD_PITCH_MIN       (-0.60f)    /* 云台 pitch 下限(rad) */
#define CMD_PITCH_MAX       (0.50f)     /* 云台 pitch 上限(rad) */

/* ============================================================================
 * 内部状态
 * ========================================================================== */

static Cmd_Data_t s_cmd;
static uint32_t s_minipc_seq = 0u;     /* 上位机指令序号(判断有没有新指令) */

/* ============================================================================
 * 小工具
 * ========================================================================== */

static float Apply_Dead_Zone(float __value, float __dead_zone)
{
    if ((__value < __dead_zone) && (__value > -__dead_zone))
    {
        return 0.0f;
    }
    return __value;
}

static float Clamp_Float(float __value, float __min, float __max)
{
    if (__value > __max)
    {
        return __max;
    }
    if (__value < __min)
    {
        return __min;
    }
    return __value;
}

/* ============================================================================
 * 上电清零(安全态)
 * ========================================================================== */

static void Cmd_Set_Idle(void)
{
    uint32_t seq = s_cmd.Seq;

    memset(&s_cmd, 0, sizeof(s_cmd));
    s_cmd.Seq = seq;
    s_cmd.Mode = CMD_MODE_IDLE;
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

void Cmd_Init(void)
{
    memset(&s_cmd, 0, sizeof(s_cmd));
    s_cmd.Mode = CMD_MODE_IDLE;
    Cmd_485_Init();
}

void Cmd_Task(void)
{
    Remote_Data_t rc;
    Referee_Data_t ref;
    uint32_t seq;
    MiniPC_Data_t host;           /* 上位机指令(USB CDC) */
    bool rc_ok;
    bool ref_ok;

#ifndef ROBOT_UPPER_ONLY
    /* 下 MCU 不接遥控器: CAN3 指令是唯一驾驶来源。
       裁判系统也只接在上板 USART1 ⇒ 功率上限等从 485 拿 */
    {
        float cvx, cvy, cw; uint8_t cmode;
        if (Cmd_485_Get_Chassis(&cvx, &cvy, &cw, &cmode))
        {
            s_cmd.Mode = CMD_MODE_REMOTE;
            s_cmd.Chassis_Vx = cvx;
            s_cmd.Chassis_Vy = cvy;
            s_cmd.Chassis_W  = cw;
            s_cmd.Chassis_Spin = (cmode & 0x02u) != 0u;

            /* 裁判(上板收)给的功率上限;0/超时 ⇒ 练习模式保守值 */
            {
                uint16_t pl = 0u;
                const uint16_t limit = Cmd_485_Get_Power_Limit(&pl) ? pl : PRACTICE_POWER_LIMIT;
                s_cmd.Chassis_Power_Limit = (limit > CMD_POWER_LIMIT) ? CMD_POWER_LIMIT : limit;
            }

            s_cmd.Seq++;
            (void)Msg_Publish(MSG_TOPIC_CMD, &s_cmd, (uint16_t)sizeof(s_cmd));
            Cmd_485_Task();
            return;
        }
    }
#endif

    /* ① 推进两个输入设备(掉线检测 + 有新帧就发布到各自话题) */
    Remote_Task();
    Referee_Task();
    MiniPC_Task();    /* miniPC:组帧/校验/掉线检测 + 有新帧就发布到 MSG_TOPIC_MINIPC */

    /* ② 从环形数组取最新数据(广播型:取最新,不弹出) */
    rc_ok = Msg_Latest(MSG_TOPIC_RC, &rc, (uint16_t)sizeof(rc), &seq);
    if (!rc_ok)
    {
        rc = *Remote_Get();         /* 还没发布过(刚上电):直接用当前值 */
    }

    ref_ok = Msg_Latest(MSG_TOPIC_REFEREE, &ref, (uint16_t)sizeof(ref), &seq);
    if (!ref_ok)
    {
        ref = *Referee_Get();
    }

    s_cmd.Seq++;
    s_cmd.Src = 0u;
    s_cmd.Referee_Online = ref.Online;
#if (ROBOT_RUN_MODE == ROBOT_RUN_COMPETITION)
    /* 比赛:功率和发弹完全听裁判 */
    s_cmd.Chassis_Power_Limit = (ref.Chassis_Power_Limit != 0u)
                              ? ref.Chassis_Power_Limit
                              : PRACTICE_POWER_LIMIT;   /* 裁判掉线也保守限 120W */
    s_cmd.Referee_Shoot_Allowed = ref.Online && Referee_Is_In_Match();
#else
    /* 练习:裁判不在线也允许调;没有裁判功率就用地板值保护 */
    s_cmd.Chassis_Power_Limit = (ref.Chassis_Power_Limit != 0u)
                              ? ref.Chassis_Power_Limit
                              : PRACTICE_POWER_LIMIT;
    s_cmd.Referee_Shoot_Allowed = true;
#endif
    if (s_cmd.Chassis_Power_Limit == 0u ||
        s_cmd.Chassis_Power_Limit > CMD_POWER_LIMIT)
    {
        s_cmd.Chassis_Power_Limit = CMD_POWER_LIMIT;
    }
    s_cmd.Chassis_Power_Buffer = ref.Chassis_Power_Buffer;
    s_cmd.Referee_Heat = ref.Heat;
    s_cmd.Referee_Heat_Limit = ref.Heat_Limit;

    /* ④ 模式:遥控器掉线 ⇒ IDLE(安全第一) */
    if ((!rc_ok) || (rc.Lost))
    {
        Cmd_Set_Idle();
        (void)Msg_Publish(MSG_TOPIC_CMD, &s_cmd, (uint16_t)sizeof(s_cmd));
        Cmd_485_Task();        /* 掉线也要让链路模块跑一拍(上板发全 0,下板刷新反馈缓存) */
        return;
    }

    s_cmd.Src |= CMD_SRC_RC;

    switch (rc.Switch[0])       /* 右拨杆 S1:1 上 = 遥控, 2 中 = 自动, 3 下 = 空闲 */
    {
    case 1u:
        s_cmd.Mode = CMD_MODE_REMOTE;
        break;
    case 2u:
        s_cmd.Mode = CMD_MODE_AUTO;
        break;
    default:
        Cmd_Set_Idle();
        (void)Msg_Publish(MSG_TOPIC_CMD, &s_cmd, (uint16_t)sizeof(s_cmd));
        Cmd_485_Task();        /* 空闲档位同样要让链路模块跑一拍 */
        return;
    }

    /* ⑤ 底盘:按 chaiss 参考
       左杆水平=Ch2(Vx), 左杆竖直=Ch3(Vy), 右杆水平=Ch0(W) */
    s_cmd.Chassis_Vx = RC_SIGN_VX * Apply_Dead_Zone(Remote_Get_Ch_Norm(2u), CMD_DEAD_ZONE) * CMD_MAX_VX;
    s_cmd.Chassis_Vy = RC_SIGN_VY * Apply_Dead_Zone(Remote_Get_Ch_Norm(3u), CMD_DEAD_ZONE) * CMD_MAX_VY;
    s_cmd.Chassis_W  = RC_SIGN_W  * Apply_Dead_Zone(Remote_Get_Ch_Norm(0u), CMD_DEAD_ZONE) * CMD_MAX_W;

    /* 小陀螺:左拨杆 S2 上;或键盘 R */
    s_cmd.Chassis_Spin = (rc.Switch[1] == 1u) || Remote_Key_Pressed(RC_KEY_R);

    /* 小陀螺时: W 占 75%, XY 占 20%;再限 xy 合成速度,避免逆解发散 */
    if (s_cmd.Chassis_Spin)
    {
        float vx = s_cmd.Chassis_Vx * CMD_SPIN_XY_SCALE;
        float vy = s_cmd.Chassis_Vy * CMD_SPIN_XY_SCALE;
        float mag = sqrtf(vx * vx + vy * vy);

        if (mag > CMD_SPIN_XY_MAX && mag > 0.0001f)
        {
            const float k = CMD_SPIN_XY_MAX / mag;
            vx *= k;
            vy *= k;
        }
        s_cmd.Chassis_Vx = vx;
        s_cmd.Chassis_Vy = vy;
        s_cmd.Chassis_W *= CMD_SPIN_W_SCALE;
    }

    /* ⑥ 云台:鼠标增量式(相对角累加),键盘 F 归中 */
    if (Remote_Key_Pressed(RC_KEY_F))
    {
        s_cmd.Gimbal_Yaw = 0.0f;
        s_cmd.Gimbal_Pitch = 0.0f;
    }
    else
    {
        s_cmd.Gimbal_Yaw += (float)rc.Mouse_X * CMD_GIMBAL_SENS;
        s_cmd.Gimbal_Pitch += (float)rc.Mouse_Y * CMD_GIMBAL_SENS;
    }
    s_cmd.Gimbal_Pitch = Clamp_Float(s_cmd.Gimbal_Pitch, CMD_PITCH_MIN, CMD_PITCH_MAX);
    s_cmd.Gimbal_Enable = (rc.Switch[1] != 3u);     /* S2 打到最下 = 云台不使能(调试/搬运) */

    /* ⑦ 发射:Shift = 摩擦轮,Q = 单发,E = 连发;非比赛期禁止 */
    if (Remote_Key_Pressed(RC_KEY_SHIFT) || Remote_Key_Pressed(RC_KEY_Q) || Remote_Key_Pressed(RC_KEY_E))
    {
        s_cmd.Src |= CMD_SRC_KEYBOARD;
    }
    s_cmd.Shoot_Enable = Remote_Key_Pressed(RC_KEY_SHIFT) && s_cmd.Referee_Shoot_Allowed;
    if (Remote_Key_Pressed(RC_KEY_Q))
    {
        s_cmd.Shoot_Rate = 1u;
    }
    else if (Remote_Key_Pressed(RC_KEY_E))
    {
        s_cmd.Shoot_Rate = 2u;
    }
    else
    {
        s_cmd.Shoot_Rate = 0u;
    }
    if (!s_cmd.Referee_Shoot_Allowed)
    {
        s_cmd.Shoot_Rate = 0u;
    }

    /* ★ 上板拨杆 S1=3 的发射指令经 485 下来:上板按 1000rpm 摩擦轮 + 100rpm 拨盘发
          (原来下板只认键盘 Q/E/Shift,所以 S1=3 时拨盘不动) */
    {
        uint8_t up_en = 0u, up_rate = 0u;
        if (Cmd_485_Get_Shoot(&up_en, &up_rate))
        {
            if (up_en != 0u)
            {
                s_cmd.Shoot_Enable = s_cmd.Referee_Shoot_Allowed;
                s_cmd.Shoot_Rate   = s_cmd.Referee_Shoot_Allowed
                                   ? ((up_rate != 0u) ? up_rate : 2u) : 0u;
            }
        }
    }

    /* ⑧ 上位机(USB CDC):自动模式下,它的指令覆盖底盘/云台/发射目标 */
    if (Msg_Is_New(MSG_TOPIC_MINIPC, s_minipc_seq) &&
        Msg_Latest(MSG_TOPIC_MINIPC, &host, (uint16_t)sizeof(host), &s_minipc_seq))
    {
        s_cmd.Src |= CMD_SRC_HOST;
        if ((s_cmd.Mode == CMD_MODE_AUTO) && host.Online)
        {
            s_cmd.Chassis_Vx = host.Cmd.Chassis_Vx;
            s_cmd.Chassis_Vy = host.Cmd.Chassis_Vy;
            s_cmd.Chassis_W = host.Cmd.Chassis_W;
            s_cmd.Gimbal_Yaw = host.Cmd.Gimbal_Yaw;
            s_cmd.Gimbal_Pitch = host.Cmd.Gimbal_Pitch;
            s_cmd.Shoot_Enable = (host.Cmd.Shoot_Enable != 0u) && s_cmd.Referee_Shoot_Allowed;
            s_cmd.Shoot_Rate = s_cmd.Referee_Shoot_Allowed ? host.Cmd.Shoot_Rate : 0u;
        }
        else if (s_cmd.Mode == CMD_MODE_AUTO)
        {
            /* miniPC 掉线:不能继续执行最后一帧自动指令 */
            s_cmd.Chassis_Vx = 0.0f;
            s_cmd.Chassis_Vy = 0.0f;
            s_cmd.Chassis_W = 0.0f;
            s_cmd.Shoot_Enable = false;
            s_cmd.Shoot_Rate = 0u;
        }
    }
    if (ref.Online)
    {
        s_cmd.Src |= CMD_SRC_REFEREE;
    }

    /* ⑨ 发出去(广播型:谁要用谁 Msg_Latest) */
#ifndef ROBOT_UPPER_ONLY
    {
        float cvx, cvy, cw; uint8_t cmode;
        if (Cmd_485_Get_Chassis(&cvx, &cvy, &cw, &cmode))
        {
            s_cmd.Mode = CMD_MODE_REMOTE;
            s_cmd.Chassis_Vx = cvx;
            s_cmd.Chassis_Vy = cvy;
            s_cmd.Chassis_W  = cw;
            s_cmd.Chassis_Spin = (cmode & 0x02u) != 0u;
        }
    }
#endif
    (void)Msg_Publish(MSG_TOPIC_CMD, &s_cmd, (uint16_t)sizeof(s_cmd));
    Cmd_485_Task();
}

const Cmd_Data_t *Cmd_Get(void)
{
    return &s_cmd;
}

Enum_Cmd_Mode Cmd_Get_Mode(void)
{
    return (Enum_Cmd_Mode)s_cmd.Mode;
}

uint32_t Cmd_Get_Seq(void)
{
    return s_cmd.Seq;
}

const char *Cmd_Get_Mode_Name(Enum_Cmd_Mode __mode)
{
    switch (__mode)
    {
    case CMD_MODE_REMOTE:   return "REMOTE";
    case CMD_MODE_AUTO:     return "AUTO";
    case CMD_MODE_TEST:     return "TEST";
    default:                return "IDLE";
    }
}
