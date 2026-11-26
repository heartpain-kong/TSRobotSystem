//
// motor_task.cpp — 电机子系统(C++):实例 + 初始化 + 接收分发 + “跑一拍”的 motor_task()
//
// 分层:
//   · 本文件:决定“有哪些电机 / 谁接收 / 何时发 / 单位换算”
//   · 驱动类:协议组帧、解析、PID(大疆的主机 PID 也在类里)
//   · 机器人层:创建 MOTOR 任务,按 robot_def.h 的周期调 motor_task()
//

#include "motor_task.h"
#include "cmd_485.h"
#include <string.h>
#include <stddef.h>
#include "crc_ccitt.h"

#include "robot_def.h"

/* ============================================================================
 * 一、配置
 * ========================================================================== */

/* ★ 台架测试:只接 CAN1 0x206 的 3508 拨盘 */
#define MOTOR_PAIR2_ENABLE      1   /* 202 调试阶段关闭 203/204 */
#define MOTOR_TASK_ENABLE       1

/* 0x206:3508 拨盘 */
#define TRIGGER_MOTOR_CAN_ID    0x202u   /* 第一组舵向 */
#define DRIVE_MOTOR_CAN_ID      0x201u   /* 第一组驱动 */
#define TRIGGER_MOTOR2_CAN_ID   0x204u   /* 第二组舵向:204 */
#define DRIVE_MOTOR2_CAN_ID     0x203u   /* 第二组驱动:203 */

/* ★ 当前只调速度环:0 = 关闭位置环/舵向角度模式。
   关闭后:
     · motor_task() 强制 motor_speed,不再跑 PID_Angle;
     · motor_task_set_steer_rad() 返回 false;
     · 0x5C 的“发送角度”命令被忽略;
     · 0x5C 的 PID 只写速度环 Kp/Ki/Kd(位置环三个值忽略)。
   要重新启用舵向位置环,改成 1 即可。 */
#define MOTOR_POS_LOOP_ENABLE   1   /* auto: speed cmd -> speed loop, angle cmd -> position loop */

/* 达妙(当前未接):反馈帧的 CAN id 是共用 Master ID,
   真正的电机身份在**数据第 1 字节的低 4 位**(高 4 位是 ERR) */
#define DM_MOTOR_N              0

#define MOTOR_DM_ENABLE_ON_INIT 0u      /* 0: 不发 DM 使能帧(总线上没有 DM ⇒ 会打成 BUS-OFF) */
#define MOTOR_DM_ENABLE_WAIT_MS 20u     /* 使能后等反馈的时长(反馈里带当前位置) */
#define MOTOR_DM_KP             30.0f   /* MIT 模式刚度:位置误差 → 力矩 */
#define MOTOR_DM_KD             1.0f    /* MIT 模式阻尼 */

/* 在线判定:100ms 没反馈 = 离线 */
#define MOTOR_ONLINE_TIMEOUT_US 100000u
#define STEER_FEEDBACK_MAX_AGE_US 10000u  /* 底盘选弧最多使用 10 ms 内的舵角 */

/* 大疆舵向 3508:
   · 台架裸电机(屁股直接看):1.0
   · 舵轮外部 1:8 减速:8.0
   PID 的 Target/Now 统一在舵轮输出侧,换算都在 Class_Motor_DJ 内部完成。 */
#define DJ_GEAR_RATIO           8.0f
/* 0x206 拨盘 3508 台架参数 */
/* ============================================================================
 * 四台电机 PID 统一表
 *   下标: 0=0x202 舵向, 1=0x201 驱动, 2=0x204 舵向, 3=0x203 驱动
 *   每台独立,修改时直接改这张表即可;运行中由调试命令回写。
 * ========================================================================== */
typedef struct
{
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float pos_kp;
    float pos_ki;
    float pos_kd;
    float speed_max_out;
    float speed_max_iout;
    float pos_max_w;
} MotorPidCfg;

enum
{
    PID_IDX_202_STEER = 0,
    PID_IDX_201_DRIVE,
    PID_IDX_204_STEER,
    PID_IDX_203_DRIVE,
    PID_IDX_207_FRICTION_L,
    PID_IDX_208_FRICTION_R,
    PID_IDX_206_TRIGGER,
    PID_IDX_6020_YAW,
    PID_IDX_4310_PITCH,
    PID_IDX_COUNT
};

static MotorPidCfg s_pid_cfg[PID_IDX_COUNT] = {
    /* 0: 202 舵向 */
    {0.10f, 0.010f, 0.0f, 20.0f, 0.0f, 0.1f,   2.5f, 0.5f, 70.0f},
    /* 1: 201 驱动 */
    {1.2f, 0.05f, 0.0f, 0.0f, 0.0f, 0.0f,  4.5f, 1.0f,  0.0f},
    /* 2: 204 舵向 */
    {0.10f, 0.010f, 0.0f, 20.0f, 0.0f, 0.1f,   2.5f, 0.5f, 70.0f},
    /* 3: 203 驱动 */
    {1.2f, 0.05f, 0.0f, 0.0f, 0.0f, 0.0f,  4.5f, 1.0f,  0.0f},
    /* 4: 207 左摩擦轮 */
    {0.005f, 0.0001f, 0.0f, 0.0f, 0.0f, 0.0f,  0.5f, 0.2f,  0.0f},
    /* 5: 208 右摩擦轮 */
    {0.005f, 0.0001f, 0.0f, 0.0f, 0.0f, 0.0f,  0.5f, 0.2f,  0.0f},
    /* 6: 206 拨盘 3508:只做速度环 */
    {1.0f, 0.01f, 0.0f,   0.0f, 0.0f, 0.0f,    4.5f, 2.0f, 0.0f},
    /* 7: 6020 yaw
       speed: kp/ki/kd + 输出/积分限幅
       pos  : Kp + 输出速度上限(pos_max_w)
       ★ 位置环的阻尼不用这里的 pos_kd(它是"每拍"增益,1kHz 下≈0),
         而是用 Set_Pos_Vel_Damp() 直接对反馈转速做阻尼(见 motor_DJ)。 */
    {0.10f, 0.001f, 0.0f, 70.0f, 0.0f, 0.0f,  2.0f, 1.5f, 80.0f},
    /* 8: 4310 pitch:只做速度环 */
    {0.1f, 0.01f, 0.0f,    0.0f, 0.0f, 0.0f,   4.5f, 2.0f,  0.0f},
};

/* ============================================================================
 * 二、依赖
 * ========================================================================== */

#include "math_support.h"       /* MATH_RPM_TO_RADPS:rpm ↔ rad/s 只在这里换算 */
#include "bsp_timestamp.h"      /* 反馈时刻 / 在线判定 */
#include "bsp_fdcan.h"          /* 每路 CAN 注册一个接收回调 */

#if MOTOR_485_ENABLE
#include "bsp_uart.h"
#include "motor_YS.h"
#endif

#include "motor_DJ.h"
#if (DM_MOTOR_N > 0)
#include "motor_DM.h"
#endif

/* ============================================================================
 * 三、实例
 * ========================================================================== */

#if (DM_MOTOR_N > 0)
static Class_Motor_DM dm_motor[DM_MOTOR_N];    /* 下标 i ↔ 电机 id (i+1) */
#endif

static Class_Motor_DJ steer_motor;   /* 0x202 第一组舵向 3508 */
static Class_Motor_DJ drive_motor;   /* 0x201 第一组驱动 3508 */
static Class_Motor_DJ steer_motor2;  /* 0x203 第二组舵向 3508 */
static Class_Motor_DJ drive_motor2;  /* 0x204 第二组驱动 3508 */
static Class_Motor_DJ friction_l;              /* 0x207 左摩擦轮 3508 屁股 */
static Class_Motor_DJ friction_r;              /* 0x208 右摩擦轮 3508 屁股 */

/* 0x1FF 组: 0x206 拨盘 3508 + 0x205 6020 yaw,速度环独立对象 */
#define AUX_TRIGGER_CAN_ID      0x206u
#define AUX_6020_CAN_ID         0x205u
/* 6020 编码器零点:raw 值对应"云台指向 +Y(车头正前)"的位置。
   数值在 robot_def.h:CFG_6020_ZERO_ENC
   标定记录:1792 → 2816 → 768 → 882 → 1110 → 996 → 1210 → 550(用户现场定)。 */
#define AUX_6020_ZERO_ENC       CFG_6020_ZERO_ENC
static Class_Motor_DJ_Group aux_group;
static volatile float g_aux_target_rpm[2] = {0.0f, 0.0f};
static volatile float g_aux_target_rad[2] = {0.0f, 0.0f};
static bool s_6020_zero_pending = true;
/* ★ 调试波形:6020 yaw 每 10ms 记一次 [pos, target, w_cmd, err],200 点 = 2s */
float g_dbg_steer[6][200];
volatile uint32_t g_dbg_steer_idx = 0u;

float g_dbg_dial[4][200];
volatile uint32_t g_dbg_dial_idx = 0u;

float g_dbg_6020_wave[4][200];
volatile uint32_t g_dbg_6020_idx = 0u;
static volatile uint32_t can3_rx_count = 0u;
static volatile uint32_t can3_last_id = 0u;
static volatile uint32_t can3_last_ms = 0u;
static volatile int16_t  can3_vx_mm_s = 0;
static volatile int16_t  can3_vy_mm_s = 0;
static volatile int16_t  can3_w_mrad_s = 0;
static volatile uint8_t  can3_mode = 0u;
static volatile uint32_t friction_rx_us[2] = {0u, 0u};
static volatile float    g_dj_target_rpm = 0.0f;   /* 速度目标 [rpm],上电默认 0 */
static volatile float    g_dj_target_rad = 0.0f;   /* 位置目标 [rad] */
static volatile uint32_t dj_rx_us = 0u;static volatile uint32_t drive_rx_us = 0u;
static volatile uint32_t dj_rx_us2 = 0u;
static volatile uint32_t drive_rx_us2 = 0u;
static volatile float    g_dj_target_rpm2 = 0.0f;
static volatile float    g_dj_target_rad2 = 0.0f;
static volatile uint32_t can_rx_us[11] = {0u};   /* 0x201..0x20B scan */            /* 最近一次反馈时刻 [us] */
static volatile bool     s_manual_override = false; /* 串口调试接管:底盘不许再写 0 号电机 */
static volatile uint32_t s_manual_override_ms = 0u;       /* 接管时刻(1s 没新命令自动交还底盘) */

static void s_manual_override_take(void)
{
    s_manual_override = true;
    s_manual_override_ms = (uint32_t)Timestamp_Get_Ms();
}
volatile uint8_t dbg_init_step = 0u;               /* 初始化进度:1~6,6=完成 */

/* ============================================================================
 * 四、RS485 电机(宇树 YS)
 * ========================================================================== */
#if MOTOR_485_ENABLE
extern volatile bool motor_485_init_ok;     /* 定义在 robot.cpp */

struct Motor485Cfg
{
    UART_HandleTypeDef     *uart;
    uint8_t                 id;
    motor_YS_Model          model;
    motor_Control_mode      mode;
    Enum_Motor_Control_Freq freq;
};

static const Motor485Cfg s_485_cfg[] = {
    { &huart3, 0u, MOTOR_YS_M8010, motor_speed, Motor_Control_Freq_1000HZ },
};

#define MOTOR_485_NUM   ((uint8_t)(sizeof(s_485_cfg) / sizeof(s_485_cfg[0])))

static Class_Motor_YS s_ys[MOTOR_485_NUM];
static uint8_t s_485_inited = 0u;
static bool    s_485_port_done[2] = { false, false };   /* 0 = huart2, 1 = huart3 */

static uint8_t Motor485_Port_Slot(UART_HandleTypeDef *__h)
{
    if (__h == &huart2) { return 0u; }
    if (__h == &huart3) { return 1u; }
    return 0xFFu;
}

/* ISR 上下文:同一串口的电机都喂一遍(recv 自己按帧长校验) */
static void Motor485_Rx_Callback(UART_HandleTypeDef *__huart, uint8_t *__buf, uint16_t __size)
{
    if ((__buf == nullptr) || (__size == 0u))
    {
        return;
    }

    for (uint8_t i = 0u; i < s_485_inited; i++)
    {
        if (s_485_cfg[i].uart == __huart)
        {
            s_ys[i].recv(0u, __buf, (uint8_t)__size);
        }
    }
}
#endif /* MOTOR_485_ENABLE */

/* ============================================================================
 * 五、接收:每条 CAN 一个回调
 *   ★ 一条总线只挂一个品牌;回调在 ISR 上下文,只解析、不阻塞
 * ========================================================================== */
static void CAN1_Callback(FDCAN_RxHeaderTypeDef Header, uint8_t *Buffer)
{
    /* 大疆反馈 id 就是电机身份(0x201~0x20B) */
    if ((Header.Identifier >= CAN_M1_ID) && (Header.Identifier <= CAN_MB_ID))
    {
        if (Buffer == nullptr || Header.IdType != FDCAN_STANDARD_ID ||
            Header.RxFrameType != FDCAN_DATA_FRAME || Header.FDFormat != FDCAN_CLASSIC_CAN ||
            Header.DataLength != FDCAN_DLC_BYTES_8 ||
            ((static_cast<uint16_t>(Buffer[0]) << 8u) | Buffer[1]) >= Class_Motor_DJ::ENCODER_NUM_PER_ROUND)
        {
            return;
        }

        const uint32_t received_us = Timestamp_Get_Us32();
        can_rx_us[Header.Identifier - CAN_M1_ID] = received_us;
        if (Header.Identifier == steer_motor.Get_CAN_Id())
        {
            steer_motor.recv(Header.Identifier, Buffer, 8u);
            dj_rx_us = received_us;
        }
        else if (Header.Identifier == drive_motor.Get_CAN_Id())
        {
            drive_motor.recv(Header.Identifier, Buffer, 8u);
            drive_rx_us = received_us;
        }
        else if (Header.Identifier == steer_motor2.Get_CAN_Id())
        {
            steer_motor2.recv(Header.Identifier, Buffer, 8u);
            dj_rx_us2 = received_us;
        }
        else if (Header.Identifier == drive_motor2.Get_CAN_Id())
        {
            drive_motor2.recv(Header.Identifier, Buffer, 8u);
            drive_rx_us2 = received_us;
        }
        else if (Header.Identifier == AUX_TRIGGER_CAN_ID)
        {
            aux_group[0].recv(Header.Identifier, Buffer, 8u);
        }
        else if (Header.Identifier == AUX_6020_CAN_ID)
        {
            aux_group[1].recv(Header.Identifier, Buffer, 8u);
        }
        else if (Header.Identifier == 0x207u)
        {
            aux_group[2].recv(Header.Identifier, Buffer, 8u);
            friction_rx_us[0] = received_us;
        }
        else if (Header.Identifier == 0x208u)
        {
            aux_group[3].recv(Header.Identifier, Buffer, 8u);
            friction_rx_us[1] = received_us;
        }
        return;
    }

#if (DM_MOTOR_N > 0)
    if (Buffer == nullptr)
    {
        return;
    }

    const uint8_t motor_id = Buffer[0] & 0x0Fu;
    if ((motor_id >= 1u) && (motor_id <= DM_MOTOR_N))
    {
        dm_motor[motor_id - 1u].recv(Header.Identifier, Buffer, 8u);
    }
#endif
}

#if 0 /* CAN3 已移到 cmd_can3.cpp */
#endif

/* ============================================================================
 * 六、跑一拍
 * ========================================================================== */
/* ---- 1kHz 电机紧凑帧(经 485 上送,PC 串口调 PID 用) ---- */
static volatile bool s_fast_stream = false;
static uint16_t      s_fast_seq = 0u;
static uint8_t       s_fast_buf[64];

extern "C" void motor_task_set_fast_stream(uint8_t __on)
{
    s_fast_stream = (__on != 0u);
}

extern "C" void motor_task(void)
{
#if MOTOR_TASK_ENABLE
    /* 调试接管 1s 无新命令 ⇒ 自动交还底盘,避免 0x5C/小陀螺把底盘永久锁死 */
    if (s_manual_override &&
        ((uint32_t)(Timestamp_Get_Ms() - s_manual_override_ms) > 1000u))
    {
        s_manual_override = false;
    }
    /* ① 写目标:模式以电机类里的 Motor_Control_mode 为唯一真源
       (位置模式用 rad,速度模式用 rpm → rad/s) */
#if MOTOR_POS_LOOP_ENABLE
    if (!motor_task_is_online(0u))
    {
        /* 位置/速度环都别在反馈丢失时继续积分 */
        g_dj_target_rpm = 0.0f;
        if (steer_motor.Get_Send_Status() == Motor_Status_ENABLE)
        {
            steer_motor.lose();
        }
    }
    else if (steer_motor.Get_Motor_Control_mode() == motor_pos)
    {
        steer_motor.Set_Pos(g_dj_target_rad);
    }
    else
    {
        steer_motor.Set_W(g_dj_target_rpm * MATH_RPM_TO_RADPS);
    }
#else
    /* 只调速度环:任何 API 误切到 motor_pos 都强制拉回 motor_speed */
    if (!motor_task_is_online(0u))
    {
        /* 反馈掉了还继续积分 ⇒ 一恢复就猛冲;直接 lose,等重新给命令再 enable */
        g_dj_target_rpm = 0.0f;
        if (steer_motor.Get_Send_Status() == Motor_Status_ENABLE)
        {
            steer_motor.lose();
        }
    }
    else
    {
        if (steer_motor.Get_Motor_Control_mode() != motor_speed)
        {
            steer_motor.Set_Motor_Control_mode(motor_speed);
            steer_motor.PID_speed.Clear();
        }
        steer_motor.Set_W(g_dj_target_rpm * MATH_RPM_TO_RADPS);
    }
#endif

    /* ② 到点:算主机 PID → 发帧(发不发由电机自己的频率决定) */
    if (steer_motor.Is_Send_Due())
    {
        steer_motor.Calculate();
        steer_motor.send();
    }
    if (drive_motor.Is_Send_Due())
    {
        drive_motor.Calculate();
        drive_motor.send();
    }

#if MOTOR_PAIR2_ENABLE
#if MOTOR_POS_LOOP_ENABLE
    if (!motor_task_is_online(2u))
    {
        g_dj_target_rpm2 = 0.0f;
        if (steer_motor2.Get_Send_Status() == Motor_Status_ENABLE)
        {
            steer_motor2.lose();
        }
    }
    else if (steer_motor2.Get_Motor_Control_mode() == motor_pos)
    {
        steer_motor2.Set_Pos(g_dj_target_rad2);
    }
    else
    {
        steer_motor2.Set_W(g_dj_target_rpm2 * MATH_RPM_TO_RADPS);
    }
#else
    if (!motor_task_is_online(2u))
    {
        g_dj_target_rpm2 = 0.0f;
        if (steer_motor2.Get_Send_Status() == Motor_Status_ENABLE)
        {
            steer_motor2.lose();
        }
    }
    else
    {
        if (steer_motor2.Get_Motor_Control_mode() != motor_speed)
        {
            steer_motor2.Set_Motor_Control_mode(motor_speed);
            steer_motor2.PID_speed.Clear();
        }
        steer_motor2.Set_W(g_dj_target_rpm2 * MATH_RPM_TO_RADPS);
    }
#endif

    if (steer_motor2.Is_Send_Due())
    {
        steer_motor2.Calculate();
        steer_motor2.send();
    }
    if (drive_motor2.Is_Send_Due())
    {
        drive_motor2.Calculate();
        drive_motor2.send();
    }
#endif

    /* 舵向波形:两个舵向轮 pos/tgt/W */
    {
        static uint32_t s_steer_ms = 0u;
        const uint32_t sn = (uint32_t)Timestamp_Get_Ms();
        if ((uint32_t)(sn - s_steer_ms) >= 10u)
        {
            const uint32_t sk = g_dbg_steer_idx % 200u;
            s_steer_ms = sn;
            g_dbg_steer[0][sk] = steer_motor.Get_Now_pos();
            g_dbg_steer[1][sk] = steer_motor.Get_Send_Data().Pos;
            g_dbg_steer[2][sk] = steer_motor.Get_Now_W();
            g_dbg_steer[3][sk] = steer_motor2.Get_Now_pos();
            g_dbg_steer[4][sk] = steer_motor2.Get_Send_Data().Pos;
            g_dbg_steer[5][sk] = steer_motor2.Get_Now_W();
            g_dbg_steer_idx = (g_dbg_steer_idx + 1u) % 200u;
        }
    }

    /* 拨盘波形:目标/实测/输出/状态 */
    {
        static uint32_t s_dial_ms = 0u;
        const uint32_t dn = (uint32_t)Timestamp_Get_Ms();
        if ((uint32_t)(dn - s_dial_ms) >= 10u)
        {
            const uint32_t dk = g_dbg_dial_idx % 200u;
            s_dial_ms = dn;
            g_dbg_dial[0][dk] = aux_group[0].Get_Send_Data().W;
            g_dbg_dial[1][dk] = aux_group[0].Get_Now_W();
            g_dbg_dial[2][dk] = aux_group[0].Get_Send_Data().T;
            {
                uint8_t en = 0u, rt = 0u;
                (void)Cmd_485_Get_Shoot(&en, &rt);
                g_dbg_dial[3][dk] = (float)((en * 10u) + rt);
            }
            g_dbg_dial_idx = (g_dbg_dial_idx + 1u) % 200u;
        }
    }

    /* ★ 调试波形采样(100Hz):180° 问题看这个 */
    {
        static uint32_t s_wave_ms = 0u;
        const uint32_t wnow = (uint32_t)Timestamp_Get_Ms();
        if ((uint32_t)(wnow - s_wave_ms) >= 10u)
        {
            const uint32_t k = g_dbg_6020_idx % 200u;
            s_wave_ms = wnow;
            g_dbg_6020_wave[0][k] = motor_task_aux_get_pos(1u);
            g_dbg_6020_wave[1][k] = g_aux_target_rad[1];
            g_dbg_6020_wave[2][k] = aux_group[1].Get_Send_Data().W;
            g_dbg_6020_wave[3][k] = Math_Angle_Diff_Pi(g_aux_target_rad[1],
                                                       motor_task_aux_get_pos(1u));
            g_dbg_6020_idx = (g_dbg_6020_idx + 1u) % 200u;
        }
    }

    /* 拨盘 / 6020 速度环 */
    if (s_6020_zero_pending && motor_task_can_online(AUX_6020_CAN_ID))
    {
        /* 先把 6020 切到位置环,再把当前位置定义为 0,否则只是速度=0,手能转动 */
        aux_group[1].Set_Motor_Control_mode(motor_pos);
        aux_group[1].PID_Angle.Clear();
        aux_group[1].PID_speed.Clear();
        aux_group[1].zero_to_encoder(AUX_6020_ZERO_ENC);
        g_aux_target_rad[1] = 0.0f;
        aux_group[1].Set_Pos(0.0f);
        s_6020_zero_pending = false;
    }

    aux_group[0].Set_W(g_aux_target_rpm[0] * MATH_RPM_TO_RADPS);
    if (aux_group[1].Get_Motor_Control_mode() == motor_pos)
    {
        aux_group[1].Set_Pos(g_aux_target_rad[1]);
    }
    else
    {
        aux_group[1].Set_W(g_aux_target_rpm[1] * MATH_RPM_TO_RADPS);
    }
    aux_group.PID_Calculate_All();
    if (aux_group.Is_Send_Due())
    {
        aux_group.Send();
    }

#endif

#if MOTOR_485_ENABLE
    /* 485 是半双工一问一答:到点才发 */
    for (uint8_t i = 0u; i < s_485_inited; i++)
    {
        if (s_ys[i].Is_Send_Due())
        {
            s_ys[i].send();
        }
    }
#endif

#if 0
#endif
#if (DM_MOTOR_N > 0)
    for (uint8_t i = 0u; i < DM_MOTOR_N; i++)
    {
        if (dm_motor[i].Is_Send_Due())
        {
            dm_motor[i].send();
        }
    }
#endif

    /* ★ 500Hz 电机紧凑帧:经 485 上送,上板原样转发到串口(PC 串口调 PID 用)
       58 B/帧 @500Hz = 29 KB/s;加号上板自己的 22 KB/s ≈ 51 KB/s，921600 串口留足余量 */
    if (s_fast_stream)
    {
        static uint32_t s_fast_us = 0u;
        const uint32_t now_us = Timestamp_Get_Us32();

        if ((uint32_t)(now_us - s_fast_us) >= 2000u)
        {
            uint8_t *p = s_fast_buf;
            uint16_t n = 0u;
            uint16_t crc;

            s_fast_us = now_us;

            p[n++] = 0x5Au;
            p[n++] = 0x01u;
            p[n++] = (uint8_t)(58u & 0xFFu);
            p[n++] = (uint8_t)(58u >> 8);
            p[n++] = 0x03u;                 /* cmd_id = 0x0103 */
            p[n++] = 0x01u;
            p[n++] = (uint8_t)(s_fast_seq & 0xFFu);
            p[n++] = (uint8_t)(s_fast_seq >> 8);
            s_fast_seq++;

            for (uint8_t i = 0u; i < 4u; i++)
            {
                float v[3];
                v[0] = motor_task_get_now_pos(i);
                v[1] = motor_task_get_now_w(i);
                v[2] = motor_task_get_now_torque(i);
                memcpy(&p[n], v, 12u);
                n = (uint16_t)(n + 12u);
            }

            crc = (uint16_t)crc_ccitt(p, n);
            p[n++] = (uint8_t)(crc & 0xFFu);
            p[n++] = (uint8_t)(crc >> 8);

            Cmd_485_Relay_Push(p, n);
        }
    }

}

/* ============================================================================
 * 七、初始化(只初始化 + 注册接收,不建任务)
 * ========================================================================== */
extern "C" void motor_task_trigger_set_rpm(float __rpm)
{
    Class_Motor_DJ *m = &aux_group[0];

    if (m->Get_Motor_Control_mode() != motor_speed)
    {
        m->Set_Motor_Control_mode(motor_speed);
        m->PID_speed.Clear();
    }
    /* ★ 1kHz 循环里 aux_group[0].Set_W() 回读 g_aux_target_rpm[0],
       不更新它就会被每拍打回 0(和摩擦轮同一类问题) */
    g_aux_target_rpm[0] = __rpm;

    m->Set_W(__rpm * MATH_RPM_TO_RADPS);
    if (m->Get_Send_Status() != Motor_Status_ENABLE)
    {
        m->enable();
    }
}

extern "C" void motor_task_friction_set_rpm(uint8_t __index, float __rpm)
{
    if (__index > 1u)
    {
        return;
    }

    Class_Motor_DJ *m = &aux_group[(__index == 0u) ? 2u : 3u];

    if (m->Get_Motor_Control_mode() != motor_speed)
    {
        m->Set_Motor_Control_mode(motor_speed);
        m->PID_speed.Clear();
    }
    m->Set_W(__rpm * MATH_RPM_TO_RADPS);
    if (m->Get_Send_Status() != Motor_Status_ENABLE)
    {
        m->enable();
    }
}

extern "C" uint8_t motor_task_can_online(uint16_t __can_id)
{
    if ((__can_id < 0x201u) || (__can_id > 0x20Bu)) return 0u;
    uint32_t rx = can_rx_us[__can_id - 0x201u];
    if (rx == 0u) return 0u;
    return ((uint32_t)(Timestamp_Get_Us32() - rx) < MOTOR_ONLINE_TIMEOUT_US) ? 1u : 0u;
}

Class_Motor_DJ *motor_task_get_dj(uint8_t __index)
{
    if (__index == 0u) return &steer_motor;
    if (__index == 1u) return &drive_motor;
    if (__index == 2u) return &steer_motor2;
    if (__index == 3u) return &drive_motor2;
    return nullptr;
}

extern "C" void motor_task_init(void)
{
#if MOTOR_TASK_ENABLE
    /* ★ 先 Init 电机,再启动总线/开中断:
       否则第一帧反馈可能在对象配置好之前进回调 */
    /* Right-rear D wheel only: 0x204 steering, 0x203 drive. */
    steer_motor.Init(&hfdcan1, TRIGGER_MOTOR_CAN_ID, MOTOR_DJ_M3508_BARE,
                     motor_speed, Motor_Control_Freq_1000HZ);
    /* 舵轮输出轴 360° = 3508 转 8 圈(外部 1:8)。
       0x5C 02 的角度、JSON m202.pos 都是舵轮输出轴角度,不是单电机角度。 */
    steer_motor.Set_Gearbox_Rate(8.0f);
    steer_motor.PID_speed.Init(s_pid_cfg[0].speed_kp, s_pid_cfg[0].speed_ki,
                               s_pid_cfg[0].speed_kd, 0.0f,
                               s_pid_cfg[0].speed_max_out, s_pid_cfg[0].speed_max_iout);
#if MOTOR_POS_LOOP_ENABLE
    steer_motor.PID_Angle.Init_PD(s_pid_cfg[0].pos_kp, s_pid_cfg[0].pos_kd,
                                  s_pid_cfg[0].pos_max_w);
    steer_motor.PID_Angle.Set_Angle_Mode(true);
    /* 初始化和定位都走这里:加刹车包线 + 反馈转速阻尼,防止定点来回摆 */
    steer_motor.Set_Pos_Max_Speed(s_pid_cfg[0].pos_max_w);
    steer_motor.Set_Pos_Brake_Decel(40.0f);
    steer_motor.Set_Pos_Vel_Damp(0.6f);
    steer_motor.Set_Pos_Deadband(0.02f);   /* 空载防抖:目标附近 1.1deg 内不追 */
    steer_motor.Set_Pos_Deadband(0.05f);   /* 1.1deg 死区,找零等待时不再抖 */
#endif

    drive_motor.Init(&hfdcan1, DRIVE_MOTOR_CAN_ID, MOTOR_DJ_M3508,
                     motor_speed, Motor_Control_Freq_1000HZ);
    drive_motor.Set_Gearbox_Rate(19.2f);
    drive_motor.PID_speed.Init(s_pid_cfg[1].speed_kp, s_pid_cfg[1].speed_ki,
                               s_pid_cfg[1].speed_kd, 0.0f,
                               s_pid_cfg[1].speed_max_out, s_pid_cfg[1].speed_max_iout);
    drive_motor.Set_W(0.0f);

#if MOTOR_PAIR2_ENABLE
    steer_motor2.Init(&hfdcan1, TRIGGER_MOTOR2_CAN_ID, MOTOR_DJ_M3508_BARE,
                      motor_speed, Motor_Control_Freq_1000HZ);
    steer_motor2.Set_Gearbox_Rate(8.0f);
    steer_motor2.PID_speed.Init(s_pid_cfg[2].speed_kp, s_pid_cfg[2].speed_ki,
                                s_pid_cfg[2].speed_kd, 0.0f,
                                s_pid_cfg[2].speed_max_out, s_pid_cfg[2].speed_max_iout);
#if MOTOR_POS_LOOP_ENABLE
    steer_motor2.PID_Angle.Init_PD(s_pid_cfg[2].pos_kp, s_pid_cfg[2].pos_kd,
                                   s_pid_cfg[2].pos_max_w);
    steer_motor2.PID_Angle.Set_Angle_Mode(true);
    steer_motor2.Set_Pos_Max_Speed(s_pid_cfg[2].pos_max_w);
    steer_motor2.Set_Pos_Brake_Decel(40.0f);
    steer_motor2.Set_Pos_Vel_Damp(0.6f);
    steer_motor2.Set_Pos_Deadband(0.02f);
    steer_motor2.Set_Pos_Deadband(0.05f);
#endif

    drive_motor2.Init(&hfdcan1, DRIVE_MOTOR2_CAN_ID, MOTOR_DJ_M3508,
                      motor_speed, Motor_Control_Freq_1000HZ);
    drive_motor2.Set_Gearbox_Rate(19.2f);
    drive_motor2.PID_speed.Init(s_pid_cfg[3].speed_kp, s_pid_cfg[3].speed_ki,
                                s_pid_cfg[3].speed_kd, 0.0f,
                                s_pid_cfg[3].speed_max_out, s_pid_cfg[3].speed_max_iout);
    drive_motor2.Set_W(0.0f);
#endif

    bsp_can_init(&hfdcan1, CAN1_Callback);
    /* CAN3 init moved to Cmd_Can3_Init() */
    drive_motor.enable();
    steer_motor.enable();
#if MOTOR_PAIR2_ENABLE
    drive_motor2.enable();
    steer_motor2.enable();
#endif

    /* 拨盘 0x206 + 6020 0x205: 同组 0x1FF, 速度环 */
    (void)aux_group.Init(&hfdcan1, 0u, AUX_TRIGGER_CAN_ID,
                         MOTOR_DJ_M3508, motor_speed, Motor_Control_Freq_1000HZ);
    (void)aux_group.Init(&hfdcan1, 1u, AUX_6020_CAN_ID,
                         MOTOR_DJ_M6020, motor_speed, Motor_Control_Freq_1000HZ);
    /* 摩擦轮 0x207/0x208:同一组 0x1FF 的第 3/4 槽(上板 CAN 收不到反馈,放这里) */
    (void)aux_group.Init(&hfdcan1, 2u, 0x207u,
                         MOTOR_DJ_M3508, motor_speed, Motor_Control_Freq_1000HZ);
    (void)aux_group.Init(&hfdcan1, 3u, 0x208u,
                         MOTOR_DJ_M3508, motor_speed, Motor_Control_Freq_1000HZ);
    aux_group[2].Set_Gearbox_Rate(19.2f);   /* 3508 摩擦轮 */
    aux_group[3].Set_Gearbox_Rate(19.2f);
    aux_group[2].PID_speed.Init(s_pid_cfg[PID_IDX_207_FRICTION_L].speed_kp,
                                s_pid_cfg[PID_IDX_207_FRICTION_L].speed_ki,
                                s_pid_cfg[PID_IDX_207_FRICTION_L].speed_kd, 0.0f,
                                s_pid_cfg[PID_IDX_207_FRICTION_L].speed_max_out,
                                s_pid_cfg[PID_IDX_207_FRICTION_L].speed_max_iout);
    aux_group[3].PID_speed.Init(s_pid_cfg[PID_IDX_208_FRICTION_R].speed_kp,
                                s_pid_cfg[PID_IDX_208_FRICTION_R].speed_ki,
                                s_pid_cfg[PID_IDX_208_FRICTION_R].speed_kd, 0.0f,
                                s_pid_cfg[PID_IDX_208_FRICTION_R].speed_max_out,
                                s_pid_cfg[PID_IDX_208_FRICTION_R].speed_max_iout);
    aux_group[0].Set_Gearbox_Rate(19.2f);   /* 3508 拨盘 */
    aux_group[1].Set_Gearbox_Rate(1.0f);    /* 6020 直驱 */
    aux_group[0].PID_speed.Init(s_pid_cfg[PID_IDX_206_TRIGGER].speed_kp,
                                s_pid_cfg[PID_IDX_206_TRIGGER].speed_ki,
                                s_pid_cfg[PID_IDX_206_TRIGGER].speed_kd, 0.0f,
                                s_pid_cfg[PID_IDX_206_TRIGGER].speed_max_out,
                                s_pid_cfg[PID_IDX_206_TRIGGER].speed_max_iout);
    aux_group[1].PID_speed.Init(s_pid_cfg[PID_IDX_6020_YAW].speed_kp,
                                s_pid_cfg[PID_IDX_6020_YAW].speed_ki,
                                s_pid_cfg[PID_IDX_6020_YAW].speed_kd, 0.0f,
                                s_pid_cfg[PID_IDX_6020_YAW].speed_max_out,
                                s_pid_cfg[PID_IDX_6020_YAW].speed_max_iout);
    aux_group[1].PID_Angle.Init_PD(s_pid_cfg[PID_IDX_6020_YAW].pos_kp,
                                   s_pid_cfg[PID_IDX_6020_YAW].pos_kd,
                                   s_pid_cfg[PID_IDX_6020_YAW].pos_max_w);
    aux_group[1].PID_Angle.Set_Angle_Mode(true);
    /* ★ 6020 位置环抗超调:
       · 输出速度上限 = 10 rad/s(≈95 rpm),别让位置环要求它跑不到的速度;
       · 反馈转速阻尼 = 1.5,相当于给位置环加速度反馈(见 motor_DJ.cpp 的 motor_pos)。 */
    aux_group[1].Set_Pos_Max_Speed(s_pid_cfg[PID_IDX_6020_YAW].pos_max_w);
    /* 位置环阻尼按用户要求关闭(0):只跑位置 PID(15 / 0 / 0 / 150)。
       刹车包线 40 rad/s²:大阶跃时按"能停住"的速度走,不冲过头。 */
    aux_group[1].Set_Pos_Vel_Damp(1.05f);   /* 位置环加反馈转速阻尼:压超调(纯P必超) */
    aux_group[1].Set_Pos_Brake_Decel(60.0f);

    /* 6020 零位等第一帧反馈后再做,否则此时 Total_Encoder 还是 0 */
    s_6020_zero_pending = true;
    aux_group.Enable();
    dbg_init_step = 6u;
#endif

#if MOTOR_485_ENABLE
    {
        bool ok485 = true;

        for (uint8_t i = 0u; i < MOTOR_485_NUM; i++)
        {
            const Motor485Cfg *cfg = &s_485_cfg[i];
            uint8_t slot = Motor485_Port_Slot(cfg->uart);

            if (slot == 0xFFu)
            {
                ok485 = false;
                continue;
            }

            s_ys[i].Init(cfg->uart, cfg->id, cfg->model, cfg->mode, cfg->freq, true);

            if (!s_485_port_done[slot])
            {
                USART_RX485_init_Ex(cfg->uart, s_ys[i].Get_Recv_Len(), Motor485_Rx_Callback);
                s_485_port_done[slot] = true;
            }

            s_485_inited = (uint8_t)(i + 1u);
        }

#if MOTOR_485_AUTO_ENABLE
        for (uint8_t i = 0u; i < s_485_inited; i++)
        {
            s_ys[i].enable();
        }
#endif

        motor_485_init_ok = ok485;
    }
#endif

#if (DM_MOTOR_N > 0) && MOTOR_DM_ENABLE_ON_INIT
    /* 达妙:先发 0xFC 使能,等第一帧反馈,再把目标填成当前位置 ⇒ 上电不冲 */
    for (uint8_t i = 0u; i < DM_MOTOR_N; i++)
    {
        dm_motor[i].Set_Kp(MOTOR_DM_KP);
        dm_motor[i].Set_Kd(MOTOR_DM_KD);
        dm_motor[i].enable();
    }

    HAL_Delay(MOTOR_DM_ENABLE_WAIT_MS);

    for (uint8_t i = 0u; i < DM_MOTOR_N; i++)
    {
        dm_motor[i].Set_Pos(dm_motor[i].Get_Now_Pos());
        dm_motor[i].Set_W(0.0f);
        dm_motor[i].Set_T(0.0f);
    }
#endif

    /* 任务不在这里建 —— 见 robot_task.cpp::OSTaskInit() */
}

/* ============================================================================
 * 八、逐台状态(给 error_task 用)
 * ========================================================================== */

extern "C" uint8_t motor_task_get_count(void)
{
#if MOTOR_TASK_ENABLE
    return MOTOR_PAIR2_ENABLE ? 4u : 2u;
#else
    return 0u;
#endif
}

extern "C" bool motor_task_is_init_ok(uint8_t __index)
{
#if MOTOR_TASK_ENABLE
    uint32_t rx;
    if (__index == 0u) rx = dj_rx_us;
    else if (__index == 1u) rx = drive_rx_us;
    else if (__index == 2u) rx = dj_rx_us2;
    else if (__index == 3u) rx = drive_rx_us2;
    else return false;
    return (rx != 0u);
#else
    (void)__index;
    return false;
#endif
}

extern "C" bool motor_task_is_online(uint8_t __index)
{
#if MOTOR_TASK_ENABLE
    uint32_t rx;
    if (__index == 0u) rx = dj_rx_us;
    else if (__index == 1u) rx = drive_rx_us;
    else if (__index == 2u) rx = dj_rx_us2;
    else if (__index == 3u) rx = drive_rx_us2;
    else return false;

    if (rx == 0u) return false;
    return ((uint32_t)(Timestamp_Get_Us32() - rx) < MOTOR_ONLINE_TIMEOUT_US);
#else
    (void)__index;
    return false;
#endif
}

extern "C" uint8_t motor_task_get_m_error(uint8_t __index)
{
    (void)__index;
    return 0u;      /* 大疆反馈里没有故障码字段 */
}

extern "C" const char *motor_task_get_name(uint8_t __index)
{
    if (__index == 0u) return "CAN1-0x202-steer";
    if (__index == 1u) return "CAN1-0x201-drive";
    if (__index == 2u) return "CAN1-0x204-steer";
    if (__index == 3u) return "CAN1-0x203-drive";
    return "N/A";
#if 0
    return "CAN1-DJ-3508";
#else
    return "N/A";
#endif
}

/* ============================================================================
 * 九、给上层写目标 / 读反馈
 *   对外:底盘用 rpm;电机接口用关节侧 rad / rad·s⁻¹ / N·m
 *   换算只在本文件做一次
 * ========================================================================== */

extern "C" bool motor_task_set_mit(uint8_t __index, float __pos, float __w, float __t, float __kp, float __kd)
{
    (void)__t; (void)__kp; (void)__kd;      /* 大疆没有 MIT 帧;PD 在类内 PID 里 */

    if (__index >= motor_task_get_count())
    {
        return false;
    }

    /* 速度模式用 __w;位置模式用 __pos。当前模式由 set_steer_rad/set_wheel_rpm 决定。
       ⚠️ 这里只更新目标缓存,真正 Set_Pos/Set_W 由 motor_task() 每拍统一做,
          否则会被 motor_task() 里的 g_dj_target_* 覆盖 */
    g_dj_target_rad = __pos;
    g_dj_target_rpm = __w / MATH_RPM_TO_RADPS;   /* rad/s → rpm */
    return true;
}

extern "C" bool motor_task_set_wheel_rpm(uint8_t __index, float __rpm, float __kd)
{
    (void)__kd;

    if (__index >= motor_task_get_count())
    {
        return false;
    }

    /* 串口调试接管后,底盘的 500Hz 写入一律忽略,避免把调试目标覆盖掉 */
    if (s_manual_override)
    {
        return false;
    }

    if (__index == 3u)
    {
        if (drive_motor2.Get_Motor_Control_mode() != motor_speed)
        {
            drive_motor2.Set_Motor_Control_mode(motor_speed);
            drive_motor2.PID_speed.Clear();
        }
        drive_motor2.Set_W(__rpm * MATH_RPM_TO_RADPS);
        if (motor_task_is_online(3u) &&
            (drive_motor2.Get_Send_Status() != Motor_Status_ENABLE))
        {
            drive_motor2.enable();
        }
        return true;
    }

    if (__index == 1u)
    {
        if (drive_motor.Get_Motor_Control_mode() != motor_speed)
        {
            drive_motor.Set_Motor_Control_mode(motor_speed);
            drive_motor.PID_speed.Clear();
        }
        drive_motor.Set_W(__rpm * MATH_RPM_TO_RADPS);
        if (motor_task_is_online(1u) &&
            (drive_motor.Get_Send_Status() != Motor_Status_ENABLE))
        {
            drive_motor.enable();
        }
        return true;
    }

    /* 模式切换时才清 PID,避免底盘 500Hz 反复调本函数把积分清掉 */
    if (steer_motor.Get_Motor_Control_mode() != motor_speed)
    {
        steer_motor.Set_Motor_Control_mode(motor_speed);
        steer_motor.PID_speed.Clear();
    }

    /* 从有速度切到 0:把积分清掉,别让 Iout 残留在零速时继续顶力矩 */
    if ((__rpm == 0.0f) && (g_dj_target_rpm != 0.0f))
    {
        steer_motor.PID_speed.Clear();
    }

    g_dj_target_rpm = __rpm;

    /* 之前因离线被 lose 过:有非零目标且已恢复在线时重新使能 */
    if ((__rpm != 0.0f) && motor_task_is_online(0u) &&
        (steer_motor.Get_Send_Status() != Motor_Status_ENABLE))
    {
        steer_motor.enable();
    }
    return true;
}

extern "C" float motor_task_get_wheel_rpm(uint8_t __index)
{
    if (__index >= motor_task_get_count())
    {
        return 0.0f;
    }

    if (__index == 1u) return drive_motor.Get_Now_W() / MATH_RPM_TO_RADPS;
    if (__index == 3u) return drive_motor2.Get_Now_W() / MATH_RPM_TO_RADPS;
    if (__index == 2u) return steer_motor2.Get_Now_W() / MATH_RPM_TO_RADPS;
    return steer_motor.Get_Now_W() / MATH_RPM_TO_RADPS;
}

extern "C" float motor_task_get_now_current(uint8_t __index)
{
    if (__index == 0u) return steer_motor.Get_Now_Current_A();
    if (__index == 1u) return drive_motor.Get_Now_Current_A();
    if (__index == 2u) return steer_motor2.Get_Now_Current_A();
    if (__index == 3u) return drive_motor2.Get_Now_Current_A();
    return 0.0f;
}

extern "C" float motor_task_get_now_torque(uint8_t __index)
{
    if (__index >= motor_task_get_count())
    {
        return 0.0f;
    }
    if (__index == 1u) return drive_motor.Get_Now_T();
    if (__index == 2u) return steer_motor2.Get_Now_T();
    if (__index == 3u) return drive_motor2.Get_Now_T();
    return steer_motor.Get_Now_T();
}

extern "C" bool motor_task_get_steer_feedback(uint8_t __index, Motor_Steer_Feedback_t *__feedback)
{
    if (__feedback == nullptr) return false;
    *__feedback = {0.0f, UINT32_MAX};

#if MOTOR_TASK_ENABLE
    if ((__index != 0u && __index != 2u) || __index >= motor_task_get_count())
    {
        return false;
    }

    const Class_Motor_DJ &motor = (__index == 0u) ? steer_motor : steer_motor2;
    const volatile uint32_t &rx_stamp = (__index == 0u) ? dj_rx_us : dj_rx_us2;

    /* 角度和时间戳来自同一次 CAN 回调，读快照时防止被下一帧打断。 */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t rx = rx_stamp;
    if (rx != 0u && motor.Read_Position_Feedback(__feedback->position_rad))
    {
        __feedback->age_us = Timestamp_Get_Us32() - rx;
    }
    __set_PRIMASK(primask);
#else
    (void)__index;
#endif

    return __feedback->age_us < STEER_FEEDBACK_MAX_AGE_US;
}

extern "C" float motor_task_get_now_pos(uint8_t __index)
{
    if (__index >= motor_task_get_count())
    {
        return 0.0f;
    }
    if (__index == 1u) return drive_motor.Get_Now_pos();
    if (__index == 2u) return steer_motor2.Get_Now_pos();
    if (__index == 3u) return drive_motor2.Get_Now_pos();
    return steer_motor.Get_Now_pos();
}

extern "C" float motor_task_get_now_w(uint8_t __index)
{
    if (__index >= motor_task_get_count())
    {
        return 0.0f;
    }
    if (__index == 1u) return drive_motor.Get_Now_W();
    if (__index == 2u) return steer_motor2.Get_Now_W();
    if (__index == 3u) return drive_motor2.Get_Now_W();
    return steer_motor.Get_Now_W();
}

extern "C" float motor_task_get_pid_out(uint8_t __index)
{
    if (__index >= motor_task_get_count())
    {
        return 0.0f;
    }
    return steer_motor.Get_Send_Data().T;   /* 速度环 PID 输出(关节侧力矩) */
}

extern "C" bool motor_task_set_steer_rad(float __rad)
{
#if !MOTOR_POS_LOOP_ENABLE
    (void)__rad;
    return false;   /* 位置环关闭:舵向角度接口不可用 */
#else
    if (motor_task_get_count() == 0u)
    {
        return false;
    }

    /* 串口调试接管后,底盘 500Hz 的舵向写入必须让路,否则会覆盖调试角度 */
    if (s_manual_override)
    {
        return false;
    }

    if (steer_motor.Get_Motor_Control_mode() != motor_pos)
    {
        steer_motor.Set_Motor_Control_mode(motor_pos);
        steer_motor.PID_Angle.Clear();
        steer_motor.PID_speed.Clear();
    }

    g_dj_target_rad = __rad;
    steer_motor.Set_Pos(__rad);
    return true;
#endif
}

extern "C" bool motor_task_set_steer_rad_index(uint8_t __index, float __rad)
{
    if (__index == 0u)
    {
        return motor_task_set_steer_rad(__rad);
    }
    if (__index == 2u)
    {
#if MOTOR_POS_LOOP_ENABLE
        if (s_manual_override) return false;
        if (!motor_task_is_online(2u)) return false;
        if (steer_motor2.Get_Motor_Control_mode() != motor_pos)
        {
            steer_motor2.Set_Motor_Control_mode(motor_pos);
            steer_motor2.PID_Angle.Clear();
            steer_motor2.PID_speed.Clear();
        }
        g_dj_target_rad2 = __rad;
        steer_motor2.Set_Pos(__rad);
        return true;
#else
        (void)__rad;
        return false;
#endif
    }
    return false;
}

extern "C" float motor_task_get_steer_rad(void)
{
    if (motor_task_get_count() == 0u)
    {
        return 0.0f;
    }
    return steer_motor.Get_Now_pos();
}

extern "C" void motor_task_set_target_rpm(float __rpm)
{
#if MOTOR_TASK_ENABLE
    g_dj_target_rpm = __rpm;
#else
    (void)__rpm;
#endif
}

extern "C" float motor_task_get_target_rpm(void)
{
#if MOTOR_TASK_ENABLE
    return g_dj_target_rpm;
#else
    return 0.0f;
#endif
}

extern "C" float motor_task_get_pid_target_rpm(void)
{
#if MOTOR_TASK_ENABLE
    /* send_data.W 才是速度环真正吃到的目标:
       速度模式 = g_dj_target_rpm;位置模式 = PID_Angle 的输出 */
    return steer_motor.Get_Send_Data().W / MATH_RPM_TO_RADPS;
#else
    return 0.0f;
#endif
}

extern "C" float motor_task_get_target_rad(void)
{
#if MOTOR_TASK_ENABLE
    return g_dj_target_rad;
#else
    return 0.0f;
#endif
}

extern "C" float motor_task_get_target_rad2(void)
{
#if MOTOR_TASK_ENABLE
    return g_dj_target_rad2;
#else
    return 0.0f;
#endif
}

extern "C" float motor_task_get_pid_target_rpm2(void)
{
#if MOTOR_TASK_ENABLE
    return steer_motor2.Get_Send_Data().W / MATH_RPM_TO_RADPS;
#else
    return 0.0f;
#endif
}

/* ============================================================================
 * 十、串口调试(上位机“电机调试”窗口,0x5C 帧)
 *   命令一旦生效就锁住底盘写入,直到复位。
 * ========================================================================== */
extern "C" void motor_task_debug_set_speed_rpm(int16_t __rpm)
{
#if MOTOR_TASK_ENABLE
    if (motor_task_get_count() == 0u)
    {
        return;
    }

    /* ★ 没有反馈时不许给速度:否则 PID 看不到实际转速,会开环输出力矩 */
    if (!motor_task_is_online(0u))
    {
        g_dj_target_rpm = 0.0f;
        return;
    }

    if (steer_motor.Get_Motor_Control_mode() != motor_speed)
    {
        steer_motor.Set_Motor_Control_mode(motor_speed);
        steer_motor.PID_speed.Clear();
        steer_motor.PID_Angle.Clear();
        g_dj_target_rad = steer_motor.Get_Now_pos();   /* 防止以后切位置模式时用旧目标 */
    }

    /* 从有速度切到 0:清积分,别让 Iout 在零速时残留成 1.2 N·m 的顶力矩 */
    if ((__rpm == 0) && (g_dj_target_rpm != 0.0f))
    {
        steer_motor.PID_speed.Clear();
    }

    g_dj_target_rpm = (float)__rpm;

    if (steer_motor.Get_Send_Status() != Motor_Status_ENABLE)
    {
        steer_motor.enable();                          /* 从 stop 状态恢复 */
    }

    s_manual_override_take();
#else
    (void)__rpm;
#endif
}

extern "C" void motor_task_debug_set_pos_mdeg(int32_t __mdeg)
{
#if (MOTOR_TASK_ENABLE && MOTOR_POS_LOOP_ENABLE)
    if (motor_task_get_count() == 0u)
    {
        return;
    }

    /* 没有反馈时位置环同样没有意义 */
    if (!motor_task_is_online(0u))
    {
        return;
    }

    const fp32 rad = (fp32)__mdeg * 0.001f * (PI / 180.0f);   /* 0.001° → rad */

    if (steer_motor.Get_Motor_Control_mode() != motor_pos)
    {
        steer_motor.Set_Motor_Control_mode(motor_pos);
        steer_motor.PID_Angle.Clear();
        steer_motor.PID_speed.Clear();
    }

    g_dj_target_rad = rad;

    if (steer_motor.Get_Send_Status() != Motor_Status_ENABLE)
    {
        steer_motor.enable();
    }

    s_manual_override_take();
#else
    (void)__mdeg;   /* 位置环关闭:0x5C 角度命令直接忽略 */
#endif
}

extern "C" void motor_task_debug_set_pid(float __skp, float __ski, float __skd,
                                         float __pkp, float __pki, float __pkd)
{
#if MOTOR_TASK_ENABLE
    /* 速度环:输出 = 关节力矩;限幅沿用当前整定值 */
    s_pid_cfg[0].speed_kp = __skp; s_pid_cfg[0].speed_ki = __ski; s_pid_cfg[0].speed_kd = __skd;
    s_pid_cfg[0].pos_kp = __pkp; s_pid_cfg[0].pos_ki = __pki; s_pid_cfg[0].pos_kd = __pkd;
    steer_motor.PID_speed.Init(s_pid_cfg[0].speed_kp, s_pid_cfg[0].speed_ki,
                               s_pid_cfg[0].speed_kd, 0.0f,
                               s_pid_cfg[0].speed_max_out, s_pid_cfg[0].speed_max_iout);

    /* 位置环参数也保存下来,方便回读;MOTOR_POS_LOOP_ENABLE=0 时它不参与控制 */
    steer_motor.PID_Angle.Init(s_pid_cfg[0].pos_kp, s_pid_cfg[0].pos_ki,
                               s_pid_cfg[0].pos_kd, 0.0f, s_pid_cfg[0].pos_max_w, 0.0f);
    steer_motor.PID_Angle.Set_Angle_Mode(true);

    s_manual_override_take();
#else
    (void)__skp; (void)__ski; (void)__skd;
    (void)__pkp; (void)__pki; (void)__pkd;
#endif
}

extern "C" void motor_task_debug_get_pid(float *__pid6)
{
    if (__pid6 == nullptr)
    {
        return;
    }

    __pid6[0] = steer_motor.PID_speed.Get_KP();
    __pid6[1] = steer_motor.PID_speed.Get_KI();
    __pid6[2] = steer_motor.PID_speed.Get_KD();
    __pid6[3] = steer_motor.PID_Angle.Get_KP();
    __pid6[4] = steer_motor.PID_Angle.Get_KI();
    __pid6[5] = steer_motor.PID_Angle.Get_KD();
}

/* ---- 台架:摩擦轮 0x207/0x208 ---- */
extern "C" uint8_t motor_task_friction_flags(uint8_t __index)
{
    uint8_t flags = 0u;
    uint32_t now = Timestamp_Get_Us32();
    uint32_t rx = (__index == 0u) ? friction_rx_us[0] : friction_rx_us[1];

    if (rx != 0u)
    {
        flags |= 0x01u;
        if ((uint32_t)(now - rx) < MOTOR_ONLINE_TIMEOUT_US)
        {
            flags |= 0x02u;
        }
    }
    return flags;
}

extern "C" float motor_task_friction_pos(uint8_t __index)
{
    return (__index == 0u) ? friction_l.Get_Now_pos() : friction_r.Get_Now_pos();
}

extern "C" float motor_task_friction_w(uint8_t __index)
{
    return (__index == 0u) ? friction_l.Get_Now_W() : friction_r.Get_Now_W();
}

extern "C" float motor_task_friction_t(uint8_t __index)
{
    return (__index == 0u) ? friction_l.Get_Now_T() : friction_r.Get_Now_T();
}

extern "C" void motor_task_friction_shoot(float __rpm)
{
    motor_task_friction_set_rpm(0u, -__rpm);
    motor_task_friction_set_rpm(1u, __rpm);
}

extern "C" void motor_task_friction_set_pid(uint8_t __index, float __kp, float __ki, float __kd)
{
    Class_Motor_DJ *m = (__index == 0u) ? &friction_l : &friction_r;
    const uint8_t pi = (__index == 0u) ? PID_IDX_207_FRICTION_L : PID_IDX_208_FRICTION_R;
    s_pid_cfg[pi].speed_kp = __kp; s_pid_cfg[pi].speed_ki = __ki; s_pid_cfg[pi].speed_kd = __kd;
    m->PID_speed.Init(s_pid_cfg[pi].speed_kp, s_pid_cfg[pi].speed_ki,
                      s_pid_cfg[pi].speed_kd, 0.0f,
                      s_pid_cfg[pi].speed_max_out, s_pid_cfg[pi].speed_max_iout);
}

extern "C" void motor_task_friction_get_pid(uint8_t __index, float *__pid3)
{
    Class_Motor_DJ *m = (__index == 0u) ? &friction_l : &friction_r;
    if (__pid3 == nullptr) return;
    __pid3[0] = m->PID_speed.Get_KP();
    __pid3[1] = m->PID_speed.Get_KI();
    __pid3[2] = m->PID_speed.Get_KD();
}


extern "C" void motor_task_aux_set_rpm(uint8_t __index, int16_t __rpm)
{
    if (__index > 1u) return;
    Class_Motor_DJ *m = &aux_group[__index];
    if (m->Get_Motor_Control_mode() != motor_speed)
    {
        m->Set_Motor_Control_mode(motor_speed);
        m->PID_speed.Clear();
    }
    g_aux_target_rpm[__index] = (float)__rpm;
    m->Set_W((float)__rpm * MATH_RPM_TO_RADPS);
    if (m->Get_Send_Status() != Motor_Status_ENABLE) m->enable();
    s_manual_override_take();
}

/* 小陀螺/云台保持用:只写目标,不夺取底盘控制权(不置 manual_override) */
static bool s_6020_debug = false;   /* 6020 调试开关:置位后底盘不再改写 6020 目标 */

extern "C" void motor_task_aux_set_pos_hold(uint8_t __index, float __rad)
{
    if (__index != 1u) return;
    if (s_6020_debug) return;   /* 调试:底盘暂停控制 6020 */
    Class_Motor_DJ *m = &aux_group[1];
    if (m->Get_Motor_Control_mode() != motor_pos)
    {
        m->Set_Motor_Control_mode(motor_pos);
        m->PID_Angle.Clear();
        m->PID_speed.Clear();
    }
    g_aux_target_rad[1] = __rad;
    m->Set_Pos(__rad);
    if (m->Get_Send_Status() != Motor_Status_ENABLE) m->enable();
}

/* 外部速度环用:只写速度,不夺取底盘控制权(6020 小陀螺外部环) */
/* 舵向找零完成:把当前位置记为 __rad(加偏置后再作为零点) */
/* 舵轮调试开关 */
static bool s_steer_debug = false;

extern "C" void motor_task_set_steer_debug(uint8_t __on)
{
    s_steer_debug = (__on != 0u);
}

extern "C" bool motor_task_steer_debug_enabled(void)
{
    return s_steer_debug;
}

extern "C" void motor_task_set_6020_debug(uint8_t __on)
{
    s_6020_debug = (__on != 0u);
}

extern "C" bool motor_task_6020_debug_enabled(void)
{
    return s_6020_debug;
}

extern "C" float motor_task_get_drive_target_rpm(uint8_t __index)
{
#if MOTOR_TASK_ENABLE
    if (__index == 0u) return drive_motor.Get_Send_Data().W / MATH_RPM_TO_RADPS;
#if MOTOR_PAIR2_ENABLE
    if (__index == 2u) return drive_motor2.Get_Send_Data().W / MATH_RPM_TO_RADPS;
#endif
#endif
    return 0.0f;
}

extern "C" void motor_task_steer_zero(uint8_t __unit, float __rad)
{
    Class_Motor_DJ *m = (__unit == 0u) ? &steer_motor : &steer_motor2;

    m->zero_to_angle(__rad);   /* 当前机械位置 = __rad */
    m->PID_Angle.Clear();
    m->PID_speed.Clear();
}

extern "C" void motor_task_aux_set_pos_vel_ff(uint8_t __index, float __ff)
{
    if (__index != 1u) return;
    aux_group[1].Set_Pos_Vel_Feedforward(__ff);
}

extern "C" void motor_task_aux_set_speed_hold(uint8_t __index, float __rpm)
{
    if (__index != 1u) return;
    Class_Motor_DJ *m = &aux_group[1];
    if (m->Get_Motor_Control_mode() != motor_speed)
    {
        m->Set_Motor_Control_mode(motor_speed);
        m->PID_speed.Clear();
    }
    m->Set_W(__rpm * MATH_RPM_TO_RADPS);
}

/* 调试串口(0x5C)用:写目标并接管,1s 无新命令自动交还 */
extern "C" void motor_task_aux_set_pos(uint8_t __index, float __rad)
{
    motor_task_aux_set_pos_hold(__index, __rad);
    s_manual_override_take();
}

extern "C" void motor_task_aux_set_pid(uint8_t __index, float __kp, float __ki, float __kd)
{
    if (__index > 1u) return;
    const uint8_t pi = (__index == 0u) ? PID_IDX_206_TRIGGER : PID_IDX_6020_YAW;
    s_pid_cfg[pi].speed_kp = __kp; s_pid_cfg[pi].speed_ki = __ki; s_pid_cfg[pi].speed_kd = __kd;
    aux_group[__index].PID_speed.Init(__kp, __ki, __kd, 0.0f,
                                      s_pid_cfg[pi].speed_max_out,
                                      s_pid_cfg[pi].speed_max_iout);
}

/* 6020 位置环在线调参(0x5C 0x13):[pos_kp][vel_damp][max_w] */
extern "C" void motor_task_aux_set_pos_pid(uint8_t __index, float __kp, float __damp, float __max_w)
{
    if (__index != 1u) return;
    if (__kp    < 0.0f) __kp    = 0.0f;
    if (__damp  < 0.0f) __damp  = 0.0f;
    if (__max_w <= 0.0f) __max_w = s_pid_cfg[PID_IDX_6020_YAW].pos_max_w;

    s_pid_cfg[PID_IDX_6020_YAW].pos_kp    = __kp;
    s_pid_cfg[PID_IDX_6020_YAW].pos_max_w = __max_w;

    aux_group[1].PID_Angle.Init_PD(__kp, 0.0f, __max_w);
    aux_group[1].PID_Angle.Set_Angle_Mode(true);
    aux_group[1].Set_Pos_Max_Speed(__max_w);
    aux_group[1].Set_Pos_Vel_Damp(__damp);
}

extern "C" float motor_task_aux_get_w_cmd(uint8_t __index)
{
    if (__index > 1u) return 0.0f;
    return aux_group[__index].Get_Send_Data().W;
}

extern "C" float motor_task_aux_get_torque(uint8_t __index)
{
    if (__index > 1u) return 0.0f;
    return aux_group[__index].Get_Now_T();
}

extern "C" float motor_task_aux_get_w(uint8_t __index)
{
    if (__index > 1u) return 0.0f;
    return aux_group[__index].Get_Now_W();
}

extern "C" uint16_t motor_task_aux_get_encoder(uint8_t __index)
{
    return (__index > 1u) ? 0u : aux_group[__index].Get_Raw_Encoder();
}

extern "C" uint32_t motor_task_can3_rx_count(void) { return can3_rx_count; }
extern "C" uint32_t motor_task_can3_last_id(void) { return can3_last_id; }

extern "C" bool motor_task_can3_get(float *__vx, float *__vy, float *__w, uint8_t *__mode)
{
    if ((uint32_t)(Timestamp_Get_Ms() - can3_last_ms) > 100u) return false;
    if (__vx) *__vx = (float)can3_vx_mm_s * 0.001f;
    if (__vy) *__vy = (float)can3_vy_mm_s * 0.001f;
    if (__w)  *__w  = (float)can3_w_mrad_s * 0.001f;
    if (__mode) *__mode = can3_mode;
    return true;
}

extern "C" uint8_t motor_task_aux_online(uint8_t __index)
{
    if (__index > 1u) return 0u;
    return motor_task_can_online(__index == 0u ? AUX_TRIGGER_CAN_ID : AUX_6020_CAN_ID);
}

extern "C" float motor_task_aux_get_target_rpm(uint8_t __index)
{
    return (__index > 1u) ? 0.0f : g_aux_target_rpm[__index];
}

/* 6020 零点校准是否已完成(建系用:完成瞬间 6020 相对角才真的是 0) */
extern "C" bool motor_task_6020_is_zeroed(void)
{
    return !s_6020_zero_pending;
}

extern "C" float motor_task_aux_get_pos(uint8_t __index)
{
    return (__index > 1u) ? 0.0f : aux_group[__index].Get_Now_pos();
}

extern "C" float motor_task_aux_get_target_rad(uint8_t __index)
{
    return (__index > 1u) ? 0.0f : g_aux_target_rad[__index];
}

extern "C" void motor_task_debug_set_drive_rpm(int16_t __rpm)
{
    if (!motor_task_is_online(1u)) return;
    if (drive_motor.Get_Motor_Control_mode() != motor_speed)
    {
        drive_motor.Set_Motor_Control_mode(motor_speed);
        drive_motor.PID_speed.Clear();
    }
    drive_motor.Set_W((float)__rpm * MATH_RPM_TO_RADPS);
    if (drive_motor.Get_Send_Status() != Motor_Status_ENABLE)
    {
        drive_motor.enable();
    }
    s_manual_override_take();
}

extern "C" void motor_task_debug_set_drive_pid(float __kp, float __ki, float __kd)
{
    s_pid_cfg[1].speed_kp = __kp; s_pid_cfg[1].speed_ki = __ki; s_pid_cfg[1].speed_kd = __kd;
    s_pid_cfg[3].speed_kp = __kp; s_pid_cfg[3].speed_ki = __ki; s_pid_cfg[3].speed_kd = __kd;
    drive_motor.PID_speed.Init(s_pid_cfg[1].speed_kp, s_pid_cfg[1].speed_ki,
                               s_pid_cfg[1].speed_kd, 0.0f,
                               s_pid_cfg[1].speed_max_out, s_pid_cfg[1].speed_max_iout);
    drive_motor2.PID_speed.Init(s_pid_cfg[3].speed_kp, s_pid_cfg[3].speed_ki,
                                s_pid_cfg[3].speed_kd, 0.0f,
                                s_pid_cfg[3].speed_max_out, s_pid_cfg[3].speed_max_iout);
}

extern "C" void motor_task_debug_set_steer2_mdeg(int32_t __mdeg)
{
#if (MOTOR_TASK_ENABLE && MOTOR_POS_LOOP_ENABLE)
    if (!motor_task_is_online(2u)) return;
    const fp32 rad = (fp32)__mdeg * 0.001f * (PI / 180.0f);
    if (steer_motor2.Get_Motor_Control_mode() != motor_pos)
    {
        steer_motor2.Set_Motor_Control_mode(motor_pos);
        steer_motor2.PID_Angle.Clear();
        steer_motor2.PID_speed.Clear();
    }
    g_dj_target_rad2 = rad;
    steer_motor2.Set_Pos(rad);
    if (steer_motor2.Get_Send_Status() != Motor_Status_ENABLE) steer_motor2.enable();
    s_manual_override_take();
#else
    (void)__mdeg;
#endif
}

extern "C" void motor_task_debug_set_steer2_rpm(int16_t __rpm)
{
    if (!motor_task_is_online(2u)) return;
    if (steer_motor2.Get_Motor_Control_mode() != motor_speed)
    {
        steer_motor2.Set_Motor_Control_mode(motor_speed);
        steer_motor2.PID_speed.Clear();
    }
    g_dj_target_rpm2 = (float)__rpm;
    steer_motor2.Set_W((float)__rpm * MATH_RPM_TO_RADPS);
    if (steer_motor2.Get_Send_Status() != Motor_Status_ENABLE) steer_motor2.enable();
}

extern "C" void motor_task_debug_set_drive2_rpm(int16_t __rpm)
{
    if (!motor_task_is_online(3u)) return;
    if (drive_motor2.Get_Motor_Control_mode() != motor_speed)
    {
        drive_motor2.Set_Motor_Control_mode(motor_speed);
        drive_motor2.PID_speed.Clear();
    }
    drive_motor2.Set_W((float)__rpm * MATH_RPM_TO_RADPS);
    if (drive_motor2.Get_Send_Status() != Motor_Status_ENABLE) drive_motor2.enable();
}

extern "C" void motor_task_debug_set_steer2_pid(float __skp, float __ski, float __skd,
                                                 float __pkp, float __pki, float __pkd)
{
    s_pid_cfg[2].speed_kp = __skp; s_pid_cfg[2].speed_ki = __ski; s_pid_cfg[2].speed_kd = __skd;
    s_pid_cfg[2].pos_kp = __pkp; s_pid_cfg[2].pos_ki = __pki; s_pid_cfg[2].pos_kd = __pkd;
    steer_motor2.PID_speed.Init(s_pid_cfg[2].speed_kp, s_pid_cfg[2].speed_ki,
                                s_pid_cfg[2].speed_kd, 0.0f,
                                s_pid_cfg[2].speed_max_out, s_pid_cfg[2].speed_max_iout);
    steer_motor2.PID_Angle.Init(s_pid_cfg[2].pos_kp, s_pid_cfg[2].pos_ki,
                                s_pid_cfg[2].pos_kd, 0.0f, s_pid_cfg[2].pos_max_w, 0.0f);
    steer_motor2.PID_Angle.Set_Angle_Mode(true);
}

extern "C" void motor_task_debug_stop(void)
{
#if MOTOR_TASK_ENABLE
    s_manual_override_take();

    g_dj_target_rpm = 0.0f;
    g_dj_target_rad = steer_motor.Get_Now_pos();

    steer_motor.lose();    /* 发一帧 0 电流 + 命令位 DISABLE;后续 send() 继续发 0 */
#endif
}
