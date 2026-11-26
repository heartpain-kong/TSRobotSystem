//
// chassis_task.cpp  2 steering + 2 omni chassis (Class_chassis_D2Q2)
//

#include "chassis_task.h"
#include "steer_tracking_lead.h"
#include "dev_electric.h"

#include "cmd_task.h"
#include "cmd_485.h"
#include "alg_chassis_control.h"
#include "imu_task.h"
#include "robot_def.h"
#include "robot.h"
#include "motor_task.h"
#include "msg_center.h"
#include "error_task.h"
#include "math_support.h"
#include "bsp_power.h"
#include "bsp_timestamp.h"
#include "servo_D_init.h"
#include "debug_task.h"

#include <string.h>

static_assert(sizeof(Chassis_Data_t) <= MSG_SLOT_SIZE, "Chassis_Data_t too large");

/* ============================================================================
 * geometry / power —— 数值全部在 robot_def.h 的「本机配置」段,这里只做别名
 * ========================================================================== */
#define CHASSIS_HALF_A          CFG_CHASSIS_HALF_A   /* 舵轮(驱动轮)前后半轴距 */
#define CHASSIS_HALF_B          CFG_CHASSIS_HALF_B   /* 舵轮(驱动轮)左右半轴距 */
/* 全向轮是从动支撑轮,不参与驱动逆解 */
#define CHASSIS_HALF_OMNI       CFG_CHASSIS_HALF_OMNI
/* 两个舵轮落在 Y 轴上:半径 = sqrt(A^2 + B^2) */
#define CHASSIS_HALF_DIAG       CFG_CHASSIS_HALF_DIAG
#define CHASSIS_IMU_TO_KINEMATIC_YAW (-1.0f) /* 实测底盘 IMU 与舵轮角速度正方向相反 */
#define CHASSIS_STICK_TO_BODY_YAW (CFG_STICK_TO_BODY_YAW_DEG * PI / 180.0f)
/* 6020 relative angle -> body-frame rotation sign (the gimbal 6020 zero = y+ reference).
 * +1 = gimbal turns CCW => wheel seat target turns CCW (wheels follow the gimbal, same direction). */
#define CHASSIS_GIMBAL_POS_SIGN   CFG_GIMBAL_POS_SIGN
#define CHASSIS_HEADING_CORR_RATIO 0.03f  /* 航向修正在轮心产生的速度占平移速度的上限(见下) */
#define CHASSIS_HEADING_KP       2.5f
#define CHASSIS_HEADING_KI       0.8f
#define CHASSIS_HEADING_KD       0.10f
#define CHASSIS_HEADING_I_MAX    0.2f
#define CHASSIS_HEADING_W_MAX    1.5f     /* 航向修正上限 [rad/s] */
/* ★ toe-in 上限 = 2·atan(0.707·CORR_RATIO):
   0.25 → 20°(两轮被差速拧成 ±10°,实测 ±12°);0.03 → 2.4°。
   航向目标永不重锁(车被转过也照样转回来),所以修正权只靠这个比例压。*/
#define CHASSIS_HEADING_W_DEADBAND 0.05f  /* 操作手自己给 W 时不介入(目标不重锁) */
#define CHASSIS_WHEEL_R         CFG_CHASSIS_WHEEL_R     /* D 杞崐寰?50mm */
#define CHASSIS_WHEEL_KD        1.0f
#define CHASSIS_POWER_LIMIT_ENABLE 0        /* 0=临时关闭功率限幅,先确认驱动能否转 */
#define CHASSIS_POWER_KP        0.5f
#define CHASSIS_BOOT_TEST_ENABLE 0
#define CHASSIS_BOOT_TEST_MS     3000u
#define CHASSIS_BOOT_TEST_VY     0.5236f   /* 100 rpm 对应的线速度 */
#define CHASSIS_BOOT_TEST_YAW    (PI / 2.0f) /* 90°: yaw=0 一+x, yaw=90° 一+y */
#define CHASSIS_STEER_ZERO    (45.0f * PI / 180.0f)   /* 鍏夌數鎵鹃浂鍚庣殑鏈烘闆朵綅 */
#define CHASSIS_HOMING_TEST_ENABLE 0
#define CHASSIS_HOMING_TEST_RPM    100.0f              /* +x 方向,201 反转 */
#define CHASSIS_HOMING_TEST_MS     0u     /* 0 = 不测试;>0 = 上电测这么久 */

#define CHASSIS_DRIVE_EFF       0.75f
#define CHASSIS_IDLE_POWER      8.0f
#define CHASSIS_POWER_MARGIN    5.0f
#define CHASSIS_BUFFER_LOW      20u
#define CHASSIS_USE_C620_CURRENT 1   /* 1 = 用 C620 反馈电流×电池电压 估算,和机械功率取较大值 */
#define CHASSIS_CURRENT_BUS_SCALE 0.6f

/* D2Q2: two steering wheels, each with steer + drive.
   The two Q wheels are passive and do not have motors. */
/* Only left-front D is wired now: steer=0x202 (index 0), drive=0x201 (index 1). */
#define D2Q2_STEER_L            0u   /* 202 */
#define D2Q2_DRIVE_L            1u   /* 201 驱动, 无光甀*/
#define D2Q2_STEER_R            2u   /* 204 舵向 */
#define D2Q2_DRIVE_R            3u   /* 203 驱动 */

static const uint8_t s_d2q2_drive[CHASSIS_D2Q2_UNIT_N] = {D2Q2_DRIVE_L, D2Q2_DRIVE_R};
static const uint8_t s_d2q2_steer[CHASSIS_D2Q2_UNIT_N] = {D2Q2_STEER_L, D2Q2_STEER_R};
static const Enum_Electric_Input s_d2q2_photo[CHASSIS_D2Q2_UNIT_N] = {
    ELECTRIC_IN_PHOTO_2,   /* 202 舵向光电 -> PE3 */
    ELECTRIC_IN_PHOTO_1    /* 204 舵向光电 -> PE2 */
};

/* 零点映射(找零只认 0→1 边沿,1→0 会被跳过继续找):
   202: 0→1 起 -45°, +bias(+225°) ⇒ 停位/零点 = 上升沿 +225° (final 180°)
   204: 0→1 起 -45°, +bias(+135°) ⇒ 停位/零点 = 上升沿 +135° (final 90°)
   zero_fall 保留在表里(1→2 已是 180° 的等价位),Servo_D_Init 现在不用它。*/
static const float s_d2q2_zero_rise[CHASSIS_D2Q2_UNIT_N] = {
    -45.0f * PI / 180.0f, -45.0f * PI / 180.0f
};
static const float s_d2q2_zero_fall[CHASSIS_D2Q2_UNIT_N] = {
    135.0f * PI / 180.0f, 135.0f * PI / 180.0f
};
static const float s_d2q2_zero_bias[CHASSIS_D2Q2_UNIT_N] = {
    225.0f * PI / 180.0f,  135.0f * PI / 180.0f
};

/* 舵向标定微调(叠加在找零算出的 seat_offset 上,不改变找零状态机):
   数值在 robot_def.h:CFG_SEAT_TRIM_DEG_202/204(202 实测偏 10°,给 -10°) */
static const float s_d2q2_seat_trim[CHASSIS_D2Q2_UNIT_N] = {
    CFG_SEAT_TRIM_DEG_202 * PI / 180.0f,   /* 0 = 202 */
    CFG_SEAT_TRIM_DEG_204 * PI / 180.0f    /* 1 = 204 */
};

/* ============================================================================
 * state
 * ========================================================================== */
static Class_chassis_D2Q2 s_d2q2;
/* 实车布局(物理不变,坐标系绕 x 转了 45°):
   两个舵轮落在 Y 轴上:D(1 2)=Y-,D(4 3)=Y+;左右两个角是被动全向轮。 */
static const chassis_D_wheel_init s_d2q2_init[CHASSIS_D2Q2_UNIT_N] = {
    { +CHASSIS_HALF_A, -CHASSIS_HALF_B, -PI / 4.0f },   /* 202/201: x+, y- (右下) */
    { -CHASSIS_HALF_A, +CHASSIS_HALF_B, PI + PI / 4.0f },   /* 204/203: x-, y+ (左上), +225deg(=+45+180) */
};

/* 0° 时 201 正转 = +x;180° 时由 D2Q2 的 180° 翻转逻辑自动给负转速。按实测 0 号 0x201 左驱动取反 → +1;1 号 0x203 右驱动保持 -1。 */
static const float s_d2q2_drive_sign[CHASSIS_D2Q2_UNIT_N] = { +1.0f, -1.0f };

static Chassis_Data_t s_data;
static Cmd_Data_t s_cmd;

/* 舵向角度反馈(0=202,1=204):实测/闭环目标,供调试遥测 */
static float g_seat_now[CHASSIS_D2Q2_UNIT_N] = {0.0f, 0.0f};
static float g_seat_tgt[CHASSIS_D2Q2_UNIT_N] = {0.0f, 0.0f};
/* 航向环诊断(0x0104 遥测用) */
static float g_heading_error = 0.0f;      /* 本次用的 yaw 误差 [rad](重锁后为 0) */
static float g_chassis_w_cmd = 0.0f;      /* 送进运动学的 ω [rad/s](含航向修正) */
static bool  g_zero_pending[CHASSIS_D2Q2_UNIT_N] = {false, false};
static uint32_t g_zero_ms[CHASSIS_D2Q2_UNIT_N] = {0u, 0u};

float Chassis_Get_Seat_Now(uint8_t __unit)
{
    return (__unit < CHASSIS_D2Q2_UNIT_N) ? g_seat_now[__unit] : 0.0f;
}

float Chassis_Get_Seat_Target(uint8_t __unit)
{
    return (__unit < CHASSIS_D2Q2_UNIT_N) ? g_seat_tgt[__unit] : 0.0f;
}

float Chassis_Get_Heading_Error(void)
{
    return g_heading_error;
}

float Chassis_Get_Yaw_Rate_Cmd(void)
{
    return g_chassis_w_cmd;
}
static uint32_t s_cmd_seq = 0u;
static uint32_t s_imu_seq = 0u;
static bool s_last_spin = false;
static uint32_t s_boot_test_ms = 0u;
static bool s_boot_test_done = false;
static bool s_boot_test_run = false;
static bool s_last_gimbal_spin = false;
static float s_gimbal_heading0 = 0.0f;

/* 小陀螺 yaw 闭环(反馈=底盘 IMU + 6020 相对角)。现在是"建系 + 直接反算位置目标",这两个增益不再参与。 */
#define CHASSIS_SPIN_HOLD_KP      10.0f
#define CHASSIS_SPIN_HOLD_KD      0.02f
#define CHASSIS_SPIN_HOLD_SIGN    (+1.0f)   /* 装反亀越给正转云台越反)改成 -1.0f */
/* 瑙掗�熷害鍓嶉(鐩镐綅瓒呭墠,鍗曚綅 s):搴曠洏杞緱瓒婂揩,鐩爣瓒婃彁鍓嶇粰鍑?琛ュ伩璺熼殢婊炲悗銆?   0 = 鍏抽棴;鍏稿瀷 0.02~0.04 s銆?*/
#define CHASSIS_SPIN_YAW_LEAD_S   (0.02f)
/* angular-acceleration feedforward [s^2]: ~0.5*T^2, T 20~30ms -> 0.0002~0.00045 */
#define CHASSIS_SPIN_ACC_FF       (0.0f)   /* 关闭:会叠加到超前量 */
/* 速度前馈里的加速度补偿时间常数 [s] */
#define CHASSIS_SPIN_FF_TAU       (0.02f)
/* 上板云台 IMU 的慢修正增益(0~1):
   杞殑鏃跺�欏皬(涓嶇劧鍜屼綅缃幆鎸崱,瀹炴祴 0.3@500Hz 浼?9Hz 鏉ュ洖缈?;
   鍋滀綇浠ュ悗鍙互缁欏ぇ涓�鐐?鎶婅嚜鏃嬫湡闂存敀涓嬬殑鍋忕疆蹇�熶慨鍥炴潵銆?*/
#define CHASSIS_SPIN_UP_GAIN      (0.30f)   /* 底盘在转:上板 IMU 主环增益 */
#define CHASSIS_SPIN_UP_GAIN_STILL (0.50f)  /* 底盘停住:可以给大一点,把残余偏置拉回 */
#define CHASSIS_SPIN_UP_SIGN      (-1.0f)   /* fix: +1 was positive feedback */
/* 每拍修正量上限 [rad]:防止 π 附近符号翻转时目标一次跳 2π(那就是 9Hz 来回甩的来源) */
#define CHASSIS_SPIN_UP_STEP_MAX  (0.10f)
#define CHASSIS_SPIN_STOP_POS_GAIN 1.0f   /* 静止时世界航向位置外环 */
#define CHASSIS_SPIN_STOP_POS_MAX  0.35f  /* 每次位置目标偏移上限 [rad] */

/* 预留:遥控器在小陀螺时改朝向 [rad/s](正的 = 世界系逆时针) */
static volatile float s_spin_yaw_rate = 0.0f;
extern "C" void Chassis_Set_Spin_Yaw_Rate(float __rate)
{
    if (__rate >  6.0f) __rate =  6.0f;
    if (__rate < -6.0f) __rate = -6.0f;
    s_spin_yaw_rate = __rate;
}
static float s_spin_ref    = 0.0f;          /* 小陀螺起始的世界航向(上板 yaw) */
static float s_spin_target = 0.0f;
#define CHASSIS_SPIN_TGT_LIMIT   6.0f
#define CHASSIS_SPIN_SPEED_SIGN  (+1.0f)   /* 外部速度环方向,反了改 -1 */

#if CHASSIS_SERVO_D_INIT_ENABLE
static Servo_D_Init s_servo_init[CHASSIS_D2Q2_UNIT_N];
static bool         s_servo_init_active[CHASSIS_D2Q2_UNIT_N] = {false};
static bool         s_servo_init_ok = true;
static bool         s_rehome_active = false;   /* 是否正在重新找零 */
static bool         s_rehome_hold = false;     /* 鍚屼竴绂荤嚎浜嬩欢鍙嚜鏍′竴娆?*/
static uint32_t     s_steer_offline_ms = 0u;
static bool         s_homing_test_active = false;
static uint32_t     s_homing_test_ms = 0u;
#endif

static Robot_Data_t s_robot;
static uint32_t s_robot_seq = 0u;
static bool s_inited = false;

#if CHASSIS_SERVO_D_INIT_ENABLE
/* 找零重定义了电机坐标，同步运动学零位和 motor_task 的位置目标。 */
static void Chassis_Sync_Steer_Targets(void)
{
    for (uint8_t i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
    {
        if (!Servo_D_Init_Is_Done(&s_servo_init[i])) continue;

        const float zero = s_servo_init[i].final_angle;
        auto def = s_d2q2.Get_Steer_Def(i);
        def.seat_offset = -zero + s_d2q2_seat_trim[i];   /* 标定微调(202 的 10°) */
        s_d2q2.Set_Steer_Def(i, def);

        if (motor_task_is_online(def.motor_index))
        {
            (void)motor_task_set_steer_rad_index(def.motor_index, zero);
        }
    }
}

static void Chassis_Stop_Drive(void)
{
    for (uint8_t i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
    {
        s_data.Wheel_Speed[i] = 0.0f;
        if (motor_task_is_init_ok(s_d2q2_drive[i]))
        {
            (void)motor_task_set_wheel_rpm(s_d2q2_drive[i], 0.0f, CHASSIS_WHEEL_KD);
        }
    }
}
#endif

/* ============================================================================
 * init
 * ========================================================================== */
void Chassis_Init(void)
{
    memset(&s_data, 0, sizeof(s_data));
    memset(&s_cmd, 0, sizeof(s_cmd));
    s_cmd_seq = 0u;
    s_imu_seq = 0u;
    s_last_spin = false;

    s_d2q2.Init(CHASSIS_HALF_A, CHASSIS_HALF_B, CHASSIS_WHEEL_R, s_d2q2_init);
    for (uint8_t i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
    {
        s_d2q2.Set_Steer_Def(i, chassis_D_steer_def{
            s_d2q2_steer[i], CHASSIS_STEER_RATIO_3508BARE_1TO8, +1.0f, 0.0f});
    }

#if CHASSIS_PHOTO_ENABLE
    Electric_Input_Init();
#endif

#if CHASSIS_SERVO_D_INIT_ENABLE
/* 两轮各自独立的找零状态机,同时启动、互不共用状态。
每轮的 Servo_D_Init_t 独立推进,不等另一轮。 */
    s_servo_init_ok = true;
    for (uint8_t i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
    {
        Class_Motor_DJ *m = motor_task_get_dj(s_d2q2_steer[i]);
        s_servo_init_active[i] = false;
        if (m == nullptr) continue;

        Servo_D_Init_Start(&s_servo_init[i], m,
                           s_d2q2_photo[i],
                           s_d2q2_zero_rise[i],
                           s_d2q2_zero_fall[i],
                           +1, 2.0f * 8.0f, s_d2q2_zero_bias[i]);
        s_servo_init_active[i] = true;
    }

    {
        uint32_t last_dbg_ms = 0u;
        for (;;)
        {
            bool any = false;
            uint32_t now_dbg_ms = (uint32_t)Timestamp_Get_Ms();
            if ((uint32_t)(now_dbg_ms - last_dbg_ms) >= 100u)
            {
                last_dbg_ms = now_dbg_ms;
                for (uint8_t d = 0u; d < CHASSIS_D2Q2_UNIT_N; d++)
                {
                    if (!s_servo_init_active[d]) continue;
                    Debug_Send_Homing(d,
                                      motor_task_get_now_pos(s_d2q2_steer[d]),
                                      Electric_Input_Get_Raw(ELECTRIC_IN_PHOTO_1),
                                      Electric_Input_Get_Raw(ELECTRIC_IN_PHOTO_2),
                                      (uint8_t)s_servo_init[d].state);
                }
            }

            for (uint8_t i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
            {
                if (!s_servo_init_active[i]) continue;
                Servo_D_Init_Task(&s_servo_init[i]);
                if (Servo_D_Init_Is_Done(&s_servo_init[i]) ||
                    Servo_D_Init_Is_Error(&s_servo_init[i]))
                {
                    if (Servo_D_Init_Is_Error(&s_servo_init[i]))
                    {
                        s_servo_init_ok = false;
                    }
                    else
                    {
                        /* 找零完成:暂不置零(置零会让 seat 映射偏移,导致原地转圈) */
                    }
                    s_servo_init_active[i] = false;
                }
                else
                {
                    any = true;
                }
            }

/* ★ 已经找零完成的轮子还要继续跑位置环(Calculate+send),
否则它的 PID 输出停在最后一拍,同组广播帧会一直带着这点电流,另一台还在扫的时候它就会自己慢慢转 —— 看起来就是"多转圈"。 */
            for (uint8_t d = 0u; d < CHASSIS_D2Q2_UNIT_N; d++)
            {
                Class_Motor_DJ *m;
                if (s_servo_init_active[d]) continue;
                if (!Servo_D_Init_Is_Done(&s_servo_init[d])) continue;
                m = motor_task_get_dj(s_d2q2_steer[d]);
                if ((m != nullptr) && m->Is_Send_Due())
                {
                    m->Calculate();
                    m->send();
                }
            }

            if (!any) break;
        }
    }
    Chassis_Sync_Steer_Targets();
#endif

#if CHASSIS_SERVO_D_INIT_ENABLE && CHASSIS_HOMING_TEST_ENABLE
    if (s_servo_init_ok)
    {
        s_homing_test_active = true;
        s_homing_test_ms = (uint32_t)Timestamp_Get_Ms();
    }
#endif

    s_data.Unit_Count = 0u;
    s_inited = true;
    s_robot.Motion_Allowed = true;
}

/* ============================================================================
 * task
 * ========================================================================== */
uint8_t Chassis_Homing_State(uint8_t __unit)
{
#if CHASSIS_SERVO_D_INIT_ENABLE
    if (__unit >= CHASSIS_D2Q2_UNIT_N) return 0xFFu;
    return (uint8_t)s_servo_init[__unit].state;
#else
    (void)__unit;
    return 0xFFu;
#endif
}

float Chassis_Homing_Scan(uint8_t __unit)
{
#if CHASSIS_SERVO_D_INIT_ENABLE
    if (__unit >= CHASSIS_D2Q2_UNIT_N) return 0.0f;
    return s_servo_init[__unit].scan_angle;
#else
    (void)__unit;
    return 0.0f;
#endif
}

void Chassis_Task(void)
{
    Cmd_Data_t cmd;
    Imu_Data_t imu;
    Error_Snapshot_t *snap;
    fp32 xyv[3];
    fp32 now_speed[CHASSIS_D2Q2_UNIT_N];
    fp32 now_seat[CHASSIS_D2Q2_UNIT_N];
    uint8_t unit_count;
    uint8_t i;
    float scale;

    if (!s_inited) return;

#if CHASSIS_SERVO_D_INIT_ENABLE
    if (!s_servo_init_ok) return;

/* DJI 反馈 1ms 一帧;任一舵向 202/204 连续 500ms 无反馈 → 判离线
       鎵�鏈夊凡鎺ヨ埖杞噸鏂拌嚜鏍′竴娆°�?*/
    {
        const uint32_t now_ms = (uint32_t)Timestamp_Get_Ms();
        bool any_offline = false;

        for (i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
        {
            if ((s_d2q2_steer[i] < motor_task_get_count()) &&
                !motor_task_is_online(s_d2q2_steer[i]))
            {
                any_offline = true;
            }
        }

        if (!any_offline)
        {
            s_steer_offline_ms = 0u;
            s_rehome_hold = false;
        }
        else if (s_steer_offline_ms == 0u)
        {
            s_steer_offline_ms = now_ms;
        }

        if (!s_rehome_active && !s_rehome_hold && (s_steer_offline_ms != 0u) &&
            ((uint32_t)(now_ms - s_steer_offline_ms) >= 500u))
        {
            for (i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
            {
                Class_Motor_DJ *m = motor_task_get_dj(s_d2q2_steer[i]);
                s_servo_init_active[i] = false;
                if (m != nullptr)
                {
                    Servo_D_Init_Start(&s_servo_init[i], m,
                                       s_d2q2_photo[i],
                                       s_d2q2_zero_rise[i],
                                       s_d2q2_zero_fall[i],
                                       +1, 2.0f * 8.0f, s_d2q2_zero_bias[i]);
                    s_servo_init_active[i] = true;
                }
            }
            s_rehome_active = true;
            s_rehome_hold = true;
            s_steer_offline_ms = 0u;
        }

        if (s_rehome_active)
        {
            Chassis_Stop_Drive();
            bool any = false;
            for (i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
            {
                if (!s_servo_init_active[i]) continue;
                Servo_D_Init_Task(&s_servo_init[i]);
                if (Servo_D_Init_Is_Done(&s_servo_init[i]) ||
                    Servo_D_Init_Is_Error(&s_servo_init[i]))
                {
                    s_servo_init_active[i] = false;
                }
                else
                {
                    any = true;
                }
            }
            if (!any)
            {
                Chassis_Sync_Steer_Targets();
                s_rehome_active = false;
            }
            return;
        }
    }
#endif

#if CHASSIS_SERVO_D_INIT_ENABLE && CHASSIS_HOMING_TEST_ENABLE
    /* 鎵鹃浂瀹屾垚鍚庣殑鏂瑰悜鑷:201 鍙嶈浆 = 杞︿綋 +x,100rpm,鎸佺画 2s */
    if (s_homing_test_active)
    {
        const uint32_t now_ms = (uint32_t)Timestamp_Get_Ms();
        if ((CHASSIS_HOMING_TEST_MS == 0u) ||
            ((uint32_t)(now_ms - s_homing_test_ms) < CHASSIS_HOMING_TEST_MS))
        {
            if ((D2Q2_DRIVE_L < motor_task_get_count()) &&
                motor_task_is_online(D2Q2_DRIVE_L))
            {
                (void)motor_task_set_wheel_rpm(D2Q2_DRIVE_L,
                                               -CHASSIS_HOMING_TEST_RPM,
                                               CHASSIS_WHEEL_KD);
            }
            return;
        }
        (void)motor_task_set_wheel_rpm(D2Q2_DRIVE_L, 0.0f, CHASSIS_WHEEL_KD);
        s_homing_test_active = false;
    }
#endif

    s_data.Seq++;

    if (Msg_Is_New(MSG_TOPIC_ROBOT, s_robot_seq) &&
        Msg_Latest(MSG_TOPIC_ROBOT, &s_robot, (uint16_t)sizeof(s_robot), &s_robot_seq))
    {
    }

    if (Msg_Is_New(MSG_TOPIC_CMD, s_cmd_seq))
    {
        if (Msg_Latest(MSG_TOPIC_CMD, &cmd, (uint16_t)sizeof(cmd), &s_cmd_seq))
        {
            s_cmd = cmd;
        }
    }

    if (!Msg_Is_New(MSG_TOPIC_IMU_CHASSIS, s_imu_seq) ||
        !Msg_Latest(MSG_TOPIC_IMU_CHASSIS, &imu, (uint16_t)sizeof(imu), &s_imu_seq))
    {
        imu = *Imu_Get(IMU_CHANNEL_CHASSIS);
    }

/* 小陀螺 / 云台 yaw 世界航向闭环(建系):
       路 寤虹郴:6020 瀹屾垚闆剁偣鏍″噯(鐩稿瑙?0 = 鏈烘 +Y)閭ｄ竴鍒?璁颁笅搴曠洏 IMU yaw 浣?0,
· 建系:云台世界朝向 = (IMU yaw − yaw0) + SIGN·6020相对角;目标 head_tgt 在这个坐标系里。
         (棰勭暀鎺ュ彛 Chassis_Set_Spin_Yaw_Rate)銆?       路 浣嶇疆鐩爣鐩存帴鍙嶇畻:pos = SIGN路(head_tgt 鈭?(IMU yaw 鈭?yaw0)),涓嶅啀绉垎,
     所以不会累积/跑偏,底盘一转下一拍就给出正确目标。*/
    {
        static float    s_head_tgt = 0.0f;     /* 浜戝彴鐩爣鏈濆悜(鏈湴寤虹郴鍧愭爣) */
static float    s_head_up = 0.0f;      /* 同一目标在上板云台 IMU 坐标系里的值 */
        static float    s_up_off = 0.0f;       /* 两个坐标系的固定偏移 */
        static bool     s_up_ok = false;
        static float    s_yaw_off = 0.0f;      /* 上次有效的上板 yaw 指令 */
        static bool     s_yaw_off_ok = false;
        static bool     s_head_set = false;    /* 鈽?鐩爣鏈濆悜鍙湪绗竴娆¤繘灏忛檧铻烘椂閿佷竴娆?*/
        static float    s_yaw0 = 0.0f;         /* 建系:6020 零位时的底盘 IMU yaw */
        static bool     s_frame_ok = false;
        static bool     s_stop_hold = false;
        static uint32_t s_stop_started = 0u;
        static uint32_t s_stop_still_since = 0u;
        static bool     s_world_hold = false;  /* 杩涜繃涓�娆″皬闄�铻?鈬?涓�鐩撮棴鐜?*/
        static float    s_pos_cont = 0.0f;
        static float    s_pos_last = 0.0f;
        static bool     s_pos_init = false;
        static uint32_t s_6020_ms = 0u;
        const uint32_t now = (uint32_t)Timestamp_Get_Ms();

        if ((uint32_t)(now - s_6020_ms) >= 2u)     /* 500Hz */
        {
            s_6020_ms = now;
            Debug_Send_Steer_Fast();               /* 0x0104:舵向 now/tgt @500Hz */
            const float now_pos = motor_task_aux_get_pos(1u);

            if (!s_pos_init) { s_pos_cont = now_pos; s_pos_last = now_pos; s_pos_init = true; }
            else { s_pos_cont += Math_Angle_Diff_Pi(now_pos, s_pos_last); s_pos_last = now_pos; }

/* ★ 建系:必须在 6020 零点校准完成的那一刻,而不是靠"角度≈0"猜 ——
               绛夋牎鍑嗘妸瀹冪疆 0 鏃?s_pos_cont 宸茬粡甯︾潃杩欎釜鍋忕Щ浜?寤虹郴浼氭案杩滅瓑涓嶅埌銆?*/
            if (!s_frame_ok && motor_task_6020_is_zeroed())
            {
                s_yaw0      = imu.Yaw_Total;
                s_pos_cont  = now_pos;      /* 鏍″噯鍒氬畬鎴?now_pos鈮?,浠庤繖鍎块噸鏂拌繛缁窡韪?*/
                s_pos_last  = now_pos;
                s_pos_init  = true;
                s_frame_ok  = true;
            }

            if (!s_frame_ok)
            {
                (void)motor_task_aux_set_pos_vel_ff(1u, 0.0f);
                (void)motor_task_aux_set_pos_hold(1u, now_pos);   /* 还没建系:原地保持 */
            }
            else
            {
                const float chassis_d = imu.Yaw_Total - s_yaw0;   /* 搴曠洏鑷缓绯昏浆杩囩殑瑙?*/

/* ★ 只在【第一次】进小陀螺时锁目标朝向。
                   鍙杩欐鍋滃緱绋嶅井甯︿竴鐐规粸鍚?涓嬩竴娆″氨鎶婃粸鍚庣殑瑙掑綋鎴愭柊鐩爣,
                   鍙嶅寮�鍏冲氨浼氫竴涓柟鍚戣秺鍋忚秺澶氥�?*/
                if (!s_world_hold && !s_cmd.Chassis_Spin)
                {
                    /* 正常模式也保持世界航向，首次对齐到实际朝向。 */
                    s_head_tgt = chassis_d + CHASSIS_SPIN_HOLD_SIGN * s_pos_cont;
                    s_world_hold = true;
                    s_up_ok = false;
                    s_yaw_off_ok = false;
                }

                if (s_cmd.Chassis_Spin && !s_last_gimbal_spin)
                {
                    s_head_tgt = chassis_d + CHASSIS_SPIN_HOLD_SIGN * s_pos_cont;
                    s_stop_hold = false;
                    s_world_hold = true;
                    s_up_ok    = false;
                    s_yaw_off_ok = false;
                }

                if (s_world_hold && !s_cmd.Chassis_Spin && s_last_gimbal_spin)
                {
                    s_stop_hold = true;
                    s_stop_started = now;
                    s_stop_still_since = 0u;
                }

                if (s_stop_hold)
                {
                    const bool upper_fresh = ((uint32_t)(now - Cmd_485_Get_Upper_Yaw_Ms()) < 50u);
                    const float upper_error = Math_Angle_Diff_Pi(s_head_up, Cmd_485_Get_Upper_Yaw());
                    const bool settled = imu.Online && fabsf(imu.Gyro[2]) < 0.2f &&
                                         (!s_up_ok || !upper_fresh || fabsf(upper_error) < 0.04f);
                    if (!settled) s_stop_still_since = 0u;
                    else if (s_stop_still_since == 0u) s_stop_still_since = now;

                    if ((s_stop_still_since != 0u && (uint32_t)(now - s_stop_still_since) >= 100u) ||
                        (uint32_t)(now - s_stop_started) >= 2000u)
                    {
                        s_stop_hold = false;
                        s_yaw_off_ok = false;
                    }
                }

                if (s_world_hold && !s_cmd.Chassis_Spin && !s_stop_hold)
                {
                    const bool command_fresh =
                        ((uint32_t)(now - Cmd_485_Get_Upper_Yaw_Ms()) < 50u);
                    if (command_fresh)
                    {
                        const float yaw_cmd = Cmd_485_Get_Gimbal_Yaw_Cmd();
                        if (s_yaw_off_ok)
                        {
                            const float delta = Math_Angle_Diff_Pi(yaw_cmd, s_yaw_off);
                            s_head_tgt += CHASSIS_SPIN_HOLD_SIGN * delta;
                            if (s_up_ok) s_head_up += CHASSIS_SPIN_HOLD_SIGN * delta;
                        }
                        s_yaw_off = yaw_cmd;
                        s_yaw_off_ok = true;
                    }
                    else
                    {
                        s_yaw_off_ok = false;
                    }
                }

                if (s_world_hold)
                {
                    /* 各模式保持世界航向，正常模式的遥控增量改变目标。 */
                    const float rate = s_cmd.Chassis_Spin ? s_spin_yaw_rate * 0.002f : 0.0f;
                    s_head_tgt += rate;

                    const bool up_fresh =
                        ((uint32_t)(now - Cmd_485_Get_Upper_Yaw_Ms()) < 50u);

/* 上板云台 IMU 做慢速修正,把底盘 IMU 转多圈累积的误差压掉。 */
                    if (!s_up_ok && up_fresh)
                    {
                        s_head_up = Cmd_485_Get_Upper_Yaw();
                        s_up_ok   = true;
                    }

                    /* 鏈湴寤虹郴缁欎富浣撶洰鏍?瀵瑰簳鐩樿浆鍔ㄥ嵆鏃跺搷搴?:
                       pos = SIGN路(head 鈭?(IMU yaw 鈭?yaw0)) 鈭?SIGN路蠅路T_lead */
                    float pos_cmd =
                        CHASSIS_SPIN_HOLD_SIGN * (s_head_tgt - chassis_d);

                    /* 速度前馈:位置环的 Kp 太小,靠误差追不上 10rad/s;
                       直接把"云台该反向转多快"(-SIGN·ω)喂给 6020 速度环,
                       再加 α·τ 补启停瞬态。位置环只负责修残差。
                       α 用低通后的值并限幅,防止陀螺噪声在停车瞬间踢一下。 */
                    {
                        static float s_gz_prev = 0.0f;
                        static float s_alpha_f = 0.0f;
                        float alpha = (imu.Gyro[2] - s_gz_prev) / 0.002f;
                        s_gz_prev = imu.Gyro[2];
                        if (alpha > 200.0f) alpha = 200.0f;
                        if (alpha < -200.0f) alpha = -200.0f;
                        s_alpha_f += 0.20f * (alpha - s_alpha_f);

                        float w_ff = -CHASSIS_SPIN_HOLD_SIGN *
                                     (imu.Gyro[2] + (s_cmd.Chassis_Spin ? CHASSIS_SPIN_FF_TAU * s_alpha_f : 0.0f));
                        if (w_ff >  15.0f) w_ff =  15.0f;
                        if (w_ff < -15.0f) w_ff = -15.0f;
                        (void)motor_task_aux_set_pos_vel_ff(1u, w_ff);
                    }

/* 上板云台 IMU 做慢速修正:把"底盘 IMU 转多圈累积的误差"压掉。 */
                    if (s_up_ok && up_fresh)
                    {
                        const float gz_abs = (imu.Gyro[2] > 0.0f) ? imu.Gyro[2] : -imu.Gyro[2];
                        const float up_gain = (gz_abs > 0.3f) ? CHASSIS_SPIN_UP_GAIN
                                                             : CHASSIS_SPIN_UP_GAIN_STILL;

                        s_head_up += rate;
                        const float up_err = Math_Angle_Diff_Pi(
                            s_head_up, Cmd_485_Get_Upper_Yaw());
                        /* guard: |up_err| near pi means the frame alignment is wrong; noise flips its sign
                            and the target would jump by 2*gain*pi for one tick. Only trim small errors,
                            and clamp the per-tick correction so a sign flip can never jump a full turn. */
                        if ((up_err < 1.5f) && (up_err > -1.5f))
                        {
                            if (!s_cmd.Chassis_Spin || gz_abs < 0.3f)
                            {
                                /* 正常模式连续使用实测世界航向位置外环，避免停车时切换目标。 */
                                float offset = -CHASSIS_SPIN_UP_SIGN * up_err *
                                               CHASSIS_SPIN_STOP_POS_GAIN;
                                if (offset > CHASSIS_SPIN_STOP_POS_MAX) offset = CHASSIS_SPIN_STOP_POS_MAX;
                                if (offset < -CHASSIS_SPIN_STOP_POS_MAX) offset = -CHASSIS_SPIN_STOP_POS_MAX;
                                pos_cmd = s_pos_cont + offset;
                            }
                            else
                            {
                                float corr = -CHASSIS_SPIN_UP_SIGN * up_err * up_gain;
                                if (corr > CHASSIS_SPIN_UP_STEP_MAX) corr = CHASSIS_SPIN_UP_STEP_MAX;
                                if (corr < -CHASSIS_SPIN_UP_STEP_MAX) corr = -CHASSIS_SPIN_UP_STEP_MAX;
                                pos_cmd += corr;
                            }
                        }
                    }

                    (void)motor_task_aux_set_pos_hold(1u, pos_cmd);
                }

            }
        }
    }
    s_last_gimbal_spin = s_cmd.Chassis_Spin;

    if (!s_robot.Motion_Allowed)
    {
        s_cmd.Chassis_Vx = 0.0f;
        s_cmd.Chassis_Vy = 0.0f;
        s_cmd.Chassis_W = 0.0f;
    }

#if CHASSIS_BOOT_TEST_ENABLE
    /* 上电找零后先沀yaw=0 / +y 方向赀3s,100rpm,方便观察 */
    if (!s_boot_test_done)
    {
        const uint32_t now = (uint32_t)Timestamp_Get_Ms();
        if (s_boot_test_ms == 0u) s_boot_test_ms = now;
        if ((uint32_t)(now - s_boot_test_ms) < CHASSIS_BOOT_TEST_MS)
        {
            s_cmd.Mode = CMD_MODE_REMOTE;
            s_cmd.Chassis_Vx = 0.0f;
            s_cmd.Chassis_Vy = CHASSIS_BOOT_TEST_VY;
            s_cmd.Chassis_W = 0.0f;
            s_cmd.Chassis_Spin = false;
            s_boot_test_run = true;
        }
        else
        {
            s_boot_test_done = true;
            s_boot_test_run = false;
        }
    }
#endif

    /* 正常模式锁住入场航向，小陀螺沿用同一个地面方向基准。 */
    static bool drive_frame_locked = false;
    static float drive_frame_yaw = 0.0f;   /* XY 指令基准(云台朝向) */
    static float heading_yaw = 0.0f;       /* 航向闭环目标(切换时的底盘 yaw) */
    static float heading_integral = 0.0f;
    const bool yaw_valid = !s_boot_test_run && s_robot.Motion_Allowed &&
                           s_cmd.Mode != CMD_MODE_IDLE && imu.Online && imu.Converged;
    if (!yaw_valid)
    {
        drive_frame_locked = false;
    }
    else
    {
        if (!drive_frame_locked || (s_last_spin && !s_cmd.Chassis_Spin))
        {
            /* 小陀螺退出时: 基准取【云台朝向】(底盘 yaw + 6020 相对角),
               保证正常模式下推 y+ 就是云台方向 */
            drive_frame_yaw = Math_Angle_Wrap_2Pi(imu.Yaw_Total + motor_task_aux_get_pos(1u));
            heading_yaw = Math_Angle_Wrap_2Pi(imu.Yaw_Total);   /* 归一化到 0~2π,避免超圈 */
        }
        if (s_cmd.Chassis_Spin && (!drive_frame_locked || !s_last_spin))
        {
            s_d2q2.Set_Top_Yaw(drive_frame_yaw);
        }
        drive_frame_locked = true;
    }
    s_last_spin = s_cmd.Chassis_Spin;

    float yaw_error = drive_frame_locked
                          ? Math_Angle_Diff_Pi(heading_yaw, imu.Yaw_Total)
                          : 0.0f;
    const bool heading_enabled = drive_frame_locked && !s_cmd.Chassis_Spin;
    if (!heading_enabled)
    {
        heading_integral = 0.0f;
    }
    /* ★ 航向目标永不重锁:车被转过多少,就按 CHASSIS_HEADING_CORR_RATIO 的比例慢慢转回来。*/

    const float translation_sq = s_cmd.Chassis_Vx * s_cmd.Chassis_Vx +
                                 s_cmd.Chassis_Vy * s_cmd.Chassis_Vy;
    float chassis_w = s_cmd.Chassis_W;
    if (heading_enabled && translation_sq > 0.0f &&
        fabsf(s_cmd.Chassis_W) <= CHASSIS_HEADING_W_DEADBAND)
    {
        const float pd = CHASSIS_HEADING_KP * yaw_error - CHASSIS_HEADING_KD * imu.Gyro[2];
        const float limit = fminf(CHASSIS_HEADING_W_MAX, CHASSIS_HEADING_CORR_RATIO *
                                                        sqrtf(translation_sq) / CHASSIS_HALF_DIAG);
        const float correction = pd + heading_integral;
        /* 限幅后只允许积分向退出饱和的方向变化。 */
        if (fabsf(correction) < limit || correction * yaw_error < 0.0f)
        {
            const float dt = CHASSIS_TASK_PERIOD_MS * 0.001f;
            heading_integral += CHASSIS_HEADING_KI * yaw_error * dt;
            LimitMax(heading_integral, CHASSIS_HEADING_I_MAX);
        }
        chassis_w += CHASSIS_IMU_TO_KINEMATIC_YAW *
                     fmaxf(-limit, fminf(limit, pd + heading_integral));
    }
    g_heading_error = yaw_error;          /* 遥测:重锁后为 0 */
    g_chassis_w_cmd = chassis_w;          /* 遥测:真正送进运动学的 ω */

    /* 反解、到位判定和电机目标共用真实 CAN 反馈位置。 */
    Motor_Steer_Feedback_t steer_feedback[CHASSIS_D2Q2_UNIT_N];
    bool steer_available[CHASSIS_D2Q2_UNIT_N] = {};
    const uint8_t motor_count = motor_task_get_count();
    for (i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
    {
        steer_available[i] = motor_task_get_steer_feedback(s_d2q2_steer[i], &steer_feedback[i]);
#if CHASSIS_SERVO_D_INIT_ENABLE
        steer_available[i] = steer_available[i] && Servo_D_Init_Is_Done(&s_servo_init[i]);
#endif
        if (steer_available[i])
        {
            g_seat_now[i] = s_d2q2.Motor_To_Seat(i, steer_feedback[i].position_rad);
            s_d2q2.Set_Seat_Now(i, g_seat_now[i]);
        }
        else
        {
            s_d2q2.Invalidate_Steer_Feedback(i);
        }
        now_seat[i] = g_seat_now[i];
    }

    /* 摇杆 XY 以云台为参考；加上云台角（同向）后转换到底盘坐标。 */
    const bool translation_allowed = motor_task_aux_online(1u);   /* 只要在线就用实测角,不强制要求已标零 */
    const float drive_yaw = translation_allowed
                                ? Math_Angle_Wrap_Pi(CHASSIS_STICK_TO_BODY_YAW +
                                                     CHASSIS_GIMBAL_POS_SIGN *
                                                         motor_task_aux_get_pos(1u))
                                : 0.0f;
    const float drive_cos = arm_cos_f32(drive_yaw);
    const float drive_sin = arm_sin_f32(drive_yaw);
    xyv[0] = 0.0f;
    xyv[1] = 0.0f;
    if (translation_allowed)
    {
        xyv[0] = s_cmd.Chassis_Vx * drive_cos - s_cmd.Chassis_Vy * drive_sin;
        xyv[1] = s_cmd.Chassis_Vx * drive_sin + s_cmd.Chassis_Vy * drive_cos;
    }
    xyv[2] = chassis_w;
    s_d2q2.count_run1(xyv);

    unit_count = 0u;
    for (i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
    {
        if ((s_d2q2_drive[i] < motor_task_get_count()) &&
            motor_task_is_init_ok(s_d2q2_drive[i]))
        {
            unit_count++;
        }
    }
    s_data.Unit_Count = unit_count;

    for (i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
    {
        now_speed[i] = motor_task_get_wheel_rpm(s_d2q2_drive[i]) * s_d2q2_drive_sign[i];
        s_data.Wheel_Now_Speed[i] = now_speed[i];
        s_data.Wheel_Now_Speed[i + 2u] = 0.0f;   /* passive omni wheels */
    }
    s_d2q2.forward_count(now_seat, now_speed);
    s_data.Vx = s_d2q2.Get_Vx();
    s_data.Vy = s_d2q2.Get_Vy();
    s_data.W = s_d2q2.Get_Vw();

    /* power estimate: all four motors, including steering */
    s_data.Power_Limit = s_cmd.Chassis_Power_Limit;
    s_data.Power_Est = 0.0f;
    s_data.Power_Est_Current = 0.0f;
    {
        const uint8_t all_motors[4] = {D2Q2_STEER_L, D2Q2_DRIVE_L,
                                       D2Q2_STEER_R, D2Q2_DRIVE_R};
        for (i = 0u; i < 4u; i++)
        {
            float t = motor_task_get_now_torque(all_motors[i]);
            float w = motor_task_get_now_w(all_motors[i]);
            s_data.Power_Est += (t < 0.0f) ? (-t * ((w < 0.0f) ? -w : w))
                                           : (t * ((w < 0.0f) ? -w : w));
        }
    }
    s_data.Power_Est = CHASSIS_IDLE_POWER + s_data.Power_Est / CHASSIS_DRIVE_EFF;

#if CHASSIS_USE_C620_CURRENT
    {
        float vbat = Power_Get_Voltage();
        float isum = 0.0f;
        const uint8_t all_motors[4] = {D2Q2_STEER_L, D2Q2_DRIVE_L,
                                       D2Q2_STEER_R, D2Q2_DRIVE_R};
        for (i = 0u; i < 4u; i++)
        {
            float cur = motor_task_get_now_current(all_motors[i]);
            isum += (cur < 0.0f) ? -cur : cur;
        }
        if (vbat < 10.0f) vbat = 24.0f;
        s_data.Power_Est_Current = vbat * isum * CHASSIS_CURRENT_BUS_SCALE;
        if (s_data.Power_Est_Current > s_data.Power_Est)
        {
            s_data.Power_Est = s_data.Power_Est_Current;
        }
    }
#endif

    scale = 1.0f;
#if CHASSIS_POWER_LIMIT_ENABLE
    if (((float)s_data.Power_Limit > (CHASSIS_POWER_MARGIN + 1.0f)) &&
        (s_data.Power_Est > ((float)s_data.Power_Limit - CHASSIS_POWER_MARGIN)))
    {
        float over = s_data.Power_Est /
                     ((float)s_data.Power_Limit - CHASSIS_POWER_MARGIN);
        scale = 1.0f - CHASSIS_POWER_KP * (over - 1.0f);
        if (scale < 0.0f) scale = 0.0f;
    }
#endif
    if ((s_cmd.Chassis_Power_Buffer != 0u) &&
        (s_cmd.Chassis_Power_Buffer < CHASSIS_BUFFER_LOW))
    {
        scale *= 0.5f;
    }

    /* 已配置的驱动轮须全部有舵角反馈且到位，才一起放行。 */
    bool ready_all = true;
    for (i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
    {
        if ((s_d2q2_drive[i] < motor_count) &&
            (!steer_available[i] || !s_d2q2.Is_Steer_Ready(i)))
        {
            ready_all = false;
        }
    }

    static SteerTrackingLead steer_lead[CHASSIS_D2Q2_UNIT_N];
    const uint32_t lead_now_ms = (uint32_t)Timestamp_Get_Ms();
    for (i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
    {
        const float rpm = ready_all ? s_d2q2.Get_Speed(i) * scale : 0.0f;
        const float seat = s_d2q2.Get_Seat(i);
        const float wheel_rpm = rpm * s_d2q2_drive_sign[i];

        s_data.Wheel_Speed[i] = rpm;
        s_data.Wheel_Speed[i + 2u] = 0.0f;
        s_data.Wheel_Seat[i] = seat;
        s_data.Wheel_Seat[i + 2u] = 0.0f;

        if ((s_d2q2_drive[i] < motor_count) &&
            motor_task_is_init_ok(s_d2q2_drive[i]))
        {
            (void)motor_task_set_wheel_rpm(s_d2q2_drive[i],
                                           wheel_rpm,
                                           CHASSIS_WHEEL_KD);
        }
        const bool lead_enabled = s_cmd.Chassis_Spin && s_robot.Motion_Allowed &&
                                  translation_allowed && translation_sq > 0.01f && ready_all &&
                                  steer_available[i] && !motor_task_steer_debug_enabled();
        const float control_seat = steer_lead[i].Update(lead_enabled, seat, now_seat[i], lead_now_ms);
        g_seat_tgt[i] = Math_Angle_Wrap_2Pi(control_seat); /* telemetry reports the applied target */
        if (steer_available[i] && !motor_task_steer_debug_enabled())
        {
            const float target = steer_feedback[i].position_rad + s_d2q2.Seat_To_Motor_Delta(i, control_seat);
            (void)motor_task_set_steer_rad_index(s_d2q2_steer[i], Math_Angle_Wrap_2Pi(target));
        }
    }

    /* 上电测试: 舵向直接 0°,驱动直接 100 rpm */
    if (s_boot_test_run)
    {
        for (i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
        {
            if ((s_d2q2_steer[i] < motor_task_get_count()) &&
                motor_task_is_init_ok(s_d2q2_steer[i]))
            {
                (void)motor_task_set_steer_rad_index(s_d2q2_steer[i], CHASSIS_BOOT_TEST_YAW);
            }
            if ((s_d2q2_drive[i] < motor_task_get_count()) &&
                motor_task_is_init_ok(s_d2q2_drive[i]))
            {
                (void)motor_task_set_wheel_rpm(s_d2q2_drive[i], 100.0f, CHASSIS_WHEEL_KD);
            }
        }
    }

    s_data.Yaw_Total = imu.Yaw_Total;
    s_data.Top_Yaw = s_d2q2.Get_Top_Yaw();
    s_data.Mode = s_cmd.Mode;
    s_data.Spin = s_cmd.Chassis_Spin;
    s_data.Online = (unit_count > 0u) && imu.Online;
    (void)Msg_Publish(MSG_TOPIC_CHASSIS, &s_data, (uint16_t)sizeof(s_data));

    snap = Error_Get_Snapshot();
    snap->Chassis_Cmd[0] = s_cmd.Chassis_Vx;
    snap->Chassis_Cmd[1] = s_cmd.Chassis_Vy;
    snap->Chassis_Cmd[2] = chassis_w;
    snap->Chassis_Now[0] = s_data.Vx;
    snap->Chassis_Now[1] = s_data.Vy;
    snap->Chassis_Now[2] = s_data.W;
    snap->Mode = s_cmd.Mode;
}

const Chassis_Data_t *Chassis_Get(void)
{
    return &s_data;
}

bool Chassis_Is_Online(void)
{
    return s_data.Online;
}

float Chassis_Get_Wheel_Rpm(uint8_t __unit)
{
    if (__unit >= CHASSIS_UNIT_N) return 0.0f;
    return s_data.Wheel_Speed[__unit];
}

void Chassis_Set_Top_Yaw_Now(void)
{
    s_d2q2.Set_Top_Yaw(Imu_Get_Yaw_Total(IMU_CHANNEL_CHASSIS));
}
