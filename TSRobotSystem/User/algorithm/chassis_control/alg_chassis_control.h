//
// Created by heartpain on 2026/10/11.
//

#ifndef ALG_CHASSIS_CONTROL_H
#define ALG_CHASSIS_CONTROL_H

#include "struct_typedef.h"
#include "alg_math_angle.h"     /* ★ 舵轮的 2π 归一/最短弧 ✓ */

#define CHASSIS_D_ARC_OPT       1            /* 1=开启 D2Q2 优弧翻转 */
#define CHASSIS_D_FLIP_THRESH   (PI / 2.0f)  /* D4 优弧翻转阈值 */
#define CHASSIS_D_READY_THRESH  (PI / 6.0f)  /* 残差达到 30° 时停止驱动 */

typedef enum{
    CHASSIS_A = 0,
    CHASSIS_B = 1
}chassis_M_angle;

struct chassis_M_wheel_init
{
    chassis_M_angle angle;
    int8_t wheel_xy[2];
};

struct chassis_M_wheel
{
    fp32 speed;
    chassis_M_angle wheel;
    fp32 XY_angle;
    int8_t xy[2];           /* 本轮在机体系里的方位(±1,由 Init 归一化而来) */
                            /* ★ 速度符号规则:**X 正半轴的轮子为正,负半轴为负** */
                            /*   (以前是写死的 chassis[2]/[3] 取负,现在由这个方位自动决定) */
};

class Class_chassis_M4{

public:

    void Init(const fp32 __a,const fp32 __b,const fp32 __r,chassis_M_wheel_init __init[4]);

    void count_run1(const fp32 __X_Y_W[3]);

    void count_run2(const fp32 __X_Y_W[3],const fp32 __yaw);

    void forward_count(const fp32 __wheel_speed[4]);

    void Set_Top_Yaw(const fp32 __top_yaw);          /* 小陀螺启动时:记下此刻 yaw 作 0 位(内部会归一到 [0,2π)) */

    inline fp32 Get_Top_Yaw() const;


    inline fp32 Get_Speed(const uint8_t &__wheel) const;

    inline fp32 Get_Vx() const;
    inline fp32 Get_Vy() const;
    inline fp32 Get_Vw() const;

protected:
    fp32 x;
    fp32 y;
    fp32 w;
    fp32 r;
    fp32 a;
    fp32 b;
    fp32 D;
    /* ======== 小陀螺的 yaw 零位(★ 很重要,别删) ========
       top_yaw 记录"**启动小陀螺那一刻**"的 yaw 值,作为小陀螺模式的 0 位。
       · **不是**"上电时的 0 位" —— 上电零位只是 IMU 当时的随机朝向,拿它当基准,
         小陀螺一开,车会朝一个没意义的方向跑;
       · 用法:按小陀螺键的那一刻 Set_Top_Yaw(当前yaw) 对齐一次,
         之后 count_run2 会自动用 (yaw - top_yaw) 去做坐标旋转 ✓ */
    fp32 top_yaw;

    chassis_M_wheel chassis[4];

};


/* 读小陀螺的 0 位(调试/上位机显示用) */
inline fp32 Class_chassis_M4::Get_Top_Yaw() const {
    return top_yaw;
}

inline fp32 Class_chassis_M4::Get_Speed(const uint8_t &__wheel) const{
    return chassis[__wheel].speed;
}

inline fp32 Class_chassis_M4::Get_Vx() const {
    return x;
}
inline fp32 Class_chassis_M4::Get_Vy() const {
    return y;
}
inline fp32 Class_chassis_M4::Get_Vw() const {
    return w;
}

/* ============================================================================
 * 舵轮(独立转向)底盘 —— 与上面的 Class_chassis_M4(麦轮)并列
 *
 * 命名沿用你和参考工程的规律:M = 麦轮(mecanum)、D = 舵轮(steering)。
 * 参考:E:\rm\chassis_task.c 的 chassisMath_D_chassis / chassis_math_D_angle_mapping
 *
 * 每个轮子 = 1 个转向电机 + 1 个驱动电机,所以反解要给出:
 *     · seat  : 转向目标角 [rad],∈ [0, 2π)
 *     · speed : 驱动目标转速 [rpm](与 Class_chassis_M4 的单位一致;
 *               若电机层要 rad/s,乘 π/30 即可)
 *
 * ★ 舵轮相对麦轮多出来的三件事(都抄自参考工程,见 .md 第 3 节):
 *     ① 指令为 0 时**保持上次转向角**(不然轮子会乱甩);
 *     ② 目标角与当前角差 > 90° ⇒ **翻转 180° 并把轮速取反**(转向电机少转半圈);
 *     ③ 转向还没到位(残差 ≥ 45°)⇒ **先不给驱动速度**(防轮胎"搓地")。
 *   为此必须**每拍把当前实际转向角喂进来**: Set_Seat_Now(i, 实测角)。
 * ========================================================================== */

/* 一个舵轮的装配参数(和 chassis_M_wheel_init 同一个位置) */
struct chassis_D_wheel_init
{
    fp32 x;                 /* 轮心在机体系的位置 X [m](车体中心为原点) */
    fp32 y;                 /* 轮心在机体系的位置 Y [m] */
    fp32 seat_offset;       /* 舵向零位偏移 [rad](装配标定值,本层只记录,配合 Set_Seat_Now 使用) */
};

/* 一个舵轮的运行状态(和 chassis_M_wheel 同一个位置) */
struct chassis_D_wheel
{
    fp32 x;                 /* 本轮的轮心位置 X [m] */
    fp32 y;                 /* 本轮的轮心位置 Y [m] */
    fp32 seat_offset;       /* 本轮舵向零位偏移 [rad](标定值,供上层换算编码器零位) */
    fp32 speed;             /* 目标驱动转速 [rpm](可为负 = 反着转) */
    fp32 seat;              /* 目标转向角 [rad] ∈ [0, 2π) */
    fp32 seat_last;         /* 上一次的目标转向角(指令为 0 时沿用它) */
    fp32 seat_now;          /* 当前实际转向角 [rad](由 Set_Seat_Now 喂) */
    fp32 V_X;               /* 该轮心的期望速度矢量(调试用) */
    fp32 V_Y;
};

/* 舵向总减速比预设(仅记录,不参与运算)
   ★ 用枚举而不是宏:配置表只能选预设,拼错会编译报错 */
enum Enum_Chassis_Steer_Ratio
{
    CHASSIS_STEER_RATIO_6020_DIRECT   = 1,   /* 6020 直驱:1:1 */
    CHASSIS_STEER_RATIO_3508BARE_1TO4 = 4,   /* 3508 屁股 1:1 × 外部 1:4 */
    CHASSIS_STEER_RATIO_3508BARE_1TO8 = 8,   /* 3508 屁股 1:1 × 外部 1:8 */
};

/* 需要数值时用这个转换(例如日志/调试);本层运算不依赖它 */
inline fp32 Chassis_Steer_Ratio_To_Fp32(Enum_Chassis_Steer_Ratio __ratio)
{
    return (fp32)__ratio;
}

/* 舵向定义:转向电机下标/总减速比/方向/零位(详见 舵轮_舵向定义.md) */
struct chassis_D_steer_def
{
    uint8_t motor_index;                    /* 舵向电机在电机注册表的下标(0xFF = 该轮不驱动) */
    Enum_Chassis_Steer_Ratio reduce_ratio;  /* 总减速比预设(仅记录) */
    fp32    dir;                            /* 装配方向:+1.0 / -1.0(电机正转时轮子反着转就填 -1) */
    fp32    seat_offset;                    /* 零位偏移 [rad](装配标定,配合 PE2 光电找零使用) */
};

class Class_chassis_D4{
public:

    /**
     * @brief 初始化
     * @param __a 半轴距(a/2)[m]   (只用于里程计里的 r² = a²+b²)
     * @param __b 半轮距(b/2)[m]
     * @param __r 轮半径 [m]
     * @param __init 4 个轮的装配参数(x/y/零位/正反装)
     */
    void Init(const fp32 __a, const fp32 __b, const fp32 __r, const chassis_D_wheel_init __init[4]);

    /** 反解:机体系速度 (vx, vy, ω) → 每轮 seat + speed */
    void count_run1(const fp32 __X_Y_W[3]);

    /** 反解(世界系/小陀螺):指令先按 yaw 旋到机体系,再送 count_run1 那套 */
    void count_run2(const fp32 __X_Y_W[3], const fp32 __yaw);

    /** 正解(里程计):由 4 个轮的实测(转向角 rad + 转速 rpm)反推车体 (vx, vy, ω) */
    void forward_count(const fp32 __seat_now[4], const fp32 __speed_now[4]);

    /** 喂某个轮子的当前实际转向角 [rad](每拍调用;不喂也能跑,但就近翻转会失效) */
    inline void Set_Seat_Now(const uint8_t &__wheel, const fp32 __seat_now);

    void Set_Top_Yaw(const fp32 __top_yaw);          /* 小陀螺启动时:记下此刻 yaw 作 0 位(内部会归一到 [0,2π)) */

    inline fp32 Get_Top_Yaw() const;

    inline fp32 Get_Speed(const uint8_t &__wheel) const;
    inline fp32 Get_Seat(const uint8_t &__wheel) const;

    /* ---- ★ 舵向定义(新增) ---- */
/* ---- ★ 舵向定义(新增):电机下标/减速比/方向/零位 ---- */

    /** 设置某轮舵向定义(越界丢弃) */
    void Set_Steer_Def(const uint8_t &__wheel, const chassis_D_steer_def &__def){
        if(__wheel < 4u){
            steer[__wheel] = __def;
        }
    }

    /** 取某轮舵向定义(越界返回 0 号) */
    inline const chassis_D_steer_def &Get_Steer_Def(const uint8_t &__wheel) const{
        return steer[(__wheel < 4u) ? __wheel : 0u];
    }

    /** 电机侧角 → 轮侧角 [rad],归一到 [0, 2π) */
    inline fp32 Motor_To_Seat(const uint8_t &__wheel, const fp32 __motor_rad) const{
        const chassis_D_steer_def &d = Get_Steer_Def(__wheel);
        const fp32 dir = (d.dir < 0.0f) ? (-1.0f) : (1.0f);
        /* 2π 归一:把多圈值折回 [0, 2π) */
        return Math_Angle_Wrap_2Pi(dir * __motor_rad + d.seat_offset);
    }

    /** 轮侧目标 → 电机侧**增量**(最短弧,抗 2π→0) */
    inline fp32 Seat_To_Motor_Delta(const uint8_t &__wheel, const fp32 __seat_cmd) const{
        const chassis_D_steer_def &d = Get_Steer_Def(__wheel);
        const fp32 dir = (d.dir < 0.0f) ? (-1.0f) : (1.0f);
        const fp32 now = chassis[(__wheel < 4u) ? __wheel : 0u].seat_now;   /* 轮侧实测角(Set_Seat_Now 喂) */

        /* ★ 优劣弧:最短带符号夹角 ∈ (-π, π] */
        return dir * Math_Angle_Diff_Pi(__seat_cmd, now);
    }

    inline fp32 Get_Vx() const;
    inline fp32 Get_Vy() const;
    inline fp32 Get_Vw() const;

protected:
    fp32 x;                 /* 车体 x 速度(正解结果/调试) */
    fp32 y;
    fp32 w;
    fp32 r;                 /* 轮半径 */
    chassis_D_steer_def steer[4];   /* ★ 四个轮的舵向定义(Set_Steer_Def 填) */
    fp32 a;                 /* 半轴距 */
    fp32 b;                 /* 半轮距 */
    /* ======== 小陀螺的 yaw 零位(★ 很重要,别删) ========
       top_yaw 记录"**启动小陀螺那一刻**"的 yaw 值,作为小陀螺模式的 0 位。
       · **不是**"上电时的 0 位" —— 上电零位只是 IMU 当时的随机朝向,拿它当基准,
         小陀螺一开,车会朝一个没意义的方向跑;
       · 用法:按小陀螺键的那一刻 Set_Top_Yaw(当前yaw) 对齐一次,
         之后 count_run2 会自动用 (yaw - top_yaw) 去做坐标旋转 ✓ */
    fp32 top_yaw;
    chassis_D_wheel chassis[4];
};

inline void Class_chassis_D4::Set_Seat_Now(const uint8_t &__wheel, const fp32 __seat_now) {
    chassis[__wheel].seat_now = __seat_now;
}

inline fp32 Class_chassis_D4::Get_Top_Yaw() const {
    return top_yaw;
}

inline fp32 Class_chassis_D4::Get_Speed(const uint8_t &__wheel) const {
    return chassis[__wheel].speed;
}

inline fp32 Class_chassis_D4::Get_Seat(const uint8_t &__wheel) const {
    return chassis[__wheel].seat;
}

inline fp32 Class_chassis_D4::Get_Vx() const {
    return x;
}
inline fp32 Class_chassis_D4::Get_Vy() const {
    return y;
}
inline fp32 Class_chassis_D4::Get_Vw() const {
    return w;
}

/* ============================================================================
 * 2 舵轮 + 2 全向轮(D2Q2)
 *
 * 构造(俯视,上 = 车头):
 *
 *        D   Q
 *        Q   D
 *
 *   D = 舵轮(转向 + 驱动,有动力)
 *   Q = 全向轮(被动支撑,无动力、无编码器)
 *
 * 只有 2 个 D 轮参与运动学:
 *   · 反解:v_i = v + ω × r_i ⇒ 每个 D 轮得到 (seat, speed)
 *   · 正解:2 个 D 轮的实测速度矢量(方向 = seat,大小 = rpm·π/30·r)
 *           联立 v_i = v + ω × r_i,最小二乘解出 (vx, vy, ω)
 *   · Q 轮只承重,不参与解算 ⇒ 不需要电机下标/编码器
 *
 * 几何约定:x 向前为正,y 向左为正,ω 逆时针为正。
 * D 轮位置由 Init 的 init[2] 给;Q 轮位置不影响解算(通常就是另外两个角)。
 * ========================================================================== */
#define CHASSIS_D2Q2_UNIT_N 2u

class Class_chassis_D2Q2
{
public:
    /**
     * @brief 初始化
     * @param __a 半轴距 [m](仅记录/诊断)
     * @param __b 半轮距 [m](仅记录/诊断)
     * @param __r 轮半径 [m]
     * @param __init 2 个 D 轮的装配参数(x/y/零位)
     */
    void Init(const fp32 __a, const fp32 __b, const fp32 __r,
              const chassis_D_wheel_init __init[CHASSIS_D2Q2_UNIT_N]);

    /** 反解:机体系 (vx[m/s], vy[m/s], ω[rad/s]) → 2 个 D 轮的 (seat[rad], speed[rpm]) */
    void count_run1(const fp32 __X_Y_W[3]);

    /** 反解(小陀螺):指令先按 yaw-top_yaw 旋到机体系,再走 count_run1 */
    void count_run2(const fp32 __X_Y_W[3], const fp32 __yaw);

    /**
     * @brief 正解(里程计):2 个 D 轮的实测 (转向角, 轮速) → 车体 (vx, vy, ω)
     * @note 每个 D 轮的速度矢量 = speed[rpm]·π/30·r,方向 = seat_now;
     *       两个点速度联立刚体速度场,解出车体 twist。
     */
    void forward_count(const fp32 __seat_now[CHASSIS_D2Q2_UNIT_N],
                       const fp32 __speed_now[CHASSIS_D2Q2_UNIT_N]);

    /** 更新 D 轮最后一次有效实测舵角 [rad]；首次反馈前禁止驱动。 */
    inline void Set_Seat_Now(const uint8_t &__wheel, const fp32 __seat_now);

    void Invalidate_Steer_Feedback(uint8_t __wheel)
    {
        if (__wheel < CHASSIS_D2Q2_UNIT_N)
        {
            steering[__wheel] = SteeringState{};
            chassis[__wheel].speed = 0.0f;
        }
    }

    void Set_Top_Yaw(const fp32 __top_yaw);       /* 小陀螺启动时记 0 位(内部归一到 [0,2π)) */

    inline fp32 Get_Top_Yaw() const;
    inline fp32 Get_Speed(const uint8_t &__wheel) const;
    inline fp32 Get_Seat(const uint8_t &__wheel) const;
    bool Is_Steer_Ready(uint8_t __wheel) const;

    /* ---- 舵向定义(电机下标/减速比/方向/零位) ---- */
    void Set_Steer_Def(const uint8_t &__wheel, const chassis_D_steer_def &__def)
    {
        if (__wheel < CHASSIS_D2Q2_UNIT_N)
        {
            steer[__wheel] = __def;
            Invalidate_Steer_Feedback(__wheel);
        }
    }

    inline const chassis_D_steer_def &Get_Steer_Def(const uint8_t &__wheel) const
    {
        return steer[(__wheel < CHASSIS_D2Q2_UNIT_N) ? __wheel : 0u];
    }

    /** 电机侧角 → 轮侧角 [rad],归一到 [0, 2π) */
    inline fp32 Motor_To_Seat(const uint8_t &__wheel, const fp32 __motor_rad) const
    {
        const chassis_D_steer_def &d = Get_Steer_Def(__wheel);
        const fp32 dir = (d.dir < 0.0f) ? (-1.0f) : (1.0f);
        return Math_Angle_Wrap_2Pi(dir * __motor_rad + d.seat_offset);
    }

    /** 轮侧目标 → 电机侧最短弧增量(抗 2π→0) */
    inline fp32 Seat_To_Motor_Delta(const uint8_t &__wheel, const fp32 __seat_cmd) const
    {
        const chassis_D_steer_def &d = Get_Steer_Def(__wheel);
        const fp32 dir = (d.dir < 0.0f) ? (-1.0f) : (1.0f);
        const uint8_t i = (__wheel < CHASSIS_D2Q2_UNIT_N) ? __wheel : 0u;
        return dir * Math_Angle_Diff_Pi(__seat_cmd, chassis[i].seat_now);
    }

    inline fp32 Get_Vx() const;
    inline fp32 Get_Vy() const;
    inline fp32 Get_Vw() const;

protected:
    fp32 x = 0.0f;              /* 车体 vx [m/s](正解) */
    fp32 y = 0.0f;              /* 车体 vy [m/s] */
    fp32 w = 0.0f;              /* 车体 ω [rad/s] */
    fp32 r = 0.06f;             /* 轮半径 [m] */
    fp32 a = 0.3f;              /* 半轴距 [m](仅记录) */
    fp32 b = 0.3f;              /* 半轮距 [m](仅记录) */
    fp32 top_yaw = 0.0f;        /* 小陀螺 0 位 */
    chassis_D_steer_def steer[CHASSIS_D2Q2_UNIT_N];
    chassis_D_wheel chassis[CHASSIS_D2Q2_UNIT_N];

private:
    struct SteeringState
    {
        bool has_feedback = false;
        fp32 last_nonzero_rpm = 0.0f;  /* 上次选弧的带符号转速，停车时保留 */
    };

    SteeringState steering[CHASSIS_D2Q2_UNIT_N];
};

inline void Class_chassis_D2Q2::Set_Seat_Now(const uint8_t &__wheel, const fp32 __seat_now) {
    const uint8_t i = (__wheel < CHASSIS_D2Q2_UNIT_N) ? __wheel : 0u;
    auto &wheel = chassis[i];
    wheel.seat_now = Math_Angle_Wrap_2Pi(__seat_now);
    if (!steering[i].has_feedback)
    {
        wheel.seat = wheel.seat_now;
        steering[i].has_feedback = true;
    }
}

inline fp32 Class_chassis_D2Q2::Get_Top_Yaw() const {
    return top_yaw;
}

inline fp32 Class_chassis_D2Q2::Get_Speed(const uint8_t &__wheel) const {
    return chassis[(__wheel < CHASSIS_D2Q2_UNIT_N) ? __wheel : 0u].speed;
}

inline fp32 Class_chassis_D2Q2::Get_Seat(const uint8_t &__wheel) const {
    return chassis[(__wheel < CHASSIS_D2Q2_UNIT_N) ? __wheel : 0u].seat;
}

inline fp32 Class_chassis_D2Q2::Get_Vx() const {
    return x;
}

inline fp32 Class_chassis_D2Q2::Get_Vy() const {
    return y;
}

inline fp32 Class_chassis_D2Q2::Get_Vw() const {
    return w;
}

#endif