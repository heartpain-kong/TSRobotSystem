//
// Created by heartpain on 2026/10/11.
//

#include "alg_chassis_control.h"
//
// Created by 20159 on 2026/4/1.
//
#include "math_support.h"
/* 目标与实测角之间的最短弧长，允许角度跨越 0/2π。 */
static fp32 Angle_Arc_Dist(fp32 ref, fp32 target)
{
    return fabsf(Math_Angle_Diff_Pi(target, ref));
}

static bool Steer_Is_Ready(fp32 measured, fp32 target)
{
    constexpr fp32 angle_roundoff = 1.0e-6f;  /* 避免 30° 边界因浮点舍入提前放行 */
    return Angle_Arc_Dist(measured, target) < CHASSIS_D_READY_THRESH - angle_roundoff;
}

//45 45 135 135

static fp32 chassis_M_math_sin_cos[2][2] = {0.707106,0.707106,0.707106,-0.707106};
static fp32 chassis_M_math_angle[4] = {PI/4,3*PI/4,5*PI/4,7*PI/4};	//45 135 225 315

/* 原接口保留(Init 在用它);实现交给算法层的 Math_Angle_Wrap_2Pi —— 全工程统一一份 */
fp32 angle_pi(fp32 __angle){
    return Math_Angle_Wrap_2Pi(__angle);
}

/* 小陀螺模式启动时调用:把"此刻的 yaw"记成 0 位(按小陀螺键的那一刻调一次)。
   ★ 这里做预处理:先归一到 [0, 2π) ——
     ① 存下来的 top_yaw 始终是规范值,Get_Top_Yaw() 读出来不会是 -π 那种别扭值;
     ② 与 count_run2 里"最短夹角"的归一化口径一致(两边都在 [0,2π) 上比)。
   (count_run2 内部仍会再取一次最短夹角 ⇒ 就算上层传了没归一的 yaw 也不会出问题)
 */
void Class_chassis_M4::Set_Top_Yaw(const fp32 __top_yaw){
    top_yaw = Math_Angle_Wrap_2Pi(__top_yaw);
}

void Class_chassis_M4::Init(const fp32 __a,const fp32 __b,const fp32 __r,chassis_M_wheel_init __init[4]){
    a = __a;
    b = __b;
    r = __r;
    top_yaw = 0.0f;      /* 上电先按当前朝向作为 0 位;开小陀螺时再 Set_Top_Yaw 对齐 */
    x = 0.0f;
    y = 0.0f;
    w = 0.0f;
    arm_sqrt_f32(a*a+b*b,&D);
/*					+Y
 *	  (-1, 1)	|		| ( 1, 1)
 *  -X							+X
 *	  (-1,-1)	|		| ( 1,-1)
 *					-Y
 */
	for(uint8_t i =0 ; i<4; ++i) {
		//限制到1或者-1
		__init[i].wheel_xy[1]=__init[i].wheel_xy[1]/abs(__init[i].wheel_xy[1]);
		__init[i].wheel_xy[0]=__init[i].wheel_xy[0]/abs(__init[i].wheel_xy[0]);
		chassis[i].XY_angle = angle_pi(atan2f(__init[i].wheel_xy[1]*__b,__init[i].wheel_xy[0]*__a));
		chassis[i].xy[0] = __init[i].wheel_xy[0];      /* ±1:用它定速度符号 */
		chassis[i].xy[1] = __init[i].wheel_xy[1];
		chassis[i].wheel = __init[i].angle;
	}

}

void Class_chassis_M4::count_run1(const fp32 __X_Y_W[3]){

    fp32 V_Y,V_X,V_W,math_M_angle,M_V_mold;
    for(uint8_t i =0 ; i<4; ++i){
			V_W = __X_Y_W[2];
            V_X = __X_Y_W[0]-D*arm_sin_f32(chassis[i].XY_angle)*V_W;
			V_Y = __X_Y_W[1]+D*arm_cos_f32(chassis[i].XY_angle)*V_W;

			//计算V的模值
			arm_sqrt_f32(V_Y *V_Y  + V_X*V_X , &M_V_mold);

			if(M_V_mold==0.0f){
				chassis[i].speed=0.0f;
				continue;
			}
			//计算角度用的是arccos 取值范围为[-1,1]
			math_M_angle=acos((V_X*chassis_M_math_sin_cos[chassis[i].wheel][0]
                +V_Y*chassis_M_math_sin_cos[chassis[i].wheel][1])
                /M_V_mold);

			//通过映射公式为speed = |V|x cos<V,speed> 计算出速度
			chassis[i].speed = M_V_mold*arm_cos_f32(math_M_angle)/r*30.0f/PI;
		}
		/* ★ 速度符号由轮子方位自动决定:**X 正半轴的轮子为正,负半轴为负**
		   (以前这里写死 chassis[2]/[3] 取负;现在 Init 传进来的 wheel_xy 说了算) */
		for(uint8_t i = 0; i < 4; ++i){
			chassis[i].speed *= (fp32)chassis[i].xy[0];
		}
}

void Class_chassis_M4::count_run2(const fp32 __X_Y_W[3],const fp32 __yaw){

    /* 小陀螺:把指令从"世界系"旋到"机体系",再复用 count_run1(两份代码合一)。
       __yaw 传**当前绝对 yaw**;内部减 top_yaw ⇒ 小陀螺启动那一刻被对齐成 0 位 ✓ */
    /* ★ 把 rel 归到 (-π, π] 再送 arm_cos_f32/arm_sin_f32:
       ① 陀螺仪 yaw 过 ±π 会跳变(π → -π),直接相减 rel 会瞬间差 2π ⇒ 底盘猛甩;
       ② cos/sin 只关心相对角,取最短夹角与直接相减等价;
       ③ 这样调用方**不需要**再做 yaw<0 ? yaw+2π 那套归一化
          (那种归一只归一了一边,top_yaw 仍是原始值,反而可能差一整圈) */
    const fp32 rel = Math_Angle_Diff_Pi(__yaw, top_yaw);
    fp32 cmd[3];

    cmd[0] = __X_Y_W[0]*arm_cos_f32(rel) - __X_Y_W[1]*arm_sin_f32(rel);
    cmd[1] = __X_Y_W[0]*arm_sin_f32(rel) + __X_Y_W[1]*arm_cos_f32(rel);
    cmd[2] = __X_Y_W[2];

    count_run1(cmd);
}
void Class_chassis_M4::forward_count(const fp32 __wheel_speed[4])
{
    /* ★ 输入是**关节侧 rpm**(电机类内部已经除过 Gearbox_Rate,例如 3508 舵向的 4.0)。
          轮心线速度:v[m/s] = rpm × π/30 × r
       ⇒ 这里**不能再出现减速比**,否则等于除了两次。
       (原先写死的 19 是“电机侧 rad/s”版本的遗留,和现在的 wheel rpm 对不上。) */
    const fp32 k = r * (PI / 30.0f);      /* wheel rpm → m/s */

    fp32 sum_x = __wheel_speed[0] + __wheel_speed[1] - __wheel_speed[2] - __wheel_speed[3];
    fp32 sum_y = -__wheel_speed[0] + __wheel_speed[1] - __wheel_speed[2] + __wheel_speed[3];
    fp32 sum_w = -__wheel_speed[0] + __wheel_speed[1] + __wheel_speed[2] - __wheel_speed[3];

    x = sum_x * k / 4.0f;
    y = sum_y * k / 4.0f;
    w = sum_w * k / (4.0f * 2.0f);  /* 2.0 沿用原经验系数;实车建议再核对一次 */
}

/* ============================================================================
 * 舵轮(独立转向)底盘 —— 反解 / 正解
 *   反解:车体 (vx, vy, ω) → 每轮 (转向角 seat, 驱动转速 speed)
 *   正解:每轮实测 (转向角, 转速) → 车体 (vx, vy, ω)   ← 里程计
 * 参考:E:\rm\chassis_task.c(chassisMath_D_chassis / chassis_math_D_angle_mapping)
 * ========================================================================== */


/* 小陀螺模式启动时调用:把"此刻的 yaw"记成 0 位(按小陀螺键的那一刻调一次)。
   ★ 这里做预处理:先归一到 [0, 2π) ——
     ① 存下来的 top_yaw 始终是规范值,Get_Top_Yaw() 读出来不会是 -π 那种别扭值;
     ② 与 count_run2 里"最短夹角"的归一化口径一致(两边都在 [0,2π) 上比)。
   (count_run2 内部仍会再取一次最短夹角 ⇒ 就算上层传了没归一的 yaw 也不会出问题)
 */
void Class_chassis_D4::Set_Top_Yaw(const fp32 __top_yaw){
    top_yaw = Math_Angle_Wrap_2Pi(__top_yaw);
}

void Class_chassis_D4::Init(const fp32 __a, const fp32 __b, const fp32 __r, const chassis_D_wheel_init __init[4]){
    a = __a;
    b = __b;
    r = __r;
    top_yaw = 0.0f;      /* 上电先按当前朝向作为 0 位;开小陀螺时再 Set_Top_Yaw 对齐 */
    x = 0.0f;
    y = 0.0f;
    w = 0.0f;
    for(uint8_t i = 0; i < 4; ++i){
        chassis[i].x = __init[i].x;
        chassis[i].y = __init[i].y;
        chassis[i].seat_offset = __init[i].seat_offset;
        chassis[i].V_X = 0.0f;
        chassis[i].V_Y = 0.0f;
        chassis[i].speed = 0.0f;
        chassis[i].seat = 0.0f;
        chassis[i].seat_last = 0.0f;
        chassis[i].seat_now = 0.0f;
    }
}

/*
 * 反解(机体系):
 *   ① 轮心速度 = 车体平动 + 绕中心转动:v_i = v + ω × r_i
 *         V_X = vx - ω·y_i      V_Y = vy + ω·x_i        (ω 逆时针为正)
 *   ② 指令为 0 ⇒ 保持上次转向角(不然轮子会乱甩,抄参考)
 *   ③ 目标角与**当前实际角**差超过 90° ⇒ 翻 180° 并把轮速取反(转向电机少转半圈)
 *   ④ 转向还没到位(残差 ≥ 45°)⇒ 先不给驱动速度(防轮胎"搓地",抄参考)
 */
void Class_chassis_D4::count_run1(const fp32 __X_Y_W[3]){

    for(uint8_t i = 0; i < 4; ++i){
        fp32 V_X, V_Y, mold, target, speed_signed;

        /* ① 轮心速度 */
        V_X = __X_Y_W[0] - __X_Y_W[2]*chassis[i].y;
        V_Y = __X_Y_W[1] + __X_Y_W[2]*chassis[i].x;
        chassis[i].V_X = V_X;
        chassis[i].V_Y = V_Y;

        arm_sqrt_f32(V_X*V_X + V_Y*V_Y, &mold);

        /* ② 完全静止:保持上次的转向角,速度给 0 */
        if(mold < 0.0001f){
            chassis[i].seat = chassis[i].seat_last;
            chassis[i].speed = 0.0f;
            continue;
        }

        target = atan2f(V_Y, V_X);          /* (-π, π] */
        speed_signed = mold;                /* 先按"不翻"算,模长保持正 */

        /* ③ 就近翻转 */
        if(Angle_Arc_Dist(chassis[i].seat_now, target) > CHASSIS_D_FLIP_THRESH){
            target -= PI;
            speed_signed = -mold;           /* 轮子反着转 */
        }
        target = Math_Angle_Wrap_2Pi(target);

        /* ④ 转向未到位先不驱动;否则线速度 → 轮转速(rpm) */
        if(!Steer_Is_Ready(chassis[i].seat_now, target)){
            chassis[i].speed = 0.0f;
        }else{
            chassis[i].speed = speed_signed / r * 30.0f/PI;
        }

        chassis[i].seat = target;
        chassis[i].seat_last = target;
    }
}

/*
 * 反解(世界系 / 小陀螺):
 *   先把指令按 yaw 旋到机体系,再走上面那套 —— 与 Class_chassis_M4::count_run2 同一个思路
 */
void Class_chassis_D4::count_run2(const fp32 __X_Y_W[3], const fp32 __yaw){

    /* ★ 把 rel 归到 (-π, π] 再送 arm_cos_f32/arm_sin_f32:
       ① 陀螺仪 yaw 过 ±π 会跳变(π → -π),直接相减 rel 会瞬间差 2π ⇒ 底盘猛甩;
       ② cos/sin 只关心相对角,取最短夹角与直接相减等价;
       ③ 这样调用方**不需要**再做 yaw<0 ? yaw+2π 那套归一化
          (那种归一只归一了一边,top_yaw 仍是原始值,反而可能差一整圈) */
    const fp32 rel = Math_Angle_Diff_Pi(__yaw, top_yaw);
    fp32 cmd[3];

    cmd[0] = __X_Y_W[0]*arm_cos_f32(rel) - __X_Y_W[1]*arm_sin_f32(rel);
    cmd[1] = __X_Y_W[0]*arm_sin_f32(rel) + __X_Y_W[1]*arm_cos_f32(rel);
    cmd[2] = __X_Y_W[2];

    count_run1(cmd);
}

/*
 * 正解(里程计):
 *   每个轮心的线速度 v_i = rpm·π/30·r,方向就是它的实测转向角 ⇒
 *         vx_i = v_i·cos(θ_i)     vy_i = v_i·sin(θ_i)
 *   车体速度取 4 轮平均;角速度用最小二乘解(4 轮 |r_i| 相同 ⇒ 化简成下面这样):
 *         ω = Σ( -vx_i·y_i + vy_i·x_i ) / (4·(a²+b²))
 */
void Class_chassis_D4::forward_count(const fp32 __seat_now[4], const fp32 __speed_now[4]){

    fp32 sum_x = 0.0f, sum_y = 0.0f, sum_w = 0.0f, d2;

    for(uint8_t i = 0; i < 4; ++i){
        fp32 v  = __speed_now[i]*PI/30.0f*r;                 /* rpm → 轮心线速度 m/s */
        fp32 vx = v*arm_cos_f32(__seat_now[i]);
        fp32 vy = v*arm_sin_f32(__seat_now[i]);

        sum_x += vx;
        sum_y += vy;
        sum_w += (-vx*chassis[i].y + vy*chassis[i].x);
    }

    d2 = a*a + b*b;

    x = sum_x/4.0f;
    y = sum_y/4.0f;
    w = (d2 > 0.000001f) ? (sum_w/(4.0f*d2)) : 0.0f;
}

/* ============================================================================
 * 2 舵轮 + 2 全向轮(D2Q2;Q 轮无动力,只在物理上支撑)
 *   构造: D Q
 *         Q D
 * ========================================================================== */
void Class_chassis_D2Q2::Init(const fp32 __a, const fp32 __b, const fp32 __r,
                              const chassis_D_wheel_init __init[CHASSIS_D2Q2_UNIT_N])
{
    a = __a;
    b = __b;
    r = __r;
    top_yaw = 0.0f;
    x = 0.0f;
    y = 0.0f;
    w = 0.0f;

    for (uint8_t i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
    {
        chassis[i] = chassis_D_wheel{};
        steering[i] = SteeringState{};
        chassis[i].x = __init[i].x;
        chassis[i].y = __init[i].y;
        chassis[i].seat_offset = __init[i].seat_offset;

        /* 默认:未分配电机、6020 直驱预设、正装、零位 0;App 用 Set_Steer_Def 覆盖 */
        steer[i] = chassis_D_steer_def{0xFFu, CHASSIS_STEER_RATIO_6020_DIRECT, 1.0f, 0.0f};
    }
}

void Class_chassis_D2Q2::Set_Top_Yaw(const fp32 __top_yaw)
{
    top_yaw = Math_Angle_Wrap_2Pi(__top_yaw);
}

/*
 * 以最后一次有效实测舵角为起点，在两个等价的舵角与轮速组合中选短弧。
 * 两条弧恰好等长时，使用上一有效舵角和驱动方向决定走哪一边。
 */
void Class_chassis_D2Q2::count_run1(const fp32 __X_Y_W[3])
{
    constexpr fp32 min_wheel_speed = 0.02f;  /* 轮心速度死区 [m/s] */

    for (uint8_t i = 0u; i < CHASSIS_D2Q2_UNIT_N; i++)
    {
        auto &wheel = chassis[i];
        auto &state = steering[i];
        wheel.V_X = __X_Y_W[0] - __X_Y_W[2] * wheel.y;
        wheel.V_Y = __X_Y_W[1] + __X_Y_W[2] * wheel.x;

        wheel.speed = 0.0f;
        if (!state.has_feedback)
        {
            continue;
        }

        fp32 speed;
        arm_sqrt_f32(wheel.V_X * wheel.V_X + wheel.V_Y * wheel.V_Y, &speed);
        if (speed < min_wheel_speed)
        {
            continue;  /* 保持上一舵角和带符号转速记录，驱动输出为 0 */
        }

        const fp32 forward_seat = Math_Angle_Wrap_2Pi(atan2f(wheel.V_Y, wheel.V_X));
        fp32 target = forward_seat;
#if CHASSIS_D_ARC_OPT
        const fp32 reverse_seat = Math_Angle_Wrap_2Pi(forward_seat + PI);
        const fp32 forward_arc = Angle_Arc_Dist(wheel.seat_now, forward_seat);
        const fp32 reverse_arc = Angle_Arc_Dist(wheel.seat_now, reverse_seat);
        bool reverse = reverse_arc < forward_arc;

        constexpr fp32 arc_roundoff = 1.0e-6f;
        if (state.last_nonzero_rpm != 0.0f && fabsf(forward_arc - reverse_arc) <= arc_roundoff)
        {
            const fp32 previous_forward_arc = Angle_Arc_Dist(wheel.seat, forward_seat);
            const fp32 previous_reverse_arc = Angle_Arc_Dist(wheel.seat, reverse_seat);
            const fp32 previous_arc_diff = previous_forward_arc - previous_reverse_arc;
            reverse = (fabsf(previous_arc_diff) <= arc_roundoff)
                          ? (state.last_nonzero_rpm < 0.0f)
                          : (previous_arc_diff > 0.0f);
        }

        if (reverse)
        {
            target = reverse_seat;
            speed = -speed;
        }
#endif
        wheel.seat = target;
        state.last_nonzero_rpm = speed / r * 30.0f / PI;
        if (Is_Steer_Ready(i))
        {
            wheel.speed = state.last_nonzero_rpm;
        }
    }
}

bool Class_chassis_D2Q2::Is_Steer_Ready(uint8_t __wheel) const
{
    return __wheel < CHASSIS_D2Q2_UNIT_N && steering[__wheel].has_feedback &&
           Steer_Is_Ready(chassis[__wheel].seat_now, chassis[__wheel].seat);
}

void Class_chassis_D2Q2::count_run2(const fp32 __X_Y_W[3], const fp32 __yaw)
{
    const fp32 rel = Math_Angle_Diff_Pi(__yaw, top_yaw);
    fp32 cmd[3];

    cmd[0] = __X_Y_W[0] * arm_cos_f32(rel) - __X_Y_W[1] * arm_sin_f32(rel);
    cmd[1] = __X_Y_W[0] * arm_sin_f32(rel) + __X_Y_W[1] * arm_cos_f32(rel);
    cmd[2] = __X_Y_W[2];

    count_run1(cmd);
}

/*
 * 正解(里程计):
 *   每个 D 轮的实际速度矢量 v_i = (speed_i·π/30·r)·(cos seat_i, sin seat_i)
 *   刚体速度场:v_i = (vx - ω·y_i, vy + ω·x_i)
 *   ⇒ 由两个点速度解 (vx, vy, ω):
 *        ω = (v0x - v1x)/(y1 - y0) = (v0y - v1y)/(x0 - x1)
 *        vx = ½[(v0x + ω·y0) + (v1x + ω·y1)]
 *        vy = ½[(v0y - ω·x0) + (v1y - ω·x1)]
 *   两个 ω 估计都有效时取平均;只有一条有效时用那条。
 */
void Class_chassis_D2Q2::forward_count(const fp32 __seat_now[CHASSIS_D2Q2_UNIT_N],
                                       const fp32 __speed_now[CHASSIS_D2Q2_UNIT_N])
{
    const fp32 v0 = __speed_now[0] * (PI / 30.0f) * r;
    const fp32 v1 = __speed_now[1] * (PI / 30.0f) * r;

    const fp32 v0x = v0 * arm_cos_f32(__seat_now[0]);
    const fp32 v0y = v0 * arm_sin_f32(__seat_now[0]);
    const fp32 v1x = v1 * arm_cos_f32(__seat_now[1]);
    const fp32 v1y = v1 * arm_sin_f32(__seat_now[1]);

    const fp32 dx = chassis[0].x - chassis[1].x;
    const fp32 dy = chassis[0].y - chassis[1].y;

    const bool has_wx = (fabsf(dy) > 0.00001f);   /* y0 != y1 */
    const bool has_wy = (fabsf(dx) > 0.00001f);   /* x0 != x1 */

    fp32 wx = 0.0f;
    fp32 wy = 0.0f;

    if (has_wx) { wx = (v0x - v1x) / (chassis[1].y - chassis[0].y); }
    if (has_wy) { wy = (v0y - v1y) / (chassis[0].x - chassis[1].x); }

    if (has_wx && has_wy)      { w = 0.5f * (wx + wy); }
    else if (has_wx)           { w = wx; }
    else if (has_wy)           { w = wy; }
    else                       { w = 0.0f; }

    x = 0.5f * ((v0x + w * chassis[0].y) + (v1x + w * chassis[1].y));
    y = 0.5f * ((v0y - w * chassis[0].x) + (v1y - w * chassis[1].x));
}