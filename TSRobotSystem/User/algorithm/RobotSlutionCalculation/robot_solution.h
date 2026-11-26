//
// robot_solution.h — 双 MCU 共用的底盘运动学/动力学
//
// 纯 C/C++ 数学层，不依赖 HAL、FreeRTOS、CAN。
// 上 MCU / 下 MCU 都可以直接编译。
//

#ifndef ROBOT_SOLUTION_H
#define ROBOT_SOLUTION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float x;            /* 轮心 x [m] */
    float y;            /* 轮心 y [m] */
    float zero_rad;     /* 机械零位偏置 [rad] */
    float dir;          /* +1 / -1 */
} RobotWheelGeometry;

typedef struct
{
    float vx;           /* 车体 x 速度 [m/s] */
    float vy;           /* 车体 y 速度 [m/s] */
    float w;            /* 车体 yaw 角速度 [rad/s] */
} RobotVelocity;

typedef struct
{
    float steer_rad;    /* 舵向目标 [rad] */
    float wheel_rpm;     /* 驱动轮目标 [rpm] */
    float vx;           /* 该轮 x 速度 [m/s] */
    float vy;           /* 该轮 y 速度 [m/s] */
    uint8_t flipped;    /* 1 = 走了 180° 翻转 */
} RobotWheelCommand;

typedef struct
{
    float steer_rad;
    float wheel_rpm;
} RobotWheelFeedback;

/**
 * D2Q2 逆运动学: 车体 (vx,vy,w) -> 每个舵轮 (steer, rpm)
 *
 * 刚体速度场:
 *   vx_i = vx - w*y_i
 *   vy_i = vy + w*x_i
 *
 * 轮速方向 = atan2(vy_i, vx_i)
 * 轮速大小 = hypot(vx_i, vy_i)
 *
 * @param wheel_radius 轮半径 [m]
 * @param max_wheel_rpm 轮速限幅 [rpm], <=0 表示不限
 * @return 0 成功
 */
int RobotSolution_Inverse2(const RobotVelocity *__vel,
                           const RobotWheelGeometry __wheel[2],
                           float __wheel_radius,
                           float __max_wheel_rpm,
                           RobotWheelCommand __out[2]);

/**
 * D2Q2 正运动学: 每个轮 (steer, rpm) -> 车体 (vx,vy,w)
 * 用于下 MCU 回传实际速度。
 */
int RobotSolution_Forward2(const RobotWheelFeedback __wheel[2],
                           const RobotWheelGeometry __geom[2],
                           float __wheel_radius,
                           RobotVelocity *__out);

/**
 * 动力学/功率粗估:
 *   P = Σ |v_i| * |F_i| / η + P_idle
 * 目前没有轮上力传感器,用轮速和力矩常数估算,只用于 120W 限幅。
 */
float RobotSolution_PowerEstimate(const RobotVelocity *__vel,
                                  const RobotWheelCommand __wheel[2],
                                  float __torque_per_wheel_nm,
                                  float __efficiency,
                                  float __idle_w);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_SOLUTION_H
