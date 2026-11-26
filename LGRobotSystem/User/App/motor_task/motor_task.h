/* ============================================================================
 * motor_task.h — 电机子系统入口(给 C 文件/机器人层用的纯声明头)
 *
 * 【分工】(与 basic_framework 的分层一致)
 *   · 本模块:**初始化电机 + 注册接收回调** + 提供"跑一拍"的 motor_task()
 *   · 机器人层(User/App/robot/):决定**谁在什么频率下调用 motor_task()**
 *   · main.c:只调 Robot_Init()
 *
 * 【为什么单独有这个头】
 *   电机的类(Class_Motor_DM/DJ/YS/LZ)都是 **C++**,而 main.c / freertos.c 是 **C** ——
 *   C 编译器解析不了 `class`/`motor_mit`。所以电机对象、任务体、接收回调都放在
 *   motor_task.cpp(C++),对 C 侧只暴露下面这些 `extern "C"` 函数。
 *   ⚠️ C 文件只能包这个头,**绝不能** include motor_XX/xxx.h。
 * ========================================================================== */

#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电机初始化:每个电机 Init + 注册 CAN/485 接收回调
 * @note 只初始化,**不建任务**(任务由机器人层 User/App/robot/robot_task.cpp 的
 *       OSTaskInit() 统一创建);调用前要先 Timestamp_Init(&htim5)(电机频率排程用 TIM5 的微秒时间戳)。
 */
void motor_task_init(void);

/**
 * @brief 电机控制,**跑一拍**(不含循环)
 *
 * 由机器人层的 MOTOR 任务按 robot_def.h 的 MOTOR_TASK_PERIOD_MS 周期调用。
 * 内部:写目标 → 需要主机闭环的电机算 PID → 各电机按自己的频率(Is_Send_Due)发帧。
 */
void motor_task(void);

/* ============================================================================
 * 逐台状态查询 —— 给 error_task 量化报错用(到底是哪个电机没初始化成功/离线/报错)
 *   下标 i 从 0 起,对应电机 id = i+1
 * ========================================================================== */

/**
 * @brief 电机总数
 * @return 台数
 */
uint8_t motor_task_get_count(void);

/** @brief 0x201..0x20B 中某个 CAN ID 最近是否在线(临时扫描用) */
uint8_t motor_task_can_online(uint16_t __can_id);

/**
 * @brief 第 i 台"初始化成功"了吗
 * @param __index 下标(0 起)
 * @return true = 成功
 * @note 判据:收到过至少一帧反馈(dj_rx_us != 0)。
 *       大疆的 Motor_Status 只是命令镜像(enable/lose 改它),不能用它判断反馈。
 */
bool motor_task_is_init_ok(uint8_t __index);

/**
 * @brief 第 i 台现在在线吗(最近 MOTOR_ONLINE_TIMEOUT_US 内收到过反馈)
 * @param __index 下标(0 起)
 * @return true = 在线
 */
bool motor_task_is_online(uint8_t __index);

/**
 * @brief 第 i 台的电机错误码(反馈帧里的 MError;0 = 无错)
 * @param __index 下标(0 起)
 * @return 错误码
 */
uint8_t motor_task_get_m_error(uint8_t __index);

/**
 * @brief 第 i 台的名字(调试/日志用,如 "CAN1-DJ-3508")
 * @param __index 下标(0 起)
 * @return 名字;越界返回 "N/A"
 */
const char *motor_task_get_name(uint8_t __index);

/* ============================================================================
 * 给上层写目标 / 读反馈(★ 单位:电机接口一律**关节侧** rad / rad·s⁻¹ / N·m;
 *   底盘算法用的是 rpm ⇒ 换算只在 motor_task_set/get_wheel_rpm 里做一次 ✓)
 * ========================================================================== */

/**
 * @brief 写一路电机的 MIT 目标(单位:关节侧 rad, rad/s, N·m)
 * @note 单台大疆没有 MIT 帧:__t/__kp/__kd 会被忽略;速度模式用 __w,位置模式用 __pos。
 *       当前模式由 set_wheel_rpm/set_steer_rad 决定。
 */
bool motor_task_set_mit(uint8_t __index, float __pos, float __w, float __t, float __kp, float __kd);

/**
 * @brief 以"轮速"驱动一路电机:rpm → 关节侧 rad/s,并置 speed 模式(大疆由主机算速度环)
 * @param __index 电机下标(0 起)
 * @param __rpm   目标轮速(rpm,正负 = 转向)
 * @param __kd    保留参数(大疆不用;兼容底盘接口)
 * @return true = 成功
 */
bool motor_task_set_wheel_rpm(uint8_t __index, float __rpm, float __kd);

/**
 * @brief 读一路电机的实测转速(**换算成 rpm** 返回,和底盘算法单位一致)
 */
float motor_task_get_wheel_rpm(uint8_t __index);

/**
 * @brief 读一路电机的实测力矩(N·m,关节侧)
 */
float motor_task_get_now_torque(uint8_t __index);

/** @brief 读 C620 反馈电流(A,电机侧;0x206 与 0x207/0x208 台架) */
float motor_task_get_now_current(uint8_t __index);

/**
 * @brief 读一路电机的实测位置(rad,关节侧)
 */
float motor_task_get_now_pos(uint8_t __index);

/**
 * @brief 读电机实测转速(关节侧,rad/s)
 * @param __index 注册表下标(0 起),>= motor_task_get_count() 时返回 0
 */
float motor_task_get_now_w(uint8_t __index);

/** @brief 读速度环 PID 输出力矩(关节侧,N·m) */
float motor_task_get_pid_out(uint8_t __index);

/**
 * @brief 舵向角度目标(切到位置模式)
 * @param __rad 关节侧角度 [rad](已除总减速比)
 */
bool motor_task_set_steer_rad(float __rad);

/** @brief 按电机注册表下标写舵向角(当前单电机台架仅 index=0 有效) */
bool motor_task_set_steer_rad_index(uint8_t __index, float __rad);

typedef struct
{
    float position_rad;
    uint32_t age_us;  /* UINT32_MAX 表示等待有效的编码器帧 */
} Motor_Steer_Feedback_t;

/** @brief 读取同一帧的舵向位置和帧年龄。注册表下标 0/2，仅有效且小于 10 ms 时返回 true。 */
bool motor_task_get_steer_feedback(uint8_t __index, Motor_Steer_Feedback_t *__feedback);

/** @brief 读关节侧实测角度 [rad] */
float motor_task_get_steer_rad(void);

/**
 * @brief 设目标转速 [rpm]
 * @note 只改目标,不切模式;要确保处于速度模式请用 motor_task_set_wheel_rpm()
 */
void  motor_task_set_target_rpm(float __rpm);

/** @brief 读当前目标转速 [rpm] */
float motor_task_get_target_rpm(void);

/** @brief 读速度环实际目标 [rpm](位置模式下是角度环输出,不是 g_dj_target_rpm) */
float motor_task_get_pid_target_rpm(void);

/** @brief 读关节侧位置目标 [rad](位置模式) */
float motor_task_get_target_rad(void);

/** @brief 读 204 舵向位置目标 [rad] */
float motor_task_get_target_rad2(void);

/** @brief 读 204 舵向速度环实际目标 [rpm] */
float motor_task_get_pid_target_rpm2(void);

/* ============================================================================
 * 串口调试接口(上位机“电机调试”窗口,0x5C 帧)
 *   当前只调速度环(MOTOR_POS_LOOP_ENABLE=0);
 *   调试命令一旦生效会锁住底盘对 0 号电机的写入,直到复位。
 * ========================================================================== */

/** @brief 调试:速度目标 [rpm](切 speed 模式 + 使能) */
void motor_task_debug_set_speed_rpm(int16_t __rpm);
/** @brief 调试:0x201 驱动速度 [rpm] */
void motor_task_debug_set_drive_rpm(int16_t __rpm);
/** @brief 调试:写 0x201 驱动速度环 PID */
void motor_task_debug_set_drive_pid(float __kp, float __ki, float __kd);
/** @brief 调试:0x204 舵向角度 [0.001°] */
void motor_task_debug_set_steer2_mdeg(int32_t __mdeg);
/** @brief 调试:0x204 舵向速度 [rpm] */
void motor_task_debug_set_steer2_rpm(int16_t __rpm);
/** @brief 调试:0x203 驱动速度 [rpm] */
void motor_task_debug_set_drive2_rpm(int16_t __rpm);
/** @brief 调试:写 0x204 舵向 PID(6 个 float) */
void motor_task_debug_set_steer2_pid(float __skp, float __ski, float __skd,
                                     float __pkp, float __pki, float __pkd);

/** @brief 调试:位置目标 [0.001°](当前位置环关闭,忽略) */
void motor_task_debug_set_pos_mdeg(int32_t __mdeg);

/** @brief 调试:写 PID(速度环 Kp/Ki/Kd + 位置环 Kp/Ki/Kd;位置环当前不参与控制) */
void motor_task_debug_set_pid(float __skp, float __ski, float __skd,
                              float __pkp, float __pki, float __pkd);

/** @brief 调试:读当前 PID(6 个 float:速度 Kp/Ki/Kd,位置 Kp/Ki/Kd) */
void motor_task_debug_get_pid(float *__pid6);

/** @brief 调试:停止输出(目标 0 + lose,继续发 0 电流) */
void motor_task_debug_stop(void);

/* ---- 台架:摩擦轮 0x207/0x208 状态(index 0=207,1=208) ---- */
uint8_t motor_task_friction_flags(uint8_t __index);
float motor_task_friction_pos(uint8_t __index);
float motor_task_friction_w(uint8_t __index);
float motor_task_friction_t(uint8_t __index);
void motor_task_trigger_set_rpm(float __rpm);
void motor_task_friction_set_rpm(uint8_t __index, float __rpm);
/** @brief 发射:左轮 -rpm、右轮 +rpm(左轮反装) */
void motor_task_friction_shoot(float __rpm);
void motor_task_friction_set_pid(uint8_t __index, float __kp, float __ki, float __kd);
void motor_task_friction_get_pid(uint8_t __index, float *__pid3);

/* 0x206 拨盘(index0) / 0x205 6020(index1) 速度环 */
void motor_task_aux_set_rpm(uint8_t __index, int16_t __rpm);
void motor_task_friction_set_rpm(uint8_t __index, float __rpm);
void motor_task_aux_set_pos(uint8_t __index, float __rad);
void motor_task_aux_set_pos_hold(uint8_t __index, float __rad);   /* 保持用:不夺取底盘控制权 */
void motor_task_aux_set_speed_hold(uint8_t __index, float __rpm);
/** @brief 6020 位置环速度前馈 [rad/s](小陀螺反向补偿) */
void motor_task_aux_set_pos_vel_ff(uint8_t __index, float __ff);
/** @brief 舵向找零完成:把当前位置记为 __rad(加偏置),0=机械零点 unit: 0=202,1=204 */
void motor_task_steer_zero(uint8_t __unit, float __rad);  /* 速度保持:不夺取控制权 */
/** @brief 舵轮调试开关:打开后底盘任务不再下发舵向目标,可由 5C 0B/0C 直接控制 */
void motor_task_set_steer_debug(uint8_t __on);
/** @brief 驱动轮目标转速(201/203),rpm;__index: 0=201, 2=203 */
float motor_task_get_drive_target_rpm(uint8_t __index);
/** @brief 6020 调试开关:打开后底盘不再改写 6020 目标,可用 5C 12/10 直接控 */
void motor_task_set_6020_debug(uint8_t __on);
bool motor_task_6020_debug_enabled(void);
bool motor_task_steer_debug_enabled(void);
void motor_task_aux_set_pid(uint8_t __index, float __kp, float __ki, float __kd);
/** @brief 6020 位置环在线调参(kp / 反馈速度阻尼 / 输出速度上限 rad/s) */
void motor_task_aux_set_pos_pid(uint8_t __index, float __kp, float __damp, float __max_w);
float motor_task_aux_get_w(uint8_t __index);
/** @brief 速度环实际目标 [rad/s](位置模式下 = 位置环输出,调超调就看它) */
float motor_task_aux_get_w_cmd(uint8_t __index);
float motor_task_aux_get_torque(uint8_t __index);
uint8_t motor_task_aux_online(uint8_t __index);
/** @brief 6020 的零点校准(Top)是否已完成:完成瞬间相对角才是机械 0 */
bool motor_task_6020_is_zeroed(void);
/** @brief 开关 1kHz 电机紧凑帧(cmd 0x0103,经 485 上送,上板转发到串口) */
void motor_task_set_fast_stream(uint8_t __on);
uint16_t motor_task_aux_get_encoder(uint8_t __index);
uint32_t motor_task_can3_rx_count(void);
uint32_t motor_task_can3_last_id(void);
bool motor_task_can3_get(float *__vx, float *__vy, float *__w, uint8_t *__mode);
float motor_task_aux_get_target_rpm(uint8_t __index);
float motor_task_aux_get_pos(uint8_t __index);
float motor_task_aux_get_target_rad(uint8_t __index);

/* ---- 云台 pitch DM4310(只有上板有;下位机编出来是空函数体) ---- */
uint8_t motor_task_4310_online(void);
float motor_task_4310_pos(void);
float motor_task_4310_w(void);
float motor_task_4310_t(void);
uint32_t motor_task_4310_rx_count(void);
void motor_task_4310_set_pitch(float __pos, float __kp, float __kd, float __t_ff);

#ifdef __cplusplus
}

/* C++ 专用:给底盘找零算法取具体 DJI 电机对象。
   当前台架映射:0=0x206,1=0x207,2=0x208;未配置返回 nullptr。 */
class Class_Motor_DJ;
Class_Motor_DJ *motor_task_get_dj(uint8_t __index);

#endif

#endif // MOTOR_TASK_H
