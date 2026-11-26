//
// imu_task.cpp — 姿态解算任务(BMI088 + 四元数 EKF)
// 说明 / 用法 / 坑 见同目录 imu_task.md
//

#include "imu_task.h"

#include "alg_quaternion_ekf.h"     /* Class_QuaternionEKF */
#include "bsp_timestamp.h"          /* dt 用 TIM5(µs) */
#include "dev_bmi088.h"             /* BMI088_Task/Read/Get_*(已是 SI 单位) */
#include "error_task.h"             /* 写关键数据快照 + IMU 错误位 */
#include "msg_center.h"             /* 发布到 MSG_TOPIC_IMU_* */

#include <string.h>

static_assert(sizeof(Imu_Data_t) <= MSG_SLOT_SIZE, "Imu_Data_t 超过 MSG_SLOT_SIZE,会被环形数组静默拒收");

/* ============================================================================
 * 参数
 * ========================================================================== */

/* EKF 参数(参考 chaiss/官方调好的那组:Q1=10, Q2=0.001, R=1000000, λ=1, lpf=0) */
#define IMU_EKF_Q1              10.0f
#define IMU_EKF_Q2              0.001f
#define IMU_EKF_R               1000000.0f
#define IMU_EKF_LAMBDA          1.0f
#define IMU_EKF_LPF             0.0f

/* dt 限幅:太短(同一微秒内重复调)跳过,太长(卡顿)按上限,避免 EKF 被 dt=0/巨大值搞炸 */
#define IMU_DT_MIN_US           100u
#define IMU_DT_MAX_US           20000u

/* ============================================================================
 * 内部状态
 * ========================================================================== */

static Class_QuaternionEKF s_ekf;
static Imu_Data_t s_imu[IMU_CHANNEL_NUM];
/* ★ 上板云台 pitch 用:上电把"当前 pitch"当成关节 0(4310 机械零点不在 0 时靠它对齐) */
static float s_pitch_zero = 0.0f;
static bool  s_pitch_zero_set = false;
static uint32_t s_last_us = 0u;

/* ============================================================================
 * 对外接口
 * ========================================================================== */

void Imu_Init(void)
{
    memset(s_imu, 0, sizeof(s_imu));
    s_last_us = 0u;

    s_ekf.Init(IMU_EKF_Q1, IMU_EKF_Q2, IMU_EKF_R, IMU_EKF_LAMBDA, IMU_EKF_LPF);
}

void Imu_Task(void)
{
    Imu_Data_t *data = &s_imu[IMU_CHANNEL_CHASSIS];
    Error_Snapshot_t *snap;
    float gx, gy, gz;
    float ax, ay, az;
    uint32_t now_us;
    uint32_t dt_us;
    float dt;

    /* ② dt:用 TIM5 的微秒时间戳(跨回绕安全) */
    now_us = Timestamp_Get_Us32();
    if (s_last_us == 0u)
    {
        s_last_us = now_us;
        return;                     /* 第一拍只记时刻 */
    }
    dt_us = (uint32_t)(now_us - s_last_us);
    if (dt_us < IMU_DT_MIN_US)
    {
        return;                     /* 还没到采样间隔 */
    }
    if (dt_us > IMU_DT_MAX_US)
    {
        dt_us = IMU_DT_MAX_US;      /* 卡顿保护 */
    }
    s_last_us = now_us;
    dt = (float)dt_us * 1.0e-6f;

    /* ③ 读一次(阻塞 SPI,几十 µs);读失败就不喂 EKF,免得把垃圾数据融进去 */
    const bool sample_ok = BMI088_Read();
    BMI088_Get_Gyro(&gx, &gy, &gz);
    BMI088_Get_Accel(&ax, &ay, &az);

    data->Online = sample_ok && BMI088_Is_Online();

    if (data->Online)
    {
        data->Gyro[0] = gx;
        data->Gyro[1] = gy;
        data->Gyro[2] = gz;
        data->Accel[0] = ax;
        data->Accel[1] = ay;
        data->Accel[2] = az;
        data->Temp_C = BMI088_Get_Temperature();

        /* ④ 融合 */
        s_ekf.Update(gx, gy, gz, ax, ay, az, dt);

        data->Euler[0] = s_ekf.Get_Roll_Rad();
        data->Euler[1] = s_ekf.Get_Pitch_Rad();
        if (s_pitch_zero_set)
        {
            data->Euler[1] -= s_pitch_zero;
        }
        data->Euler[2] = s_ekf.Get_Yaw_Rad();
        data->Yaw_Total = s_ekf.Get_YawTotalAngle_Rad();
        s_ekf.Get_Quaternion(data->Quat);
        s_ekf.Get_GyroBias(data->Gyro_Bias);
        data->ChiSquare = s_ekf.Get_ChiSquare();
        data->Update_Count = (uint32_t)s_ekf.Get_UpdateCount();
        data->Error_Count = (uint32_t)s_ekf.Get_ErrorCount();
        data->Converged = s_ekf.Is_Converged();
    }
    data->Seq++;

    /* ⑤ 发布:底盘通道 + 云台通道(本板只有一颗 IMU ⇒ 云台通道镜像底盘) */
    (void)Msg_Publish(MSG_TOPIC_IMU_CHASSIS, data, (uint16_t)sizeof(Imu_Data_t));
    s_imu[IMU_CHANNEL_GIMBAL] = *data;
    (void)Msg_Publish(MSG_TOPIC_IMU_GIMBAL, &s_imu[IMU_CHANNEL_GIMBAL], (uint16_t)sizeof(Imu_Data_t));

    /* ⑥ 写进 error_task 的关键数据快照(它按 10Hz 落盘到 Flash) */
    snap = Error_Get_Snapshot();
    snap->Imu_Euler[0] = data->Euler[0];
    snap->Imu_Euler[1] = data->Euler[1];
    snap->Imu_Euler[2] = data->Yaw_Total;   /* 快照里记连续 yaw,方便事后看小陀螺转了几圈 */

    /* ⑦ IMU 异常上报(掉线/没收敛都算,收敛前不报,免得刚上电一直橙灯) */
    Error_Set_Flag(ERROR_FLAG_MASK(ERROR_FLAG_IMU),
                   (!data->Online) || (data->Update_Count > 500u && !data->Converged));
}

/* 上板云台 pitch:把当前 pitch 记为 0(4310 机械零点对齐用) */
void Imu_Zero_Pitch_Now(void)
{
    if (!s_imu[IMU_CHANNEL_GIMBAL].Online)
    {
        return;
    }
    s_pitch_zero = s_ekf.Get_Pitch_Rad();
    s_pitch_zero_set = true;
}

const Imu_Data_t *Imu_Get(uint8_t __channel)
{
    if (__channel >= IMU_CHANNEL_NUM)
    {
        return &s_imu[IMU_CHANNEL_CHASSIS];
    }
    return &s_imu[__channel];
}

bool Imu_Is_Online(uint8_t __channel)
{
    if (__channel >= IMU_CHANNEL_NUM)
    {
        return false;
    }
    return s_imu[__channel].Online;
}

float Imu_Get_Yaw_Total(uint8_t __channel)
{
    if (__channel >= IMU_CHANNEL_NUM)
    {
        return 0.0f;
    }
    return s_imu[__channel].Yaw_Total;
}

bool Imu_Is_Converged(uint8_t __channel)
{
    if (__channel >= IMU_CHANNEL_NUM)
    {
        return false;
    }
    return s_imu[__channel].Converged;
}
