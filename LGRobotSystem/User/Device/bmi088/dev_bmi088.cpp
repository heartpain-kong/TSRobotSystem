//
// dev_bmi088.cpp — BMI088(加速度计 + 陀螺仪)驱动,走 SPI2
// 说明 / 用法 / 坑 见同目录 dev_bmi088.md

#include "dev_bmi088.h"

#include "bsp_fdcan.h"   /* 外接 IMU(CAN) */
#include "fdcan.h"
#include "bsp_timestamp.h"

#include "bsp_dwt.h"
#include "bsp_timestamp.h"

#include <math.h>
#include <string.h>

extern "C" {
extern SPI_HandleTypeDef hspi2;         /* Core/Src/spi.c */
}

/* 上电静止标定陀螺残余零偏(定义见文件后部) */
bool BMI088_Calibrate_Gyro(uint16_t __samples);

/* ============================================================================
 * 寄存器(加速度计)
 * ========================================================================== */
#define BMI088_ACC_CHIP_ID          0x00u   /* 读 → 0x1E */
#define BMI088_ACC_DATA             0x12u   /* 0x12..0x17:acc_x/y/z,各 16bit 小端 */
#define BMI088_ACC_TEMP             0x22u   /* 0x22..0x23:11bit 温度 */
#define BMI088_ACC_CONF             0x40u   /* acc_odr[7:4] | acc_bwp[3:0] */
#define BMI088_ACC_RANGE            0x41u
#define BMI088_ACC_PWR_CONF         0x7Cu   /* 0x00 = active */
#define BMI088_ACC_PWR_CTRL         0x7Du   /* 0x04 = 打开加速度计 */
#define BMI088_ACC_SOFTRESET        0x7Eu   /* 写 0xB6 */

/* 寄存器(陀螺仪) */
#define BMI088_GYR_CHIP_ID          0x00u   /* 读 → 0x0F */
#define BMI088_GYR_DATA             0x02u   /* 0x02..0x07:rate x/y/z,各 16bit 小端 */
#define BMI088_GYR_RANGE            0x0Fu
#define BMI088_GYR_BANDWIDTH        0x10u
#define BMI088_GYR_LPM1             0x11u   /* 0x00 = 正常模式 */
#define BMI088_GYR_SOFTRESET        0x14u   /* 写 0xB6 */

/* 复位命令 */
#define BMI088_SOFTRESET_CMD        0xB6u

/* CHIP_ID 期望值 */
#define BMI088_ACC_CHIP_ID_VALUE    0x1Eu
#define BMI088_GYR_CHIP_ID_VALUE    0x0Fu

/* 配置值 */
#define BMI088_ACC_CONF_VALUE       0xA8u   /* odr = 400Hz(0xA),bwp = normal(0x8) */
#define BMI088_GYR_BANDWIDTH_VALUE  0x02u   /* odr 400Hz / 滤波 47Hz */

/* 换算系数 */
#define BMI088_ACCEL_LSB_PER_G      5460.0f     /* ±6g(和 BMI088_ACCEL_RANGE_REG 对应) */
#define BMI088_GYRO_LSB_PER_DPS     16.384f     /* ±2000dps(和 BMI088_GYRO_RANGE_REG 对应) */
#define BMI088_G0                   9.80665f

/* 下 MCU IMU 校准值 */
#define GxOFFSET -0.000681414269f
#define GyOFFSET -0.00134240754f
#define GzOFFSET -0.00143384014f

/* 上电静止标定的残余零偏(由 BMI088_Calibrate_Gyro 填写,换算时额外减掉) */
static float s_gyro_bias[3] = {0.0f, 0.0f, 0.0f};
#define AxOFFSET -0.127637371f
#define AyOFFSET -0.0857178643f
#define AzOFFSET 0.0f     /* Z 轴朝上:静止时 az 必须是 +9.8,不能把重力减掉 */
#define gNORM 9.84484291f
#define BMI088_DEG_TO_RAD           0.0174532925f

/* 读写超时(ms):10MHz 下 6 字节约 5µs,给 5ms 足够 */
#define BMI088_SPI_TIMEOUT_MS       5u

/* ---- 输出 ---- */
static volatile float accel[3] = {0.0f, 0.0f, 0.0f};    /* m/s² */
static volatile float gyro[3] = {0.0f, 0.0f, 0.0f};     /* rad/s */
static volatile float temperature = 0.0f;               /* ℃ */
static volatile bool online = false;
volatile uint8_t bmi088_acc_conf = 0u;   /* 初始化时回读的 ACC_CONF(0xA8 = 正常) */

/* ★ 加速度自愈:连续读到全 0 ⇒ 说明 accel die 没在转换(偶发,实测复位后会出现)⇒ 重新配一次 */
static uint32_t s_accel_zero_cnt = 0u;
static uint32_t error_count = 0u;
static uint32_t last_read_us = 0u;

/* ============================================================================
 * 底层寄存器读写
 * ========================================================================== */

/**
 * @brief 加速度计写寄存器(写**不需要** dummy)
 * @param reg 寄存器
 * @param value 值
 * @return true = 成功
 */
static bool BMI088_Accel_Write(uint8_t reg, uint8_t value)
{
    uint8_t *tx = BSP_SPI_Get_Tx_Buffer(&hspi2);
    if (tx == nullptr)
    {
        return false;
    }
    tx[0] = reg;                    /* 写:bit7 = 0 */
    tx[1] = value;
    return BSP_SPI_Transmit_Receive(&hspi2, BMI088_ACCEL_CS_GPIO, BMI088_ACCEL_CS_PIN,
                                    BMI088_CS_ACTIVE_LEVEL, 2u, 0u, BMI088_SPI_TIMEOUT_MS);
}

/**
 * @brief 加速度计读寄存器(**多发 1 个 dummy 字节**)
 * @param reg 寄存器
 * @param buffer 输出
 * @param length 长度(<= 64)
 * @return true = 成功
 */
static bool BMI088_Accel_Read(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t *tx = BSP_SPI_Get_Tx_Buffer(&hspi2);
    uint8_t *rx = BSP_SPI_Get_Rx_Buffer(&hspi2);

    if (tx == nullptr || rx == nullptr || buffer == nullptr || length == 0u || length > 64u)
    {
        return false;
    }

    memset(tx, 0, (size_t)length + 2u);
    tx[0] = (uint8_t)(reg | 0x80u);     /* bit7 = 1 表示读 */

    /* 长度 = 1(地址) + 1(dummy) + length */
    if (!BSP_SPI_Transmit_Receive(&hspi2, BMI088_ACCEL_CS_GPIO, BMI088_ACCEL_CS_PIN,
                                  BMI088_CS_ACTIVE_LEVEL, (uint16_t)(length + 2u),
                                  (uint16_t)(length + 2u), BMI088_SPI_TIMEOUT_MS))
    {
        return false;
    }

    memcpy(buffer, &rx[2], length);     /* 数据从第 3 个字节开始 */
    return true;
}

/**
 * @brief 陀螺仪写寄存器
 * @return true = 成功
 */
static bool BMI088_Gyro_Write(uint8_t reg, uint8_t value)
{
    uint8_t *tx = BSP_SPI_Get_Tx_Buffer(&hspi2);
    if (tx == nullptr)
    {
        return false;
    }
    tx[0] = reg;
    tx[1] = value;
    return BSP_SPI_Transmit_Receive(&hspi2, BMI088_GYRO_CS_GPIO, BMI088_GYRO_CS_PIN,
                                    BMI088_CS_ACTIVE_LEVEL, 2u, 0u, BMI088_SPI_TIMEOUT_MS);
}

/**
 * @brief 陀螺仪读寄存器(**没有 dummy**)
 * @return true = 成功
 */
static bool BMI088_Gyro_Read(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t *tx = BSP_SPI_Get_Tx_Buffer(&hspi2);
    uint8_t *rx = BSP_SPI_Get_Rx_Buffer(&hspi2);

    if (tx == nullptr || rx == nullptr || buffer == nullptr || length == 0u || length > 64u)
    {
        return false;
    }

    memset(tx, 0, (size_t)length + 1u);
    tx[0] = (uint8_t)(reg | 0x80u);

    if (!BSP_SPI_Transmit_Receive(&hspi2, BMI088_GYRO_CS_GPIO, BMI088_GYRO_CS_PIN,
                                  BMI088_CS_ACTIVE_LEVEL, (uint16_t)(length + 1u),
                                  (uint16_t)(length + 1u), BMI088_SPI_TIMEOUT_MS))
    {
        return false;
    }

    memcpy(buffer, &rx[1], length);     /* 数据从第 2 个字节开始 */
    return true;
}

/* ============================================================================
 * 初始化
 * ========================================================================== */

/**
 * @brief 初始化 BMI088:软复位 → CHIP_ID 自检 → 配置
 * @return true = 两颗 die 都在
 */
bool BMI088_Init(void)
{
    uint8_t id = 0u;

    if (BSP_SPI_Get_Tx_Buffer(&hspi2) == nullptr)
    {
        return false;       /* 没先 BSP_SPI_Init(&hspi2, ...) */
    }

    /* ---- ① 陀螺仪:软复位 → 自检 → 配置 ---- */
    if (!BMI088_Gyro_Write(BMI088_GYR_SOFTRESET, BMI088_SOFTRESET_CMD))
    {
        error_count++;
        return false;
    }
    DWT_Delay(0.03f);                                   /* 手册:复位后等 30ms */

    if (!BMI088_Gyro_Read(BMI088_GYR_CHIP_ID, &id, 1u) || id != BMI088_GYR_CHIP_ID_VALUE)
    {
        error_count++;
        return false;                                   /* 陀螺不在(0x0F) */
    }

    if (!BMI088_Gyro_Write(BMI088_GYR_RANGE, BMI088_GYRO_RANGE_REG) ||
        !BMI088_Gyro_Write(BMI088_GYR_BANDWIDTH, BMI088_GYR_BANDWIDTH_VALUE) ||
        !BMI088_Gyro_Write(BMI088_GYR_LPM1, 0x00u))     /* 正常模式(非省电) */
    {
        error_count++;
        return false;
    }

    /* ---- ② 加速度计:软复位 → 打开 → 进 active → 配置 ---- */
    if (!BMI088_Accel_Write(BMI088_ACC_SOFTRESET, BMI088_SOFTRESET_CMD))
    {
        error_count++;
        return false;
    }
    DWT_Delay(0.05f);                                   /* 手册:复位后等 ≥50ms */

    /* 打开加速度计(0x04 = acc_en) */
    if (!BMI088_Accel_Write(BMI088_ACC_PWR_CTRL, 0x04u))
    {
        error_count++;
        return false;
    }

    /* ★ 手册要求:acc_pwr_ctrl=0x04 之后等 >=5ms 才能配置 ACC_CONF/ACC_RANGE,
       否则写入被忽略 ⇒ odr=0 ⇒ 加速度数据恒 0(踩过) */
    HAL_Delay(10u);
    DWT_Delay(0.05f);

    /* 进 active 模式:只有 active 下才允许改 ACC_CONF/ACC_RANGE */
    if (!BMI088_Accel_Write(BMI088_ACC_PWR_CONF, 0x00u))
    {
        error_count++;
        return false;
    }
    DWT_Delay(0.005f);

    /* ⚠️ 复位后第一次读 CHIP_ID 可能是垃圾值 —— 读两次,用第二次判断 */
    (void)BMI088_Accel_Read(BMI088_ACC_CHIP_ID, &id, 1u);
    if (!BMI088_Accel_Read(BMI088_ACC_CHIP_ID, &id, 1u) || id != BMI088_ACC_CHIP_ID_VALUE)
    {
        error_count++;
        return false;                                   /* 加速度计不在(0x1E) */
    }

    if (!BMI088_Accel_Write(BMI088_ACC_CONF, BMI088_ACC_CONF_VALUE) ||
        !BMI088_Accel_Write(BMI088_ACC_RANGE, BMI088_ACCEL_RANGE_REG))
    {
        error_count++;
        return false;
    }

    /* 回读 ACC_CONF:写没写进去一目了然(读出来不是 0xA8 ⇒ 加速度计没配好) */
    {
        uint8_t conf = 0u;
        (void)BMI088_Accel_Read(BMI088_ACC_CONF, &conf, 1u);
        bmi088_acc_conf = conf;                 /* 记下来给遥测看 */
        if (conf != BMI088_ACC_CONF_VALUE)
        {
            (void)BMI088_Accel_Write(BMI088_ACC_CONF, BMI088_ACC_CONF_VALUE);
            (void)BMI088_Accel_Read(BMI088_ACC_CONF, &conf, 1u);
            bmi088_acc_conf = conf;
        }
    }

    online = true;

    /* ★ 上电静止标定:陀螺残余零偏(z 轴零偏 = 小陀螺航向漂移的根因) */
    (void)BMI088_Calibrate_Gyro(500u);

    return true;
}

/* ============================================================================
 * 读取与换算
 * ========================================================================== */

/**
 * @brief 读一帧(加速度 6B + 温度 2B + 角速度 6B)
 * @return true = 成功
 */
bool BMI088_Read(void)
{
    uint8_t buffer[8];

    if (!online)
    {
        return false;
    }

    /* ---- 加速度计:0x12..0x17 连续 6 字节 ---- */
    if (!BMI088_Accel_Read(BMI088_ACC_DATA, buffer, 6u))
    {
        error_count++;
        return false;
    }
    {
        int16_t raw_x = (int16_t)(((uint16_t)buffer[1] << 8) | buffer[0]);
        int16_t raw_y = (int16_t)(((uint16_t)buffer[3] << 8) | buffer[2]);
        int16_t raw_z = (int16_t)(((uint16_t)buffer[5] << 8) | buffer[4]);

        accel[0] = ((float)raw_x / BMI088_ACCEL_LSB_PER_G) * BMI088_G0 - AxOFFSET;
        accel[1] = ((float)raw_y / BMI088_ACCEL_LSB_PER_G) * BMI088_G0 - AyOFFSET;
        accel[2] = ((float)raw_z / BMI088_ACCEL_LSB_PER_G) * BMI088_G0 - AzOFFSET;

        /* ★ 自愈:连续 500 次全 0(静止时 az 不可能是 0)⇒ accel die 没在转换 ⇒ 重配 */
        if ((raw_x == 0) && (raw_y == 0) && (raw_z == 0))
        {
            s_accel_zero_cnt++;
            if (s_accel_zero_cnt == 500u)
            {
                (void)BMI088_Accel_Write(BMI088_ACC_PWR_CTRL, 0x04u);
                HAL_Delay(10u);
                (void)BMI088_Accel_Write(BMI088_ACC_PWR_CONF, 0x00u);
                (void)BMI088_Accel_Write(BMI088_ACC_CONF, BMI088_ACC_CONF_VALUE);
                (void)BMI088_Accel_Read(BMI088_ACC_CONF, (uint8_t *)&bmi088_acc_conf, 1u);
            }
        }
        else
        {
            s_accel_zero_cnt = 0u;
        }
    }

    /* ---- 温度(在加速度计 die 里):11bit,0.125℃/LSB,偏置 23℃ ---- */
    if (BMI088_Accel_Read(BMI088_ACC_TEMP, buffer, 2u))
    {
        int16_t raw_temp = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
        temperature = ((float)(raw_temp >> 5)) * 0.125f + 23.0f;
    }

    /* ---- 陀螺仪:0x02..0x07 连续 6 字节 ---- */
    if (!BMI088_Gyro_Read(BMI088_GYR_DATA, buffer, 6u))
    {
        error_count++;
        return false;
    }
    {
        int16_t raw_x = (int16_t)(((uint16_t)buffer[1] << 8) | buffer[0]);
        int16_t raw_y = (int16_t)(((uint16_t)buffer[3] << 8) | buffer[2]);
        int16_t raw_z = (int16_t)(((uint16_t)buffer[5] << 8) | buffer[4]);

        gyro[0] = ((float)raw_x / BMI088_GYRO_LSB_PER_DPS) * BMI088_DEG_TO_RAD - GxOFFSET - s_gyro_bias[0];
        gyro[1] = ((float)raw_y / BMI088_GYRO_LSB_PER_DPS) * BMI088_DEG_TO_RAD - GyOFFSET - s_gyro_bias[1];
        gyro[2] = ((float)raw_z / BMI088_GYRO_LSB_PER_DPS) * BMI088_DEG_TO_RAD - GzOFFSET - s_gyro_bias[2];
    }

    last_read_us = Timestamp_Get_Us32();
    return true;
}

/**
 * @brief 上电静止标定陀螺残余零偏:采 N 帧取平均,只有静止才接受
 *        z 轴零偏是"小陀螺航向缓慢漂移"的根因,标完在换算里减掉
 * @return true = 标定成功;false = 不静止/采样失败(保持原硬编码偏置)
 */
bool BMI088_Calibrate_Gyro(uint16_t __samples)
{
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    float ax = 0.0f, ay = 0.0f, az = 0.0f;

    s_gyro_bias[0] = 0.0f;
    s_gyro_bias[1] = 0.0f;
    s_gyro_bias[2] = 0.0f;

    if ((__samples == 0u) || !online)
    {
        return false;
    }

    for (uint16_t i = 0u; i < __samples; i++)
    {
        if (!BMI088_Read())
        {
            return false;
        }
        sx += gyro[0];  sy += gyro[1];  sz += gyro[2];
        ax += accel[0]; ay += accel[1]; az += accel[2];
        HAL_Delay(1u);
    }

    {
        const float n = (float)__samples;
        const float gn = sqrtf(sx * sx + sy * sy + sz * sz) / n;
        const float an = sqrtf(ax * ax + ay * ay + az * az) / n;

        if ((gn > 0.3f) || (an < 9.3f) || (an > 10.3f))
        {
            return false;       /* 不是静止状态,不标定 */
        }

        s_gyro_bias[0] = sx / n;
        s_gyro_bias[1] = sy / n;
        s_gyro_bias[2] = sz / n;
    }

    return true;
}

/**
 * @brief 跑一拍:到周期就读一帧
 */
void BMI088_Task(void)
{
    if (!online)
    {
        return;
    }
    if ((Timestamp_Get_Us32() - last_read_us) >= (BMI088_TASK_PERIOD_MS * 1000u))
    {
        (void)BMI088_Read();
    }
}

/* ============================================================================
 * 取值
 * ========================================================================== */
void BMI088_Get_Accel(float *x, float *y, float *z)
{
    if (x != nullptr) { *x = accel[0]; }
    if (y != nullptr) { *y = accel[1]; }
    if (z != nullptr) { *z = accel[2]; }
}

void BMI088_Get_Gyro(float *x, float *y, float *z)
{
    if (x != nullptr) { *x = gyro[0]; }
    if (y != nullptr) { *y = gyro[1]; }
    if (z != nullptr) { *z = gyro[2]; }
}

float BMI088_Get_Temperature(void)
{
    return temperature;
}

bool BMI088_Is_Online(void)
{
    return online;
}

uint32_t BMI088_Get_Error_Count(void)
{
    return error_count;
}


/* ============================================================================
 * 外接 IMU(达妙 DM-IMU-L1,CAN)—原 dev_imu_ext 并全
 * ========================================================================== */
//
// dev_imu_ext.cpp — 第二陀螺仪(达妙 DM-IMU-L1,CAN 接口)实现
//
// 协议对照: dm-imu-master/02.例程/MC02_CAN收发例程/DM-IMU/dm_imu.c
//   REG(reg_id_e): REBOOT=0 ACCEL=1 GYRO=2 EULER=3 QUAT=4 SET_ZERO=5
//                  ACCEL_CALI=6 GYRO_CALI=7 MAG_CALI=8 CHANGE_COM=9 SET_DELAY=10
//                  CHANGE_ACTIVE=11 SET_BAUD=12 SET_CAN_ID=13 SET_MST_ID=14
//                  DATA_OUTPUT_SELECTION=15 SAVE_PARAM=254 RESTORE=255
//   ac: 0 = 读, 1 = 写
//   端口: COM_USB=0 COM_RS485=1 COM_CAN=2 COM_VOFA=3
//   波特: CAN_BAUD_1M=0
//





/* ============================================================================
 * 协议常量
 * ========================================================================== */

#define IMU_REG_ACCEL_DATA      1u
#define IMU_REG_GYRO_DATA       2u
#define IMU_REG_EULER_DATA      3u
#define IMU_REG_QUAT_DATA       4u
#define IMU_REG_CHANGE_ACTIVE   11u
#define IMU_REG_SET_BAUD        12u
#define IMU_REG_DATA_OUTPUT     15u
#define IMU_REG_SAVE_PARAM      254u

#define IMU_AC_READ             0u
#define IMU_AC_WRITE            1u

#define IMU_COM_CAN             2u
#define IMU_BAUD_1M             0u

/* 量程(官方 dm_imu.h) */
#define ACCEL_CAN_MAX           235.2f
#define ACCEL_CAN_MIN           (-235.2f)
#define GYRO_CAN_MAX            34.88f
#define GYRO_CAN_MIN            (-34.88f)
#define PITCH_CAN_MAX           90.0f
#define PITCH_CAN_MIN           (-90.0f)
#define ROLL_CAN_MAX            180.0f
#define ROLL_CAN_MIN            (-180.0f)
#define YAW_CAN_MAX             180.0f
#define YAW_CAN_MIN             (-180.0f)

#define IMU_EXT_DEG_TO_RAD              0.017453292519943295f

/* ============================================================================
 * 内部状态
 * ========================================================================== */

static Imu_Ext_Data_t s_imu_ext_data;
static uint32_t s_last_rx_us = 0u;
static bool s_inited = false;

/* debug counters (read via debugger; harmless) */
volatile uint32_t s_dbg_tx_calls = 0u;
volatile uint32_t s_dbg_tx_ok    = 0u;
volatile uint32_t s_dbg_tx_fifo  = 0u;
volatile uint32_t s_dbg_cb       = 0u;
volatile uint32_t s_dbg_init_ret  = 0xFFFFFFFFu;
volatile uint32_t s_dbg_filt_ret  = 0xFFFFFFFFu;
volatile uint32_t s_dbg_gfilt_ret = 0xFFFFFFFFu;
volatile uint32_t s_dbg_start_ret = 0xFFFFFFFFu;
volatile uint32_t s_dbg_notif_ret = 0xFFFFFFFFu;
volatile uint32_t s_dbg_state     = 0xFFFFFFFFu;

/* ============================================================================
 * 官方换算(照抄 dm_imu.c)
 * ========================================================================== */

static int Imu_Ext_Float_To_Uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

static float Imu_Ext_Uint_To_Float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* ============================================================================
 * 发命令: [0xCC, reg, ac, 0xDD, d0..d3] 共 8 字节
 * ========================================================================== */

bool Imu_Ext_Command(uint8_t reg_id, bool write, uint32_t data)
{
    uint8_t buf[8];

    buf[0] = 0xCCu;
    buf[1] = reg_id;
    buf[2] = write ? IMU_AC_WRITE : IMU_AC_READ;
    buf[3] = 0xDDu;
    buf[4] = (uint8_t)(data & 0xFFu);
    buf[5] = (uint8_t)((data >> 8) & 0xFFu);
    buf[6] = (uint8_t)((data >> 16) & 0xFFu);
    buf[7] = (uint8_t)((data >> 24) & 0xFFu);

    s_dbg_tx_calls++;
    s_dbg_tx_fifo = (uint32_t)HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3);

    if (fdcan_send_data_stand(&hfdcan3, IMU_EXT_CAN_ID, buf, 8u) != 0u)
    {
        s_dbg_tx_ok++;
        return true;
    }
    return false;
}

/* ============================================================================
 * 收帧: pData[0] = reg(回显), pData[2..7] = 数据(16bit 小端压缩)
 * ========================================================================== */

void Imu_Ext_Rx_Dispatch(uint32_t __id, const uint8_t *__buffer)
{
    const uint8_t *Buffer = __buffer;

    (void)__id;

    s_dbg_cb++;

    if (Buffer == nullptr)
    {
        return;
    }

    switch (Buffer[0])
    {
        case IMU_REG_ACCEL_DATA:
        {
            uint16_t ax = (uint16_t)(((uint16_t)Buffer[3] << 8) | Buffer[2]);
            uint16_t ay = (uint16_t)(((uint16_t)Buffer[5] << 8) | Buffer[4]);
            uint16_t az = (uint16_t)(((uint16_t)Buffer[7] << 8) | Buffer[6]);

            s_imu_ext_data.Accel[0] = Imu_Ext_Uint_To_Float(ax, ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
            s_imu_ext_data.Accel[1] = Imu_Ext_Uint_To_Float(ay, ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
            s_imu_ext_data.Accel[2] = Imu_Ext_Uint_To_Float(az, ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
            break;
        }

        case IMU_REG_GYRO_DATA:
        {
            uint16_t gx = (uint16_t)(((uint16_t)Buffer[3] << 8) | Buffer[2]);
            uint16_t gy = (uint16_t)(((uint16_t)Buffer[5] << 8) | Buffer[4]);
            uint16_t gz = (uint16_t)(((uint16_t)Buffer[7] << 8) | Buffer[6]);

            /* 官方给的是 °/s,这里统一换算成 rad/s(和板载 BMI088 一致) */
            s_imu_ext_data.Gyro[0] = Imu_Ext_Uint_To_Float(gx, GYRO_CAN_MIN, GYRO_CAN_MAX, 16) * IMU_EXT_DEG_TO_RAD;
            s_imu_ext_data.Gyro[1] = Imu_Ext_Uint_To_Float(gy, GYRO_CAN_MIN, GYRO_CAN_MAX, 16) * IMU_EXT_DEG_TO_RAD;
            s_imu_ext_data.Gyro[2] = Imu_Ext_Uint_To_Float(gz, GYRO_CAN_MIN, GYRO_CAN_MAX, 16) * IMU_EXT_DEG_TO_RAD;
            break;
        }

        case IMU_REG_EULER_DATA:
        {
            uint16_t pitch = (uint16_t)(((uint16_t)Buffer[3] << 8) | Buffer[2]);
            uint16_t yaw   = (uint16_t)(((uint16_t)Buffer[5] << 8) | Buffer[4]);
            uint16_t roll  = (uint16_t)(((uint16_t)Buffer[7] << 8) | Buffer[6]);

            /* 注意官方顺序:pitch / yaw / roll(不是 xyz) */
            s_imu_ext_data.Pitch = Imu_Ext_Uint_To_Float(pitch, PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
            s_imu_ext_data.Yaw   = Imu_Ext_Uint_To_Float(yaw,   YAW_CAN_MIN,   YAW_CAN_MAX,   16);
            s_imu_ext_data.Roll  = Imu_Ext_Uint_To_Float(roll,  ROLL_CAN_MIN,  ROLL_CAN_MAX,  16);
            break;
        }

        default:
            return;     /* 其它帧(quat 等)暂不处理 */
    }

    s_last_rx_us = Timestamp_Get_Us32();
    s_imu_ext_data.Rx_Count++;
    s_imu_ext_data.Seq++;
    s_imu_ext_data.Online = true;
}

/* bsp_fdcan 的原始回调:只在没有别的模块接管 FDCAN3 时用得上
   (cmd_can3 接管后,IMU 帧由 Cmd_Can3_Rx 转发进上面的 Dispatch) */
static void Imu_Ext_Rx_Callback(FDCAN_RxHeaderTypeDef Header, uint8_t *Buffer)
{
    Imu_Ext_Rx_Dispatch(Header.Identifier, Buffer);
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

bool Imu_Ext_Init(void)
{
    FDCAN_FilterTypeDef filter;
    HAL_StatusTypeDef st;

    memset(&s_imu_ext_data, 0, sizeof(s_imu_ext_data));
    s_inited = false;

    /* 1) 若已初始化就不重做(只有 RESET 时才 Init) */
    if (HAL_FDCAN_GetState(&hfdcan3) == HAL_FDCAN_STATE_RESET)
    {
        st = HAL_FDCAN_Init(&hfdcan3);
    }
    else
    {
        st = HAL_OK;
    }
    s_dbg_init_ret = (uint32_t)st;

    /* 2) 过滤器:全接受 -> FIFO0 */
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0u;
    filter.FilterType   = FDCAN_FILTER_RANGE;   /* official: accept 0x01..0xFF */
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x01u;
    filter.FilterID2    = 0xFFu;
    s_dbg_filt_ret  = (uint32_t)HAL_FDCAN_ConfigFilter(&hfdcan3, &filter);
    s_dbg_gfilt_ret = (uint32_t)HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,
                                                             FDCAN_ACCEPT_IN_RX_FIFO0,
                                                             FDCAN_ACCEPT_IN_RX_FIFO0,
                                                             FDCAN_REJECT_REMOTE,
                                                             FDCAN_REJECT_REMOTE);

    /* 3) 启动 + 通知 */
    s_dbg_start_ret = (uint32_t)HAL_FDCAN_Start(&hfdcan3);
    s_dbg_notif_ret = (uint32_t)HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0u);

    /* 4) 把回调也挂给 bsp_fdcan(万一中断能来) */
    bsp_can_init(&hfdcan3, Imu_Ext_Rx_Callback);

    /* 5) 请求应答模式(官方同) */
    (void)Imu_Ext_Command(IMU_REG_SET_BAUD, true, IMU_BAUD_1M);
    (void)Imu_Ext_Command(IMU_REG_ACCEL_DATA, false, 0u);
    (void)Imu_Ext_Command(IMU_REG_GYRO_DATA, false, 0u);

    s_dbg_state = (uint32_t)HAL_FDCAN_GetState(&hfdcan3);

    s_inited = true;
    return true;
}

void Imu_Ext_Task(void)
{
    static uint32_t s_next_req_us = 0u;
    FDCAN_FilterTypeDef filter;

    if (!s_inited)
    {
        return;
    }

    /* ★ BUS-OFF 自动恢复:
       模块的供电晚到位(或总线异常)时控制器会进 BUS-OFF,
       而 HAL 不会自动恢复 ⇒ 一直收不到✗ */
    static uint32_t s_next_bo_us = 0u;

    if (((hfdcan3.Instance->PSR & FDCAN_PSR_BO) != 0u) &&
        ((int32_t)(Timestamp_Get_Us32() - s_next_bo_us) >= 0))
    {
        s_next_bo_us = Timestamp_Get_Us32() + 1000000u;   /* 最多每 1 s 恢复一次 */
        (void)HAL_FDCAN_DeInit(&hfdcan3);
        if (HAL_FDCAN_Init(&hfdcan3) == HAL_OK)
        {
            filter.IdType       = FDCAN_STANDARD_ID;
            filter.FilterIndex  = 0u;
            filter.FilterType   = FDCAN_FILTER_RANGE;
            filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
            filter.FilterID1    = 0x01u;
            filter.FilterID2    = 0xFFu;
            (void)HAL_FDCAN_ConfigFilter(&hfdcan3, &filter);
            (void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,
                                               FDCAN_ACCEPT_IN_RX_FIFO0,
                                               FDCAN_ACCEPT_IN_RX_FIFO0,
                                               FDCAN_REJECT_REMOTE,
                                               FDCAN_REJECT_REMOTE);
            (void)HAL_FDCAN_Start(&hfdcan3);
            (void)HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0u);
            s_dbg_state = (uint32_t)HAL_FDCAN_GetState(&hfdcan3);
        }
    }

    /* 接收走 bsp_fdcan 的 RX FIFO0 中断 → Imu_Ext_Rx_Callback;
       这里不再轮询 FIFO(会和中断抢同一个 FIFO),只刷新状态快照 */
    s_dbg_state = (uint32_t)HAL_FDCAN_GetState(&hfdcan3);

    /* 请求应答模式(官方同):每 10 ms 请一次加速度+角速度 ✓
       (原来写的 1000u 实际是 1 ms ⇒ 两板同时请求会把 CAN3 占掉一大半,
        CAN3 现在还要跑上下 MCU 链路,所以按注释改成 10 ms) */
    if ((int32_t)(Timestamp_Get_Us32() - s_next_req_us) >= 0)
    {
        s_next_req_us = Timestamp_Get_Us32() + 10000u;
        (void)Imu_Ext_Command(IMU_REG_ACCEL_DATA, false, 0u);
        (void)Imu_Ext_Command(IMU_REG_GYRO_DATA, false, 0u);
    }

    /* 超时判定:200 ms 没帧 => 离线 */
    if ((Timestamp_Get_Us32() - s_last_rx_us) > (IMU_EXT_LOST_MS * 1000u))
    {
        s_imu_ext_data.Online = false;
    }
}

const Imu_Ext_Data_t *Imu_Ext_Get(void)
{
    return &s_imu_ext_data;
}

bool Imu_Ext_Is_Online(void)
{
    return s_imu_ext_data.Online;
}
