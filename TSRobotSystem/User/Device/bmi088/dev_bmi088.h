/* ============================================================================
 * dev_bmi088.h — BMI088 六轴 IMU(加速度计 + 陀螺仪)驱动,走 SPI2
 * 说明 / 用法 / 坑 见同目录 dev_bmi088.md
 * ========================================================================== */

#ifndef DEV_BMI088_H
#define DEV_BMI088_H

#include "bsp_spi.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 片选(和 Core/Src/gpio.c 里的 ACCEL_CS / GYRO_CS 一致)---- */
#define BMI088_ACCEL_CS_GPIO        GPIOC
#define BMI088_ACCEL_CS_PIN         GPIO_PIN_0
#define BMI088_GYRO_CS_GPIO         GPIOC
#define BMI088_GYRO_CS_PIN          GPIO_PIN_3
#define BMI088_CS_ACTIVE_LEVEL      GPIO_PIN_RESET     /* 低电平有效 */

/* 读一帧的周期(W25Q64 那种非阻塞不适用 —— IMU 读几个字节 ~20µs,直接阻塞读完) */
#define BMI088_TASK_PERIOD_MS       1u

/* 量程(改这里同时要改换算系数,见 .cpp 里的 BMI088_ACCEL_LSB_PER_G) */
#define BMI088_ACCEL_RANGE_REG      0x01u   /* 0x00 ±3g / 0x01 ±6g / 0x02 ±12g / 0x03 ±24g */
#define BMI088_GYRO_RANGE_REG       0x00u   /* 0x00 ±2000 / 0x01 ±1000 / 0x02 ±500 … */

/**
 * @brief 初始化:软复位两颗 die → CHIP_ID 自检 → 配置量程/带宽
 * @return true = 两颗 die 的 ID 都对(加速度 0x1E、陀螺 0x0F)
 * @note 前置条件:MX_SPI2_Init() 跑过 + BSP_SPI_Init(&hspi2, 回调)
 * @note 里面有几十 ms 的延时(等复位/上电稳定),请在**调度器起来之前**或早期任务里调
 */
bool BMI088_Init(void);

/**
 * @brief 读一帧:加速度 6B + 温度 2B + 角速度 6B(阻塞,约 20µs @10MHz)
 * @return true = 成功
 */
bool BMI088_Read(void);

/**
 * @brief ★ 跑一拍:按 BMI088_TASK_PERIOD_MS 周期调用 BMI088_Read()
 * @note 挂在 RTOS 任务里(比如 1kHz 的电机任务,或专门的 IMU 任务)
 */
void BMI088_Task(void);

/**
 * @brief 取加速度(已换算,单位 m/s²)
 * @param x/y/z 输出(传 NULL 表示不取那一轴)
 * @return void
 */
void BMI088_Get_Accel(float *x, float *y, float *z);

/**
 * @brief 取角速度(已换算,单位 rad/s)
 * @return void
 */
void BMI088_Get_Gyro(float *x, float *y, float *z);

/**
 * @brief 取温度(BMI088 的温度在加速度计 die 里,单位 ℃)
 * @return 温度
 */
float BMI088_Get_Temperature(void);

/**
 * @brief 初始化时回读到的 ACC_CONF(正常应为 0xA8;不是就说明加速度计没配好)
 */
extern volatile uint8_t bmi088_acc_conf;

/**
 * @brief IMU 是否在线(自检通过且最近一次读成功)
 * @return true = 在线
 */
bool BMI088_Is_Online(void);

/**
 * @brief 读失败次数(调试用)
 * @return 计数
 */
uint32_t BMI088_Get_Error_Count(void);



/* ============================================================================
 * 外接 IMU(第二颗 BMI088,达妙 DM-IMU-L1,走 CAN)
 *   —和板载那颗同型号(一颗 SPI2、一颗 FDCAN3)✓
 *   协议来源: dm-imu-master/02.例程/MC02_CAN收发例程/DM-IMU/dm_imu.c
 * ========================================================================== */
//
// dev_imu_ext.h — 第二陀螺仪(达妙 DM-IMU-L1,BMI088 六轴,CAN 接口)
// 说明 / 用法 / 上板自检 见同目录 dev_imu_ext.md
//
// 协议来源:达妙官方例程 dm-imu-master/02.例程/MC02_CAN收发例程/DM-IMU/dm_imu.c
//   · 命令帧 [0xCC, reg_id, ac, 0xDD, d0..d3]  -> imu.can_id
//   · 数据帧 16bit 压缩(不是 IEEE754)         -> mst_id
//






/* ---- 配置:按模块实际烧录的 ID 改 ---- */
#define IMU_EXT_CAN_ID          0x01u      /* IMU 自己的 CAN ID(出厂默认 0x00) */
#define IMU_EXT_MST_ID          0x11u      /* 主机(我们)的 ID(和 IMU 里配的一致) */
#define IMU_EXT_LOST_MS         200u       /* 超过这个时间没数据 => 离线 */

typedef struct
{
    float Gyro[3];      /* rad/s(已由 °/s 换算) */
    float Accel[3];     /* m/s2 */
    float Pitch;        /* deg */
    float Roll;         /* deg */
    float Yaw;          /* deg */
    float Temp_C;
    uint32_t Rx_Count;  /* 收到多少帧 */
    uint32_t Seq;       /* 帧序号(丢帧可算) */
    bool Online;        /* 在线 */
} Imu_Ext_Data_t;

/**
 * @brief 初始化:绑定 FDCAN3 + 切到主动上报模式
 * @return true = 初始化成功(注意:不代表模块已应答,要看 Online)
 */
bool Imu_Ext_Init(void);

/**
 * @brief 周期调用(建议 1 kHz):维护在线/离线判定
 */
void Imu_Ext_Task(void);

/**
 * @brief 取数据(只读)
 */
const Imu_Ext_Data_t *Imu_Ext_Get(void);

/**
 * @brief 是否在线
 */
bool Imu_Ext_Is_Online(void);

/**
 * @brief 发一条命令(调试/配置用)
 * @param reg_id 寄存器号(见 .md)
 * @param write  true=写 false=读
 * @param data   写入的值(读忽略)
 */
bool Imu_Ext_Command(uint8_t reg_id, bool write, uint32_t data);

/**
 * @brief 把一帧 CAN 数据交给外接 IMU 解析
 * @note FDCAN3 每根总线只有一个回调槽,而 cmd 层(CAN3 链路)要注册它,
 *       所以 cmd 层收到非 0x300~0x303 的帧时调用本函数转发进来
 * @param __id      CAN 标识符(标准帧)
 * @param __buffer  8 字节数据
 */
void Imu_Ext_Rx_Dispatch(uint32_t __id, const uint8_t *__buffer);
#ifdef __cplusplus
}
#endif

#endif // DEV_BMI088_H
