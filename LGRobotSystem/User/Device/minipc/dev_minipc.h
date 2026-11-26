/* ============================================================================
 * dev_minipc.h — miniPC 通信(USB CDC 虚拟串口)
 * 说明 / 用法 / 坑 见同目录 dev_minipc.md
 * ========================================================================== */

#ifndef DEV_MINIPC_H
#define DEV_MINIPC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 虚拟串口帧格式 ----------------
   [0]      0xA5
   [1]      control: bit7..6 = 通道(0=RC,1=RM), bit5..0 = mode
   [2]      payload 长度 n (8bit)
   [3..]    data
   [n+3..]  CRC16 小端
   [n+5]    帧尾 0x5A
   CRC16 : 0x8408 反射, init 0, 覆盖 [0 .. n+2]
   ------------------------------------------------------------------------- */
#define MINIPC_SOF                0xA5u
#define MINIPC_MIN_FRAME          6u
#define MINIPC_MAX_DATA           250u
#define MINIPC_LOST_TIMEOUT_MS    200u

#define MINIPC_CH_RC              0u
#define MINIPC_CH_RM              1u

/**
 * @brief MINIPC_ID_CMD 的数据段(小端,24 字节;和 Cmd_Data_t 的字段对齐)
 */
typedef struct
{
    uint8_t Mode;               /* 目标模式(Enum_Cmd_Mode) */
    uint8_t Shoot_Enable;
    uint8_t Shoot_Rate;
    uint8_t Reserved;
    float Chassis_Vx;           /* m/s */
    float Chassis_Vy;           /* m/s */
    float Chassis_W;            /* rad/s */
    float Gimbal_Yaw;           /* rad(相对关节角) */
    float Gimbal_Pitch;         /* rad */
} MiniPC_Cmd_Payload_t;

/**
 * @brief 解析结果(发布在 MSG_TOPIC_MINIPC)
 */
typedef struct
{
    MiniPC_Cmd_Payload_t Cmd;
    uint8_t Channel;
    uint8_t Mode;
    bool Online;
    uint32_t Last_Rx_Ms;
    uint32_t Rx_Frames;         /* 校验通过的帧数 */
    uint32_t Crc8_Errors;   /* reserved, new protocol has no CRC8 */
    uint32_t Crc16_Errors;
    uint32_t Reject_Count;
    uint32_t Lost_Count;
    uint32_t Seq;               /* 收到一帧 +1(cmd 用它判断"有没有新指令") */
} MiniPC_Data_t;

/**
 * @brief 初始化(清状态;USB 本身由 CubeMX 的 MX_USB_DEVICE_Init 初始化)
 */
bool MiniPC_Init(void);

/**
 * @brief 周期跑(由 CMD 任务按 100Hz 调用):掉线检测 + 有新数据就发布到 MSG_TOPIC_MINIPC
 */
void MiniPC_Task(void);

/**
 * @brief USB CDC 收到数据的入口(★ 在 USB 中断里调用:只搬运,不解析、不打印)
 * @param __buf 数据
 * @param __len 长度
 */
void MiniPC_Rx_Callback(uint8_t *__buf, uint16_t __len);

/**
 * @brief 读解析结果
 */
const MiniPC_Data_t *MiniPC_Get(void);

/**
 * @brief 上位机是否在线
 */
bool MiniPC_Is_Online(void);

/**
 * @brief 回传给上位机(USB CDC 发送)
 * @param __data 数据
 * @param __len  长度
 * @return true = 已提交(USB 忙时返回 false,调用方自己决定重发)
 */
bool MiniPC_Send(const void *__data, uint16_t __len);

#ifdef __cplusplus
}
#endif

#endif // DEV_MINIPC_H
