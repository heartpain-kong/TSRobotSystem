//
// can3_protocol.h — 上 MCU <-> 下 MCU CAN3 通信协议
//
// 物理层: FDCAN3, 经典 CAN, 1 Mbps, 标准帧 8 字节。
// 总线 1kHz,底盘速度帧 500Hz 发送,超时 100ms。
// 所有多字节字段小端, int16 定点。
//

#ifndef CAN3_PROTOCOL_H
#define CAN3_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN3_ID_UPPER_CMD       0x300u   /* 上 -> 下: 底盘速度 */
#define CAN3_ID_LOWER_FB        0x301u   /* 下 -> 上: 底盘反馈 */
#define CAN3_ID_UPPER_AUX       0x302u   /* 上 -> 下: yaw/拨盘 */
#define CAN3_ID_LOWER_STATUS    0x303u   /* 下 -> 上: 状态/功率 */

#define CAN3_BITRATE            1000000u
#define CAN3_BUS_HZ             1000u
#define CAN3_SEND_HZ            500u
#define CAN3_TIMEOUT_MS         100u

typedef struct
{
    uint8_t seq;
    uint8_t mode;       /* bit0 enable, bit1 spin, bit2 trigger */
    int16_t vx_mm_s;
    int16_t vy_mm_s;
    int16_t w_mrad_s;
} Can3_Cmd;

typedef struct
{
    uint8_t seq;
    uint8_t status;     /* bit0 online, bit1 fault, bit2 imu */
    int16_t vx_mm_s;
    int16_t vy_mm_s;
    int16_t w_mrad_s;
} Can3_Feedback;

typedef struct
{
    uint8_t seq;
    uint8_t flags;
    int16_t yaw_cdeg;   /* 0.01° */
    int16_t trigger_rpm;
    int16_t reserved;
} Can3_AuxCmd;

typedef struct
{
    uint8_t seq;
    uint8_t status;
    int16_t yaw_cdeg;
    int16_t power_w;
    int16_t fault;
} Can3_Status;

int Can3_EncodeCmd(const Can3_Cmd *__in, uint8_t __out[8]);
int Can3_DecodeCmd(const uint8_t __in[8], Can3_Cmd *__out);

int Can3_EncodeFeedback(const Can3_Feedback *__in, uint8_t __out[8]);
int Can3_DecodeFeedback(const uint8_t __in[8], Can3_Feedback *__out);

int Can3_EncodeAux(const Can3_AuxCmd *__in, uint8_t __out[8]);
int Can3_DecodeAux(const uint8_t __in[8], Can3_AuxCmd *__out);

int Can3_EncodeStatus(const Can3_Status *__in, uint8_t __out[8]);
int Can3_DecodeStatus(const uint8_t __in[8], Can3_Status *__out);

#ifdef __cplusplus
}
#endif

#endif // CAN3_PROTOCOL_H
