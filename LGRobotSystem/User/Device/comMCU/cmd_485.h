/* ============================================================================
 * cmd_485.h — 上/下 MCU 链路(USART2 RS485,500Hz,虚拟串口帧格式)
 *
 *   ★ 分层:放在 Device/comMCU(设备/传输层),和 dev_minipc 同级 —— 这里只管
 *     "485 帧怎么收发、字段怎么编解码";业务语义(底盘命令怎么用)在 App/cmd/cmd_task。
 *     由 Cmd_Init() 注册、Cmd_Task() 每拍调用(500 Hz)。
 *
 *   帧(固定 54 字节):
 *     [0]      0xA5
 *     [1]      channel<<6 | mode
 *     [2]      data_len = 48
 *     [3..26]  半帧 A:底盘命令(上->下) / 反馈(下->上)   24 B
 *     [27..50] 半帧 B:调试文本分片                     24 B
 *     [51..52] CRC16(小端,reflected 0x8408,init 0,覆盖 0..50)
 *     [53]     0x5A
 *
 *   ★ 一发一回:上板每 2ms 发一帧命令(通道 0),下板立刻回一帧反馈(通道 1),
 *     两帧都各自带 24 B 调试分片 ⇒ 500Hz 命令/反馈 + 双向调试透传同时满速。
 *
 *   mode 位:
 *     通道 0(命令): bit0 使能 / bit1 小陀螺 / bit2..6 下行调试有效字节数(0..24)
 *     通道 1(反馈): bit0..5 上行调试有效字节数(0..24)
 * ========================================================================== */

#ifndef CMD_485_H
#define CMD_485_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CMD485_SOF              0xA5u
#define CMD485_TAIL             0x5Au
#define CMD485_HALF_LEN         24u     /* 命令/反馈半帧 */
#define CMD485_DATA_LEN         48u     /* 整帧有效载荷 = A + B */
#define CMD485_FRAME_LEN        54u     /* 6 + 48 */

#define CMD485_CH_CMD           0u      /* 上 -> 下:命令 + 下行调试 */
#define CMD485_CH_FB            1u      /* 下 -> 上:反馈 + 上行调试 */

/* 命令 mode 低 2 位 */
#define CMD485_MODE_ENABLE      0x01u
#define CMD485_MODE_SPIN        0x02u
#define CMD485_MODE_DN_SHIFT    2u      /* 下行调试长度:mode bit2..6 */

/* 调试透传缓冲 / 整行上限(下板一行 JSON 约 1045 B) */
#define CMD485_RELAY_BUF        4096u
#define CMD485_RELAY_LINE_MAX   1280u

/* 下 -> 上反馈(半帧 A,24 B) */
typedef struct
{
    float Vx;                   /* m/s(实测) */
    float Vy;                   /* m/s */
    float W;                    /* rad/s */
    float Yaw;                  /* rad(连续 yaw) */
    float Power_W;              /* W */
    uint8_t Status;             /* bit0 在线,bit2 IMU 有效 */
    uint8_t Fault;              /* 故障码(预留) */
    uint8_t Reserved[2];
} Cmd485_Feedback_t;

/* 初始化:注册 USART2 的 RS485 定长接收(在 Robot_Init 里、Cmd_Init 时调用) */
void Cmd_485_Init(void);

/* 每拍调用(CMD 任务 500Hz):上板发命令;下板刷新反馈快照 */
void Cmd_485_Task(void);

/* 下 MCU 读上 MCU 指令;100ms 超时返回 false */
bool Cmd_485_Get_Chassis(float *__vx, float *__vy, float *__w, uint8_t *__mode);

/* 上 MCU 读下 MCU 反馈;100ms 超时返回 false */
bool Cmd_485_Get_Feedback(float *__vx, float *__vy, float *__w, uint8_t *__status);

/* 链路是否活着(100ms 内收到过合法帧) */
bool Cmd_485_Is_Link_Alive(void);

/* 调试计数 */
uint32_t Cmd_485_Get_Rx_Count(void);    /* 合法帧数 */
uint32_t Cmd_485_Get_Bad_Count(void);   /* 校验/长度错误 */
uint32_t Cmd_485_Get_Last_Channel(void);

/* ---- 调试数据透传 ----
   下板:把要查看的文本压进上行缓冲(整行压,空间不够整行丢弃)
   上板:按 '\n' 取出一条完整行,由 debug_task 加 "[L] " 前缀转发出去 */
void     Cmd_485_Relay_Push(const void *__data, uint16_t __len);
uint16_t Cmd_485_Relay_Pop_Line(uint8_t *__dst, uint16_t __max);
uint16_t Cmd_485_Relay_Used(void);
uint32_t Cmd_485_Relay_Drop(void);

/* 上 -> 下:上板串口收到的字节,随命令帧下发 */
void     Cmd_485_RelayDown_Push(const void *__data, uint16_t __len);
uint16_t Cmd_485_RelayDown_Take(uint8_t *__dst, uint16_t __max);
uint16_t Cmd_485_RelayDown_Used(void);

/* 给遥测用的原始值 */
int16_t Cmd_485_Get_Fb_Vx(void);        /* mm/s */
int16_t Cmd_485_Get_Fb_Vy(void);
int16_t Cmd_485_Get_Fb_W(void);         /* mrad/s */
uint8_t Cmd_485_Get_Fb_Status(void);

/* 上板 IMU yaw / 角速度(随命令帧下发):下板小陀螺闭环用 */
float    Cmd_485_Get_Upper_Yaw(void);
/** @brief 上板下发的云台 yaw 指令(rad,相对底盘;上板复用 Gimbal_Pitch 字段) */
float    Cmd_485_Get_Gimbal_Yaw_Cmd(void);
/** @brief 上板拨杆决定的发射指令(Shoot_Enable / Shoot_Rate);链路有效时返回 true */
bool     Cmd_485_Get_Shoot(uint8_t *__en, uint8_t *__rate);
/**
 * @brief 上板裁判系统给的底盘功率上限 [W](借半帧 A 的 Reserved 字节下发)
 * @note  裁判接收机只接在**上板 USART1**;下板没有接收机,只能拿这个值。
 *        返回 false = 链路超时或上板报告"无裁判",此时该用练习模式的保守值。
 */
bool     Cmd_485_Get_Power_Limit(uint16_t *__w);
float    Cmd_485_Get_Upper_Gyro_Z(void);
uint32_t Cmd_485_Get_Upper_Yaw_Ms(void);

#ifdef __cplusplus
}
#endif

#endif // CMD_485_H
