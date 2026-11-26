//
// Created by 20159 on 2026/3/12.
//

#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H


#include "fdcan.h"  // CubeMX 生成的 hfdcan1/2/3 extern 声明
#include "stm32h7xx_hal_fdcan.h"

//定义CAN回调函数
typedef void (*CAN_Callback)(FDCAN_RxHeaderTypeDef Header, uint8_t *Buffer);

struct CAN_Manage_Object
{
    FDCAN_HandleTypeDef *CAN_Handler;
    CAN_Callback Callback_Function;
    // 与接收相关的数据
    FDCAN_RxHeaderTypeDef Rx_Header;
    uint8_t Rx_Buffer[8];
};

extern bool system_can[3];

void can_filter_init();
void bsp_can_init(FDCAN_HandleTypeDef *hfdcan,CAN_Callback Callback_Function);

uint8_t fdcan_send_data_stand(FDCAN_HandleTypeDef *hfdcan,uint32_t id, uint8_t *data,uint32_t len);
uint8_t fdcan_send_data_Exten(FDCAN_HandleTypeDef *hfdcan,uint32_t id, uint8_t *data,uint32_t len);

/**
 * @brief 扩展帧发送(等 TX FIFO 空位,带超时,失败必须上报)
 *
 * TX FIFO 深度只有 8(CubeMX 的 TxFifoQueueElmtsNbr = 8),而灵足一次"置零"在一条
 * 总线上要灌多帧(每台电机 lose/置零/保存/lose)——用不等空位的版本会把超出的帧
 * 静默丢掉,真机现象是"每条总线只有前几个电机生效"。
 *
 * @param hfdcan     CAN 编号
 * @param id         扩展帧 id
 * @param data       数据指针
 * @param len        数据长度
 * @param timeout_ms 等空位的超时 [ms]
 * @return 0 = 已进 TX FIFO;1 = 超时/参数错(这一帧没发出去)
 */
uint8_t fdcan_send_data_Exten_wait(FDCAN_HandleTypeDef *hfdcan,uint32_t id, uint8_t *data,uint32_t len,uint32_t timeout_ms);

#endif //BSP_FDCAN_H