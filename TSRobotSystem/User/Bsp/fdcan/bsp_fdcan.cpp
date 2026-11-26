//
// Created by 20159 on 2026/3/12.
//

#include "bsp_fdcan.h"

//CAN_id初始化记录
bool system_can[3];

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

//CAN结构体
static CAN_Manage_Object CAN1_Manage_Object = {};   //只在本文件用,加 static 避免意外外部链接
static CAN_Manage_Object CAN2_Manage_Object = {};   //只在本文件用,加 static 避免意外外部链接
static CAN_Manage_Object CAN3_Manage_Object = {};   //只在本文件用,加 static 避免意外外部链接

/* 回调指针只允许落在 Flash 里(0x08000000~0x08100000);
   0xFFFFFFFF / 0x00000000 这类值一律视为未初始化,不能 blx */
static inline bool can_callback_is_valid(CAN_Callback cb)
{
    const uint32_t addr = (uint32_t)cb;
    return (addr >= 0x08000000u) && (addr < 0x08100000u);
}

/**
 * @brief CAN过滤器配置
 * @param hfdcan  can编号
 * @return void
 */
void can_filter_init()
{
    
    FDCAN_FilterTypeDef fdcan_filter;
   
    fdcan_filter.IdType = FDCAN_STANDARD_ID;


    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x00;
    fdcan_filter.FilterID2 = 0x00;

    HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
  
     
    
    fdcan_filter.IdType = FDCAN_STANDARD_ID;


    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    fdcan_filter.FilterID1 = 0x00;
    fdcan_filter.FilterID2 = 0x00;

    HAL_FDCAN_ConfigFilter(&hfdcan2,&fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_ACCEPT_IN_RX_FIFO1,FDCAN_ACCEPT_IN_RX_FIFO1,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO1, 1);
    
    
    fdcan_filter.IdType = FDCAN_STANDARD_ID;


    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x00;
    fdcan_filter.FilterID2 = 0x00;

    HAL_FDCAN_ConfigFilter(&hfdcan3,&fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);


}

/**
 * @brief CAN的初始化
 * @param hfdcan  can编号
 * @param Callback_Function 回调函数传参
 * @return void
 */
void bsp_can_init(FDCAN_HandleTypeDef *hfdcan,CAN_Callback Callback_Function)
{
	if (hfdcan->Instance == FDCAN1) {
	    /* 必须无条件覆盖:RAM 未清零/被踩时字段可能是 0xFFFFFFFF,
	       它不是 nullptr,旧的 ==nullptr 判断会直接跳过,留下坏指针 */
	    CAN1_Manage_Object.Callback_Function = Callback_Function;
	}

    else if (hfdcan->Instance == FDCAN2) {
        CAN2_Manage_Object.Callback_Function = Callback_Function;
    }

    else if (hfdcan->Instance == FDCAN3) {
        CAN3_Manage_Object.Callback_Function = Callback_Function;
    }
    
    if (hfdcan->Instance == FDCAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hfdcan;
        if(system_can[0]==0){
            can_filter_init();        /* ensure filter is configured even if robot init skipped it */
			HAL_FDCAN_Start(hfdcan);   /* official order: Start first, then activate */
			HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
			system_can[0]=1;
        }

    }
    else if (hfdcan->Instance == FDCAN2)
    {
        CAN2_Manage_Object.CAN_Handler = hfdcan;
        if(system_can[1]==0){
			
			HAL_FDCAN_Start(hfdcan);
			HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
            system_can[1]=1;
        }
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        CAN3_Manage_Object.CAN_Handler = hfdcan;
        if(system_can[2]==0){

			HAL_FDCAN_Start(hfdcan);
			HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
            system_can[2]=1;
        }
    }
}

/* 标准帧发送头"模板":只读,不参与发送 —— 每次发送时拷一份到局部变量再改写
   Identifier/DataLength,所以多任务并发调用也不会互相踩(可重入)。 */
static const FDCAN_TxHeaderTypeDef fdcan_tx_stand_header={
    .IdType=FDCAN_STANDARD_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN,
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    .MessageMarker = 0,
};

/**
 * @brief CAN 标准帧发送(不等空位,失败即丢帧)
 * @param hfdcan can 编号
 * @param id     CAN id(标准帧,11 位)
 * @param data   发送数据数组指针
 * @param len    数据长度(字节数;经典帧 ≤ 8)
 * @return 0 = 已进 TX FIFO;1 = 入队失败(TX FIFO 满,或该总线还没 Start)
 */
uint8_t fdcan_send_data_stand(FDCAN_HandleTypeDef *hfdcan,uint32_t id, uint8_t *data,uint32_t len)
{
    FDCAN_TxHeaderTypeDef tx = fdcan_tx_stand_header;   //拷模板 → 可重入
    tx.Identifier = id;
    tx.DataLength = len;

    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, data)!=HAL_OK)
        return 1;
    return 0;
}

/* 扩展帧发送头"模板":同样只读,发送时拷一份到局部变量(可重入)。
   这样 fdcan_send_data_Exten() 与 fdcan_send_data_Exten_wait() 也不会互相踩。 */
static const FDCAN_TxHeaderTypeDef fdcan_tx_Exten_header={
    .IdType=FDCAN_EXTENDED_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN,
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    .MessageMarker = 0,
};

/**
 * @brief CAN 扩展帧发送(不等空位,失败即丢帧)
 * @param hfdcan can 编号
 * @param id     CAN id(扩展帧,29 位)
 * @param data   发送数据数组指针
 * @param len    数据长度(字节数;经典帧 ≤ 8)
 * @return 0 = 已进 TX FIFO;1 = 入队失败(TX FIFO 满,或该总线还没 Start)
 */
uint8_t fdcan_send_data_Exten(FDCAN_HandleTypeDef *hfdcan,uint32_t id, uint8_t *data,uint32_t len)
{
    FDCAN_TxHeaderTypeDef tx = fdcan_tx_Exten_header;   //拷模板 → 可重入
    tx.Identifier = id;
    tx.DataLength = len;

    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, data)!=HAL_OK)
        return 1;
	return 0;
}

/**
 * @brief CAN的扩展帧发送(等 TX FIFO 空位,带超时)
 * @param hfdcan  can编号
 * @param id      CAN_id(扩展帧)
 * @param *data   发送数据数组指针
 * @param len     数组长度
 * @param timeout_ms 等 TX FIFO 空位的超时 [ms]
 * @return 0 = 已进 TX FIFO;1 = 超时/参数错(这一帧没发出去)
 */
uint8_t fdcan_send_data_Exten_wait(FDCAN_HandleTypeDef *hfdcan,uint32_t id, uint8_t *data,uint32_t len,uint32_t timeout_ms)
{
    if (hfdcan == nullptr || data == nullptr) return 1;

    FDCAN_TxHeaderTypeDef tx = fdcan_tx_Exten_header;   //拷模板 → 可重入
    tx.Identifier = id;
    tx.DataLength = len;

    //TX FIFO 只有 8 槽(TxFifoQueueElmtsNbr),满了等空位;直接查 TFQF 位,
    //不调 HAL 接口是为了避免每次失败都往 ErrorCode 里塞 FIFO_FULL
    //⚠️ 自旋依赖 HAL_GetTick()(SysTick):若在中断里、或把中断屏蔽到 SysTick
    //   优先级之上再调用它,超时永远不会到 ⇒ 死循环。等待期间也不让出 CPU,
    //   所以只在任务上下文里用(灵足的 zero/参数操作就是任务里调的)。
    uint32_t t0 = HAL_GetTick();
    while ((hfdcan->Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0u) {
        if ((HAL_GetTick() - t0) > timeout_ms) return 1;
    }

    return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, data) != HAL_OK) ? 1u : 0u;
}

/**
 * @brief CAN的接收回调函数
 * @param hfdcan  can编号
 * @param *data 接收数据数组指针
 * @return void
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

    if (hfdcan->Instance == FDCAN1)
    {
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN1_Manage_Object.Rx_Header, CAN1_Manage_Object.Rx_Buffer) == HAL_OK)
        {

            if (can_callback_is_valid(CAN1_Manage_Object.Callback_Function))
            {
                CAN1_Manage_Object.Callback_Function(CAN1_Manage_Object.Rx_Header, CAN1_Manage_Object.Rx_Buffer);
            }
        }
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN3_Manage_Object.Rx_Header, CAN3_Manage_Object.Rx_Buffer) == HAL_OK)
        {
            

            if (can_callback_is_valid(CAN3_Manage_Object.Callback_Function))
            {
                CAN3_Manage_Object.Callback_Function(CAN3_Manage_Object.Rx_Header, CAN3_Manage_Object.Rx_Buffer);
            }
        }
    }

}


void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{

	if (hfdcan->Instance == FDCAN2)
    {
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &CAN2_Manage_Object.Rx_Header, CAN2_Manage_Object.Rx_Buffer) == HAL_OK)
        {            

            if (can_callback_is_valid(CAN2_Manage_Object.Callback_Function))
            {
                CAN2_Manage_Object.Callback_Function(CAN2_Manage_Object.Rx_Header, CAN2_Manage_Object.Rx_Buffer);
            }
        }
    }


}


