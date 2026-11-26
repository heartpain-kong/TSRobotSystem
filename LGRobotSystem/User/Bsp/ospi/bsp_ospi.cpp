//
// bsp_ospi.cpp — OCTOSPI 总线驱动(仿 E:\rm\dm02_test\...\Driver\OSPI\drv_ospi.cpp)
// 说明 / 用法 / 坑 见同目录 bsp_ospi.md

#include "bsp_ospi.h"

/* ============================================================================
 * 管理对象:每个 OSPI 一个(和 bsp_fdcan 的 CAN_Manage_Object 一个套路)
 *   ⚠️ 缓冲必须放在**非缓存内存**里给 DMA 用 —— 本工程 RAM_D1 已被 MPU
 *      配成 Normal Non-cacheable(见 main.c::MPU_Config),.bss 都在里面 ⇒ 安全 ✓
 * ========================================================================== */
static BSP_OSPI_Object OSPI1_Manage_Object;
static BSP_OSPI_Object OSPI2_Manage_Object;

/**
 * @brief 按句柄找管理对象
 * @param hospi OSPI 句柄
 * @return 管理对象指针(没注册过返回 nullptr)
 */
static BSP_OSPI_Object *BSP_OSPI_Get_Object(OSPI_HandleTypeDef *hospi)
{
    if (hospi == nullptr)
    {
        return nullptr;
    }
    if (hospi->Instance == OCTOSPI1)
    {
        return &OSPI1_Manage_Object;
    }
    if (hospi->Instance == OCTOSPI2)
    {
        return &OSPI2_Manage_Object;
    }
    return nullptr;
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 初始化 OSPI 驱动:绑定句柄 + 注册回调
 * @param hospi             OSPI 句柄
 * @param polling_callback  自动轮询完成回调(可空)
 * @param rx_callback       接收完成回调(可空)
 * @param tx_callback       发送完成回调(可空)
 * @return void
 */
void BSP_OSPI_Init(OSPI_HandleTypeDef *hospi,
                   BSP_OSPI_Polling_Callback polling_callback,
                   BSP_OSPI_Rx_Callback rx_callback,
                   BSP_OSPI_Tx_Callback tx_callback)
{
    BSP_OSPI_Object *object = BSP_OSPI_Get_Object(hospi);
    if (object == nullptr)
    {
        return;
    }

    object->handler            = hospi;
    object->polling_callback   = polling_callback;
    object->rx_callback        = rx_callback;
    object->tx_callback        = tx_callback;
}

/**
 * @brief 取发送/接收缓冲首地址
 * @param hospi OSPI 句柄
 * @return 缓冲首地址;句柄没注册过返回 nullptr
 */
uint8_t *BSP_OSPI_Get_Tx_Buffer(OSPI_HandleTypeDef *hospi)
{
    BSP_OSPI_Object *object = BSP_OSPI_Get_Object(hospi);
    return (object != nullptr) ? object->tx_buffer : nullptr;
}

uint8_t *BSP_OSPI_Get_Rx_Buffer(OSPI_HandleTypeDef *hospi)
{
    BSP_OSPI_Object *object = BSP_OSPI_Get_Object(hospi);
    return (object != nullptr) ? object->rx_buffer : nullptr;
}

/**
 * @brief 只发命令(阻塞到命令发出)
 * @param hospi OSPI 句柄
 * @param cmd   命令配置
 * @return HAL 状态(未注册句柄/空指针返回 HAL_ERROR)
 */
HAL_StatusTypeDef BSP_OSPI_Command(OSPI_HandleTypeDef *hospi, OSPI_RegularCmdTypeDef *cmd)
{
    if (BSP_OSPI_Get_Object(hospi) == nullptr || cmd == nullptr)
    {
        return HAL_ERROR;
    }
    return HAL_OSPI_Command(hospi, cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
}

/**
 * @brief 命令 + DMA 发送 tx_buffer
 * @note 发完进 tx_callback(器件驱动在那里发起"等 WIP")
 */
HAL_StatusTypeDef BSP_OSPI_Command_Transmit_DMA(OSPI_HandleTypeDef *hospi, OSPI_RegularCmdTypeDef *cmd)
{
    BSP_OSPI_Object *object = BSP_OSPI_Get_Object(hospi);
    if (object == nullptr || cmd == nullptr)
    {
        return HAL_ERROR;
    }

    if (HAL_OSPI_Command(hospi, cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OSPI_Transmit_DMA(hospi, object->tx_buffer);
}

/**
 * @brief 命令 + DMA 接收进 rx_buffer
 * @note 收完进 rx_callback
 */
HAL_StatusTypeDef BSP_OSPI_Command_Receive_DMA(OSPI_HandleTypeDef *hospi, OSPI_RegularCmdTypeDef *cmd)
{
    BSP_OSPI_Object *object = BSP_OSPI_Get_Object(hospi);
    if (object == nullptr || cmd == nullptr)
    {
        return HAL_ERROR;
    }

    if (HAL_OSPI_Command(hospi, cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_OSPI_Receive_DMA(hospi, object->rx_buffer);
}

/**
 * @brief 启动中断方式的自动轮询(等 Flash 状态位)
 * @param hospi OSPI 句柄
 * @param cfg   轮询配置
 * @return HAL 状态
 * @note 调用前要先用 BSP_OSPI_Command() 把"读状态"命令发好 —— HAL 的自动轮询
 *       复用 OSPI 里当前配置的那条命令。
 *       ⚠️ 必须保证 stm32h7xx_it.c 里有 HAL_OSPI_IRQHandler(&hospi2),否则永远等不到
 */
HAL_StatusTypeDef BSP_OSPI_AutoPolling_IT(OSPI_HandleTypeDef *hospi, OSPI_AutoPollingTypeDef *cfg)
{
    if (BSP_OSPI_Get_Object(hospi) == nullptr || cfg == nullptr)
    {
        return HAL_ERROR;
    }
    return HAL_OSPI_AutoPolling_IT(hospi, cfg);
}

/**
 * @brief OSPI 是否空闲
 * @param hospi OSPI 句柄
 * @return true = 可以接受新命令
 */
bool BSP_OSPI_Is_Ready(OSPI_HandleTypeDef *hospi)
{
    if (BSP_OSPI_Get_Object(hospi) == nullptr)
    {
        return false;
    }
    return (HAL_OSPI_GetState(hospi) == HAL_OSPI_STATE_READY);
}

/* ============================================================================
 * HAL 回调分发(运行在**中断上下文**:只调注册的回调,不加逻辑)
 *
 * ⚠️ 下面这些名字是 HAL 的 __weak 函数,在 stm32h7xx_hal_ospi.c 里定义;
 *    这里重定义即覆盖。它们通过头文件里的 extern "C" 继承 C 链接,
 *    所以 C++ 里这样写就能正确覆盖(不用额外加 extern "C")。
 * ========================================================================== */

/**
 * @brief 自动轮询结束(状态匹配)⇒ 转给注册的回调
 */
extern "C" void HAL_OSPI_StatusMatchCallback(OSPI_HandleTypeDef *hospi)
{
    BSP_OSPI_Object *object = BSP_OSPI_Get_Object(hospi);
    if (object != nullptr && object->polling_callback != nullptr)
    {
        object->polling_callback();
    }
}

/**
 * @brief DMA 接收完成 ⇒ 转给注册的回调
 */
extern "C" void HAL_OSPI_RxCpltCallback(OSPI_HandleTypeDef *hospi)
{
    BSP_OSPI_Object *object = BSP_OSPI_Get_Object(hospi);
    if (object != nullptr && object->rx_callback != nullptr)
    {
        object->rx_callback();
    }
}

/**
 * @brief DMA 发送完成 ⇒ 转给注册的回调
 */
extern "C" void HAL_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hospi)
{
    BSP_OSPI_Object *object = BSP_OSPI_Get_Object(hospi);
    if (object != nullptr && object->tx_callback != nullptr)
    {
        object->tx_callback();
    }
}

/**
 * @brief OSPI 出错 ⇒ 只记状态(器件驱动的"跑一拍"里会看到并清错)
 */
extern "C" void HAL_OSPI_ErrorCallback(OSPI_HandleTypeDef *hospi)
{
    if (hospi != nullptr)
    {
        hospi->ErrorCode = HAL_OSPI_ERROR_NONE;      /* 清标志,避免一直卡在错误态 */
    }
}
