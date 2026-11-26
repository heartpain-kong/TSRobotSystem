/* ============================================================================
 * dev_ws2812.h — 板载 WS2812(可编程 RGB 灯)驱动,走 SPI6 单线发送
 * 说明 / 用法 / 坑 见同目录 dev_ws2812.md
 * ========================================================================== */

#ifndef DEV_WS2812_H
#define DEV_WS2812_H

#include "bsp_spi.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 灯珠数量(按实际硬件改) */
#ifndef WS2812_LED_NUM
#define WS2812_LED_NUM          8u
#endif

/* 当前板时钟:SPI6 = PLL2Q 50MHz / 8 = 6.25MHz。
   6.25MHz 下 1 个 SPI 字节 = 1.28µs ≈ WS2812 的 1.25µs 位周期,
   所以每个 WS2812 位用 1 个字节编码(8 个 SPI 位),每颗灯 24 字节。 */
#define WS2812_SPI_KERNEL_HZ    50000000u
#define WS2812_SPI_CLOCK_HZ     6250000u
#define WS2812_BYTES_PER_LED    24u

/* 尾部 reset:6.25MHz 下 1 字节 1.28µs,48 字节 ≈61µs(>50µs) */
#define WS2812_RESET_BYTES      48u
#define WS2812_BUFFER_SIZE      (WS2812_LED_NUM * WS2812_BYTES_PER_LED + WS2812_RESET_BYTES)

/**
 * @brief 初始化:把 SPI6 设成 6.25MHz 并把灯全灭
 * @param led_num 实际灯珠数量(必须 <= WS2812_LED_NUM)
 * @return true = 成功
 * @note 前置条件:MX_SPI6_Init() 已跑过
 */
bool WS2812_Init(uint32_t led_num);

/**
 * @brief SPI6 的时钟是否满足当前 8 位编码要求(≈6.25Mbit/s)
 * @return true = 对(当前配置 SPI6 = 50MHz / 8)
 * @note false 说明 CubeMX 里没改分频 —— 灯会显示乱码,但不会死机
 */
bool WS2812_Is_Clock_Ok(void);

/**
 * @brief 设置某颗灯的颜色(先改缓冲,再调 WS2812_Show 才会亮)
 * @param index 灯序号(0 起)
 * @param r/g/b 0~255
 * @return true = 成功
 */
bool WS2812_Set_RGB(uint32_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 设置所有灯为同一颜色
 * @return void
 */
void WS2812_Set_All(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 全部熄灭(只改缓冲,仍需 Show)
 * @return void
 */
void WS2812_Clear(void);

/**
 * @brief 把缓冲发给灯带(非阻塞:**DMA 发送**,发完自动发 reset 空白)
 * @return true = 已发起;false = 正忙/未初始化
 */
bool WS2812_Show(void);

/**
 * @brief 是否正在发送
 * @return true = 忙
 */
bool WS2812_Is_Busy(void);

/**
 * @brief ★ 跑一拍:发送超时保护(挂在周期任务里;正常情况靠中断收尾)
 * @return void
 */
void WS2812_Task(void);

/**
 * @brief SPI6 发送完成回调(WS2812_Init 内部会自动注册给 bsp_spi;中断上下文,只清标志)
 * @return void
 */
void WS2812_Tx_Complete_Callback(void);

/**
 * @brief 取原始缓冲(要给灯带发特殊数据时才用)
 * @return 缓冲首地址
 */
uint8_t *WS2812_Get_Buffer(void);

#ifdef __cplusplus
}
#endif

#endif // DEV_WS2812_H
