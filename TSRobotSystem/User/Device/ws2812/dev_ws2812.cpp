//
// dev_ws2812.cpp — 板载 WS2812 驱动(SPI6 单线发送,DMA 方式;缓冲在 D3 SRAM)
// 说明 / 用法 / 坑 见同目录 dev_ws2812.md

#include "dev_ws2812.h"

#include "bsp_timestamp.h"

#include <string.h>

/* ============================================================================
 * 灯珠数据顺序:WS2812/WS2812B 标准是 GRB;少数兼容芯片(如 SK6812-RGBW 的 RGB 变种)是 RGB。
 * 症状:改这里之前,若"红色显示成绿色、绿色显示成红色"⇒ 就是顺序反了,把下面改成 0 ✓
 * ========================================================================== */
#ifndef WS2812_ORDER_GRB
#define WS2812_ORDER_GRB   1       /* 1 = GRB(标准);0 = RGB */
#endif

extern "C" {
extern SPI_HandleTypeDef hspi6;         /* Core/Src/spi.c */
}

/* 前置声明:发送完成回调(定义在文件末尾) */
extern "C" void WS2812_Tx_Complete_Callback(void);

/* 状态 */
/* ★ 必须放在 D3 SRAM(见 .ld 的 .RAM_D3 段):
   SPI6 的 DMA 请求走 BDMA,而 BDMA **只能访问 D3 SRAM** —— 放 RAM_D1 它读不到,
   发出去就是空数据(灯不亮)。aligned(32) 是为了清 D-Cache 时按行对齐更省事。 */
static uint8_t ws2812_buffer[WS2812_BUFFER_SIZE] __attribute__((section(".RAM_D3"), aligned(32)));
static uint32_t led_count = 0u;

/* ★ 结构性防护:led_count 一旦被踩坏(内存破坏),下面所有"算偏移/发 DMA"的地方
   都会用 clamp 后的值 ⇒ 最坏退化成"只用 8 颗",绝不会变成往 67 MB 外写 ✗
   (这条是照着一次真实 HardFault 加的:BFAR = 0x3BFE8B72 = 基址 + led_count*9 ✗) */
static uint32_t Led_Count_Safe(void)
{
    return (led_count > WS2812_LED_NUM) ? WS2812_LED_NUM : led_count;
}
static volatile bool busy = false;
static uint32_t start_us = 0u;
static bool inited = false;
static bool clock_ok = false;     /* SPI6 时钟是否满足 2.5Mbit/s(见 WS2812_Is_Clock_Ok) */

/* 发送超时(ms):8 颗灯 = 72 字节 + reset,2.5MHz 下 ~300µs,给 20ms 足够 */
#define WS2812_TIMEOUT_MS   20u

/**
 * @brief 把一个颜色字节编码成 3 个 SPI 字节
 * @param value 颜色字节
 * @param out   输出(3 字节)
 * @return void
 */
static void WS2812_Encode_Byte(uint8_t value, uint8_t *out)
{
    /* 6.25MHz:1 个 SPI 字节 = 1.28µs ≈ 1 个 WS2812 位。
       0 → 1110_0000 (高 0.48µs / 低 0.80µs)
       1 → 1111_1000 (高 0.80µs / 低 0.48µs) */
    for (uint32_t i = 0u; i < 8u; i++)
    {
        out[i] = ((value >> (7u - i)) & 0x01u) ? 0xF8u : 0xE0u;
    }
}

/**
 * @brief 初始化:缓冲清零并发送一次(灯全灭)
 * @param led_num 灯珠数量
 * @return true = 成功
 * @note ★ SPI6 的时钟由 **CubeMX** 决定,本驱动不硬改它 —— 只把它**读回来校验**:
 *       · 需要 SPI6 kernel 80MHz + Prescaler 32 ⇒ 2.5Mbit/s(3 个 SPI 位 ≈ 1.2µs);
 *       · 校验结果见 WS2812_Is_Clock_Ok(),不对就说明 CubeMX 里没改成 32。
 */
bool WS2812_Init(uint32_t led_num)
{
    static const uint32_t prescaler_table[8] = {2u, 4u, 8u, 16u, 32u, 64u, 128u, 256u};
    uint32_t prescaler;
    uint32_t spi_clock_hz;

    if (led_num == 0u || led_num > WS2812_LED_NUM)
    {
        return false;
    }
    /* 自己把发送完成回调注册给 bsp_spi(用户不需要再调 BSP_SPI_Init) */
    BSP_SPI_Init(&hspi6, WS2812_Tx_Complete_Callback);
    if (BSP_SPI_Get_Tx_Buffer(&hspi6) == nullptr)
    {
        return false;
    }

    /* 读回实际分频,算出真实 SPI 时钟(当前 SPI6 内核 = PLL2Q = 50MHz) */
    prescaler = ((uint32_t)hspi6.Init.BaudRatePrescaler >> SPI_CFG1_MBR_Pos) & 0x07u;
    spi_clock_hz = WS2812_SPI_KERNEL_HZ / prescaler_table[prescaler & 0x07u];
    clock_ok = (spi_clock_hz <= (WS2812_SPI_CLOCK_HZ + 200000u)) &&
               (spi_clock_hz >= (WS2812_SPI_CLOCK_HZ - 200000u));

    led_count = led_num;
    memset(ws2812_buffer, 0, sizeof(ws2812_buffer));
    inited = true;

    /* 亮一次"全灭",让灯带上电后是干净的 */
    (void)WS2812_Show();
    return true;
}

/**
 * @brief SPI6 时钟是否满足 WS2812 编码要求(≈2.5Mbit/s)
 * @return true = 对(CubeMX 里 Prescaler 已设 32)
 */
bool WS2812_Is_Clock_Ok(void)
{
    return clock_ok;
}

/**
 * @brief 设置某颗灯的颜色(缓冲里按 GRB 排)
 * @return true = 成功
 */
bool WS2812_Set_RGB(uint32_t index, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t *p;

    if (!inited || index >= Led_Count_Safe())
    {
        return false;
    }

    /* ★ 下标保护:越界会写穿 .RAM_D3 缓冲(BDMA 读它)⇒ 必须挡住 */
    if ((index >= Led_Count_Safe()) || (index >= WS2812_LED_NUM))
    {
        return false;
    }

    p = &ws2812_buffer[index * WS2812_BYTES_PER_LED];
#if WS2812_ORDER_GRB
    WS2812_Encode_Byte(g, &p[0]);       /* G 在前(标准 WS2812B) */
    WS2812_Encode_Byte(r, &p[8]);
    WS2812_Encode_Byte(b, &p[16]);
#else
    WS2812_Encode_Byte(r, &p[0]);       /* RGB 顺序(兼容芯片) */
    WS2812_Encode_Byte(g, &p[8]);
    WS2812_Encode_Byte(b, &p[16]);
#endif
    return true;
}

/**
 * @brief 所有灯设成同一颜色
 * @return void
 */
void WS2812_Set_All(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t i;

    for (i = 0u; i < Led_Count_Safe(); i++)
    {
        (void)WS2812_Set_RGB(i, r, g, b);
    }
}

/**
 * @brief 全部熄灭(只改缓冲)
 * @return void
 */
void WS2812_Clear(void)
{
    WS2812_Set_All(0u, 0u, 0u);
}

/**
 * @brief 发送缓冲(中断方式,非阻塞)
 * @return true = 已发起
 */
bool WS2812_Show(void)
{
    uint32_t total;

    if (!inited || busy)
    {
        return false;
    }

    total = Led_Count_Safe() * WS2812_BYTES_PER_LED + WS2812_RESET_BYTES;

    /* 尾部 reset 空白置 0(每次发都要保证) */
    memset(&ws2812_buffer[Led_Count_Safe() * WS2812_BYTES_PER_LED], 0, WS2812_RESET_BYTES);

    busy = true;
    start_us = Timestamp_Get_Us32();

    /* ★ D3 SRAM 默认是可缓存区:先把刚写进缓冲的颜色刷到物理内存,否则 BDMA 可能读到旧值 */
    SCB_CleanDCache_by_Addr((uint32_t *)ws2812_buffer, (int32_t)total);

    if (HAL_SPI_Transmit_DMA(&hspi6, ws2812_buffer, (uint16_t)total) != HAL_OK)
    {
        busy = false;
        return false;
    }
    return true;
}

/**
 * @brief 是否正在发送
 * @return true = 忙
 */
bool WS2812_Is_Busy(void)
{
    return busy;
}

/**
 * @brief 跑一拍:发送超时保护
 * @return void
 */
void WS2812_Task(void)
{
    if (busy && (Timestamp_Get_Us32() - start_us) > (WS2812_TIMEOUT_MS * 1000u))
    {
        /* SPI 卡住了(极少见):把状态掰回来并中止传输,免得一直显示旧数据 */
        (void)HAL_SPI_Abort(&hspi6);
        busy = false;
    }
}

/**
 * @brief 取原始缓冲
 * @return 缓冲首地址
 */
uint8_t *WS2812_Get_Buffer(void)
{
    return ws2812_buffer;
}

/* ============================================================================
 * 发送完成 ⇒ 清忙标志(由 bsp_spi.cpp 的 HAL_SPI_TxCpltCallback 转发过来)
 *
 * 接线上要写:  BSP_SPI_Init(&hspi6, WS2812_Tx_Complete_Callback);
 * WS2812 用的是 SPI6 且不需要片选,所以 bsp_spi 里那段"拉回 CS"的逻辑不会动到任何脚
 * (cs_gpiox 为 nullptr 时它自己会跳过)。
 * ========================================================================== */

/**
 * @brief SPI6 发送完成回调(注册给 bsp_spi);中断上下文,只清标志
 * @return void
 */
extern "C" void WS2812_Tx_Complete_Callback(void)
{
    busy = false;
}
