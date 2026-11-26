/* ============================================================================
 * dev_w25q64.h — W25Q64JV (8MB SPI NOR Flash, 挂在 OCTOSPI2 上) 器件驱动
 * 说明 / 用法 / 坑 见同目录 dev_w25q64.md
 * ========================================================================== */

#ifndef DEV_W25Q64_H
#define DEV_W25Q64_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* OSPI 收发缓冲长度(1 页 = 256B,取 512 留余量) */
#define W25Q64_BUFFER_SIZE      512u

/* 一页(写入不能跨页) */
#define W25Q64_PAGE_SIZE        256u
/* 一个扇区(最小擦除单位) */
#define W25Q64_SECTOR_SIZE      4096u
/* 64KB 块 */
#define W25Q64_BLOCK64K_SIZE    65536u
#define W25Q64_BLOCK32K_SIZE    32768u
/* 整片 8MB */
#define W25Q64_CHIP_SIZE        (8u * 1024u * 1024u)

/* JEDEC ID:EF 40 17(Winbond / W25Q64) */
#define W25Q64_JEDEC_ID         0x00EF4017u   /* ★ 大端顺序:EF 40 17(以前写成 0x1740EF,判失败 ✗) */

/* 工作模式 */
typedef enum
{
    W25Q64_MODE_NORMAL = 0,         /* 间接模式:每次读都要发命令(HAL_OSPI_Receive_DMA) */
    W25Q64_MODE_MEMORY_MAPPED,      /* 内存映射:之后可直接 *(uint8_t*)(0x70000000+addr) 读 */
} W25Q64_Mode;

/**
 * @brief 初始化并**自检**:读 JEDEC ID,确认芯片在、CPU 能收到数据
 * @param mode 工作模式(建议先用 W25Q64_MODE_NORMAL 打通)
 * @return true = ID 正确(EF 40 17);false = 没读到/读错(检查 OSPI 配置、焊接、SCK)
 * @note 必须已经:① MX_OCTOSPI2_Init() 跑过 ② BSP_OSPI_Init(&hospi2, ...) 注册过回调
 * @note ⚠️ 用 W25Q64_MODE_MEMORY_MAPPED 之前,要给 0x70000000 加一条**非缓存** MPU 区,
 *       否则 D-Cache 会让你读到旧数据(详见 dev_w25q64.md)
 */
bool W25Q64_Init(W25Q64_Mode mode);

/**
 * @brief 发起一次读(非阻塞)
 * @param addr 起始地址(0 ~ 8MB-1)
 * @param len  字节数(<= W25Q64_BUFFER_SIZE)
 * @return true = 已发起(读完后 W25Q64_Is_Rx_Done() 变 true,数据在 W25Q64_Get_Buffer())
 */
bool W25Q64_Read(uint32_t addr, uint16_t len);

/**
 * @brief 发起一次页写(非阻塞)。内部自动:写使能 → 四线页编程 → 等 WIP
 * @param buf  数据源(RAM_D1 里,非缓存 ⇒ DMA 安全)
 * @param addr 起始地址(0 ~ 8MB-1)
 * @param len  字节数(**不能跨 256B 页**,即 addr%256 + len <= 256)
 * @return true = 已发起;false = 参数非法/正忙
 */
bool W25Q64_Write_Page(const uint8_t *buf, uint32_t addr, uint16_t len);

/**
 * @brief 擦除一个 4KB 扇区(非阻塞)
 * @param addr 必须是 4KB 对齐
 * @return true = 已发起
 */
bool W25Q64_Erase_Sector(uint32_t addr);

/**
 * @brief 擦除一个 32KB 块(非阻塞;命令 0x52,和厂家例程一致)
 * @param addr 必须是 32KB 对齐
 * @return true = 已发起
 */
bool W25Q64_Erase_Block_32K(uint32_t addr);

/**
 * @brief 擦除一个 64KB 块(非阻塞)
 * @param addr 必须是 64KB 对齐
 * @return true = 已发起
 */
bool W25Q64_Erase_Block_64K(uint32_t addr);

/**
 * @brief 整片擦除(非阻塞,可能几十秒 —— 超时上限按 W25Q64_TIMEOUT_CHIP_ERASE_MS)
 * @return true = 已发起
 */
bool W25Q64_Erase_Chip(void);

/**
 * @brief ★ 跑一拍:推进状态机(等 WIP/收完)、检查超时
 * @note 挂在周期任务里调用(1kHz 或 500Hz 都行);**不能**在中断里调
 */
void W25Q64_Task(void);

/**
 * @brief 是否正在忙(DMA 传输中 / 擦写中等 WIP)
 * @return true = 忙
 */
bool W25Q64_Is_Busy(void);

/**
 * @brief 最近一次读是否已完成(数据在 W25Q64_Get_Buffer())
 * @return true = 完成
 */
bool W25Q64_Is_Rx_Done(void);

/**
 * @brief 是否出过错(超时等)
 * @return true = 有错误未清
 */
bool W25Q64_Has_Error(void);

/**
 * @brief 出错次数(调试用:非 0 说明有过超时)
 * @return 累计错误计数
 */
uint32_t W25Q64_Get_Error_Count(void);

/**
 * @brief 清错误标志
 * @return void
 */
void W25Q64_Clear_Error(void);

/**
 * @brief 取接收缓冲(读操作完成后从这里取数据)
 * @return 缓冲首地址;未初始化返回 NULL
 */
const uint8_t *W25Q64_Get_Buffer(void);

/**
 * @brief 取初始化时读到的 JEDEC ID(W25Q64 = 0xEF4017)
 * @return ID;未初始化成功过返回 0
 */
uint32_t W25Q64_Get_Jedec_Id(void);

/**
 * @brief 把本驱动的三个回调(Rx/Tx/轮询)注册给 OSPI 总线驱动
 * @return void
 * @note 初始化时调一次,顺序:MX_OCTOSPI2_Init() → W25Q64_Register_Callbacks() → W25Q64_Init()
 */
void W25Q64_Register_Callbacks(void);

#ifdef __cplusplus
}
#endif

#endif // DEV_W25Q64_H
