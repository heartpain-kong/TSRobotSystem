/* ============================================================================
 * bsp_flash.h — 外部 8 MB QSPI Flash(W25Q64)存储层
 * 说明 / 用法 / 坑 见同目录 bsp_flash.md
 * ========================================================================== */

#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 分区约定(按需修改;单位:字节偏移) ---- */
/* ★ 外部 Flash 总开关:0 = 不用(硬件不通时用,避免错误码 0x0005 一直报、LED 常红)✓
   实测本板:OSPI 状态"忙(间接读)"+错误码 INVALID_SEQUENCE ⇒ 芯片不回数据 ⇒ 硬件问题 ✓ */
#define FLASH_ENABLE            1

/* ★ 自测:与 FLASH_ENABLE 无关(日志仍关闭),只跑 读ID→擦→写→读回 四步,
   每步结果进遥测的"Flash 诊断" ⇒ 不会再让板子卡死 ✓ */
#define FLASH_SELFTEST          0

#define FLASH_PARAM_ADDR        0x000000u
#define FLASH_PARAM_SIZE        0x001000u        /* 4 KB = 1 个扇区 */

#define FLASH_LOG_ADDR          0x001000u
#define FLASH_LOG_SIZE          0x2FF000u        /* 3 MB - 4KB */        /* ~1 MB */

#define FLASH_IMAGE_ADDR        0x300000u
#define FLASH_IMAGE_SIZE        0x300000u        /* 3 MB */        /* 7 MB */

/* 一颗芯片的容量与 ID */
#define FLASH_CAPACITY          (8u * 1024u * 1024u)
#define FLASH_JEDEC_ID_W25Q64   0xEF4017u

/**
 * @brief 初始化外部 Flash:注册 OSPI 回调 + W25Q64_Init(读 JEDEC ID 自检)
 * @return true = 芯片在线且 ID 正确(EF 40 17)
 * @note 依赖:MX_OCTOSPI2_Init() 已跑过(main() 里),且 Timestamp_Init() 已调过
 */
/* Flash 诊断全局量(实现在 bsp_flash.cpp;遥测里看,调试时也能直接读) */
extern volatile uint32_t flash_diag_state;
extern volatile uint32_t flash_diag_err;
extern volatile uint32_t flash_diag_cmd;
extern volatile uint32_t flash_diag_rcv;
extern volatile uint32_t flash_diag_raw;

bool Flash_Init(void);

/**
 * @brief 四步自测(读ID/擦/写/读回),结果写进 flash_diag_*
 * @return 走到的步数(4 = 全部通过)
 */
uint32_t Flash_Self_Test(void);

/**
 * @brief 是否已就绪(初始化成功过)
 * @return true = 可以读写
 */
bool Flash_Is_Ready(void);

/**
 * @brief 读回 JEDEC ID(W25Q64 应为 0xEF4017)
 * @return ID;未初始化返回 0
 */
uint32_t Flash_Get_Jedec_Id(void);

/**
 * @brief 芯片容量(字节,8 MB)
 * @return 容量
 */
uint32_t Flash_Get_Capacity(void);

/**
 * @brief 读数据(自动按驱动缓冲上限分段;**阻塞**)
 * @param addr 起始地址(0 ~ 8MB-1)
 * @param buf  输出缓冲
 * @param len  字节数
 * @return true = 成功
 */
bool Flash_Read(uint32_t addr, void *buf, uint32_t len);

/**
 * @brief 写数据(自动按 256B 页分段;**阻塞**)
 * @param addr 起始地址
 * @param buf  数据源
 * @param len  字节数
 * @return true = 成功
 * @note 目标区域必须**已擦除** —— 想省一次擦除可以先用 Flash_Is_Blank() 查
 */
bool Flash_Write(uint32_t addr, const void *buf, uint32_t len);

/**
 * @brief 擦除一段(自动按 4KB 扇区;**阻塞**,每扇区约 45ms)
 * @param addr 起始地址(会被向下对齐到扇区)
 * @param len  字节数
 * @return true = 成功
 */
bool Flash_Erase(uint32_t addr, uint32_t len);

/**
 * @brief 判断某段是否空白(全 0xFF,即已擦除)
 * @param addr 起始地址
 * @param len  字节数
 * @return true = 可以直接写,不必先擦
 */
bool Flash_Is_Blank(uint32_t addr, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif // BSP_FLASH_H
