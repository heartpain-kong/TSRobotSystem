//
// bsp_flash.cpp — 外部 8 MB QSPI Flash(W25Q64)存储层
// 说明 / 用法 / 坑 见同目录 bsp_flash.md

#include "bsp_flash.h"
#include "octospi.h"
#include "bsp_ospi.h"          /* hospi2(Flash 诊断要用) */

#include "bsp_timestamp.h"
#include "dev_w25q64.h"

#include <string.h>

/* 每次操作的外层兜底超时(µs) —— 芯片驱动的内部超时更细,这里只是防"永远等下去" */
#define FLASH_WAIT_READ_US      50000u        /* 读一段:50ms 足够(512B @40MHz ≈ 0.1ms) */
#define FLASH_WAIT_WRITE_US     50000u        /* 一页写:典型 0.4ms,给 50ms */
#define FLASH_WAIT_ERASE_US     600000u       /* 一个 4KB 扇区:典型 45ms,给 600ms */

static bool s_ready = false;

/**
 * @brief 等驱动空闲(等待期间自己泵状态机)
 * @param timeout_us 外层兜底超时(µs)
 * @return true = 空闲且无错误
 */
static bool Flash_Wait_Idle(uint32_t timeout_us)
{
    uint32_t t0 = Timestamp_Get_Us32();

    while (W25Q64_Is_Busy())
    {
        W25Q64_Task();                                  /* ★ 关键:推进底层状态机 */
        if (Timestamp_Elapsed_Us32(t0) > timeout_us)
        {
            return false;
        }
    }
    return !W25Q64_Has_Error();
}

/**
 * @brief 等一次读完成(等待期间自己泵状态机)
 * @param timeout_us 外层兜底超时(µs)
 * @return true = 读到数据
 */
static bool Flash_Wait_Rx(uint32_t timeout_us)
{
    uint32_t t0 = Timestamp_Get_Us32();

    while (!W25Q64_Is_Rx_Done())
    {
        W25Q64_Task();
        if (Timestamp_Elapsed_Us32(t0) > timeout_us)
        {
            return false;
        }
    }
    return !W25Q64_Has_Error();
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 初始化:注册 OSPI 回调 + W25Q64_Init(读 JEDEC ID 自检)
 * @return true = 芯片在线且 ID 正确
 */
/* ★ Flash 诊断:失败时逐层记录,遥测里直接看"卡在哪一层" */
volatile uint32_t flash_diag_state = 0u;     /* hospi2.State     */
volatile uint32_t flash_diag_err   = 0u;     /* hospi2.ErrorCode */
volatile uint32_t flash_diag_cmd   = 0xFFFFFFFFu; /* HAL_OSPI_Command 返回 */
volatile uint32_t flash_diag_rcv   = 0xFFFFFFFFu; /* HAL_OSPI_Receive 返回 */
volatile uint32_t flash_diag_raw   = 0xFFFFFFFFu; /* 原始 4 字节 */

/* 绕过驱动,直接发 0x9F 读 JEDEC —— 只为诊断,不改驱动行为 */
static void Flash_Diag_Probe(void)
{
    OSPI_RegularCmdTypeDef cmd = {0};
    uint8_t buf[4] = { 0xFFu, 0xFFu, 0xFFu, 0xFFu };

    flash_diag_state = (uint32_t)hospi2.State;
    flash_diag_err   = (uint32_t)hospi2.ErrorCode;

    if (hospi2.State != HAL_OSPI_STATE_READY)
    {
        /* 外设不是空闲态 ⇒ 不再发命令(避免长时间等待)✗ */
        flash_diag_cmd = 0xFFFFFFFEu;
        flash_diag_rcv = 0xFFFFFFFEu;
        return;
    }

    cmd.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId            = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction        = 0x9Fu;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_NONE;
    cmd.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
    cmd.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
    cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode           = HAL_OSPI_DATA_1_LINE;
    cmd.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
    cmd.NbData             = 3u;
    cmd.DummyCycles        = 0u;
    cmd.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    flash_diag_cmd = (uint32_t)HAL_OSPI_Command(&hospi2, &cmd, 10u);   /* 单位 ms!别写 0xFFFF(会卡 65 秒)✗ */
    flash_diag_rcv = (uint32_t)HAL_OSPI_Receive(&hospi2, buf, 10u);
    flash_diag_raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    flash_diag_state = (uint32_t)hospi2.State;
    flash_diag_err   = (uint32_t)hospi2.ErrorCode;
}

/**
 * @brief 四步自测:读ID → 擦1扇区 → 写32字节 → 读回比对
 * @note  只用 LOG 区的最后一个扇区(0xFF000),日志关闭时完全空闲 ✓
 *        每步把结果写进 flash_diag_*,遥测里能看出卡在第几步 ✓
 */
uint32_t Flash_Self_Test(void)
{
    static const uint32_t SCRATCH_ADDR = 0x000FF000u;      /* LOG 区最后一个 4KB 扇区 */
    uint8_t pattern[32];
    uint8_t *rx;
    uint32_t i;

    flash_diag_state = 0u;
    flash_diag_err   = 0u;
    flash_diag_cmd   = 0xFFFFFFFFu;
    flash_diag_rcv   = 0xFFFFFFFFu;
    flash_diag_raw   = 0xFFFFFFFFu;

    /* 0) ★ 外设可能被别处留在 BUSY(实测 State=2 BUSY_INDIRECT_WRITE)
          ⇒ 先 Abort 把它拉回 READY,再做任何命令 ✓ */
    if (hospi2.State != HAL_OSPI_STATE_READY)
    {
        (void)HAL_OSPI_Abort(&hospi2);
    }

    /* 0.1) 把关键指针记进 RAM:崩了也能 halt 读出来 ✓ */
    flash_diag_raw = (uint32_t)(uintptr_t)BSP_OSPI_Get_Rx_Buffer(&hospi2);
    flash_diag_cmd = (uint32_t)(uintptr_t)BSP_OSPI_Get_Tx_Buffer(&hospi2);
    flash_diag_rcv = (uint32_t)hospi2.Instance;
    flash_diag_err = (uint32_t)hospi2.State;

    /* 1) 初始化 + 读 ID */
    flash_diag_state = 1u;
    W25Q64_Register_Callbacks();
    if (!W25Q64_Init(W25Q64_MODE_NORMAL))
    {
        flash_diag_err = 1u;                              /* 1 = ID 读失败 */
        return flash_diag_state;
    }
    flash_diag_state = 1u;
    flash_diag_raw   = W25Q64_Get_Jedec_Id();

    /* 2) 擦一个扇区 */
    flash_diag_state = 2u;
    if (!W25Q64_Erase_Sector(SCRATCH_ADDR))
    {
        flash_diag_err = 2u;                              /* 2 = 擦失败 */
        return flash_diag_state;
    }
    flash_diag_cmd = 0u;                                  /* 擦 OK */
    flash_diag_state = 2u;

    /* 3) 写 32 字节测试图样 */
    flash_diag_state = 3u;
    for (i = 0u; i < sizeof(pattern); i++)
    {
        pattern[i] = (uint8_t)(i + 1u);
    }
    if (!W25Q64_Write_Page(pattern, SCRATCH_ADDR, (uint16_t)sizeof(pattern)))
    {
        flash_diag_err = 3u;                              /* 3 = 写失败 */
        return flash_diag_state;
    }
    flash_diag_rcv = 0u;                                  /* 写 OK */
    flash_diag_state = 3u;

    /* 4) 读回比对 */
    flash_diag_state = 4u;
    rx = BSP_OSPI_Get_Rx_Buffer(&hospi2);
    if ((rx == nullptr) || (!W25Q64_Read(SCRATCH_ADDR, (uint16_t)sizeof(pattern))))
    {
        flash_diag_err = 4u;                              /* 4 = 读失败 */
        return flash_diag_state;
    }
    for (i = 0u; i < sizeof(pattern); i++)
    {
        if (rx[i] != pattern[i])
        {
            flash_diag_err = 5u;                          /* 5 = 比对失败 */
            return flash_diag_state;
        }
    }
    flash_diag_raw   = ((uint32_t)rx[3] << 24) | ((uint32_t)rx[2] << 16)
                     | ((uint32_t)rx[1] << 8) | (uint32_t)rx[0];
    flash_diag_state = 4u;                                /* 4 = 全部通过 ✓ */
    return flash_diag_state;
}

bool Flash_Init(void)
{
    /* ★ W25Q64 的 HOLD#(PA1)/WP#(PA3) 在**单线**读时不被 OSPI 驱动(HAL_OSPI_DATA_1_LINE)
       ⇒ 会悬空;而 HOLD# 一旦为低,芯片就"挂起"⇒ 连 JEDEC 都读不到(实测 JEDEC=0x000000)。
       这里只改 PUPDR 补内部上拉,不动复用功能(四线编程时 OSPI 仍能正常驱动)✓ */
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIOA->PUPDR &= ~((3u << (1u * 2u)) | (3u << (3u * 2u)));   /* 清 PA1 / PA3 */
        GPIOA->PUPDR |=  ((1u << (1u * 2u)) | (1u << (3u * 2u)));   /* 01 = 上拉 ✓ */
    }

    W25Q64_Register_Callbacks();                 /* 把驱动的三个回调注册给 OSPI 总线 */

    s_ready = W25Q64_Init(W25Q64_MODE_NORMAL);   /* 内部会读 JEDEC ID 并与 EF 40 17 比对 */
    if (!s_ready)
    {
        Flash_Diag_Probe();      /* 失败 ⇒ 逐层诊断(状态/错误码/命令返回/原始 ID) */
    }
    return s_ready;
}

/**
 * @brief 是否已就绪
 * @return true = 可以读写
 */
bool Flash_Is_Ready(void)
{
    return s_ready;
}

/**
 * @brief 读回 JEDEC ID
 * @return ID
 */
uint32_t Flash_Get_Jedec_Id(void)
{
    return W25Q64_Get_Jedec_Id();
}

/**
 * @brief 芯片容量
 * @return 8 MB
 */
uint32_t Flash_Get_Capacity(void)
{
    return FLASH_CAPACITY;
}

/**
 * @brief 读数据(自动按驱动缓冲上限分段)
 * @return true = 成功
 */
bool Flash_Read(uint32_t addr, void *buf, uint32_t len)
{
    uint8_t *dst = (uint8_t *)buf;
    uint32_t done = 0u;

    if (!s_ready || buf == nullptr || len == 0u || (addr + len) > FLASH_CAPACITY)
    {
        return false;
    }

    while (done < len)
    {
        /* 驱动一次最多收 BSP_OSPI 缓冲那么多(512B),所以这里要分段 */
        uint16_t chunk = (uint16_t)(((len - done) > W25Q64_BUFFER_SIZE) ? W25Q64_BUFFER_SIZE
                                                                       : (len - done));

        if (!W25Q64_Read(addr + done, chunk))
        {
            return false;
        }
        if (!Flash_Wait_Rx(FLASH_WAIT_READ_US))
        {
            return false;
        }

        memcpy(&dst[done], W25Q64_Get_Buffer(), chunk);
        done += chunk;
    }
    return true;
}

/**
 * @brief 写数据(自动按 256B 页分段,不跨页)
 * @return true = 成功
 */
bool Flash_Write(uint32_t addr, const void *buf, uint32_t len)
{
    const uint8_t *src = (const uint8_t *)buf;
    uint32_t done = 0u;

    if (!s_ready || buf == nullptr || len == 0u || (addr + len) > FLASH_CAPACITY)
    {
        return false;
    }

    while (done < len)
    {
        uint32_t cur = addr + done;
        /* 一页 256B,且不能跨页 */
        uint32_t to_page_end = W25Q64_PAGE_SIZE - (cur % W25Q64_PAGE_SIZE);
        uint32_t remain = len - done;
        uint16_t chunk = (uint16_t)((remain < to_page_end) ? remain : to_page_end);

        if (!W25Q64_Write_Page(&src[done], cur, chunk))
        {
            return false;
        }
        if (!Flash_Wait_Idle(FLASH_WAIT_WRITE_US))
        {
            return false;
        }
        done += chunk;
    }
    return true;
}

/**
 * @brief 擦除一段(自动按 4KB 扇区,含向下对齐)
 * @return true = 成功
 */
bool Flash_Erase(uint32_t addr, uint32_t len)
{
    uint32_t start;
    uint32_t end;
    uint32_t cur;

    if (!s_ready || len == 0u || (addr + len) > FLASH_CAPACITY)
    {
        return false;
    }

    start = addr - (addr % W25Q64_SECTOR_SIZE);                        /* 向下对齐 */
    end = addr + len;
    if ((end % W25Q64_SECTOR_SIZE) != 0u)
    {
        end += W25Q64_SECTOR_SIZE - (end % W25Q64_SECTOR_SIZE);        /* 向上补齐 */
    }
    if (end > FLASH_CAPACITY)
    {
        end = FLASH_CAPACITY;
    }

    for (cur = start; cur < end; cur += W25Q64_SECTOR_SIZE)
    {
        if (!W25Q64_Erase_Sector(cur))
        {
            return false;
        }
        if (!Flash_Wait_Idle(FLASH_WAIT_ERASE_US))
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 判断某段是否全 0xFF(已擦除)
 * @return true = 空白,可以直接写
 */
bool Flash_Is_Blank(uint32_t addr, uint32_t len)
{
    uint8_t buf[W25Q64_BUFFER_SIZE];
    uint32_t done = 0u;

    if (!s_ready || len == 0u || (addr + len) > FLASH_CAPACITY)
    {
        return false;
    }

    while (done < len)
    {
        uint16_t chunk = (uint16_t)(((len - done) > sizeof(buf)) ? sizeof(buf) : (len - done));
        uint16_t i;

        if (!Flash_Read(addr + done, buf, chunk))
        {
            return false;
        }
        for (i = 0u; i < chunk; i++)
        {
            if (buf[i] != 0xFFu)
            {
                return false;
            }
        }
        done += chunk;
    }
    return true;
}
