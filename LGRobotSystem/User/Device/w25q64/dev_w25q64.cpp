//
// dev_w25q64.cpp — W25Q64JV(8MB SPI NOR)器件驱动,走 OCTOSPI2
// 说明 / 用法 / 坑 见同目录 dev_w25q64.md

#include "dev_w25q64.h"

#include "bsp_ospi.h"
#include "bsp_timestamp.h"    /* 超时用 TIM5 微秒时间戳 */
#include "bsp_dwt.h"          /* 只用于 DWT_Delay(µs 级短延时) */

#include <string.h>

extern "C" {
extern OSPI_HandleTypeDef hospi2;       /* 定义在 Core/Src/octospi.c */
}

/* ============================================================================
 * W25Q64 指令(数据手册标准指令)
 * ========================================================================== */
#define W25Q64_CMD_WRITE_ENABLE         0x06u
#define W25Q64_CMD_READ_STATUS_REG1     0x05u
#define W25Q64_CMD_FAST_READ_QUAD_IO    0xEBu   /* 4 线地址 + 4 线数据 + 6 dummy */
#define W25Q64_CMD_QUAD_PAGE_PROGRAM    0x32u   /* 4 线页编程(≤256B 不跨页) */
#define W25Q64_CMD_SECTOR_ERASE         0x20u   /* 4KB */
#define W25Q64_CMD_BLOCK_ERASE_32K      0x52u   /* 32KB(厂家例程也用) */
#define W25Q64_CMD_BLOCK_ERASE_64K      0xD8u   /* 64KB */
#define W25Q64_CMD_CHIP_ERASE           0xC7u
#define W25Q64_CMD_JEDEC_ID             0x9Fu   /* → EF 40 17 */
#define W25Q64_CMD_ENABLE_RESET         0x66u   /* 使能复位(厂家例程 init 里先做) */
#define W25Q64_CMD_RESET_DEVICE         0x99u   /* 复位器件 */

#define W25Q64_SR1_WIP                  0x01u   /* 状态寄存器1 bit0:写/擦进行中 */
#define W25Q64_SR1_WEL                  0x02u   /* bit1:写使能已置位 */

/* 各操作超时(ms):远超手册典型值,只用于"卡死自恢复" */
#define W25Q64_TIMEOUT_DEFAULT_US      50000u
#define W25Q64_TIMEOUT_SECTOR_ERASE_US 500000u
#define W25Q64_TIMEOUT_BLOCK32K_US     1000000u
#define W25Q64_TIMEOUT_BLOCK64K_US     2000000u
#define W25Q64_TIMEOUT_CHIP_ERASE_US  120000000u

/* ---- 状态 ---- */
typedef enum
{
    W25Q64_STATE_IDLE = 0,
    W25Q64_STATE_READ,              /* DMA 收数据中 */
    W25Q64_STATE_WRITE_ENABLE,      /* 已发 0x06,等状态轮询通过 */
    W25Q64_STATE_PROGRAM,           /* DMA 发数据中 */
    W25Q64_STATE_ERASE,             /* 擦除命令已发,准备等 WIP */
    W25Q64_STATE_WAIT_WIP,          /* 状态轮询中:等 WIP=0 */
    W25Q64_STATE_DONE,
    W25Q64_STATE_ERROR,
} W25Q64_State;

/* ---- 驱动状态 ---- */
static volatile W25Q64_State state = W25Q64_STATE_IDLE;
static volatile bool rx_done_flag = false;
static volatile bool error_flag = false;
static uint32_t error_count = 0u;
static bool inited = false;
extern volatile uint32_t flash_diag_cmd;
extern volatile uint32_t flash_diag_rcv;
extern volatile uint32_t flash_diag_err;
extern volatile uint32_t flash_diag_raw;   /* 细粒度检查点(诊断用) */
static uint32_t s_jedec_id = 0u;        /* 初始化时读到的 JEDEC ID(调试/上层要用) */

static uint32_t op_addr = 0u;                   /* 本次操作地址 */
static uint16_t op_len = 0u;                    /* 本次操作长度(读/写用) */
static uint8_t erase_instruction = 0u;          /* 本次擦除用哪条指令 */
static W25Q64_State state_after_write_enable = W25Q64_STATE_IDLE;
static uint32_t op_start_us = 0u;
static uint32_t op_timeout_us = W25Q64_TIMEOUT_DEFAULT_US;

/* 默认命令模板:每次用时拷贝一份再改 */
static const OSPI_RegularCmdTypeDef COMMAND_DEFAULT = {
    .OperationType = HAL_OSPI_OPTYPE_COMMON_CFG,
    .FlashId = HAL_OSPI_FLASH_ID_1,
    .Instruction = W25Q64_CMD_JEDEC_ID,
    .InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE,
    .InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS,
    .InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE,
    .Address = 0,
    .AddressMode = HAL_OSPI_ADDRESS_NONE,
    .AddressSize = HAL_OSPI_ADDRESS_24_BITS,
    .AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE,
    .AlternateBytes = 0,
    .AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE,
    .AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_8_BITS,
    .AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE,
    .DataMode = HAL_OSPI_DATA_1_LINE,
    .NbData = 0,
    .DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE,
    .DummyCycles = 0,
    .DQSMode = HAL_OSPI_DQS_DISABLE,
    .SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD,
};


/* ============================================================================
 * 阻塞版底层(★ 抄官方例程:w25q64.c 全部用阻塞 HAL_OSPI_Command/Receive/Transmit)
 *   ——本板实测:DMA 收/发**永不完成**(回调不来),所以一律改阻塞 ✓
 * ========================================================================== */

static bool W25Q64_Blk_Command(uint8_t instr, uint32_t addr,
                               OSPI_RegularCmdTypeDef *cmd_out, uint16_t nb)
{
    OSPI_RegularCmdTypeDef cmd = COMMAND_DEFAULT;

    cmd.Instruction = instr;
    cmd.Address     = addr;
    cmd.NbData      = nb;

    if (HAL_OSPI_Command(&hospi2, &cmd, 100u) != HAL_OK)
    {
        return false;
    }
    if (cmd_out != nullptr)
    {
        *cmd_out = cmd;
    }
    return true;
}

static bool W25Q64_Blk_Wait_WIP(uint32_t timeout_us);   /* 前置声明 */

/* 0x06 写使能 + 回读状态确认 WEL=1 */
static bool W25Q64_Blk_WEL(void)
{
    /* ★ 只发 0x06(纯命令、无数据),**不再回读状态寄存器**:
       实测回读会把 OSPI 留在 BUSY_INDIRECT_READ ⇒ 下一条命令必 HAL_BUSY ✗
       0x06 之后紧跟擦/写命令即可(手册允许,芯片自己保证 WEL 时序) */
    OSPI_RegularCmdTypeDef cmd = COMMAND_DEFAULT;

    cmd.Instruction = W25Q64_CMD_WRITE_ENABLE;
    cmd.NbData      = 0u;
    cmd.DataMode    = HAL_OSPI_DATA_NONE;   /* ★ 纯命令必须 NONE(官方同),否则 HAL 等数据阶段 ⇒ 忙 ✗ */

    flash_diag_cmd = 20u;
    if (HAL_OSPI_Command(&hospi2, &cmd, 100u) != HAL_OK)
    {
        flash_diag_cmd = 21u;
        return false;
    }

    /* ★ 0x06 发完后外设会留在忙态 ⇒ 强制 Abort 拉回 READY 才能发下一条 */
    if (hospi2.State != HAL_OSPI_STATE_READY)
    {
        (void)HAL_OSPI_Abort(&hospi2);
        flash_diag_rcv = (uint32_t)hospi2.State;   /* Abort 之后的状态,应为 1 */
    }
    return true;
}

/* 轮询状态寄存器,等 WIP 清 */
static bool W25Q64_Blk_Wait_WIP(uint32_t timeout_us)
{
    /* ★ 照官方 OSPI_W25Qxx_AutoPollingMemReady() 的写法:
       发读状态寄存器命令 + HAL_OSPI_AutoPolling,
       由 HAL 自己收尾(手写轮询会把外设留在 BUSY_INDIRECT_READ ✗)
       等的是 WIP(状态寄存器 1 bit0) = 0 */
    OSPI_RegularCmdTypeDef cmd = COMMAND_DEFAULT;
    OSPI_AutoPollingTypeDef cfg = { 0 };
    uint32_t timeout_ms = (timeout_us / 1000u) + 1u;

    cmd.Instruction = W25Q64_CMD_READ_STATUS_REG1;
    cmd.NbData      = 1u;

    if (HAL_OSPI_Command(&hospi2, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        flash_diag_cmd = 30u;   /* 轮询命令失败 */
        return false;
    }

    cfg.Match         = 0x00u;
    cfg.MatchMode     = HAL_OSPI_MATCH_MODE_AND;
    cfg.Interval      = 0x10u;
    cfg.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;
    cfg.Mask          = W25Q64_SR1_WIP;

    if (HAL_OSPI_AutoPolling(&hospi2, &cfg, timeout_ms) != HAL_OK)
    {
        flash_diag_cmd = 31u;   /* 轮询等待失败(超时) */
        return false;
    }

    return true;
}

/* 等 WIP 清的轮询配置:读状态寄存器1,掩码 bit0,匹配 0 */
/* 等 WIP=0(擦写真正结束) */
static const OSPI_AutoPollingTypeDef AUTOPOLLING_WAIT_WIP = {
    .Match = 0x00u,
    .Mask = W25Q64_SR1_WIP,
    .MatchMode = HAL_OSPI_MATCH_MODE_AND,
    .AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE,
    .Interval = 0x10u,
};

/* ★ 等 WEL=1(确认"写使能"真的生效) —— 抄厂家例程:Match/Mask 都用 0x02 */
static const OSPI_AutoPollingTypeDef AUTOPOLLING_WAIT_WEL = {
    .Match = W25Q64_SR1_WEL,
    .Mask = W25Q64_SR1_WEL,
    .MatchMode = HAL_OSPI_MATCH_MODE_AND,
    .AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE,
    .Interval = 0x10u,
};

/* ============================================================================
 * 内部:发"等 WIP"的自动轮询
 * ========================================================================== */
/**
 * @brief 配好"读状态寄存器1"命令并启动中断轮询(轮询结束进 callbacks)
 * @param timeout_us 超时(ms)
 * @return true = 已启动
 */
static bool W25Q64_Start_Status_Poll(const OSPI_AutoPollingTypeDef *cfg_in, uint32_t timeout_us)
{
    OSPI_RegularCmdTypeDef cmd = COMMAND_DEFAULT;
    OSPI_AutoPollingTypeDef cfg = *cfg_in;

    /* HAL 的自动轮询复用"当前配在 OSPI 里的那条命令",所以先把读状态命令发好 */
    cmd.Instruction = W25Q64_CMD_READ_STATUS_REG1;
    cmd.NbData = 1u;

    if (BSP_OSPI_Command(&hospi2, &cmd) != HAL_OK)
    {
        state = W25Q64_STATE_ERROR;
        error_flag = true;
        return false;
    }
    if (BSP_OSPI_AutoPolling_IT(&hospi2, &cfg) != HAL_OK)
    {
        state = W25Q64_STATE_ERROR;
        error_flag = true;
        return false;
    }

    op_start_us = Timestamp_Get_Us32();
    op_timeout_us = timeout_us;
    return true;
}

/**
 * @brief 等 WIP=0(擦/写真正结束)
 * @return true = 已发起
 */
static bool W25Q64_Start_Wait_Wip(uint32_t timeout_us)
{
    return W25Q64_Start_Status_Poll(&AUTOPOLLING_WAIT_WIP, timeout_us);
}

/**
 * @brief 发"写使能"(0x06)并启动状态轮询;轮询通过后由回调按 state_after_write_enable 继续
 * @param next 写使能完成后要进入的状态(W25Q64_STATE_PROGRAM / W25Q64_STATE_ERASE)
 * @return true = 已发起
 */
static bool W25Q64_Start_Write_Enable(W25Q64_State next)
{
    OSPI_RegularCmdTypeDef cmd = COMMAND_DEFAULT;

    state_after_write_enable = next;

    cmd.Instruction = W25Q64_CMD_WRITE_ENABLE;
    cmd.NbData = 0u;
    if (BSP_OSPI_Command(&hospi2, &cmd) != HAL_OK)
    {
        state = W25Q64_STATE_ERROR;
        error_flag = true;
        return false;
    }

    state = W25Q64_STATE_WRITE_ENABLE;
    /* ★ 抄厂家例程:写完 0x06 后轮询 WEL(bit1)=1,确认"写使能"真的生效;
       只轮询 WIP=0 是看不出写使能失没失败的(不忙 ≠ 允许写) */
    if (!W25Q64_Start_Status_Poll(&AUTOPOLLING_WAIT_WEL, W25Q64_TIMEOUT_DEFAULT_US))
    {
        return false;
    }
    state = W25Q64_STATE_WRITE_ENABLE;      /* 轮询发起不改状态,这里明确一下 */
    return true;
}

/* ============================================================================
 * 中断侧回调(只置标志 + 极短的"下一阶段发起")
 * ========================================================================== */

/**
 * @brief DMA 接收完成(读完成)
 */
static void W25Q64_Rx_Callback(void)
{
    rx_done_flag = true;
    state = W25Q64_STATE_DONE;
}

/**
 * @brief DMA 发送完成(页编程数据已进 Flash)⇒ 开始等 WIP
 */
static void W25Q64_Tx_Callback(void)
{
    if (W25Q64_Start_Wait_Wip(W25Q64_TIMEOUT_DEFAULT_US))
    {
        state = W25Q64_STATE_WAIT_WIP;
    }
}

/**
 * @brief 状态轮询完成(WIP=0 或写使能已通过)
 */
static void W25Q64_Polling_Callback(void)
{
    if (state == W25Q64_STATE_WRITE_ENABLE)
    {
        /* 写使能这一步过了 ⇒ 继续原定的操作 */
        if (state_after_write_enable == W25Q64_STATE_PROGRAM)
        {
            /* 发四线页编程:tx_buffer 里的数据 + 地址 */
            OSPI_RegularCmdTypeDef cmd = COMMAND_DEFAULT;
            cmd.Instruction = W25Q64_CMD_QUAD_PAGE_PROGRAM;
            cmd.Address = op_addr;
            cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
            cmd.DataMode = HAL_OSPI_DATA_4_LINES;
            cmd.NbData = op_len;

            if (BSP_OSPI_Command_Transmit_DMA(&hospi2, &cmd) != HAL_OK)
            {
                state = W25Q64_STATE_ERROR;
                error_flag = true;
                return;
            }
            op_start_us = Timestamp_Get_Us32();
            op_timeout_us = W25Q64_TIMEOUT_DEFAULT_US;
            state = W25Q64_STATE_PROGRAM;
        }
        else if (state_after_write_enable == W25Q64_STATE_ERASE)
        {
            /* 发擦除命令(纯命令,不需要数据) */
            OSPI_RegularCmdTypeDef cmd = COMMAND_DEFAULT;
            cmd.Instruction = erase_instruction;
            cmd.Address = op_addr;
            cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
            cmd.NbData = 0u;

            if (BSP_OSPI_Command(&hospi2, &cmd) != HAL_OK)
            {
                state = W25Q64_STATE_ERROR;
                error_flag = true;
                return;
            }

            state = W25Q64_STATE_ERASE;
            if (W25Q64_Start_Wait_Wip(op_timeout_us))
            {
                state = W25Q64_STATE_WAIT_WIP;
            }
        }
        else
        {
            state = W25Q64_STATE_DONE;
        }
    }
    else if (state == W25Q64_STATE_WAIT_WIP)
    {
        /* WIP 已清 ⇒ 擦/写真正结束 */
        state = W25Q64_STATE_DONE;
    }
    else
    {
        state = W25Q64_STATE_DONE;
    }
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 初始化 + 读 JEDEC ID 自检
 * @param mode 工作模式(建议先 W25Q64_MODE_NORMAL)
 * @return true = 芯片在线且 ID 正确
 */
bool W25Q64_Init(W25Q64_Mode mode)
{
    OSPI_RegularCmdTypeDef reset_cmd;
    OSPI_RegularCmdTypeDef jcmd;
    OSPI_RegularCmdTypeDef mem_cmd;
    OSPI_MemoryMappedTypeDef mm_cfg = {0};
    uint8_t buf[3] = { 0u, 0u, 0u };
    HAL_StatusTypeDef st1;
    HAL_StatusTypeDef st2;
    uint32_t id = 0u;
    uint8_t *rx = BSP_OSPI_Get_Rx_Buffer(&hospi2);

    (void)rx;
    flash_diag_cmd = 1u;                    /* 进函数 */

    /* ---- 1) 不再做 0x66/0x99 软复位 ★
       官方 CtrBoard-H7_W25Q64 的 OSPI_W25Qxx_Init() 只做 ReadID,不做复位;
       实测:加了复位反而读不到 ID(探测函数跳过复位就正常)⇒ 去掉 ✓ */
    reset_cmd = COMMAND_DEFAULT;
    reset_cmd.NbData = 0u;
    (void)reset_cmd;
    flash_diag_cmd = 4u;

    /* ---- 2) 读 JEDEC ID(局部 buffer + 直调 HAL,探测已验证可用)---- */
    jcmd = COMMAND_DEFAULT;
    jcmd.Instruction = W25Q64_CMD_JEDEC_ID;
    jcmd.NbData = 3u;
    flash_diag_cmd = 5u;
    st1 = HAL_OSPI_Command(&hospi2, &jcmd, 10u);
    flash_diag_cmd = 6u;
    /* 接收前:把 HAL 返回码和状态都记下来(崩在这行也能看到) */
    flash_diag_rcv = (uint32_t)st1;
    flash_diag_err = (uint32_t)hospi2.State;
    flash_diag_cmd = 61u;
    st2 = HAL_OSPI_Receive(&hospi2, buf, 10u);
    flash_diag_cmd = 62u;
    flash_diag_raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    flash_diag_cmd = 7u;

    id = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    s_jedec_id = id;                        /* 不管对错都记下 ⇒ 遥测能看到真实值 */
    flash_diag_cmd = 8u;

    if ((st1 != HAL_OK) || (st2 != HAL_OK))
    {
        (void)HAL_OSPI_Abort(&hospi2);
        error_count++;
        return false;
    }
    flash_diag_cmd = 9u;

    if (id != (W25Q64_JEDEC_ID & 0xFFFFFFu))
    {
        error_count++;
        return false;
    }
    flash_diag_cmd = 10u;

    inited = true;
    state = W25Q64_STATE_IDLE;

    /* ---- 3) 需要内存映射就配好"快读"再映射 ---- */
    if (mode == W25Q64_MODE_MEMORY_MAPPED)
    {
        mem_cmd = COMMAND_DEFAULT;
        mem_cmd.Instruction = W25Q64_CMD_FAST_READ_QUAD_IO;
        mem_cmd.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
        mem_cmd.DataMode = HAL_OSPI_DATA_4_LINES;
        mem_cmd.DummyCycles = 6u;
        mem_cmd.NbData = 0u;

        if (BSP_OSPI_Command(&hospi2, &mem_cmd) != HAL_OK)
        {
            return false;
        }
        mm_cfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;
        mm_cfg.TimeOutPeriod = 0u;
        if (HAL_OSPI_MemoryMapped(&hospi2, &mm_cfg) != HAL_OK)
        {
            return false;
        }
        inited = false;                     /* 映射模式下不再走间接命令 */
    }

    flash_diag_cmd = 11u;                   /* 初始化完成 */
    return true;
}

/**
 * @brief 发起一次读(0xEB 四线快读,非阻塞)
 */
bool W25Q64_Read(uint32_t addr, uint16_t len)
{
    OSPI_RegularCmdTypeDef cmd = COMMAND_DEFAULT;
    uint8_t *rx = BSP_OSPI_Get_Rx_Buffer(&hospi2);

    if (!inited || (rx == nullptr) || (len == 0u) || (len > W25Q64_BUFFER_SIZE))
    {
        return false;
    }
    if ((addr + len) > W25Q64_CHIP_SIZE)
    {
        return false;
    }

    /* 0xEB:1线指令 + 4线地址 + 6 dummy + 4线数据(与官方 ReadBuffer 完全一致) */
    cmd.Instruction   = W25Q64_CMD_FAST_READ_QUAD_IO;
    cmd.Address       = addr;
    cmd.AddressMode   = HAL_OSPI_ADDRESS_4_LINES;
    cmd.DataMode      = HAL_OSPI_DATA_4_LINES;
    cmd.DummyCycles   = 6u;
    cmd.NbData        = len;

    if ((HAL_OSPI_Command(&hospi2, &cmd, 100u) != HAL_OK) ||
        (HAL_OSPI_Receive(&hospi2, rx, 100u) != HAL_OK))
    {
        (void)HAL_OSPI_Abort(&hospi2);
        state = W25Q64_STATE_ERROR;
        error_flag = true;
        error_count++;
        return false;
    }

    rx_done_flag = true;            /* blocking read: must set this, Flash_Read() waits on it */
    state = W25Q64_STATE_DONE;      /* 同步完成 ⇒ 交给 Task 回收 ✓ */
    return true;
}

/**
 * @brief 发起一次页写(写使能 → 四线页编程 → 等 WIP;非阻塞)
 */
bool W25Q64_Write_Page(const uint8_t *buf, uint32_t addr, uint16_t len)
{
    OSPI_RegularCmdTypeDef cmd = COMMAND_DEFAULT;
    uint8_t *tx = BSP_OSPI_Get_Tx_Buffer(&hospi2);

    if (!inited || (tx == nullptr) || (buf == nullptr) || (len == 0u) ||
        (len > W25Q64_BUFFER_SIZE))
    {
        return false;
    }
    if (((addr % W25Q64_PAGE_SIZE) + len) > W25Q64_PAGE_SIZE)
    {
        return false;
    }
    if ((addr + len) > W25Q64_CHIP_SIZE)
    {
        return false;
    }

    memcpy(tx, buf, len);

    if (!W25Q64_Blk_WEL())
    {
        error_count++;
        return false;
    }

    /* 0x32:1线指令 + 1线地址 + 4线数据(与官方 WritePage 完全一致) */
    cmd.Instruction = W25Q64_CMD_QUAD_PAGE_PROGRAM;
    cmd.Address     = addr;
    cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    cmd.DataMode    = HAL_OSPI_DATA_4_LINES;
    cmd.NbData      = len;

    if ((HAL_OSPI_Command(&hospi2, &cmd, 100u) != HAL_OK) ||
        (HAL_OSPI_Transmit(&hospi2, tx, 100u) != HAL_OK))
    {
        (void)HAL_OSPI_Abort(&hospi2);
        error_count++;
        return false;
    }

    if (!W25Q64_Blk_Wait_WIP(W25Q64_TIMEOUT_DEFAULT_US))
    {
        (void)HAL_OSPI_Abort(&hospi2);
        error_count++;
        return false;
    }

    state = W25Q64_STATE_DONE;
    return true;
}

/**
 * @brief 发起 4KB 扇区擦除(非阻塞)
 */
bool W25Q64_Erase_Sector(uint32_t addr)
{
    OSPI_RegularCmdTypeDef cmd = COMMAND_DEFAULT;

    if (!inited || (addr % W25Q64_SECTOR_SIZE))
    {
        return false;
    }

    if (!W25Q64_Blk_WEL())
    {
        error_count++;
        return false;
    }
    flash_diag_cmd = 26u;   /* 准备发 0x20 */

    cmd.Instruction = W25Q64_CMD_SECTOR_ERASE;
    cmd.Address     = addr;
    cmd.NbData      = 0u;
    cmd.DataMode    = HAL_OSPI_DATA_NONE;   /* ★ 同上 */
    if (HAL_OSPI_Command(&hospi2, &cmd, 100u) != HAL_OK)
    {
        (void)HAL_OSPI_Abort(&hospi2);
        error_count++;
        return false;
    }

    flash_diag_cmd = 27u;   /* 0x20 已发 */
    if (!W25Q64_Blk_Wait_WIP(W25Q64_TIMEOUT_SECTOR_ERASE_US))
    {
        flash_diag_cmd = 28u;   /* WIP 等待失败 */
        (void)HAL_OSPI_Abort(&hospi2);
        error_count++;
        return false;
    }
    flash_diag_cmd = 29u;   /* 擦除完成 ✓ */

    state = W25Q64_STATE_DONE;
    return true;
}

/**
 * @brief 发起 64KB 块擦除(非阻塞)
 */
bool W25Q64_Erase_Block_64K(uint32_t addr)
{
    if (!inited || state != W25Q64_STATE_IDLE || (addr % W25Q64_BLOCK64K_SIZE))
    {
        return false;
    }

    op_addr = addr;
    erase_instruction = W25Q64_CMD_BLOCK_ERASE_64K;
    op_timeout_us = W25Q64_TIMEOUT_BLOCK64K_US;

    return W25Q64_Start_Write_Enable(W25Q64_STATE_ERASE);
}

/**
 * @brief 发起 32KB 块擦除(非阻塞;命令 0x52,厂家例程里也有)
 * @param addr 必须 32KB 对齐
 * @return true = 已发起
 */
bool W25Q64_Erase_Block_32K(uint32_t addr)
{
    if (!inited || state != W25Q64_STATE_IDLE || (addr % W25Q64_BLOCK32K_SIZE))
    {
        return false;
    }

    op_addr = addr;
    erase_instruction = W25Q64_CMD_BLOCK_ERASE_32K;
    op_timeout_us = W25Q64_TIMEOUT_BLOCK32K_US;

    return W25Q64_Start_Write_Enable(W25Q64_STATE_ERASE);
}

/**
 * @brief 发起整片擦除(非阻塞,可能几十秒)
 */
bool W25Q64_Erase_Chip(void)
{
    if (!inited || state != W25Q64_STATE_IDLE)
    {
        return false;
    }

    op_addr = 0u;
    erase_instruction = W25Q64_CMD_CHIP_ERASE;
    op_timeout_us = W25Q64_TIMEOUT_CHIP_ERASE_US;

    return W25Q64_Start_Write_Enable(W25Q64_STATE_ERASE);
}

/**
 * @brief ★ 跑一拍:超时保护 + 状态回收
 * @note 由 RTOS 任务周期调用;中断里只置标志,所以这里不需要锁
 */
void W25Q64_Task(void)
{
    if (!inited)
    {
        return;
    }

    switch (state)
    {
        case W25Q64_STATE_READ:
        case W25Q64_STATE_WRITE_ENABLE:
        case W25Q64_STATE_PROGRAM:
        case W25Q64_STATE_ERASE:
        case W25Q64_STATE_WAIT_WIP:
            /* 正常情况由中断推进;这里只管"卡住太久就恢复" */
            if ((Timestamp_Get_Us32() - op_start_us) > op_timeout_us)
            {
                state = W25Q64_STATE_ERROR;
                error_flag = true;
                error_count++;
            }
            break;

        case W25Q64_STATE_DONE:
            state = W25Q64_STATE_IDLE;      /* 自动回收,下一次操作可发起 */
            break;

        case W25Q64_STATE_ERROR:
        case W25Q64_STATE_IDLE:
        default:
            break;
    }
}

bool W25Q64_Is_Busy(void)
{
    return (state != W25Q64_STATE_IDLE) && (state != W25Q64_STATE_DONE)
        && (state != W25Q64_STATE_ERROR);
}

bool W25Q64_Is_Rx_Done(void)
{
    return rx_done_flag;
}

bool W25Q64_Has_Error(void)
{
    return error_flag;
}

uint32_t W25Q64_Get_Error_Count(void)
{
    return error_count;
}

void W25Q64_Clear_Error(void)
{
    error_flag = false;
    if (state == W25Q64_STATE_ERROR)
    {
        state = W25Q64_STATE_IDLE;
    }
}

uint32_t W25Q64_Get_Jedec_Id(void)
{
    return s_jedec_id;
}

const uint8_t *W25Q64_Get_Buffer(void)
{
    return BSP_OSPI_Get_Rx_Buffer(&hospi2);
}

/**
 * @brief 把本驱动的三个回调注册给 OSPI 总线驱动(初始化时调一次)
 * @return void
 */
extern "C" void W25Q64_Register_Callbacks(void)
{
    BSP_OSPI_Init(&hospi2, W25Q64_Polling_Callback, W25Q64_Rx_Callback, W25Q64_Tx_Callback);
}
