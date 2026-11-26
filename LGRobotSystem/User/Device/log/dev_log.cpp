//
// dev_log.cpp — 运行日志(内存环形 → 外部 Flash 追加落盘 + 可选实时控制台)
// 说明 / 用法 / 坑 见同目录 dev_log.md

#include "dev_log.h"

#include "bsp_flash.h"
#include "bsp_timestamp.h"
#include "crc_ccitt.h"

#include "main.h"       /* __get_PRIMASK / GPIO 等 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * 存储布局(基于 User/Bsp/flash 划好的 LOG 区)
 * ========================================================================== */

#define LOG_SECTOR_SIZE     4096u
#define LOG_HEADER_ADDR     (FLASH_LOG_ADDR)                                 /* 0x001000 */
#define LOG_DATA_ADDR       (FLASH_LOG_ADDR + LOG_SECTOR_SIZE)               /* 0x002000 */
#define LOG_DATA_SIZE       (FLASH_LOG_SIZE - LOG_SECTOR_SIZE)               /* ≈1MB - 4KB */
#define LOG_MAX_RECORDS     (LOG_DATA_SIZE / LOG_RECORD_SIZE)                /* ≈4064 条 */

/* 头部检查点槽的魔数(小端写出来是 31 47 4F 4C) */
#define LOG_HEADER_MAGIC    0x4C4F4732u   /* v2:槽 32B,带本/上次首地址+写地址 */

/* 记录里的魔数:字节 'L' 'G' */
#define LOG_RECORD_MAGIC_L  'L'
#define LOG_RECORD_MAGIC_G  'G'

/* 记录字段偏移 */
#define LOG_REC_OFF_MAGIC   0u
#define LOG_REC_OFF_LEN     2u
#define LOG_REC_OFF_SEQ     4u
#define LOG_REC_OFF_CRC     6u
#define LOG_REC_OFF_PAYLOAD 8u

/* ============================================================================
 * 内部状态
 * ========================================================================== */
/* 落盘状态 */
static bool s_ready = false;
static bool s_full = false;
static bool s_sector_erased = false;    /* 当前记录所在扇区是否已擦 */
static uint32_t s_data_offset = 0u;     /* 已写入字节数(相对 LOG_DATA_ADDR,256 对齐) */
static uint32_t s_seq = 0u;             /* 已写记录条数(绝对序号) */
static uint32_t s_checkpoint_index = 0u;/* 下一个检查点槽号(0~255 循环用,模运算) */
static uint32_t s_boot_id = 1u;
static uint32_t s_boot_start = 0u;
static uint32_t s_prev_offset = 0xFFFFFFFFu;
static uint32_t s_prev_start = 0xFFFFFFFFu;
static uint32_t s_wrap_count = 0u;

/* 内存环形缓冲(单生产者可多个,消费者只有 Log_Task) */
static uint8_t s_ring[LOG_RING_BYTES];
static volatile uint16_t s_ring_head = 0u;
static volatile uint16_t s_ring_tail = 0u;

/* 统计 */
static uint32_t s_dropped = 0u;

/* 记录组装缓冲(256B;payload 放在 +8 处) */
static uint8_t s_record[LOG_RECORD_SIZE];

/* 控制台 */
static Log_Console_Callback s_console = nullptr;
static bool s_console_enabled = false;

/* Log_Task 限速 */
static uint32_t s_next_task_us = 0u;

/* ============================================================================
 * 小工具
 * ========================================================================== */

static void Log_Write_U16(uint8_t *__p, uint16_t __v)
{
    __p[0] = (uint8_t)(__v & 0xFFu);
    __p[1] = (uint8_t)((__v >> 8) & 0xFFu);
}

static uint16_t Log_Read_U16(const uint8_t *__p)
{
    return (uint16_t)((uint16_t)__p[0] | ((uint16_t)__p[1] << 8));
}

static void Log_Write_U32(uint8_t *__p, uint32_t __v)
{
    __p[0] = (uint8_t)(__v & 0xFFu);
    __p[1] = (uint8_t)((__v >> 8) & 0xFFu);
    __p[2] = (uint8_t)((__v >> 16) & 0xFFu);
    __p[3] = (uint8_t)((__v >> 24) & 0xFFu);
}

static uint32_t Log_Read_U32(const uint8_t *__p)
{
    return ((uint32_t)__p[0]) | ((uint32_t)__p[1] << 8) |
           ((uint32_t)__p[2] << 16) | ((uint32_t)__p[3] << 24);
}

/**
 * @brief 一条 256B 记录是否有效(魔数 + 长度 + payload CRC)
 */
static bool Log_Record_Is_Valid(const uint8_t *__rec)
{
    uint16_t len;
    uint16_t crc;

    if ((__rec[LOG_REC_OFF_MAGIC] != (uint8_t)LOG_RECORD_MAGIC_L) ||
        (__rec[LOG_REC_OFF_MAGIC + 1u] != (uint8_t)LOG_RECORD_MAGIC_G))
    {
        return false;
    }

    len = Log_Read_U16(&__rec[LOG_REC_OFF_LEN]);
    if ((len == 0u) || (len > LOG_RECORD_PAYLOAD_MAX))
    {
        return false;
    }

    crc = Log_Read_U16(&__rec[LOG_REC_OFF_CRC]);
    return (crc == crc_ccitt(&__rec[LOG_REC_OFF_PAYLOAD], len));
}

/* ============================================================================
 * 环形缓冲(2KB)
 *   push:Log_Printf / Log_Record(任意任务,短临界区保护)
 *   peek/pop:只有 Log_Task 用(写盘成功后才 pop,失败就留在里面)
 * ========================================================================== */

static uint16_t Ring_Used(void)
{
    uint16_t head = s_ring_head;
    uint16_t tail = s_ring_tail;

    return (uint16_t)((head >= tail) ? (head - tail) : (uint16_t)(LOG_RING_BYTES - tail + head));
}

static bool Ring_Push(const uint8_t *__data, uint16_t __len)
{
    uint32_t primask;
    uint16_t used;
    uint16_t free_bytes;
    uint16_t head;
    uint16_t need;
    uint16_t i;

    if ((__data == nullptr) || (__len == 0u) || (__len > LOG_FRAME_MAX))
    {
        s_dropped += __len;
        return false;
    }

    need = (uint16_t)(__len + 2u);

    primask = __get_PRIMASK();
    __disable_irq();

    used = Ring_Used();
    free_bytes = (uint16_t)(LOG_RING_BYTES - used - 1u);

    if (need > free_bytes)
    {
        __set_PRIMASK(primask);
        s_dropped += __len;         /* 满了就丢,绝不阻塞调用者 */
        return false;
    }

    head = s_ring_head;
    s_ring[head] = (uint8_t)(__len & 0xFFu);
    head = (uint16_t)((head + 1u) % LOG_RING_BYTES);
    s_ring[head] = (uint8_t)((__len >> 8) & 0xFFu);
    head = (uint16_t)((head + 1u) % LOG_RING_BYTES);

    for (i = 0u; i < __len; i++)
    {
        s_ring[head] = __data[i];
        head = (uint16_t)((head + 1u) % LOG_RING_BYTES);
    }

    s_ring_head = head;
    __set_PRIMASK(primask);
    return true;
}

/**
 * @brief 看一眼队首那一帧(不移动 tail)
 * @return true = 有完整的一帧(长度写进 __len,内容写进 __out)
 */
static bool Ring_Peek(uint8_t *__out, uint16_t *__len)
{
    uint16_t used = Ring_Used();
    uint16_t pos = s_ring_tail;
    uint16_t len;
    uint16_t i;

    if (used < 2u)
    {
        return false;
    }

    len = (uint16_t)s_ring[pos];
    pos = (uint16_t)((pos + 1u) % LOG_RING_BYTES);
    len |= (uint16_t)((uint16_t)s_ring[pos] << 8);
    pos = (uint16_t)((pos + 1u) % LOG_RING_BYTES);

    /* 长度不可能非法(只有本模块写);真出现就当环形坏了,整体丢掉,避免死循环 */
    if ((len == 0u) || (len > LOG_FRAME_MAX))
    {
        s_dropped += used;
        s_ring_tail = s_ring_head;
        return false;
    }

    if (used < (uint16_t)(len + 2u))
    {
        return false;               /* 整帧还没到齐 */
    }

    for (i = 0u; i < len; i++)
    {
        __out[i] = s_ring[pos];
        pos = (uint16_t)((pos + 1u) % LOG_RING_BYTES);
    }

    *__len = len;
    return true;
}

static void Ring_Pop(uint16_t __len)
{
    s_ring_tail = (uint16_t)((s_ring_tail + __len + 2u) % LOG_RING_BYTES);
}

/* ============================================================================
 * 落盘
 * ========================================================================== */

/**
 * @brief 写一个头部检查点(槽用满一轮就先擦头部扇区)
 */
static bool Log_Write_Checkpoint(void)
{
    uint8_t slot[LOG_HEADER_SLOT_SIZE];
    uint32_t slot_addr;
    uint16_t crc;

    if ((s_checkpoint_index % LOG_HEADER_SLOT_COUNT) == 0u)
    {
        /* 256 个槽用完了 ⇒ 擦掉整个头部扇区,从槽 0 重新开始(磨损均衡) */
        flash_diag_state = 104u;
        if (!Flash_Erase(LOG_HEADER_ADDR, LOG_SECTOR_SIZE))
        {
            return false;
        }
    }

    slot_addr = LOG_HEADER_ADDR + (s_checkpoint_index % LOG_HEADER_SLOT_COUNT) * LOG_HEADER_SLOT_SIZE;

    memset(slot, 0xFF, sizeof(slot));
    Log_Write_U32(&slot[0],  LOG_HEADER_MAGIC);
    Log_Write_U32(&slot[4],  s_boot_id);
    Log_Write_U32(&slot[8],  s_data_offset);
    Log_Write_U32(&slot[12], s_boot_start);
    Log_Write_U32(&slot[16], s_prev_offset);
    Log_Write_U32(&slot[20], s_prev_start);
    crc = crc_ccitt(slot, 24u);
    Log_Write_U16(&slot[24], crc);

    if (!Flash_Write(slot_addr, slot, LOG_HEADER_SLOT_SIZE))
    {
        return false;
    }

    s_checkpoint_index++;
    return true;
}

/**
 * @brief 保证"当前记录所在扇区"已经擦过
 */
static bool Log_Ensure_Sector(void)
{
    uint32_t sector_addr;

    if (s_sector_erased)
    {
        return true;
    }

    sector_addr = LOG_DATA_ADDR + (s_data_offset - (s_data_offset % LOG_SECTOR_SIZE));

    if (!Flash_Erase(sector_addr, LOG_SECTOR_SIZE))
    {
        return false;
    }

    s_sector_erased = true;
    return true;
}

/**
 * @brief 把一帧 payload 落成一条 256B 记录
 */
static bool Log_Write_Record(const uint8_t *__payload, uint16_t __len)
{
    uint16_t crc;

    if (s_full)
    {
        s_dropped += __len;
        return false;
    }

    if ((__len == 0u) || (__len > LOG_RECORD_PAYLOAD_MAX))
    {
        s_dropped += __len;
        return false;
    }

    if (!Log_Ensure_Sector())
    {
        s_dropped += __len;
        return false;
    }

    /* 组装记录(payload 已在 +8 处) */
    memset(s_record, 0xFF, LOG_RECORD_SIZE);
    s_record[LOG_REC_OFF_MAGIC] = (uint8_t)LOG_RECORD_MAGIC_L;
    s_record[LOG_REC_OFF_MAGIC + 1u] = (uint8_t)LOG_RECORD_MAGIC_G;
    Log_Write_U16(&s_record[LOG_REC_OFF_LEN], __len);
    Log_Write_U16(&s_record[LOG_REC_OFF_SEQ], (uint16_t)(s_seq & 0xFFFFu));
    crc = crc_ccitt(__payload, __len);
    Log_Write_U16(&s_record[LOG_REC_OFF_CRC], crc);

    if (!Flash_Write(LOG_DATA_ADDR + s_data_offset, s_record, LOG_RECORD_SIZE))
    {
        s_dropped += __len;
        return false;
    }

    s_data_offset += LOG_RECORD_SIZE;
    s_seq++;

    /* 跨进新扇区:下一条写之前要先擦它 */
    if ((s_data_offset % LOG_SECTOR_SIZE) == 0u)
    {
        s_sector_erased = false;
    }

    if (s_data_offset >= LOG_DATA_SIZE)
    {
        /* 写满 ⇒ 回绕:从开头擦掉重写(不再停住)✓ */
        s_data_offset = 0u;
        s_sector_erased = false;
        s_wrap_count++;
    }

    /* 定期落检查点 */
    if ((s_seq % LOG_CHECKPOINT_RECORDS) == 0u)
    {
        (void)Log_Write_Checkpoint();
    }

    return true;
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 初始化:读头部检查点并对齐到上次的尾部
 */
extern volatile uint32_t flash_diag_state;

bool Log_Init(void)
{
    flash_diag_state = 100u;   /* enter Log_Init */
    uint8_t chunk[512];
    uint8_t *slot;
    bool found = false;
    uint32_t best_seq = 0u;
    uint32_t best_offset = 0u;
    uint32_t best_slot = 0u;
    uint32_t best_boot = 0u;
    uint32_t best_start = 0u;
    uint32_t best_prev_offset = 0xFFFFFFFFu;
    uint32_t best_prev_start = 0xFFFFFFFFu;
    uint32_t boot_id;
    uint32_t seq;
    uint32_t offset;
    uint16_t crc;
    uint32_t i;
    uint32_t j;

    /* 先复位自己的状态 */
    s_ready = false;
    s_full = false;
    s_sector_erased = false;
    s_data_offset = 0u;
    s_seq = 0u;
    s_checkpoint_index = 0u;
    s_dropped = 0u;
    s_ring_head = 0u;
    s_ring_tail = 0u;
    s_next_task_us = Timestamp_Get_Us32();

    flash_diag_state = 101u;
    if (!Flash_Is_Ready())
    {
        return false;
    }

    /* ---- 扫 256 个检查点槽(每次 512B = 32 槽,共 8 次读) ---- */
    for (i = 0u; (i * 512u) < (LOG_HEADER_SLOT_COUNT * LOG_HEADER_SLOT_SIZE); i++)
    {
        flash_diag_state = 102u;
        if (!Flash_Read(LOG_HEADER_ADDR + i * 512u, chunk, 512u))
        {
            return false;
        }

        for (j = 0u; j < (512u / LOG_HEADER_SLOT_SIZE); j++)
        {
            slot = &chunk[j * LOG_HEADER_SLOT_SIZE];

            if (Log_Read_U32(&slot[0]) != LOG_HEADER_MAGIC)
            {
                continue;
            }

            crc = Log_Read_U16(&slot[24]);   /* v2:CRC 在 +24,覆盖 0~23 */
            if (crc != crc_ccitt(slot, 24u))
            {
                continue;
            }

            boot_id = Log_Read_U32(&slot[4]);
            offset  = Log_Read_U32(&slot[8]);

            if ((offset % LOG_RECORD_SIZE) != 0u)
            {
                continue;
            }

            if ((!found) || (boot_id > best_boot) ||
                ((boot_id == best_boot) && ((i * (512u / LOG_HEADER_SLOT_SIZE) + j) >= best_slot)))
            {
                found = true;
                best_boot = boot_id;
                best_offset = offset;
                best_start = Log_Read_U32(&slot[12]);
                best_prev_offset = Log_Read_U32(&slot[16]);
                best_prev_start = Log_Read_U32(&slot[20]);
                best_slot = i * (512u / LOG_HEADER_SLOT_SIZE) + j;
            }
        }
    }

    /* ---- 没有任何有效检查点:全新开始 ---- */
    if (!found)
    {
        if (!Flash_Erase(LOG_HEADER_ADDR, LOG_SECTOR_SIZE))
        {
            return false;
        }
        flash_diag_state = 105u;
        if (!Flash_Erase(LOG_DATA_ADDR, LOG_SECTOR_SIZE))
        {
            return false;
        }

        s_data_offset = 0u;
        s_seq = 0u;
        s_checkpoint_index = 0u;
        s_boot_id = 1u;
        s_boot_start = 0u;
        s_prev_offset = 0xFFFFFFFFu;
        s_prev_start = 0xFFFFFFFFu;
        s_wrap_count = 0u;
        s_sector_erased = true;     /* 首扇区刚擦过 */

        flash_diag_state = 106u;
        if (!Log_Write_Checkpoint())
        {
            return false;
        }

        s_ready = true;
        flash_diag_state = 120u;
        return true;
    }

    /* ---- 有检查点:接着它的位置往后扫,补齐检查点之后写的那几条 ---- */
    s_data_offset = best_offset;
    s_seq = best_seq;
    s_checkpoint_index = best_slot + 1u;

    /* 上电编号 +1,并把上一次的 start/stop 记下来(保险用)✓ */
    s_boot_id = best_boot + 1u;
    s_boot_start = best_offset;
    s_prev_offset = best_offset;
    s_prev_start = (best_start != 0u) ? best_start : best_offset;

    /* 当前扇区上次"进入时"已擦过 ⇒ 剩下是空白;正好在扇区头上则还没擦 */
    s_sector_erased = ((s_data_offset % LOG_SECTOR_SIZE) != 0u);

    while ((s_data_offset + LOG_RECORD_SIZE) <= LOG_DATA_SIZE)
    {
        if (!Flash_Read(LOG_DATA_ADDR + s_data_offset, s_record, LOG_RECORD_SIZE))
        {
            return false;
        }

        if (!Log_Record_Is_Valid(s_record))
        {
            break;
        }

        s_data_offset += LOG_RECORD_SIZE;
        s_seq++;

        /* 最多补扫一个检查点周期,避免异常时扫很久 */
        if ((s_seq - best_seq) >= LOG_CHECKPOINT_RECORDS)
        {
            break;
        }
    }

    if (s_data_offset >= LOG_DATA_SIZE)
    {
        s_full = true;
    }

    s_ready = true;
    flash_diag_state = 121u;
    return true;
}

/**
 * @brief 周期跑:把环形里的帧落盘(内部限速)
 */
void Log_Task(void)
{
    uint16_t len;
    uint8_t i;

    if (!s_ready)
    {
        return;
    }

    if ((int32_t)(Timestamp_Get_Us32() - s_next_task_us) < 0)
    {
        return;
    }
    s_next_task_us += LOG_TASK_INTERVAL_US;

    for (i = 0u; i < LOG_TASK_MAX_RECORDS; i++)
    {
        /* payload 直接 peek 到记录缓冲的 +8 处,省一次拷贝 */
        if (!Ring_Peek(&s_record[LOG_REC_OFF_PAYLOAD], &len))
        {
            break;
        }

        /* 写成功才 pop ⇒ 写失败时这一帧留在环形里,下拍再试 */
        if (!Log_Write_Record(&s_record[LOG_REC_OFF_PAYLOAD], len))
        {
            break;
        }

        Ring_Pop(len);
    }
}

/**
 * @brief 写一条文本日志
 */
void Log_Printf(Enum_Log_Level __level, const char *__format, ...)
{
    static const char *const tags[] = {"I:", "W:", "E:"};
    char line[LOG_FRAME_MAX];
    const char *tag = ((uint32_t)__level < 3u) ? tags[(uint32_t)__level] : tags[0];
    uint64_t ms = Timestamp_Get_Ms();
    int head_len;
    int body_len;
    uint16_t total;
    va_list args;

    if (__format == nullptr)
    {
        return;
    }

    /* 前缀:[秒.毫秒] 级别   —— 全整数,不依赖浮点 printf */
    head_len = snprintf(line, sizeof(line), "[%lu.%03lu] %s ",
                        (unsigned long)(ms / 1000u), (unsigned long)(ms % 1000u), tag);
    if (head_len < 0)
    {
        return;
    }
    if (head_len > (int)(sizeof(line) - 1u))
    {
        head_len = (int)(sizeof(line) - 1u);
    }

    va_start(args, __format);
    body_len = vsnprintf(&line[head_len], sizeof(line) - (size_t)head_len, __format, args);
    va_end(args);
    if (body_len < 0)
    {
        return;
    }

    /* 截断保护 + 补 CRLF */
    total = (uint16_t)(head_len + ((body_len > (int)(sizeof(line) - (size_t)head_len - 1)) ? (int)(sizeof(line) - (size_t)head_len - 1) : body_len));

    if ((total + 2u) <= (uint16_t)sizeof(line))
    {
        line[total] = '\r';
        line[total + 1u] = '\n';
        total = (uint16_t)(total + 2u);
    }

    (void)Ring_Push((const uint8_t *)line, total);

    /* 控制台(比如 USB CDC) */
    if (s_console_enabled && (s_console != nullptr))
    {
        s_console(line, total);
    }
}

/**
 * @brief 写一帧原始二进制数据
 */
void Log_Record(const void *__data, uint16_t __len)
{
    if (__data == nullptr)
    {
        return;
    }

    /* 二进制不进控制台,只进环形 */
    (void)Ring_Push((const uint8_t *)__data, __len);
}

/**
 * @brief 注册实时控制台回调
 */
void Log_Set_Console(Log_Console_Callback __callback)
{
    s_console = __callback;
}

/**
 * @brief 打开/关闭实时控制台
 */
void Log_Console_Enable(bool __enable)
{
    s_console_enabled = __enable;
}

/**
 * @brief 是否就绪
 */
bool Log_Is_Ready(void)
{
    return s_ready;
}

/**
 * @brief 已落盘字节数
 */
uint32_t Log_Get_Written_Bytes(void)
{
    return s_data_offset;
}

/**
 * @brief 已写记录条数
 */
uint32_t Log_Get_Seq(void)
{
    return s_seq;
}

/**
 * @brief 丢弃字节数
 */
uint32_t Log_Get_Dropped_Bytes(void)
{
    return s_dropped;
}

/**
 * @brief 记录区是否已满
 */
bool Log_Is_Full(void)
{
    /* 环形:写满后会从开头擦掉重写 ⇒ 这里表示"已经覆盖过旧数据"(回绕过至少一次) */
    return (s_wrap_count > 0u);
}

/**
 * @brief 读回记录区内容(给上位机 dump)
 */
bool Log_Read(uint32_t __offset, void *__buffer, uint32_t __len)
{
    if ((!s_ready) || (__buffer == nullptr) || (__len == 0u))
    {
        return false;
    }

    if ((__offset + __len) > LOG_DATA_SIZE)
    {
        return false;
    }

    return Flash_Read(LOG_DATA_ADDR + __offset, __buffer, __len);
}

/**
 * @brief 清空日志(擦头部扇区 + 记录区首扇区,从头记)
 */
bool Log_Erase_All(void)
{
    if (!Flash_Is_Ready())
    {
        return false;
    }

    if (!Flash_Erase(LOG_HEADER_ADDR, LOG_SECTOR_SIZE))
    {
        return false;
    }
    if (!Flash_Erase(LOG_DATA_ADDR, LOG_SECTOR_SIZE))
    {
        return false;
    }

    s_data_offset = 0u;
    s_seq = 0u;
    s_checkpoint_index = 0u;
    s_sector_erased = true;
    s_full = false;
    s_record[LOG_REC_OFF_PAYLOAD] = 0u;      /* 无害,只是别让静态分析抱怨 */

    return Log_Write_Checkpoint();
}

/**
 * @brief float → 字符串(替代 %f;参考工程 dev_log 里 Float2Str 的用途)
 */
void Log_Float2Str(char *__str, float __value)
{
    int32_t flag;
    int32_t head;
    int32_t point;

    if (__str == nullptr)
    {
        return;
    }

    flag = (__value < 0.0f) ? 1 : 0;
    head = (int32_t)__value;
    point = (int32_t)((__value - (float)head) * 1000.0f);

    if (head < 0)
    {
        head = -head;
    }
    if (point < 0)
    {
        point = -point;
    }

    if (flag != 0)
    {
        (void)sprintf(__str, "-%ld.%03ld", (long)head, (long)point);
    }
    else
    {
        (void)sprintf(__str, "%ld.%03ld", (long)head, (long)point);
    }
}

uint32_t Log_Get_Boot_Id(void)      { return s_boot_id; }
uint32_t Log_Get_Boot_Start(void)   { return s_boot_start; }
uint32_t Log_Get_Prev_Offset(void)  { return s_prev_offset; }
uint32_t Log_Get_Prev_Start(void)   { return s_prev_start; }
uint32_t Log_Get_Wrap_Count(void)   { return s_wrap_count; }
