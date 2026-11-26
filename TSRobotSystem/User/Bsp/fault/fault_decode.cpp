//
// fault_decode.cpp — HardFault 现场解码(自动判断栈帧在 PSP 还是 MSP 上)
//

#include "fault_decode.h"

#include "main.h"       /* g_fault_psp / g_fault_msp / SCB */

/* ============================================================================
 * 记录量(调试器直接看)
 * ========================================================================== */

volatile uint32_t g_fault_pc = 0u;
volatile uint32_t g_fault_frame_sp = 0u;
volatile bool     g_fault_frame_ok = false;
volatile bool     g_fault_frame_on_msp = false;

/* ============================================================================
 * 工具
 * ========================================================================== */

/* 只在这几个 RAM 窗口里读,避免"解码故障现场时二次触发总线错误"✗ */
static bool Safe_To_Read(uint32_t __addr)
{
    return ((__addr >= 0x20000000u) && (__addr < 0x20020000u)) ||     /* DTCM */
           ((__addr >= 0x24000000u) && (__addr < 0x24080000u)) ||     /* AXI/D1 */
           ((__addr >= 0x30000000u) && (__addr < 0x30048000u)) ||     /* D2 */
           ((__addr >= 0x38000000u) && (__addr < 0x38004000u));       /* D3(SRAM4) */
}

/* 一个"异常栈帧"= r0 r1 r2 r3 r12 LR PC xPSR ⇒ PC 是下标 6,xPSR 是下标 7 */
static bool Frame_Valid(const uint32_t *__frame)
{
    uint32_t pc;
    uint32_t xpsr;
    uint32_t lr;

    if (!Safe_To_Read((uint32_t)__frame))
    {
        return false;
    }

    pc = __frame[6];
    xpsr = __frame[7];
    lr = __frame[5];

    /* ① PC 必须在内部 Flash(0x08000000 ~ 0x08200000) */
    if ((pc < 0x08000000u) || (pc >= 0x08200000u))
    {
        return false;
    }
    /* ② 必须是 Thumb 指令地址(偶数) */
    if ((pc & 1u) != 0u)
    {
        return false;
    }
    /* ③ xPSR 的 T 位(bit9)必须为 1 —— 异常帧里恒为 1,数据残渣基本不会满足 */
    if ((xpsr & (1u << 9)) == 0u)
    {
        return false;
    }
    /* ④ LR 要么是代码地址,要么是 EXC_RETURN(0xFFFFFFFx) */
    if (((lr < 0x08000000u) || (lr >= 0x08200000u)) && ((lr & 0xFFFFFF00u) != 0xFFFFFF00u))
    {
        return false;
    }
    return true;
}

/* ============================================================================
 * 对外接口
 * ========================================================================== */

void Fault_Capture(void)
{
    uint32_t exc_return;
    __asm volatile ("mov %0, lr" : "=r" (exc_return));
    uint32_t base;
    uint32_t off;
    bool on_msp;

    g_fault_pc = 0u;
    g_fault_frame_sp = 0u;
    g_fault_frame_ok = false;
    g_fault_frame_on_msp = false;

    /* EXC_RETURN bit2: 0 = 被中断用的是 MSP,1 = PSP */
    if ((exc_return & 0xFFFFFF00u) == 0xFFFFFF00u)
    {
        on_msp = ((exc_return & 0x4u) == 0u);
    }
    else
    {
        on_msp = true;   /* 异常返回码不可信时先按 MSP 找 */
    }

    base = on_msp ? __get_MSP() : __get_PSP();

    for (off = 0u; off < 512u; off += 4u)
    {
        uint32_t cand = base + off;
        if (!Safe_To_Read(cand))
        {
            break;
        }
        if (Frame_Valid((const uint32_t *)cand))
        {
            g_fault_pc = ((const uint32_t *)cand)[6];
            g_fault_pc_psp = g_fault_pc;
            g_fault_frame_sp = cand;
            g_fault_frame_ok = true;
            g_fault_frame_on_msp = on_msp;
            return;
        }
    }
}

void Fault_Decode(void)
{
    uint32_t psp = g_fault_psp;
    uint32_t msp = g_fault_msp;
    uint32_t off;

    g_fault_pc = 0u;
    g_fault_frame_sp = 0u;
    g_fault_frame_ok = false;
    g_fault_frame_on_msp = false;

    /* ① 先试 PSP:任务上下文被打断时,帧在任务栈顶上 */
    if (Frame_Valid((const uint32_t *)psp))
    {
        g_fault_pc = ((const uint32_t *)psp)[6];
        g_fault_frame_sp = psp;
        g_fault_frame_ok = true;
        return;
    }

    /* ② 再试 MSP:中断里跑飞时帧在 MSP 上。
          注意 g_fault_msp 是"进了 C handler 之后"的 MSP ⇒ 帧在它**上面**(地址更大),
          所以从 msp 往高地址扫(最多 512 字节,够覆盖 handler 自己的压栈)✓ */
    for (off = 0u; off < 512u; off += 4u)
    {
        uint32_t cand = msp + off;

        if (!Safe_To_Read(cand))
        {
            break;
        }
        if (Frame_Valid((const uint32_t *)cand))
        {
            g_fault_pc = ((const uint32_t *)cand)[6];
            g_fault_frame_sp = cand;
            g_fault_frame_ok = true;
            g_fault_frame_on_msp = true;
            return;
        }
    }

    /* ③ 两个都没有:SP 可能已经被踩坏;绝不能再读非法 PSP,否则二次 HardFault */
    if (Safe_To_Read(psp))
    {
        g_fault_pc = ((const uint32_t *)psp)[6];
        g_fault_frame_sp = psp;
    }
    else
    {
        g_fault_pc = 0u;
        g_fault_frame_sp = 0u;
    }
}
