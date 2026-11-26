/* ============================================================================
 * fault_decode.h — HardFault 现场解码(自动判断栈帧在 PSP 还是 MSP 上)
 *
 * 为什么需要:
 *   Cortex-M 异常压栈用的是"被中断那一层的 SP":
 *     · 任务上下文被打断 ⇒ 帧在 PSP 上
 *     · 中断/Handler 里跑飞 ⇒ 帧在 MSP 上
 *   ⇒ 只用 PSP 取 PC 时,中断里跑飞会拿到栈上的**垃圾**(比如某次 atanf 的残留返回地址),
 *     看起来像代码地址,其实是数据 ✗ —— 我们就踩过这个坑。
 *
 * 用法:在 HardFault_Handler 的 USER CODE 0 里调用 Fault_Decode(); 然后看 g_fault_pc。
 * ========================================================================== */

#ifndef FAULT_DECODE_H
#define FAULT_DECODE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t g_fault_pc;        /* ★ 真正出错指令的地址(已校正,可直接 addr2line) */
extern volatile uint32_t g_fault_frame_sp;  /* 找到可信栈帧的那个 SP(PSP 或 MSP) */
extern volatile bool     g_fault_frame_ok;  /* true = 找到了可信帧;false = 两个都没有(SP 可能被踩坏) */
extern volatile bool     g_fault_frame_on_msp; /* true = 帧在 MSP 上 ⇒ 中断里跑飞 */

/**
 * @brief 在 HardFault_Handler 里调用:挑出可信的异常栈帧并记下 PC
 * @note 只读 RAM 窗口内的地址,不会二次触发总线错误 ✓
 */
void Fault_Decode(void);

/**
 * @brief HardFault 入口立即抓原始异常帧(不调用 Fault_Decode,避免二次异常)
 * @note 先按 EXC_RETURN 判断 MSP/PSP,再在 512B 内找 PC 合法的帧
 */
void Fault_Capture(void);

#ifdef __cplusplus
}
#endif

#endif // FAULT_DECODE_H
