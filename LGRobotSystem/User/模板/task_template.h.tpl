/* ============================================================================
 * xxx_task.h — 一句话说清这是什么任务(照 User/模板/task_template.h.tpl 改)
 * 说明 / 用法 / 坑 见同目录 xxx_task.md
 * ★ 用法:复制到 User/App/<模块>/ → 去掉 .tpl 后缀 → 全文替换 xxx/XXX/Xxx
 * ========================================================================== */

#ifndef XXX_TASK_H
#define XXX_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 可配置宏(★ = 需要按实车改的)
 * ========================================================================== */
#define XXX_MAX_VALUE           1.0f        /* ★ 按实车改 */

/**
 * @brief 发布在 MSG_TOPIC_XXX 的数据(单位一律 SI:rad / rad·s⁻¹ / m / m·s⁻¹ / N·m)
 */
typedef struct
{
    float Value;                /* 主输出 */
    uint8_t Mode;               /* 当前模式(需要就加) */
    bool Online;                /* 就绪 */
    uint32_t Seq;               /* 每拍 +1 */
} Xxx_Data_t;

/**
 * @brief 初始化(清状态;设备初始化放到 Robot_Init 里做)
 */
void Xxx_Init(void);

/**
 * @brief 跑一拍(由 XXX 任务按 XXX_TASK_PERIOD_MS 周期调用)
 */
void Xxx_Task(void);

/**
 * @brief 读数据(只读,调试/上层直接取用)
 */
const Xxx_Data_t *Xxx_Get(void);

/**
 * @brief 是否就绪
 */
bool Xxx_Is_Online(void);

#ifdef __cplusplus
}
#endif

#endif // XXX_TASK_H
