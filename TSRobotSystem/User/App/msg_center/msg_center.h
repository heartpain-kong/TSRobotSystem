/* ============================================================================
 * msg_center.h — 任务间通信:环形数组(每话题一个环形数组,静态、无 malloc)
 * 说明 / 用法 / 坑 见同目录 msg_center.md
 * ========================================================================== */

#ifndef MSG_CENTER_H
#define MSG_CENTER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MSG_RING_DEPTH      4u      /* 每个话题保留最近几条(环形数组深度) */
#define MSG_SLOT_SIZE       96u     /* 单条消息最大字节数(超了拒收并计数) */

/**
 * @brief 话题表(与 E:\rm\流程图.jpg 的数据流一一对应)
 */
typedef enum
{
    MSG_TOPIC_RC = 0,       /* 遥控器/键鼠  → cmd       : Remote_Data_t */
    MSG_TOPIC_MINIPC,         /* miniPC(上位机转发) → cmd : 控制指令 */
    MSG_TOPIC_REFEREE,      /* 裁判系统      → cmd/robot  : 比赛状态/血量/热量 */
    MSG_TOPIC_CMD,          /* cmd           → 各任务     : 控制指令 */
    MSG_TOPIC_IMU_CHASSIS,  /* 底盘 imu      → chassis/gimbal */
    MSG_TOPIC_IMU_GIMBAL,   /* 云台 imu      → gimbal    */
    MSG_TOPIC_CHASSIS,      /* chassis       → gimbal/highRise/error */
    MSG_TOPIC_GIMBAL,       /* gimbal        → highRise/error */
    MSG_TOPIC_HIGHRISE,     /* highRise      → error */
    MSG_TOPIC_MOTOR,        /* 电机/设备状态 → error */
    MSG_TOPIC_ROBOT,        /* robot         → chassis/gimbal/highRise : 决策层门控 */
    MSG_TOPIC_COUNT
} Enum_Msg_Topic;

/* ---------------- 生产者 ---------------- */

/**
 * @brief 清空所有话题(上电初始化用)
 */
void Msg_Init(void);

/**
 * @brief 发布一条消息(写进该话题的环形数组)
 * @param __topic 话题
 * @param __data  数据
 * @param __len   长度(<= MSG_SLOT_SIZE,超了拒收)
 * @return true = 已写入;false = 参数错/超长
 * @note 不阻塞:环形满了会**覆盖最旧的**(广播型话题这是正常的)
 */
bool Msg_Publish(Enum_Msg_Topic __topic, const void *__data, uint16_t __len);

/* ---------------- 消费者 ----------------
   广播型(一个话题多个消费者,例如 cmd 要给 chassis/gimbal/highRise 都看):
   每个消费者自己记一个 seq,用 Msg_Is_New() 判断有没有新数据,互不影响 ✓ */

/**
 * @brief 取该话题**最新**一条(不弹出,谁都能读)
 * @param __topic    话题
 * @param __out      输出缓冲
 * @param __out_size 缓冲大小(通常 sizeof(结构体))
 * @param __seq      输出:这条消息的序号(可传给 Msg_Is_New)
 * @return true = 有数据
 */
bool Msg_Latest(Enum_Msg_Topic __topic, void *__out, uint16_t __out_size, uint32_t *__seq);

/**
 * @brief 有没有比 __last_seq 更新的数据
 * @return true = 有新数据
 */
bool Msg_Is_New(Enum_Msg_Topic __topic, uint32_t __last_seq);

/**
 * @brief 取该话题**最旧**一条并弹出(独占型消费者用,例如记录/日志)
 * @return true = 取到了
 */
bool Msg_Pop(Enum_Msg_Topic __topic, void *__out, uint16_t __out_size);

/* ---------------- 诊断 ---------------- */

/**
 * @brief 最新消息序号(发布一条 +1)
 */
uint32_t Msg_Get_Seq(Enum_Msg_Topic __topic);

/**
 * @brief 累计发布条数
 */
uint32_t Msg_Get_Published(Enum_Msg_Topic __topic);

/**
 * @brief 被覆盖掉的历史条数(广播话题里增长是正常的;超长拒收也会记在这里)
 */
uint32_t Msg_Get_Overwritten(Enum_Msg_Topic __topic);

/**
 * @brief 话题名(日志/调试用)
 */
const char *Msg_Get_Topic_Name(Enum_Msg_Topic __topic);

#ifdef __cplusplus
}
#endif

#endif // MSG_CENTER_H
