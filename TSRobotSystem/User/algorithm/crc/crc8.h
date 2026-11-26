/* ============================================================================
 * crc8.h — CRC8(poly 0x31, init 0xFF),裁判系统 / 上位机协议共用
 * 说明见 crc.md
 * ========================================================================== */

#ifndef CRC8_H
#define CRC8_H

#include <stdint.h>

/**
 * @brief CRC8:多项式 0x31,初值 0xFF(裁判系统协议规定的那个)
 * @param __data 数据
 * @param __len  长度
 * @return CRC8 值
 * @note 逐位算、不用表:帧率只有几十~几百 Hz,开销可忽略,而且头文件里零状态
 */
static inline uint8_t crc8(const uint8_t *__data, uint16_t __len)
{
    uint8_t crc = 0xFFu;

    for (uint16_t i = 0u; i < __len; i++)
    {
        crc ^= __data[i];
        for (uint8_t bit = 0u; bit < 8u; bit++)
        {
            crc = ((crc & 0x80u) != 0u) ? (uint8_t)((uint8_t)(crc << 1) ^ 0x31u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

#endif // CRC8_H
