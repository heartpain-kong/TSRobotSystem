#ifndef __CRC32_H
#define __CRC32_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief 计算 CRC32(宇树 IM6014 协议)
 * @param data 数据指针
 * @param len  字节数(必须是 4 的倍数,否则尾部不足 4 字节不参与计算)
 * @return CRC32 值(未做末尾异或,可直接与小端写在线上的 4 字节比较)
 */
uint32_t crc32_mpeg2(const uint8_t *data, size_t len);

#endif // __CRC32_H
