#include "crc32.h"

uint32_t crc32_mpeg2(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    const uint32_t polynomial = 0x04C11DB7u;

    if (data == nullptr) return crc;

    /* ★ 官方就是 len/4 组,不足 4 字节的尾巴不计入 */
    const size_t word_count = len / 4u;

    for (size_t i = 0; i < word_count; ++i)
    {
        /* 小端内存里取出来拼成大端 word(与官方 ((uint32_t)data[i*4+3]<<24)|... 一致) */
        uint32_t word = ((uint32_t)data[i * 4u + 3u] << 24)
                      | ((uint32_t)data[i * 4u + 2u] << 16)
                      | ((uint32_t)data[i * 4u + 1u] << 8)
                      | ((uint32_t)data[i * 4u + 0u]);

        uint32_t xbit = 0x80000000u;
        for (int bit = 0; bit < 32; ++bit)
        {
            uint32_t msb_crc = crc & 0x80000000u;
            crc <<= 1;
            if (msb_crc)
                crc ^= polynomial;
            if (word & xbit)
                crc ^= polynomial;
            xbit >>= 1;
        }
    }

    return crc;
}
