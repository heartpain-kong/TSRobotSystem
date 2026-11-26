//
// alg_math_convert.cpp — 定点/浮点换算 + 字节序
//   三个换算函数的实现**原样**从 motor_mit.cpp 搬来(公式一个字符都没改)。
//

#include "alg_math_convert.h"

fp32 uint_to_float(uint16_t x_int, fp32 x_min, fp32 x_max, uint8_t bits)
{
    fp32 span = x_max - x_min;
    fp32 offset = x_min;

    return ((fp32)x_int) * span / ((fp32)((1 << bits) - 1)) + offset;
}

uint16_t float_to_uint_f(fp32 x, fp32 x_min, fp32 x_max, uint8_t bits)
{
    fp32 span = x_max - x_min;
    fp32 offset = x_min;

    return (uint16_t)((x - offset) * ((fp32)((1 << bits) - 1)) / span);
}

uint16_t float_to_uint_i(fp32 x, uint16_t x_min, uint16_t x_max, uint8_t bits)
{
    fp32 span = x_max - x_min;
    fp32 offset = x_min;

    return (uint16_t)((x - offset) * ((fp32)((1 << bits) - 1)) / span);
}

uint16_t Math_Endian_Reverse_16(void *Source, void *Destination)
{
    uint8_t *tmp_address_8;
    uint16_t tmp_value_16;

    tmp_address_8 = (uint8_t *)Source;
    tmp_value_16 = tmp_address_8[0] << 8 | tmp_address_8[1];

    if (Destination != nullptr)
    {
        uint8_t *tmp_source, *tmp_destination;
        tmp_source = (uint8_t *)Source;
        tmp_destination = (uint8_t *)Destination;
        tmp_destination[0] = tmp_source[1];
        tmp_destination[1] = tmp_source[0];
    }

    return (tmp_value_16);
}