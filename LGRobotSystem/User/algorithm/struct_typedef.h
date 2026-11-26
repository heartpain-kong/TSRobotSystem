#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

/*
 * int8_t/int16_t/... 等定宽类型一律取自编译器自带的 <stdint.h>。
 * 不要在这里自行 typedef：arm-none-eabi 的 newlib 中 int32_t 是 long int，
 * 若这里定义成 signed int，凡是同时包含 HAL 头文件(链到 stdint)与本文件的
 * 编译单元都会报 conflicting declaration 错误。
 */
#include <stdint.h>

/* 非标准类型定义 */
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#endif
