//
// can3_protocol.cpp — CAN3 8 字节帧编解码
//
// 经典 CAN 帧只有 8 字节,应用层不再加 CRC;
// CAN 控制器自身有 15 位 CRC,足够链路检错。
//

#include "can3_protocol.h"

#include <string.h>

static void PutI16(uint8_t *__p, int16_t __v)
{
    __p[0] = (uint8_t)((uint16_t)__v & 0xFFu);
    __p[1] = (uint8_t)(((uint16_t)__v >> 8) & 0xFFu);
}

static int16_t GetI16(const uint8_t *__p)
{
    return (int16_t)((uint16_t)__p[0] | ((uint16_t)__p[1] << 8));
}

int Can3_EncodeCmd(const Can3_Cmd *__in, uint8_t __out[8])
{
    if ((__in == 0) || (__out == 0)) return -1;
    memset(__out, 0, 8);
    __out[0] = __in->seq;
    __out[1] = __in->mode;
    PutI16(&__out[2], __in->vx_mm_s);
    PutI16(&__out[4], __in->vy_mm_s);
    PutI16(&__out[6], __in->w_mrad_s);
    return 0;
}

int Can3_DecodeCmd(const uint8_t __in[8], Can3_Cmd *__out)
{
    if ((__in == 0) || (__out == 0)) return -1;
    __out->seq = __in[0];
    __out->mode = __in[1];
    __out->vx_mm_s = GetI16(&__in[2]);
    __out->vy_mm_s = GetI16(&__in[4]);
    __out->w_mrad_s = GetI16(&__in[6]);
    return 0;
}

int Can3_EncodeFeedback(const Can3_Feedback *__in, uint8_t __out[8])
{
    if ((__in == 0) || (__out == 0)) return -1;
    memset(__out, 0, 8);
    __out[0] = __in->seq;
    __out[1] = __in->status;
    PutI16(&__out[2], __in->vx_mm_s);
    PutI16(&__out[4], __in->vy_mm_s);
    PutI16(&__out[6], __in->w_mrad_s);
    return 0;
}

int Can3_DecodeFeedback(const uint8_t __in[8], Can3_Feedback *__out)
{
    if ((__in == 0) || (__out == 0)) return -1;
    __out->seq = __in[0];
    __out->status = __in[1];
    __out->vx_mm_s = GetI16(&__in[2]);
    __out->vy_mm_s = GetI16(&__in[4]);
    __out->w_mrad_s = GetI16(&__in[6]);
    return 0;
}

int Can3_EncodeAux(const Can3_AuxCmd *__in, uint8_t __out[8])
{
    if ((__in == 0) || (__out == 0)) return -1;
    memset(__out, 0, 8);
    __out[0] = __in->seq;
    __out[1] = __in->flags;
    PutI16(&__out[2], __in->yaw_cdeg);
    PutI16(&__out[4], __in->trigger_rpm);
    PutI16(&__out[6], __in->reserved);
    return 0;
}

int Can3_DecodeAux(const uint8_t __in[8], Can3_AuxCmd *__out)
{
    if ((__in == 0) || (__out == 0)) return -1;
    __out->seq = __in[0];
    __out->flags = __in[1];
    __out->yaw_cdeg = GetI16(&__in[2]);
    __out->trigger_rpm = GetI16(&__in[4]);
    __out->reserved = GetI16(&__in[6]);
    return 0;
}

int Can3_EncodeStatus(const Can3_Status *__in, uint8_t __out[8])
{
    if ((__in == 0) || (__out == 0)) return -1;
    memset(__out, 0, 8);
    __out[0] = __in->seq;
    __out[1] = __in->status;
    PutI16(&__out[2], __in->yaw_cdeg);
    PutI16(&__out[4], __in->power_w);
    PutI16(&__out[6], __in->fault);
    return 0;
}

int Can3_DecodeStatus(const uint8_t __in[8], Can3_Status *__out)
{
    if ((__in == 0) || (__out == 0)) return -1;
    __out->seq = __in[0];
    __out->status = __in[1];
    __out->yaw_cdeg = GetI16(&__in[2]);
    __out->power_w = GetI16(&__in[4]);
    __out->fault = GetI16(&__in[6]);
    return 0;
}
