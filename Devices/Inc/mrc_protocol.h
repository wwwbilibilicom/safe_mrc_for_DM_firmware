/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#ifndef __MRC_PROTOCOL_H__
#define __MRC_PROTOCOL_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef enum MRC_Mode
{
    FREE,
    FIX_LIMIT,
    ADAPTATION,
    DEBUG,
    MRC_RESET,
    ZERO,
    REFRESH
} MRC_Mode;

#pragma pack(1)
    typedef struct MRC_Cmd_Protocol     //message buffer typedef for the command protocol
    {
        uint8_t head[2];                // 2 bytes 0xFE, 0xEE
        uint8_t id;                     // 1 byte
        MRC_Mode mode;                  // 1 byte  (requires -fshort-enums)
        int32_t des_coil_current;       // 4 bytes, mA (value × 1000), little-endian
        uint16_t CRC16Data;             // 2 bytes
    } MRC_Cmd_Protocol;                 // 10 bytes total
#pragma pack()

#pragma pack(1)
    typedef struct MRC_Fbk_Protocol //message buffer typedef for the feedback protocol
    {
        uint8_t head[2];            // 2 bytes
        uint8_t id;                 // 1 byte
        MRC_Mode mode;              // 1 byte  (requires -fshort-enums)
        uint8_t collision_flag;     // 1 byte  0x00: safely, 0x01: collision happened.
        int32_t encoder_value;      // 4 bytes, rad × 65535, little-endian
        int32_t encoder_velocity;   // 4 bytes, rad/s × 1000, little-endian
        int16_t present_current;    // 2 bytes, mA × 1000, little-endian
        uint16_t CRC16Data;         // 2 bytes
    } MRC_Fbk_Protocol;             // 17 bytes total
#pragma pack()

#ifdef __cplusplus
}
#endif

#endif /* __MRC_PROTOCOL_H__ */
