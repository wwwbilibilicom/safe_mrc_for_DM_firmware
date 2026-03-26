/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#ifndef __CAN_COM_H__
#define __CAN_COM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "fdcan.h"
#include "bsp_fdcan.h"
#include "mrc_protocol.h"
#include "sys_clock.h"

/* ---------------------------------------------------------------------------
 * CAN ID scheme  (11-bit standard IDs)
 *
 *   Bits [10:8] = direction prefix
 *   Bits  [7:0] = device ID (1–254)
 *
 *   Host → Device (command):  0x100 | device_id   e.g. 0x101 for id=1
 *   Device → Host (feedback): 0x200 | device_id   e.g. 0x201 for id=1
 * --------------------------------------------------------------------------*/
#define CAN_CMD_ID(device_id)  ((uint16_t)(0x100U | ((uint8_t)(device_id))))
#define CAN_FBK_ID(device_id)  ((uint16_t)(0x200U | ((uint8_t)(device_id))))

/* ---------------------------------------------------------------------------
 * Command frame payload  (Host → Device, DLC = 8 bytes)
 *
 *  Byte   Field               Type      Scale
 *  -----  ------------------  --------  ---------------------------
 *  0      mode                uint8_t   MRC_Mode enum value
 *  1      reserved            uint8_t   0x00
 *  2–5    des_coil_current    int32_t   mA  (value × 1000), little-endian
 *  6–7    reserved            uint8_t   0x00
 *
 *  No header bytes (device address is in CAN ID).
 *  No software CRC (CAN hardware CRC-15 + ACK provides frame integrity).
 * --------------------------------------------------------------------------*/
#pragma pack(1)
typedef struct {
    uint8_t  mode;              /* MRC_Mode enum cast to uint8 */
    uint8_t  reserved;          /* padding / future flags, host sets to 0 */
    int32_t  des_coil_current;  /* desired current in mA (× 1000) */
    uint8_t  reserved2[2];      /* padding to reach 8 bytes */
} CAN_Cmd_t;                    /* exactly 8 bytes */
#pragma pack()

/* ---------------------------------------------------------------------------
 * Feedback frame payload  (Device → Host, DLC = 8 bytes)
 *
 *  Byte   Field               Type      Scale
 *  -----  ------------------  --------  ---------------------------
 *  0      mode                uint8_t   MRC_Mode enum value
 *  1      collision_flag      uint8_t   0x00 = safe, 0x01 = collision
 *  2–5    encoder_value       int32_t   CurrentEncoderValRad × 65535, little-endian
 *  6–7    present_current     int16_t   mA (value × 1000), little-endian
 *
 *  encoder_velocity is omitted vs RS485 (saves 4 bytes).
 *  Host can derive velocity from consecutive position samples at 1 kHz.
 *  All retained fields use identical scaling to the RS485 protocol.
 * --------------------------------------------------------------------------*/
#pragma pack(1)
typedef struct {
    uint8_t  mode;              /* MRC_Mode enum cast to uint8 */
    uint8_t  collision_flag;    /* 0x00 = safe, 0x01 = collision */
    int32_t  encoder_value;     /* encoder position: rad × 65535 */
    int16_t  present_current;   /* coil current: mA (× 1000) */
} CAN_Fbk_t;                    /* exactly 8 bytes */
#pragma pack()

/* ---------------------------------------------------------------------------
 * Per-device CAN communication state
 * --------------------------------------------------------------------------*/
#pragma pack(1)
typedef struct {
    uint8_t          id;               /* device address (1–254) */
    hcan_t          *hfdcan;           /* FDCAN peripheral handle */

    CAN_Cmd_t        cmd_msg;          /* last received command payload */
    CAN_Fbk_t        fbk_msg;          /* feedback being built for TX */

    uint8_t          RxFlag;           /* set by ISR when a new command arrives */
    uint8_t          cmd_correct;      /* 1 = CAN ID matched this device */

    Caculate_Freq_t  freq_calculator;  /* receive-rate measurement */
    uint64_t         rx_time;          /* high-res RX timestamp (ns) */
    uint64_t         tx_time;          /* high-res TX timestamp (ns) */
    float            time_delay;       /* TX–RX latency (µs) */
} MRC_Can_Com_t;
#pragma pack()

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------*/

/**
 * @brief  Initialise CAN communication state for one device node.
 * @param  can_com  Pointer to MRC_Can_Com_t.
 * @param  hfdcan   Pointer to HAL FDCAN handle (e.g. &hfdcan1).
 * @param  id       Device address (1–254).
 * @return 0 on success, -1 on null pointer.
 */
int CAN_Com_Init(MRC_Can_Com_t *can_com, hcan_t *hfdcan, uint8_t id);

/**
 * @brief  Validate and unpack an incoming CAN command frame.
 *         Call from fdcan1_rx_callback() after fdcanx_receive().
 * @param  can_com  Pointer to MRC_Can_Com_t.
 * @param  rx_id    11-bit CAN ID from fdcanx_receive().
 * @param  rx_data  8-byte payload from fdcanx_receive().
 * @return 0 if the frame is addressed to this device, -1 otherwise.
 */
int CAN_Com_UnpackCmd(MRC_Can_Com_t *can_com,
                      uint16_t       rx_id,
                      const uint8_t  rx_data[8]);

/**
 * @brief  Pack feedback fields into can_com->fbk_msg ready for transmission.
 * @param  can_com          Pointer to MRC_Can_Com_t.
 * @param  mode             Current MRC_Mode.
 * @param  encoder_value    Encoder position: rad × 65535 (int32).
 * @param  present_current  Coil current × 1000 (int32, clamped to int16).
 * @param  collision_flag   0 = safe, 1 = collision.
 * @return 0 on success, -1 on null pointer.
 */
int CAN_Com_PackFbk(MRC_Can_Com_t *can_com,
                    MRC_Mode       mode,
                    int32_t        encoder_value,
                    int32_t        present_current,
                    uint8_t        collision_flag);

/**
 * @brief  Transmit the packed feedback frame via FDCAN1 (Classic CAN).
 * @param  can_com  Pointer to MRC_Can_Com_t.
 * @return 0 on HAL_OK, -1 on transmit error (e.g. TX FIFO full).
 */
int CAN_Com_SendFbk(MRC_Can_Com_t *can_com);

/**
 * @brief  Reset communication state (clear flags, zero cmd_msg).
 * @param  can_com  Pointer to MRC_Can_Com_t.
 */
void CAN_Com_Reset(MRC_Can_Com_t *can_com);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_COM_H__ */
