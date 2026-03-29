/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include "can_com.h"
#include <string.h>
#include <stdio.h>

/* INT32 clamp helpers */
#define INT16_CLAMP(x) \
    (((x) > 32767) ? 32767 : (((x) < -32768) ? -32768 : (int16_t)(x)))

/* ---------------------------------------------------------------------------
 * CAN_Com_Init
 * --------------------------------------------------------------------------*/
int CAN_Com_Init(MRC_Can_Com_t *can_com, hcan_t *hfdcan, uint8_t id)
{
    if (can_com == NULL || hfdcan == NULL) {
        return -1;
    }

    memset(can_com, 0, sizeof(MRC_Can_Com_t));

    can_com->id     = id;
    can_com->hfdcan = hfdcan;

    /* Pre-fill fixed feedback fields */
    can_com->fbk_msg.mode           = (uint8_t)FREE;
    can_com->fbk_msg.collision_flag = 0;
    can_com->fbk_msg.encoder_value  = 0;
    can_com->fbk_msg.present_current = 0;

    return 0;
}

/* ---------------------------------------------------------------------------
 * CAN_Com_UnpackCmd
 *
 * Called from fdcan1_rx_callback() (ISR context) after fdcanx_receive().
 * CAN hardware has already verified frame integrity (CRC-15 + ACK), so no
 * software CRC is computed here.
 * --------------------------------------------------------------------------*/
int CAN_Com_UnpackCmd(MRC_Can_Com_t *can_com,
                      uint16_t       rx_id,
                      const uint8_t  rx_data[8])
{
    if (can_com == NULL || rx_data == NULL) {
        return -1;
    }

    /* Check whether this frame is addressed to this device */
    if (rx_id != CAN_CMD_ID(can_com->id)) {
        return -1;
    }

    /* Copy payload into cmd_msg struct */
    memcpy(&can_com->cmd_msg, rx_data, sizeof(CAN_Cmd_t));

    can_com->cmd_correct = 1;
    getFreq(&can_com->freq_calculator);

    return 0;
}

/* ---------------------------------------------------------------------------
 * CAN_Com_PackFbk
 * --------------------------------------------------------------------------*/
int CAN_Com_PackFbk(MRC_Can_Com_t *can_com,
                    MRC_Mode       mode,
                    int32_t        encoder_value,
                    int32_t        present_current,
                    uint8_t        collision_flag)
{
    if (can_com == NULL) {
        return -1;
    }

    can_com->fbk_msg.mode            = (uint8_t)mode;
    can_com->fbk_msg.collision_flag  = collision_flag;
    can_com->fbk_msg.encoder_value   = encoder_value;
    can_com->fbk_msg.present_current = INT16_CLAMP(present_current);

    return 0;
}

/* ---------------------------------------------------------------------------
 * CAN_Com_SendFbk
 *
 * Builds the TX header directly with FDCAN_CLASSIC_CAN and FDCAN_BRS_OFF.
 * Note: fdcanx_send_data() in bsp_fdcan.c also handles Classic CAN correctly
 * (it checks hfdcan->Init.FrameFormat), but we keep this dedicated function
 * for clarity and to avoid coupling with the generic BSP send path.
 * --------------------------------------------------------------------------*/
int CAN_Com_SendFbk(MRC_Can_Com_t *can_com)
{
    if (can_com == NULL || can_com->hfdcan == NULL) {
        return -1;
    }

    FDCAN_TxHeaderTypeDef tx_header = {0};
    tx_header.Identifier          = CAN_FBK_ID(can_com->id);
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.DataLength          = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_OFF;        /* Classic CAN: no BRS */
    tx_header.FDFormat            = FDCAN_CLASSIC_CAN;    /* NOT FDCAN_FD_CAN   */
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(can_com->hfdcan,
                                      &tx_header,
                                      (uint8_t *)&can_com->fbk_msg) != HAL_OK)
    {
        printf("CAN_Com_SendFbk: TX FIFO full or error\n");
        return -1;
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * CAN_Com_Reset
 * --------------------------------------------------------------------------*/
void CAN_Com_Reset(MRC_Can_Com_t *can_com)
{
    if (can_com == NULL) {
        return;
    }

    can_com->RxFlag      = 0;
    can_com->cmd_correct = 0;
    memset(&can_com->cmd_msg, 0, sizeof(CAN_Cmd_t));
}
