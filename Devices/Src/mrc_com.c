/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include "mrc_com.h"
#include "crc_ccitt.h"
#include <string.h>
#include "stdio.h"

// Protocol constant definitions
#define MRC_HEAD_HIGH    0xFE
#define MRC_HEAD_LOW     0xEE

/**
 * @brief Initialize MRC communication module
 * @param mrc_com: MRC communication structure pointer
 * @param huart: UART handle pointer
 * @param id: Device ID
 * @return 0: Success, -1: Failure
 */
int MRC_Com_Init(MRC_Com_t *mrc_com, UART_HandleTypeDef *huart, uint8_t id)
{
    if (mrc_com == NULL || huart == NULL) {
        return -1;
    }
    
    // Initialize structure
    memset(mrc_com, 0, sizeof(MRC_Com_t));
    mrc_com->id = id;
    mrc_com->mrc_huart = huart;
    mrc_com->RxFlag = 0;
    mrc_com->RxLen = sizeof(MRC_Cmd_Protocol);
    mrc_com->TxLen = sizeof(MRC_Fbk_Protocol);
    mrc_com->cmd_correct = 0;
    
    // Clear DMA receive structure (since DMA receives directly into cmd_msg)
    memset((uint8_t*)&mrc_com->cmd_msg, 0, mrc_com->RxLen);
    memset(mrc_com->cmd_msg_buffer, 0, MRC_CMD_MSG_BUFFER_SIZE);
    
    // Initialize feedback protocol structure
    mrc_com->fbk_msg.head[0] = MRC_HEAD_HIGH;
    mrc_com->fbk_msg.head[1] = MRC_HEAD_LOW;
    mrc_com->fbk_msg.id = id;
    mrc_com->fbk_msg.mode = FREE;
    mrc_com->fbk_msg.collision_flag = 0;
    mrc_com->fbk_msg.encoder_value = 0;
    mrc_com->fbk_msg.present_current = 0;
    mrc_com->fbk_msg.CRC16Data = 0;
    
    // Start DMA reception for command messages
    HAL_UARTEx_ReceiveToIdle_DMA(mrc_com->mrc_huart, mrc_com->cmd_msg_buffer, MRC_CMD_MSG_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(mrc_com->mrc_huart->hdmarx, DMA_IT_HT);
    
    return 0;
}

void MRC_Com_Tx_Enable(void)
{
    HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_SET);
}

void MRC_Com_Rx_Enable(void)
{
    HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET);
}


/**
 * @brief Unpack command data from DMA buffer (device side)
 * @param mrc_com: MRC communication structure pointer
 * @return 0: Success, -1: Failure
 */
int MRC_Com_UnpackCmd(MRC_Com_t *mrc_com)
{
    getFreq(&mrc_com->freq_calculator);
    if (mrc_com == NULL) {
        return -1;
    }
    // Verify CRC checksum
    uint16_t received_crc = mrc_com->cmd_msg.CRC16Data;
    uint16_t calculated_crc = crc_ccitt(0xFFFF, (uint8_t*)&mrc_com->cmd_msg, mrc_com->RxLen - 2);
    // printf("received_crc: %x, calculated_crc: %x\n", received_crc, calculated_crc);
    if(received_crc == calculated_crc)
    {
        mrc_com->cmd_correct = 1;
    }
    else
    {
        mrc_com->cmd_correct = 0;
        memset((uint8_t*)&mrc_com->cmd_msg, 0, mrc_com->RxLen);
        printf("CRC mismatch!\n");
        return -1;
    }
    
    return 0;
}

/**
 * @brief Pack feedback data for transmission (device side)
 * @param mrc_com: MRC communication structure pointer
 * @param mode: Current work mode
 * @param encoder_value: Encoder angle value
 * @param present_current: Current torque value
 * @param collision_flag: Collision flag
 * @return 0: Success, -1: Failure
 */
int MRC_Com_PackFbk(MRC_Com_t *mrc_com, MRC_Mode mode, int32_t encoder_value, int32_t encoder_velocity,
    int32_t present_current, uint8_t collision_flag)
{
    if (mrc_com == NULL) {
        return -1;
    }
    
    // Update feedback message content
    mrc_com->fbk_msg.head[0] = MRC_HEAD_HIGH;
    mrc_com->fbk_msg.head[1] = MRC_HEAD_LOW;
    mrc_com->fbk_msg.id = mrc_com->id;
    mrc_com->fbk_msg.mode = mode;
    mrc_com->fbk_msg.encoder_value = encoder_value;
    mrc_com->fbk_msg.encoder_velocity = encoder_velocity;
    mrc_com->fbk_msg.present_current = present_current;
    mrc_com->fbk_msg.collision_flag = collision_flag;
    
    // Calculate and add CRC checksum
    mrc_com->fbk_msg.CRC16Data = crc_ccitt(0xFFFF, (uint8_t*)&mrc_com->fbk_msg, mrc_com->TxLen - 2);
    
    return 0;
}

/**
 * @brief Send feedback data using DMA (device side response)
 * @param mrc_com: MRC communication structure pointer
 * @return 0: Success, -1: Failure
 */
int MRC_Com_SendFbk(MRC_Com_t *mrc_com)
{
    if (mrc_com == NULL || mrc_com->mrc_huart == NULL) {
        return -1;
    }
    
    // Send feedback data using DMA
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(mrc_com->mrc_huart, 
                                                    (uint8_t*)&mrc_com->fbk_msg, 
                                                    mrc_com->TxLen);
    
    return (status == HAL_OK) ? 0 : -1;
}


/**
 * @brief Get command correctness flag
 * @param mrc_com: MRC communication structure pointer
 * @return 1: Command correct, 0: Command error
 */
int MRC_Com_IsCmdCorrect(MRC_Com_t *mrc_com)
{
    if (mrc_com == NULL) {
        return 0;
    }
    
    return mrc_com->cmd_correct;
}

/**
 * @brief Reset communication status
 * @param mrc_com: MRC communication structure pointer
 */
void MRC_Com_Reset(MRC_Com_t *mrc_com)
{
    if (mrc_com == NULL) {
        return;
    }
    
    mrc_com->RxFlag = 0;
    mrc_com->cmd_correct = 0;
    
    // Clear the receive buffer
    memset((uint8_t*)&mrc_com->cmd_msg, 0, mrc_com->RxLen);
    
    // Restart ReceiveToIdle DMA reception
    HAL_UARTEx_ReceiveToIdle_DMA(mrc_com->mrc_huart, mrc_com->cmd_msg_buffer, MRC_CMD_MSG_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(mrc_com->mrc_huart->hdmarx, DMA_IT_HT);
}

