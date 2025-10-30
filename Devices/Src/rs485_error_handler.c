#include "rs485_error_handler.h"

/**
 * @brief  Handle UART errors for RS485 communication.
 *         It decodes the error source and prints detailed information.
 * @param  huart: UART handle
 */
void RS485_HandleUartError(UART_HandleTypeDef *huart, uint8_t * buffer, uint16_t size)
{
    uint32_t error = HAL_UART_GetError(huart);

    // Print the error flags
    printf("\r\n[RS485 ERROR] UART instance: 0x%p\r\n", huart->Instance);

    if (error == HAL_UART_ERROR_NONE)
    {
        printf("[RS485] No UART error detected.\r\n");
        return;
    }
    
    if (error & HAL_UART_ERROR_PE)
    {
        __HAL_UART_CLEAR_PEFLAG(huart);
        printf("[RS485 ERROR] Parity Error detected.\r\n");
    }
    
    if (error & HAL_UART_ERROR_NE)
    {
        __HAL_UART_CLEAR_NEFLAG(huart);
        printf("[RS485 ERROR] Noise Error detected.\r\n");
    }
    
    if (error & HAL_UART_ERROR_FE)
    {
        __HAL_UART_CLEAR_FEFLAG(huart);
        printf("[RS485 ERROR] Frame Error detected.\r\n");
    }

    if (error & HAL_UART_ERROR_ORE)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        printf("[RS485 ERROR] Overrun Error detected.\r\n");
    }

    if (error & HAL_UART_ERROR_DMA)
    {
        printf("[RS485 ERROR] DMA Transfer Error detected.\r\n");
    }

    // Stop the current recieve
    HAL_UART_AbortReceive_IT(huart);
    HAL_UART_DMAStop(huart);
    memset(buffer, 0, size);

    // Attempt to re-enable receive interrupt to continue communication
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, size) != HAL_OK)
    {
        printf("[RS485 ERROR] Failed to restart UART receive interrupt.\r\n");
    }
    else
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        printf("[RS485] UART receive interrupt restarted.\r\n");
    }

    printf("[RS485] Error handling complete.\r\n");
}
