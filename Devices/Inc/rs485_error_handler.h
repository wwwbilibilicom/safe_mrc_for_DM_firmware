#ifndef __RS485_ERROR_HANDLER_H__
#define __RS485_ERROR_HANDLER_H__

#include "main.h"
#include <stdio.h>
#include <string.h>

/**
 * @brief  Handle UART error events.
 * @param  huart: Pointer to UART handle structure.
 * @note   This function should be called inside HAL_UART_ErrorCallback().
 */
void RS485_HandleUartError(UART_HandleTypeDef *huart, uint8_t * buffer, uint16_t size);

#endif /* __RS485_ERROR_HANDLER_H__ */
