#include "rs485.h"

void RS485_Tx_Enable(void)
{
    HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_SET);
}

void RS485_Rx_Enable(void)
{
    HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET);
}
