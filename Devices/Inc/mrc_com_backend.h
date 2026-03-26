/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#ifndef __MRC_COM_BACKEND_H__
#define __MRC_COM_BACKEND_H__

/**
 * @brief  Compile-time physical communication backend selector.
 *
 * The safeMRC board selects RS485 or Classic CAN via a 0-ohm resistor.
 * Change this single define to match the hardware configuration.
 *
 *   MRC_COM_RS485 — USART2/RS485 DMA+IDLE path (original)
 *   MRC_COM_CAN   — FDCAN1 Classic CAN path (1 Mbps, standard 11-bit IDs)
 */

#define MRC_COM_RS485  0
#define MRC_COM_CAN    1

/* >>>>>> EDIT THIS LINE to switch backend <<<<<< */
#define MRC_COM_BACKEND  MRC_COM_CAN

#endif /* __MRC_COM_BACKEND_H__ */
