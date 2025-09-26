/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include "mrc_debugcli.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "drv_mrc.h"
#include "mrc_statemachine.h"
#include "flash.h"

char usart1_rx_buffer[USART1_RX_BUFFER_SIZE];
volatile uint16_t usart1_rx_index = 0;

// 安全范围定义
#define DEBUGCLI_VOLTAGE_MIN   -12.0f
#define DEBUGCLI_VOLTAGE_MAX    12.0f
#define DEBUGCLI_CURRENT_MIN   -5.0f
#define DEBUGCLI_CURRENT_MAX    5.0f
#define DEBUGCLI_ID_MIN         0
#define DEBUGCLI_ID_MAX         15

void MRC_DebugCLI_Parse(const char *cmd_str, Device_MRC_t *mrc) {
    if (!cmd_str || !mrc || strlen(cmd_str) == 0) return;
    char cmd[16] = {0};
    char arg1[16] = {0};
    char arg2[16] = {0};
    float value = 0.0f;
    int n = sscanf(cmd_str, "%15s %15s %15s", cmd, arg1, arg2);
    if (n < 1) {
        printf("[USART1] Invalid command.\n");
        return;
    }
    for (int i = 0; cmd[i]; ++i) cmd[i] = toupper(cmd[i]);
    for (int i = 0; arg1[i]; ++i) arg1[i] = toupper(arg1[i]);
    for (int i = 0; arg2[i]; ++i) arg2[i] = toupper(arg2[i]);

    // Help menu (Linux style)
    if (strcmp(cmd, "-H") == 0 || strcmp(cmd, "--HELP") == 0) {
        MRC_DebugCLI_Print_Help_Menu();
        return;
    }

    // DEBUG ON/OFF commands
    if (strcmp(cmd, "DEBUG") == 0) {
        if (strcmp(arg1, "ON") == 0) {
            // MRC_StateMachine_SetMode(&mrc->statemachine, DEBUG, mrc->filtered_coil_current);
            MRC_SetMode(mrc, DEBUG);
            printf("[USART1] DEBUG mode enabled.\n");
        } else if (strcmp(arg1, "OFF") == 0) {
            // MRC_StateMachine_SetMode(&mrc->statemachine, FIX_LIMIT, mrc->filtered_coil_current);
            MRC_SetMode(mrc, FIX_LIMIT);
            mrc->control_mode = MRC_CURRENT_CONTROL;
            printf("[USART1] DEBUG mode disabled. Now in FIX_LIMIT mode.\n");
        } else {
            printf("[USART1] Unknown DEBUG command: %s\n", arg1);
        }
        return;
    }

    // Only allow other commands in DEBUG mode
    if (mrc->statemachine.current_mode != DEBUG) {
        const char *mode_str = "UNKNOWN";
        switch (mrc->statemachine.current_mode) {
            case FREE: mode_str = "FREE"; break;
            case FIX_LIMIT: mode_str = "FIX_LIMIT"; break;
            case ADAPTATION: mode_str = "ADAPTATION"; break;
            case DEBUG: mode_str = "DEBUG"; break;
            default: break;
        }
        printf("[USART1] DEBUG mode is not enabled! Current mode: %s\n", mode_str);
        return;
    }

    if (n < 2) {
        printf("[USART1] Invalid command.\n");
        return;
    }

    if (strcmp(cmd, "MODE") == 0) {
        if (strcmp(arg1, "VOLTAGE") == 0) {
            if (!MRC_StateMachine_CanSetVoltage(&mrc->statemachine)) {
                printf("[USART1] Cannot switch to VOLTAGE mode in current state.\n");
                return;
            }
            mrc->control_mode = MRC_VOLTAGE_CONTROL;
            printf("[USART1] Switched to VOLTAGE control mode.\n");
        } else if (strcmp(arg1, "CURRENT") == 0) {
            if (!MRC_StateMachine_CanSetCurrent(&mrc->statemachine)) {
                printf("[USART1] Cannot switch to CURRENT mode in current state.\n");
                return;
            }
            mrc->control_mode = MRC_CURRENT_CONTROL;
            printf("[USART1] Switched to CURRENT control mode.\n");
        } else {
            printf("[USART1] Unknown mode: %s\n", arg1);
        }
    } else if (strcmp(cmd, "ID") == 0) {
        if (strcmp(arg1, "CHECK") == 0) {
            printf("[USART1] Current Device ID: %u\n", (unsigned)mrc->com.id);
        } else if (strcmp(arg1, "CHANGE") == 0) {
            if (n < 3) {
                printf("[USART1] ID CHANGE requires a value. Usage: ID CHANGE <0-15>\n");
                return;
            }
            // Validate natural number
            for (int i = 0; arg2[i]; ++i) {
                if (!isdigit((unsigned char)arg2[i])) {
                    printf("[USART1] Invalid ID: must be an integer between %d and %d.\n", DEBUGCLI_ID_MIN, DEBUGCLI_ID_MAX);
                    return;
                }
            }
            long new_id = strtol(arg2, NULL, 10);
            if (new_id < DEBUGCLI_ID_MIN || new_id > DEBUGCLI_ID_MAX) {
                printf("[USART1] ID out of range. Allowed range: %d~%d.\n", DEBUGCLI_ID_MIN, DEBUGCLI_ID_MAX);
                return;
            }
            uint8_t old_id = mrc->com.id;
            mrc->com.id = (uint8_t)new_id;
            // Also update feedback packet ID immediately
            mrc->com.fbk_msg.id = (uint8_t)new_id;
            // Persist to flash
            flash_erase(FLASH_ID_ADDRESS, 1U);
            uint8_t id_byte = (uint8_t)new_id;
            flash_write(FLASH_ID_ADDRESS, &id_byte, 1U);
            printf("[USART1] Device ID changed: %u -> %u (saved)\n", (unsigned)old_id, (unsigned)mrc->com.id);
        } else {
            printf("[USART1] Unknown ID command. Use: ID CHECK | ID CHANGE <0-15>\n");
        }
    } else if (strcmp(cmd, "SET") == 0) {
        if (n < 3) {
            printf("[USART1] SET command missing value.\n");
            return;
        }
        value = atof(arg2);
        if (strcmp(arg1, "VOLTAGE") == 0) {
            if (!MRC_StateMachine_CanSetVoltage(&mrc->statemachine) || mrc->control_mode != MRC_VOLTAGE_CONTROL) {
                printf("[USART1] Please switch to VOLTAGE control mode first using: MODE VOLTAGE\n");
                return;
            }
            if (value < DEBUGCLI_VOLTAGE_MIN || value > DEBUGCLI_VOLTAGE_MAX) {
                printf("[USART1] Voltage out of range (%.1f~%.1f V)!\n", DEBUGCLI_VOLTAGE_MIN, DEBUGCLI_VOLTAGE_MAX);
                return;
            }
            mrc->VNH7040.des_voltage = value;
            MRC_set_voltage(mrc);
            printf("[USART1] Set target voltage: %.3f V\n", value);
        } else if (strcmp(arg1, "CURRENT") == 0) {
            if (!MRC_StateMachine_CanSetCurrent(&mrc->statemachine) || mrc->control_mode != MRC_CURRENT_CONTROL) {
                printf("[USART1] Please switch to CURRENT control mode first using: MODE CURRENT\n");
                return;
            }
            if (value < DEBUGCLI_CURRENT_MIN || value > DEBUGCLI_CURRENT_MAX) {
                printf("[USART1] Current out of range (%.1f~%.1f A)!\n", DEBUGCLI_CURRENT_MIN, DEBUGCLI_CURRENT_MAX);
                return;
            }
            mrc->des_coil_current = value;
            printf("[USART1] Set target current: %.3f A\n", value);
        } else {
            printf("[USART1] Unknown SET arg: %s\n", arg1);
        }
    } else if (strcmp(cmd, "RES") == 0) {
        if (strcmp(arg1, "CHECK") == 0) {
            printf("[USART1] Coil resistance: %.4f Ohm\n", mrc->coil_resistance);
        } else if (strcmp(arg1, "CHANGE") == 0) {
            if (n < 3) {
                printf("[USART1] RES CHANGE requires a value. Usage: RES CHANGE <ohm>\n");
                return;
            }
            float new_res = strtof(arg2, NULL);
            if (!(new_res > 0.01f && new_res < 1000.0f)) {
                printf("[USART1] Invalid resistance value. Allowed: 0.01~1000.0 Ohm\n");
                return;
            }
            float old_res = mrc->coil_resistance;
            mrc->coil_resistance = new_res;
            // Persist as float (4 bytes) to dedicated sector
            flash_erase(FLASH_RES_ADDRESS, 4U);
            flash_write(FLASH_RES_ADDRESS, (uint8_t*)&new_res, sizeof(float));
            printf("[USART1] Coil resistance changed: %.4f -> %.4f Ohm (saved)\n", old_res, new_res);
        } else {
            printf("[USART1] Unknown RES command. Use: RES CHECK | RES CHANGE <ohm>\n");
        }
    } else {
        printf("[USART1] Unknown command: %s\n", cmd);
    }
    
}

void MRC_DebugCLI_Print_Help_Menu(void)
{
    printf("Usage: <COMMAND> [ARGS]\n");
    printf("\n");
    printf("General:\n");
    printf("  -h, --help                Show this help and exit\n");
    printf("  DEBUG ON|OFF              Enable/disable DEBUG work mode\n");
    printf("\n");
    printf("Mode:\n");
    printf("  MODE VOLTAGE              Switch to voltage control mode\n");
    printf("  MODE CURRENT              Switch to current control mode\n");
    printf("\n");
    printf("Set targets (DEBUG mode only):\n");
    printf("  SET VOLTAGE <v>           Set target voltage [%f, %f] V\n", DEBUGCLI_VOLTAGE_MIN, DEBUGCLI_VOLTAGE_MAX);
    printf("  SET CURRENT <i>           Set target current [-%f, %f] A\n", DEBUGCLI_CURRENT_MAX, DEBUGCLI_CURRENT_MAX);
    printf("\n");
    printf("Device ID:\n");
    printf("  ID CHECK                  Print current device ID\n");
    printf("  ID CHANGE <n>             Set device ID, natural number [0, 15] (persist)\n");
    printf("\n");
    printf("Coil resistance:\n");
    printf("  RES CHECK                 Print coil resistance (Ohm)\n");
    printf("  RES CHANGE <ohm>          Set coil resistance (0.01 ~ 10.0 Ohm, persist)\n");
}

void MRC_DebugCLI_Init(UART_HandleTypeDef *huart) {
    // Start DMA reception for USART1 CLI
    memset(usart1_rx_buffer, 0, USART1_RX_BUFFER_SIZE);
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    //__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE); //使能接收中断
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); // Enable idle interrupt
    HAL_UART_Receive_DMA(huart, (uint8_t*)usart1_rx_buffer, USART1_RX_BUFFER_SIZE);
    printf("[USART1] Debug CLI initialized. Type '--help' or '-h' for available commands.\n");
    //MRC_DebugCLI_Print_Help_Menu();
}

void MRC_DebugCLI_UART_IdleHandler(UART_HandleTypeDef *huart) {
    extern Device_MRC_t MRC;
    // Clear idle flag
    __HAL_UART_CLEAR_IDLEFLAG(huart); 
    // Stop DMA
    HAL_UART_DMAStop(huart);
    // Calculate received length
    uint16_t len = USART1_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (len > 0 && len < USART1_RX_BUFFER_SIZE) {
        usart1_rx_buffer[len] = '\0';
        MRC_DebugCLI_Parse((const char*)usart1_rx_buffer, &MRC);
    }
    // Restart DMA reception
    memset(usart1_rx_buffer, 0, USART1_RX_BUFFER_SIZE);
    HAL_UART_Receive_DMA(huart, (uint8_t*)usart1_rx_buffer, USART1_RX_BUFFER_SIZE);

}

