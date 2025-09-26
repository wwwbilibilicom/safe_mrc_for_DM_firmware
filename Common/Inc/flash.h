#ifndef __FLASH_H__
#define __FLASH_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

// For STM32H723xG (1MB Flash, single bank), sector size is 128KB.
// Use the last sector base address to store device ID persistently.
// FLASH base: 0x0800_0000, last sector base: 0x080E_0000.
#define FLASH_ID_ADDRESS ((uint32_t)0x080E0000U)

// Reserve the second-to-last sector for coil resistance (float) to avoid
// erasing the ID when updating resistance.
// Second-to-last sector base: 0x080C_0000.
#define FLASH_RES_ADDRESS ((uint32_t)0x080C0000U)

void flash_read(uint32_t address, uint8_t *data, uint32_t size);
void flash_write(uint32_t address, uint8_t *data, uint32_t size);
void flash_erase(uint32_t address, uint32_t size);
void flash_lock(void);
void flash_unlock(void);


#ifdef __cplusplus
}
#endif

#endif
