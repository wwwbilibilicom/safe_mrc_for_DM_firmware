#include "flash.h"
#include <string.h>

#ifndef FLASHWORD_SIZE
#define FLASHWORD_SIZE           32U              // 256-bit programming unit
#endif

static uint32_t align_down(uint32_t addr, uint32_t align)
{
    return addr - (addr % align);
}

static uint32_t get_bank(uint32_t address)
{
#if defined(FLASH_BANK_2) && defined(FLASH_BANK2_BASE)
    return (address >= FLASH_BANK2_BASE) ? FLASH_BANK_2 : FLASH_BANK_1;
#else
    (void)address;
    return FLASH_BANK_1;
#endif
}

static uint32_t get_bank_base(uint32_t bank)
{
#if defined(FLASH_BANK_2) && defined(FLASH_BANK2_BASE)
    return (bank == FLASH_BANK_2) ? FLASH_BANK2_BASE : FLASH_BASE;
#else
    (void)bank;
    return FLASH_BASE;
#endif
}

static uint32_t get_sector_index(uint32_t address)
{
    uint32_t bank = get_bank(address);
    uint32_t base = get_bank_base(bank);
    return (address - base) / FLASH_SECTOR_SIZE;
}
void flash_unlock(void)
{
    HAL_FLASH_Unlock();
}

void flash_lock(void)
{
    HAL_FLASH_Lock();
}

void flash_read(uint32_t address, uint8_t *data, uint32_t size)
{
    if (data == NULL || size == 0U) {
        return;
    }
    memcpy(data, (const void *)address, size);
}

void flash_erase(uint32_t address, uint32_t size)
{
    if (size == 0U) {
        return;
    }

    uint32_t start = align_down(address, FLASH_SECTOR_SIZE);
    uint32_t end = align_down(address + size - 1U, FLASH_SECTOR_SIZE);

    while (start <= end) {
        uint32_t bank = get_bank(start);
        uint32_t sector = get_sector_index(start);

        FLASH_EraseInitTypeDef erase = {0};
        erase.TypeErase = FLASH_TYPEERASE_SECTORS;
        erase.Banks = bank;
        erase.Sector = sector;
        erase.NbSectors = 1U;

        uint32_t sector_error = 0U;
        HAL_FLASH_Unlock();
        HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &sector_error);
        HAL_FLASH_Lock();
        (void)st; // optional: user can add error handling/logging

        start += FLASH_SECTOR_SIZE;
    }
}

void flash_write(uint32_t address, uint8_t *data, uint32_t size)
{
    if (data == NULL || size == 0U) {
        return;
    }

    // H7 requires 256-bit (32-byte) flashword programming aligned to 32 bytes
    uint32_t write_addr = align_down(address, FLASHWORD_SIZE);
    uint32_t end_addr = address + size;

    // Temporary RAM buffer for one flashword (must be word-aligned)
    uint8_t buf[FLASHWORD_SIZE];

    HAL_FLASH_Unlock();

    while (write_addr < end_addr) {
        // Prepare 32-byte block
        memset(buf, 0xFF, sizeof(buf));

        for (uint32_t i = 0; i < FLASHWORD_SIZE; ++i) {
            uint32_t src_addr = write_addr + i;
            if (src_addr >= address && src_addr < end_addr) {
                buf[i] = data[src_addr - address];
            } else {
                // keep 0xFF for bytes outside requested range
            }
        }

        // Program one flashword
        HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, write_addr, (uint32_t)buf);
        (void)st; // optional: add error handling if needed

        // Optional verify
        // if (memcmp((const void *)write_addr, buf, FLASHWORD_SIZE) != 0) { /* handle error */ }

        write_addr += FLASHWORD_SIZE;
    }

    HAL_FLASH_Lock();
}


