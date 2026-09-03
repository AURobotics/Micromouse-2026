#include "eeprom.h"
#include "cmsis_os.h"
#include "stdio.h"

//memory is organized as 4096/8192 words of 8-bits each fa lazem el data el ne save ne put in a uint8_t array
bool writeCalibration(I2C_HandleTypeDef *i2c, uint16_t start_addr, const uint8_t *data, size_t len) {
    size_t offset = 0;
    while (offset < len) {
        size_t space_in_page = PAGE_SIZE - ((start_addr + offset) % PAGE_SIZE);
        size_t chunk = MIN_VAL(space_in_page, len - offset);
        uint16_t mem_addr = start_addr + offset;

        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(i2c, EEPROM_ADDR << 1, mem_addr,
                           I2C_MEMADD_SIZE_16BIT,
                           (uint8_t*)(&data[offset]), chunk,
                           HAL_MAX_DELAY);
        if (status != HAL_OK) {
            uint32_t err = HAL_I2C_GetError(i2c);
            printf("I2C write failed: status=%d, error=0x%lx\n", status, err);
            return false;
        }

        osDelay(5); // tWR, self-timed write cycle, max 5ms per datasheet
        offset += chunk;
    }
    return true;
}

bool readCalibration(I2C_HandleTypeDef *i2c, uint16_t start_addr, uint8_t *data, size_t len) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(i2c, EEPROM_ADDR << 1, start_addr,
                                I2C_MEMADD_SIZE_16BIT, data, len,
                                HAL_MAX_DELAY);
    if (status != HAL_OK) {
        uint32_t err = HAL_I2C_GetError(i2c);
        printf("I2C read failed: status=%d, error=0x%lx\n", status, err);
        return false;
    }

    return true;
}
