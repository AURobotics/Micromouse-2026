#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

//changed its name 3ashan el compiler za3lan min kanet bt clash with smth else
#define MIN_VAL(a, b) ((a) < (b) ? (a) : (b))

#define EEPROM_ADDR 0x50
#define PAGE_SIZE 8
#define EEPROM_SIZE 128

#ifdef __cplusplus
extern "C" {
#endif

bool writeCalibration(I2C_HandleTypeDef *i2c, uint16_t start_addr, const uint8_t *data, size_t len);
bool readCalibration(I2C_HandleTypeDef *i2c, uint16_t start_addr, uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif