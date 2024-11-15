/*
 * DS3231.c
 *
 *  Created on: Nov 15, 2024
 *      Author: ajayb
 */
#include "DS3231.h"

#define DS3231_ADDRESS 0xD0  // 7-bit I2C address of DS3231 (0x68 shifted left by 1)


extern I2C_HandleTypeDef hi2c1;

uint8_t BCDToDec(uint8_t val) { return ((val / 16 * 10) + (val % 16)); }
uint8_t DecToBCD(uint8_t val) { return ((val / 10 * 16) + (val % 10)); }


void DS3231_Init(void)
{
	  uint8_t controlReg = 0;
	  HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x0E, 1, &controlReg, 1, HAL_MAX_DELAY);
	  controlReg &= ~(1 << 7); // Clear the "Oscillator Stop Flag" (OSF) bit, if set
	  HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, 0x0E, 1, &controlReg, 1, HAL_MAX_DELAY);
}

// Function to set time (HH, MM, SS)
void DS3231_SetTime(uint8_t hours, uint8_t minutes, uint8_t seconds) {
    uint8_t buffer[7];

    buffer[0] = ((seconds / 10) << 4) | (seconds % 10); // Convert to BCD
    buffer[1] = ((minutes / 10) << 4) | (minutes % 10);
    buffer[2] = ((hours / 10) << 4) | (hours % 10);
    buffer[3] = DecToBCD(0);
    buffer[4] = DecToBCD(0);
    buffer[5] = DecToBCD(0);
    buffer[6] = DecToBCD(0);

    HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, 0x00, 1, buffer, 7, HAL_MAX_DELAY);
}

// Function to read time


//uint8_t readByte(uint8_t reg) {
//    uint8_t data;
//    HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, reg, 1, &data, 1, HAL_MAX_DELAY);
//    return data;
//}
//
//void DS3231_ReadTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds) {
//    uint8_t raw_seconds = readByte(0x00);
//    uint8_t raw_minutes = readByte(0x01);
//    uint8_t raw_hours   = readByte(0x02);
//
//    printf("Raw Seconds: 0x%X, Raw Minutes: 0x%X, Raw Hours: 0x%X\n", raw_seconds, raw_minutes, raw_hours);
//
//    *seconds = BCDToDec(raw_seconds);
//    *minutes = BCDToDec(raw_minutes);
//    *hours   = BCDToDec(raw_hours);
//
//    printf("Converted Time: %02d:%02d:%02d\n", *hours, *minutes, *seconds);
//}

void DS3231_ReadTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds) {
    uint8_t buffer[3];
    for (int retry = 0; retry < 3; retry++) {
        HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x00, 1, buffer, 3, HAL_MAX_DELAY);

        *seconds = BCDToDec(buffer[0]);
        *minutes = BCDToDec(buffer[1]);
        *hours   = BCDToDec(buffer[2]);

        if (*hours != 0 || *minutes != 0 || *seconds != 0) {
            // Successfully read non-zero time
            break;
        }
    }
}

