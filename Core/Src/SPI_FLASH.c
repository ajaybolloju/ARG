#include "SPI_FLASH.h"

extern SPI_HandleTypeDef hspi1;

#define FLASH_CS_LOW()  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
#define FLASH_CS_HIGH() HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)


void W25QXX_WriteEnable(void) {
    uint8_t cmd = 0x06; // Write Enable
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS LOW
    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    FLASH_CS_HIGH();
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS HIGH
}

uint8_t W25QXX_ReadStatus(void) {
    uint8_t cmd = 0x05;  // Read Status Register
    uint8_t status = 0;
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS LOW
    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &status, 1, HAL_MAX_DELAY);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS HIGH
    FLASH_CS_HIGH();
    return status;
}

void W25QXX_WriteData(uint32_t address, uint8_t* data, uint16_t size) {
    W25QXX_WriteEnable();  // Enable writing
    uint8_t cmd[4] = { 0x02,  // Page Program Command
                       (address >> 16) & 0xFF,  // Address bytes
                       (address >> 8) & 0xFF,
                       address & 0xFF };
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS LOW
    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);      // Send command & address
    HAL_SPI_Transmit(&hspi1, data, size, HAL_MAX_DELAY);  // Send data
    FLASH_CS_HIGH();
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS HIGH
    while (W25QXX_ReadStatus() & 0x01);                   // Wait for write to finish
}

void W25QXX_ReadData(uint32_t address, uint8_t* buffer, uint16_t size) {
    uint8_t cmd[4] = { 0x03,  // Read Data Command
                       (address >> 16) & 0xFF,
                       (address >> 8) & 0xFF,
                       address & 0xFF };
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS LOW
    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);      // Send command & address
    HAL_SPI_Receive(&hspi1, buffer, size, HAL_MAX_DELAY); // Receive data
    FLASH_CS_HIGH();
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS HIGH
}
