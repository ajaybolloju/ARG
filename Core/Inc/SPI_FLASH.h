#include "main.h"

void W25QXX_WriteEnable(void);
uint8_t W25QXX_ReadStatus(void);
void W25QXX_WriteData(uint32_t address, uint8_t* data, uint16_t size);
void W25QXX_ReadData(uint32_t address, uint8_t* buffer, uint16_t size);

