/*
 * DS3231.h
 *
 *  Created on: Nov 15, 2024
 *      Author: ajayb
 */

#ifndef INC_DS3231_H_
#define INC_DS3231_H_

#include "main.h"

void DS3231_SetTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
void DS3231_ReadTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds);
void DS3231_Init(void);

#endif /* INC_DS3231_H_ */
