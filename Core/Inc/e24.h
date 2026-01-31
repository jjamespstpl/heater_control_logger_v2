/*
 * e24.h
 *
 *  Created on: Jan 31, 2026
 *      Author: Jewel James
 */

#ifndef INC_E24_H_
#define INC_E24_H_

#include "main.h"
#define EE24_ADDRESS_DEFAULT 0xA0
#define EE24_SIZE      		 1
#define EE24_PSIZE      	 8

typedef struct
{
  I2C_HandleTypeDef      *HI2c;
  uint8_t                Address;
  uint8_t                Lock;
} EE24_HandleTypeDef;

uint8_t EE24_Init(EE24_HandleTypeDef *Handle, I2C_HandleTypeDef *HI2c, uint8_t I2CAddress);
uint8_t EE24_Read(EE24_HandleTypeDef *Handle, uint32_t Address, uint8_t *Data, size_t Len, uint32_t Timeout);
uint8_t EE24_Write(EE24_HandleTypeDef *Handle, uint32_t Address, uint8_t *Data, size_t Len, uint32_t Timeout);


#endif /* INC_E24_H_ */
