/*
 * e24.c
 *
 *  Created on: Jan 31, 2026
 *      Author: Jewel James
 */

#include "e24.h"


void EE24_Delay(uint32_t Delay);
void EE24_Lock(EE24_HandleTypeDef *Handle);
void EE24_Unlock(EE24_HandleTypeDef *Handle);

uint8_t EE24_Init(EE24_HandleTypeDef *Handle, I2C_HandleTypeDef *HI2c, uint8_t I2CAddress) {
	uint8_t answer = 0;
	do
	{
		if ((Handle == NULL) || (HI2c == NULL))
		{
			break;
		}
		Handle->HI2c = HI2c;
		Handle->Address = I2CAddress;
		if (HAL_I2C_IsDeviceReady(Handle->HI2c, Handle->Address, 2, 100) == HAL_OK)
		{
			answer = 1;
		}
	}
	while (0);
	return answer;
}

uint8_t EE24_Read(EE24_HandleTypeDef *Handle, uint32_t Address, uint8_t *Data, size_t Len, uint32_t Timeout)
{
	  EE24_Lock(Handle);
	  uint8_t answer = 0;
	  do
	  {
		if (HAL_I2C_Mem_Read(Handle->HI2c, Handle->Address | ((Address & 0x0100) >> 7), (Address & 0xff), I2C_MEMADD_SIZE_8BIT, Data, Len, Timeout) == HAL_OK)
	    {
	      answer = 1;
	    }
	  }
	  while (0);
	  EE24_Unlock(Handle);
	  return answer;
}

uint8_t EE24_Write(EE24_HandleTypeDef *Handle, uint32_t Address, uint8_t *Data, size_t Len, uint32_t Timeout)
{
	EE24_Lock(Handle);
	uint8_t answer = 0;
	do
	{
		uint16_t w;
		uint32_t startTime = HAL_GetTick();
		while (1)
		{
			w = EE24_PSIZE - (Address  % EE24_PSIZE);
			if (w > Len)
			{
				w = Len;
			}
			if (HAL_I2C_Mem_Write(Handle->HI2c, Handle->Address | ((Address & 0x0100) >> 7), (Address & 0xff), I2C_MEMADD_SIZE_8BIT, Data, w, Timeout) == HAL_OK)
			{
				EE24_Delay(10);
				Len -= w;
				Data += w;
				Address += w;
				if (Len == 0)
				{
					answer = 1;
					break;
				}
				if (HAL_GetTick() - startTime >= Timeout)
				{
					break;
				}
			}
			else
			{
				break;
			}
		}
	}
	while (0);
	EE24_Unlock(Handle);
	return answer;
}

void EE24_Delay(uint32_t Delay)
{
	HAL_Delay(Delay);
}

void EE24_Lock(EE24_HandleTypeDef *Handle)
{
	while (Handle->Lock)
	{
		EE24_Delay(1);
	}
	Handle->Lock = 1;
}


void EE24_Unlock(EE24_HandleTypeDef *Handle)
{
	Handle->Lock = 0;
}
