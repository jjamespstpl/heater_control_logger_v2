/*
 * temp.h
 *
 *  Created on: Oct 19, 2024
 *      Author: Jewel James
 */

#ifndef INC_TEMP_H_
#define INC_TEMP_H_

#define SENSOR_COUNT 2

#include "main.h"

#define TEMP1_CS(SET_OR_RESET)	(SET_OR_RESET ? (GPIOB->BSRR = GPIO_PIN_11) : (GPIOB->BRR = GPIO_PIN_11))		// PB11
#define TEMP2_CS(SET_OR_RESET)	(SET_OR_RESET ? (GPIOB->BSRR = GPIO_PIN_12) : (GPIOB->BRR = GPIO_PIN_12))		// PB11
#define TEMP3_CS(SET_OR_RESET)	(SET_OR_RESET ? (GPIOB->BSRR = GPIO_PIN_13) : (GPIOB->BRR = GPIO_PIN_13))		// PB11
#define TEMP4_CS(SET_OR_RESET)	(SET_OR_RESET ? (GPIOB->BSRR = GPIO_PIN_14) : (GPIOB->BRR = GPIO_PIN_14))		// PB11
#define TEMP5_CS(SET_OR_RESET)	(SET_OR_RESET ? (GPIOB->BSRR = GPIO_PIN_15) : (GPIOB->BRR = GPIO_PIN_15))		// PB11
#define TEMP6_CS(SET_OR_RESET)	(SET_OR_RESET ? (GPIOA->BSRR = GPIO_PIN_8) : (GPIOA->BRR = GPIO_PIN_8))			// PA8

void sensor_rx_select(uint8_t);
void sensor_rx_disable();

#endif /* INC_TEMP_H_ */
