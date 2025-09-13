/*
 * temp.c
 *
 *  Created on: Oct 19, 2024
 *      Author: Jewel James
 */

#include "temp.h"

void sensor_rx_disable() {
	TEMP1_CS(1);
	TEMP2_CS(1);
	TEMP3_CS(1);
	TEMP4_CS(1);
	TEMP5_CS(1);
	TEMP6_CS(1);
}

void sensor_rx_select(uint8_t index) {
	TEMP1_CS(1);
	TEMP2_CS(1);
	TEMP3_CS(1);
	TEMP4_CS(1);
	TEMP5_CS(1);
	TEMP6_CS(1);
	switch(index) {
	case 0:
		return;
	case 1:
		TEMP1_CS(0);
		break;
	case 2:
		TEMP5_CS(0);
		break;
	case 3:
		TEMP3_CS(0);
		break;
	case 4:
		TEMP4_CS(0);
		break;
	case 5:
		TEMP2_CS(0);
		break;
	case 6:
		TEMP6_CS(0);
		break;
	}
}
