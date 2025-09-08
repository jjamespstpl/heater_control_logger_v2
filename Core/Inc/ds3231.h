/*
 * ds3231.h
 *
 *  Created on: Sep 4, 2024
 *      Author: Jewel James
 */

#ifndef INC_DS3231_H_
#define INC_DS3231_H_

#include "main.h"

#define DS3231_ADDRESS	 		0x68

#define DS3231_REG_TIME         0x00
#define DS3231_REG_ALARM1       0x07
#define DS3231_REG_ALARM2       0x0B
#define DS3231_REG_CONTROL      0x0E
#define DS3231_REG_STATUS       0x0F
#define DS3231_REG_TEMP         0x11

#define DS3231_REG_SECOND 		0x00
#define DS3231_REG_MINUTE 		0x01
#define DS3231_REG_HOUR  		0x02
#define DS3231_REG_DOW    		0x03
#define DS3231_REG_ALARM2		0x0B
#define DS3231_REG_DATE   		0x04
#define DS3231_REG_MONTH  		0x05
#define DS3231_REG_YEAR   		0x06

#define DS3231_CON_EOSC         0x80
#define DS3231_CON_BBSQW        0x40
#define DS3231_CON_CONV         0x20
#define DS3231_CON_RS2          0x10
#define DS3231_CON_RS1          0x08
#define DS3231_CON_INTCN        0x04
#define DS3231_CON_A2IE         0x02
#define DS3231_CON_A1IE         0x01

#define DS3231_STA_OSF          0x80
#define DS3231_STA_32KHZ        0x08
#define DS3231_STA_BSY          0x04
#define DS3231_STA_A2F          0x02
#define DS3231_STA_A1F          0x01

#define DS3231_CENTURY 			7

#define DS3231_TEMP_MSB		0x11
#define DS3231_TEMP_LSB		0x12

#define DS3231_TIMEOUT		2000

typedef struct DateTime {
	uint8_t day;
	uint8_t month;
	uint8_t dow;
	uint8_t year;
	uint8_t hr;
	uint8_t min;
	uint8_t sec;
} DateTime;

typedef enum {
  ALARM_MODE_ALL_MATCHED = 0,
  ALARM_MODE_HOUR_MIN_SEC_MATCHED,
  ALARM_MODE_MIN_SEC_MATCHED,
  ALARM_MODE_SEC_MATCHED,
  ALARM_MODE_ONCE_PER_SECOND
} AlarmMode;

/**
 * @defgroup	DS3231_USER_FUNCTIONS
 * @brief		Functions for the user to define
 * @{
 */
void ds3231_i2c_tx(uint8_t* buff, uint8_t size);
void ds3231_i2c_rx(uint8_t* buff, uint8_t size);
/**
 * }
 */

/**
 * @defgroup	DS3231_LIB_FUNCTIONS
 * @brief		Main library functions
 * @{
 */
void ds3231_init();
void ds3231_settime(DateTime* dt);
void ds3231_gettime(DateTime* dt);
uint8_t ds3231_setalarm1(AlarmMode mode, uint8_t date, uint8_t hour, uint8_t min, uint8_t sec);
uint8_t ds3231_clearalarm1(void);
void ds3231_clearflagalarm1(void);
float ds3231_gettemp();
void alarmcheck();
void force_temp_conv(void);
/**
 * }
 */

#endif /* INC_DS3231_H_ */
