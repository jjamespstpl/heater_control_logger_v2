/*
 * ds3231.c
 *
 *  Created on: Sep 4, 2024
 *      Author: Jewel James
 */
#include "main.h"
#include "ds3231.h"

extern I2C_HandleTypeDef* _ds3231_hi2c;

/* Internal funcs */
int _bcd_to_dec(uint8_t bcd);
uint8_t _dec_to_bcd(int dec);
void _ds3231_setreg(uint8_t addr, uint8_t val);
uint8_t _ds3231_getreg(uint8_t addr);

// function to set time
void ds3231_settime(DateTime* t) {
	uint8_t set_time[7];
	set_time[0] = _dec_to_bcd(t->sec);
	set_time[1] = _dec_to_bcd(t->min);
	set_time[2] = _dec_to_bcd(t->hr);
	set_time[3] = _dec_to_bcd(t->dow);
	set_time[4] = _dec_to_bcd(t->day);
	set_time[5] = _dec_to_bcd(t->month);
	set_time[6] = _dec_to_bcd(t->year);

	HAL_I2C_Mem_Write(_ds3231_hi2c, ((DS3231_ADDRESS << 1) | 1), 0x00, 1, set_time, 7, 1000);
}

void ds3231_gettime(DateTime* t) {
	uint8_t get_time[7] = {};
	HAL_I2C_Mem_Read(_ds3231_hi2c, DS3231_ADDRESS << 1, 0x00, 1, get_time, 7, 1000);
	t->sec = _bcd_to_dec(get_time[0]);
	t->min = _bcd_to_dec(get_time[1]);
	t->hr =_bcd_to_dec(get_time[2]);
	t->dow = _bcd_to_dec(get_time[3]);
	t->day = _bcd_to_dec(get_time[4]);
	t->month = _bcd_to_dec(get_time[5]);
	t->year = _bcd_to_dec(get_time[6]);
}

float ds3231_gettemp(void) {
	uint8_t temp[2] = {};

	HAL_I2C_Mem_Read(_ds3231_hi2c, DS3231_ADDRESS << 1, 0x11, 1, temp, 2, 1000);
	return ((temp[0]) + (temp[1] >> 6) / 4.0);
}

/**
 * @brief Set the byte in the designated DS3231 register to value.
 * @param addr Register address to write.
 * @param val Value to set, 0 to 255.
 */
void _ds3231_setreg(uint8_t addr, uint8_t val) {
	uint8_t bytes[2] = { addr, val };
	HAL_I2C_Master_Transmit(_ds3231_hi2c, DS3231_ADDRESS << 1, bytes, 2, DS3231_TIMEOUT);
}

/**
 * @brief Gets the byte in the designated DS3231 register.
 * @param addr Register address to read.
 * @return Value stored in the register, 0 to 255.
 */
uint8_t _ds3231_getreg(uint8_t addr) {
	uint8_t val;
	HAL_I2C_Master_Transmit(_ds3231_hi2c, DS3231_ADDRESS << 1, &addr, 1, DS3231_TIMEOUT);
	HAL_I2C_Master_Receive(_ds3231_hi2c, DS3231_ADDRESS << 1, &val, 1, DS3231_TIMEOUT);
	return val;
}

uint8_t ds3231_setalarm1(AlarmMode mode, uint8_t date, uint8_t hour, uint8_t min, uint8_t sec) {
  uint8_t alarmSecond = _dec_to_bcd(sec);
  uint8_t alarmMinute = _dec_to_bcd(min);
  uint8_t alarmHour = _dec_to_bcd(hour);
  uint8_t alarmDate = _dec_to_bcd(date);

  switch(mode) {
  case ALARM_MODE_ALL_MATCHED:
	  break;
  case ALARM_MODE_HOUR_MIN_SEC_MATCHED:
	  alarmDate |= 0x80;
	  break;
  case ALARM_MODE_MIN_SEC_MATCHED:
	  alarmDate |= 0x80;
	  alarmHour |= 0x80;
	  break;
  case ALARM_MODE_SEC_MATCHED:
	  alarmDate |= 0x80;
	  alarmHour |= 0x80;
	  alarmMinute |= 0x80;
	  break;
  case ALARM_MODE_ONCE_PER_SECOND:
	  alarmDate |= 0x80;
	  alarmHour |= 0x80;
	  alarmMinute |= 0x80;
	  alarmSecond |= 0x80;
	  break;
  default:
	  break;
  }

  /* Write Alarm Registers */
  uint8_t startAddr = DS3231_REG_ALARM1;
  uint8_t buffer[5] = {startAddr, alarmSecond, alarmMinute, alarmHour, alarmDate};
  if(HAL_I2C_Master_Transmit(_ds3231_hi2c, DS3231_ADDRESS << 1, buffer, sizeof(buffer), DS3231_TIMEOUT) != HAL_OK) return 0;

  /* Enable Alarm1 at Control Register */
  uint8_t ctrlReg = 0x00;
  ctrlReg = _ds3231_getreg(DS3231_REG_CONTROL);
  ctrlReg |= DS3231_CON_A1IE;
  ctrlReg |= DS3231_CON_INTCN;
  _ds3231_setreg(DS3231_REG_CONTROL, ctrlReg);

  return 1;
}

uint8_t ds3231_clearalarm1() {
  uint8_t ctrlReg;
  uint8_t statusReg;

  /* Clear Control Register */
  ctrlReg = _ds3231_getreg(DS3231_REG_CONTROL);
  ctrlReg &= ~DS3231_CON_A1IE;
  _ds3231_setreg(DS3231_REG_CONTROL, ctrlReg);

  /* Clear Status Register */
  statusReg = _ds3231_getreg(DS3231_REG_STATUS);
  statusReg &= ~DS3231_STA_A1F;
  _ds3231_setreg(DS3231_REG_STATUS, statusReg);

  return 1;
}

void ds3231_clearflagalarm1() {
  /* Clear Status Register */
  uint8_t statusReg = _ds3231_getreg(DS3231_REG_STATUS);
  if(statusReg & DS3231_STA_A1F) {
	  statusReg &= ~DS3231_STA_A1F;
	  _ds3231_setreg(DS3231_REG_STATUS, statusReg);
  }
}

void alarmcheck() {
	uint8_t reg = 0;
	reg = _ds3231_getreg(DS3231_REG_ALARM1);
	reg = _ds3231_getreg(DS3231_REG_ALARM1+1);
	reg = _ds3231_getreg(DS3231_REG_ALARM1+2);
	reg = _ds3231_getreg(DS3231_REG_ALARM1+3);
}

/**
 * @brief Decodes the raw binary value stored in registers to decimal format.
 * @param bin Binary-coded decimal value retrieved from register, 0 to 255.
 * @return Decoded decimal value.
 */
int _bcd_to_dec(uint8_t val) {
	return (int)( (val/16*10) + (val%16) );
}

/**
 * @brief Encodes a decimal number to binaty-coded decimal for storage in registers.
 * @param dec Decimal number to encode.
 * @return Encoded binary-coded decimal value.
 */
uint8_t _dec_to_bcd(int val) {
	return (uint8_t)((val/10*16) + (val%10) );
}

void force_temp_conv (void)
{
	uint8_t status=0;
	uint8_t control=0;
	HAL_I2C_Mem_Read(_ds3231_hi2c, DS3231_ADDRESS, 0x0F, 1, &status, 1, 100);  // read status register
	if (!(status&0x04))
	{
		HAL_I2C_Mem_Read(_ds3231_hi2c, DS3231_ADDRESS, 0x0E, 1, &control, 1, 100);  // read control register
		HAL_I2C_Mem_Write(_ds3231_hi2c, DS3231_ADDRESS, 0x0E, 1, (uint8_t *)(control|(0x20)), 1, 100);
	}
}
