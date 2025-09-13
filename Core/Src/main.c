/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "temp.h"
#include "ds3231.h"
#include "../../ECUAL/I2C_LCD/I2C_LCD.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GSM_WAIT_TIME_LOW		500
#define GSM_WAIT_TIME_MED		10000
#define GSM_WAIT_TIME_HIGH		20000

#define MODE_OFF	0x00
#define MODE_ON		0x01
#define MODE_CTRL	0x02

/*###*/
#define ACS37800_I2C_ADDR				(127)
#define ACS37800_REG_VIRMS				(0x20) /* IRMSAVGONESEC / VRMSAVGONESEC */
#define ACS37800_REG_PACTAVGONEMIN		(0x22) /* LSW */
#define ACS37800_REG_SLADDR				(0x0F) /* LSW */
#define ACS37800_CURR_SENS_RANGE		(30) /* ACS37800KMACTR-030B3-I2C is a 30.0A part */

uint8_t acs37800_vi_buffer[4];
uint8_t acs37800_p_buffer[4];
uint16_t pavg;
float vrms_final = 0;
float irms_final = 0;
float pavg_final = 0;
/*###*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define BTN1_READ()		(HAL_GPIO_ReadPin(BTN1_IN_GPIO_Port, BTN1_IN_Pin))
#define BTN2_READ()		(HAL_GPIO_ReadPin(BTN2_IN_GPIO_Port, BTN2_IN_Pin))
#define BTN3_READ()		(HAL_GPIO_ReadPin(BTN3_IN_GPIO_Port, BTN3_IN_Pin))

#define TRIAC1_SET(SET_OR_RESET) (HAL_GPIO_WritePin(TRIAC1_GPIO_Port, TRIAC1_Pin, SET_OR_RESET))
#define TRIAC2_SET(SET_OR_RESET) (HAL_GPIO_WritePin(TRIAC2_GPIO_Port, TRIAC2_Pin, SET_OR_RESET))

#define TRIAC_TRIGGER_TIME    10 /* 100us, 10ms total time for TRIAC to be on */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
uint32_t baudRate;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM16_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/** LCD **/
#define LCD1 I2C_LCD_1

/* Button reading */
#define LED_BLINK_TIME	1000
#define LED_ON()		(HAL_GPIO_WritePin(UP_LED_GPIO_Port, UP_LED_Pin, GPIO_PIN_SET))
#define LED_OFF()		(HAL_GPIO_WritePin(UP_LED_GPIO_Port, UP_LED_Pin, GPIO_PIN_RESET))

uint8_t btn1_flag, btn2_flag, btn3_flag;
uint16_t btn1_timer, btn2_timer, btn3_timer, led_blink_timer;
uint8_t btn1_stat;
uint8_t btn2_stat;
uint8_t btn3_stat;
uint8_t led_blink_flag;

void led_blink() {
	LED_ON();
	led_blink_flag = 1;
}

uint8_t btn1_read(uint8_t is_long) {
	if(BTN3_READ() == 0) {
		btn1_flag = 1;
		if(btn1_flag && btn1_timer > (is_long ? 10000: 1100)) {
			btn1_timer = 0;
			return 1;
		}
	}
	else btn1_flag = 0;
	return 0;
}
uint8_t btn2_read() {
	if(BTN2_READ() == 0) {
		btn2_flag = 1;
		if(btn2_flag && btn2_timer > 1100) {
			btn2_timer = 0;
			return 1;
		}
	}
	else btn2_flag = 0;
	return 0;
}
uint8_t btn3_read(uint8_t is_long) {
	if(BTN1_READ() == 0) {
		btn3_flag = 1;
		if(btn3_flag && btn3_timer > (is_long ? 10000: 1100)) {
			btn3_timer = 0;
			return 1;
		}
	}
	else btn3_flag = 0;
	return 0;
}


I2C_HandleTypeDef* _ds3231_hi2c = &hi2c1;
uint8_t sensor_idx;
uint8_t sensor_refresh_flag;
uint32_t us;
uint32_t ms;
uint8_t sec;
uint32_t min;
uint16_t hr;

/* ADC stuff */
/* ADC data points */
#define ADC_CHANNEL_COUNT    3 /* TODO change this */
#define ADC_SAMPLE_COUNT     500 /* TODO change this */
float adc_arr[ADC_CHANNEL_COUNT];
float adc_conv_fact[ADC_CHANNEL_COUNT] = { 0.108675, 0.001932, 0 }; /* TODO add current and temp values */
uint16_t adc_raw[ADC_CHANNEL_COUNT];

float kwh;

typedef enum adc_params {
	VOLT,
	CUR,
	TEMP,
} adc_param;

uint8_t period = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	static uint32_t adc_avg[ADC_CHANNEL_COUNT] = {};
	static uint16_t sample_count = 0;
	if(sample_count >= ADC_SAMPLE_COUNT) {
		for(uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
			adc_avg[i] = adc_avg[i] / ADC_SAMPLE_COUNT;
			if(adc_avg[i] < 200) adc_avg[i] = 0;
			adc_arr[i] = (float)adc_avg[i] * adc_conv_fact[i];
			adc_avg[i] = 0;
		}
		sample_count = 0;
	}
	else {
		for(uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
			adc_avg[i] += adc_raw[i];
		}
		sample_count++;
	}
}

uint16_t triac_trigger_timer;
uint8_t triac_temp_ctrl;
uint8_t triac_trigger_flag;

/* GPIO EXTI */
uint8_t triac_timer_flag;
float triac_timer, triac_time;
uint32_t tim = 0;
uint16_t triac_on_time;
uint8_t triac_mode = MODE_OFF;
uint8_t mode = 0;
uint8_t kwh_time_count; /* TODO remove */
/*###*/
uint8_t kwh_update_flag;
uint8_t vi_update_flag;
/*###*/
#define EEPROM_KWH_MEM_ADDR		0xA


void HAL_GPIO_EXTI_Falling_Callback(uint16_t pin) {
	// TODO pin check
	if(pin == GPIO_PIN_4) {
		/* zero crossing detection */
//		lastime = TIM16->CNT;
		triac_timer = 0;
		triac_timer_flag = 1; /* allow the timer to run */
		/* keep the TRIACs low before triggering */
		TRIAC1_SET(0); /* trigger delay */
		TRIAC2_SET(0);
	}
	if(pin == GPIO_PIN_6) {
		/* RTC interrupt */
		/*###*/
		kwh_update_flag = 1;
		/*###*/
	}
}


float temperatures[SENSOR_COUNT + 1];

/* GSM stuff */
#define GSM_OK		0
#define GSM_WAIT	2
#define GSM_NOK 	99

#define GSM_CMD_LAST_IDX	13

volatile uint8_t gsm_tx_busy, gsm_rx_busy, gsm_status;
volatile uint16_t gsm_rx_timer, gsm_rx_timeout;
uint8_t gsm_cmd_step;
volatile uint8_t gsm_rx_flag;
#define GSM_RX_BUFFER_SIZE	100
uint8_t gsm_tx_buffer[300];
char gsm_rx_buffer[GSM_RX_BUFFER_SIZE];
char gsm_match_resp[20];

uint8_t upload_running = 0;
uint8_t upload_flag = 0;

uint32_t lastime;

/* Util funcs */
// Check if target string exists in buffer
uint8_t find_string_in_buffer(const char* buffer, const char* target) {
}

uint8_t gsm_cmd(char *cmd, char *op_check, uint16_t wtime) {
	char cmd_string[500];
	memset(cmd_string, 0, 20);
	sprintf(cmd_string, "%s%s", cmd, "\r\n" );
	gsm_tx_busy = 1;
	gsm_rx_timeout = wtime * 10;
	strcpy(gsm_match_resp, op_check);
	gsm_status = GSM_WAIT;
	return HAL_UART_Transmit_DMA(&huart3, (uint8_t *)cmd_string, strlen(cmd_string));
}

uint8_t gsm_is_valid_resp() {
    return strstr(gsm_rx_buffer, gsm_match_resp) != NULL;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART3) {
		gsm_tx_busy = 0;
		gsm_rx_busy = 1;
		memset(gsm_rx_buffer, 0, GSM_RX_BUFFER_SIZE);
		HAL_UART_Receive_DMA(huart, gsm_rx_buffer, GSM_RX_BUFFER_SIZE);
		gsm_rx_flag = 1;
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if(htim->Instance == TIM16) {
		if(ms > 10000) {
			if(sec > 60) {
				if(min > 60) {
					min = 0;
					hr++;
				}
				else min++;
				sec = 0;
			} else sec++;
			ms = 0;
			vi_update_flag = 1;
			/*###*/
		} else ms++;

		if(ms % 5000 == 0)
			sensor_refresh_flag = 1;
		btn1_timer = btn1_flag ? btn1_timer + 1: 0;
		btn2_timer = btn2_flag ? btn2_timer + 1: 0;
		btn3_timer = btn3_flag ? btn3_timer + 1: 0;
		led_blink_timer = led_blink_flag ? led_blink_timer + 1: 0;
		if(led_blink_timer > LED_BLINK_TIME) {
			led_blink_flag =  0;
			LED_OFF();
		}

		/*B*/
		/* If time up, trigger TRIAC */
		if(triac_mode == MODE_CTRL) {
			triac_timer = triac_timer_flag ? triac_timer + 0.1 : 0;

			if(triac_timer >= triac_time) {
				/* trigger TRIAC */
				triac_timer_flag = 0;
				TRIAC1_SET(1); /* trigger pulse */
				TRIAC2_SET(1);
				for(uint8_t i = 0; i < 100; i++);
				TRIAC1_SET(0); /* turn it off */
				TRIAC2_SET(0);
			}
		} else {
			TRIAC1_SET(0); /* trigger TRIAC */
			TRIAC2_SET(0);
			triac_time = 0;
		}
		/*B*/
		gsm_rx_timer = gsm_rx_flag ? gsm_rx_timer + 1: 0;
		if(gsm_rx_timer > gsm_rx_timeout) {
			gsm_rx_timer = 0;
			/* TODO process gsm_rx_buffer */
			if(gsm_is_valid_resp())
				gsm_status = GSM_OK;
			else
				gsm_status = GSM_NOK;

//			memset(gsm_rx_buffer, 0, GSM_RX_BUFFER_SIZE);
			gsm_rx_flag = 0; /* clear everything... */
			gsm_tx_busy = 0; /* ...to read data again */
			gsm_rx_busy = 0;
		}
	}
}
/* EEPROM */
#define EEPROM_I2C &hi2c1
// EEPROM ADDRESS (8bits)
#define EEPROM_ADDR 0xAE

//// Define the Page Size and number of pages
//#define PAGE_SIZE 16     // in Bytes
//#define PAGE_NUM  32    // number of pages
//
//void EEPROM_Write(uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
//{
//
//	// Find out the number of bit, where the page addressing starts
//	int paddrposition = log(PAGE_SIZE)/log(2);
//
//	// calculate the start page and the end page
//	uint16_t startPage = page;
//	uint16_t endPage = page + ((size+offset)/PAGE_SIZE);
//
//	// number of pages to be written
//	uint16_t numofpages = (endPage-startPage) + 1;
//	uint16_t pos=0;
//
//	// write the data
//	for (int i=0; i<numofpages; i++)
//	{
//		/* calculate the address of the memory location
//		 * Here we add the page address with the byte address
//		 */
//		uint16_t MemAddress = startPage<<paddrposition | offset;
//		uint16_t bytesremaining = bytestowrite(size, offset);  // calculate the remaining bytes to be written
//
//		HAL_I2C_Mem_Write(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, &data[pos], bytesremaining, 1000);  // write the data to the EEPROM
//
//		startPage += 1;  // increment the page, so that a new page address can be selected for further write
//		offset=0;   // since we will be writing to a new page, so offset will be 0
//		size = size-bytesremaining;  // reduce the size of the bytes
//		pos += bytesremaining;  // update the position for the data buffer
//
//		HAL_Delay (5);  // Write cycle delay (5ms)/*TODO implement using timer: eeprom_busy_flag */
//	}
//}
//
//void EEPROM_Read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
//{
//	int paddrposition = log(PAGE_SIZE)/log(2);
//
//	uint16_t startPage = page;
//	uint16_t endPage = page + ((size+offset)/PAGE_SIZE);
//
//	uint16_t numofpages = (endPage-startPage) + 1;
//	uint16_t pos=0;
//
//	for (int i=0; i<numofpages; i++)
//	{
//		uint16_t MemAddress = startPage<<paddrposition | offset;
//		uint16_t bytesremaining = bytestowrite(size, offset);
//		HAL_I2C_Mem_Read(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, &data, 2, 1000);
//		startPage += 1;
//		offset=0;
//		size = size-bytesremaining;
//		pos += bytesremaining;
//	}
//}



void eeprom_write(uint16_t idx, uint8_t data) {
	uint8_t d = data;
	uint8_t status, err;
	uint8_t buffer[2];
	buffer[0] = 0x0;
	buffer[1] = data;
//	status = HAL_I2C_Mem_Write(EEPROM_I2C, EEPROM_ADDR ,0, I2C_MEMADD_SIZE_8BIT, &d, 1, HAL_MAX_DELAY);  // write the data to the EEPROM
    status = HAL_I2C_Master_Transmit(&hi2c1, EEPROM_ADDR, buffer, 2, HAL_MAX_DELAY);
	err = hi2c1.ErrorCode;
	HAL_Delay(5);
}
uint8_t eeprom_read(uint16_t idx) {
	uint8_t d = 0;
	uint8_t data = 0;
	uint8_t status, err;
	/* using "Current Read" */
	status = HAL_I2C_Master_Transmit(EEPROM_I2C, EEPROM_ADDR, &d, 1, HAL_MAX_DELAY);
	err = hi2c1.ErrorCode;
	HAL_I2C_Master_Receive(EEPROM_I2C, EEPROM_ADDR, &data, 1, 1000);  // write the data to the EEPROM
	err = hi2c1.ErrorCode;
	return data;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM16_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	TRIAC1_SET(0);
	TRIAC2_SET(0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t sdo[2] = { 0, 0 };
	uint16_t temp_word;
	uint8_t temp_state = 0;
	uint16_t temp12b = 0;

	TEMP1_CS(1);
	TEMP1_CS(0);
	TEMP5_CS(0);
	TEMP5_CS(1);
	TEMP1_CS(1);
	TEMP1_CS(0);
	TEMP5_CS(0);
	TEMP5_CS(1);
	TEMP1_CS(1);
	TEMP1_CS(0);
	TEMP5_CS(0);
	TEMP5_CS(1);
	TEMP2_CS(1);
	TEMP4_CS(1);
	TEMP5_CS(1);
	TEMP6_CS(1);
	TEMP1_CS(0);

	adc_raw[0] = 0;
	adc_raw[1] = 0;
	adc_raw[2] = 0;
//	HAL_ADC_Start_DMA(&hadc1, adc_raw, 3); /*A*/
	/* GSM stuff */
	char content_string[200] = "";
	char api_key[20] = "F1LOAYMJF47UO4LD"; /* key for testing */
	// "01VH0OM4JU4KG9KN"; // API key
	/* GSM powerkey dance */
	/* TODO implement this using timer interrupts */
	HAL_GPIO_WritePin(MCU_RESET_GPIO_Port,MCU_RESET_Pin,GPIO_PIN_RESET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(MCU_RESET_GPIO_Port,MCU_RESET_Pin,GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(MCU_PWRKEY_GPIO_Port,MCU_PWRKEY_Pin,GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(MCU_PWRKEY_GPIO_Port,MCU_PWRKEY_Pin,GPIO_PIN_RESET);
	HAL_Delay(700);
	HAL_GPIO_WritePin(MCU_PWRKEY_GPIO_Port,MCU_PWRKEY_Pin,GPIO_PIN_SET);
	HAL_Delay(15000);
	uint8_t prev_idx = 1;

	/* key variables */
	uint8_t active_sensor_idx = 0;
	uint16_t set_point = 400; /* Cut the TRIAC off above 400 */

	/* Initialization */
	HAL_TIM_Base_Start_IT(&htim16);
	triac_timer_flag = 0;
	gsm_cmd_step = -1;

	/* ds3231 init */
	DateTime ti;
	DateTime time = {0};
	ti.day = 14;
	ti.month = 11;
	ti.year = 24;
	ti.dow = 2;
	ti.hr = 21;
	ti.min = 14;
	ti.sec = 0;

	/*A*/
//	ds3231_settime(&ti);
//	ds3231_gettime(&time);
//
//	ds3231_clearalarm1();
//	//DS3231_SetAlarm1(ALARM_MODE_ONCE_PER_SECOND, 0, 0, 0, 0);
	ds3231_clearflagalarm1(); /* clear alarm flag */
	ds3231_setalarm1(ALARM_MODE_SEC_MATCHED, 0, 0, 0, 10);
//	alarmcheck();
	/*A*/

	while (1)
	{

	/*A*/
	/*A*/

		/* update kwh */
		/*###*/
//		if(lastime-TIM16->CNT > 20) {
//			if(triac_mode == MODE_CTRL) {
//				triac_timer = triac_timer_flag ? triac_timer + 1 : 0;
//
//				if(triac_timer >= triac_time * 100) {
//					/* trigger TRIAC */
//					triac_timer_flag = 0;
//					TRIAC1_SET(1); /* trigger TRIAC */
//					TRIAC2_SET(1);
//					for(uint16_t i = 0; i < 80; i++);
//					TRIAC1_SET(0); /* trigger TRIAC */
//					TRIAC2_SET(0);
//					// if(triac_trigger_timer > TRIAC_TRIGGER_TIME) {
//					//   TRIAC1_SET(0); /* trigger delay */
//					//   TRIAC2_SET(0);
//					// }
//				}
//			} else {
//				TRIAC1_SET(0); /* trigger TRIAC */
//				TRIAC2_SET(0);
//				triac_time = 0;
//			}
//		}

		if(kwh_update_flag == 1) {
			/* reading ACS37800 */
			HAL_I2C_Mem_Read(&hi2c1, (ACS37800_I2C_ADDR << 1), ACS37800_REG_PACTAVGONEMIN, I2C_MEMADD_SIZE_8BIT, acs37800_p_buffer, 4, 100);
			uint16_t pavg_raw = (acs37800_p_buffer[1] << 8) | acs37800_p_buffer[0];
			pavg_final = pavg_raw;
			float LSBpermW = 3.08; // LSB per mW
			LSBpermW  *= 30.0 / ACS37800_CURR_SENS_RANGE; // Correct for sensor version
			pavg_final /= LSBpermW; // Convert from codes to mW
			//Correct for the voltage divider: (RISO1 + RISO2 + RSENSE) / RSENSE
			//Or:  (RISO1 + RISO2 + RISO3 + RISO4 + RSENSE) / RSENSE
			pavg_final /= 0.0008243;
			pavg_final /= 1000; // Convert from mW to W

			kwh = kwh + (pavg_final * (1/(float)60));
			/* TODO update kwh in EEPROM */
			kwh_update_flag = 0; /* wait till next min */
			ds3231_clearflagalarm1(); /* clear alarm flag */
		}
		if(vi_update_flag == 1) {
			HAL_I2C_Mem_Read(&hi2c1, (ACS37800_I2C_ADDR << 1), ACS37800_REG_VIRMS, I2C_MEMADD_SIZE_8BIT, acs37800_vi_buffer, 4, 100);
			uint16_t vrms_raw = (acs37800_vi_buffer[1] << 8) | acs37800_vi_buffer[0];
			vrms_final = vrms_raw / (float)55000;
			vrms_final = vrms_final * 250;
			vrms_final = vrms_final / 1000;
			vrms_final = vrms_final / 0.0008243;
			uint16_t irms_raw = (acs37800_vi_buffer[3] << 8) | acs37800_vi_buffer[2];
			irms_final = irms_raw / (float)55000;
			irms_final = irms_final * ACS37800_CURR_SENS_RANGE;
			if(irms_final < 0.050)
				irms_final = 0;
			vi_update_flag = 0; /* wait till next sec */
		}
		/*###*/
		/* routines */

		/*### Sensor read ###*/
		/*A*/
		if(sensor_refresh_flag == 1) {
			sensor_rx_select(sensor_idx);
			HAL_SPI_Receive(&hspi2, (uint8_t *)sdo, 2, 10);
			sensor_rx_disable(); // Disables all IC comms
			temp_state = (((sdo[0] | (sdo[1] << 8)) >> 2) & 0x0001);
			temp_word = (sdo[0] | sdo[1] << 8);
			temp12b = (temp_word & 0b111111111111000) >> 3;
			/* store the temp */
			if(temp_state == 1) {
				temperatures[sensor_idx - 1] = -99;
			}
			else {
				temperatures[sensor_idx - 1] = (float)(temp12b*0.25);
			}
			sensor_idx = sensor_idx >= SENSOR_COUNT ? 1 : sensor_idx + 1;
			sensor_refresh_flag = 0;
		}
		//	/* read two sensors, average it if both are working */
		//	if(temperatures[0] != -99 && temperatures[1] != -99) {
		//		temperatures[2] = (temperatures[0] + temperatures[1])/2;
		//		active_sensor_idx = 2;
		//	}
		//	else if(temperatures[0] != -99 && temperatures[1] == -99) {
		//		active_sensor_idx = 0;
		//	}
		//	else if(temperatures[0] == -99 && temperatures[1] != -99) {
		//		active_sensor_idx = 1;
		//	}
		//	else {
		//		temperatures[2] = -99;
		//		active_sensor_idx = 2;
		//	}
		sdo[0] = 0;
		sdo[1] = 0;
		temp_word = 0;
		temp12b = 0;
		//
		/*### ON-OFF Control ###*/
		if(temperatures[0] >= set_point || temperatures[1] >= set_point) {
			/* Turn TRIAC off */
			TRIAC1_SET(0);
			TRIAC2_SET(0);
			triac_temp_ctrl = 0;
		}
		else {
			triac_temp_ctrl = 1;
			/* Use TRIAC control logic to control output */
		}

		/*### Selector switch read ###*/
		if(triac_temp_ctrl == 1) {
			if(BTN1_READ() == 0) {
				if(BTN1_READ() == 0) {
					mode = 1;
					triac_time = 4.5; /* 130V */
					triac_mode = MODE_CTRL; /* Never trigger TRIACs */
				}
			}
			else if(BTN2_READ() == 0) {
				if(BTN2_READ() == 0) {
					mode = 2;
					triac_time = 3.37; /* 170V */
					triac_mode = MODE_CTRL; /* Never trigger TRIACs */
				}
			}
			else if(BTN3_READ() == 0) {
				if(BTN3_READ() == 0) {
					mode = 3;
					triac_time = 2.4; /* 205V */
					triac_mode = MODE_CTRL; /* Never trigger TRIACs */
				}
			}
			else {
				mode = 0;
				triac_mode = MODE_OFF; /* Never trigger TRIACs */
				/* keep triacs off */
				TRIAC1_SET(0);
				TRIAC2_SET(0);
			}
		}
		else {
			mode = 0;
			triac_mode = MODE_OFF; /* Never trigger TRIACs */
			/* keep triacs off */
			TRIAC1_SET(0);
			TRIAC2_SET(0);
		}

		/*A*/
		/* GSM stuff */
		/*########################################################################*/
		if(gsm_status != GSM_WAIT) {
			if(upload_running) {
				if(gsm_status == GSM_OK || gsm_status == GSM_NOK) {
					if(gsm_cmd_step >= GSM_CMD_LAST_IDX) {
						upload_flag = 1; /* successful upload */
						led_blink();
						gsm_cmd_step = 0; /* prep for next upload */
						upload_running = 0; /* wait for next time slot */
					}
					else
						gsm_cmd_step += 1;
				}
//				else if(gsm_status == GSM_NOK) {
//					gsm_cmd_step = 0;
//					upload_running = 0; /* cancel upload seq */
//				}
				switch(gsm_cmd_step) {
				case 0:
					break;
				case 1:
					gsm_cmd("AT+NETCLOSE","OK", GSM_WAIT_TIME_LOW);
					break;
				case 2:
					gsm_cmd("AT+CCHMODE=1","OK", GSM_WAIT_TIME_LOW);
					break;
				case 3:
					gsm_cmd("AT+CCHSET=1","OK", GSM_WAIT_TIME_LOW);
					break;
				case 4:
					gsm_cmd("AT+CCHSTART","OK", GSM_WAIT_TIME_LOW);
					break;
				case 5:
					gsm_cmd("AT+CCHSSLCFG=0,0","OK",GSM_WAIT_TIME_LOW);
					break;
				case 6:
					gsm_cmd("AT+CSOCKSETPN=1","OK", GSM_WAIT_TIME_LOW);
					break;
				case 7:
					gsm_cmd("AT+CIPMODE=0","OK", GSM_WAIT_TIME_LOW);
					break;
				case 8:
					gsm_cmd("AT+NETOPEN","OK", GSM_WAIT_TIME_LOW);
					break;
				case 9:
					gsm_cmd("AT+CGATT=1","OK", GSM_WAIT_TIME_LOW);
					break;
				case 10:
					gsm_cmd("AT+CGACT=1,1","OK", GSM_WAIT_TIME_LOW);
					break;
				case 11:
					gsm_cmd("AT+IPADDR","OK", GSM_WAIT_TIME_MED);
					break;
				case 12:
					gsm_cmd("AT+CCHOPEN=0,\"api.thingspeak.com\",443,2","CONNECT 115200", GSM_WAIT_TIME_MED);
					break;
				case 13:
					sprintf(content_string, "GET /update?api_key=%s&field1=%d&field2=%d&field3=%d&field4=%.1f&field5=%d&field6=%d\r\n" \
							"HTTP/1.1\r\nHost: api.thingspeak.com\r\n", \
							api_key, (int)temperatures[0], (int)temperatures[1], (int)mode, \
							(float)irms_final, (int)vrms_final, kwh);
					/* to upload:
					 * cur
					 * vol
					 * kwh
					 * temp 1
					 * temp 2
					 *
					 */
					gsm_cmd(content_string, "200 OK", GSM_WAIT_TIME_MED);
					break;
				case 14:
					gsm_cmd("AT+CIPCLOSE=0", "OK", GSM_WAIT_TIME_LOW);
					break;
				default:
				}
			}
			else gsm_cmd_step = 0;
		}
		if(sec % 30 == 0 && sec != 0) {
			if(upload_running == 0 && upload_flag == 0) { /* upload flag indicates if an */
				upload_running = 1; /* start uploading */
				gsm_cmd_step = 0; /* with the first command */
			}
		} else upload_flag = 0;

		/*########################################################################*/
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV12;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0060112F;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 32;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 100;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TRIAC2_Pin|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, TRIAC1_Pin|UP_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MCU_RESET_Pin|MCU_PWRKEY_Pin|CS_TC6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_TC1_Pin|CS_TC2_Pin|CS_TC5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TRIAC2_Pin PC14 PC15 */
  GPIO_InitStruct.Pin = TRIAC2_Pin|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIAC1_Pin UP_LED_Pin */
  GPIO_InitStruct.Pin = TRIAC1_Pin|UP_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : ZCD_Pin */
  GPIO_InitStruct.Pin = ZCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ZCD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_RESET_Pin MCU_PWRKEY_Pin CS_TC6_Pin */
  GPIO_InitStruct.Pin = MCU_RESET_Pin|MCU_PWRKEY_Pin|CS_TC6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_TC1_Pin CS_TC2_Pin CS_TC5_Pin */
  GPIO_InitStruct.Pin = CS_TC1_Pin|CS_TC2_Pin|CS_TC5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RTC_INT_Pin */
  GPIO_InitStruct.Pin = RTC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RTC_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN3_IN_Pin BTN2_IN_Pin BTN1_IN_Pin */
  GPIO_InitStruct.Pin = BTN3_IN_Pin|BTN2_IN_Pin|BTN1_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
