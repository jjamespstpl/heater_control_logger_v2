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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
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

		/* If time up, trigger TRIAC */
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /* Initialization */
  HAL_TIM_Base_Start_IT(&htim16);
  triac_timer_flag = 0;
  gsm_cmd_step = -1;

  float prms = 0;
  uint32_t sample = 0;

  while (1)
  {

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
					sprintf(content_string, "GET /update?api_key=%s&field1=%d&field2=%d&field3=%d&field4=%.1f&field5=%d&field6=%.1f\r\n" \
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
  huart3.Init.BaudRate = 115200;
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
  HAL_GPIO_WritePin(GPIOA, MCU_RESET_Pin|MCU_PWRKEY_Pin|CS_TC6_Pin|LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_TC1_Pin|CS_TC2_Pin|CS_TC5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_2_Pin|LED_3_Pin|SPI1_CS_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : MCU_RESET_Pin MCU_PWRKEY_Pin CS_TC6_Pin LED_1_Pin */
  GPIO_InitStruct.Pin = MCU_RESET_Pin|MCU_PWRKEY_Pin|CS_TC6_Pin|LED_1_Pin;
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

  /*Configure GPIO pins : LED_2_Pin LED_3_Pin SPI1_CS_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin|LED_3_Pin|SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
