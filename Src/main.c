/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_host.h"

/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int sampleNB = 100; //CHANGE FOR SAMPLE NUMBER
int sample = 0;

#define MATH_ARRAY_SIZE 5
int displayMode = 0;
float filterMemory [] = {0, 0, 0, 0, 0};
int adc_val;
float filtered_adc;
float mathResults [MATH_ARRAY_SIZE];
extern uint8_t systickFlag;
extern uint8_t buttonFlag;

int padEntries [] = {0, 0, 0, 0};
int padVal;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
void MX_USB_HOST_Process(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void FIR_C(float input, float *output);
int buttonPressed(void);
void displayNum (int num, int pos);
void C_math (float * inputArray, float * outputArray, int length);
void display (int mode, float num);
int readPad (void);
void updateEnt (int newEnt);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	
	
	
	
	
	
	
	
	
	
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		padVal = readPad();
		if (padVal != -1) {
			updateEnt (padVal);
		}
		
		for (int i = 0; i<4; i++){
			displayNum (padEntries[i], i);
		}
		
		
		
  /* USER CODE END WHILE */
    MX_USB_HOST_Process();

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2S3 init function */
static void MX_I2S3_Init(void)
{

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PB10   ------> I2S2_CK
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|Dig_1_Pin|Dig_2_Pin|Dig_L_Pin 
                          |Dig_3_Pin|Dig_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Row_1_Pin|Row_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Row_3_Pin|Row_4_Pin|Seg_DP_Pin|Seg_G_Pin 
                          |Seg_F_Pin|Seg_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Seg_D_Pin|Seg_C_Pin|Seg_B_Pin|Seg_A_Pin 
                          |LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin Dig_1_Pin Dig_2_Pin Dig_L_Pin 
                           Dig_3_Pin Dig_4_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|Dig_1_Pin|Dig_2_Pin|Dig_L_Pin 
                          |Dig_3_Pin|Dig_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin Row_1_Pin Row_2_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|Row_1_Pin|Row_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Row_3_Pin Row_4_Pin Seg_DP_Pin Seg_G_Pin 
                           Seg_F_Pin Seg_E_Pin */
  GPIO_InitStruct.Pin = Row_3_Pin|Row_4_Pin|Seg_DP_Pin|Seg_G_Pin 
                          |Seg_F_Pin|Seg_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Col_1_Pin Col_2_Pin Col_3_Pin Col_4_Pin */
  GPIO_InitStruct.Pin = Col_1_Pin|Col_2_Pin|Col_3_Pin|Col_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Seg_D_Pin Seg_C_Pin Seg_B_Pin Seg_A_Pin 
                           LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = Seg_D_Pin|Seg_C_Pin|Seg_B_Pin|Seg_A_Pin 
                          |LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int readPad (){
	
	int keymap[4][3]=
	{
	{1, 2, 3},
	{4, 5, 6},
	{7, 8, 9},
	{10, 0, 11}
	};
	
	for (int i = 0; i < 4; i++){
		switch (i) {
			case 0 : 
				HAL_GPIO_WritePin(GPIOC, Row_1_Pin, GPIO_PIN_SET);
				if (HAL_GPIO_ReadPin(GPIOE, Col_1_Pin) == 1){
					return keymap [i][0];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_2_Pin) == 1){
					return keymap [i][1];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_3_Pin) == 1){
					return keymap [i][2];
				}
				HAL_GPIO_WritePin(GPIOC, Row_1_Pin, GPIO_PIN_RESET);
			break;
			
			case 1:
				HAL_GPIO_WritePin(GPIOC, Row_2_Pin, GPIO_PIN_SET);
				if (HAL_GPIO_ReadPin(GPIOE, Col_1_Pin) == 1){
					return keymap [i][0];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_2_Pin) == 1){
					return keymap [i][1];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_3_Pin) == 1){
					return keymap [i][2];
				}
				HAL_GPIO_WritePin(GPIOC, Row_1_Pin, GPIO_PIN_RESET);
			break;
			
			case 2:
				HAL_GPIO_WritePin(GPIOC, Row_2_Pin, GPIO_PIN_SET);
				if (HAL_GPIO_ReadPin(GPIOE, Col_1_Pin) == 1){
					return keymap [i][0];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_2_Pin) == 1){
					return keymap [i][1];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_3_Pin) == 1){
					return keymap [i][2];
				}
				HAL_GPIO_WritePin(GPIOC, Row_1_Pin, GPIO_PIN_RESET);
			break;
			
			case 3:
				HAL_GPIO_WritePin(GPIOC, Row_3_Pin, GPIO_PIN_SET);
				if (HAL_GPIO_ReadPin(GPIOE, Col_1_Pin) == 1){
					return keymap [i][0];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_2_Pin) == 1){
					return keymap [i][1];
				}
				else if (HAL_GPIO_ReadPin(GPIOE, Col_3_Pin) == 1){
					return keymap [i][2];
				}
				HAL_GPIO_WritePin(GPIOC, Row_1_Pin, GPIO_PIN_RESET);
			break;
			
			default:
				
			break;
			
		}
	}
	return -1;
}

/**
  * @brief  This function displays a integer on one of the digits of a 7 segiment, 4 digit led display
	* @param  num: integer number to be displayed 
						pos: the location in a 4 digit display,
  * @retval none
  */
void displayNum (int num, int pos) {       //function used to diplay a single digit on LED segments
	switch (num){
		case 0:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_SET);
			break;
		
		case 3:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_SET);
			break;
		
		case 4:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_SET);
			break;
		
		case 5:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_SET);
			break;
		
		case 6:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_SET);
			break;
		
		case 7:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_RESET);
			break;
		
		case 8:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_SET);
			break;
		case 9:
			HAL_GPIO_WritePin(GPIOD, Seg_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, Seg_D_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_E_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Seg_F_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Seg_G_Pin, GPIO_PIN_SET);
			break;
		default :
			break;
	}
		
		switch (pos){
			case 0:
				HAL_GPIO_WritePin(GPIOE, Dig_1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, Dig_2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_L_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, Seg_DP_Pin, GPIO_PIN_RESET);
			break;
			case 1:
				HAL_GPIO_WritePin(GPIOE, Dig_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, Dig_L_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, Seg_DP_Pin, GPIO_PIN_SET);
			break;
			case 2:
				HAL_GPIO_WritePin(GPIOE, Dig_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_L_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, Dig_4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, Seg_DP_Pin, GPIO_PIN_RESET);
			break;
			case 3 :
				HAL_GPIO_WritePin(GPIOE, Dig_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_L_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, Dig_4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, Seg_DP_Pin, GPIO_PIN_RESET);
			break;
			default :
				break;
		}
	
}
/**
  * @brief  This is the FIR function from LAB 1
						Filters data using FIR to eliminate unwanted noise
	* @param  Input: data to be filtered
						Output: filtered data
  * @retval none
  */
void FIR_C(float Input, float *Output) {   
	float coef [] = {0.2, 0.2, 0.2, 0.2, 0.2};          //coefficients -> SHOULD ADD UP TO 1 !
	float out = 0.0;
	for (int i = 4; i>0; i--){
		filterMemory[i] = filterMemory[i-1];
	}
	filterMemory [0] = Input;
	for (int i = 0; i<5; i++){
		out += coef[i]*filterMemory[i];
	}
	*Output = out;
}


/**
  * @brief  This is the C?math function from lab1
						calculates rms, max, min, min index, and max index from the given array
	* @param  inputArray: contains the samples to be analyzed
						outputArray: contains the array where the results are stored
						length: length of the input array
  * @retval none
  */
void C_math (float * inputArray, float * outputArray, int length){      //C_math function from lab 1

	float RMS=0;
	float maxVal = inputArray[0];
	float minVal = inputArray[0];
	int maxInd = 0;
	int minInd = 0;
	
	
	for (int i = 0; i< (length); i++){
		
		RMS += (inputArray[i])*(inputArray[i]);
		
		if ((inputArray[i]) > maxVal){
			maxVal = inputArray[i];
			maxInd = i;
		}
		if ((inputArray[i]) < minVal){
			minVal = (inputArray[i]);
			minInd = i;
		}
	}
	
	RMS = RMS/length;
	RMS = sqrt(RMS);
	
	outputArray [0]=RMS;
	outputArray [1]= maxVal;
	outputArray [2]= minVal;
	outputArray [3]= maxInd;
	outputArray [4]= minInd;
	
}

/**
  * @brief  detects a blue button press
  * @param  none
  * @retval returns 1 when button is pressed
						0 otherwise
  */
/*
int buttonPressed(){    
	if ( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0 ) == SET) {
		HAL_Delay(200);
		printf("button (pulled)!");
		return 1;
	}
	else 
	{
		return 0;
	}
}
*/
/**
  * @brief  function used to display a float value on the LED segments
  * @param  num: the floating number to be displayed at digits 1, 2, and 3
*         	mode: the int thats displayed at the firist digit to indicate rms/min/max mode 
  * @retval None
  */
void display (int mode, float num){     //
	
	
	displayNum (mode, 0); //display mode at location 0 (first LED display)
	HAL_Delay(1);
	
	float temp = num;
	int digit = (int) num;
	
	
	digit = digit % 10;
	displayNum (digit, 1); //display unit at location 1
	HAL_Delay(1);
	
	temp = num*10;
	digit = (int) temp;
	digit = digit % 10;
	displayNum (digit, 2); //display digit at location 2
	HAL_Delay(1);
	
	temp = num*100;
	digit = (int) temp;
	digit = digit % 10;
	displayNum (digit, 3); //display digit at location 3
	HAL_Delay(1);
}

void updateEnt (int newEnt) {
	
	for (int i = 3; i>0; i++){
		padEntries[i] = padEntries [i-1];
	}
	padEntries [0] = newEnt;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
