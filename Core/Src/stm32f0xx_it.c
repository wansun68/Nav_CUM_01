/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

extern FlagsT flag;
extern uint16_t pwrVal;
extern uint8_t pwr_cnt;
extern uint8_t led_cnt;

char buffStr[3] = {0};
char TM_Str[17] = {0};
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc);
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */

  flag.TM_flag = 1;
  TeleMetrySend();

  /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */
  flag.TIM16_flag = 1;
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
  /* USER CODE END TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM17 global interrupt.
  */
void TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM17_IRQn 0 */

  /* USER CODE END TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM17_IRQn 1 */
  /*if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==GPIO_PIN_SET && flag.btnOn==0)
  {
	  flag.btnOn = 1;
	  if (pwr_cnt < 4)
	  {
		  pwr_cnt++;
	  }
	  else pwr_cnt = 1;
  }
  else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==GPIO_PIN_RESET && flag.btnOn==1) flag.btnOn = 0;*/
  /* USER CODE END TIM17_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void TeleMetrySend(void)
{
	HAL_ADC_Start(&hadc);
	  HAL_ADC_PollForConversion(&hadc, 10);
	  pwrVal = HAL_ADC_GetValue(&hadc);
	  HAL_ADC_Stop(&hadc);

	  pwrVal = pec_Update(pwrVal);
	  Bin2Str(pwrVal, buffStr);

	  TM_Str[0]='#';
	  TM_Str[1]='T';
	  TM_Str[2]='M';
	  TM_Str[3]=',';
	  TM_Str[4]=buffStr[0];
	  TM_Str[5]=buffStr[1];
	  TM_Str[6]='.';
	  TM_Str[7]=buffStr[2];
	  TM_Str[8]=',';
	  TM_Str[9]='R';
	  TM_Str[10]=',';
	  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==GPIO_PIN_SET) TM_Str[11]='W';
	  else TM_Str[11]=' ';
	  TM_Str[12]='*';
	  CrcBuild();
	  TM_Str[15]='\n';
	  TM_Str[16]='\r';

	  HAL_UART_Transmit(&huart1, TM_Str, 17, 0xFFFF);
	  HAL_TIM_Base_Start_IT(&htim16);

	  if(pwrVal <= 300) pwr_cnt = 1;
	  else if(pwrVal > 300 && pwrVal <= 340) pwr_cnt = 2;
	  else if(pwrVal > 340 && pwrVal <= 380) pwr_cnt = 3;
	  else pwr_cnt = 4;

	  led_cnt = pwr_cnt*2;
}

void CrcBuild (void)
{
	uint8_t crc = 0;
	for (uint8_t i = 0; i <=12; i++)
	{
		crc += TM_Str[i];
	}
	uint8_t LB = crc&0x0f;
	uint8_t HB = crc>>4;
	uint8_t LB_cnt = 0;
	uint8_t HB_cnt = 0;

	for (uint8_t a = 0; a <=HB; a++)
	{
		HB_cnt++;
	}

	if (HB_cnt == 0) TM_Str[13] = '0';
	else if (HB_cnt == 1) TM_Str[13] = '1';
	else if (HB_cnt == 2) TM_Str[13] = '2';
	else if (HB_cnt == 3) TM_Str[13] = '3';
	else if (HB_cnt == 4) TM_Str[13] = '4';
	else if (HB_cnt == 5) TM_Str[13] = '5';
	else if (HB_cnt == 6) TM_Str[13] = '6';
	else if (HB_cnt == 7) TM_Str[13] = '7';
	else if (HB_cnt == 8) TM_Str[13] = '8';
	else if (HB_cnt == 9) TM_Str[13] = '9';
	else if (HB_cnt == 10) TM_Str[13] = 'A';
	else if (HB_cnt == 11) TM_Str[13] = 'B';
	else if (HB_cnt == 12) TM_Str[13] = 'C';
	else if (HB_cnt == 13) TM_Str[13] = 'D';
	else if (HB_cnt == 14) TM_Str[13] = 'E';
	else if (HB_cnt == 15) TM_Str[13] = 'F';

	for (uint8_t b = 0; b <=LB; b++)
	{
		LB_cnt++;
	}
	if (LB_cnt == 0) TM_Str[14] = '0';
	else if (LB_cnt == 1) TM_Str[14] = '1';
	else if (LB_cnt == 2) TM_Str[14] = '2';
	else if (LB_cnt == 3) TM_Str[14] = '3';
	else if (LB_cnt == 4) TM_Str[14] = '4';
	else if (LB_cnt == 5) TM_Str[14] = '5';
	else if (LB_cnt == 6) TM_Str[14] = '6';
	else if (LB_cnt == 7) TM_Str[14] = '7';
	else if (LB_cnt == 8) TM_Str[14] = '8';
	else if (LB_cnt == 9) TM_Str[14] = '9';
	else if (LB_cnt == 10) TM_Str[14] = 'A';
	else if (LB_cnt == 11) TM_Str[14] = 'B';
	else if (LB_cnt == 12) TM_Str[14] = 'C';
	else if (LB_cnt == 13) TM_Str[14] = 'D';
	else if (LB_cnt == 14) TM_Str[14] = 'E';
	else if (LB_cnt == 15) TM_Str[14] = 'F';
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
