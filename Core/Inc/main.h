/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef struct
{
	uint8_t TM_flag:1; 		//Флаг отправки телеметрии
	uint8_t Last_TM_flag:1; //Предыдущее значение флага отправки телеметрии
	uint8_t TM_Send:1; 		//Флаг наличия пакета для отправки телеметрии
	uint8_t TIM16_flag:1;   //Флаг срабатывания таймера задержки свечения индикатора уровня заряда
	uint8_t btnOn:1;        //Флаг нажатия кнопки
	uint8_t flag_send:1;    //Флаг наличия сообщения для отправки nmea строки
	uint8_t flag_true:1;    //Флаг прохождения nmea строки фильтра строк (Принята GGA строка)
}FlagsT;

FlagsT flag;
uint16_t pwrVal;
uint8_t pwr_cnt;            //Счетчик миганий для отображения уровня заряда аккумулятора
uint8_t led_cnt;
char buffStr[3];
char TM_Str[17];
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint16_t pec_Update(uint16_t pec);        //Преобразование значения с АЦП в напряжение
void Bin2Str(uint16_t data, char* pbuf);  //Преобразование значения напряжения в строку для отправки по UART
void TeleMetrySend(void);
void CrcBuild (void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
