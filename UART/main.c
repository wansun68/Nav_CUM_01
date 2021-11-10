/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#define size_tx_buffer 100 //Äëèíà áóôåðà tx
bool flag_send; //ôëàã íà ðàçðåøåíèå âûâîäà tx_buffer[] â serial port
bool flag_true;
uint8_t queue_message; //î÷åðåäü ñîîáùåíèé (ò.ê. ðàáîòàåò ñ ïðåðûâàíèÿìè èëè DMA. ñì. óðîê 6)
uint8_t tx_buffer[size_tx_buffer]; //ñîáñòâåííî tx_buffer, êîòîðûé áóäåò âûâîäèòü â ïîðò òî, ÷òî ïðèíÿë ÌÊ ïî rx.
uint8_t rx_buffer[1]; // Áóôåð, êîòîðûé áóäåò ïðèíèìàòü èíôîðìàöèþ. (Ïî îäíîìó ñèìâîëó, ÷òîá ïîñòîÿííî â ïðåðûâàíèå çàõîäèòü è îáðàáàòûâàòü âõîäÿùåå ñîîáùåíèå.)
uint8_t error_message[] = "tx buffer is crowded\n\r"; //Ñîîáùåíèå î òîì, ÷òî tx_buffer[size_tx_buffer] ïåðåïîëíåí.
uint8_t error_counter; // ñëóæèò äëÿ òîãî, ÷òîá ñîîáùåíèå "tx buffer is crowded\n\r" âûâîäèëîñü òîëüêî ðàç.
volatile uint8_t rx_counter; // Ñ÷åò÷èê äëÿ âõîäÿùèõ ñèìâîëîâ.
volatile uint8_t size_message; // Ðàçìåð ñîîáùåíèÿ äëÿ âûâîäà â ïîðò.
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_CRC_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {//Ñèìâîë ïðèíÿëñÿ, ñðàáàòûâàåò ïðåðûâàíèå, çäåñü ìû áóäåì óæå ÷òî-òî äåëàòü ñ ïðèíÿòûìè äàííûìè.
	if (rx_buffer[0] != '\n')
		{ //åñëè âõîäÿùèé ñèìâîë íå ðàâåí '\r'
				if(rx_counter == 0)
				{
					if(rx_buffer[0]=='$') flag_true = 1;
					else flag_true = 0;
				}
				else if (rx_counter == 3 && flag_true)
				{
					if(rx_buffer[0]=='G') flag_true = 1;
					else flag_true = 0;
				}
				else if (rx_counter == 4  && flag_true)
				{
					if(rx_buffer[0]=='G') flag_true = 1;
					else flag_true = 0;
				}
				else if (rx_counter == 5  && flag_true)
					{
						if(rx_buffer[0]=='A') flag_true = 1;
						else flag_true = 0;
					}
			tx_buffer[rx_counter] = rx_buffer[0]; //äîáàâèì ýòîò ñèìâîë â tx_buffer
			rx_counter++;//óâåëè÷èâàåì ñ÷åò÷èê âõîäÿùèõ ñèìâîëîâ
			size_message = rx_counter; //ðàçìåð ñîîáùåíèÿ, êîòîðîå â ñåðèàë ïîðò áóäåò îòïðàâëÿòüñÿ
			if (size_message >= size_tx_buffer && error_counter == 0)
				{//åñëè ðàçìåð ñîîáùåíèÿ ïðåâûøàåò ðàçìåð áóôåðà, òî âûâîäèì ñîîáùåíèå, ÷òî áóôåð ïåðåïîëíåí.
					HAL_UART_Transmit(&huart1, error_message, sizeof error_message / sizeof error_message[0],0xffff);
					error_counter++;//ñ÷åò÷èê âûâîäà ñîîáùåíèé îøèáêè. ×òîá îòïðàâèëîñü òîëüêî ðàç.
					flag_send = 0;//çàïðåùàåì âûâîäèòü ñîîáùåíèå â ïîðò, âåäü îíî ïðåâûøåíî ïî ðàçìåðó.
				}
		}
	else if (rx_buffer[0] == '\n')
	{//åñëè æå âõîäÿùèé ñèìâîë ðàâåí '\r'
		tx_buffer[rx_counter] = '\n';//òî íàøå ñîîáùåíèå ïîëíîñòüþ ïîëó÷åíî. Äàâàéòå äîáàâèì ê íåìó ïåðåâîä íà ñëåäóþùóþ ñòðîêó
		//tx_buffer[rx_counter+1] = '\n';//òàê æå äîáàâèì âîçâðàò êàðåòêè
		//tx_buffer[rx_counter + 2] = '\0';//ðó÷êàìè ïðîïèøåì íóëåâîé ñèìâîë, ò.ê. îí ñàì ñåáÿ íå äîáàâèò
		size_message = rx_counter+1;//ñîîòâåòñòâåííî ìû äîëæíû óâåëè÷èòü ðàçìåð îòïðàâëÿåìîãî ñîîáùåíèÿ â ïîðò.
		rx_counter = 0;//ñáðîñèì ñ÷åò÷èê âõîäÿùèé ñèìâîëîâ, ÷òîá çàíîâî ïîëó÷àòü ñîîáùåíèå.
		if (size_message >= size_tx_buffer)
			{//îïÿòü æå, åñëè ðàçìåð ñîîáùåíèÿ ïðåâûøàåò ðàçìåð áóôåðà, òî âûâîäèì ñîîáùåíèå, ÷òî áóôåð ïåðåïîëíåí.
				flag_send = 0;////çàïðåùàåì âûâîäèòü ñîîáùåíèå â ïîðò, âåäü îíî ïðåâûøåíî ïî ðàçìåðó.
				HAL_UART_Transmit(&huart1, error_message,sizeof error_message / sizeof error_message[0],0xffff);
			}
		else
		{
			if (flag_true)
				{
					flag_send = 1;//åñëè æå ñîîáùåíèå íå ïðåâûøåíî ïî ðàçìåðó tx_buffer, òî ðàçðåøàåì âûâîäèòü â ïîðò òî, ÷òî âûâåëè
					error_counter = 0;//ñáðîñèì ñ÷åò÷èê îøèáîê
				}
			else flag_send = 0;
			}
	}
	HAL_UART_Receive_IT(&huart1, rx_buffer, 1);//Çàïóñêàåì ïðèåì äàííûõ ïîñëå êàæäîãî ïðåðûâàíèÿ.
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
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_CRC_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  flag_send = 0;//Îòïðàâêà ñîîáùåíèé åùå çàïðåùåíà, âåäü îòïðàâëÿòü åùå íå÷åãî.
  	queue_message = 1;//î÷åðåäü îòïðàâêè ñîîáùåíèé èçíà÷àëüíî ðàâíà 1
  	rx_counter = 0;//Ñèìâîëû åùå íå ïðèõîäèëè. Áóäåò 0.
  	size_message = 0;//Ðàçìåðà îòïðàâëÿåìîãî ñîîáùåíèÿ åùå íå çíàåì. Áóäåò 0.
  	error_counter = 0;//Îøèáîê åùå íå áûëî. Áóäåò 0.
  	HAL_UART_Receive_IT(&huart1, rx_buffer, 1);//Ðàçðåøàåì ïðèåì äàííûõ, ðàçìåðîì â 1 ñèìâîë.
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (flag_send == 1 && queue_message != 255)
	  	  {//åñëè ðàçðåøåíî îòñûëàòü ñîîáùåíèå è î÷åðåäü íå ðàâíà ìàêñèìàëüíîé, òî
	  	  	  if (queue_message == 1 && huart1.gState == HAL_UART_STATE_READY)// && tx_buffer[0] == '$' && tx_buffer[1] == 'G' && tx_buffer[2] == 'N')// && tx_buffer[3] == 'G')
	  	  	  	  { //åñëè ïîðò ñâîáîäåí, è î÷åðåäü ñîîáùåíèÿ ñîîòâåòñòâóåò, òî
	  	  		  	  HAL_UART_Transmit(&huart1, tx_buffer, size_message,0xffff);//îòïðàâëÿåì ñîîáùåíèå, ÷åãî íàì òàì ïðèøëî â rx_buffer
	  	  		  	  queue_message = 255;//ñòàâèì ñëåäóþùóþ î÷åðåäü, åñëè çàõîòèì åùå ÷òî-òî îòñûëàòü â ïîðò.
	  	  	  	  }
	  	  }
	  else if (flag_send == 1 && queue_message == 255)
	  	  {//åñëè æå î÷åðåäü äîøëà äî ìàêñèìàëüíîé, òî
	  	  	  queue_message = 1;//ñáðîñèì î÷åðåäü îòïðàâêè, ÷òîá ñîîáùåíèÿ ñíîâà ìîãëè ïî î÷åðåäè îòñûëàòüñÿ
	  		  flag_send = 0;//îòïðàâêó ñîîáùåíèé çàïðåòèì. Ðàçðåøåíèå ïîëó÷èì â ïðåðûâàíèè.
	  	  }
  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
