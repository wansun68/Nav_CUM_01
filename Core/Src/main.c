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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define size_tx_buffer 100 // Размер буффера приема-передачи nmea строки
uint8_t queue_message;
uint8_t tx_buffer[size_tx_buffer];
uint8_t rx_buffer[1];
uint8_t error_message[] = "error!\n\r";
uint8_t error_counter;
volatile uint8_t rx_counter;
volatile uint8_t size_message;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM17_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {//Ñèìâîë ïðèíÿëñÿ, ñðàáàòûâàåò ïðåðûâàíèå, çäåñü ìû áóäåì óæå ÷òî-òî äåëàòü ñ ïðèíÿòûìè äàííûìè.
	if (rx_buffer[0] != '\n')
		{ //åñëè âõîäÿùèé ñèìâîë íå ðàâåí '\r'
				if(rx_counter == 0)
				{
					if(rx_buffer[0]=='$') flag.flag_true = 1;
					else flag.flag_true = 0;
				}
				else if (rx_counter == 3 && flag.flag_true)
				{
					if(rx_buffer[0]=='G') flag.flag_true = 1;
					else flag.flag_true = 0;
				}
				else if (rx_counter == 4  && flag.flag_true)
				{
					if(rx_buffer[0]=='G') flag.flag_true = 1;
					else flag.flag_true = 0;
				}
				else if (rx_counter == 5  && flag.flag_true)
					{
						if(rx_buffer[0]=='A') flag.flag_true = 1;
						else flag.flag_true = 0;
					}
			tx_buffer[rx_counter] = rx_buffer[0]; //äîáàâèì ýòîò ñèìâîë â tx_buffer
			rx_counter++;//óâåëè÷èâàåì ñ÷åò÷èê âõîäÿùèõ ñèìâîëîâ
			size_message = rx_counter; //ðàçìåð ñîîáùåíèÿ, êîòîðîå â ñåðèàë ïîðò áóäåò îòïðàâëÿòüñÿ
			if (size_message >= size_tx_buffer && error_counter == 0)
				{//åñëè ðàçìåð ñîîáùåíèÿ ïðåâûøàåò ðàçìåð áóôåðà, òî âûâîäèì ñîîáùåíèå, ÷òî áóôåð ïåðåïîëíåí.
					HAL_UART_Transmit(&huart1, error_message, sizeof error_message / sizeof error_message[0],0xffff);
					error_counter++;//ñ÷åò÷èê âûâîäà ñîîáùåíèé îøèáêè. ×òîá îòïðàâèëîñü òîëüêî ðàç.
					flag.flag_send = 0;//çàïðåùàåì âûâîäèòü ñîîáùåíèå â ïîðò, âåäü îíî ïðåâûøåíî ïî ðàçìåðó.
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
			flag.flag_send = 0;////çàïðåùàåì âûâîäèòü ñîîáùåíèå â ïîðò, âåäü îíî ïðåâûøåíî ïî ðàçìåðó.
				HAL_UART_Transmit(&huart1, error_message,sizeof error_message / sizeof error_message[0],0xffff);
			}
		else
		{
			if (flag.flag_true)
				{
				flag.flag_send = 1;//åñëè æå ñîîáùåíèå íå ïðåâûøåíî ïî ðàçìåðó tx_buffer, òî ðàçðåøàåì âûâîäèòü â ïîðò òî, ÷òî âûâåëè
					error_counter = 0;//ñáðîñèì ñ÷åò÷èê îøèáîê
				}
			else flag.flag_send = 0;
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

  flag.TM_flag = 0;
  flag.TIM16_flag = 0;
  flag.btnOn = 0;
  flag.flag_send = 0;
  flag.flag_true = 0;

  pwrVal = 0;
  pwr_cnt = 0;
  led_cnt = 0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  MX_TIM17_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim14);
  queue_message = 1;//î÷åðåäü îòïðàâêè ñîîáùåíèé èçíà÷àëüíî ðàâíà 1
  rx_counter = 0;//Ñèìâîëû åùå íå ïðèõîäèëè. Áóäåò 0.
  size_message = 0;//Ðàçìåðà îòïðàâëÿåìîãî ñîîáùåíèÿ åùå íå çíàåì. Áóäåò 0.
  error_counter = 0;//Îøèáîê åùå íå áûëî. Áóäåò 0.
  HAL_UART_Receive_IT(&huart1, rx_buffer, 1);//Ðàçðåøàåì ïðèåì äàííûõ, ðàçìåðîì â 1 ñèìâîë.
  //HAL_TIM_Base_Start_IT(&htim17);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (flag.flag_send == 1 && queue_message != 255)
	  	  	  {//åñëè ðàçðåøåíî îòñûëàòü ñîîáùåíèå è î÷åðåäü íå ðàâíà ìàêñèìàëüíîé, òî
	  	  	  	  if (queue_message == 1 && huart1.gState == HAL_UART_STATE_READY)// && tx_buffer[0] == '$' && tx_buffer[1] == 'G' && tx_buffer[2] == 'N')// && tx_buffer[3] == 'G')
	  	  	  	  	  { //åñëè ïîðò ñâîáîäåí, è î÷åðåäü ñîîáùåíèÿ ñîîòâåòñòâóåò, òî
	  	  	  		  	  HAL_UART_Transmit(&huart1, tx_buffer, size_message,0xffff);//îòïðàâëÿåì ñîîáùåíèå, ÷åãî íàì òàì ïðèøëî â rx_buffer
	  	  	  		  	  queue_message = 255;//ñòàâèì ñëåäóþùóþ î÷åðåäü, åñëè çàõîòèì åùå ÷òî-òî îòñûëàòü â ïîðò.
	  	  	  	  	  }
	  	  	  }
	  	  else if (flag.flag_send == 1 && queue_message == 255)
	  	  	  {//åñëè æå î÷åðåäü äîøëà äî ìàêñèìàëüíîé, òî
	  	  	  	  queue_message = 1;//ñáðîñèì î÷åðåäü îòïðàâêè, ÷òîá ñîîáùåíèÿ ñíîâà ìîãëè ïî î÷åðåäè îòñûëàòüñÿ
	  	  		  flag.flag_send = 0;//îòïðàâêó ñîîáùåíèé çàïðåòèì. Ðàçðåøåíèå ïîëó÷èì â ïðåðûâàíèè.
	  	  	  }
	  if (flag.TM_flag == 1)
	  {
		  if (led_cnt > 0 && flag.TIM16_flag)
		  {
			  flag.TIM16_flag = 0;
			  led_cnt--;
		  }
		  else if (led_cnt == 0)
			  {
			  flag.TM_flag = 0;
			  HAL_TIM_Base_Stop_IT(&htim16);
			  }
	  }

    /* USER CODE END WHILE */

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;

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
  hadc.Init.Resolution = ADC_RESOLUTION_8B;
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 999;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 4999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  htim16.Init.Prescaler = 999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 249;
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
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 999;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 29;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Bin2Str(uint16_t data, char* pbuf)
{
    //pbuf += 3 - 1;  // Установка указателя буфера на последний (старший)         //
    *pbuf++ = (data / 100)|0x30;   // Вычисление 3 разряда двоично-десятичного числа
    data %= 100;            //
    *pbuf++ = (data / 10)|0x30;    // Вычисление 2 разряда двоично-десятичного числа
    data %= 10;             //
    *pbuf++ = (data / 1)|0x30;     // Вычисление 1 разряда двоично-десятичного числа
    data %= 1;
}
uint16_t pec_Update(uint16_t pec)
{
   static const  uint16_t lookup[256] =
   {
		   0x000, 0x001, 0x003, 0x004, 0x006, 0x008, 0x009, 0x00B,
		   0x00D, 0x00E, 0x010, 0x012, 0x013, 0x015, 0x017, 0x018,
		   0x01A, 0x01C, 0x01D, 0x01F, 0x020, 0x022, 0x024, 0x025,
		   0x027, 0x029, 0x02A, 0x02C, 0x02E, 0x02F, 0x031, 0x033,
		   0x034, 0x036, 0x038, 0x039, 0x03B, 0x03C, 0x03E, 0x040,
		   0x041, 0x043, 0x045, 0x046, 0x048, 0x04A, 0x04B, 0x04D,
		   0x04F, 0x050, 0x052, 0x054, 0x055, 0x057, 0x058, 0x05A,
		   0x05C, 0x05D, 0x05F, 0x061, 0x062, 0x064, 0x066, 0x067,
		   0x069, 0x06B, 0x06C, 0x06E, 0x070, 0x071, 0x073, 0x074,
		   0x076, 0x078, 0x079, 0x07B, 0x07D, 0x07E, 0x080, 0x082,
		   0x083, 0x085, 0x087, 0x088, 0x08A, 0x08C, 0x08D, 0x08F,
		   0x090, 0x092, 0x094, 0x095, 0x097, 0x099, 0x09A, 0x09C,
		   0x09E, 0x09F, 0x0A1, 0x0A3, 0x0A4, 0x0A6, 0x0A8, 0x0A9,
		   0x0AB, 0x0AC, 0x0AE, 0x0B0, 0x0B1, 0x0B3, 0x0B5, 0x0B6,
		   0x0B8, 0x0BA, 0x0BB, 0x0BD, 0x0BF, 0x0C0, 0x0C2, 0x0C4,
		   0x0C5, 0x0C7, 0x0C8, 0x0CA, 0x0CC, 0x0CD, 0x0CF, 0x0D1,
		   0x0D2, 0x0D4, 0x0D6, 0x0D7, 0x0D9, 0x0DB, 0x0DC, 0x0DE,
		   0x0E0, 0x0E1, 0x0E3, 0x0E4, 0x0E6, 0x0E8, 0x0E9, 0x0EB,
		   0x0ED, 0x0EE, 0x0F0, 0x0F2, 0x0F3, 0x0F5, 0x0F7, 0x0F8,
		   0x0FA, 0x0FC, 0x0FD, 0x0FF, 0x100, 0x102, 0x104, 0x105,
		   0x107, 0x109, 0x10A, 0x10C, 0x10E, 0x10F, 0x111, 0x113,
		   0x114, 0x116, 0x118, 0x119, 0x11B, 0x11C, 0x11E, 0x120,
		   0x121, 0x123, 0x125, 0x126, 0x128, 0x12A, 0x12B, 0x12D,
		   0x12F, 0x130, 0x132, 0x134, 0x135, 0x137, 0x138, 0x13A,
		   0x13C, 0x13D, 0x13F, 0x141, 0x142, 0x144, 0x146, 0x147,
		   0x149, 0x14B, 0x14C, 0x14E, 0x150, 0x151, 0x153, 0x154,
		   0x156, 0x158, 0x159, 0x15B, 0x15D, 0x15E, 0x160, 0x162,
		   0x163, 0x165, 0x167, 0x168, 0x16A, 0x16C, 0x16D, 0x16F,
		   0x170, 0x172, 0x174, 0x175, 0x177, 0x179, 0x17A, 0x17C,
		   0x17E, 0x17F, 0x181, 0x183, 0x184, 0x186, 0x188, 0x189,
		   0x18B, 0x18C, 0x18E, 0x190, 0x191, 0x193, 0x195, 0x196,
		   0x198, 0x19A, 0x19B, 0x19D, 0x19F, 0x1A0, 0x1A2, 0x1A4
   };
  pec = lookup[pec];
  return pec;
}
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
