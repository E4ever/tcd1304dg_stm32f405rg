/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define ADC_BUF_SIZE	8
#define amount_of_pixels 3692

#define fAPB1 60000000 //Частота тактирования шины APB1 (Гц)
#define fM_freq 2000000	//Частота тактирования линейки (Гц)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

volatile uint32_t T_SH = 10000; //Период SH (время интегрирования линейки)
volatile uint32_t T_ICG = 10000; //Период ICG
volatile uint32_t T = 0;

uint8_t CDC_Rx_Buf[11];
//uint8_t CDC_Rx_Buf[11];  //По правильному (т.к. число 4294967296 (2 в 32 степени) имеет 10 символов + символ команды)
uint8_t CDC_Rx_Len = 0;
//uint8_t CDC_Tx_Buf[8];
uint32_t Return_Buf[1];
uint16_t CCD_Out_Buf[amount_of_pixels];
//uint16_t ADC_buffer[ADC_BUF_SIZE];

char *ptr;

volatile uint8_t InReading = 0;
volatile uint8_t NeedToTransmit = 0;
volatile uint8_t PulseCounter = 0;
volatile uint8_t Sent = 1;
//uint16_t exp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */

void TIM_ICG_Conf(uint32_t ICG);
void TIM_SH_Conf(uint32_t SH);
void CCD_Flush(void);
void TIM(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!  MX_DMA_Init(); должна быть перед MX_ADC1_Init(); !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//  CDC_Tx_Buf[0] = 0x01;
//  CDC_Transmit_FS(CDC_Tx_Buf, 8);
  SET_BIT(DMA2_Stream0->CR, DMA_SxCR_PSIZE_0); //Установили размер АЦП 16 бит
  CLEAR_BIT(DMA2_Stream0->CR, DMA_SxCR_PSIZE_1);

//  TIM2 -> CNT = 0x2; //Запись задержки (500 нс) в регистр счетчика TIM2 (для задержки между импульсами SH и ICG)
/*  if(HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
	  Error_Handler();
  if(HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4) != HAL_OK)
  	  Error_Handler();
  if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
  	  Error_Handler();
  if(HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) != HAL_OK)
  	  Error_Handler();*/

/*  TIM3 -> CNT = 0x8; //Запись задержки (500 нс) в регистр счетчика TIM2 (для задержки между импульсами SH и ICG)
  TIM5 -> CNT = 0x5;
  TIM2 -> CNT = 0x1;
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);*/
//  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
//  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4);

  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);

/*  CCD_Flush();
  while (InReading == 1); //Ожидание окончания преобразования АЦП + ДМА
  HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
  TIM_SH_Conf(T_SH);
  TIM_ICG_Conf(T_ICG);
  TIM2 -> CNT = 0x1;
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (CDC_Rx_Len > 0){
		  switch (CDC_Rx_Buf[0]){
		  	  case 0x65:  //e

		  		  /* Если время интегрирования с ПК приходит в миллисекундах, тогда максимальное число должно быть 4294967, а если в микросекундах, то 4294967295 */
		  		  /* T_SH */ T = strtoul((const char *)CDC_Rx_Buf+1, &ptr, 10); //Преобразование строковых элементов массива в 10-ный формат, причем *ptr - элемент массива, следующий за последним преобразованным элементом, а 10 - система счисления
		  		  T_SH = T * 1000;  //Нужно, если время интегрирования с ПК приходит в миллисекундах

		  		  if (T_SH < 10){
		  			  T_SH = 10;
		  		  }

		  		  if (T_SH <= 7500){
		  			  T_ICG = T_SH;
		  			  while (T_ICG < 7500){
		  				  T_ICG += T_SH;
		  			  }
		  		  } else {
		  			  T_ICG = T_SH;
		  		  }

		  		  Return_Buf[0] = T_SH;
//		  		  Return_Buf[0] = T;  //Нужно, если время интегрирования с ПК приходит в миллисекундах

		  		  while (InReading == 1); //Ожидание окончания преобразования АЦП + ДМА

		  		  CCD_Flush();
//		  		  PulseCounter = 0;

//		      	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
//		  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

		  		  while (InReading == 1); //Ожидание окончания преобразования АЦП + ДМА

/*		  		  CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN);
		  		  CLEAR_BIT(TIM5->CR1, TIM_CR1_CEN);
		  		  TIM2 -> CNT = 0x00000000;
		  		  TIM5 -> CNT = 0x00000000;*/

		  		  HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_2);
		  		  HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
//		  		  HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_1);

//		  		  Error_Handler();

		  		  TIM_SH_Conf(T_SH);
		  		  TIM_ICG_Conf(T_ICG);
//		  		  MX_TIM3_Init();

//		  		  TIM3 -> CNT = 0x8;
//		  		  TIM5 -> CNT = 0x1;
		  		  TIM2 -> CNT = 0x1;

/*		  		  SET_BIT(TIM2->CR1, TIM_CR1_CEN);
		  		  SET_BIT(TIM5->CR1, TIM_CR1_CEN);*/

		  		  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
		  		  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
//		  		  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);

		  		  CDC_Transmit_FS(Return_Buf, 4);

		  		  memset(CDC_Rx_Buf, 0, sizeof(CDC_Rx_Buf));     //обнуление значений элементов буфера

		  		  break;

		  	  case 0x72:  //r
		  		  NeedToTransmit = 1;
		  		  memset(CDC_Rx_Buf, 0, sizeof(CDC_Rx_Buf));     //обнуление значений элементов буфера
//		  	      HAL_ADC_Start_DMA(&hadc1, (uint32_t*)CCD_Out_Buf, 3694);
//		  	      CDC_Transmit_FS(CCD_Out_Buf, 7388);
/*		  	      if (flag){
		  	    	  flag = 0;
		  	    	  HAL_ADC_Stop_DMA(&hadc1);
		  	    	  CDC_Transmit_FS(CCD_Out_Buf, 3694);
		  	      }*/
		  	      break;

/*		  	  case 0x61:
		  		  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_buffer, ADC_BUF_SIZE) != HAL_OK)
		  			  Error_Handler();
		  		  break;

		  	  case 0x6C:
		  		  HAL_Delay(500);
		  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		  		  HAL_Delay(500);
		  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		  		  break;

		  	  case 0x6D:
		  		  Error_Handler();
		  		  break;

		  	  case 0x72:
		  		  if(READ_BIT(DMA2_Stream0->CR, DMA_SxCR_PSIZE_0)){
		  			 HAL_Delay(500);
		  			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		  			 HAL_Delay(250);
		  			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		  			 HAL_Delay(250);
		  			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		  			 HAL_Delay(250);
		  			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		  		  }
		  		  break;

		  	  case 0x74:
//		  		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		  		  HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_1);
		  		  break;*/
		  }



/*
		  if (SH_time == 3){
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			  HAL_Delay(200);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		  }
*/

/*		  if (CDC_Rx_Buf[0] == 0x01 && CDC_Rx_Buf[1] != 0x02){
			  HAL_Delay(2000);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			  HAL_Delay(200);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			  CDC_Transmit_FS(CDC_Tx_Buf, 8);
		  } else if (CDC_Rx_Buf[0] == 0x01 && CDC_Rx_Buf[1] == 0x02){
			  HAL_Delay(2000);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			  HAL_Delay(200);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			  HAL_Delay(200);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			  HAL_Delay(200);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			  HAL_Delay(200);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			  HAL_Delay(200);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		  } else if (CDC_Rx_Buf[0] == 0x02){
			  HAL_Delay(2000);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			  HAL_Delay(200);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			  HAL_Delay(200);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			  HAL_Delay(200);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		  }
*/
		  CDC_Rx_Len = 0;
	  }

	  if (NeedToTransmit == 1 && Sent == 0){
		  	  	  while(PulseCounter < 4);
	  			  NeedToTransmit = 0;
	  			  Sent = 1;
	  			  CLEAR_BIT(TIM2 -> DIER, TIM_DIER_CC2IE);		//Выключить прерывания по подьему сигнала ICG
	  			  while (InReading == 1); //Ожидание окончания преобразования АЦП + ДМА
	  			  CDC_Transmit_FS(CCD_Out_Buf, 2 * amount_of_pixels);

//	  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
//	  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	  			  CLEAR_BIT(TIM2 -> SR, TIM_SR_CC2IF);	//сбросили флаг прерывания
	  			  SET_BIT(TIM2 -> DIER, TIM_DIER_CC2IE);		//Включить прерывания по подьему сигнала ICG
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_CC4;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 29;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 14799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 14;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 59;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 29;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 19;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 4;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void TIM(){

}

void TIM_ICG_Conf(uint32_t ICG)
{
	//------------------TIM2(ICG)---------------------
//	TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = fAPB1 / fM_freq - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = ICG * (fM_freq / 1000000) - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
/*	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
	  Error_Handler();
	}*/
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
	  Error_Handler();
	}
/*	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	sSlaveConfig.InputTrigger = TIM_TS_ITR2;
	if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
	{
	  Error_Handler();
	}*/
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = 5 * fM_freq / 1000000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
	  Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim2);
}

void TIM_SH_Conf(uint32_t SH)
{
	//------------------TIM5(SH)---------------------
	TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim5.Instance = TIM5;
	htim5.Init.Prescaler = fAPB1 / fM_freq - 1;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = SH * (fM_freq / 1000000) - 1;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	{
	  Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
	{
	  Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = 2 * fM_freq / 1000000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
	  Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim5);
}

void CCD_Flush()
{
	HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
//	HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_1);

	PulseCounter = 0;

	TIM_SH_Conf(10);
	TIM_ICG_Conf(7500);
//	MX_TIM3_Init();

//	TIM5 -> CNT = 0x1;
	TIM2 -> CNT = 0x1;

	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
//	HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);

	while(PulseCounter < 1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc -> Instance == ADC1) //check if the interrupt comes from ADC1
    {
    	InReading = 0;
    	Sent = 0;
//    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
//    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
/*
    	if(HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
    		Error_Handler();
*/
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
//        if(htim->Instance == TIM2)
//        {
//        	ADC_Conversion = 1;
        	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)CCD_Out_Buf, amount_of_pixels);
        	InReading = 1;
        	PulseCounter++;
        	if (PulseCounter == 255)
        		PulseCounter = 3;
//        }
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
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
