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
#include "NRF24.h"
#include "NRF24_reg_addresses.h"
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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define INTERVAL_MAX 10
#define MAX_SIZE 1024
#define NRF_SIZE 32
#define GGA_SIZE 100

#define TX

uint8_t countTimeout = 0;
uint8_t rover_mode = 0;
uint8_t nrf_rtcm_data[MAX_SIZE];
uint16_t nrf_packet_id = 0;


// RX
volatile uint16_t size = 0;
volatile uint16_t countCallBack = 0;
uint8_t nrf_rx_packet[NRF_SIZE];
uint8_t nrf_gga_data[GGA_SIZE];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	size = Size;
	countCallBack++;

#ifdef TX
	if (size < MAX_SIZE - 2){
		nrf_rtcm_data[size] = '\r';
		nrf_rtcm_data[size+1] = '\n';
	}
#endif

#ifdef RX
	if (size < GGA_SIZE - 2){
		nrf_gga_data[size] = '\r';
		nrf_gga_data[size+1] = '\n';
	}
#endif
}


// TX
uint8_t send_flat = 0;
uint8_t TxData[] = { "$GNGGA,073934.900,1605.841499,N,10804.577336,E,4,38,0.45,35.9,M,-6.0,M,1.0,3335*76\r\n" };
uint8_t GGA_set_command[18] = { 0x24, 0x50, 0x41, 0x49, 0x52, 0x30, 0x36, 0x32, 0x2C, 0x30, 0x2C, 0x30, 0x31, 0x2A, 0x30, 0x46, 0x0D, 0x0A }; //$PAIR062,0,01*0F


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		if (countTimeout <= INTERVAL_MAX)
		{
			countTimeout++;
		}
		else
		{
			countTimeout = 0;

			if (send_flat == 0){
				send_flat = 1;
			}
		}

		// blink led to check rover mode
		switch (rover_mode)
		{
			case '0':
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			break;
			case '4':
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				break;
			case '5':
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				break;
			default:
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				break;
		}
	}
}

int16_t Get_End_RTCM_Index(uint8_t* data, uint16_t size, uint16_t start)
{
	uint8_t d3_count = 0;

	int16_t rtcm_end_id = 0;

	if (start == 0)
		start = 1;

	for (int i = start; i < size - 1; i++){
		if (data[i] == 0xD3 && data[i + 1] == 0x00)
			d3_count++;

		if(d3_count >=5 && data[i-1] == 0x0D && data[i] == 0x0A)
		{
			rtcm_end_id = i + 1;
			break;
		}
	}

	return rtcm_end_id;
}

int16_t Get_End_GGA_Index(uint8_t* data, uint16_t size, uint16_t start)
{
	// check start gga

	int16_t gga_end_id = 0;

	if (start == 0)
		start = 1;

	for (int i = start; i < size; i++){
		if(data[i-1] == 0x0D && data[i] == 0x0A)
		{
			gga_end_id = i + 1;
			break;
		}
	}

	return gga_end_id;
}

uint8_t Send_Large_Data(uint8_t* data, uint16_t size)
{
	uint8_t ret = 0;
	uint8_t chunk[NRF_SIZE];

	for (uint16_t i = 0; i < size; i += NRF_SIZE) {

		uint16_t packet_size = 0;

		if (size - i < NRF_SIZE){
			packet_size = size - i;
		}
		else{
			packet_size = NRF_SIZE;
		}

		memcpy(chunk, data + i, packet_size * sizeof(uint8_t));

		// Send the 32-byte chunk

		//HAL_UART_Transmit(&huart2, chunk, NRF_SIZE, 5);
		ret = nrf24_transmit(chunk, sizeof(chunk));
		while(ret == 1){
			// TODO: timer break while
			HAL_Delay(5);

			ret = nrf24_transmit(chunk, sizeof(chunk));
		}

		memset(chunk, 0, sizeof(chunk));

		// Short delay if necessary to avoid overload
		HAL_Delay(1);
	}
	return ret;
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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  csn_high();
  nrf24_init();
  nrf24_tx_pwr(_0dbm);
  nrf24_data_rate(_1mbps);
  nrf24_set_channel(78);
  nrf24_set_crc(en_crc, _1byte);
  nrf24_pipe_pld_size(0, NRF_SIZE);

  uint8_t addr_base[5] = { 0x10, 0x21, 0x32, 0x43, 0x54 };
  uint8_t addr_rover[5] = { 0x10, 0x21, 0x32, 0x43, 0x53 };

  HAL_Delay(500); // waiting initial Station

#ifdef TX

  nrf24_open_tx_pipe(addr_base);
  nrf24_open_rx_pipe(0, addr_rover);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, nrf_rtcm_data, MAX_SIZE);

#endif

#ifdef RX

  nrf24_open_tx_pipe(addr_rover);
  nrf24_open_rx_pipe(1, addr_base);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, nrf_gga_data, GGA_SIZE);

#endif

  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

#ifdef TX

	  if (size != 0)
	  {
		  int16_t rtcm_end_id = Get_End_RTCM_Index(nrf_rtcm_data, MAX_SIZE, 0);

		  if (rtcm_end_id > 0){
			  // separate data and send chunk
			  uint8_t check[rtcm_end_id]; // to extend to send

			  memcpy(check, nrf_rtcm_data, sizeof(check));

			  nrf24_stop_listen();

			  Send_Large_Data(check, rtcm_end_id);

			  size = 0;

			  memset(nrf_rtcm_data, 0, rtcm_end_id);

			  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, nrf_rtcm_data, MAX_SIZE);

			  HAL_Delay(1);
		  }
	  }

	  nrf24_listen();

	  if (nrf24_data_available()){

		  nrf24_receive(nrf_rx_packet, sizeof(nrf_rx_packet));

		  memcpy(&nrf_gga_data[nrf_packet_id], nrf_rx_packet, sizeof(nrf_rx_packet));

		  nrf_packet_id += NRF_SIZE;

		  memset(nrf_rx_packet, 0, sizeof(nrf_rx_packet));

		  int16_t gga_end_id = Get_End_GGA_Index(nrf_gga_data, GGA_SIZE, 0);

		  if (gga_end_id > 0 && nrf_gga_data[0] == '$'){

			  HAL_UART_Transmit_DMA(&huart2, nrf_gga_data, gga_end_id);

			  memset(nrf_gga_data, 0, sizeof(nrf_gga_data));

			  nrf_packet_id = 0;
		  }

		  if (nrf_packet_id > GGA_SIZE)
		  {
			  nrf_packet_id = 0;
			  memset(nrf_gga_data, 0, sizeof(nrf_gga_data));
		  }

		  HAL_Delay(1);
	  }

#endif

#ifdef RX
	  // NRF Receiver Processing


	  if (size != 0)
	  {
		  int16_t end_gga_index = Get_End_GGA_Index(nrf_gga_data, sizeof(nrf_gga_data), 0);

		  if (end_gga_index > 0){

			  nrf24_stop_listen();

			  Send_Large_Data(nrf_gga_data, end_gga_index);

			  int comma_count = 0;
			  for(int j = 0; j < GGA_SIZE; j++){
				  if (nrf_gga_data[j] == 0x2C){ // 2C is ','
					  comma_count++;
					  if (comma_count == 6) // from $GNGGA to Rover Mode
					  {
						  rover_mode = nrf_gga_data[j + 1];
						  break;
					  }
				  }
			  }

			  size = 0;

			  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, nrf_gga_data, GGA_SIZE);

			  memset(nrf_gga_data, 0, sizeof(nrf_gga_data));
		  }
	  }

	  nrf24_listen();

	  if (nrf24_data_available()){

		  nrf24_receive(nrf_rx_packet, sizeof(nrf_rx_packet));

		  memcpy(&nrf_rtcm_data[nrf_packet_id], nrf_rx_packet, sizeof(nrf_rx_packet));

		  nrf_packet_id += NRF_SIZE;
		  memset(nrf_rx_packet, 0, sizeof(nrf_rx_packet));

		  int16_t end_rtcm_id = Get_End_RTCM_Index(nrf_rtcm_data, MAX_SIZE, 0);
		  if (end_rtcm_id > 0){

			  HAL_UART_Transmit_DMA(&huart2, nrf_rtcm_data, end_rtcm_id);

			  memset(nrf_rtcm_data, 0, sizeof(nrf_rtcm_data));

			  nrf_packet_id = 0;
		  }

		  HAL_Delay(1);
	  }
#endif

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1599;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CE_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
