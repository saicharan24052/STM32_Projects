/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include<stdio.h>
#include<stdbool.h>
#define WREG 0x00
#define RREG 0x01
static bool ecgFIFOIntFlag = 0;
/*
volatile bool ecgFIFOIntFlag = 0;
void ecgFIFO_callback()  {

    ecgFIFOIntFlag = 1;

}
*/
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == INT_Pin)
	{

		if(ecgFIFOIntFlag ==0)
	{
			ecgFIFOIntFlag =1;
	}

	}
}


/* USER CODE END Includes */
uint32_t FIFO, readECGSamples, idx, ETAG[32], status,RtoR;
int16_t ecgSample[32];
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

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar (int ch)
	{

	HAL_UART_Transmit(&huart3,((uint8_t *)&ch),1,100);
	return ch;
	}


float convert_ecg_value(uint8_t *spi_buf) {
    int32_t raw_value = (spi_buf[0] << 16) | (spi_buf[1] << 8) | spi_buf[2]; // Concatenate received bytes to form 24-bit integer
    return raw_value / 1.0; // Divide by 2^24 (or 16777216) to obtain voltage value between 0 and 1
}

 void RegWrite(SPI_HandleTypeDef *hspi, uint8_t WRITE_ADDRESS, uint32_t data)
{
	uint8_t txBuffer[4];
	HAL_GPIO_WritePin(MAX30003_CS_GPIO_Port, MAX30003_CS_Pin, GPIO_PIN_RESET);
	txBuffer[0] = (WRITE_ADDRESS << 1) | WREG;
	txBuffer[1] = (data >> 16);
	txBuffer[2] = (data >> 8);
	txBuffer[3] = data;
	HAL_SPI_Transmit(&hspi1, txBuffer, 4, HAL_MAX_DELAY);

	// Set the chip select pin to high
	HAL_GPIO_WritePin(MAX30003_CS_GPIO_Port, MAX30003_CS_Pin, GPIO_PIN_SET);
}

  uint32_t  RegRead(SPI_HandleTypeDef *hspi, uint8_t READ_ADDRESS, uint8_t *rx1Buffer)
{

	uint8_t tx1Buffer[4];
	tx1Buffer[0] = (READ_ADDRESS << 1) | RREG;
	  tx1Buffer[1] = 0x00;
	  tx1Buffer[2] = 0x00;// Dummy byte to receive data
	  tx1Buffer[3] = 0x00;
	  HAL_GPIO_WritePin(MAX30003_CS_GPIO_Port, MAX30003_CS_Pin, GPIO_PIN_RESET);

	  // Send the buffer over SPI
	  HAL_SPI_TransmitReceive(&hspi1, tx1Buffer, rx1Buffer, 4, HAL_MAX_DELAY);

	  // Set the chip select pin to high
	  HAL_GPIO_WritePin(MAX30003_CS_GPIO_Port, MAX30003_CS_Pin, GPIO_PIN_SET);

	  uint32_t val0 = (uint32_t)(rx1Buffer[0]);
	 	  		 	  val0 = val0 <<24;
	 	  		 	 uint32_t val1 = (uint32_t)(rx1Buffer[1]);
	 	  		 	  val1 = val1 <<16;
	 	  		 	 uint32_t val2 =(uint32_t)(rx1Buffer[2]);
	 	  		 	  val2 = val2 << 8;
	 	  		 	  //val2 = val2 & 0x03;
	 	  		 	uint32_t  val3 = (uint32_t)(rx1Buffer[3]);
	 	  		 	val3 = val3 << 0;
	 	  		 	 uint32_t val = (uint32_t) (val0 | val1 | val2 | val3);

	  return val;
}






int main(void)
{
	 // Constants
	    const int EINT_STATUS_MASK =  1 << 23;
	    const int FIFO_OVF_MASK =  0x7;
	    const int FIFO_VALID_SAMPLE_MASK =  0x0;
	    const int FIFO_FAST_SAMPLE_MASK =  0x1;
	    const int ETAG_BITS_MASK = 0x7;
	    const int RTOR_STATUS =  1 << 10;
	        const int RTOR_REG_OFFSET = 10;
	        const float RTOR_LSB_RES = 0.0078125f;



  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  RegWrite(&hspi1,0x08 ,0x00);
  RegWrite(&hspi1,0x10 ,0x80213);
  	  RegWrite(&hspi1,0x15 ,0x835000); // 0x435000 for 285 samples per sec ; 0x835000 for 125 s/s //0x805000 for 20v/v  //435000
  	  RegWrite(&hspi1,0x14 ,0x00);
  	  RegWrite(&hspi1,0x1D ,0x3FB300); //0X1FD180
  	RegWrite(&hspi1,0x04,0x180014); // mng int
  	RegWrite(&hspi1,0x02 ,0xC00403); // int
  	 RegWrite(&hspi1,0x05 ,0x3F0000); // Disable fast recovery
  RegWrite(&hspi1,0x09 ,0x00);// sync


 // RegWrite(&hspi1,0x08 ,0x00); //sys reset


  while (1)
   {
	  HAL_GPIO_EXTI_Callback(INT_Pin);
	  uint8_t rxBuffer_status[4], rxBuffer_FIFO[4],rxBuffer_rtor[4];
	  float BPM;

	  if( ecgFIFOIntFlag ) {

	              ecgFIFOIntFlag = 0;
	               status =  RegRead(&hspi1,0x01, rxBuffer_status);

	               //BPM


	               // Check if R-to-R interrupt asserted
	               if( ( status & RTOR_STATUS ) == RTOR_STATUS ){

	                 //  printf("R-to-R Interrupt \r\n");

	                   // Read RtoR register
	                   RtoR = RegRead( &hspi1,0x25, rxBuffer_rtor) >>  RTOR_REG_OFFSET;

	                   // Convert to BPM
	                   BPM = 1.0f / ( RtoR * RTOR_LSB_RES / 60.0f );

	                 //  printf("Current BPM is %3.2f\r\n\r\n", BPM);
	                //  printf("RtoR : %ld\r\n\r\n", RtoR);
	               }

	            //   printf(" test\r\n");
	              // Check if EINT interrupt asserted
	                          if ( ( status & EINT_STATUS_MASK ) == EINT_STATUS_MASK ) {

	                              readECGSamples = 0;                        // Reset sample counter

	                              do {
	                                  FIFO = RegRead(&hspi1,0x21, rxBuffer_FIFO);       // Read FIFO  0x21
	                                  ecgSample[readECGSamples] = FIFO >> 8;                  // Isolate voltage data
	                                  ETAG[readECGSamples] = ( FIFO >> 3 ) & ETAG_BITS_MASK;  // Isolate ETAG
	                                  readECGSamples++;                                          // Increment sample counter

	                              // Check that sample is not last sample in FIFO
	                              } while ( ETAG[readECGSamples-1] == FIFO_VALID_SAMPLE_MASK ||
	                                        ETAG[readECGSamples-1] == FIFO_FAST_SAMPLE_MASK );

	                              // Check if FIFO has overflowed
	                              if( ETAG[readECGSamples - 1] == FIFO_OVF_MASK ){
	                            	  RegWrite(&hspi1,0x0A ,0x00); // Reset FIFO 0x0A

	                              }
	                              for( idx = 0; idx < readECGSamples; idx++ ) {
	                                                printf("%d\r\n",ecgSample[idx]);

	                              }

	  		 //	uint32_t Nmngint =  RegRead(&hspi1,0x01, rxBuffermngint);
	  		//  uint8_t rxBufferstatus[4];
	  			  		 //	uint32_t status =  RegRead(&hspi1,0x01, rxBufferstatus);
	  			  		 //	uint32_t Nstatus = status | 0x180014;
	  		 //	uint32_t   ecgdata = (uint32_t) (val);



//HAL_Delay(10);
 //   printf("0x%02x  spi_rx_buf[0]\r\n", (uint8_t)rxBuffer[0]);
 //   printf("0x%02x  spi_rx_buf[1]\r\n", (uint8_t)rxBuffer[1]);
 //  printf("0x%02x  spi_rx_buf[2]\r\n", (uint8_t)rxBuffer[2]);
 //  printf("0x%02x  spi_rx_buf[3]\r\n", (uint8_t)rxBuffer[3]);
   //printf("spi_tx_buf[0]: 0x%02x\r\n", (uint8_t)rxBuffer[0]);


   //  HAL_Delay(50);
   }
 }
	 	//printf(" status %lu\r\n",status);
   }
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAX30003_CS_GPIO_Port, MAX30003_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MAX30003_CS_Pin */
  GPIO_InitStruct.Pin = MAX30003_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MAX30003_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_Pin */
  GPIO_InitStruct.Pin = INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

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
