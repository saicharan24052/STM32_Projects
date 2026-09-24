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
#include <string.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
int __io_putchar (int ch)
	{

	HAL_UART_Transmit(&huart3,((uint8_t *)&ch),1,100);
	return ch;
	}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
uint8_t VL6180X_ADDRESS =0x29 << 1; // Use 8-bit address  0x29 << 1   0x52
uint8_t REG_DIST = 0x18;
uint8_t REG_RES = 0x04D;
/**
  * @brief  The application entry point.
  * @retval int
  */
void WriteReg(uint16_t regAddr, uint8_t data) {
	HAL_StatusTypeDef rett;
    uint8_t buffer[3];
    buffer[0] = (regAddr >>8) & 0xFF;
    buffer[1] = regAddr & 0xFF;
    buffer[2] = data;

    // Send the register address and data to write

    rett = HAL_I2C_Master_Transmit(&hi2c1, VL6180X_ADDRESS, buffer, 3, 100);
		 if ( rett != HAL_OK ) {
				  	   	    	         printf( "Error at write reg \r\n");
				  	   	    	          }
}
char ReadReg(uint16_t regAddr) {
	HAL_StatusTypeDef rett1;
	HAL_StatusTypeDef rett2;
    // Split the 16-bit register address into two bytes
    uint8_t data_write[2];
    uint8_t data_read[1];
    data_write[0] = (regAddr >> 8) & 0xFF; // MSB of register address
    data_write[1] = regAddr & 0xFF;        // LSB of register address

    // Make sure you have initialized the I²C communication peripheral (hi2c) before using it
    // Replace "hi2c" with the actual name of your I²C handle variable

    rett1=  HAL_I2C_Master_Transmit(&hi2c1, VL6180X_ADDRESS, data_write, 2, HAL_MAX_DELAY);
    if ( rett1 != HAL_OK ) {
    				  	   	    	         printf( "Error at Read reg transmit \r\n");
    				  	   	    	          }

    rett2 =HAL_I2C_Master_Receive(&hi2c1, VL6180X_ADDRESS, data_read, 1, HAL_MAX_DELAY);
    if ( rett2 != HAL_OK ) {
    				  	   	    	         printf( "Error at Read reg receive \r\n");
    				  	   	    	          }
    return (char)data_read[0];
}
int VL6180X_Init() {
	   char reset;
	   reset = ReadReg(0x016);
	   if (reset==1){ // check to see has it be Initialised already

	  // Added latest settings here - see Section 8
		   WriteReg(0x0207, 0x01);
		  	  WriteReg(0x0208, 0x01);
		  	  WriteReg(0x0096, 0x00);
		  	  WriteReg(0x0097, 0xfd);
		  	  WriteReg(0x00e3, 0x00);
		  	  WriteReg(0x00e4, 0x04);
		  	  WriteReg(0x00e5, 0x02);
		  	  WriteReg(0x00e6, 0x01);
		  	  WriteReg(0x00e7, 0x03);
		  	  WriteReg(0x00f5, 0x02);
		  	  WriteReg(0x00d9, 0x05);
		  	  WriteReg(0x00db, 0xce);
		  	  WriteReg(0x00dc, 0x03);
		  	  WriteReg(0x00dd, 0xf8);
		  	  WriteReg(0x009f, 0x00);
		  	  WriteReg(0x00a3, 0x3c);
		  	  WriteReg(0x00b7, 0x00);
		  	  WriteReg(0x00bb, 0x3c);
		  	  WriteReg(0x00b2, 0x09);
		  	  WriteReg(0x00ca, 0x09);
		  	  WriteReg(0x0198, 0x01);
		  	  WriteReg(0x01b0, 0x17);
		  	  WriteReg(0x01ad, 0x00);
		  	  WriteReg(0x00ff, 0x05);
		  	  WriteReg(0x0100, 0x05);
		  	  WriteReg(0x0199, 0x05);
		  	  WriteReg(0x01a6, 0x1b);
		  	  WriteReg(0x01ac, 0x3e);
		  	  WriteReg(0x01a7, 0x1f);
		  	  WriteReg(0x0030, 0x00);


		  	  // Recommended : Public registers - See data sheet for more detail
		  	  WriteReg(0x0011, 0x10); // Enables polling for ‘New Sample ready’ when measurement completes
		  	  WriteReg(0x010a, 0x30); // Set the averaging sample period (compromise between lower noise and increased execution time)
		  	  WriteReg(0x003f, 0x46); // Sets the light and dark gain (upper nibble). Dark gain should not be changed.
		  	  WriteReg(0x0031, 0xFF); // sets the # of range measurements after which auto calibration of system is performed
		  	  WriteReg(0x0040, 0x63); // Set ALS integration time to 100ms
		  	  WriteReg(0x002e, 0x01); // perform a single temperature calibration of the ranging sensor
		  	 // Optional: Public registers - See data sheet for more detail
		  	  WriteReg(0x001b, 0x09); // Set default ranging inter-measurement period to 100ms
		  	  WriteReg(0x003e, 0x31); // Set default ALS inter-measurement period to 500ms
		  	  WriteReg(0x0014, 0x24); // Configures interrupt on ‘New Sample
		  	   // Ready threshold event’
		   WriteReg(0x016, 0x00); //change fresh out of set status to 0
	   }
	   return 0;
	  }
	  // Start a range measurement in single shot mode

	  int VL6180X_Start_Range() {
		  WriteReg(0x018,0x03);
	   return 0;
	  }
	  ///////////////////////////////////////////////////////////////////
	  // poll for new sample ready ready
	  ///////////////////////////////////////////////////////////////////
	  int VL6180X_Poll_Range() {
	   char status;
	   char range_status;

	   // check the status
	   status = ReadReg(0x04f);
	   range_status = status & 0x07;

	   // wait for new measurement ready status
	   while (range_status != 0x04) {
	   status = ReadReg(0x04f);
	   range_status = status & 0x07;
	   HAL_Delay(1); // (can be removed)
	   }
	   return 0;
	  }
	  // Read range result (mm)

	  int VL6180X_Read_Range() {
		  int range;
		   range=ReadReg(0x062);
		   return range;
		  }

		  // clear interrupts

		  int VL6180X_Clear_Interrupts() {
		   WriteReg(0x015,0x07);
		   return 0;
		  }
int main(void)
{
  /* USER CODE BEGIN 1 */

		//  int16_t val;
		//  float distance;
		  int range;

		   // load settings onto VL6180X

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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  printf("hello  \r\n");
  HAL_Delay(1000);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  VL6180X_Init();
 		   VL6180X_Start_Range();
  while (1)
    {
	  // start single range measurement


	   // poll the VL6180X till new sample ready
	   VL6180X_Poll_Range();

	   // read range result
	   range = VL6180X_Read_Range();

	   // clear the interrupt on VL6180X
	   VL6180X_Clear_Interrupts();

	   // send range to pc by serial
	   printf("%d mm\r\n", range);
	   HAL_Delay(100);

    /* USER CODE END 3 */
  }
  }

/**
  * @brief System Clock Configuration
  * @retval None
  */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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
  hi2c1.Init.Timing = 0x10707DBC;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
