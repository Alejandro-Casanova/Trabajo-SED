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
#include <string.h>
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

/* USER CODE BEGIN PV */
uint8_t slave_address_write = 0x78;
uint8_t slave_address_read = 0x78 | 0x01;

bool COM_ERROR = false;
#define alto_display 64
#define ancho_display 128
#define n_paginas 8
uint8_t buffer[n_paginas * ancho_display];

uint8_t i2cTxBuf[2];
uint8_t i2cRxBuf;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
bool oled_command(uint8_t command);
void oled_init();
void alimentacion(bool enable);
void clear(bool enable);
void display();
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


  oled_init();


  i2cTxBuf[0] = 0x00; //Command
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  i2cTxBuf[1] = 0xA4;
	  if(HAL_I2C_Master_Transmit(&hi2c1, slave_address_write, i2cTxBuf, 2, HAL_MAX_DELAY) != HAL_OK){
		  COM_ERROR = true;
	  }
	  HAL_Delay(500);
	  i2cTxBuf[1] = 0xA5;
	  HAL_I2C_Master_Transmit(&hi2c1, slave_address_write, i2cTxBuf, 2, HAL_MAX_DELAY);
	  HAL_Delay(500);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 240;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
bool oled_command(uint8_t command){
	i2cTxBuf[0] = 0x00;
	i2cTxBuf[1] = command;
	if(HAL_I2C_Master_Transmit(&hi2c1, slave_address_write, i2cTxBuf, 2, HAL_MAX_DELAY) == HAL_OK)
		return true;
	else
		return false;
}
void oled_init(){
	uint8_t init_buffer[] = {
			0x00, //Byte de control, indica que sólo siguen comandos
			0xAE, //Apaga el display
			0xD5, //Establece el divisor de reloj y freq del oscilador
			0x80, //(foscilador +15%)
			0xA8, //multiplex ratio
			alto_display - 1,
			0xD3, //Display Offset
			0x00, //(sin offset)
			0x40, //start line address = 0 (para el scrolling)
			0x8D, //charge pump X
			0x14, //internalX
			0x20, //Memory addressing mode X
			0x00, //(Horizontal) X
//			0x32, //Set pump voltage (8V)
//			0xAD, //DC-DC Set
//			0x8B, //(DC-DC On)
			0xA1, //Segment remap
			0xC8, //Common Output Scan Direction
			0xDA, //Common pads hardware configuration
			0x12, //(Alternative)
			0x81, //Contrast Control
			0xCF,
			0xD9, //Pre-charge period
			0xF1,
			0xDB, //Vcomh deselect level
			0x40,
			0xA4, //Normal RAM display out
			0x2E, //Stop Scrolling X
			0xA6 //Display mode Normal
	};/*
	for(int i = 0; i < sizeof(init_buffer); i++){
		if(!oled_command(init_buffer[i]))
			COM_ERROR = true;
	}*/

	if (HAL_I2C_Master_Transmit(&hi2c1, slave_address_write, init_buffer, sizeof(init_buffer), HAL_MAX_DELAY) != HAL_OK){
		COM_ERROR = true;
	}

	clear(false);
	display();
	alimentacion(true);

}
void alimentacion(bool enable){
	uint8_t power_on[] = {
			0x00, //Command
			0x8D, 0x14, //DC-DC ON
			0xAF //Display ON
	};
	uint8_t power_off[] = {
			0x00, //Command
			0xAE, //Display OFF
			0x8D, 0x10, //DC-DC OFF
	};
	uint8_t send[sizeof(power_off)];

	if(enable)
		memcpy(send, power_on, sizeof(send));
	else
		memcpy(send, power_off, sizeof(send));


	if(HAL_I2C_Master_Transmit(&hi2c1, slave_address_write, send, sizeof(send), HAL_MAX_DELAY) != HAL_OK){
		COM_ERROR = true;
	}
	/*
	for(int i = 0; i < sizeof(send); i++){
		if(!oled_command(send[i]))
			COM_ERROR = true;
	}
	*/
	HAL_Delay(100);
}

void clear(bool enable){
	if(enable)
		memset(buffer, 0xFF, sizeof(buffer));
	else
		memset(buffer, 0x00, sizeof(buffer));
}

void display(){
	uint16_t i = 0;
	uint8_t data_buffer[ancho_display + 1];
	data_buffer[0] = 0x40; //Datos
	for(uint8_t pagina = 0; pagina < n_paginas; pagina++){
		uint8_t command_buffer[] = {
				0x00,
				0xB0 + pagina, //Establece página
				0x00, //Lower colum address
				0x10 //Upper colum address
		};
		if(HAL_I2C_Master_Transmit(&hi2c1, slave_address_write, command_buffer, sizeof(command_buffer), HAL_MAX_DELAY) != HAL_OK)
			COM_ERROR = true;

		memcpy(&data_buffer[1], &buffer[i], ancho_display);
		i += ancho_display;
		if(HAL_I2C_Master_Transmit(&hi2c1, slave_address_write, data_buffer, sizeof(data_buffer), HAL_MAX_DELAY) != HAL_OK)
					COM_ERROR = true;

	}
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
