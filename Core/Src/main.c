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
#include <stdio.h>
#include <stdlib.h>
//#include <time.h>

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

// VARIABLES DISPLAY
//Direcciones I2C y buffers
uint8_t slave_address_write = 0x78;
uint8_t slave_address_read = 0x78 | 0x01;
uint8_t i2cTxBuf[2];
uint8_t i2cRxBuf;
bool COM_ERROR = false; //Indica error de comunicación I2C con el display
bool UART_ERROR = false; //Indica error de envío UART

#define alto_display 64
#define ancho_display 128
#define n_paginas 8 //El direccionamiento de las celdas SRAM se hace mediante páginas (8 páginas de 128 x 8 bits)
uint8_t buffer[n_paginas * ancho_display]; //Información que se cargará en la SRAM del display

//VARIABLES JUEGO
uint8_t map_layout[ancho_display][alto_display]; //Matriz 2D del mapa del juego, implementado de esta forma por
												 //comodidad. Cada entero almacenará '0' o '1' para indicar bit
												 //encendido o apagado
volatile bool refresh = false; //Actualizado por interrupciones de teporizador, indicará que el juego debe avanzar un
				      //frame y que el display debe ser actualizado

//Cuerpo de la serpiente
#define LONGITUD_INICIAL 10
#define LONGITUD_MAXIMA 99
struct modulo{ //La serpiente estará compuesta por módulos, las frutas serán módulos individuales
	uint8_t x;
	uint8_t y;
}typedef modulo;
modulo serpiente[LONGITUD_MAXIMA]; //Cuerpo de la serpiente
uint8_t s_Longitud = LONGITUD_INICIAL; //Longitud actual. Este valor será enviado a la FPGA como "puntuación"

//Frutas para comer por la serpiente
#define N_FRUTAS 20 //Frutas que habrá en el mapa en todo momento
modulo frutas[N_FRUTAS]; //Contiene todas las frutas actuales

//Dirección de la serpiente
#define s_DERECHA 'r'
#define s_IZQUIERDA 'i'
#define s_ARRIBA 'u'
#define s_ABAJO 'd'
#define s_SENTIDO_INICIAL s_ARRIBA
uint8_t s_Sentido = s_SENTIDO_INICIAL; //Almacena el estado de la serpiente (en qué dirección se está moviendo).

//VARIABLES BOTON
volatile bool boton = false;

//VARIABLES POTENCIÓMETRO
uint32_t adc_buffer = 0;
uint8_t pot_read = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM7_Init(void);
static void MX_RNG_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

//FUNCIONES OLED
bool oled_command(uint8_t command); //Envía un comando simple al display (NO UTILIZADO)
void oled_init(); //Inicialización Obligatoria
void alimentacion(bool enable); //Enciende o apaga el display
void clear(bool enable); //Carga todos los valores del buffer con un mismo valor
void display(); //Carga el buffer en la SRAM del display
void dibuja_pixel(uint8_t x, uint8_t y, bool borra); //Dibuja/Borra un sólo pixel del buffer
//void flash_screen(); //Hace parpadear la pantalla (Todo ON/Valores SRAM)
void flashScreen(); //Hace parpadear la pantalla (invertida/normal) 4 veces
void setContrast(uint8_t value); //Define el contraste del display, "value" entre 0 y 255

//FUNCIONES JUEGO
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim); //Callback encargada de avanzar el juego y
															 //refrescar la pantalla cada 50ms (20Hz)
void clearLayout(); //Limpia el layout
void setLayout(); //Dibuja las paredes y BORRA LO DEM�?S
void init_serpiente(); //Inicializa la serpiente en posicion inicial
void init_frutas(); //Inicializa las posiciones iniciales de las frutas (debe ejecutarse después de setSerpiente)
void lanzaFruta(); //Posiciona una fruta en un lugar aleatorio
void setSerpiente(); //Dibuja la serpiente en el layout
void setFrutas(); //Dibuja las frutas en el layout
void transferMapToBuffer(); //Carga el mapa en el buffer que luego se envía al display
bool giraSerpiente(uint8_t indicacion); //Cambia la dirección de la serpiente (Hay macros con cada dirección)
										//Devuelve false si no se pudo ejecutar el giro
void giraDerecha(); //Gira la serpiente 90º a la derecha
void giraIzquierda(); //Gira la serpiente 90º a la izquierda
void avanzaSerpiente(); //Hace avanzar la serpiente una casilla (en el layout)
int colision(uint8_t x, uint8_t y); //Devuelve 1 si la colisión es con una fruta, 2 con pared,
									//-1 si es la propia serpiente y 0 si no hay colisión
void gameOver();
void initGame();

//FUNCIONES EXTI
bool debounce(volatile bool* boton);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);

	if (GPIO_Pin == GPIO_PIN_0)
			boton = true;
	else if (GPIO_Pin == GPIO_PIN_1)
		giraDerecha();
	else if (GPIO_Pin == GPIO_PIN_2)
		giraIzquierda();

	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

//FUNCIONES ADC Y DMA
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc){
	if (hadc->Instance == ADC1){
		pot_read = adc_buffer;;
	}
}

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
  MX_TIM7_Init();
  MX_RNG_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  oled_init(); //INICIALIZACIÓN OBLIGATORIA
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  initGame();
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_ADC_Start_DMA(&hadc1, &adc_buffer, 1);
  uint8_t last_pot_read = pot_read;
  if(HAL_HalfDuplex_EnableTransmitter(&huart4) != HAL_OK)
	  UART_ERROR = true;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t count = HAL_GetTick();
	  while(HAL_GetTick() - count < 120000){ //Se apaga automáticamente para ahorrar energía,
		  	  	  	  	  	  	  	  	  	 //cuando termina el bucle
		  //Avanza el juego 1 "frame"
		  if(refresh){
			  avanzaSerpiente();
			  setLayout();
			  setSerpiente();
			  setFrutas();
			  transferMapToBuffer();
			  display();
			  refresh = false;
		  }

		  //Gira si el jugador lo indica (pulsador del micro)
		  if(debounce(&boton))
			  giraDerecha();

		  //Ajusta el contraste si el valor del potenciómetro ha cambiado
		  if(abs(last_pot_read - pot_read) > 5){
			  HAL_ADC_Stop_DMA(&hadc1);
			  last_pot_read = pot_read;
			  setContrast(pot_read);
			  HAL_ADC_Start_DMA(&hadc1, &adc_buffer, 1);
		  }
	  }
	  HAL_Delay(5000);
	  alimentacion(false); //Apaga el display al salir del bucle
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 15999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 49;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
	if (htim->Instance == TIM7)
			refresh = true;
}

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
void dibuja_pixel(uint8_t x, uint8_t y, bool borra){
	if (x >= ancho_display || y >= alto_display)
		return;
	if(!borra)
		buffer[x + (y / 8) * ancho_display] |= (1 << (y & 7)); // set bit
	else
		buffer[x + (y / 8) * ancho_display] &= ~(1 << (y & 7)); // clear bit
}
/*void flash_screen(){
	i2cTxBuf[0] = 0x00; //Command
	i2cTxBuf[1] = 0xA4;
	if(HAL_I2C_Master_Transmit(&hi2c1, slave_address_write, i2cTxBuf, 2, HAL_MAX_DELAY) != HAL_OK){
	  COM_ERROR = true;
	}
	HAL_Delay(500);
	i2cTxBuf[1] = 0xA5;
	HAL_I2C_Master_Transmit(&hi2c1, slave_address_write, i2cTxBuf, 2, HAL_MAX_DELAY);
	HAL_Delay(500);
}*/

void flashScreen(){
	uint8_t flash_on[] = {
		0x00, //Command
		0xA7 //Inverse Display
	};
	uint8_t flash_off[] = {
		0x00, //Command
		0xA6 //Normal Display
	};

	for(int i = 0; i < 4; i++){
		if (HAL_I2C_Master_Transmit(&hi2c1, slave_address_write, flash_on, sizeof(flash_on), HAL_MAX_DELAY) != HAL_OK){
				COM_ERROR = true;
		}
		HAL_Delay(250);
		if (HAL_I2C_Master_Transmit(&hi2c1, slave_address_write, flash_off, sizeof(flash_off), HAL_MAX_DELAY) != HAL_OK){
				COM_ERROR = true;
		}
		HAL_Delay(250);
	}
}

void setContrast(uint8_t value){
	uint8_t contrast_config[] = {
			0x00, //Command
			0x81, //Contrast Control (0x81 en ssd1306)
			value
	};

	if (HAL_I2C_Master_Transmit(&hi2c1, slave_address_write, contrast_config, sizeof(contrast_config), HAL_MAX_DELAY) != HAL_OK){
			COM_ERROR = true;
	}

}

//FUNCIONES JUEGO
void clearLayout(){
	for(int i = 0; i < ancho_display; i++){
		memset(map_layout[i], 0, sizeof(&map_layout[i]));
	}
}

void setLayout(){
	for(int i = 0; i < ancho_display; i++){
		for(int j = 0; j < alto_display; j++){
			if(i == 0 || i == (ancho_display - 1) || j == 0 || j == (alto_display - 1))
				map_layout[i][j] = 1;
			else
				map_layout[i][j] = 0;
		}
	}
}

void init_serpiente(){
	s_Sentido = s_SENTIDO_INICIAL;
	for(int i = 0; i < s_Longitud; i++){
		serpiente[i].x = ancho_display / 2 -i;
		serpiente[i].y = alto_display / 2;
	}
}

void init_frutas(){
	//uint32_t x, y;
	for(int i = 0; i < N_FRUTAS; i++){
		/*HAL_RNG_GenerateRandomNumber(&hrng, &x);
		HAL_RNG_GenerateRandomNumber(&hrng, &y);
		x = (x % (ancho_display - 20)) + 10;
		y = (y % (alto_display - 20)) + 10;
		if(map_layout[x][y] == 0){
			frutas[i].x = x;
			frutas[i].y = y;
			i++;
		}*/

		lanzaFruta(&frutas[i]);
	}
}

void lanzaFruta(modulo* fruta){ //Posiciona una fruta en un lugar aleatorio
	uint32_t x, y;
	//for(int i = 0; i < N_FRUTAS;){
	do{
		HAL_RNG_GenerateRandomNumber(&hrng, &x);
		HAL_RNG_GenerateRandomNumber(&hrng, &y);
		x = (x % (ancho_display - 20)) + 10;
		y = (y % (alto_display - 20)) + 10;
	}while(map_layout[x][y] != 0);

	fruta->x = x;
	fruta->y = y;
}
void setSerpiente(){
	for(int i = 0; i < s_Longitud; i++){
		map_layout[serpiente[i].x][serpiente[i].y] = 1;
	}
}

void setFrutas(){
	for(int i = 0; i < N_FRUTAS; i++){
		map_layout[frutas[i].x][frutas[i].y] = 1;
	}
}

void transferMapToBuffer(){
	for(int i = 0; i < ancho_display; i++){
		for(int j = 0; j < alto_display; j++){
			if(map_layout[i][j] == 0)
				dibuja_pixel(i, j, true);
			else
				dibuja_pixel(i, j, false);
		}
	}
}

bool giraSerpiente(uint8_t indicacion){
	if(s_Sentido == s_IZQUIERDA || s_Sentido == s_DERECHA){ //Movimiento Horizontal
		if(indicacion == s_ARRIBA || indicacion == s_ABAJO){ //Giro perpendicular
			s_Sentido = indicacion;
			return true;
		}
	}else //Movimiento Vertical
		if(indicacion == s_IZQUIERDA || indicacion == s_DERECHA){ //Giro perpendicular
			s_Sentido = indicacion;
			return true;
	}

	return false; //No se ejecutó giro
}

void giraDerecha(){
	if(s_Sentido == s_DERECHA)
		s_Sentido = s_ABAJO;
	else if(s_Sentido == s_ABAJO)
		s_Sentido = s_IZQUIERDA;
	else if(s_Sentido == s_IZQUIERDA)
		s_Sentido = s_ARRIBA;
	else
		s_Sentido = s_DERECHA;
}

void giraIzquierda(){
	if(s_Sentido == s_DERECHA)
		s_Sentido = s_ARRIBA;
	else if(s_Sentido == s_ABAJO)
		s_Sentido = s_DERECHA;
	else if(s_Sentido == s_IZQUIERDA)
		s_Sentido = s_ABAJO;
	else
		s_Sentido = s_IZQUIERDA;
}

void avanzaSerpiente(){

	//No comprueba si el sentido es apropiado (dicha comprobación se hace en la función giraSerpiente()
	switch(s_Sentido){
		case s_DERECHA:
			serpiente[0].x += 1;
		break;
		case s_IZQUIERDA:
			serpiente[0].x -= 1;
		break;
		case s_ARRIBA:
			serpiente[0].y -= 1;
		break;
		case s_ABAJO:
			serpiente[0].y += 1;
		break;

	}

	switch(colision(serpiente[0].x, serpiente[0].y)){
		case -1: //Serpiente
		case 2: //Pared
			gameOver();
			break;
		case 0: //Sin Colisión
			break;
		case 1: //Fruta (serpiente crece 1)
			s_Longitud++;
			serpiente[s_Longitud - 1].x = serpiente[s_Longitud - 2].x - (serpiente[s_Longitud - 3].x - serpiente[s_Longitud - 2].x);
			serpiente[s_Longitud - 1].y = serpiente[s_Longitud - 2].y - (serpiente[s_Longitud - 3].y - serpiente[s_Longitud - 2].y);
			if(HAL_UART_Transmit(&huart4, &s_Longitud, sizeof(s_Longitud), HAL_MAX_DELAY) != HAL_OK)
				UART_ERROR = true;
			break;
	}

	/*
	if( (serpiente[0].x >= ancho_display -2) ||
		(serpiente[0].x <= 2) ||
		(serpiente[0].y >= alto_display -2) ||
		(serpiente[0].y <= 2) )
		return; //Si la serpiente alcanza alguno de los márgenes, se detiene
	*/
	for(int i = 0; i < s_Longitud; i++){
		serpiente[s_Longitud - i] = serpiente[s_Longitud - i - 1];
	}

}

int colision(uint8_t x, uint8_t y){

	if(map_layout[x][y] == 1){ //Colisión con ente desconocido
		if(x == ancho_display - 1 || x == 0 || y == alto_display - 1 || y == 0) //Colisión con pared
			return 2;

		for(int i = 0; i < N_FRUTAS; i++){
			if(frutas[i].x == x && frutas[i].y == y){ //Comprueba colisión con fruta
				lanzaFruta(&frutas[i]); //Repone la fruta en otro sitio
				return 1;
			}
		}

		for(int i = 0; i < s_Longitud; i++){ //Colisión con serpiente
			if(serpiente[i].x == x && serpiente[i].y == y)
				return -1;
		}
	}

	return 0;
}

bool debounce(volatile bool* boton){
	static int counter = 0;
	//static bool first_time = true;

	if(*boton == true){
		if((HAL_GetTick() - counter) > 50 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){ //No es rebote
			counter = HAL_GetTick();
			*boton = false;
			return true;
		}else{ //Es Rebote
			counter = HAL_GetTick();
			*boton = false;
			return false;
		}
	}else{
		return false;
	}

}

void gameOver(){
	flashScreen();
	initGame();
}

void initGame(){
	s_Longitud = LONGITUD_INICIAL;
	init_serpiente();
	//clearLayout(); //setLayout() lo hace
	setLayout();
	setSerpiente();
	init_frutas();
	setFrutas();
	transferMapToBuffer();
	display();
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
