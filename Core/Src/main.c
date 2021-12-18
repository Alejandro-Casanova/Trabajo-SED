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

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
// VARIABLES DISPLAY
//Direcciónes I2C
uint8_t slave_address_write = 0x78;
uint8_t slave_address_read = 0x78 | 0x01;

bool COM_ERROR = false; //Indica error de comunicación I2C con el display
#define alto_display 64
#define ancho_display 128
#define n_paginas 8
uint8_t buffer[n_paginas * ancho_display]; //Información que se cargará en la SRAM del display

uint8_t i2cTxBuf[2];
uint8_t i2cRxBuf;

//VARIABLES JUEGO
uint8_t map_layout[ancho_display][alto_display];
bool refresh = false;

//Cuerpo de la serpiente
struct modulo{
	uint8_t x;
	uint8_t y;
};
struct modulo serpiente[200];
uint8_t s_Longitud = 10;

//Dirección de la serpiente
#define s_DERECHA 'r'
#define s_IZQUIERDA 'i'
#define s_ARRIBA 'u'
#define s_ABAJO 'd'
uint8_t s_Sentido = s_DERECHA;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM7_Init(void);

/* USER CODE BEGIN PFP */
//FUNCIONES OLED
bool oled_command(uint8_t command); //Envía un comando simple al display (NO UTILIZADO)
void oled_init(); //Inicialización Obligatoria
void alimentacion(bool enable); //Enciende o apaga el display
void clear(bool enable); //Carga todos los valores del buffer con un mismo valor
void display(); //Carga el buffer en la SRAM del display
void dibuja_pixel(uint8_t x, uint8_t y, bool borra); //Dibuja/Borra un sólo pixel del buffer
void flash_screen(); //Hace parpadear la pantalla (Todo ON/Valores SRAM)

//FUNCIONES JUEGO
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim); //Callback encargada de avanzar el juego y refrescar la pantalla cada 50ms
void clearLayout(); //Limpia el layout
void setLayout(); //Dibuja las paredes y BORRA LO DEM�?S
void init_serpiente(); //Inicializa la serpiente en posicion inicial
void setSerpiente(); //Dibuja la serpiente en el layout
void transferMapToBuffer(); //Carga el mapa en el buffer que luego se envía al display
bool giraSerpiente(uint8_t indicacion); //Cambia la dirección de la serpiente (Hay macros con cada dirección) Devuelve false si no se pudo ejecutar el giro
void giraDerecha(); //Gira la serpiente 90º a la derecha
void avanzaSerpiente(); //Hace avanzar la serpiente una casilla (en el layout)
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
  /* USER CODE BEGIN 2 */


  oled_init(); //INICIALIZACIÓN OBLIGATORIA
  init_serpiente();

  //clearLayout(); //setLayout() lo hace
  setLayout();
  setSerpiente();
  transferMapToBuffer();
  display();

  HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int a = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  for(int i = 0; i < 10; i++){
//		  avanzaSerpiente();
//		  setLayout();
//		  setSerpiente();
//		  transferMapToBuffer();
//		  display();
//		  HAL_Delay(50);
//	  }
//	  giraSerpiente(s_IZQUIERDA); //No tiene efecto
//	  giraSerpiente(s_ABAJO);
//	  giraSerpiente(s_ARRIBA); //No tiene efecto
//	  for(int i = 0; i < 10; i++){
//		  avanzaSerpiente();
//		  setLayout();
//		  setSerpiente();
//		  transferMapToBuffer();
//		  display();
//		  HAL_Delay(50);
//	  }
//	  giraSerpiente(s_IZQUIERDA);
//	  for(int i = 0; i < 10; i++){
//		  avanzaSerpiente();
//		  setLayout();
//		  setSerpiente();
//		  transferMapToBuffer();
//		  display();
//		  HAL_Delay(50);
//	  }
//	  giraSerpiente(s_ARRIBA);
//	  for(int i = 0; i < 10; i++){
//		  avanzaSerpiente();
//		  setLayout();
//		  setSerpiente();
//		  transferMapToBuffer();
//		  display();
//		  HAL_Delay(50);
//	  }
//	  giraSerpiente(s_DERECHA);
	  while(a < 100){
		  if(refresh){
			  avanzaSerpiente();
			  setLayout();
			  setSerpiente();
			  transferMapToBuffer();
			  display();

			  a++;
			  refresh = false;
			  if((a % 10) == 0)
				  giraDerecha();
		  }
	  }
	  HAL_Delay(5000);
	  alimentacion(false);
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
void flash_screen(){
	i2cTxBuf[0] = 0x00; //Command
	i2cTxBuf[1] = 0xA4;
	if(HAL_I2C_Master_Transmit(&hi2c1, slave_address_write, i2cTxBuf, 2, HAL_MAX_DELAY) != HAL_OK){
	  COM_ERROR = true;
	}
	HAL_Delay(500);
	i2cTxBuf[1] = 0xA5;
	HAL_I2C_Master_Transmit(&hi2c1, slave_address_write, i2cTxBuf, 2, HAL_MAX_DELAY);
	HAL_Delay(500);
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
	s_Sentido = s_DERECHA;
	for(int i = 0; i < s_Longitud; i++){
		serpiente[i].x = ancho_display / 2 -i;
		serpiente[i].y = alto_display / 2;
	}
}

void setSerpiente(){
	for(int i = 0; i < s_Longitud; i++){
		map_layout[serpiente[i].x][serpiente[i].y] = 1;
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

void avanzaSerpiente(){
	if( (serpiente[0].x >= ancho_display -2) ||
		(serpiente[0].x <= 2) ||
		(serpiente[0].y >= alto_display -2) ||
		(serpiente[0].y <= 2) )
		return; //Si la serpiente alcanza alguno de los márgenes, se detiene

	for(int i = 0; i < s_Longitud; i++){
		serpiente[s_Longitud - i] = serpiente[s_Longitud - i - 1];
	}
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
