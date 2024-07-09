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
#include <ILI9486.hpp>
#include <TouchScreen.hpp>
#include "string.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Param For 3.5" for ADC 12 bit
#define TS_MINX 347
#define TS_MAXX 3600

#define TS_MINY 360
#define TS_MAXY 3850
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t ID;
ILI9486 tft(320, 480);
uint8_t mx = 0, my = 0, mv = 0, ml = 0;
TSPoint p;
uint32_t a;
uint32_t tickCount = 0;
uint32_t elapseTime = 0;
bool touchDetect = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
unsigned long testLines(uint16_t color) {
	int x1, y1, x2, y2, w = tft.width(), h = tft.height();

	tft.fillScreen(BLACK);
	x1 = y1 = 0;
	y2 = h - 1;
	for (x2 = 0; x2 < w; x2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);
	x2 = w - 1;
	for (y2 = 0; y2 < h; y2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);

	tft.fillScreen(BLACK);

	x1 = w - 1;
	y1 = 0;
	y2 = h - 1;
	for (x2 = 0; x2 < w; x2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);
	x2 = 0;
	for (y2 = 0; y2 < h; y2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);

	tft.fillScreen(BLACK);

	x1 = 0;
	y1 = h - 1;
	y2 = 0;
	for (x2 = 0; x2 < w; x2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);
	x2 = w - 1;
	for (y2 = 0; y2 < h; y2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);

	tft.fillScreen(BLACK);

	x1 = w - 1;
	y1 = h - 1;
	y2 = 0;
	for (x2 = 0; x2 < w; x2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);
	x2 = 0;
	for (y2 = 0; y2 < h; y2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);

	return 0;
}

void ConfigInputPin(GPIO_TypeDef *port, uint16_t pin) {
	uint8_t i = 0;
	while (!(pin & 0x01)) {
		i++;
		pin >>= 1;
	}
	port->MODER &= ~(3 << (i * 2));
}
void ConfigOutputPin(GPIO_TypeDef *port, uint16_t pin) {
	uint8_t i = 0;
	while (!(pin & 0x01)) {
		i++;
		pin >>= 1;
	}
	port->MODER |= (1 << (i * 2));
}

void ConfigAnalogXM(uint8_t rank) {
	__HAL_RCC_ADC1_CLK_ENABLE();

	// Config analog
	GPIOA->MODER |= (3 << (4 * 2)); //PA4
	GPIOA->PUPDR &= ~(3 << (4 * 2));

	// Clear and set sample time of channel 4
	ADC1->SMPR2 &= ~ADC_SMPR2(ADC_SMPR2_SMP0, ADC_CHANNEL_4);
	ADC1->SMPR2 |= ADC_SMPR2(ADC_SAMPLETIME_3CYCLES, ADC_CHANNEL_4);
	// Clear input rank and set channel 8 is input rank
	ADC1->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, rank);
	ADC1->SQR3 |= ADC_SQR3_RK(ADC_CHANNEL_4, rank);
}

void ConfigAnalogYP(uint8_t rank) {
	__HAL_RCC_ADC1_CLK_ENABLE();

	// Config analog
	GPIOB->MODER |= (3 << (0 * 2)); //PB0
	GPIOB->PUPDR &= ~(3 << (0 * 2));

	// Clear and set sample time of channel 8
	ADC1->SMPR2 &= ~ADC_SMPR2(ADC_SMPR2_SMP0, ADC_CHANNEL_8);
	ADC1->SMPR2 |= ADC_SMPR2(ADC_SAMPLETIME_3CYCLES, ADC_CHANNEL_8);
	// Clear input rank and set channel 8 is input rank
	ADC1->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, rank);
	ADC1->SQR3 |= ADC_SQR3_RK(ADC_CHANNEL_8, rank);
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	uint8_t val = 0x08;
	val |= my << 7 | mx << 6 | mv << 5 | ml << 4;
	tft.Reset();
	tft.Begin();
	tft.setRotation(0);
	tft.fillRect(0, 0, 320, 480, WHITE);

//	TFT_CS_ACTIVE;
//	tft.WriteCommand(0x36);
//	Write8bit(0x08);
//	TFT_CS_IDLE;

//	tft.setCursor(0, 0);
//	tft.setTextColor(BLUE);
//	tft.setTextSize(4);
//	tft.print((char*) "SpiritBoi\n");
//	tft.print((char*) "Truong Minh Khoa\n");
	char s[20] = {0};
	tft.TouchOn();
//	testLines(CYAN);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		p = tft.GetPoint();
		if (tft.IsTouchDisable()) {
			for (uint8_t i = 0; i < 100; i += 5) {
				tft.drawCircle(p.y, p.x, 5 + i, BLUE);
			}
			for (uint8_t i = 0; i < 100; i += 5) {
				tft.drawCircle(p.y, p.x, 5 + i, WHITE);
			}
			tft.TouchOn();
		}
		if (p.y > TS_MINY && p.z > 1500 && p.z < 3000) {
			p.x = tft.width() - map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
			p.y = map(p.y, TS_MINY, TS_MAXY, tft.height(), 0);
			if (p.y > 0 && p.x > 0) {
				sprintf(s, ">X:%d\n", p.x);
				HAL_UART_Transmit(&huart2, (uint8_t*) s, strlen(s), 1);
				memset(s, 0, strlen(s));
				sprintf(s, ">Y:%d\n", p.y);
				HAL_UART_Transmit(&huart2, (uint8_t*) s, strlen(s), 1);
				memset(s, 0, strlen(s));
				sprintf(s, ">P:%d\n", p.z);
				HAL_UART_Transmit(&huart2, (uint8_t*) s, strlen(s), 1);
				memset(s, 0, strlen(s));
				tft.SaveTouchPoint(p);
				tft.TouchOff();
			}
		}
	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 64;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1 | GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC1 PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA1 PA4 PA8
	 PA9 PA10 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB10 PB3 PB4 PB5
	 PB6 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
