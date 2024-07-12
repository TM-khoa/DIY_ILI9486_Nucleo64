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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ILI9486.hpp>
#include <TouchScreen.hpp>
#include "string.h"
#include <stdio.h>
#include "fatfs_sd.h"
#include "file_handling.h"
#include "ringbuffer.h"
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
uint32_t tftID;
ILI9486 tft(320, 480);
uint8_t mx = 0, my = 0, mv = 0, ml = 0;
TSPoint p;
uint32_t tickCount = 0;
uint32_t elapseTime = 0;
bool touchDetect = false;
char buffer[1024] = {0};
char path[300] = {0};

/*********************************************/
// This procedure reads a bitmap and draws it to the screen
// its sped up by reading many pixels worth of data at a time
// instead of just one pixel at a time. increading the buffer takes
// more RAM but makes the drawing a little faster. 20 pixels' worth
// is probably a good place
#define BUFFPIXEL       80                      // must be a divisor of 320 
#define BUFFPIXEL_X3    240                     // BUFFPIXELx3

uint32_t __Gnbmp_image_offset = 0;        			// offset
const int __Gnbmp_height = 480;                 // 480 bmp height
const int __Gnbmp_width = 320;                 // 320 bmp width

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
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

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// LITTLE ENDIAN!
uint16_t read16(char *fileName, FSIZE_t readOffset) {
	FRESULT fresult;
	uint16_t d = 0;
	UINT byteRead = 0;
	fresult = fhl_read_chunk(fileName, (void*) &d, 2, readOffset, &byteRead);
	if (fresult != FR_OK) d = 0;
	return d;
}

// LITTLE ENDIAN!
uint32_t read32(char *fileName, FSIZE_t readOffset) {
	FRESULT fresult;
	uint32_t d = 0;
	UINT byteRead = 0;
	fresult = fhl_read_chunk(fileName, (void*) &d, 4, readOffset, &byteRead);
	if (fresult != FR_OK) d = 0;
	return d;
}

void TFT_Init() {
	tft.Reset();
	tft.Begin();
	tft.setRotation(1);
	tft.fillRect(0, 0, 320, 480, BLACK);
	for (uint16_t i = 0; i < 320; i++) {
		tft.drawFastVLine(0, i, 480, WHITE);
	}
	for (uint16_t i = 0; i < 480; i++) {
		tft.drawFastVLine(i, 0, 320, WHITE);
	}
//	tft.setCursor(0, 0);
//	tft.setTextColor(BLUE);
//	tft.setTextSize(4);
//	tft.print((char*) "SpiritBoi\n");
//	tft.print((char*) "Truong Minh Khoa\n");

//	tft.TouchOn();
//	testLines(CYAN);
}
void TFT_Stuff() {
	char s[20] = {0};
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

void SendMessage(char *s) {
	HAL_UART_Transmit(&huart2, (uint8_t*) s, strlen(s), HAL_MAX_DELAY);
}

void TestTransferData() {
	uint32_t readOffset = 0;
	UINT byteRead = 0;
	tft.SetAddrWindow(100, 100 + 320, 0, 303);
	while (fhl_read_chunk((char*) "streamdata.txt", buffer, 1000, readOffset, &byteRead) == FR_OK) {
		if (byteRead == 0) break;
		readOffset += byteRead;
		memset(buffer, 0, strlen(buffer));
	}
}

bool BitmapReadHeader(char *fileName) {
	// read header
	uint32_t tmp;
	uint8_t bmpDepth;

	char s[30] = {0};
	// magic bytes missing
	if (read16(fileName, 0) != 0x4D42) return false;
	// offset after magic bytes
	tmp = read32(fileName, 2);
	sprintf(s, "Image size:%lu\n", tmp);
	SendMessage(s);
	__Gnbmp_image_offset = read32(fileName, 10);
	sprintf(s, "offset:%lu\n", __Gnbmp_image_offset);
	SendMessage(s);
	// read DIB header
	tmp = read32(fileName, 14);
	sprintf(s, "Header size:%lu\n", tmp);
	SendMessage(s);
	uint32_t bmp_width = read32(fileName, 18);
	uint32_t bmp_height = read32(fileName, 22);
	sprintf(s, "Image dimension:%lux%lu\n", bmp_width, bmp_height);
	SendMessage(s);
	if (read16(fileName, 26) != 1) return false;
	bmpDepth = read16(fileName, 28);
	sprintf(s, "Bit depth:%u\n", bmpDepth);
	SendMessage(s);
	if (read32(fileName, 30) != 0) return false;
	return true;
}

void BitmapDraw() {

}
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
	MX_ADC1_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */
//	TFT_Init();
	fhl_init(buffer, sizeof(buffer), path, sizeof(path));
	fhl_register_notify_status(&SendMessage);
	fhl_register_notify_error(&SendMessage);
	fhl_mount_sd();
	fhl_scan_files((char*) "");
	while (BitmapReadHeader((char*) "ori.bmp") != true);
//	TestTransferData();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
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
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_8B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
