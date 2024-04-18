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
#include <string.h>
#include <ctype.h>
#include "fonts.h"
#include "ILI9341_STM32_Driver.h"
#include "Adafruit_AMG88xx.h"
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

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float buf[64];

float bufIntp[64][64];
uint16_t PaletteWhiteHot[] = {WHITE, 0xE73C, 0xCE59, 0xAD55, 0x8C71, 0x6B6D, 0x528A, 0x3186, 0x10A2, BLACK};
uint16_t PaletteBlackHot[] = {BLACK, 0x10A2, 0x3186, 0x528A, 0x6B6D, 0x8C71, 0xAD55, 0xCE59, 0xE73C, WHITE};
uint16_t PaletteRedHot[] = {RED,  0xFA20, 0x8800, ORANGE, 0xFC60, 0xDD24, YELLOW, 0x0011,  0x0010, BLUE};

int DEAD_ZONE = 2;
float TEMP_LOW = 0;
float TEMP_HIGH = 50;
float TEMP_MAX = -100;
float TEMP_MIN = 200;
float TEMP_TARG = 0;
int MODE = 3;
int RAW = 0;

void interpolate(){
    double arr[4][4];
	  int i,j;
	  int f = 4;int h = 8;int w = 8;

	  TEMP_MAX = -100;
	  TEMP_MIN = 200;

		for(i=0;i<4;i++)
			for(j=0;j<4;j++)
				arr[i][j]=0;

	     for(int i=0; i<f*h; i++) {
	        for(int j=0; j<f*w; j++) {
	            for( int l = 0; l < 4; l++ ) {
	            	for( int k = 0; k < 4; k++ ) {
	            		if((int)i/f + l < h && (int)j/f + k < w) {
	            			arr[l][k]=(double)buf[(int)i/f + (int) (j/f)];
						}
	        		}
	        	}
	          bufIntp[i][j] = (float)bicubicpol((double)(i%f)/f,(double)(j%f)/f ,arr);

	          if(bufIntp[i][j] > TEMP_MAX)
	        	TEMP_MAX = bufIntp[j][i];
	          if(bufIntp[i][j] < TEMP_MIN)
	        	TEMP_MIN = bufIntp[j][i];
	        }
		}


}

void bilinear_interpolation() {
    float x_ratio, y_ratio;
    float *data = buf;
    float *output = bufIntp;
	uint32_t input_width = 8;
	uint32_t input_height = 8;
	uint32_t output_width = 64;
	uint32_t output_height = 64;

    if (output_width > 1) {
        x_ratio = ((float)input_width - 1.0) / ((float)output_width - 1.0);
    } else {
        x_ratio = 0;
    }

    if (output_height > 1) {
        y_ratio = ((float)input_height - 1.0) / ((float)output_height - 1.0);
    } else {
        y_ratio = 0;
    }

    for (int i = 0; i < output_height; i++) {
        for (int j = 0; j < output_width; j++) {
            float x_l = floor(x_ratio * (float)j);
            float y_l = floor(y_ratio * (float)i);
            float x_h = ceil(x_ratio * (float)j);
            float y_h = ceil(y_ratio * (float)i);

            float x_weight = (x_ratio * (float)j) - x_l;
            float y_weight = (y_ratio * (float)i) - y_l;

            float a = data[(int)y_l * input_width + (int)x_l];
            float b = data[(int)y_l * input_width + (int)x_h];
            float c = data[(int)y_h * input_width + (int)x_l];
            float d = data[(int)y_h * input_width + (int)x_h];

            float pixel = a * (1.0 - x_weight) * (1.0 - y_weight) +
                          b * x_weight * (1.0 - y_weight) +
                          c * y_weight * (1.0 - x_weight) +
                          d * x_weight * y_weight;

            bufIntp[i][j] = pixel;
        }
    }
}

uint16_t redHotCol(float temp){
	int k;
	uint16_t ind = (((0 & 0xf8)<<8) + ((0 & 0xfc)<<3) + (0>>3)); ;
	if(temp > 40.0){
		k = (50 - temp) * 255 / 10;
		ind = (((255 & 0xf8)<<8) + ((k & 0xfc)<<3) + (0>>3));
		return ind;
	}
	if(temp > 30.0){
		k = (temp - 30) * 255 / 10;
		ind = (((k & 0xf8)<<8) + ((255 & 0xfc)<<3) + (0>>3));
		return ind;
	}
	if(temp > 20.0){
		k = (30 - temp) * 255 / 10;
		ind = (((0 & 0xf8)<<8) + ((255 & 0xfc)<<3) + (k>>3));
		return ind;
	}
	if(temp > 10.0){
		k = (temp - 10.0) * 255 / 10;
		ind = (((0 & 0xf8)<<8) + ((k & 0xfc)<<3) + (255>>3));
		return ind;
	}
	if(temp > 0.0){
		k = (10 - temp) * 255 / 10;
		ind = (((k & 0xf8)<<8) + ((0 & 0xfc)<<3) + (255>>3));
		return ind;
	}
	return ind;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
 // HAL_UART_Transmit(&huart1, "Init\n", 5, 1000);

  	uint16_t *color;
  	uint16_t background;
	ILI9341_Init();
	ILI9341_SetRotation(SCREEN_HORIZONTAL_1);

	switch(MODE % 3){
	case(1):
		color = PaletteWhiteHot;
		background = BLACK;
		ILI9341_FillScreen(BLACK);
		break;
	case(2):
		color = PaletteBlackHot;
		background = WHITE;
		ILI9341_FillScreen(WHITE);
		break;
	case(3):
		color = PaletteRedHot;
		background = BLUE;
		ILI9341_FillScreen(BLUE);
		break;
	default:
		color = PaletteRedHot;
		background = BLUE;
		ILI9341_FillScreen(BLUE);
		break;
	}
	HAL_Delay(1000);
	amg88xxInit();
	HAL_Delay(1000);

//  HAL_UART_Transmit(&huart1, "Start\n", 6, 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  ILI9341_SetRotation(SCREEN_HORIZONTAL_1);
	  readPixels(buf , 64);
	  TEMP_TARG = (buf[28] + buf[29] + buf[36] + buf[37]) / 4;
	  TEMP_MAX = -100;
	  TEMP_MIN = 200;
	  for(int i = 0; i < 8; i++){
		  for(int j = 0; j < 8; j++){
			  if(buf[i * 8 + j] > TEMP_MAX)
			  	 TEMP_MAX = buf[i * 8 + j];
			  if(buf[i * 8 + j] < TEMP_MAX)
			  	 TEMP_MIN = buf[i * 8 + j];
		  }
	  }

	      if(RAW){
	  		for(int i = 0; i < 8; i++){
	  			for(int j = 0; j < 8; j++){
	  				if(buf[i * 8 + j] > TEMP_MAX)
	  					TEMP_MAX = buf[i * 8 + j];
	  				if(buf[i * 8 + j] < TEMP_MAX)
	  					TEMP_MIN = buf[i * 8 + j];
	  				if(buf[i * 8 + j] < (TEMP_LOW + 0.1)){
	  					ILI9341_DrawRectangle(i * 32, j * 32, 32, 32, color[9]);
	  					continue;
	  				}
	  				if(buf[i * 8 + j] > (TEMP_HIGH + 0.1)){
	  						ILI9341_DrawRectangle(i * 32, j * 32, 32, 32, color[0]);
	  						continue;
	  				}
	  				int index = (TEMP_HIGH - buf[i * 8 + j]) / DEAD_ZONE;
	  				ILI9341_DrawRectangle(i * 32  , j * 32 , 32, 32, color[index % 10]);

	  			}
	  		}
	      }

	      if(!RAW){
			 // interpolate();
	    	  bilinear_interpolation();

	    	  uint16_t Rgb565;


				for(int i = 0; i < 64; i++){
					for(int j = 0; j < 64; j++){

						if(bufIntp[i][j] < (TEMP_LOW - 0.1)){
							ILI9341_DrawRectangle(i * 4, j * 4, 4, 4, color[9]);
							continue;
						}
						if(bufIntp[i][j] > (TEMP_HIGH + 0.1)){
								ILI9341_DrawRectangle(i * 4, j * 4, 4, 4, color[0]);
								continue;
						}
						int k;
						switch(MODE){
						case(1):
							k = (bufIntp[i][j] - TEMP_LOW) * 255 / (TEMP_MAX - TEMP_LOW);
							Rgb565 = (((k & 0xf8)<<8) + ((k & 0xfc)<<3) + (k>>3));
							break;
						case(2):
							k = (TEMP_MAX - bufIntp[i][j]) * 255 / (TEMP_MAX - TEMP_LOW);
							Rgb565 = (((k & 0xf8)<<8) + ((k & 0xfc)<<3) + (k>>3));
							break;
						case (3):
							Rgb565 = redHotCol(bufIntp[i][j]);
							break;
						}
						ILI9341_DrawRectangle(i * 4  , j * 4 , 4, 4, Rgb565);
					}
			}
	      }

	      	ILI9341_SetRotation(SCREEN_VERTICAL_2);
	      	ILI9341_DrawFilledRectangleCoord(115, 191, 125, 193, background);
	      	ILI9341_DrawFilledRectangleCoord(119, 187, 121, 197, background);
	  		char Text[16];
	  		sprintf(Text, "MAX: %.1f", TEMP_MAX);
	  		ILI9341_DrawText(Text, FONT3, 5, 5, color[0], background);
	  		sprintf(Text, "MIN: %.1f", TEMP_MIN);
	  		ILI9341_DrawText(Text, FONT3, 5, 20, color[0], background);
	  		sprintf(Text, "TEMP: %.1f", TEMP_TARG);
	  		ILI9341_DrawText(Text, FONT3, 100, 5, color[0], background);
	  		sprintf(Text, "MODE: %d", MODE);
	  		ILI9341_DrawText(Text, FONT3, 100, 20, color[0], background);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RST_Pin|LCD_DC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

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
