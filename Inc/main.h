/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/**
 * Helper structure read lines from UART.
 */
#define INPUT_BUF_SIZE 32
typedef struct {
	volatile char buf[INPUT_BUF_SIZE];
	volatile size_t pos;
	UART_HandleTypeDef *huart;
} input_buf_t;
// declare global variable
extern input_buf_t project_input_buf;
/**
 * Initialize input_line_buf_t object.
 */
int input_buf_init(input_buf_t *input_buf, UART_HandleTypeDef *huart);
/**
 * Read current content of the buffer.
 */
int input_buf_read_content(input_buf_t *input_buf, char*output);
/**
 * Process UART interruption.
 */
int input_buf_process_rxne_it(input_buf_t *input_buf);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define LCD_RST_Pin GPIO_PIN_1
#define LCD_RST_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_2
#define LCD_DC_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_0
#define LCD_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
