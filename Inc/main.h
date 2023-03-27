/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_4
#define LED3_GPIO_Port GPIOE
#define LED4_Pin GPIO_PIN_5
#define LED4_GPIO_Port GPIOE
#define M_FrameReq_Pin GPIO_PIN_13
#define M_FrameReq_GPIO_Port GPIOC
#define M_RST_Pin GPIO_PIN_14
#define M_RST_GPIO_Port GPIOC
#define SPI2_CS_Pin GPIO_PIN_2
#define SPI2_CS_GPIO_Port GPIOC
#define PWRRST_Pin GPIO_PIN_3
#define PWRRST_GPIO_Port GPIOA
#define KEY_ENTER_Pin GPIO_PIN_10
#define KEY_ENTER_GPIO_Port GPIOE
#define DS1302_CLK_Pin GPIO_PIN_13
#define DS1302_CLK_GPIO_Port GPIOE
#define DS1302_DATA_Pin GPIO_PIN_14
#define DS1302_DATA_GPIO_Port GPIOE
#define DS1302_RST_Pin GPIO_PIN_15
#define DS1302_RST_GPIO_Port GPIOE
#define OLED_RST_Pin GPIO_PIN_11
#define OLED_RST_GPIO_Port GPIOD
#define OLED_CD_Pin GPIO_PIN_12
#define OLED_CD_GPIO_Port GPIOD
#define OLED_CS_Pin GPIO_PIN_13
#define OLED_CS_GPIO_Port GPIOD
#define M_FlagAck_Pin GPIO_PIN_6
#define M_FlagAck_GPIO_Port GPIOC
#define M_DIN_Pin GPIO_PIN_7
#define M_DIN_GPIO_Port GPIOC
#define M_DOUT_Pin GPIO_PIN_8
#define M_DOUT_GPIO_Port GPIOC
#define M_CLK_Pin GPIO_PIN_9
#define M_CLK_GPIO_Port GPIOC
#define SPI3_CS_Pin GPIO_PIN_2
#define SPI3_CS_GPIO_Port GPIOD
#define SPI3_REQ_Pin GPIO_PIN_3
#define SPI3_REQ_GPIO_Port GPIOD
#define SPI3_ACK_Pin GPIO_PIN_4
#define SPI3_ACK_GPIO_Port GPIOD
#define OLED_DATA_Pin GPIO_PIN_0
#define OLED_DATA_GPIO_Port GPIOE
#define OLED_CLK_Pin GPIO_PIN_1
#define OLED_CLK_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
