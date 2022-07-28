/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#include "math.h"
#include "user_codex.h"
#include "user_eeprom.h"
#include "user_LCD.h"
#include "user_touch.h"
#include "user_joypad.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY2_Pin GPIO_PIN_2
#define KEY2_GPIO_Port GPIOE
#define KEY2_EXTI_IRQn EXTI2_IRQn
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOE
#define KEY1_EXTI_IRQn EXTI3_IRQn
#define KEY0_Pin GPIO_PIN_4
#define KEY0_GPIO_Port GPIOE
#define KEY0_EXTI_IRQn EXTI4_IRQn
#define VS_RST_Pin GPIO_PIN_6
#define VS_RST_GPIO_Port GPIOE
#define VS_DERQ_Pin GPIO_PIN_13
#define VS_DERQ_GPIO_Port GPIOC
#define VS_XDCS_Pin GPIO_PIN_6
#define VS_XDCS_GPIO_Port GPIOF
#define VS_XCS_Pin GPIO_PIN_7
#define VS_XCS_GPIO_Port GPIOF
#define LIGHT_SENSOR_Pin GPIO_PIN_8
#define LIGHT_SENSOR_GPIO_Port GPIOF
#define T_MOSI_Pin GPIO_PIN_9
#define T_MOSI_GPIO_Port GPIOF
#define PEN_Pin GPIO_PIN_10
#define PEN_GPIO_Port GPIOF
#define PEN_EXTI_IRQn EXTI15_10_IRQn
#define LCD_BL_Pin GPIO_PIN_0
#define LCD_BL_GPIO_Port GPIOB
#define T_SCL_Pin GPIO_PIN_1
#define T_SCL_GPIO_Port GPIOB
#define T_MISO_Pin GPIO_PIN_2
#define T_MISO_GPIO_Port GPIOB
#define T_CS_Pin GPIO_PIN_11
#define T_CS_GPIO_Port GPIOF
#define JOYPAD_DAT_Pin GPIO_PIN_10
#define JOYPAD_DAT_GPIO_Port GPIOB
#define JOYPAD_LAT_Pin GPIO_PIN_11
#define JOYPAD_LAT_GPIO_Port GPIOB
#define JOYPAD_CLK_Pin GPIO_PIN_3
#define JOYPAD_CLK_GPIO_Port GPIOD
#define LED0_Pin GPIO_PIN_5
#define LED0_GPIO_Port GPIOB
#define BEEP_Pin GPIO_PIN_8
#define BEEP_GPIO_Port GPIOB
#define REMOTE_IN_Pin GPIO_PIN_9
#define REMOTE_IN_GPIO_Port GPIOB
#define REMOTE_IN_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */
#define MENUCNT	6
#define MP3_BITRATE 128000
#define MUSIC_STOP 0
#define MUSIC_CYCLE 1
#define MUSIC_SHUFFLE 2
#define MUSIC_RECYCLE 3
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
