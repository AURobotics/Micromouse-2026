/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define ADC1_F_L_Pin GPIO_PIN_0
#define ADC1_F_L_GPIO_Port GPIOC
#define ADC1_L_D_Pin GPIO_PIN_1
#define ADC1_L_D_GPIO_Port GPIOC
#define ADC1_L_Pin GPIO_PIN_2
#define ADC1_L_GPIO_Port GPIOC
#define L_ENC_A_Pin GPIO_PIN_0
#define L_ENC_A_GPIO_Port GPIOA
#define L_ENC_B_Pin GPIO_PIN_1
#define L_ENC_B_GPIO_Port GPIOA
#define ADC2_R_Pin GPIO_PIN_4
#define ADC2_R_GPIO_Port GPIOA
#define R_ENC_A_Pin GPIO_PIN_6
#define R_ENC_A_GPIO_Port GPIOA
#define R_ENC_B_Pin GPIO_PIN_7
#define R_ENC_B_GPIO_Port GPIOA
#define ADC2_R_D_Pin GPIO_PIN_4
#define ADC2_R_D_GPIO_Port GPIOC
#define ADC2_F_R_Pin GPIO_PIN_5
#define ADC2_F_R_GPIO_Port GPIOC
#define CYBLE_SLEEP_Pin GPIO_PIN_1
#define CYBLE_SLEEP_GPIO_Port GPIOB
#define BNO_SCL_Pin GPIO_PIN_10
#define BNO_SCL_GPIO_Port GPIOB
#define BTN_3_Pin GPIO_PIN_12
#define BTN_3_GPIO_Port GPIOB
#define BTN_2_Pin GPIO_PIN_13
#define BTN_2_GPIO_Port GPIOB
#define BTN_1_Pin GPIO_PIN_14
#define BTN_1_GPIO_Port GPIOB
#define SIDE_TRIG_Pin GPIO_PIN_6
#define SIDE_TRIG_GPIO_Port GPIOC
#define DIAGONAL_TRIG_Pin GPIO_PIN_7
#define DIAGONAL_TRIG_GPIO_Port GPIOC
#define FRONT_TRIG_Pin GPIO_PIN_8
#define FRONT_TRIG_GPIO_Port GPIOC
#define BLE_TX_Pin GPIO_PIN_10
#define BLE_TX_GPIO_Port GPIOC
#define BLE_RX_Pin GPIO_PIN_11
#define BLE_RX_GPIO_Port GPIOC
#define BNO_SDA_Pin GPIO_PIN_12
#define BNO_SDA_GPIO_Port GPIOC
#define BNO_INT_Pin GPIO_PIN_2
#define BNO_INT_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_4
#define LED_2_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_5
#define LED_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
