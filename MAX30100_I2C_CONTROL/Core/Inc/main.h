/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define DEPTH			255 	/* How many samples the program takes into
								account in its SpO2 Average calculations; The
								greater the number, the longer the delay between
								measure and output reaction. Note that, because
								each individual sample is actually the average
								of 4 samples extracted from the Oximeter, the
								true number of measured samples is actually 4
								times as high. Default 255. */
#define CH1_DC_MAX		65535 	/* The desired maximum value of the duty cycle
								(65535 corresponds to 100%); Lowering it likewise
								lowers output voltage by an equivalent value.
								Default 65535. */

/* The following values are particularly important for the purpose of controlling
 * output. Recommendation is to always keep UPTICK at a lower value than DOWNTICK.
 * This is to prevent noise buildup from too easily starting a false output.
 */
#define THRESHOLD 		100		/* The number the control variable SpO2_c must
								reach to enable the DAC output. Default 90.*/
#define UPTICK			15		/* How much a tick 'upwards' weighs on the
 	 	 	 	 	 	 	 	control variable. Default 15. */
#define DOWNTICK		30		/* How much a tick 'downwards' weighs on the
								control variable. Default 30. */


#define TIMER7PERIOD	1575	/* How quickly the program advances through the
								desired waveform array. Default 3000, for a
								frequency of 84MHz / 8000 / 3000 = 3.5 Hz*/
#define TEMP_ONOFF		0		/* Whether or not the temperature reading from
								the in-circuit thermometer is toggled on.
								Default 0. */
#define POWER_REG_ONOFF 0		/* Whether or not the power supply readings are
								toggled on. Default 0. */
#define LED_POWER		0x88	/* The values for LED power configuration.
								The leftmost byte configures the Red LED, while
								the rightmost configures infrared LED.
								Default 0x88. */
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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
