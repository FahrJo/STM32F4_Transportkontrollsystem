/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include <stdint.h>
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
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define PDM_OUT_Pin GPIO_PIN_3
#define PDM_OUT_GPIO_Port GPIOC
#define User_Button_Pin GPIO_PIN_0
#define User_Button_GPIO_Port GPIOA
#define User_Button_EXTI_IRQn EXTI0_IRQn
#define ADC_Temp_Pin GPIO_PIN_1
#define ADC_Temp_GPIO_Port GPIOA
#define INT_Photodiode_Pin GPIO_PIN_2
#define INT_Photodiode_GPIO_Port GPIOA
#define INT_Photodiode_EXTI_IRQn EXTI2_IRQn
#define INT_Acceleration_Pin GPIO_PIN_3
#define INT_Acceleration_GPIO_Port GPIOA
#define INT_Acceleration_EXTI_IRQn EXTI3_IRQn
#define I2S3_WS_Pin GPIO_PIN_4
#define I2S3_WS_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define INT_Acceleration2_Pin GPIO_PIN_4
#define INT_Acceleration2_GPIO_Port GPIOC
#define INT_Acceleration2_EXTI_IRQn EXTI4_IRQn
#define GPS_Reset_out_Pin GPIO_PIN_0
#define GPS_Reset_out_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define GPS_WakeUp_Pin GPIO_PIN_10
#define GPS_WakeUp_GPIO_Port GPIOE
#define GPS_ONOFF_Pin GPIO_PIN_12
#define GPS_ONOFF_GPIO_Port GPIOE
#define CLK_IN_Pin GPIO_PIN_10
#define CLK_IN_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_12
#define LED4_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOD
#define LED5_Pin GPIO_PIN_14
#define LED5_GPIO_Port GPIOD
#define LED6_Pin GPIO_PIN_15
#define LED6_GPIO_Port GPIOD
#define I2C_INIT_SCL_Pin GPIO_PIN_6
#define I2C_INIT_SCL_GPIO_Port GPIOC
#define I2C_INIT_SDA_Pin GPIO_PIN_7
#define I2C_INIT_SDA_GPIO_Port GPIOC
#define SDA_Accelerometer_Pin GPIO_PIN_9
#define SDA_Accelerometer_GPIO_Port GPIOC
#define SCL_Accelerometer_Pin GPIO_PIN_8
#define SCL_Accelerometer_GPIO_Port GPIOA
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define SD_detect_Pin GPIO_PIN_7
#define SD_detect_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

typedef enum {
	workmode_sleep = 0,
	workmode_active,
	workmode_log
} workmode_type;

typedef enum {
	no_event = 0,
	open_event,
	moving_event,
	hit_event,
	temp_event,
	eject_card,
	deactivate_gnss
} event_type;

typedef struct {
  uint16_t MAX_TEMP;														/* Temperatur in K, UEBER der ein Interrupt ausgeloest wird */
	uint16_t MIN_TEMP;														/* Temperatur in K, UNTER der ein Interrupt ausgeloest wird */
	uint16_t MAX_TEMP_RAW;												/* Oberer Schwellwert des Analog WDG*/
	uint16_t MIN_TEMP_RAW;												/* Unterer Schwellwert des Analog WDG*/
	float MAX_ACC;
	uint8_t ACCELERATION_ENABLE;
	uint8_t GNSS_ENABLE; 
	uint8_t LIGHT_ENABLE;
	uint8_t TEMP_ENABLE;
	uint16_t ACC_MAX_ANZAHL_WERTE;
} configuration;
	
	 
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
