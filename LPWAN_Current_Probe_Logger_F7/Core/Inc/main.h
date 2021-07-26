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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// LCD + UI
#include <string.h>
#include "u8g2.h"
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
#define ASW1_Pin GPIO_PIN_2
#define ASW1_GPIO_Port GPIOE
#define ASW4_Pin GPIO_PIN_3
#define ASW4_GPIO_Port GPIOE
#define ASW2_Pin GPIO_PIN_4
#define ASW2_GPIO_Port GPIOE
#define ASW3_Pin GPIO_PIN_5
#define ASW3_GPIO_Port GPIOE
#define PE6_Pin GPIO_PIN_6
#define PE6_GPIO_Port GPIOE
#define PC13_Pin GPIO_PIN_13
#define PC13_GPIO_Port GPIOC
#define PC2_Pin GPIO_PIN_2
#define PC2_GPIO_Port GPIOC
#define PC3_Pin GPIO_PIN_3
#define PC3_GPIO_Port GPIOC
#define PA0_Pin GPIO_PIN_0
#define PA0_GPIO_Port GPIOA
#define PA3_Pin GPIO_PIN_3
#define PA3_GPIO_Port GPIOA
#define PS_EN_Pin GPIO_PIN_4
#define PS_EN_GPIO_Port GPIOA
#define USB_OTG_FS_OverCurrent_Pin GPIO_PIN_0
#define USB_OTG_FS_OverCurrent_GPIO_Port GPIOB
#define USB_OTG_FS_OverCurrent_EXTI_IRQn EXTI0_IRQn
#define BUTTON_MEASURING_Pin GPIO_PIN_1
#define BUTTON_MEASURING_GPIO_Port GPIOB
#define BUTTON_MEASURING_EXTI_IRQn EXTI1_IRQn
#define Extra_GPIO_Pin GPIO_PIN_2
#define Extra_GPIO_GPIO_Port GPIOB
#define Extra_GPIO_EXTI_IRQn EXTI2_IRQn
#define LED_BLUE_Pin GPIO_PIN_9
#define LED_BLUE_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_10
#define BUZZER_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_15
#define LED_GREEN_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_10
#define LED_RED_GPIO_Port GPIOB
#define BUTTON_DOWN_Pin GPIO_PIN_14
#define BUTTON_DOWN_GPIO_Port GPIOB
#define BUTTON_DOWN_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_PREV_Pin GPIO_PIN_15
#define BUTTON_PREV_GPIO_Port GPIOB
#define BUTTON_PREV_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_NEXT_Pin GPIO_PIN_8
#define BUTTON_NEXT_GPIO_Port GPIOD
#define BUTTON_NEXT_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_UP_Pin GPIO_PIN_9
#define BUTTON_UP_GPIO_Port GPIOD
#define BUTTON_UP_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_ESC_Pin GPIO_PIN_10
#define BUTTON_ESC_GPIO_Port GPIOD
#define BUTTON_ESC_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_ENTER_Pin GPIO_PIN_11
#define BUTTON_ENTER_GPIO_Port GPIOD
#define BUTTON_ENTER_EXTI_IRQn EXTI15_10_IRQn
#define USB_OTG_FS_VBUS_Pin GPIO_PIN_9
#define USB_OTG_FS_VBUS_GPIO_Port GPIOA
#define LCD_RST_Pin GPIO_PIN_10
#define LCD_RST_GPIO_Port GPIOA
#define PA15_Pin GPIO_PIN_15
#define PA15_GPIO_Port GPIOA
#define SDMMC1_CD_Pin GPIO_PIN_0
#define SDMMC1_CD_GPIO_Port GPIOD
#define SDMMC1_WP_Pin GPIO_PIN_1
#define SDMMC1_WP_GPIO_Port GPIOD
#define PD3_Pin GPIO_PIN_3
#define PD3_GPIO_Port GPIOD
#define PD3_EXTI_IRQn EXTI3_IRQn
#define PD4_Pin GPIO_PIN_4
#define PD4_GPIO_Port GPIOD
#define PD4_EXTI_IRQn EXTI4_IRQn
#define ADC_RVS_Pin GPIO_PIN_5
#define ADC_RVS_GPIO_Port GPIOD
#define SPI1_CS_Pin GPIO_PIN_6
#define SPI1_CS_GPIO_Port GPIOD
#define ADC_CONV_Pin GPIO_PIN_5
#define ADC_CONV_GPIO_Port GPIOB
#define ADC_RST_Pin GPIO_PIN_6
#define ADC_RST_GPIO_Port GPIOB
#define RANGE_MA_Pin GPIO_PIN_9
#define RANGE_MA_GPIO_Port GPIOB
#define RANGE_UA_Pin GPIO_PIN_0
#define RANGE_UA_GPIO_Port GPIOE
#define RANGE_NA_Pin GPIO_PIN_1
#define RANGE_NA_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */


#define BUTTON_ENTER_PORT	BUTTON_ENTER_GPIO_Port
#define BUTTON_ENTER_PIN	BUTTON_ENTER_Pin
#define BUTTON_ESC_PORT		BUTTON_ESC_GPIO_Port
#define BUTTON_ESC_PIN		BUTTON_ESC_Pin
#define BUTTON_UP_PORT		BUTTON_UP_GPIO_Port
#define BUTTON_UP_PIN		BUTTON_UP_Pin
#define BUTTON_DOWN_PORT	BUTTON_DOWN_GPIO_Port
#define BUTTON_DOWN_PIN		BUTTON_DOWN_Pin
#define BUTTON_LEFT_PORT	BUTTON_PREV_GPIO_Port
#define BUTTON_LEFT_PIN		BUTTON_PREV_Pin
#define BUTTON_RIGHT_PORT	BUTTON_NEXT_GPIO_Port
#define BUTTON_RIGHT_PIN	BUTTON_NEXT_Pin

#define BUTTON_MEASURE_PORT	BUTTON_MEASURING_GPIO_Port
#define BUTTON_MEASURE_PIN	BUTTON_MEASURING_Pin

#define BUZZER_PORT			BUZZER_GPIO_Port
#define BUZZER_PIN			BUZZER_Pin


#define LED_PORT			LED_RED_GPIO_Port
#define LED_PIN				LED_RED_Pin

#define LED_GREEN_PORT		LED_GREEN_GPIO_Port
#define LED_GREEN_PIN		LED_GREEN_Pin
#define LED_BLUE_PORT		LED_BLUE_GPIO_Port
#define LED_BLUE_PIN		LED_BLUE_Pin
#define LED_RED_PORT		LED_RED_GPIO_Port
#define LED_RED_PIN			LED_RED_Pin


#define PS_EN_PORT			PS_EN_GPIO_Port
#define PS_EN_PIN			PS_EN_Pin

#define EXTRA_GPIO_PORT		Extra_GPIO_GPIO_Port
#define EXTRA_GPIO_PIN		Extra_GPIO_Pin

#define USB_OTG_POWER_EN_PORT			USB_OTG_FS_VBUS_GPIO_Port
#define USB_OTG_POWER_EN_PIN			USB_OTG_FS_VBUS_Pin

/////// ADC ADS8691  ////////////
//INPUT COMMAND WORD AND REGISTER WRITE OPERATION
#define ADC_CLEAR_HWORD 0X60
#define ADC_READ_HWORD 0xCE	// 0xC8
#define ADC_READ 0x48
#define ADC_WRITE 0xD0
#define ADC_WRITE_MS 0xD2
#define ADC_WRITE_LS 0xD4
#define ADC_SET_HWORD 0xD8

//DEVICE CONFIGURATION AND REGISTER MAPS
#define ADC_DEVICE_ID_REG 0x02 //Device ID register
#define ADC_RST_PWRCTL_REG 0x04 //Reset and power control register
#define ADC_SDI_CTL_REG 0x08 //SDI data input control register
#define ADC_SDO_CTL_REG 0x0C //SDO-x data input control register
#define ADC_DATAOUT_CTL_REG 0x10 //Ouput data control register
#define ADC_RANGE_SEL_REG 0x14 //Input range selection control register
#define ADC_ALARM_REG 0x20 //ALARM output register
#define ADC_ALARM_H_TH_REG 0x24 //ALARM high threshold and hysteresis register
#define ADC_ALARM_L_TH_REG 0x28 //ALARM low threshold register

//INPUT RANGE SELECTION CONTROL REGISTER VALUES
#define ADC_RANGE_BIDIR_3VREF_INT_REF 0x0 //+- 3xVref, Internal reference
#define ADC_RANGE_BIDIR_25VREF_INT_REF 0x1 //+- 3xVref, Internal reference
#define ADC_RANGE_BIDIR_15VREF_INT_REF 0x2 //+- 3xVref, Internal reference
#define ADC_RANGE_BIDIR_125REF_INT_REF 0x3 //+- 3xVref, Internal reference
#define ADC_RANGE_BIDIR_0625VREF_INT_REF 0x4 //+- 3xVref, Internal reference
#define ADC_RANGE_UNIDIR_3VREF_INT_REF 0x8 //+- 3xVref, Internal reference
#define ADC_RANGE_UNIDIR_25VREF_INT_REF 0x9 //+- 3xVref, Internal reference
#define ADC_RANGE_UNIDIR_15VREF_INT_REF 0xA //+- 3xVref, Internal reference
#define ADC_RANGE_UNIDIR_125REF_INT_REF 0xB //+- 3xVref, Internal reference


// pins for range switching analog switchtransistors
#define RANGE_SELECT_PIN_AS_GND			ASW1_Pin
#define RANGE_SELECT_PIN_AS_GND_PORT	ASW1_GPIO_Port
#define RANGE_SELECT_PIN_AS_NA			ASW4_Pin
#define RANGE_SELECT_PIN_AS_NA_PORT		ASW4_GPIO_Port
#define RANGE_SELECT_PIN_AS_UA			ASW3_Pin
#define RANGE_SELECT_PIN_AS_UA_PORT		ASW3_GPIO_Port
#define RANGE_SELECT_PIN_AS_MA			ASW2_Pin
#define RANGE_SELECT_PIN_AS_MA_PORT		ASW2_GPIO_Port

// pins for range switching transistors
#define RANGE_SELECT_PIN_TRANS_NA		RANGE_NA_Pin
#define RANGE_SELECT_PIN_TRANS_NA_PORT	RANGE_NA_GPIO_Port
#define RANGE_SELECT_PIN_TRANS_UA		RANGE_UA_Pin
#define RANGE_SELECT_PIN_TRANS_UA_PORT	RANGE_UA_GPIO_Port
#define RANGE_SELECT_PIN_TRANS_MA		RANGE_MA_Pin
#define RANGE_SELECT_PIN_TRANS_MA_PORT	RANGE_MA_GPIO_Port

// pins for ADC control
#define ADC_RESET_PIN ADC_RST_Pin
#define ADC_RESET_PORT ADC_RST_GPIO_Port
#define ADC_RSV_PIN	ADC_RVS_Pin
#define ADC_RSV_PORT ADC_RVS_GPIO_Port
#define ADC_ALARM_PIN Extra_GPIO_Pin
#define ADC_ALARM_PORT Extra_GPIO_GPIO_Port
#define ADC_CONV_PIN ADC_CONV_Pin
#define ADC_CONV_PORT ADC_CONV_GPIO_Port


// define lower and upper limit for switching + change ratio (linear regresion --> derivation --> koef)
#define RANGE_UPPER_LIMIT_NA			2.
#define RANGE_LOWER_LIMIT_NA			0.001
#define RANGE_UPPER_CHANGE_RATIO_NA		0.3
#define RANGE_LOWER_CHANGE_RATIO_NA		0.3
#define RANGE_UPPER_TOTAL_LIMIT_NA 		4.75
#define RANGE_LOWER_TOTAL_LIMIT_NA		0.001

#define RANGE_UPPER_LIMIT_UA			RANGE_UPPER_LIMIT_NA
#define RANGE_LOWER_LIMIT_UA			RANGE_LOWER_LIMIT_NA
#define RANGE_UPPER_CHANGE_RATIO_UA		RANGE_UPPER_CHANGE_RATIO_NA
#define RANGE_LOWER_CHANGE_RATIO_UA		RANGE_LOWER_CHANGE_RATIO_NA
#define RANGE_UPPER_TOTAL_LIMIT_UA 		RANGE_UPPER_TOTAL_LIMIT_NA
#define RANGE_LOWER_TOTAL_LIMIT_UA		RANGE_LOWER_TOTAL_LIMIT_NA

#define RANGE_UPPER_LIMIT_MA			RANGE_UPPER_LIMIT_NA
#define RANGE_LOWER_LIMIT_MA			RANGE_LOWER_LIMIT_NA
#define RANGE_UPPER_CHANGE_RATIO_MA		RANGE_UPPER_CHANGE_RATIO_NA
#define RANGE_LOWER_CHANGE_RATIO_MA		RANGE_LOWER_CHANGE_RATIO_NA
#define RANGE_UPPER_TOTAL_LIMIT_MA 		RANGE_UPPER_TOTAL_LIMIT_NA
#define RANGE_LOWER_TOTAL_LIMIT_MA		RANGE_LOWER_TOTAL_LIMIT_NA

// define number of previous samples for decision
#define SAMPLES 10

#define SAMPLING_TOLERANCE 10

//#define CONSOLE_TIMEOUT 1000000 /// 1 sec
#define CONSOLE_TIMEOUT 5000 /// 5000 msec = 5 sec

#define AUTORANGE_MODE 0	// 0 - standard, only previous value; 1 - linear regression, more complex

// ADC parameters
// OLD config (default alfter restart)
//#define ADC_SCALE 262144		// 18 bit ADC
//#define ADC_REF_VALUE 4096		// ref voltage 4,096 V
//#define ADC_PGA 3				// PGA gain set to +- 3xVref

#define ADC_SCALE 262144		// 18 bit ADC
#define ADC_REF_VALUE 5000		// ref voltage 5,000 V
#define ADC_PGA 1.25				// PGA gain set to +- 1.25xVref
#define ADC_BIDIRECTIONAL 0			// 0 - unidirectional, 1 - bidirectional
#define ADC_DIRECTION 1		// 1 - unidirectional, 2 - bidirectional
//#define ADC_RESOLUTION ((ADC_REF_VALUE*ADC_PGA*ADC_DIRECTION)/ADC_SCALE)
//#define ADC_RESOLUTION 0.01953125‬

#define MS_BYTE(x) ((UINT8)(x >> 8))

#define LS_BYTE(x) ((UINT8)(x & 0xff))


// I2C EEPROM
#define EEPROM_ADDRESS			0xA0
#define EEPROM_MAXPKT			64              //(page size)		// original 32
#define EEPROM_WRITE			10              //time to wait in ms
#define EEPROM_TIMEOUT			5*EEPROM_WRITE  //timeout while writing
#define EEPROM_SECTIONSIZE		64				// default 64, for AT24C256 512 pages bz 64 bytes

// Power source
#define LT3045_NUMBER			1 		/// 1 - single LDO, 2 - paralel mode = 2 LDOs
#define POT_A_VALUE				50000 	// 50k ohm
#define POT_B_VALUE				50000	// 50k ohm
#define POT_A_STEPS				256 	// 8 bit pot
#define POT_B_STEPS				256 	// 8 bit pot
#define VOLTAGE_SENSE_CURRENT 	0.0001 	// 100 uA per LDO
#define CURRENT_BASE			150		// 150 mA / kOhm

struct deviceSettings{

	uint8_t isLoggingToConsole;
	uint8_t isLoggingToSD;
	uint32_t measuringInterval;
	uint32_t samplingInterval;
	uint8_t isAveraging;
	uint8_t isTriggerActive;
	double triggerLevel;
	double lastOffsetValue;
	uint16_t powerSourceVoltage;
	uint16_t powerSourceCurrent;
	uint8_t powerSourceEnable;
	uint8_t powerSourceEnableMode;
	uint8_t isLoggingToUSB;
	uint8_t isLoggingToEthernet;
	//double ADC_RESOLUTION;

};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
