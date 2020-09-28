/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    PeripheralControl.h
  * @brief   Defines of peripheral functions for Payload board.
  ******************************************************************************
  * @attention
  *
  *  *  Created on: Jan. 14, 2020
 *      Author: niccasson
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PERIPHERALCONTROL_H_
#define PERIPHERALCONTROL_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "BoardDefinitions.h"

 /* Definitions --------------------------------------------------------------*/
//#define AVG_SLOPE 2.5
//#define V25 0.76
#define TEMP_SENS_RESOLUTION 0.028
#define ADC_TO_VOLTAGE 0.01412

/* Exported functions prototypes ---------------------------------------------*/
void WriteMotor(Pin_Mapping motorPin, GPIO_PinState state);
double GetVoltsValueADC(ADC_HandleTypeDef hadc, ADC_ChannelConfTypeDef adc_Config);
char ReadBlueTooth(UART_HandleTypeDef huartx);
double GetTempVal(ADC_HandleTypeDef hadcx, ADC_ChannelConfTypeDef config);
void ResetMotors(void);

#endif /* PERIPHERALCONTROL_H_ */
/************************ (C) COPYRIGHT UMSATS *****END OF FILE****/
