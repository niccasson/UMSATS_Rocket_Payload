/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    BoardDefinitions.h
  * @brief   Defines constants for pin mappings of stm32f401re.
  ******************************************************************************
  * @attention
  *
  *  *  Created on: Jan. 15, 2020
 *      Author: niccasson
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_BOARDDEFINITIONS_H_
#define INC_BOARDDEFINITIONS_H_

/* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx_hal.h"

/* Structures for pin mapping ------------------------------------------------*/
typedef struct{
	GPIO_TypeDef* gpio;
	uint16_t pinName;
}Pin_Mapping;

/* Pin mappings --------------------------------------------------------------*/
extern Pin_Mapping out1_A;
extern Pin_Mapping out2_A;
extern Pin_Mapping en_A;
extern Pin_Mapping en_B;
extern Pin_Mapping out1_B;
extern Pin_Mapping out2_B;

extern Pin_Mapping temp;

extern Pin_Mapping bluetooth_RX;

/* Prototypes ---------------------------------------------------------------*/
void x(void);

#endif /* INC_BOARDDEFINITIONS_H_ */
