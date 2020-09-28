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

/* Definitions --------------------------------------------------------------*/
#define MOTORPINCOUNT 12

/* Structures for pin mapping ------------------------------------------------*/
typedef struct{
	GPIO_TypeDef* gpio;
	uint16_t pinName;
	uint8_t en_flag;
}Pin_Mapping;

/* Pin mappings --------------------------------------------------------------*/
//Body Motors
extern Pin_Mapping front_fwd;
extern Pin_Mapping front_rev;
extern Pin_Mapping en_front_body;
extern Pin_Mapping en_rear_body;
extern Pin_Mapping rear_fwd;
extern Pin_Mapping rear_rev;
//Drive Arm Motors
extern Pin_Mapping left_fwd;
extern Pin_Mapping left_rev;
extern Pin_Mapping en_left_drive;
extern Pin_Mapping en_right_drive;
extern Pin_Mapping right_fwd;
extern Pin_Mapping right_rev;

extern Pin_Mapping temp;

/* Arrays -------------------------------------------------------------------------*/
extern Pin_Mapping *motorPins[MOTORPINCOUNT];


///* Prototypes ---------------------------------------------------------------*/

#endif /* INC_BOARDDEFINITIONS_H_ */
