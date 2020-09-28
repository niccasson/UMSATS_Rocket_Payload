 /* USER CODE BEGIN Header */
 /**
   ******************************************************************************
   * @file    BoardDefinitions.c
   * @brief   Initializes constants for pin mappings of stm32f401re.
   ******************************************************************************
   * @attention
   *
   *  *  Created on: Jan. 15, 2020
  *      Author: niccasson
   *
   ******************************************************************************
   */
 /* USER CODE END Header */

  /* Includes ------------------------------------------------------------------*/
 #include "BoardDefinitions.h"

/* Pin mappings --------------------------------------------------------------*/
//Body Motors
Pin_Mapping front_fwd = {GPIOB, GPIO_PIN_9, 1};
Pin_Mapping front_rev = {GPIOB, GPIO_PIN_5, 1};
Pin_Mapping en_front_body = {GPIOA, GPIO_PIN_10, 0};
Pin_Mapping en_rear_body = {GPIOC, GPIO_PIN_1, 0};
Pin_Mapping rear_fwd = {GPIOA, GPIO_PIN_0, 2};
Pin_Mapping rear_rev = {GPIOA, GPIO_PIN_1, 2};
//Drive Arm Motors
Pin_Mapping left_fwd = {GPIOB, GPIO_PIN_6, 3};
Pin_Mapping left_rev = {GPIOB, GPIO_PIN_7, 3};
Pin_Mapping en_left_drive = {GPIOB, GPIO_PIN_8, 5};
Pin_Mapping en_right_drive = {GPIOC, GPIO_PIN_10, 5};
Pin_Mapping right_fwd = {GPIOC, GPIO_PIN_11, 4};
Pin_Mapping right_rev = {GPIOC, GPIO_PIN_12, 4};

Pin_Mapping temp = {GPIOB, GPIO_PIN_0};
Pin_Mapping bat_p = {GPIOA, GPIO_PIN_6};

/* Arrays -------------------------------------------------------------------------*/
Pin_Mapping *motorPins[MOTORPINCOUNT] = {&front_fwd, &front_rev, &en_front_body,
		&en_rear_body, &rear_fwd, &rear_rev, &left_fwd, &left_rev, &en_right_drive, &right_fwd,
		&right_rev, &en_left_drive};


 /************************ (C) COPYRIGHT UMSATS *****END OF FILE****/
