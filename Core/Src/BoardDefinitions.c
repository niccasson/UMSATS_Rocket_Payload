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
Pin_Mapping out1_A = {GPIOB, GPIO_PIN_4};
Pin_Mapping out2_A = {GPIOB, GPIO_PIN_5};
Pin_Mapping en_A = {GPIOA, GPIO_PIN_10};
Pin_Mapping en_B = {GPIOC, GPIO_PIN_1};
Pin_Mapping out1_B = {GPIOA, GPIO_PIN_0};
Pin_Mapping out2_B = {GPIOA, GPIO_PIN_1};

Pin_Mapping temp = {GPIOB, GPIO_PIN_0};

Pin_Mapping bluetooth_RX = {GPIOB, GPIO_PIN_7};

 /******************************************************************************/
 /*          		          			X									   */
 /******************************************************************************/
 /* @brief This function does? */
 void x(void)
 {

 }


 /************************ (C) COPYRIGHT UMSATS *****END OF FILE****/
