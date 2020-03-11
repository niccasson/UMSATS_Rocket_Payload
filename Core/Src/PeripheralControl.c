 /* USER CODE BEGIN Header */
 /**
   ******************************************************************************
   * @file    PeripheralControl.c
   * @brief   Peripheral functions for Payload board.
   ******************************************************************************
   * @attention
   *
   *  *  Created on: Jan. 14, 2020
  *      Author: niccasson
   *
   ******************************************************************************
   */
 /* USER CODE END Header */

 /* Includes ------------------------------------------------------------------*/
 #include "PeripheralControl.h"
 #include "BoardDefinitions.h"
 #include "stm32f4xx_hal.h"
 #include "main.h"

 /******************************************************************************/
 /*          STM32F401RE GPIO_Output functions		         			       */
 /******************************************************************************/
 /**
   * @brief This function writes to motors.
 */
 void WriteMotor(Pin_Mapping motorPin, GPIO_PinState state)
 {
	 //Enable the motor
	 Pin_Mapping enable = GPIOB == motorPin.gpio ? en_A : en_B;
	 HAL_GPIO_WritePin(enable.gpio, enable.pinName, state);

	 //Write to the motor
	 HAL_GPIO_WritePin(motorPin.gpio, motorPin.pinName, state);
 }

 /******************************************************************************/
 /*          STM32F401RE ADC_Input and USART_Tx/Rx functions		       	   */
 /******************************************************************************/
 /**
   * @brief This function reads temp sensor data and returns it.
 */
 double GetVoltsValueADC(ADC_HandleTypeDef hadc, ADC_ChannelConfTypeDef adc_Config, uint32_t channel)
 {
	 adc_Config.Channel = channel;

	 double returnVal;

	 HAL_ADC_Start(&hadc);
	 if(HAL_ADC_PollForConversion(&hadc, 5) == HAL_OK)
		 returnVal = HAL_ADC_GetValue(&hadc);
	 HAL_ADC_Stop(&hadc);
	 return returnVal*ADC_TO_VOLTAGE;
 }
 /**
   * @brief This function reads temp sensor data and returns it.
 */
 double GetTempVal(void)
 {
	double returnVal = GetVoltsValueADC(hadc1, sConfig, ADC_CHANNEL_8)/TEMP_SENS_RESOLUTION;
	return returnVal;
 }
 /**
   * @brief This function reads BlueTooth data and stores it in the data buffer.
 */
 void ReadBlueTooth(char dataBuffer){
	 HAL_UART_Receive(&huart1, &dataBuffer, 1, 10);
 }

 /************************ (C) COPYRIGHT UMSATS *****END OF FILE****/
