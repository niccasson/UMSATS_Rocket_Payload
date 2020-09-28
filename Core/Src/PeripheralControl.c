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
	 Pin_Mapping enable;
	 if(motorPin.en_flag == 1)
		 enable = en_front_body;
	 else if(motorPin.en_flag == 2)
		 enable = en_rear_body;
	 else if(motorPin.en_flag == 3)
		 enable = en_left_drive;
	 else if(motorPin.en_flag == 4)
		 enable = en_right_drive;
	 else
		 enable = en_front_body;

	 HAL_GPIO_WritePin(enable.gpio, enable.pinName, state);

	 //Write to the motor
	 HAL_GPIO_WritePin(motorPin.gpio, motorPin.pinName, state);
 }
 /**
   * @brief This function resets all motors.
 */
 void ResetMotors(void)
 {
	 for(int i = 0; i < MOTORPINCOUNT; i++)
		 HAL_GPIO_WritePin(motorPins[i]->gpio, motorPins[i]->pinName, GPIO_PIN_RESET);
 }

 /******************************************************************************/
 /*          STM32F401RE ADC_Input functions		       	                   */
 /******************************************************************************/

 /**
   * @brief This function reads temp sensor data and returns it.
 */
 double GetVoltsValueADC(ADC_HandleTypeDef hadc, ADC_ChannelConfTypeDef adc_Config)
 {
//	 adc_Config.Channel = channel;

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
 double GetTempVal(ADC_HandleTypeDef hadcx, ADC_ChannelConfTypeDef config)
 {
	double returnVal = GetVoltsValueADC(hadcx, config)/TEMP_SENS_RESOLUTION;
	return returnVal;
 }

 /******************************************************************************/
 /*          STM32F401RE USART_Tx/Rx functions		             	       	   */
 /******************************************************************************/

 /**
   * @brief This function reads BlueTooth data and stores it in the data buffer.
 */
 char ReadBlueTooth(UART_HandleTypeDef huartx){
	 uint8_t dataBuffer;
	 HAL_UART_Receive_DMA(&huartx, &dataBuffer, 1);
	 return (char)dataBuffer;
 }

 /************************ (C) COPYRIGHT UMSATS *****END OF FILE****/
