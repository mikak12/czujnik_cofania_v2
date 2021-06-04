/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hcSensor.c
  * @brief          : 
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "hcSensor.h"

void uDelayTim1(uint64_t uSec)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < uSec);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	const float soundSpeed = 0.0343;
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if(waitOnFallingEdge==0)
		{
			valueRisingEdge = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			waitOnFallingEdge = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (waitOnFallingEdge == 1)
		{
			valueFallingEdge = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			__HAL_TIM_SET_COUNTER(htim, 0);

			if(valueFallingEdge >= valueRisingEdge)
			{
				measureTime = valueFallingEdge - valueRisingEdge;
			}
			else if (valueRisingEdge > valueFallingEdge)
			{
				measureTime = (0xffff - valueRisingEdge) + valueFallingEdge;
			}

			distance = measureTime * soundSpeed/2;
			waitOnFallingEdge = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);

		}
	}

}

uint64_t triggerMeasureCenter(void)
{
	HAL_GPIO_WritePin(PORT_CENTER, PIN_CENTER, GPIO_PIN_SET);
	uDelayTim1(10);
	HAL_GPIO_WritePin(PORT_CENTER, PIN_CENTER, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);

	distanceCenter = distance;

	return distanceCenter;
}

uint64_t triggerMeasureLeft(void)
{
	HAL_GPIO_WritePin(PORT_LEFT, PIN_LEFT, GPIO_PIN_SET);
	uDelayTim1(10);
	HAL_GPIO_WritePin(PORT_LEFT, PIN_LEFT, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);

	distanceLeft = distance;

	return distanceLeft;
}

uint64_t triggerMeasureRight(void)
{
	HAL_GPIO_WritePin(PORT_RIGHT, PIN_RIGHT, GPIO_PIN_SET);
	uDelayTim1(10);
	HAL_GPIO_WritePin(PORT_RIGHT, PIN_RIGHT, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);

	distanceRight = distance;

	return distanceRight;
}


/************************ (C) COPYRIGHT *****END OF FILE****/
