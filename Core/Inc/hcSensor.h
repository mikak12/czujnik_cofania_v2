/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hcSensor.h
  * @brief          : Header for hcSensor.c file.
  *                   Function for hcSensor
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HCSENSOR_H
#define __HCSENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

TIM_HandleTypeDef htim1;

uint64_t distanceCenter;
uint64_t distanceLeft;
uint64_t distanceRight;
uint64_t distance;
uint64_t valueRisingEdge;
uint64_t valueFallingEdge;
uint64_t measureTime;
uint8_t waitOnFallingEdge;

#define PIN_CENTER	GPIO_PIN_3
#define PORT_CENTER	GPIOF

#define PIN_LEFT	GPIO_PIN_5
#define PORT_LEFT	GPIOF

#define PIN_RIGHT	GPIO_PIN_7
#define PORT_RIGHT	GPIOF



struct HC_SR04
{

};

void uDelayTim1(uint64_t uSec);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
uint64_t triggerMeasureCenter(void);
uint64_t triggerMeasureLeft(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
