/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for the adc.c file
  * @author Petar Crnjak
  ******************************************************************************
  * @attention
  *
  * Copyright (c) Source robotics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

extern ADC_HandleTypeDef hadc1;

void MX_ADC1_Init(void);

void ADC_CHANNEL_6_SELECT_SENSE1(void);
int ADC_CHANNEL_6_READ_SENSE1(void);
extern volatile uint8_t adc_fast_on; // parameter 71: the register path for the ADC reads and the encoder's SPI
void ADC_fast_prepare(void);         // the sample times the register path needs

void ADC_CHANNEL_8_SELECT_SENSE2(void);
int ADC_CHANNEL_8_READ_SENSE2(void);

void ADC_CHANNEL_4_SELECT_VBUS(void);
int ADC_CHANNEL_4_READ_VBUS(void);

void ADC_CHANNEL_5_SELECT_TEMP(void);
int ADC_CHANNEL_5_READ_TEMP(void);

int READ_ALL_ADC_CHANNELS(void);

#ifdef __cplusplus
}
#endif

#endif