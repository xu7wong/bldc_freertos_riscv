/*
 * hw.h
 *
 *  Created on: 6/07/2022
 *      Author: Carl
 */

#ifndef VESC_HW_H_
#define VESC_HW_H_
#include "debug.h"

#define SYSTEM_CORE_CLOCK 144000000
#define ADC_TOTAL_CHANNELS 10

extern uint16_t ADC_Value[ADC_TOTAL_CHANNELS];
extern int16_t Calibrattion_Val1;
extern int16_t Calibrattion_Val2;

#define LED1_ON() GPIO_ResetBits(GPIOE, GPIO_Pin_0)
#define LED1_OFF() GPIO_SetBits(GPIOE, GPIO_Pin_0)
#define LED2_ON() GPIO_ResetBits(GPIOE, GPIO_Pin_1)
#define LED2_OFF() GPIO_SetBits(GPIOE, GPIO_Pin_1)

void hw_init_gpio(void);
void hw_setup_adc_channels(void);

uint16_t Get_ConversionVal1(int16_t val);
uint16_t Get_ConversionVal2(int16_t val);
#endif /* VESC_HW_H_ */
