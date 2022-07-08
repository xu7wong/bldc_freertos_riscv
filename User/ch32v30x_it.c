/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_it.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main Interrupt Service Routines.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "ch32v30x_it.h"
#include "hw.h"
void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel1_IRQHandler(void)   __attribute__((interrupt("WCH-Interrupt-fast")));
/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}
//static volatile uint8_t debug = 0;
void TIM2_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM2, TIM_IT_CC2)!=RESET)
  {
      TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
      TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
//      if(debug){
//          debug = 0;
          GPIO_ResetBits(GPIOE, GPIO_Pin_1);
//      }
//      else{
//          debug = 1;
//          GPIO_SetBits(GPIOE, GPIO_Pin_1);
//      }
  }
}

void DMA1_Channel1_IRQHandler()
{
    if(DMA_GetITStatus(DMA1_IT_TC1)==SET ){
        DMA_ClearITPendingBit(DMA1_IT_TC1);

        GPIO_SetBits(GPIOE, GPIO_Pin_1);
#if 0
        printf("ADC 0~3 %d, %d, %d, %d\n",Get_ConversionVal1(ADC_Value[0]), Get_ConversionVal2(ADC_Value[2]), Get_ConversionVal1(ADC_Value[4]), Get_ConversionVal2(ADC_Value[6]));
        printf("ADC 4~7 %d, %d, %d, %d\n",Get_ConversionVal1(ADC_Value[8]), Get_ConversionVal2(ADC_Value[1]), Get_ConversionVal1(ADC_Value[3]), Get_ConversionVal2(ADC_Value[5]));
        printf("ADC 8~9 %d, %d\n",Get_ConversionVal1(ADC_Value[7]), Get_ConversionVal2(ADC_Value[9]));
#endif
    }
}
