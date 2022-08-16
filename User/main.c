/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

#include "math.h"
#include <stdlib.h>
#include "debug.h"

#include "hw.h"
#include "timer.h"
#include "mcpwm_foc.h"
//#include "can.h"

#include "utils.h"
#include "mcconf_default.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
uint32_t t_flag1 = 0;
uint32_t t_flag2 = 0;

float current_set = 1.0;
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);
    printf("\n\nSystemClk:%d\r\n",SystemCoreClock);


    timer_init();
    timer_sleep_ms(500);
    hw_init_gpio();
    timer_sleep_ms(500);

    //hw_init_peripherals();

    mc_interface_init();

    //printf("%d.%d%d\n", (int)val1, t, t1);
    //mc_interface_set_current((float)1.0);

    t_flag1 = timer_1by10milliseconds_elapsed_since(0);
    t_flag2 = timer_1by10milliseconds_elapsed_since(0);
    while(1){


        uint32_t t = timer_1by10milliseconds_elapsed_since(0);
        if(t - t_flag2 >=10){
            t_flag2 = t;
            thread_foc_run();
        }
        if(t - t_flag1 >=15000){
            t_flag1 = t;

            current_set = current_set + 0.5;

            if(current_set>=10.0){
                current_set = 1.0;

            }

            if(current_set>=9.0){
                mc_interface_set_brake_current(-20.0);
            }
            else mc_interface_set_current(-current_set);
            printf("now:%ld, %dA\r\n",t_flag1, (uint16_t)current_set);
            //float current = mcpwm_foc_get_tot_current_in_filtered_motor(false);
            //printf("i = %d.%d\n", (int16_t)current, ((int16_t)(abs(current*10.0)))%10);
        }

//        LED1_ON();
//        LED2_ON();
//        Delay_Ms(200);
//        LED1_OFF();
//        LED2_OFF();
//        Delay_Ms(200);
//        printf("adc:0=%d,2=%d,4=%d\r\n", ADC_Value[0], ADC_Value[2], ADC_Value[4]);
//        printf("adc:1=%d,3=%d,5=%d\r\n", ADC_Value[1], ADC_Value[3], ADC_Value[5]);
//
//        printf("adc:6=%d,7=%d,8=%d,9=%d,\r\n", ADC_Value[6], ADC_Value[7], ADC_Value[8], ADC_Value[9]);

    }
}

