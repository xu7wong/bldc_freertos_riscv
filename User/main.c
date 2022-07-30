/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

/*
 *@Note
 串口打印调试例程：
 USART1_Tx(PA9)。
 本例程演示使用 USART1(PA9) 作打印调试口输出。

*/

#include "debug.h"
#include "math.h"

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
    printf("SystemClk:%d\r\n",SystemCoreClock);


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

        //timer_sleep_ms(1);
        uint32_t t = timer_1by10milliseconds_elapsed_since(0);
        if(t - t_flag2 >=10){
            t_flag2 = t;
            thread_foc_run();
        }
        if(t - t_flag1 >=5000){
            t_flag1 = t;


            current_set = current_set + 0.5;

            if(current_set>=9.0){
                current_set = 1.0;

            }

            if(current_set>=7.0){
                            mc_interface_set_brake_current(15.0);
                        }
            else mc_interface_set_current(current_set);
            printf("now:%ld, %d\r\n",t_flag1, (uint16_t)current_set);
        }
        //LED2_ON();
        //mcpwm_foc_adc_int_handler();

        //LED2_OFF();
        //Delay_Ms(150);
    }
}

