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
 task1 and task2 alternate printing
 */
#include <string.h>
#include <math.h>
#include "debug.h"
//#include "FreeRTOS.h"
//#include "task.h"

#include "ch32v30x_misc.h"

#include "hw.h"
#include "timer.h"
#include "mcpwm_foc.h"
#include "can.h"

#include "utils.h"
#include "mcconf_default.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"
//#include "ch32v30x_usbhs_device.h"

//TaskHandle_t Task_Loader_Handler;

//void task_loader(void *pvParameters)
//{
//
//
//    mc_interface_set_current((float)4.0);
//    while(1)
//    {
//
//        //printf("task2 entry\n");
//        LED1_ON();
//        vTaskDelay(500);
//        LED1_OFF();
//        vTaskDelay(500);
//
//    }
//}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\n",SystemCoreClock);
//    printf("FreeRTOS Kernel Version:%s\n",tskKERNEL_VERSION_NUMBER);
    timer_init();

    hw_init_gpio();
    timer_sleep_ms(500);

    hw_init_peripherals();

    mc_interface_init();
//    float a = 2.0;
//    a = sqrtf(a);

    /* create two task */
//    xTaskCreate((TaskFunction_t )task2_task,
//            (const char*    )"task2",
//            (uint16_t       )TASK2_STK_SIZE,
//            (void*          )NULL,
//            (UBaseType_t    )TASK2_TASK_PRIO,
//            (TaskHandle_t*  )&Task2Task_Handler);

//    xTaskCreate((TaskFunction_t )task_loader,
//            (const char*    )"Task_Loader",
//            (uint16_t       )512,
//            (void*          )NULL,
//            (UBaseType_t    )5,
//            (TaskHandle_t*  )&Task_Loader_Handler);
//    vTaskStartScheduler();
//    mc_interface_set_current((float)4.0);
    while(1)
    {
//        printf("shouldn't run at here!!\n");

//        thread_foc_run();
        timer_sleep_ms(1);
    }
}
