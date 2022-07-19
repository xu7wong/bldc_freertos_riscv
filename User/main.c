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
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hw.h"
#include "timer.h"
#include "mcpwm_foc.h"
//#include "ch32v30x_usbhs_device.h"
/* Global define */
#define TASK1_TASK_PRIO     5
#define TASK1_STK_SIZE      512
#define TASK2_TASK_PRIO     5
#define TASK2_STK_SIZE      512

/* Global Variable */
TaskHandle_t Task1Task_Handler;
TaskHandle_t Task2Task_Handler;

/*********************************************************************
 * @fn      task1_task
 *
 * @brief   task1 program.
 *
 * @param  *pvParameters - Parameters point of task1
 *
 * @return  none
 */
void task1_task(void *pvParameters)
{
    while(1)
    {
        //printf("task1 entry %ld\n", timer_milliseconds_elapsed_since(0));
        //for(uint8_t i = 0; i < ADC_TOTAL_CHANNELS/2; i++){
            //ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        //    vTaskDelay(1);
        //}

        LED1_ON();
        vTaskDelay(250);
        LED1_OFF();
        vTaskDelay(250);
        //printf("ADC 0~1 %d, %d\n",Get_ConversionVal2(ADC_Value[0]), Get_ConversionVal2(ADC_Value[1]));
        //printf("ADC 0~3 %d, %d, %d, %d\n",Get_ConversionVal1(ADC_Value[0]), Get_ConversionVal1(ADC_Value[2]), Get_ConversionVal1(ADC_Value[4]), Get_ConversionVal1(ADC_Value[6]));
        //printf("ADC 4~7 %d, %d, %d, %d\n",Get_ConversionVal1(ADC_Value[8]), Get_ConversionVal2(ADC_Value[1]), Get_ConversionVal2(ADC_Value[3]), Get_ConversionVal2(ADC_Value[5]));
        //printf("ADC 8~9 %d, %d\n",Get_ConversionVal2(ADC_Value[7]), Get_ConversionVal2(ADC_Value[9]));

        //int32_t val_mv = (Get_ConversionVal2(ADC_Value[9])*3300/4096);

        //printf("mv-T-%d,%0dC\n",val_mv ,TempSensor_Volt_To_Temper(val_mv));
    }
}

/*********************************************************************
 * @fn      task2_task
 *
 * @brief   task2 program.
 *
 * @param  *pvParameters - Parameters point of task2
 *
 * @return  none
 */
void task2_task(void *pvParameters)
{
    while(1)
    {

        //printf("task2 entry\n");
        //GPIO_ResetBits(GPIOE, GPIO_Pin_1);
        vTaskDelay(500);
        //GPIO_SetBits(GPIOE, GPIO_Pin_1);
        //vTaskDelay(500);

    }
}

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
	printf("FreeRTOS Kernel Version:%s\n",tskKERNEL_VERSION_NUMBER);
	timer_init();

	hw_init_gpio();
	timer_sleep_ms(100);
	hw_init_peripherals();

	mcpwm_foc_init();

	/* create two task */
    xTaskCreate((TaskFunction_t )task2_task,
                        (const char*    )"task2",
                        (uint16_t       )TASK2_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )TASK2_TASK_PRIO,
                        (TaskHandle_t*  )&Task2Task_Handler);

    xTaskCreate((TaskFunction_t )task1_task,
                    (const char*    )"task1",
                    (uint16_t       )TASK1_STK_SIZE,
                    (void*          )NULL,
                    (UBaseType_t    )TASK1_TASK_PRIO,
                    (TaskHandle_t*  )&Task1Task_Handler);
    vTaskStartScheduler();

	while(1)
	{
	    printf("shouldn't run at here!!\n");
	    timer_sleep_ms(500);
	}
}
