/*
 * timer.c
 *
 *  Created on: 6/07/2022
 *      Author: Carl
 */

#include "timer.h"
//#include "debug.h"
#include "ch32v30x_conf.h"
#define TIMER_HZ            12000000
#define TIMER_KHZ           12000
// system core clock 144000000
void timer_init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    uint16_t PrescalerValue = (uint16_t) (SystemCoreClock / TIMER_HZ) - 1;
    TIM_CounterModeConfig(TIM3, TIM_CounterMode_Up);
    TIM_CounterModeConfig(TIM4, TIM_CounterMode_Up);
    TIM_CounterModeConfig(TIM5, TIM_CounterMode_Up);
//    TIM_SetAutoreload(TIM1, 0xFFFF);
    TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);

    TIM_ITRxExternalClockConfig(TIM4, TIM_TS_ITR2);
    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_External1);
    TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Update);

    TIM_ITRxExternalClockConfig(TIM5, TIM_TS_ITR2);
    TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_External1);

    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
}
//uint32_t timer_count(uint8_t i){
//    if(i==1){
//        return (uint32_t)TIM3->CNT;
//    }
//    else if(i==2){
//        return (uint32_t)TIM4->CNT;
//    }
//    else if(i==3){
//        return (uint32_t)TIM5->CNT;
//    }
//    return 0;
//}
uint64_t timer_time_now(void) {
    return (uint64_t)TIM3->CNT | ((uint64_t)TIM4->CNT << 16) | ((uint64_t)TIM5->CNT << 32);
}
float timer_seconds_elapsed_since(uint64_t time) {
    uint64_t diff = timer_time_now() - time;
    return (float)diff / (float)TIMER_HZ;
}
uint32_t timer_milliseconds_elapsed_since(uint64_t time) {
    uint64_t diff = timer_time_now() - time;
    return (uint32_t)(diff / TIMER_KHZ);
}
void timer_sleep_ms(uint32_t ms){
    uint64_t start_t = timer_time_now();
    while(timer_milliseconds_elapsed_since(start_t) < ms);
}
