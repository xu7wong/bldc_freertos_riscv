/*
 * hw.c
 *
 *  Created on: 6/07/2022
 *      Author: Carl
 */
#include "hw.h"
#include "ch32v30x_misc.h"
#include "ch32v30x_rcc.h"
#include "ch32v30x_adc.h"
#include "ch32v30x_gpio.h"
#include "ch32v30x_usart.h"

#include "timer.h"
#include "datatypes.h"
#include "mcconf_default.h"

//#include "ble.h"
//#include "can.h"
//uint16_t ADC_Value[ADC_TOTAL_CHANNELS];
int16_t Calibrattion_Val1 = 0;
int16_t Calibrattion_Val2 = 0;


//static int ble_write(BLEMode mode, char *buf, int size);
//static int ble_read(BLEMode mode, char *buf, int size, uint32_t timeout);


//volatile uint8_t ble_rx_buffer_index = 0;
//static uint8_t ble_tx_buffer_index = 0;

//static uint16_t ble_rx_DMA_index_old = 0;
//static uint16_t ble_rx_DMA_index_new = 0;

//volatile uint8_t ble_idle_flag = 0;
void hw_init_gpio(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA  | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure={0};

    /* TIM10_CH1/CH2/CH3 or Hall sensor input x3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* TIM1_CH1 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure );
    /* TIM1_CH1N */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    /* ADC CH0 ~ CH10 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* TIM2 */
    //GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
    //CH0
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //GPIO_Init( GPIOA, &GPIO_InitStructure );

    /* UART8 */
    GPIO_PinRemapConfig(GPIO_FullRemap_USART8, ENABLE);
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOE, &GPIO_InitStructure );
    //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOE, &GPIO_InitStructure );

    /* I2C1 */
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
    //SCK
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );
    //SDA
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    /* SPI1 */
    GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);
    //SCK & MOSI
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );
    //MISO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );
    //CS
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_SetBits(GPIOD, GPIO_Pin_7);

    /* UART2 */
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );
    //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );

    /* CAN1 */
    GPIO_PinRemapConfig( GPIO_Remap2_CAN1, ENABLE);
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure);
    //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init( GPIOD, &GPIO_InitStructure);

    /* SPI3 */
    GPIO_PinRemapConfig(GPIO_Remap_SPI3, ENABLE);
    //SCK & MOSI
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );
    //MISO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );
    //CS
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_15);

    /* LED */
    //LED1, LED2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    LED1_OFF();
    LED2_OFF();

//    BLE_AT_MODE();
}
void hw_init_peripherals(void){


//    can_bus_init();
//    ble_init();



}
uint16_t Get_ConversionVal1(int16_t val)
{
    if((val+Calibrattion_Val1)<0) return 0;
    if((Calibrattion_Val1+val)>4095) return 4095;
    return (val+Calibrattion_Val1);
}
uint16_t Get_ConversionVal2(int16_t val)
{
    if((val+Calibrattion_Val2)<0) return 0;
    if((Calibrattion_Val2+val)>4095) return 4095;
    return (val+Calibrattion_Val2);
}

void hw_setup_adc_channels(void){
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_7Cycles5 ); //0, VS1

    ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_7Cycles5 ); //1, CS1

    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_7Cycles5 ); //2, VS2

    ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 2, ADC_SampleTime_7Cycles5 ); //3, CS2

    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_7Cycles5 ); //4, VS3

    ADC_RegularChannelConfig(ADC2, ADC_Channel_7, 3, ADC_SampleTime_7Cycles5 ); //5, CS3

    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 4, ADC_SampleTime_7Cycles5 ); //6 ADC_Vin

    ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 4, ADC_SampleTime_7Cycles5 ); //7 ADC_EXT

    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 5, ADC_SampleTime_7Cycles5 ); //8 ADC_V12V

    ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 5, ADC_SampleTime_7Cycles5 );//9 Motor Temperature
}



