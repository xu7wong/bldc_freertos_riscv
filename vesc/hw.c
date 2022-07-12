/*
 * hw.c
 *
 *  Created on: 6/07/2022
 *      Author: Carl
 */
#include "hw.h"
#include "timer.h"
uint16_t ADC_Value[ADC_TOTAL_CHANNELS];
int16_t Calibrattion_Val1 = 0;
int16_t Calibrattion_Val2 = 0;

static void ble_write(BLEMode mode, char *buf, int size);
static int ble_read(BLEMode mode, char *buf, int size, uint32_t timeout);

void hw_init_gpio(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA  | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure={0};

    /* TIM10_CH1/CH2/CH3 or Hall sensor input x3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
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
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* TIM2 */
    //GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
    //CH0
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure );

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
    GPIO_PinRemapConfig( GPIO_Remap1_CAN1, ENABLE);
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure);
    //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init( GPIOB, &GPIO_InitStructure);

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

}
void hw_init_peripherals(void){
    USART_InitTypeDef USART_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8, ENABLE);
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(UART8, &USART_InitStructure);
    USART_Cmd(UART8, ENABLE);

    //BLE_AT_MODE();
    ble_write(BLE_AT, "AT+SHOW\r\n", 9);
    char buf[64];
    int r = ble_read(BLE_AT, buf, 64, 100);

    //r = ble_read(BLE_AT, &buf[16], 16, 100);
    int d = 0;
    d = 0;
    d = ble_read(BLE_AT, &buf[16], 16, 100);
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
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_7Cycles5 );
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_7Cycles5 );
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_7Cycles5 );
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 4, ADC_SampleTime_7Cycles5 );
    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_7Cycles5 );

    ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 1, ADC_SampleTime_7Cycles5 );
    ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 2, ADC_SampleTime_7Cycles5 );
    ADC_RegularChannelConfig(ADC2, ADC_Channel_7, 3, ADC_SampleTime_7Cycles5 );
    ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 4, ADC_SampleTime_7Cycles5 );
    ADC_RegularChannelConfig(ADC2, ADC_Channel_TempSensor, 5, ADC_SampleTime_7Cycles5 );
}
static void ble_write(BLEMode mode, char *buf, int size)
{
    if(mode == BLE_AT){
        BLE_AT_MODE();
    }
    else if(mode == BLE_TRANSPARENT){
        BLE_TRANSPARENT_MODE();
    }
    int i;

    for(i = 0; i < size; i++)
    {
        while(USART_GetFlagStatus(UART8, USART_FLAG_TC) == RESET);
        USART_SendData(UART8, *buf++);
    }
}
static int ble_read(BLEMode mode, char *buf, int size, uint32_t timeout){
    if(mode == BLE_AT){
        BLE_AT_MODE();
    }
    else if(mode == BLE_TRANSPARENT){
        BLE_TRANSPARENT_MODE();
    }
    int i;
    uint64_t t = timer_time_now();
    for(i = 0; i < size; i++)
    {
        while(USART_GetFlagStatus(UART8, USART_FLAG_RXNE) == RESET)
        {
            if(timer_milliseconds_elapsed_since(t) >= timeout){
                return i;
            }
        }
        buf[i] = USART_ReceiveData(UART8);
    }
    return i;
}
