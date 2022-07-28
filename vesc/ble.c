/*
 * ble.c
 *
 *  Created on: 14/07/2022
 *      Author: Carl
 */
#include "ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "ch32v30x_usart.h"
#include "ch32v30x_dma.h"
#include "ch32v30x_misc.h"
#include "ch32v30x_rcc.h"
#include "ch32v30x_gpio.h"

#define BUFFER_BLE_RX_SIZE 128
#define BUFFER_BLE_TX_SIZE 128
#define TASK_PRIO_BLE_READ 7
#define TASK_STK_SIZE_BLE_READ 512
#define TASK_PRIO_BLE_MANAGER 7
#define TASK_STK_SIZE_BLE_MANAGER 512
static TaskHandle_t Task_Handler_BLE_Read;
static TaskHandle_t Task_Handler_BLE_Manager;
static void task_ble_read(void *pvParameters);
static void task_ble_manager(void *pvParameters);
static uint8_t ble_rx_buffer[BUFFER_BLE_RX_SIZE];
static uint8_t ble_tx_buffer[BUFFER_BLE_TX_SIZE];

static BLEConfig ble_config;

void ble_init(void){

    ble_config.mode = BLE_AT;
    ble_config.config_status = BLE_RESET;
    memset(ble_config.device_name, 0, sizeof(ble_config.device_name));
    char* device_name = BLE_DEFAULT_NAME;
    memcpy(ble_config.device_name, device_name, strlen(device_name));

    /*
     * UART8 DMA TX + RX without interrupt
     */
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


    //    NVIC_InitTypeDef NVIC_InitStructure={0};
    //    NVIC_InitStructure.NVIC_IRQChannel = UART8_IRQn;
    //    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    //    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //    NVIC_Init(&NVIC_InitStructure);
    //
    //    USART_ITConfig(UART8, USART_IT_IDLE, ENABLE);


    DMA_InitTypeDef DMA_InitStructure={0};
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA2, ENABLE );
    DMA_DeInit(DMA2_Channel11);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART8->DATAR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ble_rx_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = BUFFER_BLE_RX_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init( DMA2_Channel11, &DMA_InitStructure );

    DMA_Cmd( DMA2_Channel11, ENABLE );

    DMA_DeInit(DMA2_Channel10);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART8->DATAR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ble_tx_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = BUFFER_BLE_TX_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init( DMA2_Channel10, &DMA_InitStructure );

    DMA_Cmd( DMA2_Channel10, ENABLE );

    USART_DMACmd(UART8, USART_DMAReq_Rx, ENABLE);

//    NVIC_InitTypeDef NVIC_InitStructure={0};
//            NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel10_IRQn;
//            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
//            NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//            NVIC_Init(&NVIC_InitStructure);
//
//
//    DMA_ITConfig( DMA2_Channel10, DMA_IT_TC, ENABLE );

    xTaskCreate((TaskFunction_t )task_ble_read,
            (const char*    )"task_ble_read",
            (uint16_t       )TASK_STK_SIZE_BLE_READ,
            (void*          )NULL,
            (UBaseType_t    )TASK_PRIO_BLE_READ,
            (TaskHandle_t*  )&Task_Handler_BLE_Read);
    xTaskCreate((TaskFunction_t )task_ble_manager,
            (const char*    )"task_ble_manager",
            (uint16_t       )TASK_STK_SIZE_BLE_MANAGER,
            (void*          )NULL,
            (UBaseType_t    )TASK_PRIO_BLE_MANAGER,
            (TaskHandle_t*  )&Task_Handler_BLE_Manager);
}
static int16_t at_command_valid(uint8_t* buffer, uint16_t size){
    if(size < 4 || size > 128 || size == 5 || size == 6){
        return -1;
    }
    if(size == 4){
        if(buffer[0] == 'O' && buffer[1] == 'K' && buffer[2]== '\r' && buffer[3]== '\n'){
            return 0;
        }
    }
    if(buffer[size-1]== '\n' && buffer[size-2]== '\r' && buffer[size-5]== '\n' && buffer[size-6]== '\r'){
        if(buffer[size-4] == 'O' && buffer[size-3] == 'K'){
            return size-6;
        }
    }
    return -1;
}
static void ble_dma_write(BLEMode mode, uint8_t* buffer, uint16_t len){
    memcpy(ble_tx_buffer, buffer, len);
    if(mode == BLE_AT)BLE_AT_MODE();
    else if(mode == BLE_TRANSPARENT)BLE_TRANSPARENT_MODE();
    ble_config.mode = mode;
    DMA_SetCurrDataCounter(DMA2_Channel10, len);
    USART_DMACmd(UART8, USART_DMAReq_Tx, ENABLE);
#if BLE_DEBUG
    if(mode == BLE_AT)printf("write-> %s\n", (char*)buffer);
#endif
}
static void task_ble_manager(void *pvParameters){
    while(1)
    {
        if(ble_config.config_status == BLE_RESET){
            char* msg = "AT+NAME?\r\n";
            ble_dma_write(BLE_AT, (uint8_t*)msg, strlen(msg));
//
//            memcpy(ble_tx_buffer, msg, strlen(msg));
//            BLE_AT_MODE();
//            bleConfig.mode = BLE_AT;
//            DMA_SetCurrDataCounter(DMA2_Channel10, strlen(msg));
//            USART_DMACmd(UART8, USART_DMAReq_Tx, ENABLE);
//#if BLE_DEBUG
//            printf("request ble name\n");
//#endif
        }
        else if(ble_config.config_status == BLE_NAME_ERROR){

            char* msg1 = "AT+NAME=";
            char* msg2 = "\r\n";
            char msg[strlen(msg1) + strlen(ble_config.device_name) + strlen(msg2)];
            memcpy(&msg[0], msg1, strlen(msg1));
            memcpy(&msg[0 + strlen(msg1)], ble_config.device_name, strlen(ble_config.device_name));
            memcpy(&msg[0 + strlen(msg1) + strlen(ble_config.device_name)], msg2, strlen(msg2));

            ble_dma_write(BLE_AT, (uint8_t*)msg, strlen(msg1) + strlen(ble_config.device_name) + strlen(msg2));
        }
        else if(ble_config.config_status == BLE_RENAMED){
            char* msg = "AT+RESET\r\n";
            ble_dma_write(BLE_AT, (uint8_t*)msg, strlen(msg));
        }
        else if(ble_config.config_status == BLE_NORMAL){
            char* msg = "AT+BLESTA?\r\n";
            ble_dma_write(BLE_AT, (uint8_t*)msg, strlen(msg));
        }
        else if(ble_config.config_status == BLE_BUSY){
#if BLE_DEBUG
            char* msg = "{\"heartbeat\": 1}";
            ble_dma_write(BLE_TRANSPARENT, (uint8_t*)msg, strlen(msg));
#endif
        }
        vTaskDelay(2000);
    }
}

static void task_ble_read(void *pvParameters){

    while(1)
    {
        uint16_t ble_rx_DMA_index_new = DMA_GetCurrDataCounter(DMA2_Channel11);
        if(USART_GetFlagStatus(UART8, USART_FLAG_IDLE) == SET && ble_rx_DMA_index_new != BUFFER_BLE_RX_SIZE){

            USART_ClearFlag(UART8, USART_FLAG_IDLE);
            DMA_Cmd( DMA2_Channel11, DISABLE );
            DMA_SetCurrDataCounter(DMA2_Channel11, BUFFER_BLE_RX_SIZE);
            uint16_t buffer_len = BUFFER_BLE_RX_SIZE - ble_rx_DMA_index_new;
#if BLE_DEBUG
            for(uint8_t i = 0; i < buffer_len; i++)
            {
                while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
                USART_SendData(USART3, ble_rx_buffer[i]);
            }
#endif
            if(ble_config.mode == BLE_AT){
                int16_t at_len = at_command_valid(ble_rx_buffer, buffer_len);
                if(ble_config.config_status == BLE_RESET){

                    if(at_len > 0){
                        if(at_len != strlen(ble_config.device_name)){
                            ble_config.config_status = BLE_NAME_ERROR;
                        }
                        else if(memcmp(ble_config.device_name, ble_rx_buffer, strlen(ble_config.device_name)) == 0){
                            ble_config.config_status = BLE_NORMAL;
                        }
                        else{
                            ble_config.config_status = BLE_NAME_ERROR;
                        }
                    }
                }
                else if(ble_config.config_status == BLE_NAME_ERROR){
                    if(at_len == 0){
                        ble_config.config_status = BLE_RENAMED;
                    }
                }
                else if(ble_config.config_status == BLE_NORMAL){
                    if(at_len > 0){
                        if(at_len == 2){
                            if(ble_rx_buffer[0] == '0' && ble_rx_buffer[1] == '5'){
                                ble_config.connect_status = BLE_CONNECTED;
                                BLE_TRANSPARENT_MODE();
                                ble_config.mode = BLE_TRANSPARENT;
                                ble_config.config_status = BLE_BUSY;
                            }
                            else{
                                ble_config.connect_status = BLE_DISCONNECTED;
                            }
                        }
                    }
                }

                //BLE_TRANSPARENT_MODE();
            }
            else if(ble_config.mode == BLE_TRANSPARENT){

            }
            DMA_Cmd( DMA2_Channel11, ENABLE );
        }
        vTaskDelay(1);
    }
}
