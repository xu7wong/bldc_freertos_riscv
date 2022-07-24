/*
 * can.c
 *
 *  Created on: 24/07/2022
 *      Author: Carl
 */

#include "can.h"
#include "hw.h"
#include "ch32v30x_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
volatile CANRXPayload can_rx_payload;

static TaskHandle_t Task_Handler_Comm_CAN_Manager;

#define TASK_PRIO_COMM_CAN_MANAGER 7
#define TASK_STK_SIZE_COMM_CAN_MANAGER 512

static void task_comm_can_manager(void *pvParameters);

void can_bus_init(){
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_CAN1, ENABLE );

    CAN_InitTypeDef CAN_InitSturcture={0};
    CAN_FilterInitTypeDef CAN_FilterInitSturcture={0};

    CAN_InitSturcture.CAN_TTCM = DISABLE;
    CAN_InitSturcture.CAN_ABOM = DISABLE;
    CAN_InitSturcture.CAN_AWUM = DISABLE;
    CAN_InitSturcture.CAN_NART = ENABLE;
    CAN_InitSturcture.CAN_RFLM = DISABLE;
    CAN_InitSturcture.CAN_TXFP = DISABLE;
    CAN_InitSturcture.CAN_Mode = CAN_Mode_Normal;
    CAN_InitSturcture.CAN_SJW = CAN_SJW_1tq;
    CAN_InitSturcture.CAN_BS1 = CAN_BS1_6tq;
    CAN_InitSturcture.CAN_BS2 = CAN_BS2_5tq;
    CAN_InitSturcture.CAN_Prescaler = 48;
    CAN_Init( CAN1, &CAN_InitSturcture );
    //Bps =Fpclk1/((tpb1+1+tbs2+1+1)*brp)  -> 250Kbps

    CAN_FilterInitSturcture.CAN_FilterNumber = 0;
    CAN_FilterInitSturcture.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitSturcture.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitSturcture.CAN_FilterIdHigh = 0;
    CAN_FilterInitSturcture.CAN_FilterIdLow = 0;
    CAN_FilterInitSturcture.CAN_FilterMaskIdHigh = 0;
    CAN_FilterInitSturcture.CAN_FilterMaskIdLow = 0x0006;

    CAN_FilterInitSturcture.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitSturcture.CAN_FilterActivation = ENABLE;
    CAN_FilterInit( &CAN_FilterInitSturcture );

    NVIC_InitTypeDef NVIC_InitStructure={0};
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_ITConfig( CAN1, CAN_IT_FMP0, ENABLE );

    xTaskCreate((TaskFunction_t )task_comm_can_manager,
                (const char*    )"task_comm_can_manager",
                (uint16_t       )TASK_STK_SIZE_COMM_CAN_MANAGER,
                (void*          )NULL,
                (UBaseType_t    )TASK_PRIO_COMM_CAN_MANAGER,
                (TaskHandle_t*  )&Task_Handler_Comm_CAN_Manager);

}

void can_send_msg( uint8_t *msg, uint8_t len )
{
    if(len>8)return;
//    uint8_t mbox;
//    uint16_t i = 0;

    CanTxMsg CanTxStructure;

    CanTxStructure.StdId = 0x317;
    CanTxStructure.IDE = CAN_Id_Standard;

    CanTxStructure.RTR = CAN_RTR_Data;
    CanTxStructure.DLC = len;
    memcpy(CanTxStructure.Data, msg, len);
//    for( i=0; i<len; i++ )
//    {
//        CanTxStructure.Data[i] = msg[i];
//    }
    CAN_Transmit( CAN1, &CanTxStructure);
//    mbox = CAN_Transmit( CAN1, &CanTxStructure);
//    i = 0;

//    while( ( CAN_TransmitStatus( CAN1, mbox ) != CAN_TxStatus_Ok ) && (i < 0xFFF) )
//    {
//        i++;
//    }
//
//    if( i == 0xFFF )
//    {
//        return 1;//fail
//    }
//    else
//    {
//        return 0;
//    }
}

static void task_comm_can_manager(void *pvParameters){
    while(1)
    {
        if(can_rx_payload.finished){
            can_rx_payload.finished = 0;
        }

        vTaskDelay(1);
    }
}

