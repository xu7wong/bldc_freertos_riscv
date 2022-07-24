/*
 * can.h
 *
 *  Created on: 24/07/2022
 *      Author: Carl
 */

#ifndef VESC_CAN_H_
#define VESC_CAN_H_

#include <stdint.h>

void can_bus_init();
void can_send_msg( uint8_t *msg, uint8_t len );

typedef struct
{
    uint8_t buffer[256];
    uint16_t length;
    uint8_t finished;
} CANRXPayload;

extern volatile CANRXPayload can_rx_payload;
#endif /* VESC_CAN_H_ */
