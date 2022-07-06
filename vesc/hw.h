/*
 * hw.h
 *
 *  Created on: 6/07/2022
 *      Author: Carl
 */

#ifndef VESC_HW_H_
#define VESC_HW_H_
#include "debug.h"

#define SYSTEM_CORE_CLOCK 144000000

void hw_init_gpio(void);
void hw_setup_adc_channels(void);

#endif /* VESC_HW_H_ */
