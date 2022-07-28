/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef MC_INTERFACE_H_
#define MC_INTERFACE_H_

// #include "conf_general.h"
#include "hw.h"

// Functions
void mc_interface_init(void);
void mc_interface_select_motor_thread(int motor);
//const volatile mc_configuration* mc_interface_get_configuration(void);
//void mc_interface_set_configuration(mc_configuration *configuration);
bool mc_interface_dccal_done(void);
void mc_interface_lock(void);
void mc_interface_unlock(void);
mc_fault_code mc_interface_get_fault(void);
mc_state mc_interface_get_state(void);

void mc_interface_set_current(float current);
void mc_interface_set_brake_current(float current);
void mc_interface_set_current_rel(float val);
void mc_interface_release_motor(void);
float mc_interface_get_input_voltage_filtered(void);
float mc_interface_temp_motor_filtered(void);

void mc_interface_ignore_input(int time_ms);
void mc_interface_ignore_input_both(int time_ms);
void mc_interface_set_current_off_delay(float delay_sec);

int mc_interface_try_input(void);
void mc_interface_mc_timer_isr(bool is_second_motor);
void mc_interface_stat_reset(void);
// External variables
extern volatile uint16_t ADC_Value[];
extern volatile float ADC_curr_norm_value[];

// Common fixed parameters
#ifndef HW_DEAD_TIME_NSEC
#define HW_DEAD_TIME_NSEC				360.0	// Dead time
#endif


#endif /* MC_INTERFACE_H_ */
