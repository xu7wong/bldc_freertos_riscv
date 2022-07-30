/*
 * mcpwm_foc.h
 *
 *  Created on: 6/07/2022
 *      Author: Carl
 */

#ifndef VESC_MCPWM_FOC_H_
#define VESC_MCPWM_FOC_H_

#include "datatypes.h"

void mcpwm_foc_init(volatile mc_configuration *conf_m1, volatile mc_configuration *conf_m2);
bool mcpwm_foc_init_done(void);
bool mcpwm_foc_is_dccal_done(void);
void mcpwm_foc_tim_sample_int_handler(void);
void mcpwm_foc_adc_int_handler(void);
int mcpwm_foc_dc_cal(bool cal_undriven);

void mcpwm_foc_set_current(float current);
void mcpwm_foc_set_brake_current(float current);

void mcpwm_foc_set_current_off_delay(float delay_sec);

// Functions where the motor can be selected
float mcpwm_foc_get_id(void);
float mcpwm_foc_get_iq(void);
float mcpwm_foc_get_vd(void);
float mcpwm_foc_get_vq(void);
float mcpwm_foc_get_tot_current_motor(bool is_second_motor);
float mcpwm_foc_get_tot_current_filtered_motor(bool is_second_motor);
float mcpwm_foc_get_tot_current_in_motor(bool is_second_motor);
float mcpwm_foc_get_tot_current_in_filtered_motor(bool is_second_motor);
float mcpwm_foc_get_abs_motor_current_motor(bool is_second_motor);
float mcpwm_foc_get_abs_motor_current_filtered_motor(bool is_second_motor);
mc_state mcpwm_foc_get_state_motor(bool is_second_motor);
mc_state mcpwm_foc_get_state(void);

void thread_foc_run(void);
#endif /* VESC_MCPWM_FOC_H_ */
