/*
	Copyright 2016 - 2020 Benjamin Vedder	benjamin@vedder.se

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

#include "mc_interface.h"
// #include "mcpwm.h"
#include "mcpwm_foc.h"
// #include "ledpwm.h"
//#include "stm32f4xx_conf.h"
#include "hw.h"
// #include "terminal.h"
#include "utils.h"
// #include "ch.h"
//#include "hal.h"
// #include "commands.h"
// #include "encoder.h"
// #include "buffer.h"
// #include "gpdrive.h"
// #include "comm_can.h"
// #include "shutdown.h"
// #include "app.h"
#include "utils.h"
// #include "mempools.h"
// #include "crc.h"
// #include "bms.h"
// #include "events.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// Macros
#define DIR_MULT		(motor_now()->m_conf.m_invert_direction ? -1.0 : 1.0)

// Global variables
volatile uint16_t ADC_Value[HW_ADC_CHANNELS + HW_ADC_CHANNELS_EXTRA];
volatile float ADC_curr_norm_value[6];

typedef struct {
	volatile mc_configuration m_conf;
	mc_fault_code m_fault_now;
	setup_stats m_stats;
	int m_ignore_iterations;
	int m_drv_fault_iterations;
	unsigned int m_cycles_running;
	bool m_lock_enabled;
	bool m_lock_override_once;
	float m_motor_current_sum;
	float m_input_current_sum;
	float m_motor_current_iterations;
	float m_input_current_iterations;
	float m_motor_id_sum;
	float m_motor_iq_sum;
	float m_motor_id_iterations;
	float m_motor_iq_iterations;
	float m_motor_vd_sum;
	float m_motor_vq_sum;
	float m_motor_vd_iterations;
	float m_motor_vq_iterations;
	float m_amp_seconds;
	float m_amp_seconds_charged;
	float m_watt_seconds;
	float m_watt_seconds_charged;
	float m_position_set;
	float m_temp_fet;
	float m_temp_motor;
	float m_gate_driver_voltage;
	float m_motor_current_unbalance;
	float m_motor_current_unbalance_error_rate;
	float m_f_samp_now;
	float m_input_voltage_filtered;
	float m_input_voltage_filtered_slower;

	// Backup data counters
	uint64_t m_odometer_last;
	uint64_t m_runtime_last;
} motor_if_state_t;

// Private variables
static volatile motor_if_state_t m_motor_1;
#ifdef HW_HAS_DUAL_MOTORS
static volatile motor_if_state_t m_motor_2;
#endif

// Sampling variables
//#define ADC_SAMPLE_MAX_LEN		2000
//__attribute__((section(".ram4"))) static volatile int16_t m_curr0_samples[ADC_SAMPLE_MAX_LEN];
//__attribute__((section(".ram4"))) static volatile int16_t m_curr1_samples[ADC_SAMPLE_MAX_LEN];
//__attribute__((section(".ram4"))) static volatile int16_t m_ph1_samples[ADC_SAMPLE_MAX_LEN];
//__attribute__((section(".ram4"))) static volatile int16_t m_ph2_samples[ADC_SAMPLE_MAX_LEN];
//__attribute__((section(".ram4"))) static volatile int16_t m_ph3_samples[ADC_SAMPLE_MAX_LEN];
//__attribute__((section(".ram4"))) static volatile int16_t m_vzero_samples[ADC_SAMPLE_MAX_LEN];
//__attribute__((section(".ram4"))) static volatile uint8_t m_status_samples[ADC_SAMPLE_MAX_LEN];
//__attribute__((section(".ram4"))) static volatile int16_t m_curr_fir_samples[ADC_SAMPLE_MAX_LEN];
//__attribute__((section(".ram4"))) static volatile int16_t m_f_sw_samples[ADC_SAMPLE_MAX_LEN];
//__attribute__((section(".ram4"))) static volatile int8_t m_phase_samples[ADC_SAMPLE_MAX_LEN];

static volatile int m_sample_len;
static volatile int m_sample_int;
static volatile bool m_sample_raw;
static volatile debug_sampling_mode m_sample_mode;
static volatile debug_sampling_mode m_sample_mode_last;
static volatile int m_sample_now;
static volatile int m_sample_trigger;
static volatile float m_last_adc_duration_sample;
static volatile bool m_sample_is_second_motor;
static volatile mc_fault_code m_fault_stop_fault;
static volatile bool m_fault_stop_is_second_motor;

// Private functions
// static void update_override_limits(volatile motor_if_state_t *motor, volatile mc_configuration *conf);
// static void run_timer_tasks(volatile motor_if_state_t *motor);
// static void update_stats(volatile motor_if_state_t *motor);
static volatile motor_if_state_t *motor_now(void);

// Function pointers
static void(*pwn_done_func)(void) = 0;


void mc_interface_init(void) {
	memset((void*)&m_motor_1, 0, sizeof(motor_if_state_t));
#ifdef HW_HAS_DUAL_MOTORS
	memset((void*)&m_motor_2, 0, sizeof(motor_if_state_t));
#endif
	confgenerator_set_defaults_mcconf((mc_configuration *)&m_motor_1.m_conf);
	// conf_general_read_mc_configuration((mc_configuration*)&m_motor_1.m_conf, false);
// #ifdef HW_HAS_DUAL_MOTORS
// 	conf_general_read_mc_configuration((mc_configuration*)&m_motor_2.m_conf, true);
// #endif

#ifdef HW_HAS_DUAL_MOTORS
	m_motor_1.m_conf.motor_type = MOTOR_TYPE_FOC;
	m_motor_2.m_conf.motor_type = MOTOR_TYPE_FOC;
#endif

	m_last_adc_duration_sample = 0.0;
	m_sample_len = 1000;
	m_sample_int = 1;
	m_sample_now = 0;
	m_sample_raw = false;
	m_sample_trigger = 0;
	m_sample_mode = DEBUG_SAMPLING_OFF;
	m_sample_mode_last = DEBUG_SAMPLING_OFF;
	m_sample_is_second_motor = false;

	mc_interface_stat_reset();

	m_motor_1.m_conf.lo_current_max = 80;//lo_max;
	m_motor_1.m_conf.lo_current_min = 0;//lo_min;

	m_motor_1.m_conf.lo_in_current_max = 80;//utils_min_abs(conf->l_in_current_max, lo_in_max);
	m_motor_1.m_conf.lo_in_current_min = 0;//utils_min_abs(conf->l_in_current_min, lo_in_min);
	
	m_motor_1.m_conf.lo_current_motor_max_now = 80;//conf->lo_current_max;
	m_motor_1.m_conf.lo_current_motor_min_now = 0;//conf->lo_current_min;

	// Start threads
	// chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);
	// chThdCreateStatic(sample_send_thread_wa, sizeof(sample_send_thread_wa), NORMALPRIO - 1, sample_send_thread, NULL);
	// chThdCreateStatic(fault_stop_thread_wa, sizeof(fault_stop_thread_wa), HIGHPRIO - 3, fault_stop_thread, NULL);
	// chThdCreateStatic(stat_thread_wa, sizeof(stat_thread_wa), NORMALPRIO, stat_thread, NULL);

	// int motor_old = mc_interface_get_motor_thread();
	mc_interface_select_motor_thread(1);
#ifdef HW_HAS_DRV8301
	// drv8301_set_oc_mode(motor_now()->m_conf.m_drv8301_oc_mode);
	// drv8301_set_oc_adj(motor_now()->m_conf.m_drv8301_oc_adj);
#elif defined(HW_HAS_DRV8320S)
	drv8320s_set_oc_mode(motor_now()->m_conf.m_drv8301_oc_mode);
	drv8320s_set_oc_adj(motor_now()->m_conf.m_drv8301_oc_adj);
#elif defined(HW_HAS_DRV8323S)
	drv8323s_set_oc_mode(motor_now()->m_conf.m_drv8301_oc_mode);
	drv8323s_set_oc_adj(motor_now()->m_conf.m_drv8301_oc_adj);
	DRV8323S_CUSTOM_SETTINGS();
#endif

#if defined HW_HAS_DUAL_MOTORS || defined HW_HAS_DUAL_PARALLEL
	mc_interface_select_motor_thread(2);
#ifdef HW_HAS_DRV8301
	drv8301_set_oc_mode(motor_now()->m_conf.m_drv8301_oc_mode);
	drv8301_set_oc_adj(motor_now()->m_conf.m_drv8301_oc_adj);
#elif defined(HW_HAS_DRV8320S)
	drv8320s_set_oc_mode(motor_now()->m_conf.m_drv8301_oc_mode);
	drv8320s_set_oc_adj(motor_now()->m_conf.m_drv8301_oc_adj);
#elif defined(HW_HAS_DRV8323S)
	drv8323s_set_oc_mode(motor_now()->m_conf.m_drv8301_oc_mode);
	drv8323s_set_oc_adj(motor_now()->m_conf.m_drv8301_oc_adj);
	DRV8323S_CUSTOM_SETTINGS();
#endif
#endif
	// mc_interface_select_motor_thread(motor_old);

	// Initialize encoder
	// switch (motor_now()->m_conf.m_sensor_port_mode) {
	// case SENSOR_PORT_MODE_ABI:
	// 	SENSOR_PORT_3V3();
	// 	encoder_init_abi(motor_now()->m_conf.m_encoder_counts);
	// 	break;

	// case SENSOR_PORT_MODE_AS5047_SPI:
	// 	SENSOR_PORT_3V3();
	// 	encoder_init_as5047p_spi();
	// 	break;

	// case SENSOR_PORT_MODE_MT6816_SPI:
	// 	encoder_init_mt6816_spi();
	// 	break;

	// case SENSOR_PORT_MODE_AD2S1205:
	// 	encoder_init_ad2s1205_spi();
	// 	break;

	// case SENSOR_PORT_MODE_SINCOS:
	// 	encoder_init_sincos(motor_now()->m_conf.foc_encoder_sin_gain, motor_now()->m_conf.foc_encoder_sin_offset,
	// 						motor_now()->m_conf.foc_encoder_cos_gain, motor_now()->m_conf.foc_encoder_cos_offset,
	// 						motor_now()->m_conf.foc_encoder_sincos_filter_constant);
	// 	break;

	// case SENSOR_PORT_MODE_TS5700N8501:
	// case SENSOR_PORT_MODE_TS5700N8501_MULTITURN: {
	// 	app_configuration *appconf = mempools_alloc_appconf();
	// 	conf_general_read_app_configuration(appconf);
	// 	if (appconf->app_to_use == APP_ADC ||
	// 			appconf->app_to_use == APP_UART ||
	// 			appconf->app_to_use == APP_PPM_UART ||
	// 			appconf->app_to_use == APP_ADC_UART) {
	// 		appconf->app_to_use = APP_NONE;
	// 		conf_general_store_app_configuration(appconf);
	// 	}
	// 	mempools_free_appconf(appconf);
	// 	encoder_init_ts5700n8501();
	// } break;

	// default:
	// 	SENSOR_PORT_5V();
	// 	break;
	// }

	// Initialize selected implementation
	switch (motor_now()->m_conf.motor_type) {

	case MOTOR_TYPE_FOC:
		mcpwm_foc_init(&m_motor_1.m_conf, &m_motor_1.m_conf);
		break;

	// case MOTOR_TYPE_GPD:
	// 	gpdrive_init(&motor_now()->m_conf);
	// 	break;

	default:
		break;
	}

	// bms_init((bms_config*)&m_motor_1.m_conf.bms);
}


void mc_interface_select_motor_thread(int motor) {
// #if defined HW_HAS_DUAL_MOTORS || defined HW_HAS_DUAL_PARALLEL
// 	if (motor == 0 || motor == 1 || motor == 2) {
// 		chThdGetSelfX()->motor_selected = motor;
// 	}
// #else
	(void)motor;
// #endif
}

// /**
//  * Get the motor selected for the current thread.
//  *
//  * @return
//  * 0: no specific motor selected, the last motor will be used.
//  * 1: motor 1 selected (default).
//  * 2: motor 2 selected.
//  */
// int mc_interface_get_motor_thread(void) {
// 	return 1;//chThdGetSelfX()->motor_selected;
// }

//const volatile mc_configuration* mc_interface_get_configuration(void) {
//	return &motor_now()->m_conf;
//}

//void mc_interface_set_configuration(mc_configuration *configuration) {
//	volatile motor_if_state_t *motor = motor_now();
//
//
//#ifdef HW_HAS_DRV8301
//	drv8301_set_oc_mode(configuration->m_drv8301_oc_mode);
//	drv8301_set_oc_adj(configuration->m_drv8301_oc_adj);
//#elif defined(HW_HAS_DRV8320S)
//	drv8320s_set_oc_mode(configuration->m_drv8301_oc_mode);
//	drv8320s_set_oc_adj(configuration->m_drv8301_oc_adj);
//#elif defined(HW_HAS_DRV8323S)
//	drv8323s_set_oc_mode(configuration->m_drv8301_oc_mode);
//	drv8323s_set_oc_adj(configuration->m_drv8301_oc_adj);
//#endif
//
//
//
//
//	motor->m_conf = *configuration;
//
//	//&motor->m_conf.lo_current_max = 80;
//	motor->m_conf.lo_current_max = 80;//lo_max;
//	motor->m_conf.lo_current_min = 0;//lo_min;
//
//	motor->m_conf.lo_in_current_max = 80;//utils_min_abs(conf->l_in_current_max, lo_in_max);
//	motor->m_conf.lo_in_current_min = 0;//utils_min_abs(conf->l_in_current_min, lo_in_min);
//
//	motor->m_conf.lo_current_motor_max_now = 80;//conf->lo_current_max;
//	motor->m_conf.lo_current_motor_min_now = 0;//conf->lo_current_min;
//
//
//	switch (motor->m_conf.motor_type) {
//
//		case MOTOR_TYPE_FOC:
//
//			mcpwm_foc_set_configuration(&motor->m_conf);
//			break;
//
//		default:
//			break;
//	}
//
//	// bms_init(&configuration->bms);
//}

bool mc_interface_dccal_done(void) {
	bool ret = false;
	switch (motor_now()->m_conf.motor_type) {

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_is_dccal_done();
		break;

	// case MOTOR_TYPE_GPD:
	// 	ret = gpdrive_is_dccal_done();
	// 	break;

	default:
		break;
	}

	return ret;
}

/**
 * Lock the control by disabling all control commands.
 */
void mc_interface_lock(void) {
	motor_now()->m_lock_enabled = true;
}

/**
 * Unlock all control commands.
 */
void mc_interface_unlock(void) {
	motor_now()->m_lock_enabled = false;
}

mc_fault_code mc_interface_get_fault(void) {
	return motor_now()->m_fault_now;
}

mc_state mc_interface_get_state(void) {
	mc_state ret = MC_STATE_OFF;
	switch (motor_now()->m_conf.motor_type) {

	case MOTOR_TYPE_FOC:
		ret = mcpwm_foc_get_state();
		break;

	default:
		break;
	}

	return ret;
}

void mc_interface_set_current(float current) {
	if (fabsf(current) > 0.001) {
		// SHUTDOWN_RESET();
	}

	if (mc_interface_try_input()) {
		return;
	}

	switch (motor_now()->m_conf.motor_type) {

	case MOTOR_TYPE_FOC:
		mcpwm_foc_set_current(DIR_MULT * current);
		break;

	default:
		break;
	}

	// events_add("set_current", current);
}

void mc_interface_set_brake_current(float current) {
	if (fabsf(current) > 0.001) {
		// SHUTDOWN_RESET();
	}

	if (mc_interface_try_input()) {
		return;
	}

	switch (motor_now()->m_conf.motor_type) {

	case MOTOR_TYPE_FOC:
		mcpwm_foc_set_brake_current(DIR_MULT * current);
		break;

	// case MOTOR_TYPE_GPD:
	// 	// For timeout to stop the output
	// 	gpdrive_set_mode(GPD_OUTPUT_MODE_NONE);
	// 	break;

	default:
		break;
	}

	// events_add("set_current_brake", current);
}

/**
 * Set current relative to the minimum and maximum current limits.
 *
 * @param current
 * The relative current value, range [-1.0 1.0]
 */
void mc_interface_set_current_rel(float val) {
	if (fabsf(val) > 0.001) {
		// SHUTDOWN_RESET();
	}

	mc_interface_set_current(val * motor_now()->m_conf.lo_current_motor_max_now);
}

/**
 * Disconnect the motor and let it turn freely.
 */
void mc_interface_release_motor(void) {
	mc_interface_set_current(0.0);
}




float mc_interface_get_input_voltage_filtered(void) {
	return motor_now()->m_input_voltage_filtered;
}

/**
 * Get filtered motor temperature. The temperature is pre-calculated, so this
 * functions is fast.
 *
 * @return
 * The filtered motor temperature.
 */
float mc_interface_temp_motor_filtered(void) {
	return motor_now()->m_temp_motor;
}

/**
 * Ignore motor control commands for this amount of time.
 */
void mc_interface_ignore_input(int time_ms) {
	volatile motor_if_state_t *motor = motor_now();

	if (time_ms > motor->m_ignore_iterations) {
		motor->m_ignore_iterations = time_ms;
	}
}

/**
 * Ignore motor control commands for this amount of time on both motors.
 */
void mc_interface_ignore_input_both(int time_ms) {
	if (time_ms > m_motor_1.m_ignore_iterations) {
		m_motor_1.m_ignore_iterations = time_ms;
	}
}

void mc_interface_set_current_off_delay(float delay_sec) {
	if (mc_interface_try_input()) {
		return;
	}

	UTILS_NAN_ZERO(delay_sec);
	if (delay_sec > 5.0) {
		delay_sec = 5.0;
	}

	switch (motor_now()->m_conf.motor_type) {

	case MOTOR_TYPE_FOC:
		mcpwm_foc_set_current_off_delay(delay_sec);
		break;

	default:
		break;
	}
}

// MC implementation functions

/**
 * A helper function that should be called before sending commands to control
 * the motor. If the state is detecting, the detection will be stopped.
 *
 * @return
 * The amount if milliseconds left until user commands are allowed again.
 *
 */
int mc_interface_try_input(void) {
	// TODO: Remove this later
	if (mc_interface_get_state() == MC_STATE_DETECTING) {
		// mcpwm_stop_pwm();
		// motor_now()->m_ignore_iterations = MCPWM_DETECT_STOP_TIME;
	}

	int retval = motor_now()->m_ignore_iterations;

	if (!motor_now()->m_ignore_iterations && motor_now()->m_lock_enabled) {
		if (!motor_now()->m_lock_override_once) {
			retval = 1;
		} else {
			motor_now()->m_lock_override_once = false;
		}
	}

	switch (motor_now()->m_conf.motor_type) {

	case MOTOR_TYPE_FOC:
		if (!mcpwm_foc_init_done()) {
			retval = 1;
		}
		break;

	default:
		break;
	}

	return retval;
}

void mc_interface_mc_timer_isr(bool is_second_motor) {
	// ledpwm_update_pwm();

#ifdef HW_HAS_DUAL_MOTORS
	volatile motor_if_state_t *motor = is_second_motor ? &m_motor_2 : &m_motor_1;
#else
	volatile motor_if_state_t *motor = &m_motor_1;
	(void)is_second_motor;
#endif

	volatile mc_configuration *conf_now = &motor->m_conf;
	const float input_voltage = GET_INPUT_VOLTAGE();
	UTILS_LP_FAST(motor->m_input_voltage_filtered, input_voltage, 0.02);

	// Check for faults that should stop the motor

	static float wrong_voltage_integrator = 0.0;
	float voltage_diff_now = 0.0;

	if (input_voltage < conf_now->l_min_vin) {
		voltage_diff_now = conf_now->l_min_vin - input_voltage;
	} else if (input_voltage > conf_now->l_max_vin) {
		voltage_diff_now = input_voltage - conf_now->l_max_vin;
	}

	if (voltage_diff_now > 1.0e-3) {
		wrong_voltage_integrator += voltage_diff_now;

		const float max_voltage = (conf_now->l_max_vin * 0.05);
		if (wrong_voltage_integrator > max_voltage) {
			// mc_interface_fault_stop(input_voltage < conf_now->l_min_vin ?
			// 		FAULT_CODE_UNDER_VOLTAGE : FAULT_CODE_OVER_VOLTAGE, is_second_motor, true);

			// Windup protection
			wrong_voltage_integrator = max_voltage * 2.0;
		}
	} else {
		if (wrong_voltage_integrator > 1.0) {
			wrong_voltage_integrator -= 1.0;
		} else {
			wrong_voltage_integrator = 0.0;
		}
	}

	// Fetch these values in a config-specific way to avoid some overhead of the general
	// functions. That will make this interrupt run a bit faster.
	mc_state state = MC_STATE_OFF;
	float current = 0;
	float current_filtered = 0;
	float current_in_filtered = 0;
	float abs_current = 0;
	float abs_current_filtered = 0;
	if (conf_now->motor_type == MOTOR_TYPE_FOC) {
		state = mcpwm_foc_get_state_motor(is_second_motor);
		current = mcpwm_foc_get_tot_current_motor(is_second_motor);
		current_filtered = mcpwm_foc_get_tot_current_filtered_motor(is_second_motor);
		current_in_filtered = mcpwm_foc_get_tot_current_in_filtered_motor(is_second_motor);
		abs_current = mcpwm_foc_get_abs_motor_current_motor(is_second_motor);
		abs_current_filtered = mcpwm_foc_get_abs_motor_current_filtered_motor(is_second_motor);
	} 
	// else {
	// 	state = mcpwm_get_state();
	// 	current = mcpwm_get_tot_current();
	// 	current_filtered = mcpwm_get_tot_current_filtered();
	// 	current_in_filtered = mcpwm_get_tot_current_in_filtered();
	// 	abs_current = mcpwm_get_tot_current();
	// 	abs_current_filtered = current_filtered;
	// }

	if (state == MC_STATE_RUNNING) {
		motor->m_cycles_running++;
	} else {
		motor->m_cycles_running = 0;
	}

	if (pwn_done_func) {
		pwn_done_func();
	}

	motor->m_motor_current_sum += current_filtered;
	motor->m_input_current_sum += current_in_filtered;
	motor->m_motor_current_iterations++;
	motor->m_input_current_iterations++;

	motor->m_motor_id_sum += mcpwm_foc_get_id();
	motor->m_motor_iq_sum += mcpwm_foc_get_iq();
	motor->m_motor_id_iterations++;
	motor->m_motor_iq_iterations++;

	motor->m_motor_vd_sum += mcpwm_foc_get_vd();
	motor->m_motor_vq_sum += mcpwm_foc_get_vq();
	motor->m_motor_vd_iterations++;
	motor->m_motor_vq_iterations++;


	float f_samp = motor->m_f_samp_now;

	// Watt and ah counters
	if (fabsf(current_filtered) > 1.0) {
		// Some extra filtering
		static float curr_diff_sum = 0.0;
		static float curr_diff_samples = 0;

		curr_diff_sum += current_in_filtered / f_samp;
		curr_diff_samples += 1.0 / f_samp;

		if (curr_diff_samples >= 0.01) {
			if (curr_diff_sum > 0.0) {
				motor->m_amp_seconds += curr_diff_sum;
				motor->m_watt_seconds += curr_diff_sum * input_voltage;
			} else {
				motor->m_amp_seconds_charged -= curr_diff_sum;
				motor->m_watt_seconds_charged -= curr_diff_sum * input_voltage;
			}

			curr_diff_samples = 0.0;
			curr_diff_sum = 0.0;
		}
	}

}

static volatile motor_if_state_t *motor_now(void) {
// #ifdef HW_HAS_DUAL_MOTORS
// 	return mc_interface_motor_now() == 1 ? &m_motor_1 : &m_motor_2;
// #else
	return &m_motor_1;
// #endif
}

void mc_interface_stat_reset(void) {
	volatile setup_stats *s = &motor_now()->m_stats;
	memset((void*)s, 0, sizeof(setup_stats));
	// s->time_start = chVTGetSystemTimeX();
	s->max_temp_mos = -300.0;
	s->max_temp_motor = -300.0;
}

