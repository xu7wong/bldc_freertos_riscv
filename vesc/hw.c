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
#include "ble.h"
#include "can.h"

#include "datatypes.h"
#include "mcconf_default.h"
//uint16_t ADC_Value[HW_ADC_CHANNELS];
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
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
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
//    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
//    //TX
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init( GPIOD, &GPIO_InitStructure );
//    //RX
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init( GPIOD, &GPIO_InitStructure );

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

//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
//    USART_InitTypeDef USART_InitStructure;
//    USART_InitStructure.USART_BaudRate = 115200;
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;
//    USART_InitStructure.USART_Parity = USART_Parity_No;
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_InitStructure.USART_Mode = USART_Mode_Tx;
//    USART_Init(USART2, &USART_InitStructure);
//    USART_Cmd(USART2, ENABLE);

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
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_7Cycles5 ); //VS1

    ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 1, ADC_SampleTime_7Cycles5 ); //CS1

    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_7Cycles5 ); //VS2

    ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 2, ADC_SampleTime_7Cycles5 ); //CS2

    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_7Cycles5 ); //VS3

    ADC_RegularChannelConfig(ADC2, ADC_Channel_7, 3, ADC_SampleTime_7Cycles5 ); //CS3

    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 4, ADC_SampleTime_7Cycles5 ); //ADC_Vin

    ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 4, ADC_SampleTime_7Cycles5 ); //ADC_EXT

    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 5, ADC_SampleTime_7Cycles5 ); //ADC_V12V

    ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 5, ADC_SampleTime_7Cycles5 );//Motor Temperature
}

uint8_t conf_general_calculate_deadtime(float deadtime_ns, float core_clock_freq) {
    uint8_t DTG = 0;
    float timebase = 1.0 / (core_clock_freq / 1000000.0) * 1000.0;

    if (deadtime_ns <= (timebase * 127.0)) {
        DTG = deadtime_ns / timebase;
    } else {
        if (deadtime_ns <= ((63.0 + 64.0) * 2.0 * timebase)) {
            DTG = deadtime_ns / (2.0 * timebase) - 64.0;
            DTG |= 0x80;
        } else {
            if (deadtime_ns <= ((31.0 + 32.0) * 8.0 * timebase)) {
                DTG = deadtime_ns / (8.0 * timebase) - 32.0;
                DTG |= 0xC0;
            } else {
                if (deadtime_ns <= ((31.0 + 32) * 16 * timebase)) {
                    DTG = deadtime_ns / (16.0 * timebase) - 32.0;
                    DTG |= 0xE0;
                } else {
                    // Deadtime requested is longer than max achievable. Set deadtime at
                    // longest possible value
                    DTG = 0xFF;
                    // assert_param(1); //catch this
                }
            }
        }
    }

    return DTG;
}

void confgenerator_set_defaults_mcconf(mc_configuration *conf) {
    conf->pwm_mode = MCCONF_PWM_MODE;
    conf->comm_mode = MCCONF_COMM_MODE;
    conf->motor_type = MCCONF_DEFAULT_MOTOR_TYPE;
    conf->sensor_mode = MCCONF_SENSOR_MODE;
    conf->l_current_max = MCCONF_L_CURRENT_MAX;
    conf->l_current_min = MCCONF_L_CURRENT_MIN;
    conf->l_in_current_max = MCCONF_L_IN_CURRENT_MAX;
    conf->l_in_current_min = MCCONF_L_IN_CURRENT_MIN;
    conf->l_abs_current_max = MCCONF_L_MAX_ABS_CURRENT;
    conf->l_min_erpm = MCCONF_L_RPM_MIN;
    conf->l_max_erpm = MCCONF_L_RPM_MAX;
    conf->l_erpm_start = MCCONF_L_RPM_START;
    conf->l_max_erpm_fbrake = MCCONF_L_CURR_MAX_RPM_FBRAKE;
    conf->l_max_erpm_fbrake_cc = MCCONF_L_CURR_MAX_RPM_FBRAKE_CC;
    conf->l_min_vin = MCCONF_L_MIN_VOLTAGE;
    conf->l_max_vin = MCCONF_L_MAX_VOLTAGE;
    conf->l_battery_cut_start = MCCONF_L_BATTERY_CUT_START;
    conf->l_battery_cut_end = MCCONF_L_BATTERY_CUT_END;
    conf->l_slow_abs_current = MCCONF_L_SLOW_ABS_OVERCURRENT;
    conf->l_temp_fet_start = MCCONF_L_LIM_TEMP_FET_START;
    conf->l_temp_fet_end = MCCONF_L_LIM_TEMP_FET_END;
    conf->l_temp_motor_start = MCCONF_L_LIM_TEMP_MOTOR_START;
    conf->l_temp_motor_end = MCCONF_L_LIM_TEMP_MOTOR_END;
    conf->l_temp_accel_dec = MCCONF_L_LIM_TEMP_ACCEL_DEC;
    conf->l_min_duty = MCCONF_L_MIN_DUTY;
    conf->l_max_duty = MCCONF_L_MAX_DUTY;
    conf->l_watt_max = MCCONF_L_WATT_MAX;
    conf->l_watt_min = MCCONF_L_WATT_MIN;
    conf->l_current_max_scale = MCCONF_L_CURRENT_MAX_SCALE;
    conf->l_current_min_scale = MCCONF_L_CURRENT_MIN_SCALE;
    conf->l_duty_start = MCCONF_L_DUTY_START;
    conf->sl_min_erpm = MCCONF_SL_MIN_RPM;
    conf->sl_min_erpm_cycle_int_limit = MCCONF_SL_MIN_ERPM_CYCLE_INT_LIMIT;
    conf->sl_max_fullbreak_current_dir_change = MCCONF_SL_MAX_FB_CURR_DIR_CHANGE;
    conf->sl_cycle_int_limit = MCCONF_SL_CYCLE_INT_LIMIT;
    conf->sl_phase_advance_at_br = MCCONF_SL_PHASE_ADVANCE_AT_BR;
    conf->sl_cycle_int_rpm_br = MCCONF_SL_CYCLE_INT_BR;
    conf->sl_bemf_coupling_k = MCCONF_SL_BEMF_COUPLING_K;
    conf->hall_table[0] = MCCONF_HALL_TAB_0;
    conf->hall_table[1] = MCCONF_HALL_TAB_1;
    conf->hall_table[2] = MCCONF_HALL_TAB_2;
    conf->hall_table[3] = MCCONF_HALL_TAB_3;
    conf->hall_table[4] = MCCONF_HALL_TAB_4;
    conf->hall_table[5] = MCCONF_HALL_TAB_5;
    conf->hall_table[6] = MCCONF_HALL_TAB_6;
    conf->hall_table[7] = MCCONF_HALL_TAB_7;
    conf->hall_sl_erpm = MCCONF_HALL_ERPM;
    conf->foc_current_kp = MCCONF_FOC_CURRENT_KP;
    conf->foc_current_ki = MCCONF_FOC_CURRENT_KI;
    conf->foc_f_zv = MCCONF_FOC_F_ZV;
    conf->foc_dt_us = MCCONF_FOC_DT_US;
    conf->foc_encoder_inverted = MCCONF_FOC_ENCODER_INVERTED;
    conf->foc_encoder_offset = MCCONF_FOC_ENCODER_OFFSET;
    conf->foc_encoder_ratio = MCCONF_FOC_ENCODER_RATIO;
    conf->foc_encoder_sin_gain = MCCONF_FOC_ENCODER_SIN_GAIN;
    conf->foc_encoder_cos_gain = MCCONF_FOC_ENCODER_COS_GAIN;
    conf->foc_encoder_sin_offset = MCCONF_FOC_ENCODER_SIN_OFFSET;
    conf->foc_encoder_cos_offset = MCCONF_FOC_ENCODER_COS_OFFSET;
    conf->foc_encoder_sincos_filter_constant = MCCONF_FOC_ENCODER_SINCOS_FILTER;
    conf->foc_sensor_mode = MCCONF_FOC_SENSOR_MODE;
    conf->foc_pll_kp = MCCONF_FOC_PLL_KP;
    conf->foc_pll_ki = MCCONF_FOC_PLL_KI;
    conf->foc_motor_l = MCCONF_FOC_MOTOR_L;
    conf->foc_motor_ld_lq_diff = MCCONF_FOC_MOTOR_LD_LQ_DIFF;
    conf->foc_motor_r = MCCONF_FOC_MOTOR_R;
    conf->foc_motor_flux_linkage = MCCONF_FOC_MOTOR_FLUX_LINKAGE;
    conf->foc_observer_gain = MCCONF_FOC_OBSERVER_GAIN;
    conf->foc_observer_gain_slow = MCCONF_FOC_OBSERVER_GAIN_SLOW;
    conf->foc_observer_offset = MCCONF_FOC_OBSERVER_OFFSET;
    conf->foc_duty_dowmramp_kp = MCCONF_FOC_DUTY_DOWNRAMP_KP;
    conf->foc_duty_dowmramp_ki = MCCONF_FOC_DUTY_DOWNRAMP_KI;
    conf->foc_openloop_rpm = MCCONF_FOC_OPENLOOP_RPM;
    conf->foc_openloop_rpm_low = MCCONF_FOC_OPENLOOP_RPM_LOW;
    conf->foc_d_gain_scale_start = MCCONF_FOC_D_GAIN_SCALE_START;
    conf->foc_d_gain_scale_max_mod = MCCONF_FOC_D_GAIN_SCALE_MAX_MOD;
    conf->foc_sl_openloop_hyst = MCCONF_FOC_SL_OPENLOOP_HYST;
    conf->foc_sl_openloop_time_lock = MCCONF_FOC_SL_OPENLOOP_T_LOCK;
    conf->foc_sl_openloop_time_ramp = MCCONF_FOC_SL_OPENLOOP_T_RAMP;
    conf->foc_sl_openloop_time = MCCONF_FOC_SL_OPENLOOP_TIME;
    conf->foc_hall_table[0] = MCCONF_FOC_HALL_TAB_0;
    conf->foc_hall_table[1] = MCCONF_FOC_HALL_TAB_1;
    conf->foc_hall_table[2] = MCCONF_FOC_HALL_TAB_2;
    conf->foc_hall_table[3] = MCCONF_FOC_HALL_TAB_3;
    conf->foc_hall_table[4] = MCCONF_FOC_HALL_TAB_4;
    conf->foc_hall_table[5] = MCCONF_FOC_HALL_TAB_5;
    conf->foc_hall_table[6] = MCCONF_FOC_HALL_TAB_6;
    conf->foc_hall_table[7] = MCCONF_FOC_HALL_TAB_7;
    conf->foc_hall_interp_erpm = MCCONF_FOC_HALL_INTERP_ERPM;
    conf->foc_sl_erpm = MCCONF_FOC_SL_ERPM;
    conf->foc_sample_v0_v7 = MCCONF_FOC_SAMPLE_V0_V7;
    conf->foc_sample_high_current = MCCONF_FOC_SAMPLE_HIGH_CURRENT;
    conf->foc_sat_comp = MCCONF_FOC_SAT_COMP;
    conf->foc_temp_comp = MCCONF_FOC_TEMP_COMP;
    conf->foc_temp_comp_base_temp = MCCONF_FOC_TEMP_COMP_BASE_TEMP;
    conf->foc_current_filter_const = MCCONF_FOC_CURRENT_FILTER_CONST;
    conf->foc_cc_decoupling = MCCONF_FOC_CC_DECOUPLING;
    conf->foc_observer_type = MCCONF_FOC_OBSERVER_TYPE;
    conf->foc_hfi_voltage_start = MCCONF_FOC_HFI_VOLTAGE_START;
    conf->foc_hfi_voltage_run = MCCONF_FOC_HFI_VOLTAGE_RUN;
    conf->foc_hfi_voltage_max = MCCONF_FOC_HFI_VOLTAGE_MAX;
    conf->foc_sl_erpm_hfi = MCCONF_FOC_SL_ERPM_HFI;
    conf->foc_hfi_start_samples = MCCONF_FOC_HFI_START_SAMPLES;
    conf->foc_hfi_obs_ovr_sec = MCCONF_FOC_HFI_OBS_OVR_SEC;
    conf->foc_hfi_samples = MCCONF_FOC_HFI_SAMPLES;
    conf->foc_offsets_cal_on_boot = MCCONF_FOC_OFFSETS_CAL_ON_BOOT;
    conf->foc_offsets_current[0] = MCCONF_FOC_OFFSETS_CURRENT_0;
    conf->foc_offsets_current[1] = MCCONF_FOC_OFFSETS_CURRENT_1;
    conf->foc_offsets_current[2] = MCCONF_FOC_OFFSETS_CURRENT_2;
    conf->foc_offsets_voltage[0] = MCCONF_FOC_OFFSETS_VOLTAGE_0;
    conf->foc_offsets_voltage[1] = MCCONF_FOC_OFFSETS_VOLTAGE_1;
    conf->foc_offsets_voltage[2] = MCCONF_FOC_OFFSETS_VOLTAGE_2;
    conf->foc_offsets_voltage_undriven[0] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_0;
    conf->foc_offsets_voltage_undriven[1] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_1;
    conf->foc_offsets_voltage_undriven[2] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_2;
    conf->foc_phase_filter_enable = MCCONF_FOC_PHASE_FILTER_ENABLE;
    conf->foc_phase_filter_max_erpm = MCCONF_FOC_PHASE_FILTER_MAX_ERPM;
    conf->foc_mtpa_mode = MCCONF_FOC_MTPA_MODE;
    conf->foc_fw_current_max = MCCONF_FOC_FW_CURRENT_MAX;
    conf->foc_fw_duty_start = MCCONF_FOC_FW_DUTY_START;
    conf->foc_fw_ramp_time = MCCONF_FOC_FW_RAMP_TIME;
    conf->foc_fw_q_current_factor = MCCONF_FOC_FW_Q_CURRENT_FACTOR;
    conf->gpd_buffer_notify_left = MCCONF_GPD_BUFFER_NOTIFY_LEFT;
    conf->gpd_buffer_interpol = MCCONF_GPD_BUFFER_INTERPOL;
    conf->gpd_current_filter_const = MCCONF_GPD_CURRENT_FILTER_CONST;
    conf->gpd_current_kp = MCCONF_GPD_CURRENT_KP;
    conf->gpd_current_ki = MCCONF_GPD_CURRENT_KI;
    conf->sp_pid_loop_rate = MCCONF_SP_PID_LOOP_RATE;
    conf->s_pid_kp = MCCONF_S_PID_KP;
    conf->s_pid_ki = MCCONF_S_PID_KI;
    conf->s_pid_kd = MCCONF_S_PID_KD;
    conf->s_pid_kd_filter = MCCONF_S_PID_KD_FILTER;
    conf->s_pid_min_erpm = MCCONF_S_PID_MIN_RPM;
    conf->s_pid_allow_braking = MCCONF_S_PID_ALLOW_BRAKING;
    conf->s_pid_ramp_erpms_s = MCCONF_S_PID_RAMP_ERPMS_S;
    conf->p_pid_kp = MCCONF_P_PID_KP;
    conf->p_pid_ki = MCCONF_P_PID_KI;
    conf->p_pid_kd = MCCONF_P_PID_KD;
    conf->p_pid_kd_proc = MCCONF_P_PID_KD_PROC;
    conf->p_pid_kd_filter = MCCONF_P_PID_KD_FILTER;
    conf->p_pid_ang_div = MCCONF_P_PID_ANG_DIV;
    conf->p_pid_gain_dec_angle = MCCONF_P_PID_GAIN_DEC_ANGLE;
    conf->p_pid_offset = MCCONF_P_PID_OFFSET;
    conf->cc_startup_boost_duty = MCCONF_CC_STARTUP_BOOST_DUTY;
    conf->cc_min_current = MCCONF_CC_MIN_CURRENT;
    conf->cc_gain = MCCONF_CC_GAIN;
    conf->cc_ramp_step_max = MCCONF_CC_RAMP_STEP;
    conf->m_fault_stop_time_ms = MCCONF_M_FAULT_STOP_TIME;
    conf->m_duty_ramp_step = MCCONF_M_RAMP_STEP;
    conf->m_current_backoff_gain = MCCONF_M_CURRENT_BACKOFF_GAIN;
    conf->m_encoder_counts = MCCONF_M_ENCODER_COUNTS;
    conf->m_sensor_port_mode = MCCONF_M_SENSOR_PORT_MODE;
    conf->m_invert_direction = MCCONF_M_INVERT_DIRECTION;
    conf->m_drv8301_oc_mode = MCCONF_M_DRV8301_OC_MODE;
    conf->m_drv8301_oc_adj = MCCONF_M_DRV8301_OC_ADJ;
    conf->m_bldc_f_sw_min = MCCONF_M_BLDC_F_SW_MIN;
    conf->m_bldc_f_sw_max = MCCONF_M_BLDC_F_SW_MAX;
    conf->m_dc_f_sw = MCCONF_M_DC_F_SW;
    conf->m_ntc_motor_beta = MCCONF_M_NTC_MOTOR_BETA;
    conf->m_out_aux_mode = MCCONF_M_OUT_AUX_MODE;
    conf->m_motor_temp_sens_type = MCCONF_M_MOTOR_TEMP_SENS_TYPE;
    conf->m_ptc_motor_coeff = MCCONF_M_PTC_MOTOR_COEFF;
    conf->m_hall_extra_samples = MCCONF_M_HALL_EXTRA_SAMPLES;
    conf->si_motor_poles = MCCONF_SI_MOTOR_POLES;
    conf->si_gear_ratio = MCCONF_SI_GEAR_RATIO;
    conf->si_wheel_diameter = MCCONF_SI_WHEEL_DIAMETER;
    conf->si_battery_type = MCCONF_SI_BATTERY_TYPE;
    conf->si_battery_cells = MCCONF_SI_BATTERY_CELLS;
    conf->si_battery_ah = MCCONF_SI_BATTERY_AH;
    conf->si_motor_nl_current = MCCONF_SI_MOTOR_NL_CURRENT;
    conf->bms.type = MCCONF_BMS_TYPE;
    conf->bms.t_limit_start = MCCONF_BMS_T_LIMIT_START;
    conf->bms.t_limit_end = MCCONF_BMS_T_LIMIT_END;
    conf->bms.soc_limit_start = MCCONF_BMS_SOC_LIMIT_START;
    conf->bms.soc_limit_end = MCCONF_BMS_SOC_LIMIT_END;
    conf->bms.fwd_can_mode = MCCONF_BMS_FWD_CAN_MODE;
}


