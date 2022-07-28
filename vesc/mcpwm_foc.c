/*
 * mcpwm_foc.c
 *
 *  Created on: 6/07/2022
 *      Author: Carl
 */
#include "mcpwm_foc.h"
#include "hw.h"
//#include "FreeRTOS.h"
//#include "task.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "ch32v30x_misc.h"
#include "ch32v30x_rcc.h"
#include "ch32v30x_gpio.h"
#include "ch32v30x_tim.h"
#include "ch32v30x_dma.h"
#include "ch32v30x_adc.h"

#include "datatypes.h"
#include "mc_interface.h"
#include "digital_filter.h"
#include "utils.h"
#include "timer.h"

#define TIMER_UPDATE_DUTY_M1(duty1, duty2, duty3) \
        TIM1->CTLR1 |= TIM_UDIS; \
        TIM1->CH1CVR = duty1; \
        TIM1->CH2CVR = duty2; \
        TIM1->CH3CVR = duty3; \
        TIM1->CTLR1 &= ~TIM_UDIS;


#define TIMER_UPDATE_SAMP(samp) \
        TIM2->CH2CVR = (samp / 2);

#define TIMER_UPDATE_SAMP_TOP_M1(samp, top) \
        TIM1->CTLR1 |= TIM_CR1_UDIS; \
        TIM2->CTLR1 |= TIM_UDIS; \
        TIM1->ATRLR = top; \
        TIM2->CH2CVR = samp / 2; \
        TIM1->CTLR1 &= ~TIM_UDIS; \
        TIM2->CTLR1 &= ~TIM_UDIS;
#define M_MOTOR(is_second_motor)  (((void)is_second_motor), &m_motor_1)


// Private types
typedef struct {
    float va;
    float vb;
    float vc;
    float v_mag_filter;
    float mod_alpha_filter;
    float mod_beta_filter;
    float mod_alpha_measured;
    float mod_beta_measured;
    float id_target;
    float iq_target;
    float max_duty;
    float duty_now;
    float phase;
    float phase_cos;
    float phase_sin;
    float i_alpha;
    float i_beta;
    float i_abs;
    float i_abs_filter;
    float i_bus;
    float v_bus;
    float v_alpha;
    float v_beta;
    float mod_d;
    float mod_q;
    float mod_q_filter;
    float id;
    float iq;
    float id_filter;
    float iq_filter;
    float vd;
    float vq;
    float vd_int;
    float vq_int;
    float speed_rad_s;
    uint32_t svm_sector;
    bool is_using_phase_filters;
} motor_state_t;
typedef struct {
    int sample_num;
    float avg_current_tot;
    float avg_voltage_tot;
} mc_sample_t;
typedef struct {
    void(*fft_bin0_func)(float*, float*, float*);
    void(*fft_bin1_func)(float*, float*, float*);
    void(*fft_bin2_func)(float*, float*, float*);

    int samples;
    int table_fact;
    float buffer[32];
    float buffer_current[32];
    bool ready;
    int ind;
    bool is_samp_n;
    float prev_sample;
    float angle;
    int est_done_cnt;
    float observer_zero_time;
    int flip_cnt;
} hfi_state_t;
typedef struct {
    volatile mc_configuration *m_conf;
    mc_state m_state;
    mc_control_mode m_control_mode;
    motor_state_t m_motor_state;
    float m_curr_unbalance;
    float m_currents_adc[3];
    bool m_phase_override;
    float m_phase_now_override;
    float m_duty_cycle_set;
    float m_id_set;
    float m_iq_set;
    float m_i_fw_set;
    float m_current_off_delay;
    float m_openloop_speed;
    float m_openloop_phase;
    bool m_output_on;
    float m_pos_pid_set;
    float m_speed_pid_set_rpm;
    float m_speed_command_rpm;
    float m_phase_now_observer;
    float m_phase_now_observer_override;
    float m_observer_x1_override;
    float m_observer_x2_override;
    bool m_phase_observer_override;
    float m_phase_now_encoder;
    float m_phase_now_encoder_no_index;
    float m_observer_x1;
    float m_observer_x2;
    float m_pll_phase;
    float m_pll_speed;
    mc_sample_t m_samples;
    int m_tachometer;
    int m_tachometer_abs;
    float m_pos_pid_now;
    float m_gamma_now;
    bool m_using_encoder;
    float m_speed_est_fast;
    float m_speed_est_faster;
    int m_duty1_next, m_duty2_next, m_duty3_next;
    bool m_duty_next_set;
    hfi_state_t m_hfi;
    int m_hfi_plot_en;
    float m_hfi_plot_sample;

    // For braking
    float m_br_speed_before;
    float m_br_vq_before;
    int m_br_no_duty_samples;

    float m_duty_abs_filtered;
    float m_duty_filtered;
    bool m_was_control_duty;
    float m_duty_i_term;
    float m_openloop_angle;
    float m_x1_prev;
    float m_x2_prev;
    float m_phase_before_speed_est;
    int m_tacho_step_last;
    float m_pid_div_angle_last;
    float m_pid_div_angle_accumulator;
    float m_min_rpm_hyst_timer;
    float m_min_rpm_timer;
    bool m_cc_was_hfi;
    float m_pos_i_term;
    float m_pos_prev_error;
    float m_pos_dt_int;
    float m_pos_prev_proc;
    float m_pos_dt_int_proc;
    float m_pos_d_filter;
    float m_pos_d_filter_proc;
    float m_speed_i_term;
    float m_speed_prev_error;
    float m_speed_d_filter;
    int m_ang_hall_int_prev;
    bool m_using_hall;
    float m_ang_hall;
    float m_ang_hall_rate_limited;
    float m_hall_dt_diff_last;
    float m_hall_dt_diff_now;

    // Resistance observer
    float m_r_est;
    float m_r_est_state;
} motor_all_state_t;

// Private variables
static volatile bool m_dccal_done = false;
// static volatile float m_last_adc_isr_duration;
static volatile bool m_init_done = false;
static volatile motor_all_state_t m_motor_1;
#ifdef HW_HAS_DUAL_MOTORS
static volatile motor_all_state_t m_motor_2;
#endif
static volatile int m_isr_motor = 0;

static volatile bool timer_thd_stop;
//TaskHandle_t Task_MCPCM_FOC_Timer_Handler;

static void timer_reinit(int f_zv);
static volatile motor_all_state_t *motor_now(void);
static void stop_pwm_hw(volatile motor_all_state_t *motor);
static void start_pwm_hw(volatile motor_all_state_t *motor);
static void timer_thread(void *pvParameters);
static void input_current_offset_measurement(void);
static void timer_update(volatile motor_all_state_t *motor, float dt);

static void pll_run(float phase, float dt, volatile float *phase_var,
        volatile float *speed_var, volatile mc_configuration *conf);
static void control_current(volatile motor_all_state_t *motor, float dt);
static void update_valpha_vbeta(volatile motor_all_state_t *motor, float mod_alpha, float mod_beta);
static void svm(float alpha, float beta, uint32_t PWMFullDutyCycle,
        uint32_t* tAout, uint32_t* tBout, uint32_t* tCout, uint32_t *svm_sector);
static float correct_hall(float angle, float dt, volatile motor_all_state_t *motor);

void observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta,
        float dt, volatile float *x1, volatile float *x2, volatile float *phase, volatile motor_all_state_t *motor);


void mcpwm_foc_init(volatile mc_configuration *conf_m1, volatile mc_configuration *conf_m2){
#ifndef HW_HAS_DUAL_MOTORS
    (void)conf_m2;
#endif

    m_init_done = false;

    // Initialize variables
    memset((void*)&m_motor_1, 0, sizeof(motor_all_state_t));
    m_isr_motor = 0;

    m_motor_1.m_conf = conf_m1;
    m_motor_1.m_state = MC_STATE_OFF;
    m_motor_1.m_control_mode = CONTROL_MODE_NONE;
    m_motor_1.m_hall_dt_diff_last = 1.0;
    // update_hfi_samples(m_motor_1.m_conf->foc_hfi_samples, &m_motor_1);

    // virtual_motor_init(conf_m1);


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1  , ENABLE );
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2  , ENABLE );
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    ADC_InitTypeDef ADC_InitStructure={0};

    ADC_DeInit(ADC1);
    ADC_DeInit(ADC2);

    ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = HW_ADC_CHANNELS/2;
    ADC_InitStructure.ADC_OutputBuffer = ADC_OutputBuffer_Disable;
    ADC_InitStructure.ADC_Pga = ADC_Pga_1;

    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_ExternalTrigConvCmd(ADC1, ENABLE);

    //ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_BufferCmd(ADC1, DISABLE);   //disable buffer
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    Calibrattion_Val1 = Get_CalibrationValue(ADC1);

    ADC_BufferCmd(ADC1, ENABLE);   //enable buffer

    //    ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
    //    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    //    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    //    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    //    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    //    ADC_InitStructure.ADC_NbrOfChannel = ADC_TOTAL_CHANNELS/2;
    //    ADC_InitStructure.ADC_OutputBuffer = ADC_OutputBuffer_Disable;
    //    ADC_InitStructure.ADC_Pga = ADC_Pga_1;

    ADC_Init(ADC2, &ADC_InitStructure);

    ADC_ExternalTrigConvCmd(ADC2, ENABLE);
    ADC_Cmd(ADC2, ENABLE);

    ADC_BufferCmd(ADC2, DISABLE);   //disable buffer
    ADC_ResetCalibration(ADC2);
    while(ADC_GetResetCalibrationStatus(ADC2));
    ADC_StartCalibration(ADC2);
    while(ADC_GetCalibrationStatus(ADC2));
    Calibrattion_Val2 = Get_CalibrationValue(ADC2);

    ADC_BufferCmd(ADC2, ENABLE);   //enable buffer

    //ADC_TempSensorVrefintCmd(ENABLE);

    DMA_InitTypeDef DMA_InitStructure={0};
    NVIC_InitTypeDef NVIC_InitStructure={0};

    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );

    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->RDATAR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC_Value;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = HW_ADC_CHANNELS/2;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init( DMA1_Channel1, &DMA_InitStructure );

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //DMA_ITConfig( DMA1_Channel1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE, ENABLE );
    DMA_ITConfig( DMA1_Channel1, DMA_IT_TC, ENABLE );
    DMA_Cmd( DMA1_Channel1, ENABLE );


    hw_setup_adc_channels();

    timer_reinit((int)m_motor_1.m_conf->foc_f_zv);

//    stop_pwm_hw(&m_motor_1);
//
//    if (m_motor_1.m_conf->foc_offsets_cal_on_boot) {
//        uint64_t cal_start_time = timer_time_now();//chVTGetSystemTimeX();
//        float cal_start_timeout = 10.0;
//
//        // Wait for input voltage to rise above minimum voltage
//        printf("foc cal 0: %d\n", (uint16_t)(mc_interface_get_input_voltage_filtered() * 1000));
//        while (mc_interface_get_input_voltage_filtered() < m_motor_1.m_conf->l_min_vin) {
//            printf("foc cal 1: %d\n", (uint16_t)(mc_interface_get_input_voltage_filtered() * 1000));
//            // vTaskDelay(10);
//            timer_sleep_ms(10);
//            if (UTILS_AGE_S(cal_start_time) >= cal_start_timeout) {
//                m_dccal_done = true;
//                printf("foc cal 1 start_timeout\n");
//                break;
//            }
//        }
//        printf("foc cal 2: %d\n", (uint16_t)(mc_interface_get_input_voltage_filtered() * 1000));
//
//        // Wait for input voltage to settle
//        if (!m_dccal_done) {
//            float v_in_last = mc_interface_get_input_voltage_filtered();
//            uint64_t v_in_stable_time = timer_time_now();//chVTGetSystemTimeX();
//            while (UTILS_AGE_S(v_in_stable_time) < 2.0) {
//                // vTaskDelay(1);
//                timer_sleep_ms(1);
//                //printf("foc cal 1: settle\n");
//                float v_in_now = mc_interface_get_input_voltage_filtered();
//                if (fabsf(v_in_now - v_in_last) > 1.5) {
//                    v_in_last = v_in_now;
//                    v_in_stable_time = timer_time_now();//chVTGetSystemTimeX();
//                }
//
//                if (UTILS_AGE_S(cal_start_time) >= cal_start_timeout) {
//                    m_dccal_done = true;
//                    printf("foc cal 2 start_timeout\n");
//                    break;
//                }
//            }
//        }
//
//        if (!m_dccal_done) {
//            for (int i = 0;i < 3;i++) {
//                m_motor_1.m_conf->foc_offsets_voltage[i] = 0.0;
//                m_motor_1.m_conf->foc_offsets_voltage_undriven[i] = 0.0;
//                m_motor_1.m_conf->foc_offsets_current[i] = 2048;
//
//#ifdef HW_HAS_DUAL_MOTORS
//                m_motor_2.m_conf->foc_offsets_voltage[i] = 0.0;
//                m_motor_2.m_conf->foc_offsets_voltage_undriven[i] = 0.0;
//                m_motor_2.m_conf->foc_offsets_current[i] = 2048;
//#endif
//            }
//            printf("mcpwm_foc_dc_cal run\n");
//            mcpwm_foc_dc_cal(false);
//            printf("mcpwm_foc_dc_cal finish\n");
//        }
//    } else {
//        m_dccal_done = true;
//    }
//
//    timer_thd_stop = false;
    //xTaskCreate((TaskFunction_t )timer_thread,
    //        (const char*    )"mcpwm_foc_timer",
    //        (uint16_t       )1024,
    //        (void*          )NULL,
    //        (UBaseType_t    )7,
    //        (TaskHandle_t*  )&Task_MCPCM_FOC_Timer_Handler);
    m_init_done = true;
}

void thread_foc_run(void) {
    // (void)arg;

    // chRegSetThreadName("foc timer");

    //for(;;) {
        const float dt = 0.001;

        if (timer_thd_stop) {
            timer_thd_stop = false;
            return;
        }

        timer_update(&m_motor_1, dt);
#ifdef HW_HAS_DUAL_MOTORS
        timer_update(&m_motor_2, dt);
#endif

        input_current_offset_measurement();


    //}
}

bool mcpwm_foc_init_done(void) {
    return m_init_done;
}
bool mcpwm_foc_is_dccal_done(void) {
    return m_dccal_done;
}

//void mcpwm_foc_set_configuration(volatile mc_configuration *configuration) {
//    motor_now()->m_conf = configuration;
//
//    // Below we check if anything in the configuration changed that requires stopping the motor.
//
//    uint32_t top = SYSTEM_CORE_CLOCK / (int)configuration->foc_f_zv;
//    if (TIM1->ARR != top) {
//#ifdef HW_HAS_DUAL_MOTORS
//        m_motor_1.m_control_mode = CONTROL_MODE_NONE;
//        m_motor_1.m_state = MC_STATE_OFF;
//        stop_pwm_hw(&m_motor_1);
//
//        m_motor_2.m_control_mode = CONTROL_MODE_NONE;
//        m_motor_2.m_state = MC_STATE_OFF;
//        stop_pwm_hw(&m_motor_2);
//
//        timer_reinit((int)configuration->foc_f_zv);
//#else
//        motor_now()->m_control_mode = CONTROL_MODE_NONE;
//        motor_now()->m_state = MC_STATE_OFF;
//        stop_pwm_hw(motor_now());
//        TIMER_UPDATE_SAMP_TOP_M1(MCPWM_FOC_CURRENT_SAMP_OFFSET, top);
//#ifdef  HW_HAS_DUAL_PARALLEL
//        TIMER_UPDATE_SAMP_TOP_M2(MCPWM_FOC_CURRENT_SAMP_OFFSET, top);
//#endif
//#endif
//    }
//
//    if (((1 << motor_now()->m_conf->foc_hfi_samples) * 8) != motor_now()->m_hfi.samples) {
//        motor_now()->m_control_mode = CONTROL_MODE_NONE;
//        motor_now()->m_state = MC_STATE_OFF;
//        stop_pwm_hw(motor_now());
//        // update_hfi_samples(motor_now()->m_conf->foc_hfi_samples, motor_now());
//    }
//
//    // virtual_motor_set_configuration(configuration);
//}

/**
 * Use current control and specify a goal current to use. The sign determines
 * the direction of the torque. Absolute values less than
 * conf->cc_min_current will release the motor.
 *
 * @param current
 * The current to use.
 */
void mcpwm_foc_set_current(float current) {
    motor_now()->m_control_mode = CONTROL_MODE_CURRENT;
    motor_now()->m_iq_set = current;
    motor_now()->m_id_set = 0;

    if (fabsf(current) < motor_now()->m_conf->cc_min_current) {
        return;
    }

    if (motor_now()->m_state != MC_STATE_RUNNING) {
        motor_now()->m_state = MC_STATE_RUNNING;
    }
}

/**
 * Brake the motor with a desired current. Absolute values less than
 * conf->cc_min_current will release the motor.
 *
 * @param current
 * The current to use. Positive and negative values give the same effect.
 */
void mcpwm_foc_set_brake_current(float current) {
    motor_now()->m_control_mode = CONTROL_MODE_CURRENT_BRAKE;
    motor_now()->m_iq_set = current;

    if (fabsf(current) < motor_now()->m_conf->cc_min_current) {
        return;
    }

    if (motor_now()->m_state != MC_STATE_RUNNING) {
        motor_now()->m_state = MC_STATE_RUNNING;
    }
}

/**
 * Get the direct axis motor current.
 *
 * @return
 * The D axis current.
 */
float mcpwm_foc_get_id(void) {
    return motor_now()->m_motor_state.id;
}

/**
 * Get the quadrature axis motor current.
 *
 * @return
 * The Q axis current.
 */
float mcpwm_foc_get_iq(void) {
    return motor_now()->m_motor_state.iq;
}
float mcpwm_foc_get_vd(void) {
    return motor_now()->m_motor_state.vd;
}

float mcpwm_foc_get_vq(void) {
    return motor_now()->m_motor_state.vq;
}
/**
 * Set current off delay. Prevent the current controller from switching off modulation
 * for target currents < cc_min_current for this amount of time.
 */
void mcpwm_foc_set_current_off_delay(float delay_sec) {
    if (motor_now()->m_current_off_delay < delay_sec) {
        motor_now()->m_current_off_delay = delay_sec;
    }
}

float mcpwm_foc_get_tot_current_motor(bool is_second_motor) {
    volatile motor_all_state_t *motor = M_MOTOR(is_second_motor);
    return SIGN(motor->m_motor_state.vq * motor->m_motor_state.iq) * motor->m_motor_state.i_abs;
}

float mcpwm_foc_get_tot_current_filtered_motor(bool is_second_motor) {
    volatile motor_all_state_t *motor = M_MOTOR(is_second_motor);
    return SIGN(motor->m_motor_state.vq * motor->m_motor_state.iq_filter) * motor->m_motor_state.i_abs_filter;
}

float mcpwm_foc_get_tot_current_in_motor(bool is_second_motor) {
    return M_MOTOR(is_second_motor)->m_motor_state.i_bus;
}

float mcpwm_foc_get_tot_current_in_filtered_motor(bool is_second_motor) {
    // TODO: Filter current?
    return M_MOTOR(is_second_motor)->m_motor_state.i_bus;
}

float mcpwm_foc_get_abs_motor_current_motor(bool is_second_motor) {
    return M_MOTOR(is_second_motor)->m_motor_state.i_abs;
}

float mcpwm_foc_get_abs_motor_current_filtered_motor(bool is_second_motor) {
    return M_MOTOR(is_second_motor)->m_motor_state.i_abs_filter;
}

mc_state mcpwm_foc_get_state_motor(bool is_second_motor) {
    return M_MOTOR(is_second_motor)->m_state;
}
mc_state mcpwm_foc_get_state(void) {
    return motor_now()->m_state;
}
/**
 * Calibrate voltage and current offsets. For the observer to work at low modulation it
 * is very important to get all current and voltage offsets right. Therefore we store
 * the offsets for when the motor is undriven and when it is driven separately. The
 * motor is driven at 50% modulation on all phases when measuring the driven offset, which
 * corresponds to space-vector modulation with 0 amplitude.
 *
 * cal_undriven:
 * Calibrate undriven voltages too. This requires the motor to stand still.
 *
 * return:
 * -1: Timed out while waiting for fault code to go away.
 * 1: Success
 *
 */
int mcpwm_foc_dc_cal(bool cal_undriven) {
    //    // Wait max 5 seconds for DRV-fault to go away
    //    int cnt = 0;
    //    while(IS_DRV_FAULT()){
    //        chThdSleepMS(1);
    //        cnt++;
    //        if (cnt > 5000) {
    //            return -1;
    //        }
    //    };
    //
    //    vTaskDelay(1000);
    timer_sleep_ms(1000);
    // // Disable timeout
    // systime_t tout = timeout_get_timeout_msec();
    // float tout_c = timeout_get_brake_current();
    // KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
    // timeout_reset();
    // timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

    // Measure driven offsets

    const float samples = 1000.0;
    float current_sum[3] = {0.0, 0.0, 0.0};
    float voltage_sum[3] = {0.0, 0.0, 0.0};

    TIMER_UPDATE_DUTY_M1(TIM1->ATRLR / 2, TIM1->ATRLR / 2, TIM1->ATRLR / 2);

    // Start PWM on phase 1
    stop_pwm_hw(&m_motor_1);
    // PHASE_FILTER_ON();
    TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

    // #ifdef HW_HAS_DUAL_MOTORS
    //  float current_sum_m2[3] = {0.0, 0.0, 0.0};
    //  float voltage_sum_m2[3] = {0.0, 0.0, 0.0};
    //  TIMER_UPDATE_DUTY_M2(TIM8->ARR / 2, TIM8->ARR / 2, TIM8->ARR / 2);

    //  stop_pwm_hw(&m_motor_2);
    //  PHASE_FILTER_ON_M2();
    //  TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_OCMode_PWM1);
    //  TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
    //  TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Enable);
    //  TIM_GenerateEvent(TIM8, TIM_EventSource_COM);
    // #endif

    //    vTaskDelay(10);
    timer_sleep_ms(10);
    for (float i = 0;i < samples;i++) {
        current_sum[0] += m_motor_1.m_currents_adc[0];
        voltage_sum[0] += ADC_VOLTS(ADC_IND_SENS1);
#ifdef HW_HAS_DUAL_MOTORS
        current_sum_m2[0] += m_motor_2.m_currents_adc[0];
        voltage_sum_m2[0] += ADC_VOLTS(ADC_IND_SENS4);
#endif
        //        vTaskDelay(1);
        timer_sleep_ms(1);
    }

    // Start PWM on phase 2
    stop_pwm_hw(&m_motor_1);
    // PHASE_FILTER_ON();
    TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

    // #ifdef HW_HAS_DUAL_MOTORS
    //  stop_pwm_hw(&m_motor_2);
    //  PHASE_FILTER_ON_M2();
    //  TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_OCMode_PWM1);
    //  TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
    //  TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Enable);
    //  TIM_GenerateEvent(TIM8, TIM_EventSource_COM);
    // #endif

    // chThdSleep(1);
    //    vTaskDelay(1);
    timer_sleep_ms(1);
    for (float i = 0;i < samples;i++) {
        current_sum[1] += m_motor_1.m_currents_adc[1];
        voltage_sum[1] += ADC_VOLTS(ADC_IND_SENS2);
#ifdef HW_HAS_DUAL_MOTORS
        current_sum_m2[1] += m_motor_2.m_currents_adc[1];
        voltage_sum_m2[1] += ADC_VOLTS(ADC_IND_SENS5);
#endif
        // chThdSleep(1);
        //        vTaskDelay(1);
        timer_sleep_ms(1);
    }

    // Start PWM on phase 3
    stop_pwm_hw(&m_motor_1);
    // PHASE_FILTER_ON();
    TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

    // #ifdef HW_HAS_DUAL_MOTORS
    //  stop_pwm_hw(&m_motor_2);
    //  PHASE_FILTER_ON_M2();
    //  TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_OCMode_PWM1);
    //  TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);
    //  TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Enable);
    //  TIM_GenerateEvent(TIM8, TIM_EventSource_COM);
    // #endif

    // chThdSleep(1);
    //    vTaskDelay(1);
    timer_sleep_ms(1);
    for (float i = 0;i < samples;i++) {
        current_sum[2] += m_motor_1.m_currents_adc[2];
        voltage_sum[2] += ADC_VOLTS(ADC_IND_SENS3);
#ifdef HW_HAS_DUAL_MOTORS
        current_sum_m2[2] += m_motor_2.m_currents_adc[2];
        voltage_sum_m2[2] += ADC_VOLTS(ADC_IND_SENS6);
#endif
        // chThdSleep(1);
        //        vTaskDelay(1);
        timer_sleep_ms(1);
    }

    stop_pwm_hw(&m_motor_1);

    m_motor_1.m_conf->foc_offsets_current[0] = current_sum[0] / samples;
    m_motor_1.m_conf->foc_offsets_current[1] = current_sum[1] / samples;
    m_motor_1.m_conf->foc_offsets_current[2] = current_sum[2] / samples;

    voltage_sum[0] /= samples;
    voltage_sum[1] /= samples;
    voltage_sum[2] /= samples;
    float v_avg = (voltage_sum[0] + voltage_sum[1] + voltage_sum[2]) / 3.0;

    m_motor_1.m_conf->foc_offsets_voltage[0] = voltage_sum[0] - v_avg;
    m_motor_1.m_conf->foc_offsets_voltage[1] = voltage_sum[1] - v_avg;
    m_motor_1.m_conf->foc_offsets_voltage[2] = voltage_sum[2] - v_avg;

#ifdef HW_HAS_DUAL_MOTORS
    stop_pwm_hw(&m_motor_2);

    m_motor_2.m_conf->foc_offsets_current[0] = current_sum_m2[0] / samples;
    m_motor_2.m_conf->foc_offsets_current[1] = current_sum_m2[1] / samples;
    m_motor_2.m_conf->foc_offsets_current[2] = current_sum_m2[2] / samples;

    voltage_sum_m2[0] /= samples;
    voltage_sum_m2[1] /= samples;
    voltage_sum_m2[2] /= samples;
    v_avg = (voltage_sum_m2[0] + voltage_sum_m2[1] + voltage_sum_m2[2]) / 3.0;

    m_motor_2.m_conf->foc_offsets_voltage[0] = voltage_sum_m2[0] - v_avg;
    m_motor_2.m_conf->foc_offsets_voltage[1] = voltage_sum_m2[1] - v_avg;
    m_motor_2.m_conf->foc_offsets_voltage[2] = voltage_sum_m2[2] - v_avg;
#endif

    // Measure undriven offsets

    if (cal_undriven) {
        //        vTaskDelay(10);
        timer_sleep_ms(10);
        voltage_sum[0] = 0.0; voltage_sum[1] = 0.0; voltage_sum[2] = 0.0;
#ifdef HW_HAS_DUAL_MOTORS
        voltage_sum_m2[0] = 0.0; voltage_sum_m2[1] = 0.0; voltage_sum_m2[2] = 0.0;
#endif

        for (float i = 0;i < samples;i++) {
            v_avg = (ADC_VOLTS(ADC_IND_SENS1) + ADC_VOLTS(ADC_IND_SENS2) + ADC_VOLTS(ADC_IND_SENS3)) / 3.0;
            voltage_sum[0] += ADC_VOLTS(ADC_IND_SENS1) - v_avg;
            voltage_sum[1] += ADC_VOLTS(ADC_IND_SENS2) - v_avg;
            voltage_sum[2] += ADC_VOLTS(ADC_IND_SENS3) - v_avg;
#ifdef HW_HAS_DUAL_MOTORS
            v_avg = (ADC_VOLTS(ADC_IND_SENS4) + ADC_VOLTS(ADC_IND_SENS5) + ADC_VOLTS(ADC_IND_SENS6)) / 3.0;
            voltage_sum_m2[0] += ADC_VOLTS(ADC_IND_SENS4) - v_avg;
            voltage_sum_m2[1] += ADC_VOLTS(ADC_IND_SENS5) - v_avg;
            voltage_sum_m2[2] += ADC_VOLTS(ADC_IND_SENS6) - v_avg;
#endif
            // chThdSleep(1);
            //            vTaskDelay(1);
            timer_sleep_ms(1);
        }

        stop_pwm_hw(&m_motor_1);

        voltage_sum[0] /= samples;
        voltage_sum[1] /= samples;
        voltage_sum[2] /= samples;

        m_motor_1.m_conf->foc_offsets_voltage_undriven[0] = voltage_sum[0];
        m_motor_1.m_conf->foc_offsets_voltage_undriven[1] = voltage_sum[1];
        m_motor_1.m_conf->foc_offsets_voltage_undriven[2] = voltage_sum[2];
#ifdef HW_HAS_DUAL_MOTORS
        stop_pwm_hw(&m_motor_2);

        voltage_sum_m2[0] /= samples;
        voltage_sum_m2[1] /= samples;
        voltage_sum_m2[2] /= samples;

        m_motor_2.m_conf->foc_offsets_voltage_undriven[0] = voltage_sum_m2[0];
        m_motor_2.m_conf->foc_offsets_voltage_undriven[1] = voltage_sum_m2[1];
        m_motor_2.m_conf->foc_offsets_voltage_undriven[2] = voltage_sum_m2[2];
#endif
    }

    // TODO: Make sure that offsets are no more than e.g. 5%, as larger values indicate hardware problems.

    // Enable timeout
    // timeout_configure(tout, tout_c, tout_ksw);
    mc_interface_unlock();

    m_dccal_done = true;

    return 1;
}

void mcpwm_foc_tim_sample_int_handler(void) {
    if (m_init_done) {
        // Generate COM event here for synchronization
        TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
        // TIM_GenerateEvent(TIM8, TIM_EventSource_COM);

        // virtual_motor_int_handler(
        //      m_motor_1.m_motor_state.v_alpha,
        //      m_motor_1.m_motor_state.v_beta);
    }
}

void mcpwm_foc_adc_int_handler(void) {
    // (void)p;
    // (void)flags;

    static int skip = 0;
    if (++skip == FOC_CONTROL_LOOP_FREQ_DIVIDER) {
        skip = 0;
    } else {
        return;
    }

    // uint32_t t_start = timer_time_now();

    bool is_v7 = !(TIM1->CTLR1 & TIM_DIR);
    int norm_curr_ofs = 0;

#ifdef HW_HAS_DUAL_MOTORS
    bool is_second_motor = is_v7;
    norm_curr_ofs = is_second_motor ? 3 : 0;
    volatile motor_all_state_t *motor_now = is_second_motor ? &m_motor_2 : &m_motor_1;
    volatile motor_all_state_t *motor_other = is_second_motor ? &m_motor_1 : &m_motor_2;
    m_isr_motor = is_second_motor ? 2 : 1;
#ifdef HW_HAS_3_SHUNTS
    volatile TIM_TypeDef *tim = TIM1;//is_second_motor ? TIM8 : TIM1;
#endif
#else
    volatile motor_all_state_t *motor_other = &m_motor_1;
    volatile motor_all_state_t *motor_now = &m_motor_1;;
    m_isr_motor = 1;
#ifdef HW_HAS_3_SHUNTS
    volatile TIM_TypeDef *tim = TIM1;
#endif
#endif

    volatile mc_configuration *conf_now = motor_now->m_conf;

    if (motor_other->m_duty_next_set) {
        motor_other->m_duty_next_set = false;
        // #ifdef HW_HAS_DUAL_MOTORS
        //      if (is_second_motor) {
        //          TIMER_UPDATE_DUTY_M1(motor_other->m_duty1_next, motor_other->m_duty2_next, motor_other->m_duty3_next);
        //      } else {
        //          TIMER_UPDATE_DUTY_M2(motor_other->m_duty1_next, motor_other->m_duty2_next, motor_other->m_duty3_next);
        //      }
        // #else
        TIMER_UPDATE_DUTY_M1(motor_now->m_duty1_next, motor_now->m_duty2_next, motor_now->m_duty3_next);
        // #ifdef HW_HAS_DUAL_PARALLEL
        //      TIMER_UPDATE_DUTY_M2(motor_now->m_duty1_next, motor_now->m_duty2_next, motor_now->m_duty3_next);
        // #endif
        // #endif
    }

#ifndef HW_HAS_DUAL_MOTORS
#ifdef HW_HAS_PHASE_SHUNTS
    if (!conf_now->foc_sample_v0_v7 && is_v7) {
        //return;
    }
#else
    if (is_v7) {
        return;
    }
#endif
#endif

    // Reset the watchdog
    // timeout_feed_WDT(THREAD_MCPWM);

#ifdef AD2S1205_SAMPLE_GPIO
    // force a position sample in the AD2S1205 resolver IC (falling edge)
    palClearPad(AD2S1205_SAMPLE_GPIO, AD2S1205_SAMPLE_PIN);
#endif

#ifdef HW_HAS_DUAL_MOTORS
    float curr0 = 0;
    float curr1 = 0;

    if (is_second_motor) {
        curr0 = GET_CURRENT1_M2();
        curr1 = GET_CURRENT2_M2();
    } else {
        curr0 = GET_CURRENT1();
        curr1 = GET_CURRENT2();
    }
#else
    float curr0 = GET_CURRENT1();
    float curr1 = GET_CURRENT2();
#ifdef HW_HAS_DUAL_PARALLEL
    curr0 += GET_CURRENT1_M2();
    curr1 += GET_CURRENT2_M2();
#endif
#endif

#ifdef HW_HAS_3_SHUNTS
#ifdef HW_HAS_DUAL_MOTORS
    float curr2 = is_second_motor ? GET_CURRENT3_M2() : GET_CURRENT3();
#else
    float curr2 = GET_CURRENT3();
#ifdef HW_HAS_DUAL_PARALLEL
    curr2 += GET_CURRENT3_M2();
#endif
#endif
#endif

    motor_now->m_currents_adc[0] = curr0;
    motor_now->m_currents_adc[1] = curr1;
#ifdef HW_HAS_3_SHUNTS
    motor_now->m_currents_adc[2] = curr2;
#else
    motor_now->m_currents_adc[2] = 0.0;
#endif

    curr0 -= conf_now->foc_offsets_current[0];
    curr1 -= conf_now->foc_offsets_current[1];
#ifdef HW_HAS_3_SHUNTS
    curr2 -= conf_now->foc_offsets_current[2];
    motor_now->m_curr_unbalance = curr0 + curr1 + curr2;
#endif

    ADC_curr_norm_value[0 + norm_curr_ofs] = curr0;
    ADC_curr_norm_value[1 + norm_curr_ofs] = curr1;
#ifdef HW_HAS_3_SHUNTS
    ADC_curr_norm_value[2 + norm_curr_ofs] = curr2;
#else
    ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);
#endif

    // Use the best current samples depending on the modulation state.
#ifdef HW_HAS_3_SHUNTS
    if (conf_now->foc_sample_high_current) {
        // High current sampling mode. Choose the lower currents to derive the highest one
        // in order to be able to measure higher currents.
        const float i0_abs = fabsf(ADC_curr_norm_value[0 + norm_curr_ofs]);
        const float i1_abs = fabsf(ADC_curr_norm_value[1 + norm_curr_ofs]);
        const float i2_abs = fabsf(ADC_curr_norm_value[2 + norm_curr_ofs]);

        if (i0_abs > i1_abs && i0_abs > i2_abs) {
            ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
        } else if (i1_abs > i0_abs && i1_abs > i2_abs) {
            ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
        } else if (i2_abs > i0_abs && i2_abs > i1_abs) {
            ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
        }
    } else {
#ifdef HW_HAS_PHASE_SHUNTS
        if (is_v7) {
            if (tim->CH1CVR > 500 && tim->CH2CVR > 500) {
                // Use the same 2 shunts on low modulation, as that will avoid jumps in the current reading.
                // This is especially important when using HFI.
                ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
            } else {
                if (tim->CH1CVR < tim->CH2CVR && tim->CH1CVR < tim->CH3CVR) {
                    ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
                } else if (tim->CH2CVR < tim->CH1CVR && tim->CH2CVR < tim->CH3CVR) {
                    ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
                } else if (tim->CH3CVR < tim->CH1CVR && tim->CH3CVR < tim->CH2CVR) {
                    ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
                }
            }
        } else {
            if (tim->CH1CVR < (tim->ATRLR - 500) && tim->CH2CVR < (tim->ATRLR - 500)) {
                // Use the same 2 shunts on low modulation, as that will avoid jumps in the current reading.
                // This is especially important when using HFI.
                ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
            } else {
                if (tim->CH1CVR > tim->CH2CVR && tim->CH1CVR > tim->CH3CVR) {
                    ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
                } else if (tim->CH2CVR > tim->CH1CVR && tim->CH2CVR > tim->CH3CVR) {
                    ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
                } else if (tim->CH3CVR > tim->CH1CVR && tim->CH3CVR > tim->CH2CVR) {
                    ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
                }
            }
        }
#else
        if (tim->CCR1 < (tim->ARR - 500) && tim->CCR2 < (tim->ARR - 500)) {
            // Use the same 2 shunts on low modulation, as that will avoid jumps in the current reading.
            // This is especially important when using HFI.
            ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
        } else {
            if (tim->CCR1 > tim->CCR2 && tim->CCR1 > tim->CCR3) {
                ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
            } else if (tim->CCR2 > tim->CCR1 && tim->CCR2 > tim->CCR3) {
                ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
            } else if (tim->CCR3 > tim->CCR1 && tim->CCR3 > tim->CCR2) {
                ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
            }
        }
#endif
    }
#endif

    float ia = ADC_curr_norm_value[0 + norm_curr_ofs] * FAC_CURRENT;
    float ib = ADC_curr_norm_value[1 + norm_curr_ofs] * FAC_CURRENT;
    //  float ic = -(ia + ib);

#ifdef HW_HAS_PHASE_SHUNTS
    float dt;
    if (conf_now->foc_sample_v0_v7) {
        dt = 1.0 / conf_now->foc_f_zv;
    } else {
        dt = 1.0 / (conf_now->foc_f_zv / 2.0);
    }
#else
    float dt = 1.0 / (conf_now->foc_f_zv / 2.0);
#endif

    // This has to be done for the skip function to have any chance at working with the
    // observer and control loops.
    // TODO: Test this.
    dt *= (float)FOC_CONTROL_LOOP_FREQ_DIVIDER;

    UTILS_LP_FAST(motor_now->m_motor_state.v_bus, GET_INPUT_VOLTAGE(), 0.1);

    volatile float enc_ang = 0;
    volatile bool encoder_is_being_used = false;

    // if (virtual_motor_is_connected()) {
    //  if (conf_now->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER ) {
    //      enc_ang = virtual_motor_get_angle_deg();
    //      encoder_is_being_used = true;
    //  }
    // } else {
    // if (encoder_is_configured()) {
    //  enc_ang = encoder_read_deg();
    //  encoder_is_being_used = true;
    // }
    // }

    if (encoder_is_being_used) {
        float phase_tmp = enc_ang;
        if (conf_now->foc_encoder_inverted) {
            phase_tmp = 360.0 - phase_tmp;
        }
        phase_tmp *= conf_now->foc_encoder_ratio;
        phase_tmp -= conf_now->foc_encoder_offset;
        utils_norm_angle((float*)&phase_tmp);
        motor_now->m_phase_now_encoder = DEG2RAD_f(phase_tmp);
    }

    if (motor_now->m_state == MC_STATE_RUNNING) {
        // Clarke transform assuming balanced currents
        motor_now->m_motor_state.i_alpha = ia;
        motor_now->m_motor_state.i_beta = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;

        const float duty_now = motor_now->m_motor_state.duty_now;
        const float duty_abs = fabsf(duty_now);
        const float vq_now = motor_now->m_motor_state.vq;
        const float speed_fast_now = motor_now->m_pll_speed;

        float id_set_tmp = motor_now->m_id_set;
        float iq_set_tmp = motor_now->m_iq_set;
        motor_now->m_motor_state.max_duty = conf_now->l_max_duty;

        if (motor_now->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
            utils_truncate_number_abs(&iq_set_tmp, -conf_now->lo_current_min);
        }

        UTILS_LP_FAST(motor_now->m_duty_abs_filtered, duty_abs, 0.01);
        utils_truncate_number_abs((float*)&motor_now->m_duty_abs_filtered, 1.0);

        UTILS_LP_FAST(motor_now->m_duty_filtered, duty_now, 0.01);
        utils_truncate_number_abs((float*)&motor_now->m_duty_filtered, 1.0);

        float duty_set = motor_now->m_duty_cycle_set;
        bool control_duty = motor_now->m_control_mode == CONTROL_MODE_DUTY ||
                motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY ||
                motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY_PHASE;

        // Short all phases (duty=0) the moment the direction or modulation changes sign. That will avoid
        // active braking or changing direction. Keep all phases shorted (duty == 0) until the
        // braking current reaches the set or maximum value, then go back to current control
        // mode. Stay in duty=0 for at least 10 cycles to avoid jumping in and out of that mode rapidly
        // around the threshold.
        if (motor_now->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
            if ((SIGN(speed_fast_now) != SIGN(motor_now->m_br_speed_before) ||
                    SIGN(vq_now) != SIGN(motor_now->m_br_vq_before) ||
                    fabsf(motor_now->m_duty_filtered) < 0.001 || motor_now->m_br_no_duty_samples < 10) &&
                    motor_now->m_motor_state.i_abs_filter < fabsf(iq_set_tmp)) {
                control_duty = true;
                duty_set = 0.0;
                motor_now->m_br_no_duty_samples = 0;
            } else if (motor_now->m_br_no_duty_samples < 10) {
                control_duty = true;
                duty_set = 0.0;
                motor_now->m_br_no_duty_samples++;
            }
        } else {
            motor_now->m_br_no_duty_samples = 0;
        }

        motor_now->m_br_speed_before = speed_fast_now;
        motor_now->m_br_vq_before = vq_now;

        // Brake when set ERPM is below min ERPM
        if (motor_now->m_control_mode == CONTROL_MODE_SPEED &&
                fabsf(motor_now->m_speed_pid_set_rpm) < conf_now->s_pid_min_erpm) {
            control_duty = true;
            duty_set = 0.0;
        }

        // Reset integrator when leaving duty cycle mode, as the windup protection is not too fast. Making
        // better windup protection is probably better, but not easy.
        if (!control_duty && motor_now->m_was_control_duty) {
            motor_now->m_motor_state.vq_int = motor_now->m_motor_state.vq;
            if (conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_BEMF ||
                    conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS_BEMF) {
                motor_now->m_motor_state.vq_int -= motor_now->m_motor_state.speed_rad_s * conf_now->foc_motor_flux_linkage;
            }
        }
        motor_now->m_was_control_duty = control_duty;

        if (!control_duty) {
            motor_now->m_duty_i_term = motor_now->m_motor_state.iq / conf_now->lo_current_max;
        }

        if (control_duty) {
            // Duty cycle control
            if (fabsf(duty_set) < (duty_abs - 0.05) ||
                    (SIGN(motor_now->m_motor_state.vq) * motor_now->m_motor_state.iq) < conf_now->lo_current_min) {
                // Truncating the duty cycle here would be dangerous, so run a PID controller.

                // Compensation for supply voltage variations
                float scale = 1.0 / motor_now->m_motor_state.v_bus;

                // Compute error
                float error = duty_set - motor_now->m_motor_state.duty_now;

                // Compute parameters
                float p_term = error * conf_now->foc_duty_dowmramp_kp * scale;
                motor_now->m_duty_i_term += error * (conf_now->foc_duty_dowmramp_ki * dt) * scale;

                // I-term wind-up protection
                utils_truncate_number((float*)&motor_now->m_duty_i_term, -1.0, 1.0);

                // Calculate output
                float output = p_term + motor_now->m_duty_i_term;
                utils_truncate_number(&output, -1.0, 1.0);
                iq_set_tmp = output * conf_now->lo_current_max;
            } else {
                // If the duty cycle is less than or equal to the set duty cycle just limit
                // the modulation and use the maximum allowed current.
                motor_now->m_duty_i_term = motor_now->m_motor_state.iq / conf_now->lo_current_max;
                motor_now->m_motor_state.max_duty = duty_set;
                if (duty_set > 0.0) {
                    iq_set_tmp = conf_now->lo_current_max;
                } else {
                    iq_set_tmp = -conf_now->lo_current_max;
                }
            }
        } else if (motor_now->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
            // Braking
            iq_set_tmp = -SIGN(speed_fast_now) * fabsf(iq_set_tmp);
        }

        // Set motor phase
        //{
        if (!motor_now->m_phase_override) {
            observer_update(motor_now->m_motor_state.v_alpha, motor_now->m_motor_state.v_beta,
                    motor_now->m_motor_state.i_alpha, motor_now->m_motor_state.i_beta, dt,
                    &motor_now->m_observer_x1, &motor_now->m_observer_x2, &motor_now->m_phase_now_observer, motor_now);

            // Compensate from the phase lag caused by the switching frequency. This is important for motors
            // that run on high ERPM compared to the switching frequency.
            motor_now->m_phase_now_observer += motor_now->m_pll_speed * dt * (0.5 + conf_now->foc_observer_offset);
            utils_norm_angle_rad((float*)&motor_now->m_phase_now_observer);
        }

        switch (conf_now->foc_sensor_mode) {
        case FOC_SENSOR_MODE_ENCODER:
            // if (encoder_index_found()) {
            // if (encoder_index_found() || virtual_motor_is_connected()) {
            //  motor_now->m_motor_state.phase = correct_encoder(
            //          motor_now->m_phase_now_observer,
            //          motor_now->m_phase_now_encoder,
            //          motor_now->m_speed_est_fast,
            //          conf_now->foc_sl_erpm,
            //          motor_now);
            // } else {
            // Rotate the motor in open loop if the index isn't found.
            motor_now->m_motor_state.phase = motor_now->m_phase_now_encoder_no_index;
            // }

            if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
                id_set_tmp = 0.0;
            }
            break;
        case FOC_SENSOR_MODE_HALL:
            motor_now->m_phase_now_observer = correct_hall(motor_now->m_phase_now_observer, dt, motor_now);
            motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;

            if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
                id_set_tmp = 0.0;
            }
            break;
        case FOC_SENSOR_MODE_SENSORLESS:
            if (motor_now->m_phase_observer_override) {
                motor_now->m_motor_state.phase = motor_now->m_phase_now_observer_override;
                motor_now->m_observer_x1 = motor_now->m_observer_x1_override;
                motor_now->m_observer_x2 = motor_now->m_observer_x2_override;
            } else {
                motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;
            }

            if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
                id_set_tmp = 0.0;
            }
            break;

        case FOC_SENSOR_MODE_HFI_START:
            motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;

            if (motor_now->m_phase_observer_override) {
                motor_now->m_hfi.est_done_cnt = 0;
                motor_now->m_hfi.flip_cnt = 0;

                motor_now->m_min_rpm_hyst_timer = 0.0;
                motor_now->m_min_rpm_timer = 0.0;
                motor_now->m_phase_observer_override = false;
            }

            if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
                id_set_tmp = 0.0;
            }
            break;

        case FOC_SENSOR_MODE_HFI:
            // if (fabsf(RADPS2RPM_f(motor_now->m_speed_est_fast)) > conf_now->foc_sl_erpm_hfi) {
            //  motor_now->m_hfi.observer_zero_time = 0;
            // } else {
            //  motor_now->m_hfi.observer_zero_time += dt;
            // }

            // if (motor_now->m_hfi.observer_zero_time < conf_now->foc_hfi_obs_ovr_sec) {
            //  motor_now->m_hfi.angle = motor_now->m_phase_now_observer;
            // }

            // motor_now->m_motor_state.phase = correct_encoder(
            //      motor_now->m_phase_now_observer,
            //      motor_now->m_hfi.angle,
            //      motor_now->m_speed_est_fast,
            //      conf_now->foc_sl_erpm_hfi,
            //      motor_now);

            // if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
            //  id_set_tmp = 0.0;
            // }
            break;
        }

        if (motor_now->m_control_mode == CONTROL_MODE_HANDBRAKE) {
            // Force the phase to 0 in handbrake mode so that the current simply locks the rotor.
            motor_now->m_motor_state.phase = 0.0;
        } else if (motor_now->m_control_mode == CONTROL_MODE_OPENLOOP ||
                motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY) {
            motor_now->m_openloop_angle += dt * motor_now->m_openloop_speed;
            utils_norm_angle_rad((float*)&motor_now->m_openloop_angle);
            motor_now->m_motor_state.phase = motor_now->m_openloop_angle;
        } else if (motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_PHASE ||
                motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY_PHASE) {
            motor_now->m_motor_state.phase = motor_now->m_openloop_phase;
        }

        if (motor_now->m_phase_override) {
            motor_now->m_motor_state.phase = motor_now->m_phase_now_override;
        }

        utils_fast_sincos_better(motor_now->m_motor_state.phase,
                (float*)&motor_now->m_motor_state.phase_sin,
                (float*)&motor_now->m_motor_state.phase_cos);
        //}

        // Apply MTPA. See: https://github.com/vedderb/bldc/pull/179
        const float ld_lq_diff = conf_now->foc_motor_ld_lq_diff;
        if (conf_now->foc_mtpa_mode != MTPA_MODE_OFF && ld_lq_diff != 0.0) {
            const float lambda = conf_now->foc_motor_flux_linkage;

            float iq_ref = iq_set_tmp;
            if (conf_now->foc_mtpa_mode == MTPA_MODE_IQ_MEASURED) {
                iq_ref = utils_min_abs(iq_set_tmp, motor_now->m_motor_state.iq_filter);
            }

            id_set_tmp = (lambda - sqrtf(SQ(lambda) + 8.0 * SQ(ld_lq_diff) * SQ(iq_ref))) / (4.0 * ld_lq_diff);
            iq_set_tmp = SIGN(iq_set_tmp) * sqrtf(SQ(iq_set_tmp) - SQ(id_set_tmp));
        }

        const float mod_q = motor_now->m_motor_state.mod_q_filter;

        // Running FW from the 1 khz timer seems fast enough.
        //      run_fw(motor_now, dt);
        id_set_tmp -= motor_now->m_i_fw_set;
        iq_set_tmp -= SIGN(mod_q) * motor_now->m_i_fw_set * conf_now->foc_fw_q_current_factor;

        // Apply current limits
        // TODO: Consider D axis current for the input current as well.
        if (mod_q > 0.001) {
            utils_truncate_number(&iq_set_tmp, conf_now->lo_in_current_min / mod_q, conf_now->lo_in_current_max / mod_q);
        } else if (mod_q < -0.001) {
            utils_truncate_number(&iq_set_tmp, conf_now->lo_in_current_max / mod_q, conf_now->lo_in_current_min / mod_q);
        }

        if (mod_q > 0.0) {
            utils_truncate_number(&iq_set_tmp, conf_now->lo_current_min, conf_now->lo_current_max);
        } else {
            utils_truncate_number(&iq_set_tmp, -conf_now->lo_current_max, -conf_now->lo_current_min);
        }

        float current_max_abs = fabsf(utils_max_abs(conf_now->lo_current_max, conf_now->lo_current_min));
        utils_truncate_number_abs(&id_set_tmp, current_max_abs);
        utils_truncate_number_abs(&iq_set_tmp, sqrtf(SQ(current_max_abs) - SQ(id_set_tmp)));

        motor_now->m_motor_state.id_target = id_set_tmp;
        motor_now->m_motor_state.iq_target = iq_set_tmp;

        control_current(motor_now, dt);
    }
    else {
        // Motor is not running

        // The current is 0 when the motor is undriven
        motor_now->m_motor_state.i_alpha = 0.0;
        motor_now->m_motor_state.i_beta = 0.0;
        motor_now->m_motor_state.id = 0.0;
        motor_now->m_motor_state.iq = 0.0;
        motor_now->m_motor_state.id_filter = 0.0;
        motor_now->m_motor_state.iq_filter = 0.0;
#ifdef HW_HAS_INPUT_CURRENT_SENSOR
        GET_INPUT_CURRENT_OFFSET(); // TODO: should this be done here?
#endif
        motor_now->m_motor_state.i_bus = 0.0;
        motor_now->m_motor_state.i_abs = 0.0;
        motor_now->m_motor_state.i_abs_filter = 0.0;

        // Track back emf
        update_valpha_vbeta(motor_now, 0.0, 0.0);

        // Run observer
        observer_update(motor_now->m_motor_state.v_alpha, motor_now->m_motor_state.v_beta,
                motor_now->m_motor_state.i_alpha, motor_now->m_motor_state.i_beta, dt,
                &motor_now->m_observer_x1, &motor_now->m_observer_x2, 0, motor_now);

        motor_now->m_phase_now_observer = utils_fast_atan2(motor_now->m_x2_prev + motor_now->m_observer_x2,
                motor_now->m_x1_prev + motor_now->m_observer_x1);

        // The observer phase offset has to be added here as well, with 0.5 switching cycles offset
        // compared to when running. Otherwise going from undriven to driven causes a current
        // spike.
        motor_now->m_phase_now_observer += motor_now->m_pll_speed * dt * conf_now->foc_observer_offset;
        utils_norm_angle_rad((float*)&motor_now->m_phase_now_observer);

        motor_now->m_x1_prev = motor_now->m_observer_x1;
        motor_now->m_x2_prev = motor_now->m_observer_x2;

        // Set motor phase
        //{
        switch (conf_now->foc_sensor_mode) {
        case FOC_SENSOR_MODE_ENCODER:
            // motor_now->m_motor_state.phase = correct_encoder(
            //      motor_now->m_phase_now_observer,
            //      motor_now->m_phase_now_encoder,
            //      motor_now->m_speed_est_fast,
            //      conf_now->foc_sl_erpm,
            //      motor_now);
            break;
        case FOC_SENSOR_MODE_HALL:
            motor_now->m_phase_now_observer = correct_hall(motor_now->m_phase_now_observer, dt, motor_now);
            motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;
            break;
        case FOC_SENSOR_MODE_SENSORLESS:
            motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;
            break;
        case FOC_SENSOR_MODE_HFI:
        case FOC_SENSOR_MODE_HFI_START:{
            motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;
            if (fabsf(RADPS2RPM_f(motor_now->m_pll_speed)) < (conf_now->foc_sl_erpm_hfi * 1.1)) {
                motor_now->m_hfi.est_done_cnt = 0;
                motor_now->m_hfi.flip_cnt = 0;
            }
        } break;

        }

        utils_fast_sincos_better(motor_now->m_motor_state.phase,
                (float*)&motor_now->m_motor_state.phase_sin,
                (float*)&motor_now->m_motor_state.phase_cos);
        //}

        // HFI Restore
        // CURRENT_FILTER_ON();
        motor_now->m_hfi.ind = 0;
        motor_now->m_hfi.ready = false;
        motor_now->m_hfi.is_samp_n = false;
        motor_now->m_hfi.prev_sample = 0.0;
        motor_now->m_hfi.angle = motor_now->m_motor_state.phase;

        float s = motor_now->m_motor_state.phase_sin;
        float c = motor_now->m_motor_state.phase_cos;

        // Park transform
        float vd_tmp = c * motor_now->m_motor_state.v_alpha + s * motor_now->m_motor_state.v_beta;
        float vq_tmp = c * motor_now->m_motor_state.v_beta  - s * motor_now->m_motor_state.v_alpha;

        UTILS_NAN_ZERO(motor_now->m_motor_state.vd);
        UTILS_NAN_ZERO(motor_now->m_motor_state.vq);

        UTILS_LP_FAST(motor_now->m_motor_state.vd, vd_tmp, 0.2);
        UTILS_LP_FAST(motor_now->m_motor_state.vq, vq_tmp, 0.2);

        // Set the current controller integrator to the BEMF voltage to avoid
        // a current spike when the motor is driven again. Notice that we have
        // to take decoupling into account.
        motor_now->m_motor_state.vd_int = motor_now->m_motor_state.vd;
        motor_now->m_motor_state.vq_int = motor_now->m_motor_state.vq;

        if (conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_BEMF ||
                conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS_BEMF) {
            motor_now->m_motor_state.vq_int -= motor_now->m_motor_state.speed_rad_s * conf_now->foc_motor_flux_linkage;
        }

        // Update corresponding modulation
        /* voltage_normalize = 1/(2/3*V_bus) */
        const float voltage_normalize = 1.5 / motor_now->m_motor_state.v_bus;

        motor_now->m_motor_state.mod_d = motor_now->m_motor_state.vd * voltage_normalize;
        motor_now->m_motor_state.mod_q = motor_now->m_motor_state.vq * voltage_normalize;
        UTILS_NAN_ZERO(motor_now->m_motor_state.mod_q_filter);
        UTILS_LP_FAST(motor_now->m_motor_state.mod_q_filter, motor_now->m_motor_state.mod_q, 0.2);
        utils_truncate_number_abs((float*)&motor_now->m_motor_state.mod_q_filter, 1.0);
    }

    // Calculate duty cycle
    motor_now->m_motor_state.duty_now = SIGN(motor_now->m_motor_state.vq) *
            sqrtf(SQ(motor_now->m_motor_state.mod_d) + SQ(motor_now->m_motor_state.mod_q)) / SQRT3_BY_2;

    // Run PLL for speed estimation
    pll_run(motor_now->m_motor_state.phase, dt, &motor_now->m_pll_phase, &motor_now->m_pll_speed, conf_now);
    motor_now->m_motor_state.speed_rad_s = motor_now->m_pll_speed;

    // Low latency speed estimation, for e.g. HFI.
    {
        float diff = utils_angle_difference_rad(motor_now->m_motor_state.phase, motor_now->m_phase_before_speed_est);
        utils_truncate_number(&diff, -M_PI / 3.0, M_PI / 3.0);

        UTILS_LP_FAST(motor_now->m_speed_est_fast, diff / dt, 0.01);
        UTILS_NAN_ZERO(motor_now->m_speed_est_fast);

        UTILS_LP_FAST(motor_now->m_speed_est_faster, diff / dt, 0.2);
        UTILS_NAN_ZERO(motor_now->m_speed_est_faster);

        // pll wind-up protection
        utils_truncate_number_abs((float*)&motor_now->m_pll_speed, fabsf(motor_now->m_speed_est_fast) * 3.0);

        motor_now->m_phase_before_speed_est = motor_now->m_motor_state.phase;
    }

    // Update tachometer (resolution = 60 deg as for BLDC)
    float ph_tmp = motor_now->m_motor_state.phase;
    utils_norm_angle_rad(&ph_tmp);
    int step = (int)floorf((ph_tmp + M_PI) / (2.0 * M_PI) * 6.0);
    utils_truncate_number_int(&step, 0, 5);
    int diff = step - motor_now->m_tacho_step_last;
    motor_now->m_tacho_step_last = step;

    if (diff > 3) {
        diff -= 6;
    } else if (diff < -2) {
        diff += 6;
    }

    motor_now->m_tachometer += diff;
    motor_now->m_tachometer_abs += abs(diff);

    // Track position control angle
    float angle_now = 0.0;
    // if (encoder_is_configured()) {
    //  if (conf_now->m_sensor_port_mode == SENSOR_PORT_MODE_TS5700N8501_MULTITURN) {
    //      angle_now = encoder_read_deg_multiturn();
    //  } else {
    //      angle_now = enc_ang;
    //  }
    // } else {
    angle_now = RAD2DEG_f(motor_now->m_motor_state.phase);
    // }

    utils_norm_angle(&angle_now);

    if (conf_now->p_pid_ang_div > 0.98 && conf_now->p_pid_ang_div < 1.02) {
        motor_now->m_pos_pid_now = angle_now;
    } else {
        if (angle_now < 90.0 && motor_now->m_pid_div_angle_last > 270.0) {
            motor_now->m_pid_div_angle_accumulator += 360.0 / conf_now->p_pid_ang_div;
            utils_norm_angle((float*)&motor_now->m_pid_div_angle_accumulator);
        } else if (angle_now > 270.0 && motor_now->m_pid_div_angle_last < 90.0) {
            motor_now->m_pid_div_angle_accumulator -= 360.0 / conf_now->p_pid_ang_div;
            utils_norm_angle((float*)&motor_now->m_pid_div_angle_accumulator);
        }

        motor_now->m_pid_div_angle_last = angle_now;

        motor_now->m_pos_pid_now = motor_now->m_pid_div_angle_accumulator + angle_now / conf_now->p_pid_ang_div;
        utils_norm_angle((float*)&motor_now->m_pos_pid_now);
    }

#ifdef AD2S1205_SAMPLE_GPIO
    // Release sample in the AD2S1205 resolver IC.
    palSetPad(AD2S1205_SAMPLE_GPIO, AD2S1205_SAMPLE_PIN);
#endif

#ifdef HW_HAS_DUAL_MOTORS
    mc_interface_mc_timer_isr(is_second_motor);
#else
    mc_interface_mc_timer_isr(false);
#endif

    m_isr_motor = 0;
    // m_last_adc_isr_duration = timer_seconds_elapsed_since(t_start);
}


static void run_fw(volatile motor_all_state_t *motor, float dt) {
    if (motor->m_conf->foc_fw_current_max < motor->m_conf->cc_min_current) {
        return;
    }

    // Field Weakening
    // FW is used in the current and speed control modes. If a different mode is used
    // this code also runs if field weakening was active before. This allows
    // changing control mode even while in field weakening.
    if (motor->m_state == MC_STATE_RUNNING &&
            (motor->m_control_mode == CONTROL_MODE_CURRENT ||
                    motor->m_control_mode == CONTROL_MODE_CURRENT_BRAKE ||
                    motor->m_control_mode == CONTROL_MODE_SPEED ||
                    motor->m_i_fw_set > motor->m_conf->cc_min_current)) {
        float fw_current_now = 0.0;
        float duty_abs = motor->m_duty_abs_filtered;

        if (motor->m_conf->foc_fw_duty_start < 0.99 &&
                duty_abs > motor->m_conf->foc_fw_duty_start * motor->m_conf->l_max_duty) {
            fw_current_now = utils_map(duty_abs,
                    motor->m_conf->foc_fw_duty_start * motor->m_conf->l_max_duty,
                    motor->m_conf->l_max_duty,
                    0.0, motor->m_conf->foc_fw_current_max);

            // m_current_off_delay is used to not stop the modulation too soon after leaving FW. If axis decoupling
            // is not working properly an oscillation can occur on the modulation when changing the current
            // fast, which can make the estimated duty cycle drop below the FW threshold long enough to stop
            // modulation. When that happens the body diodes in the MOSFETs can see a lot of current and unexpected
            // braking happens. Therefore the modulation is left on for some time after leaving FW to give the
            // oscillation a chance to decay while the MOSFETs are still driven.
            motor->m_current_off_delay = 1.0;
        }

        if (motor->m_conf->foc_fw_ramp_time < dt) {
            motor->m_i_fw_set = fw_current_now;
        } else {
            utils_step_towards((float*)&motor->m_i_fw_set, fw_current_now,
                    (dt / motor->m_conf->foc_fw_ramp_time) * motor->m_conf->foc_fw_current_max);
        }
    }
}
static void timer_update(volatile motor_all_state_t *motor, float dt) {
    run_fw(motor, dt);

    // Check if it is time to stop the modulation. Notice that modulation is kept on as long as there is
    // field weakening current.
    // utils_sys_lock_cnt();
    utils_step_towards((float*)&motor->m_current_off_delay, 0.0, dt);
    if (!motor->m_phase_override && motor->m_state == MC_STATE_RUNNING &&
            (motor->m_control_mode == CONTROL_MODE_CURRENT ||
                    motor->m_control_mode == CONTROL_MODE_CURRENT_BRAKE ||
                    motor->m_control_mode == CONTROL_MODE_HANDBRAKE ||
                    motor->m_control_mode == CONTROL_MODE_OPENLOOP ||
                    motor->m_control_mode == CONTROL_MODE_OPENLOOP_PHASE)) {
        // For various control modes, check whether the controller needs to be disabled. Disable when current request is below threshold and field weakening has stopped.
        if (fabsf(motor->m_iq_set) < motor->m_conf->cc_min_current &&
                fabsf(motor->m_id_set) < motor->m_conf->cc_min_current &&
                motor->m_i_fw_set < motor->m_conf->cc_min_current &&
                motor->m_current_off_delay < dt) {
            motor->m_control_mode = CONTROL_MODE_NONE;
            motor->m_state = MC_STATE_OFF;
            stop_pwm_hw(motor);
        }
    }
    // utils_sys_unlock_cnt();

    // Use this to study the openloop timers under experiment plot
#if 0
    {
        static bool plot_started = false;
        static int plot_div = 0;
        static float plot_int = 0.0;
        static int get_fw_version_cnt = 0;

        if (commands_get_fw_version_sent_cnt() != get_fw_version_cnt) {
            get_fw_version_cnt = commands_get_fw_version_sent_cnt();
            plot_started = false;
        }

        plot_div++;
        if (plot_div >= 10) {
            plot_div = 0;
            if (!plot_started) {
                plot_started = true;
                commands_init_plot("Time", "Val");
                commands_plot_add_graph("m_min_rpm_timer");
                commands_plot_add_graph("m_min_rpm_hyst_timer");
            }

            commands_plot_set_graph(0);
            commands_send_plot_points(plot_int, motor->m_min_rpm_timer);
            commands_plot_set_graph(1);
            commands_send_plot_points(plot_int, motor->m_min_rpm_hyst_timer);
            plot_int++;
        }
    }
#endif

    // Use this to study the observer state in a XY-plot
#if 0
    {
        static bool plot_started = false;
        static int plot_div = 0;
        static int get_fw_version_cnt = 0;

        if (commands_get_fw_version_sent_cnt() != get_fw_version_cnt) {
            get_fw_version_cnt = commands_get_fw_version_sent_cnt();
            plot_started = false;
        }

        plot_div++;
        if (plot_div >= 10) {
            plot_div = 0;
            if (!plot_started) {
                plot_started = true;
                commands_init_plot("X1", "X2");
                commands_plot_add_graph("Observer");
                commands_plot_add_graph("Observer Mag");
            }

            commands_plot_set_graph(0);
            commands_send_plot_points(m_motor_1.m_observer_x1, m_motor_1.m_observer_x2);
            float mag = sqrtf(SQ(m_motor_1.m_observer_x1) + SQ(m_motor_1.m_observer_x2));
            commands_plot_set_graph(1);
            commands_send_plot_points(0.0, mag);
        }
    }
#endif

    float t_lock = motor->m_conf->foc_sl_openloop_time_lock;
    float t_ramp = motor->m_conf->foc_sl_openloop_time_ramp;
    float t_const = motor->m_conf->foc_sl_openloop_time;

    float openloop_rpm_max = utils_map(fabsf(motor->m_motor_state.iq_filter),
            0.0, motor->m_conf->l_current_max,
            motor->m_conf->foc_openloop_rpm_low * motor->m_conf->foc_openloop_rpm,
            motor->m_conf->foc_openloop_rpm);

    utils_truncate_number_abs(&openloop_rpm_max, motor->m_conf->foc_openloop_rpm);

    float openloop_rpm = openloop_rpm_max;
    if (motor->m_conf->foc_sensor_mode != FOC_SENSOR_MODE_ENCODER) {
        float time_fwd = t_lock + t_ramp + t_const - motor->m_min_rpm_timer;
        if (time_fwd < t_lock) {
            openloop_rpm = 0.0;
        } else if (time_fwd < (t_lock + t_ramp)) {
            openloop_rpm = utils_map(time_fwd, t_lock,
                    t_lock + t_ramp, 0.0, openloop_rpm);
        }
    }

    utils_truncate_number_abs(&openloop_rpm, openloop_rpm_max);

    float add_min_speed = 0.0;
    if (motor->m_motor_state.duty_now > 0.0) {
        add_min_speed = RPM2RADPS_f(openloop_rpm) * dt;
    } else {
        add_min_speed = -RPM2RADPS_f(openloop_rpm) * dt;
    }

    // Open loop encoder angle for when the index is not found
    motor->m_phase_now_encoder_no_index += add_min_speed;
    utils_norm_angle_rad((float*)&motor->m_phase_now_encoder_no_index);

    if (fabsf(motor->m_pll_speed) < RPM2RADPS_f(openloop_rpm_max) &&
            motor->m_min_rpm_hyst_timer < motor->m_conf->foc_sl_openloop_hyst) {
        motor->m_min_rpm_hyst_timer += dt;
    } else if (motor->m_min_rpm_hyst_timer > 0.0) {
        motor->m_min_rpm_hyst_timer -= dt;
    }

    // Don't use this in brake mode.
    if (motor->m_control_mode == CONTROL_MODE_CURRENT_BRAKE ||
            (motor->m_state == MC_STATE_RUNNING && fabsf(motor->m_motor_state.duty_now) < 0.001)) {
        motor->m_min_rpm_hyst_timer = 0.0;
        motor->m_min_rpm_timer = 0.0;
        motor->m_phase_observer_override = false;
    }

    bool started_now = false;
    if (motor->m_min_rpm_hyst_timer >= motor->m_conf->foc_sl_openloop_hyst &&
            motor->m_min_rpm_timer <= 0.0001) {
        motor->m_min_rpm_timer = t_lock + t_ramp + t_const;
        started_now = true;
    }

    if (motor->m_state != MC_STATE_RUNNING) {
        motor->m_min_rpm_timer = 0.0;
    }

    if (motor->m_min_rpm_timer > 0.0) {
        motor->m_phase_now_observer_override += add_min_speed;

        // When the motor gets stuck it tends to be 90 degrees off, so start the open loop
        // sequence by correcting with 60 degrees.
        if (started_now) {
            if (motor->m_motor_state.duty_now > 0.0) {
                motor->m_phase_now_observer_override += M_PI / 3.0;
            } else {
                motor->m_phase_now_observer_override -= M_PI / 3.0;
            }
        }

        utils_norm_angle_rad((float*)&motor->m_phase_now_observer_override);
        motor->m_phase_observer_override = true;
        motor->m_min_rpm_timer -= dt;
        motor->m_min_rpm_hyst_timer = 0.0;

        // Set observer state to help it start tracking when leaving open loop.
        float s, c;
        utils_fast_sincos_better(motor->m_phase_now_observer_override + SIGN(motor->m_motor_state.duty_now) * M_PI / 4.0, &s, &c);
        motor->m_observer_x1_override = c * motor->m_conf->foc_motor_flux_linkage;
        motor->m_observer_x2_override = s * motor->m_conf->foc_motor_flux_linkage;
    } else {
        motor->m_phase_now_observer_override = motor->m_phase_now_observer;
        motor->m_phase_observer_override = false;
    }

    // Samples
    if (motor->m_state == MC_STATE_RUNNING) {
        const volatile float vd_tmp = motor->m_motor_state.vd;
        const volatile float vq_tmp = motor->m_motor_state.vq;
        const volatile float id_tmp = motor->m_motor_state.id;
        const volatile float iq_tmp = motor->m_motor_state.iq;
        motor->m_samples.avg_current_tot += sqrtf(SQ(id_tmp) + SQ(iq_tmp));
        motor->m_samples.avg_voltage_tot += (float)sqrtf(SQ(vd_tmp) + SQ(vq_tmp));
        motor->m_samples.sample_num++;
    }

    // Observer gain scaling, based on bus voltage and duty cycle
    float gamma_tmp = utils_map(fabsf(motor->m_motor_state.duty_now),
            0.0, 40.0 / motor->m_motor_state.v_bus,
            0, motor->m_conf->foc_observer_gain);
    if (gamma_tmp < (motor->m_conf->foc_observer_gain_slow * motor->m_conf->foc_observer_gain)) {
        gamma_tmp = motor->m_conf->foc_observer_gain_slow * motor->m_conf->foc_observer_gain;
    }

    // 4.0 scaling is kind of arbitrary, but it should make configs from old VESC Tools more likely to work.
    motor->m_gamma_now = gamma_tmp * 4.0;

    // Run resistance observer
    // See "An adaptive flux observer for the permanent magnet synchronous motor"
    // https://doi.org/10.1002/acs.2587
    {
        float res_est_gain = 0.00002;
        float i_abs_sq = SQ(motor->m_motor_state.i_abs);
        motor->m_r_est = motor->m_r_est_state - 0.5 * res_est_gain * motor->m_conf->foc_motor_l * i_abs_sq;
        float res_dot = -res_est_gain * (motor->m_r_est * i_abs_sq + motor->m_speed_est_fast *
                (motor->m_motor_state.i_beta * motor->m_observer_x1 - motor->m_motor_state.i_alpha * motor->m_observer_x2) -
                (motor->m_motor_state.i_alpha * motor->m_motor_state.v_alpha + motor->m_motor_state.i_beta * motor->m_motor_state.v_beta));
        motor->m_r_est_state += res_dot * dt;

        utils_truncate_number((float*)&motor->m_r_est_state, motor->m_conf->foc_motor_r * 0.25, motor->m_conf->foc_motor_r * 3.0);
    }
}

// See http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
void observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta,
        float dt, volatile float *x1, volatile float *x2, volatile float *phase, volatile motor_all_state_t *motor) {

    volatile mc_configuration *conf_now = motor->m_conf;

    float R = conf_now->foc_motor_r;
    float L = conf_now->foc_motor_l;
    float lambda = conf_now->foc_motor_flux_linkage;

    // Saturation compensation
    const float comp_fact = conf_now->foc_sat_comp * (motor->m_motor_state.i_abs_filter / conf_now->l_current_max);
    L -= L * comp_fact;
    lambda -= lambda * comp_fact;

    // Temperature compensation
    const float t = mc_interface_temp_motor_filtered();
    if (conf_now->foc_temp_comp && t > -30.0) {
        R += R * 0.00386 * (t - conf_now->foc_temp_comp_base_temp);
    }

    float ld_lq_diff = conf_now->foc_motor_ld_lq_diff;
    float id = motor->m_motor_state.id;
    float iq = motor->m_motor_state.iq;

    // Adjust inductance for saliency.
    if (fabsf(id) > 0.1 || fabsf(iq) > 0.1) {
        L = L - ld_lq_diff / 2.0 + ld_lq_diff * SQ(iq) / (SQ(id) + SQ(iq));
    }

    const float L_ia = L * i_alpha;
    const float L_ib = L * i_beta;
    const float R_ia = R * i_alpha;
    const float R_ib = R * i_beta;
    const float lambda_2 = SQ(lambda);
    const float gamma_half = motor->m_gamma_now * 0.5;

    switch (conf_now->foc_observer_type) {
    case FOC_OBSERVER_ORTEGA_ORIGINAL: {
        float err = lambda_2 - (SQ(*x1 - L_ia) + SQ(*x2 - L_ib));

        // Forcing this term to stay negative helps convergence according to
        //
        // http://cas.ensmp.fr/Publications/Publications/Papers/ObserverPermanentMagnet.pdf
        // and
        // https://arxiv.org/pdf/1905.00833.pdf
        if (err > 0.0) {
            err = 0.0;
        }

        float x1_dot = v_alpha - R_ia + gamma_half * (*x1 - L_ia) * err;
        float x2_dot = v_beta - R_ib + gamma_half * (*x2 - L_ib) * err;

        *x1 += x1_dot * dt;
        *x2 += x2_dot * dt;
    } break;

    default:
        break;
    }

    UTILS_NAN_ZERO(*x1);
    UTILS_NAN_ZERO(*x2);

    // Prevent the magnitude from getting too low, as that makes the angle very unstable.
    float mag = sqrtf(SQ(*x1) + SQ(*x2));
    if (mag < (conf_now->foc_motor_flux_linkage * 0.5)) {
        *x1 *= 1.1;
        *x2 *= 1.1;
    }

    if (phase) {
        *phase = utils_fast_atan2(*x2 - L_ib, *x1 - L_ia);
    }
}

static void timer_reinit(int f_zv){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_DeInit(TIM1);
    TIM_DeInit(TIM2);
    TIM1->CNT = 0;
    TIM2->CNT = 0;

    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure={0};

    TIM_TimeBaseInitStructure.TIM_Period = SYSTEM_CORE_CLOCK / f_zv;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = TIM1->ATRLR / 2;;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
    TIM_OC1Init( TIM1, &TIM_OCInitStructure );
    TIM_OC2Init( TIM1, &TIM_OCInitStructure );
    TIM_OC3Init( TIM1, &TIM_OCInitStructure );
    TIM_OC1PreloadConfig( TIM1, TIM_OCPreload_Enable );
    TIM_OC2PreloadConfig( TIM1, TIM_OCPreload_Enable );
    TIM_OC3PreloadConfig( TIM1, TIM_OCPreload_Enable );

    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime = 0x03;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
    TIM_CCPreloadControl(TIM1, ENABLE);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 250;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure );
    TIM_OC2Init(TIM2, &TIM_OCInitStructure );
    TIM_CCPreloadControl(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
    TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);

    //? TIM_SlaveMode_External1 vs TIM_SlaveMode_Trigger
    //TIM_ITRxExternalClockConfig(TIM2, TIM_TS_ITR0);
    //TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_External1);

    TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);


    TIM_Cmd( TIM1, ENABLE );
    TIM_Cmd( TIM2, ENABLE );
    TIM_CtrlPWMOutputs(TIM1, ENABLE );
    TIM_CtrlPWMOutputs(TIM2, ENABLE );

    TIMER_UPDATE_SAMP(MCPWM_FOC_CURRENT_SAMP_OFFSET);

    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static volatile motor_all_state_t *motor_now(void) {
    // #ifdef HW_HAS_DUAL_MOTORS
    //  return mc_interface_motor_now() == 1 ? &m_motor_1 : &m_motor_2;
    // #else
    return &m_motor_1;
    // #endif
}
static void stop_pwm_hw(volatile motor_all_state_t *motor) {
    motor->m_id_set = 0.0;
    motor->m_iq_set = 0.0;

    if (motor == &m_motor_1) {
        TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
        TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

        TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
        TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

        TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
        TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

        TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

        // #ifdef HW_HAS_DUAL_PARALLEL
        //      TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_ForcedAction_InActive);
        //      TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
        //      TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Disable);

        //      TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_ForcedAction_InActive);
        //      TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
        //      TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Disable);

        //      TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_ForcedAction_InActive);
        //      TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);
        //      TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Disable);

        //      TIM_GenerateEvent(TIM8, TIM_EventSource_COM);
        // #endif

#ifdef HW_HAS_DRV8313
        DISABLE_BR();
#endif

        motor->m_output_on = false;
        // PHASE_FILTER_OFF();
    }

    //  else {
    //      TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_ForcedAction_InActive);
    //      TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
    //      TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Disable);

    //      TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_ForcedAction_InActive);
    //      TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
    //      TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Disable);

    //      TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_ForcedAction_InActive);
    //      TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);
    //      TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Disable);

    //      TIM_GenerateEvent(TIM8, TIM_EventSource_COM);

    // #ifdef HW_HAS_DRV8313_2
    //      DISABLE_BR_2();
    // #endif

    //      motor->m_output_on = false;
    //      PHASE_FILTER_OFF_M2();
    //  }
}

static void start_pwm_hw(volatile motor_all_state_t *motor) {
    if (motor == &m_motor_1) {
        TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
        TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

        TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
        TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

        TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
        TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

        // #ifdef HW_HAS_DUAL_PARALLEL
        //      TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_OCMode_PWM1);
        //      TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
        //      TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Enable);

        //      TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_OCMode_PWM1);
        //      TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
        //      TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Enable);

        //      TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_OCMode_PWM1);
        //      TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);
        //      TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Enable);

        //      PHASE_FILTER_ON_M2();
        // #endif

        // Generate COM event in ADC interrupt to get better synchronization
        //  TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

#ifdef HW_HAS_DRV8313
        ENABLE_BR();
#endif
        motor->m_output_on = true;
        // PHASE_FILTER_ON();
    }
    //  else {
    //      TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_OCMode_PWM1);
    //      TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
    //      TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Enable);

    //      TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_OCMode_PWM1);
    //      TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
    //      TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Enable);

    //      TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_OCMode_PWM1);
    //      TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);
    //      TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Enable);

    // #ifdef HW_HAS_DRV8313_2
    //      ENABLE_BR_2();
    // #endif
    //      motor->m_output_on = true;
    //      PHASE_FILTER_ON_M2();
    //  }
}
static void timer_thread(void *pvParameters)
{

    while(1)
    {
        const float dt = 0.001;

        if (timer_thd_stop) {
            timer_thd_stop = false;
            return;
        }

        timer_update(&m_motor_1, dt);
#ifdef HW_HAS_DUAL_MOTORS
        timer_update(&m_motor_2, dt);
#endif

        input_current_offset_measurement();

        //        vTaskDelay(1);
        timer_sleep_ms(1);

    }
}

// TODO: This won't work for dual motors
static void input_current_offset_measurement(void) {
#ifdef HW_HAS_INPUT_CURRENT_SENSOR
    static uint16_t delay_current_offset_measurement = 0;

    if (delay_current_offset_measurement < 1000) {
        delay_current_offset_measurement++;
    } else {
        if (delay_current_offset_measurement == 1000) {
            delay_current_offset_measurement++;
            MEASURE_INPUT_CURRENT_OFFSET();
        }
    }
#endif
}

static void pll_run(float phase, float dt, volatile float *phase_var,
        volatile float *speed_var, volatile mc_configuration *conf) {
    UTILS_NAN_ZERO(*phase_var);
    float delta_theta = phase - *phase_var;
    utils_norm_angle_rad(&delta_theta);
    UTILS_NAN_ZERO(*speed_var);
    *phase_var += (*speed_var + conf->foc_pll_kp * delta_theta) * dt;
    utils_norm_angle_rad((float*)phase_var);
    *speed_var += conf->foc_pll_ki * delta_theta * dt;
}

/**
 * Run the current control loop.
 *
 * @param state_m
 * The motor state.
 *
 * Parameters that shall be set before calling this function:
 * id_target
 * iq_target
 * max_duty
 * phase
 * i_alpha
 * i_beta
 * v_bus
 * speed_rad_s
 *
 * Parameters that will be updated in this function:
 * i_bus
 * i_abs
 * i_abs_filter
 * v_alpha
 * v_beta
 * mod_d
 * mod_q
 * id
 * iq
 * id_filter
 * iq_filter
 * vd
 * vq
 * vd_int
 * vq_int
 * svm_sector
 *
 * @param dt
 * The time step in seconds.
 */
static void control_current(volatile motor_all_state_t *motor, float dt) {
    volatile motor_state_t *state_m = &motor->m_motor_state;
    volatile mc_configuration *conf_now = motor->m_conf;

    float s = state_m->phase_sin;
    float c = state_m->phase_cos;

    // float abs_rpm = fabsf(RADPS2RPM_f(motor->m_speed_est_fast));

    // bool do_hfi = (conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI ||
    //      (conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_START &&
    //              motor->m_control_mode != CONTROL_MODE_CURRENT_BRAKE &&
    //              fabsf(state_m->iq_target) > conf_now->cc_min_current)) &&
    //      !motor->m_phase_override &&
    //      abs_rpm < (conf_now->foc_sl_erpm_hfi * (motor->m_cc_was_hfi ? 1.8 : 1.5));

    // Only allow Q axis current after the HFI ambiguity is resolved. This causes
    // a short delay when starting.
    // if (do_hfi && motor->m_hfi.est_done_cnt < conf_now->foc_hfi_start_samples) {
    //  state_m->iq_target = 0;
    // } else if (conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_START) {
    //  do_hfi = false;
    // }

    // motor->m_cc_was_hfi = do_hfi;

    float max_duty = fabsf(state_m->max_duty);
    utils_truncate_number(&max_duty, 0.0, conf_now->l_max_duty);

    // Park transform: transforms the currents from stator to the rotor reference frame
    state_m->id = c * state_m->i_alpha + s * state_m->i_beta;
    state_m->iq = c * state_m->i_beta  - s * state_m->i_alpha;

    // Low passed currents are used for less time critical parts, not for the feedback
    UTILS_LP_FAST(state_m->id_filter, state_m->id, conf_now->foc_current_filter_const);
    UTILS_LP_FAST(state_m->iq_filter, state_m->iq, conf_now->foc_current_filter_const);

    float d_gain_scale = 1.0;
    if (conf_now->foc_d_gain_scale_start < 0.99) {
        float max_mod_norm = fabsf(state_m->duty_now / max_duty);
        if (max_duty < 0.01) {
            max_mod_norm = 1.0;
        }
        if (max_mod_norm > conf_now->foc_d_gain_scale_start) {
            d_gain_scale = utils_map(max_mod_norm, conf_now->foc_d_gain_scale_start, 1.0,
                    1.0, conf_now->foc_d_gain_scale_max_mod);
            if (d_gain_scale < conf_now->foc_d_gain_scale_max_mod) {
                d_gain_scale = conf_now->foc_d_gain_scale_max_mod;
            }
        }
    }

    float Ierr_d = state_m->id_target - state_m->id;
    float Ierr_q = state_m->iq_target - state_m->iq;

    state_m->vd = state_m->vd_int + Ierr_d * conf_now->foc_current_kp * d_gain_scale; //Feedback (PI controller). No D action needed because the plant is a first order system (tf = 1/(Ls+R))
    state_m->vq = state_m->vq_int + Ierr_q * conf_now->foc_current_kp;

    // Temperature compensation
    const float t = mc_interface_temp_motor_filtered();
    float ki = conf_now->foc_current_ki;
    if (conf_now->foc_temp_comp && t > -30.0) {
        ki += ki * 0.00386 * (t - conf_now->foc_temp_comp_base_temp);
    }

    state_m->vd_int += Ierr_d * (ki * d_gain_scale * dt);
    state_m->vq_int += Ierr_q * (ki * dt);

    // Decoupling. Using feedforward this compensates for the fact that the equations of a PMSM
    // are not really decoupled (the d axis current has impact on q axis voltage and visa-versa):
    //      Resistance  Inductance   Cross terms   Back-EMF   (see www.mathworks.com/help/physmod/sps/ref/pmsm.html)
    // vd = Rs*id   +   Ld*did/dt �????  ωe*iq*Lq
    // vq = Rs*iq   +   Lq*diq/dt +  ωe*id*Ld     + ωe*ψm
    float dec_vd = 0.0;
    float dec_vq = 0.0;
    float dec_bemf = 0.0;

    if (motor->m_control_mode < CONTROL_MODE_HANDBRAKE && conf_now->foc_cc_decoupling != FOC_CC_DECOUPLING_DISABLED) {
        float lq = conf_now->foc_motor_l + conf_now->foc_motor_ld_lq_diff * 0.5;
        float ld = conf_now->foc_motor_l - conf_now->foc_motor_ld_lq_diff * 0.5;

        switch (conf_now->foc_cc_decoupling) {
        case FOC_CC_DECOUPLING_CROSS:
            dec_vd = state_m->iq_filter * motor->m_speed_est_fast * lq; // m_speed_est_fast is ωe in [rad/s]
            dec_vq = state_m->id_filter * motor->m_speed_est_fast * ld;
            break;

        case FOC_CC_DECOUPLING_BEMF:
            dec_bemf = motor->m_speed_est_fast * conf_now->foc_motor_flux_linkage;
            break;

        case FOC_CC_DECOUPLING_CROSS_BEMF:
            dec_vd = state_m->iq_filter * motor->m_speed_est_fast * lq;
            dec_vq = state_m->id_filter * motor->m_speed_est_fast * ld;
            dec_bemf = motor->m_speed_est_fast * conf_now->foc_motor_flux_linkage;
            break;

        default:
            break;
        }
    }

    state_m->vd -= dec_vd; //Negative sign as in the PMSM equations
    state_m->vq += dec_vq + dec_bemf;

    // Calculate the max length of the voltage space vector without overmodulation.
    // Is simply 1/sqrt(3) * v_bus. See https://microchipdeveloper.com/mct5001:start. Adds margin with max_duty.
    float max_v_mag = ONE_BY_SQRT3 * max_duty * state_m->v_bus;

    // Saturation and anti-windup. Notice that the d-axis has priority as it controls field
    // weakening and the efficiency.
    float vd_presat = state_m->vd;
    utils_truncate_number_abs((float*)&state_m->vd, max_v_mag);
    state_m->vd_int += (state_m->vd - vd_presat);

    float max_vq = sqrtf(SQ(max_v_mag) - SQ(state_m->vd));
    float vq_presat = state_m->vq;
    utils_truncate_number_abs((float*)&state_m->vq, max_vq);
    state_m->vq_int += (state_m->vq - vq_presat);

    utils_saturate_vector_2d((float*)&state_m->vd, (float*)&state_m->vq, max_v_mag);

    // mod_d and mod_q are normalized such that 1 corresponds to the max possible voltage:
    //    voltage_normalize = 1/(2/3*V_bus)
    // This includes overmodulation and therefore cannot be made in any direction.
    // Note that this scaling is different from max_v_mag, which is without over modulation.
    const float voltage_normalize = 1.5 / state_m->v_bus;
    state_m->mod_d = state_m->vd * voltage_normalize;
    state_m->mod_q = state_m->vq * voltage_normalize;
    UTILS_NAN_ZERO(state_m->mod_q_filter);
    UTILS_LP_FAST(state_m->mod_q_filter, state_m->mod_q, 0.2);

    // TODO: Have a look at this?
#ifdef HW_HAS_INPUT_CURRENT_SENSOR
    state_m->i_bus = GET_INPUT_CURRENT();
#else
    state_m->i_bus = state_m->mod_alpha_measured * state_m->i_alpha + state_m->mod_beta_measured * state_m->i_beta;
    // TODO: Also calculate motor power based on v_alpha, v_beta, i_alpha and i_beta. This is much more accurate
    // with phase filters than using the modulation and bus current.
#endif
    state_m->i_abs = sqrtf(SQ(state_m->id) + SQ(state_m->iq));
    state_m->i_abs_filter = sqrtf(SQ(state_m->id_filter) + SQ(state_m->iq_filter));

    // Inverse Park transform: transforms the (normalized) voltages from the rotor reference frame to the stator frame
    float mod_alpha = c * state_m->mod_d - s * state_m->mod_q;
    float mod_beta  = c * state_m->mod_q + s * state_m->mod_d;

    update_valpha_vbeta(motor, mod_alpha, mod_beta);

    // Dead time compensated values for vd and vq. Note that these are not used to control the switching times.
    state_m->vd = c * motor->m_motor_state.v_alpha + s * motor->m_motor_state.v_beta;
    state_m->vq = c * motor->m_motor_state.v_beta  - s * motor->m_motor_state.v_alpha;

    // HFI
    // if (do_hfi) {
    //  // CURRENT_FILTER_OFF();

    //  float mod_alpha_tmp = mod_alpha;
    //  float mod_beta_tmp = mod_beta;

    //  float hfi_voltage;
    //  if (motor->m_hfi.est_done_cnt < conf_now->foc_hfi_start_samples) {
    //      hfi_voltage = conf_now->foc_hfi_voltage_start;
    //  } else {
    //      hfi_voltage = utils_map(fabsf(state_m->iq), -0.01, conf_now->l_current_max,
    //                              conf_now->foc_hfi_voltage_run, conf_now->foc_hfi_voltage_max);
    //  }

    //  utils_truncate_number_abs(&hfi_voltage, state_m->v_bus * (1.0 - fabsf(state_m->duty_now)) * SQRT3_BY_2 * (2.0 / 3.0) * 0.95);

    //  if (motor->m_hfi.is_samp_n) {
    //      float sample_now = (utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_alpha -
    //              utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_beta);
    //      float di = (sample_now - motor->m_hfi.prev_sample);

    //      motor->m_hfi.buffer_current[motor->m_hfi.ind] = di;

    //      if (di > 0.01) {
    //          motor->m_hfi.buffer[motor->m_hfi.ind] = hfi_voltage / (conf_now->foc_f_zv * di);
    //      }

    //      motor->m_hfi.ind++;
    //      if (motor->m_hfi.ind == motor->m_hfi.samples) {
    //          motor->m_hfi.ind = 0;
    //          motor->m_hfi.ready = true;
    //      }

    //      mod_alpha_tmp += hfi_voltage * utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
    //      mod_beta_tmp  -= hfi_voltage * utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
    //  } else {
    //      motor->m_hfi.prev_sample = utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_alpha -
    //              utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_beta;

    //      mod_alpha_tmp -= hfi_voltage * utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
    //      mod_beta_tmp  += hfi_voltage * utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
    //  }

    //  utils_saturate_vector_2d(&mod_alpha_tmp, &mod_beta_tmp, SQRT3_BY_2 * 0.95);
    //  motor->m_hfi.is_samp_n = !motor->m_hfi.is_samp_n;

    //  if (conf_now->foc_sample_v0_v7) {
    //      mod_alpha = mod_alpha_tmp;
    //      mod_beta = mod_beta_tmp;
    //  } else {
    //      // Delay adding the HFI voltage when not sampling in both 0 vectors, as it will cancel
    //      // itself with the opposite pulse from the previous HFI sample. This makes more sense
    //      // when drawing the SVM waveform.
    //      svm(mod_alpha_tmp, mod_beta_tmp, TIM1->ARR,
    //          (uint32_t*)&motor->m_duty1_next,
    //          (uint32_t*)&motor->m_duty2_next,
    //          (uint32_t*)&motor->m_duty3_next,
    //          (uint32_t*)&state_m->svm_sector);
    //      motor->m_duty_next_set = true;
    //  }
    // } else {
    // CURRENT_FILTER_ON();
    motor->m_hfi.ind = 0;
    motor->m_hfi.ready = false;
    motor->m_hfi.is_samp_n = false;
    motor->m_hfi.prev_sample = 0.0;
    // }

    // Set output (HW Dependent)
    uint32_t duty1, duty2, duty3, top;
    top = TIM1->ATRLR;

    // Calculate the duty cycles for all the phases. This also injects a zero modulation signal to
    // be able to fully utilize the bus voltage. See https://microchipdeveloper.com/mct5001:start
    svm(mod_alpha, mod_beta, top, &duty1, &duty2, &duty3, (uint32_t*)&state_m->svm_sector);

    if (motor == &m_motor_1) {
        TIMER_UPDATE_DUTY_M1(duty1, duty2, duty3);
        // #ifdef HW_HAS_DUAL_PARALLEL
        //      TIMER_UPDATE_DUTY_M2(duty1, duty2, duty3);
        // #endif
    } else {
        // #ifndef HW_HAS_DUAL_PARALLEL
        //      TIMER_UPDATE_DUTY_M2(duty1, duty2, duty3);
        // #endif
    }

    // do not allow to turn on PWM outputs if virtual motor is used
    // if(virtual_motor_is_connected() == false) {
    if (!motor->m_output_on) {
        start_pwm_hw(motor);
    }
    // }
}

static void update_valpha_vbeta(volatile motor_all_state_t *motor, float mod_alpha, float mod_beta) {
    volatile motor_state_t *state_m = &motor->m_motor_state;
    volatile mc_configuration *conf_now = motor->m_conf;
    float Va, Vb, Vc;

    volatile float *ofs_volt = conf_now->foc_offsets_voltage_undriven;
    if (motor->m_state == MC_STATE_RUNNING) {
        ofs_volt = conf_now->foc_offsets_voltage;
    }

#ifdef HW_HAS_DUAL_MOTORS
#ifdef HW_HAS_3_SHUNTS
    if (&m_motor_1 != motor) {
        Va = (ADC_VOLTS(ADC_IND_SENS4) - ofs_volt[0]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
        Vb = (ADC_VOLTS(ADC_IND_SENS5) - ofs_volt[1]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
        Vc = (ADC_VOLTS(ADC_IND_SENS6) - ofs_volt[2]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
    } else {
        Va = (ADC_VOLTS(ADC_IND_SENS1) - ofs_volt[0]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
        Vb = (ADC_VOLTS(ADC_IND_SENS2) - ofs_volt[1]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
        Vc = (ADC_VOLTS(ADC_IND_SENS3) - ofs_volt[2]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
    }
#else
    if (&m_motor_1 != motor) {
        Va = (ADC_VOLTS(ADC_IND_SENS4) - ofs_volt[0]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
        Vb = (ADC_VOLTS(ADC_IND_SENS6) - ofs_volt[2]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
        Vc = (ADC_VOLTS(ADC_IND_SENS5) - ofs_volt[1]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
    } else {
        Va = (ADC_VOLTS(ADC_IND_SENS1) - ofs_volt[0]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
        Vb = (ADC_VOLTS(ADC_IND_SENS3) - ofs_volt[2]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
        Vc = (ADC_VOLTS(ADC_IND_SENS2) - ofs_volt[1]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
    }
#endif
#else
#ifdef HW_HAS_3_SHUNTS
    Va = (ADC_VOLTS(ADC_IND_SENS1) - ofs_volt[0]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
    Vb = (ADC_VOLTS(ADC_IND_SENS2) - ofs_volt[1]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
    Vc = (ADC_VOLTS(ADC_IND_SENS3) - ofs_volt[2]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
#else
    Va = (ADC_VOLTS(ADC_IND_SENS1) - ofs_volt[0]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
    Vb = (ADC_VOLTS(ADC_IND_SENS3) - ofs_volt[2]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
    Vc = (ADC_VOLTS(ADC_IND_SENS2) - ofs_volt[1]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
#endif
#endif

    // Deadtime compensation
    float s = state_m->phase_sin;
    float c = state_m->phase_cos;
    const float i_alpha_filter = c * state_m->id_filter - s * state_m->iq_filter;
    const float i_beta_filter = c * state_m->iq_filter + s * state_m->id_filter;
    const float ia_filter = i_alpha_filter;
    const float ib_filter = -0.5 * i_alpha_filter + SQRT3_BY_2 * i_beta_filter;
    const float ic_filter = -0.5 * i_alpha_filter - SQRT3_BY_2 * i_beta_filter;

    // mod_alpha_sign = 2/3*sign(ia) - 1/3*sign(ib) - 1/3*sign(ic)
    // mod_beta_sign  = 1/sqrt(3)*sign(ib) - 1/sqrt(3)*sign(ic)
    const float mod_alpha_filter_sgn = (1.0 / 3.0) * (2.0 * SIGN(ia_filter) - SIGN(ib_filter) - SIGN(ic_filter));
    const float mod_beta_filter_sgn = ONE_BY_SQRT3 * (SIGN(ib_filter) - SIGN(ic_filter));

    const float mod_comp_fact = conf_now->foc_dt_us * 1e-6 * conf_now->foc_f_zv;
    const float mod_alpha_comp = mod_alpha_filter_sgn * mod_comp_fact;
    const float mod_beta_comp = mod_beta_filter_sgn * mod_comp_fact;

    mod_alpha -= mod_alpha_comp;
    mod_beta -= mod_beta_comp;

    state_m->va = Va;
    state_m->vb = Vb;
    state_m->vc = Vc;
    state_m->mod_alpha_measured = mod_alpha;
    state_m->mod_beta_measured = mod_beta;

    // v_alpha = 2/3*Va - 1/3*Vb - 1/3*Vc
    // v_beta  = 1/sqrt(3)*Vb - 1/sqrt(3)*Vc
    float v_alpha = (1.0 / 3.0) * (2.0 * Va - Vb - Vc);
    float v_beta = ONE_BY_SQRT3 * (Vb - Vc);

    // Keep the modulation updated so that the filter stays updated
    // even when the motor is undriven.
    if (motor->m_state != MC_STATE_RUNNING) {
        /* voltage_normalize = 1/(2/3*V_bus) */
        const float voltage_normalize = 1.5 / state_m->v_bus;

        mod_alpha = v_alpha * voltage_normalize;
        mod_beta = v_beta * voltage_normalize;
    }

    float abs_rpm = fabsf(RADPS2RPM_f(motor->m_pll_speed));

    float filter_const = 1.0;
    if (abs_rpm < 10000.0) {
        filter_const = utils_map(abs_rpm, 0.0, 10000.0, 0.01, 1.0);
    }

    float v_mag = sqrtf(SQ(v_alpha) + SQ(v_beta));
    // The 0.1 * v_mag term below compensates for the filter attenuation as the speed increases.
    // It is chosen by trial and error, so this can be improved.
    UTILS_LP_FAST(state_m->v_mag_filter, v_mag + 0.1 * v_mag * filter_const, filter_const);
    UTILS_LP_FAST(state_m->mod_alpha_filter, mod_alpha, filter_const);
    UTILS_LP_FAST(state_m->mod_beta_filter, mod_beta, filter_const);
    UTILS_NAN_ZERO(state_m->v_mag_filter);
    UTILS_NAN_ZERO(state_m->mod_alpha_filter);
    UTILS_NAN_ZERO(state_m->mod_beta_filter);

    mod_alpha = state_m->mod_alpha_filter;
    mod_beta = state_m->mod_beta_filter;

    if (motor->m_state == MC_STATE_RUNNING) {
#ifdef HW_HAS_PHASE_FILTERS
        if (conf_now->foc_phase_filter_enable && abs_rpm < conf_now->foc_phase_filter_max_erpm) {
            // Compensate for the phase delay by using the direction of the modulation
            // together with the magnitude from the phase filters
            float mod_mag = sqrtf(SQ(mod_alpha) + SQ(mod_beta));
            if (mod_mag > 0.04) {
                state_m->v_alpha = mod_alpha / mod_mag * state_m->v_mag_filter;
                state_m->v_beta = mod_beta / mod_mag * state_m->v_mag_filter;
            } else {
                state_m->v_alpha = v_alpha;
                state_m->v_beta = v_beta;
            }

            state_m->is_using_phase_filters = true;
        } else {
#endif
            state_m->v_alpha = mod_alpha * (2.0 / 3.0) * state_m->v_bus;
            state_m->v_beta = mod_beta * (2.0 / 3.0) * state_m->v_bus;
            state_m->is_using_phase_filters = false;
#ifdef HW_HAS_PHASE_FILTERS
        }
#endif
    }
    else {
        state_m->v_alpha = v_alpha;
        state_m->v_beta = v_beta;
        state_m->is_using_phase_filters = false;

#ifdef HW_USE_LINE_TO_LINE
        // rotate alpha-beta 30 degrees to compensate for line-to-line phase voltage sensing
        float x_tmp = state_m->v_alpha;
        float y_tmp = state_m->v_beta;

        state_m->v_alpha = x_tmp * COS_MINUS_30_DEG - y_tmp * SIN_MINUS_30_DEG;
        state_m->v_beta = x_tmp * SIN_MINUS_30_DEG + y_tmp * COS_MINUS_30_DEG;

        // compensate voltage amplitude
        state_m->v_alpha *= ONE_BY_SQRT3;
        state_m->v_beta *= ONE_BY_SQRT3;
#endif
    }
}

/**
 * @brief svm Space vector modulation. Magnitude must not be larger than sqrt(3)/2, or 0.866 to avoid overmodulation.
 *        See https://github.com/vedderb/bldc/pull/372#issuecomment-962499623 for a full description.
 * @param alpha voltage
 * @param beta Park transformed and normalized voltage
 * @param PWMFullDutyCycle is the peak value of the PWM counter.
 * @param tAout PWM duty cycle phase A (0 = off all of the time, PWMFullDutyCycle = on all of the time)
 * @param tBout PWM duty cycle phase B
 * @param tCout PWM duty cycle phase C
 */
static void svm(float alpha, float beta, uint32_t PWMFullDutyCycle,
        uint32_t* tAout, uint32_t* tBout, uint32_t* tCout, uint32_t *svm_sector) {
    uint32_t sector;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (ONE_BY_SQRT3 * beta > alpha) {
                sector = 2;
            } else {
                sector = 1;
            }
        } else {
            //quadrant II
            if (-ONE_BY_SQRT3 * beta > alpha) {
                sector = 3;
            } else {
                sector = 2;
            }
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV5
            if (-ONE_BY_SQRT3 * beta > alpha) {
                sector = 5;
            } else {
                sector = 6;
            }
        } else {
            //quadrant III
            if (ONE_BY_SQRT3 * beta > alpha) {
                sector = 4;
            } else {
                sector = 5;
            }
        }
    }

    // PWM timings
    uint32_t tA, tB, tC;

    switch (sector) {

    // sector 1-2
    case 1: {
        // Vector on-times
        uint32_t t1 = (alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
        uint32_t t2 = (TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;

        // PWM timings
        tA = (PWMFullDutyCycle + t1 + t2) / 2;
        tB = tA - t1;
        tC = tB - t2;

        break;
    }

    // sector 2-3
    case 2: {
        // Vector on-times
        uint32_t t2 = (alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
        uint32_t t3 = (-alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

        // PWM timings
        tB = (PWMFullDutyCycle + t2 + t3) / 2;
        tA = tB - t3;
        tC = tA - t2;

        break;
    }

    // sector 3-4
    case 3: {
        // Vector on-times
        uint32_t t3 = (TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;
        uint32_t t4 = (-alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

        // PWM timings
        tB = (PWMFullDutyCycle + t3 + t4) / 2;
        tC = tB - t3;
        tA = tC - t4;

        break;
    }

    // sector 4-5
    case 4: {
        // Vector on-times
        uint32_t t4 = (-alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
        uint32_t t5 = (-TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;

        // PWM timings
        tC = (PWMFullDutyCycle + t4 + t5) / 2;
        tB = tC - t5;
        tA = tB - t4;

        break;
    }

    // sector 5-6
    case 5: {
        // Vector on-times
        uint32_t t5 = (-alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
        uint32_t t6 = (alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

        // PWM timings
        tC = (PWMFullDutyCycle + t5 + t6) / 2;
        tA = tC - t5;
        tB = tA - t6;

        break;
    }

    // sector 6-1
    case 6: {
        // Vector on-times
        uint32_t t6 = (-TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;
        uint32_t t1 = (alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

        // PWM timings
        tA = (PWMFullDutyCycle + t6 + t1) / 2;
        tC = tA - t1;
        tB = tC - t6;

        break;
    }
    }

    *tAout = tA;
    *tBout = tB;
    *tCout = tC;
    *svm_sector = sector;
}
static float correct_hall(float angle, float dt, volatile motor_all_state_t *motor) {
    volatile mc_configuration *conf_now = motor->m_conf;
    motor->m_hall_dt_diff_now += dt;

    float rad_per_sec = (M_PI / 3.0) / motor->m_hall_dt_diff_last;
    float rpm_abs_fast = fabsf(RADPS2RPM_f(motor->m_speed_est_fast));
    float rpm_abs_hall = fabsf(RADPS2RPM_f(rad_per_sec));

    // Hysteresis 5 % of total speed
    float hyst = conf_now->foc_sl_erpm * 0.1;
    if (motor->m_using_hall) {
        if (fminf(rpm_abs_fast, rpm_abs_hall) > (conf_now->foc_sl_erpm + hyst)) {
            motor->m_using_hall = false;
        }
    } else {
        if (rpm_abs_fast < (conf_now->foc_sl_erpm - hyst)) {
            motor->m_using_hall = true;
        }
    }

    int ang_hall_int = conf_now->foc_hall_table[utils_read_hall(
            motor != &m_motor_1, conf_now->m_hall_extra_samples)];

    // Only override the observer if the hall sensor value is valid.
    if (ang_hall_int < 201) {
        // Scale to the circle and convert to radians
        float ang_hall_now = ((float)ang_hall_int / 200.0) * 2 * M_PI;

        if (motor->m_ang_hall_int_prev < 0) {
            // Previous angle not valid
            motor->m_ang_hall_int_prev = ang_hall_int;
            motor->m_ang_hall = ang_hall_now;
        } else if (ang_hall_int != motor->m_ang_hall_int_prev) {
            int diff = ang_hall_int - motor->m_ang_hall_int_prev;
            if (diff > 100) {
                diff -= 200;
            } else if (diff < -100) {
                diff += 200;
            }

            // This is only valid if the direction did not just change. If it did, we use the
            // last speed together with the sign right now.
            if (SIGN(diff) == SIGN(motor->m_hall_dt_diff_last)) {
                if (diff > 0) {
                    motor->m_hall_dt_diff_last = motor->m_hall_dt_diff_now;
                } else {
                    motor->m_hall_dt_diff_last = -motor->m_hall_dt_diff_now;
                }
            } else {
                motor->m_hall_dt_diff_last = -motor->m_hall_dt_diff_last;
            }

            motor->m_hall_dt_diff_now = 0.0;

            // A transition was just made. The angle is in the middle of the new and old angle.
            int ang_avg = motor->m_ang_hall_int_prev + diff / 2;
            ang_avg %= 200;

            // Scale to the circle and convert to radians
            motor->m_ang_hall = ((float)ang_avg / 200.0) * 2 * M_PI;
        }

        motor->m_ang_hall_int_prev = ang_hall_int;

        if (RADPS2RPM_f((M_PI / 3.0) /
                fmaxf(fabsf(motor->m_hall_dt_diff_now),
                        fabsf(motor->m_hall_dt_diff_last))) < conf_now->foc_hall_interp_erpm) {
            // Don't interpolate on very low speed, just use the closest hall sensor. The reason is that we might
            // get stuck at 60 degrees off if a direction change happens between two steps.
            motor->m_ang_hall = ang_hall_now;
        } else {
            // Interpolate
            float diff = utils_angle_difference_rad(motor->m_ang_hall, ang_hall_now);
            if (fabsf(diff) < ((2.0 * M_PI) / 12.0)) {
                // Do interpolation
                motor->m_ang_hall += rad_per_sec * dt;
            } else {
                // We are too far away with the interpolation
                motor->m_ang_hall -= diff / 100.0;
            }
        }

        // Limit hall sensor rate of change. This will reduce current spikes in the current controllers when the angle estimation
        // changes fast.
        float angle_step = (fmaxf(rpm_abs_hall, conf_now->foc_hall_interp_erpm) / 60.0) * 2.0 * M_PI * dt * 1.5;
        float angle_diff = utils_angle_difference_rad(motor->m_ang_hall, motor->m_ang_hall_rate_limited);
        if (fabsf(angle_diff) < angle_step) {
            motor->m_ang_hall_rate_limited = motor->m_ang_hall;
        } else {
            motor->m_ang_hall_rate_limited += angle_step * SIGN(angle_diff);
        }

        utils_norm_angle_rad((float*)&motor->m_ang_hall_rate_limited);
        utils_norm_angle_rad((float*)&motor->m_ang_hall);

        if (motor->m_using_hall) {
            angle = motor->m_ang_hall_rate_limited;
        }
    } else {
        // Invalid hall reading. Don't update angle.
        motor->m_ang_hall_int_prev = -1;

        // Also allow open loop in order to behave like normal sensorless
        // operation. Then the motor works even if the hall sensor cable
        // gets disconnected (when the sensor spacing is 120 degrees).
        if (motor->m_phase_observer_override && motor->m_state == MC_STATE_RUNNING) {
            angle = motor->m_phase_now_observer_override;
        }
    }

    return angle;
}

