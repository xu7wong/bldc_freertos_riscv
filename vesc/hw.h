/*
 * hw.h
 *
 *  Created on: 6/07/2022
 *      Author: Carl
 */

#ifndef VESC_HW_H_
#define VESC_HW_H_
#include <stdint.h>

//#include "can.h"
#include "ch32v30x_gpio.h"

#include "datatypes.h"







#define HW_NAME                 "UBCO"

#ifndef SYSTEM_CORE_CLOCK
#define SYSTEM_CORE_CLOCK           144000000
#endif

#ifndef FOC_CONTROL_LOOP_FREQ_DIVIDER
#define FOC_CONTROL_LOOP_FREQ_DIVIDER   1
#endif

//#define HW_HALL_ENC_GPIO1       GPIOC
//#define HW_HALL_ENC_PIN1        6
//#define HW_HALL_ENC_GPIO2       GPIOC
//#define HW_HALL_ENC_PIN2        7
//#define HW_HALL_ENC_GPIO3       GPIOC
//#define HW_HALL_ENC_PIN3        8

#define READ_HALL1()            GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_9)
#define READ_HALL2()            GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11)
#define READ_HALL3()            GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13)

#ifndef READ_HALL1_2
#define READ_HALL1_2()          READ_HALL1()
#endif
#ifndef READ_HALL2_2
#define READ_HALL2_2()          READ_HALL2()
#endif
#ifndef READ_HALL3_2
#define READ_HALL3_2()          READ_HALL3()
#endif

#define HW_ADC_CHANNELS         10
// #define HW_ADC_INJ_CHANNELS      3
#define HW_ADC_NBR_CONV         5

#define HW_ADC_CHANNELS_EXTRA   0

// ADC Indexes
#define ADC_IND_SENS1           0
#define ADC_IND_SENS2           2
#define ADC_IND_SENS3           4
#define ADC_IND_CURR1           1
#define ADC_IND_CURR2           3
#define ADC_IND_CURR3           5

// #define ADC_IND_TEMP_MOS     7
#define ADC_IND_TEMP_MOTOR       9
#define ADC_IND_VIN_SENS        6
// #define ADC_IND_VREFINT          8
#define ADC_IND_V12V          8
#define ADC_IND_VEXT          7

#define ENABLE_GATE()           asm ("nop")//palSetPad(GPIOB, 5)
#define DISABLE_GATE()          asm ("nop")//palClearPad(GPIOB, 5)
// #endif
// #define DCCAL_ON()
// #define DCCAL_OFF()
#define IS_DRV_FAULT()          0//(!palReadPad(GPIOB, 7))

#define LED_GREEN_ON()          palSetPad(GPIOB, 0)
#define LED_GREEN_OFF()         palClearPad(GPIOB, 0)
#define LED_RED_ON()            palSetPad(GPIOB, 1)
#define LED_RED_OFF()           palClearPad(GPIOB, 1)

// #define HW_HAS_DRV8301
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS


#ifndef V_REG
#define V_REG                   3.28
#endif
#ifndef VIN_R1
#define VIN_R1                  56000.0
#endif
#ifndef VIN_R2
#define VIN_R2                  2200.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN        45.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES       0.0003
#endif

#define FAC_CURRENT                 ((V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN))

// Input voltage
#define GET_INPUT_VOLTAGE()     ((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

#define ADC_V_L1                ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2                ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3                ADC_Value[ADC_IND_SENS3]

// #ifndef ADC_V_L4
// #define ADC_V_L4             ADC_V_L1
// #endif
// #ifndef ADC_V_L5
// #define ADC_V_L5             ADC_V_L2
// #endif
// #ifndef ADC_V_L6
// #define ADC_V_L6             ADC_V_L3
// #endif


#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE     0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE     0
#endif
#ifndef CURR3_DOUBLE_SAMPLE
#define CURR3_DOUBLE_SAMPLE     0
#endif

#ifndef GET_CURRENT1
#define GET_CURRENT1()      ADC_Value[ADC_IND_CURR1]
#endif

#ifndef GET_CURRENT2
#define GET_CURRENT2()      ADC_Value[ADC_IND_CURR2]
#endif

#ifndef GET_CURRENT3
#define GET_CURRENT3()      ADC_Value[ADC_IND_CURR3]
#endif

#ifndef ADC_VOLTS_PH_FACTOR
#define ADC_VOLTS_PH_FACTOR     1.0
#endif
#ifndef ADC_VOLTS_INPUT_FACTOR
#define ADC_VOLTS_INPUT_FACTOR  1.0
#endif

#define ADC_VOLTS(ch)           ((float)ADC_Value[ch] / 4096.0 * V_REG)
// #define ADC_V_ZERO               (ADC_Value[ADC_IND_VIN_SENS] / 2)



#define LED1_ON() GPIO_ResetBits(GPIOE, GPIO_Pin_0)
#define LED1_OFF() GPIO_SetBits(GPIOE, GPIO_Pin_0)
#define LED2_ON() GPIO_ResetBits(GPIOE, GPIO_Pin_1)
#define LED2_OFF() GPIO_SetBits(GPIOE, GPIO_Pin_1)




//extern uint16_t ADC_Value[HW_ADC_CHANNELS];
extern int16_t Calibrattion_Val1;
extern int16_t Calibrattion_Val2;

void hw_init_gpio(void);
void hw_init_peripherals(void);
void hw_setup_adc_channels(void);

void confgenerator_set_defaults_mcconf(mc_configuration *conf);
uint8_t conf_general_calculate_deadtime(float deadtime_ns, float core_clock_freq);

uint16_t Get_ConversionVal1(int16_t val);
uint16_t Get_ConversionVal2(int16_t val);


#endif /* VESC_HW_H_ */
