/*
 * timer.h
 *
 *  Created on: 6/07/2022
 *      Author: Carl
 */

#ifndef VESC_TIMER_H_
#define VESC_TIMER_H_
#include <stdint.h>
void timer_init(void);
uint64_t timer_time_now(void);
float timer_seconds_elapsed_since(uint64_t time);
uint32_t timer_milliseconds_elapsed_since(uint64_t time);
void timer_sleep_ms(uint32_t ms);


//uint32_t timer_count(uint8_t i);

#endif /* VESC_TIMER_H_ */
