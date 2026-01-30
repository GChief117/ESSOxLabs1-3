/*
 * pwm_driver.c
 *
 *  Created on: 26 Jan 2022
 *      Author: zhuangzhuangdai
 */

#include "pwm_driver.h"

#define CYCLE_TIME 100
#define PWM_MAX 100


struct pwm_state
{
uint32_t counter;
uint32_t ch0_compare;
uint32_t ch1_compare;
uint32_t ch2_compare;
uint32_t ch3_compare;
LED_t * ch0;
LED_t * ch1;
LED_t * ch2;
LED_t * ch3;
};

static struct pwm_state state;

void pwm_driver_init(LED_t * ch0, LED_t * ch1, LED_t * ch2, LED_t * ch3)
{
	state.counter = 0;
	state.ch0_compare = 0;
	state.ch1_compare = 0;
	state.ch2_compare = 0;
	state.ch3_compare = 0;
	state.ch0 = ch0;
	state.ch1 = ch1;
	state.ch2 = ch2;
	state.ch3 = ch3;
}

void pwm_driver_set(uint8_t channel, uint8_t value)
{
	if (value > PWM_MAX){
		value = PWM_MAX;
	}
	switch (channel)
	{
	case(GREEN):
			state.ch0_compare = value;
	break;
	case(ORANGE):
			state.ch1_compare = value;
	break;
	case(RED):
			state.ch2_compare = value;
	break;
	case(BLUE):
			state.ch3_compare = value;
	break;
	}
}

void pwm_driver_update(void)
{
	if (state.ch0_compare > state.counter)
	{
		led_on(state.ch0);
	}
	else{
		led_off(state.ch0);
	}
	if (state.ch1_compare > state.counter)
	{
		led_on(state.ch1);
	}
	else{
		led_off(state.ch1);
	}
	if (state.ch2_compare > state.counter)
	{
		led_on(state.ch2);
	}
	else{
		led_off(state.ch2);
	}
	if (state.ch3_compare > state.counter)
	{
		led_on(state.ch3);
	}
	else{
		led_off(state.ch3);
	}

	if ((state.counter++) > PWM_MAX)
	{
		state.counter = 0;
	}
}

