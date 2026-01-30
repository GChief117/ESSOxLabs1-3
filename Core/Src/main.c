/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ess_helper.h"
#include "led_driver.h"
#include "pwm_driver.h"
#include "stdio.h"
#include "spi_driver.h"
#include "acc_driver.h"
#include "tmp_driver.h"


int _write(int file, char *ptr, int len){
	int i = 0;
	for (i=0;i<len;i++){
		ITM_SendChar((*ptr++));
	}
	return  len;
}

void delay_usec(uint32_t delay){
	volatile uint32_t dummy;
	while (delay-- > 0){
		for (uint32_t i=0;i<21;i++){
			dummy++;
		}
	}
}

// Initialize Timer 3, 4
TIM_HandleTypeDef TIM_Handle3; //define a handle
TIM_HandleTypeDef TIM_Handle4;
volatile uint8_t tmr3_flag = 0;
volatile uint8_t tmr4_flag = 0;

// Initialize Timer 4 for interrupts
void TMR4_Init_ISR(void) {
	/* Enable clock for TIM4 */
	__TIM4_CLK_ENABLE();
	TIM_Handle4.Instance = TIM4;
	//Same timer whose clocks we enable
	// timer_tick_frequency = 84000000 / (0 + 1) = 84000000
	TIM_Handle4.Init.Prescaler = 0;
	/* Count up */
	TIM_Handle4.Init.CounterMode = TIM_COUNTERMODE_UP;
	/*
Set timer period when it must reset
First you have to know max value for timer
In our case it is 16bit = 65535
Frequency = timer_tick_frequency / (TIM_Period + 1)
If you get Period larger than max timer value (in our case 65535),
you have to choose larger prescaler and slow down timer tick frequency
	 */
	TIM_Handle4.Init.Period = 8399;// <your value here> 32Hz
	TIM_Handle4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM_Handle4.Init.RepetitionCounter = 0;
	TIM_Handle4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	/* Initialize TIM4 */
	HAL_TIM_Base_Init(&TIM_Handle4);
	/* Start count on TIM4 */
	HAL_TIM_Base_Start_IT(&TIM_Handle4);
	/* Enable the TIM4 gloabal Interrupt */
	HAL_NVIC_SetPriority(TIM4_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

// This is triggered when the counter overflows
void TIM4_IRQHandler(void)
{
	//In case other interrupts are also running
	if (__HAL_TIM_GET_FLAG(&TIM_Handle4, TIM_FLAG_UPDATE) != RESET)
	{
		if (__HAL_TIM_GET_ITSTATUS(&TIM_Handle4, TIM_IT_UPDATE) != RESET)
		{
			__HAL_TIM_CLEAR_FLAG(&TIM_Handle4, TIM_FLAG_UPDATE);
			/*put your code here */
			tmr4_flag = 1;
		}
	}
}

// Initialize Timer 3 for interrupts
void TMR3_Init_ISR(void) {
	/* Enable clock for TIM3 */
	__TIM3_CLK_ENABLE();
	TIM_Handle3.Instance = TIM3;
	//Same timer whose clocks we enable
	// timer_tick_frequency = 84000000 / (0 + 1) = 84000000
	TIM_Handle3.Init.Prescaler = 0;
	/* Count up */
	TIM_Handle3.Init.CounterMode = TIM_COUNTERMODE_UP;
	/*
Set timer period when it must reset
First you have to know max value for timer
In our case it is 16bit = 65535
Frequency = timer_tick_frequency / (TIM_Period + 1)
If you get Period larger than max timer value (in our case 65535),
you have to choose larger prescaler and slow down timer tick frequency
	 */
	TIM_Handle3.Init.Period = 2624999;// <your value here> 10kHz
	TIM_Handle3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM_Handle3.Init.RepetitionCounter = 0;
	TIM_Handle3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	/* Initialize TIM4 */
	HAL_TIM_Base_Init(&TIM_Handle3);
	/* Start count on TIM4 */
	HAL_TIM_Base_Start_IT(&TIM_Handle3);
	/* Enable the TIM4 gloabal Interrupt */
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
}
// This is triggered when the counter overflows
void TIM3_IRQHandler(void)
{
	//In case other interrupts are also running
	if (__HAL_TIM_GET_FLAG(&TIM_Handle3, TIM_FLAG_UPDATE) != RESET)
	{
		if (__HAL_TIM_GET_ITSTATUS(&TIM_Handle3, TIM_IT_UPDATE) != RESET)
		{
			__HAL_TIM_CLEAR_FLAG(&TIM_Handle3, TIM_FLAG_UPDATE);
			/*put your code here */
			tmr3_flag = 1;
		}
	}
}

typedef enum {
	DISPLAY_OFF = 0x00,
	DISPLAY_TILT,
	DISPLAY_VIBRATION,
	DISPLAY_IMPACT,
	IMPACT_STATE} display_state_t;

int main(void)
{
	/* Initialize system */
	HAL_Init();
	/* Initialize peripherals on board */
	ess_helper_init();

	LED_t led_green;
	LED_t led_orange;
	LED_t led_red;
	LED_t led_blue;

	led_init(&led_green, PORTD, GREEN);
	led_init(&led_orange, PORTD, ORANGE);
	led_init(&led_red, PORTD, RED);
	led_init(&led_blue, PORTD, BLUE);

	// Display FSM states
	static unsigned char state = DISPLAY_OFF;
	static unsigned char leds_state = ONLY_GREEN;
	static unsigned char flash_state = CLOCKWISE;

	// Initial test
	//uint32_t * ptr = PORTD;
	//*ptr = 0xF000;
	//*(uint32_t*)0x40020C14 = 0x1000;

	// Button press
	uint8_t button_press = 0;
	volatile uint16_t shadow = 0;

	// PWM Init
	pwm_driver_init(&led_green, &led_orange, &led_red, &led_blue);
	pwm_driver_set(GREEN, 0);
	pwm_driver_set(ORANGE, 0);
	pwm_driver_set(RED, 0);
	pwm_driver_set(BLUE, 0);

	// PWM loop counter
	uint32_t loop_counter = 0;
	uint32_t brightness_counter = 0;

	// Init ISRs
	TMR3_Init_ISR();
	TMR4_Init_ISR();

	// Accelerometer
	acc3_t acc_readings;
	acc3_t ave;
	acc3_t vib;
	uint16_t brightness = 0;

	// Impact, vibration, tilt
	acc3_t cir_buff[16];
	acc3_t* ptr_buff;
	ptr_buff = cir_buff;
	uint8_t full_flag = 0;
	int32_t sum_x = 0;
	int32_t sum_y = 0;
	int32_t sum_z = 0;
	uint32_t impact = 0;
	uint8_t impact_flag = 0;

	if (AccInit()){
		printf("AccInit Successful!");
	}

    while(1){
    	// Catch rising edge
    	if ((GPIOA->IDR & 0x0001) && ((shadow & 0x0001) == 0)){
    		button_press = 1;
    	}
    	shadow = GPIOA->IDR;

    	/*
    	// LED ON/OFF FSM
    	switch(leds_state){
    	case ONLY_GREEN:
    		led_on(&led_green);
    		if (button_press){
    			button_press = 0;
    			leds_state = ONLY_RED;
    			led_off(&led_green);
    			printf("Switch to Red LED.\n");
    		}
    		break;
    	case ONLY_RED:
    		led_on(&led_red);
    		if (button_press){
    			button_press = 0;
    			leds_state = ONLY_GREEN;
    			led_off(&led_red);
    			printf("Switch to Green LED.\n");
    		}
    		break;
    	default:
    		break;
    	}
    	*/

    	/*
    	// ON-OFF Ratio tuning
    	if (loop_counter++ >= CYCLE_TIME){
    		loop_counter = 0;
    		pwm_driver_set(GREEN, brightness_counter % PWM_MAX);
    		pwm_driver_set(ORANGE, (brightness_counter + 25) % PWM_MAX);
    		pwm_driver_set(RED, (brightness_counter + 50) % PWM_MAX);
    		pwm_driver_set(BLUE, (brightness_counter + 75) % PWM_MAX);
    		if (brightness_counter++ > PWM_MAX){
    			brightness_counter = 0;
    		}
    	}
    	*/

    	/*
    	// CLOCKWISE_COUNTER-CLOCKWISE LED FLASH FSM
    	switch(flash_state){
    	case CLOCKWISE:
    		// ON-OFF Ratio tuning
    		if (loop_counter++ >= CYCLE_TIME){
    			loop_counter = 0;
    			pwm_driver_set(GREEN, brightness_counter % PWM_MAX);
    			pwm_driver_set(ORANGE, (brightness_counter + 25) % PWM_MAX);
    			pwm_driver_set(RED, (brightness_counter + 50) % PWM_MAX);
    			pwm_driver_set(BLUE, (brightness_counter + 75) % PWM_MAX);
    			if (brightness_counter++ > PWM_MAX){
    				brightness_counter = 0;
    			}
    		}
    		if (button_press){
    			button_press = 0;
    			flash_state = COUNTER_CLOCKWISE;
    			printf("Switch to Counter Clockwise.\n");
    		}
    		break;
    	case COUNTER_CLOCKWISE:
    		if (loop_counter++ >= CYCLE_TIME){
    			loop_counter = 0;
    			pwm_driver_set(GREEN, brightness_counter % PWM_MAX);
    			pwm_driver_set(ORANGE, (brightness_counter + 25) % PWM_MAX);
    			pwm_driver_set(RED, (brightness_counter + 50) % PWM_MAX);
    			pwm_driver_set(BLUE, (brightness_counter + 75) % PWM_MAX);
    			if (brightness_counter-- <= 0){
    				brightness_counter = PWM_MAX - 1;
    			}
    		}
    		if (button_press){
    			button_press = 0;
    			flash_state = CLOCKWISE;
    			printf("Switch to Clockwise.\n");
    		}
    		break;
    	}


    	// ON-OFF Action
    	delay_usec(100);
    	pwm_driver_update();
    	*/

    	if (tmr3_flag){
    		tmr3_flag = 0;
    		// Sensing
    		AccRead(&acc_readings);

    		// Display tilt angle
    		if (state == DISPLAY_TILT){
    			// X axis
    			if (acc_readings.x > 11584){
    				// +x Fully bright
    				pwm_driver_set(RED, 100);
    				pwm_driver_set(GREEN, 0);
    			}
    			else if (acc_readings.x < -11584){
    				// -x Fully bright
    				pwm_driver_set(RED, 0);
    				pwm_driver_set(GREEN, 100);
    			}
    			else if ((acc_readings.x > -1000) && (acc_readings.x < 1000)) {
    				// Off both
    				pwm_driver_set(RED, 0);
    				pwm_driver_set(GREEN, 0);
    			}
    			else if (acc_readings.x > 0){
    				// Smooth RED
    				pwm_driver_set(GREEN, 0);
    				brightness = (int16_t)acc_readings.x * 100 / (uint16_t)11586;
    				pwm_driver_set(RED, brightness);
    			}
    			else {
    				// Smooth GREEN
    				pwm_driver_set(RED, 0);
    				brightness = (int16_t)acc_readings.x * 100 / (uint16_t)11586;
    				pwm_driver_set(GREEN, brightness);
    			}
    			// Y axis
    			if (acc_readings.y > 11584){
    				// +x Fully bright
    				pwm_driver_set(ORANGE, 100);
    				pwm_driver_set(BLUE, 0);
    			}
    			else if (acc_readings.y < -11584){
    				// -x Fully bright
    				pwm_driver_set(ORANGE, 0);
    				pwm_driver_set(BLUE, 100);
    			}
    			else if ((acc_readings.y > -1000) && (acc_readings.y < 1000)) {
    				// Off both
    				pwm_driver_set(ORANGE, 0);
    				pwm_driver_set(BLUE, 0);
    			}
    			else if (acc_readings.y > 0){
    				// Smooth RED
    				pwm_driver_set(BLUE, 0);
    				brightness = (int16_t)acc_readings.y * 100 / (uint16_t)11586;
    				pwm_driver_set(ORANGE, brightness);
    			}
    			else {
    				// Smooth GREEN
    				pwm_driver_set(ORANGE, 0);
    				brightness = (int16_t)acc_readings.y * 100 / (uint16_t)11586;
    				pwm_driver_set(BLUE, brightness);
    			}
    		}

    		// Tilt: cir_buf[16] to hold 0.5s of Acc data
    		*ptr_buff = acc_readings;
    		if (full_flag){
    			full_flag = 0;
    			ptr_buff = cir_buff;

    			for(uint8_t i=0; i<16; i++){
    				sum_x += cir_buff[i].x;
    				sum_y += cir_buff[i].y;
    				sum_z += cir_buff[i].z;
    			}
    			ave.x = (int16_t)sum_x / (uint16_t)16;
    			ave.y = (int16_t)sum_y / (uint16_t)16;
    			ave.z = (int16_t)sum_z / (uint16_t)16;
    			sum_x = 0;
    			sum_y = 0;
    			sum_z = 0;
    			// Vibration: variance
    			for(uint8_t i=0; i<16; i++){
    				sum_x += (cir_buff[i].x - ave.x) * (cir_buff[i].x - ave.x);
    				sum_y += (cir_buff[i].y - ave.y) * (cir_buff[i].y - ave.y);
    				sum_z += (cir_buff[i].z - ave.z) * (cir_buff[i].z - ave.z);
    			}
    			vib.x = (int16_t)sum_x / (uint16_t)16;
    			vib.y = (int16_t)sum_y / (uint16_t)16;
    			vib.z = (int16_t)sum_z / (uint16_t)16;
    			sum_x = 0;
    			sum_y = 0;
    			sum_z = 0;
    		}
    		// Impact
    		impact = ptr_buff->x * ptr_buff->x + ptr_buff->y * ptr_buff->y + ptr_buff->z * ptr_buff->z;
    		if (impact > ((uint32_t)0x01 << 30)){
    			impact_flag = 1;
    			printf("Shaking your board");
    		}
    		if (ptr_buff++ == &cir_buff[15]){
    			full_flag = 1;
    		}

    		// Display FSM
    		switch(state)
    		{
    		case DISPLAY_OFF:
    			// Display None
    			if (button_press)
    			{
    				button_press = 0;
    				state = DISPLAY_TILT;
    			}
    			break;
    		case DISPLAY_TILT:
    			// Display tilt angle
    			if (button_press)
    			{
    				button_press = 0;
    				state = DISPLAY_IMPACT;
    				pwm_driver_set(GREEN, 0);
    				pwm_driver_set(ORANGE, 0);
    				pwm_driver_set(RED, 0);
    				pwm_driver_set(BLUE, 0);
    			}
    			break;
    		case DISPLAY_IMPACT:
    			// Display Full LEDs ON if there is "Shake-shake"
    			if (impact_flag){
    				impact_flag = 0;
    				pwm_driver_set(GREEN, 100);
    				pwm_driver_set(ORANGE, 100);
    				pwm_driver_set(RED, 100);
    				pwm_driver_set(BLUE, 100);
    			}
    			if (button_press)
    			{
    				button_press = 0;
    				state = DISPLAY_OFF;
    				pwm_driver_set(GREEN, 0);
    				pwm_driver_set(ORANGE, 0);
    				pwm_driver_set(RED, 0);
    				pwm_driver_set(BLUE, 0);
    			}
    			break;
    		}
    	}

    	// PWM refresh
    	if (tmr4_flag){
    		tmr4_flag = 0;
    		pwm_driver_update();
    	}

    	/*
		// Another timer for Temperature @10Hz?
		uint32_t temp;
		float v_sense;
		float temp_c;
		get_tmp_value(&temp);
		v_sense = (float)temp * 3.3f / 4095.0f;
		temp_c = (v_sense - 0.76f) / 0.0025f + 25.0f;
		if (temp_c > 60.0f)
		{
			printf("Temperature is %.2f\n", temp_c);
		}
		*/
    }
}
