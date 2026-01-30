/**
 * @file pwm_driver.h
 * @brief Software PWM Driver - LED Brightness Control
 *
 * Created on: 26 Jan 2022
 * Author: zhuangzhuangdai
 * Enhanced with comments for ESS Lab 2 study
 *
 * ============================================================================
 * PWM (Pulse Width Modulation) CONCEPT:
 * ============================================================================
 *
 * PWM controls average power by rapidly switching between ON and OFF states.
 * The "duty cycle" determines the percentage of time the signal is HIGH.
 *
 *   100% duty: ████████████████████ (Always ON - Full brightness)
 *    75% duty: ███████████████      (75% ON - Bright)
 *    50% duty: ██████████           (50% ON - Medium)
 *    25% duty: █████                (25% ON - Dim)
 *     0% duty:                      (Always OFF)
 *
 * At high enough frequency (>100 Hz), the eye perceives an "average"
 * brightness rather than flicker.
 *
 * ============================================================================
 * SOFTWARE PWM IMPLEMENTATION:
 * ============================================================================
 *
 * This driver implements "software PWM" using a counter-compare approach:
 *
 * 1. A counter increments from 0 to PWM_MAX (100)
 * 2. Each LED has a "compare" value (0-100)
 * 3. If counter < compare: LED ON
 *    If counter >= compare: LED OFF
 *
 * Example with compare = 75:
 *   Counter: 0  10  20  30  40  50  60  70  80  90  100 (repeats)
 *   LED:     ON  ON  ON  ON  ON  ON  ON  ON OFF OFF OFF
 *                                    └─ compare value
 *
 * The pwm_driver_update() function must be called at a high frequency
 * (e.g., 10 kHz by TIM4 interrupt) for smooth brightness control.
 *
 * ============================================================================
 */

#ifndef INC_PWM_DRIVER_H_
#define INC_PWM_DRIVER_H_

#include <stdint.h>
#include "led_driver.h"

/**
 * @brief Flash direction states for rotating LED demo
 */
typedef enum {
    CLOCKWISE = 0x01,
    COUNTER_CLOCKWISE,
} flash_states_t;

/* ============================================================================
 * PWM Driver API
 * ============================================================================ */

/**
 * @brief Initialize the PWM driver with 4 LED channels
 *
 * @param ch0   Pointer to LED_t for channel 0 (Green)
 * @param ch1   Pointer to LED_t for channel 1 (Orange)
 * @param ch2   Pointer to LED_t for channel 2 (Red)
 * @param ch3   Pointer to LED_t for channel 3 (Blue)
 *
 * @example
 *   pwm_driver_init(&led_green, &led_orange, &led_red, &led_blue);
 */
void pwm_driver_init(LED_t * ch0, LED_t *ch1, LED_t * ch2, LED_t * ch3);

/**
 * @brief Set PWM duty cycle for a channel
 *
 * @param channel   LED to control (GREEN, ORANGE, RED, or BLUE)
 * @param value     Duty cycle 0-100 (0=off, 100=full brightness)
 *
 * @example
 *   pwm_driver_set(GREEN, 50);   // Green at 50% brightness
 *   pwm_driver_set(RED, 100);    // Red at full brightness
 *   pwm_driver_set(BLUE, 0);     // Blue off
 */
void pwm_driver_set(uint8_t channel, uint8_t value);

/**
 * @brief Update PWM outputs (call at high frequency, e.g., 10 kHz)
 *
 * This function increments the internal counter and compares it
 * against each channel's duty cycle to determine ON/OFF state.
 *
 * MUST be called regularly (typically from a timer interrupt)
 * for smooth PWM operation.
 *
 * @example
 *   // In TIM4 interrupt handler (10 kHz):
 *   void TIM4_IRQHandler(void) {
 *       // Clear flag...
 *       pwm_driver_update();
 *   }
 */
void pwm_driver_update(void);

#endif /* INC_PWM_DRIVER_H_ */
