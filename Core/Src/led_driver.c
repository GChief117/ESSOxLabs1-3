/**
 * @file led_driver.c
 * @brief LED Driver Implementation - Bit Manipulation for GPIO Control
 *
 * Created on: 23 Jan 2022
 * Author: zhuangzhuangdai
 * Enhanced with comments for ESS Lab study
 *
 * ============================================================================
 * BIT MANIPULATION TECHNIQUES:
 * ============================================================================
 *
 * 1. SET A BIT (turn LED on):
 *    register |= (1 << bit_position);
 *
 *    Example for pin 12:
 *    - (1 << 12) = 0001 0000 0000 0000 (binary) = 0x1000 (hex)
 *    - OR with register sets bit 12, leaves others unchanged
 *
 * 2. CLEAR A BIT (turn LED off):
 *    register &= ~(1 << bit_position);
 *
 *    Example for pin 12:
 *    - (1 << 12) = 0001 0000 0000 0000
 *    - ~(1 << 12) = 1110 1111 1111 1111 (inverted)
 *    - AND with register clears bit 12, leaves others unchanged
 *
 * 3. TOGGLE A BIT:
 *    register ^= (1 << bit_position);
 *
 * 4. TEST A BIT:
 *    if (register & (1 << bit_position)) { // bit is set }
 *
 * ============================================================================
 */

#include "main.h"
#include "led_driver.h"

/**
 * @brief Initialize an LED
 *
 * Stores the port and pin configuration in the LED structure,
 * then ensures the LED starts in the OFF state.
 *
 * WHY USE A STRUCT?
 * - Decouples LED logic from specific pins
 * - Same functions work for any LED
 * - Easy to add more LEDs or change pins
 * - Follows the "Open-Closed Principle" (open for extension, closed for modification)
 *
 * @param led   Pointer to LED_t structure
 * @param port  Pointer to GPIO ODR (Output Data Register)
 * @param pin   Pin number (0-15)
 */
void led_init(LED_t * led, volatile uint32_t * port, uint32_t pin)
{
    /* Store configuration in the structure */
    led->port = port;
    led->pin = pin;
    
    /* Start with LED off - known initial state */
    led_off(led);
}

/**
 * @brief Turn LED off (clear the output bit)
 *
 * TECHNIQUE: Bit Clear using AND with inverted mask
 *
 * Step-by-step for pin 12:
 *   1. (0x01 << 12) = 0x1000 = 0001 0000 0000 0000
 *   2. ~(0x01 << 12) = 0xEFFF = 1110 1111 1111 1111
 *   3. *port &= 0xEFFF clears bit 12, preserves all other bits
 *
 * WHY THIS WORKS:
 *   - x AND 1 = x (bit unchanged)
 *   - x AND 0 = 0 (bit cleared)
 *
 * @param led   Pointer to initialized LED_t
 */
void led_off(LED_t* led)
{
    *led->port &= ~(0x01 << led->pin);
}

/**
 * @brief Turn LED on (set the output bit)
 *
 * TECHNIQUE: Bit Set using OR
 *
 * Step-by-step for pin 12:
 *   1. (0x01 << 12) = 0x1000 = 0001 0000 0000 0000
 *   2. *port |= 0x1000 sets bit 12, preserves all other bits
 *
 * WHY THIS WORKS:
 *   - x OR 0 = x (bit unchanged)
 *   - x OR 1 = 1 (bit set)
 *
 * @param led   Pointer to initialized LED_t
 */
void led_on(LED_t* led)
{
    *led->port |= 0x01 << led->pin;
}
