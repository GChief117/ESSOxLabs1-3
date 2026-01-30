/**
 * @file led_driver.h
 * @brief LED Driver API - Hardware Abstraction Layer for STM32F4 Discovery LEDs
 *
 * Created on: 23 Jan 2022
 * Author: zhuangzhuangdai
 * Enhanced with comments for ESS Lab study
 *
 * ============================================================================
 * CONCEPTS DEMONSTRATED (Lab 1 Tasks 3-5):
 * ============================================================================
 *
 * 1. MEMORY-MAPPED I/O
 *    In ARM Cortex-M, peripheral registers appear as memory addresses.
 *    GPIOD_ODR at 0x40020C14 controls PD0-PD15 output states.
 *
 * 2. VOLATILE KEYWORD
 *    'volatile' tells compiler the value can change unexpectedly (by hardware).
 *    Without it, compiler might cache the value or optimize away reads/writes.
 *
 * 3. ABSTRACT DATA TYPE (ADT)
 *    LED_t struct encapsulates LED state, allowing generic functions.
 *    Same code works for any LED - just pass different LED_t pointers.
 *
 * 4. HARDWARE ABSTRACTION LAYER (HAL) BENEFITS:
 *    - Portability: Change hardware, not application code
 *    - Readability: led_on(&green) vs *(uint32_t*)0x40020C14 |= 0x1000
 *    - Maintainability: Bug fixes in one place
 *    - Testability: Can mock for unit testing
 *    - Reusability: DRY (Don't Repeat Yourself) principle
 *
 * ============================================================================
 * HARDWARE REFERENCE:
 * ============================================================================
 *
 * STM32F4 Discovery Board LED connections:
 *   - PD12 (pin 12): Green LED
 *   - PD13 (pin 13): Orange LED
 *   - PD14 (pin 14): Red LED
 *   - PD15 (pin 15): Blue LED
 *
 * GPIO Port D Output Data Register (ODR):
 *   Address: 0x40020C14
 *   Bits 12-15 control LEDs (1 = ON, 0 = OFF)
 *
 * ============================================================================
 */

#ifndef INC_LED_DRIVER_H_
#define INC_LED_DRIVER_H_

#include <stdint.h>

/* ============================================================================
 * Hardware Definitions
 * ============================================================================ */

/**
 * @brief Pointer to GPIOD Output Data Register
 *
 * Memory address breakdown:
 *   GPIOD base:   0x40020C00
 *   ODR offset:   +0x14
 *   GPIOD_ODR:    0x40020C14
 *
 * Writing 1 to a bit drives the corresponding pin HIGH.
 * Writing 0 drives it LOW.
 */
#define PORTD ((volatile uint32_t*)0x40020C14)

/**
 * @brief LED Pin Definitions
 *
 * These correspond to bits in the ODR register.
 * Example: To turn on Green LED:
 *   *PORTD |= (1 << GREEN);  // Set bit 12
 */
#define GREEN  12   /* PD12 - Green LED */
#define ORANGE 13   /* PD13 - Orange LED */
#define RED    14   /* PD14 - Red LED */
#define BLUE   15   /* PD15 - Blue LED */

/* ============================================================================
 * LED Abstract Data Type
 * ============================================================================ */

/**
 * @brief LED structure - encapsulates LED hardware configuration
 *
 * This ADT allows the same functions to control any LED by storing:
 *   - port: Which GPIO port's ODR register
 *   - pin:  Which bit (0-15) in that register
 *
 * @note The 'volatile' qualifier on port is CRITICAL:
 *       - Tells compiler the memory can change unexpectedly
 *       - Prevents dangerous optimizations
 *       - Required for all hardware register pointers
 *
 * @example
 *   LED_t my_led;
 *   led_init(&my_led, PORTD, GREEN);
 *   led_on(&my_led);
 */
typedef struct {
    volatile uint32_t * port;   /**< Pointer to GPIO ODR register */
    uint32_t pin;               /**< Pin number (0-15) */
} LED_t;

/**
 * @brief LED state enumeration for state machine examples
 */
typedef enum {
    ONLY_GREEN = 0x01,
    ONLY_ORANGE,
    ONLY_RED,
    ONLY_BLUE
} led_states_t;

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/**
 * @brief Initialize an LED structure
 *
 * Configures the LED_t structure with port and pin, then turns LED off.
 *
 * @param led   Pointer to LED_t structure to initialize
 * @param port  Pointer to GPIO ODR register (e.g., PORTD)
 * @param pin   Pin number (0-15, use GREEN/ORANGE/RED/BLUE)
 *
 * @note GPIO clock must be enabled and pin configured as output BEFORE calling.
 *       This is done by ess_helper_init().
 *
 * @example
 *   LED_t green_led;
 *   led_init(&green_led, PORTD, GREEN);  // Initialize green LED, starts OFF
 */
void led_init(LED_t* led, volatile uint32_t* port, uint32_t pin);

/**
 * @brief Turn an LED off
 *
 * Clears the corresponding bit in the ODR register.
 * Implementation: *led->port &= ~(1 << led->pin);
 *
 * @param led   Pointer to initialized LED_t structure
 *
 * @example
 *   led_off(&green_led);  // Green LED turns off
 */
void led_off(LED_t* led);

/**
 * @brief Turn an LED on
 *
 * Sets the corresponding bit in the ODR register.
 * Implementation: *led->port |= (1 << led->pin);
 *
 * @param led   Pointer to initialized LED_t structure
 *
 * @example
 *   led_on(&green_led);  // Green LED turns on
 */
void led_on(LED_t* led);

#endif /* INC_LED_DRIVER_H_ */
