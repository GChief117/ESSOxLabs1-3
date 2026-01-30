/**
 * @file main.c
 * @brief ESS Labs 1-4: Smart Pallet Prototype - Study Version
 * 
 * @author ESS Course / Oxford (Enhanced for self-study)
 * @board STM32F4 Discovery (STM32F407VGT6)
 * 
 * This version includes selectable DEMO_MODE to test each lab task individually.
 * Change DEMO_MODE below to explore different functionality.
 * 
 * ============================================================================
 * DEMO MODES - Change this value to test different lab tasks
 * ============================================================================
 */
#define DEMO_MODE 9  /* <-- CHANGE THIS to select demo (1-9) */

/*
 * Available modes:
 * 
 * LAB 1 - LED Driver:
 *   1 = Task 2: Direct register access (all LEDs on)
 *   2 = Task 3c: Simple green LED on using driver
 *   3 = Task 3d: Fast blink (demonstrates why we need delay)
 *   4 = Task 4: Rotating LED pattern with delay
 *   5 = Task 6: Button-controlled LED cycling
 * 
 * LAB 2 - PWM Driver:
 *   6 = PWM brightness sweep (all LEDs fade in/out)
 * 
 * LAB 4 - Accelerometer:
 *   7 = Tilt display (tilt board to light different LEDs)
 *   8 = Impact detection (shake board to trigger)
 * 
 * FULL SYSTEM:
 *   9 = Complete demo (button cycles through modes)
 */

/* ============================================================================
 * Includes
 * ============================================================================ */
#include "main.h"
#include "ess_helper.h"
#include "led_driver.h"
#include "pwm_driver.h"
#include "stdio.h"
#include "spi_driver.h"
#include "acc_driver.h"
#include "tmp_driver.h"

/* ============================================================================
 * Printf Redirect for SWV Debugging (Task 7)
 * ============================================================================
 * 
 * This allows printf() output to appear in the SWV ITM Data Console.
 * 
 * Setup in STM32CubeIDE:
 * 1. Debug Configuration → Debugger → Enable SWV, Core Clock = 168 MHz
 * 2. Debug mode: Window → Show View → SWV → SWV ITM Data Console
 * 3. Click wrench icon, enable Port 0
 * 4. Click "Start Trace" before resuming execution
 */
int _write(int file, char *ptr, int len)
{
    int i = 0;
    for (i = 0; i < len; i++) {
        ITM_SendChar((*ptr++));
    }
    return len;
}

/* ============================================================================
 * Delay Functions
 * ============================================================================ */

/**
 * @brief Microsecond delay (approximate)
 * 
 * Uses a busy-wait loop calibrated for 168 MHz.
 * NOT accurate - use hardware timers for precise timing.
 */
void delay_usec(uint32_t delay)
{
    volatile uint32_t dummy;
    while (delay-- > 0) {
        for (uint32_t i = 0; i < 21; i++) {
            dummy++;
        }
    }
}

/**
 * @brief Millisecond delay (approximate)
 */
void delay_msec(uint32_t delay)
{
    while (delay-- > 0) {
        delay_usec(1000);
    }
}

/* ============================================================================
 * Timer Configuration (for Labs 2-4)
 * ============================================================================
 * 
 * Timer Calculation:
 *   APB1 Timer Clock = 84 MHz (HCLK/2, then x2 for timers)
 *   
 *   Frequency = Timer_Clock / ((Prescaler + 1) * (Period + 1))
 *   
 *   TIM4: 84 MHz / (1 * 8400) = 10 kHz (for PWM refresh)
 *   TIM3: 84 MHz / (1 * 2625000) = 32 Hz (for accelerometer sampling)
 */

TIM_HandleTypeDef TIM_Handle3;
TIM_HandleTypeDef TIM_Handle4;
volatile uint8_t tmr3_flag = 0;
volatile uint8_t tmr4_flag = 0;

/**
 * @brief Initialize Timer 4 for 10 kHz interrupts (PWM refresh)
 */
void TMR4_Init_ISR(void)
{
    __TIM4_CLK_ENABLE();
    TIM_Handle4.Instance = TIM4;
    TIM_Handle4.Init.Prescaler = 0;
    TIM_Handle4.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM_Handle4.Init.Period = 8399;  /* 84 MHz / 8400 = 10 kHz */
    TIM_Handle4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_Handle4.Init.RepetitionCounter = 0;
    TIM_Handle4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&TIM_Handle4);
    HAL_TIM_Base_Start_IT(&TIM_Handle4);
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&TIM_Handle4, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_ITSTATUS(&TIM_Handle4, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_FLAG(&TIM_Handle4, TIM_FLAG_UPDATE);
            tmr4_flag = 1;
        }
    }
}

/**
 * @brief Initialize Timer 3 for 32 Hz interrupts (accelerometer sampling)
 */
void TMR3_Init_ISR(void)
{
    __TIM3_CLK_ENABLE();
    TIM_Handle3.Instance = TIM3;
    TIM_Handle3.Init.Prescaler = 0;
    TIM_Handle3.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM_Handle3.Init.Period = 2624999;  /* 84 MHz / 2625000 = 32 Hz */
    TIM_Handle3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_Handle3.Init.RepetitionCounter = 0;
    TIM_Handle3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&TIM_Handle3);
    HAL_TIM_Base_Start_IT(&TIM_Handle3);
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&TIM_Handle3, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_ITSTATUS(&TIM_Handle3, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_FLAG(&TIM_Handle3, TIM_FLAG_UPDATE);
            tmr3_flag = 1;
        }
    }
}

/* ============================================================================
 * Display State Machine (for full system demo)
 * ============================================================================ */
typedef enum {
    DISPLAY_OFF = 0x00,
    DISPLAY_TILT,
    DISPLAY_VIBRATION,
    DISPLAY_IMPACT,
    IMPACT_STATE
} display_state_t;

/* ============================================================================
 * Main Function
 * ============================================================================ */
int main(void)
{
    /* Initialize system clock (168 MHz) and peripherals */
    HAL_Init();
    ess_helper_init();

    /* ========================================================================
     * LED Initialization (Lab 1 Task 4 - Generic LED Driver)
     * ======================================================================== */
    LED_t led_green;
    LED_t led_orange;
    LED_t led_red;
    LED_t led_blue;

    led_init(&led_green, PORTD, GREEN);
    led_init(&led_orange, PORTD, ORANGE);
    led_init(&led_red, PORTD, RED);
    led_init(&led_blue, PORTD, BLUE);

    /* Button debouncing variables */
    uint8_t button_press = 0;
    volatile uint16_t shadow = 0;

    printf("\r\n========================================\r\n");
    printf("ESS Labs - Smart Pallet Prototype\r\n");
    printf("Demo Mode: %d\r\n", DEMO_MODE);
    printf("========================================\r\n\r\n");

/* ============================================================================
 * DEMO 1: Lab 1 Task 2 - Direct Register Access
 * ============================================================================
 * 
 * This is the simplest way to control LEDs - directly writing to the
 * GPIO Output Data Register (ODR) at address 0x40020C14.
 * 
 * 0xF000 = 0b1111 0000 0000 0000
 *          Bits 15,14,13,12 = Blue, Red, Orange, Green LEDs
 */
#if DEMO_MODE == 1

    printf("Lab 1 Task 2: Direct Register Access\r\n");
    printf("All LEDs should be ON\r\n");
    printf("Memory address 0x40020C14 = GPIOD_ODR\r\n\r\n");

    /* Method 1: Direct write */
    *(uint32_t*)0x40020C14 = 0xF000;

    /* Alternative: Try these to see individual LEDs:
     * *(uint32_t*)0x40020C14 = 0x1000;  // Green only
     * *(uint32_t*)0x40020C14 = 0x2000;  // Orange only
     * *(uint32_t*)0x40020C14 = 0x4000;  // Red only
     * *(uint32_t*)0x40020C14 = 0x8000;  // Blue only
     */

    while (1) { /* LEDs stay on */ }

/* ============================================================================
 * DEMO 2: Lab 1 Task 3c - Simple Green LED On (Using Driver)
 * ============================================================================ */
#elif DEMO_MODE == 2

    printf("Lab 1 Task 3c: Simple Green LED\r\n");
    printf("Green LED should be ON using led_on() function\r\n\r\n");

    led_on(&led_green);
    printf("led_on(&led_green) called\r\n");

    while (1) { /* LED stays on */ }

/* ============================================================================
 * DEMO 3: Lab 1 Task 3d - Fast Blink (Why No Visible Blink?)
 * ============================================================================
 * 
 * This demonstrates why rapid on/off doesn't show visible blinking:
 * 1. At 168 MHz, the loop runs millions of times per second
 * 2. Human eye can only see changes up to ~60 Hz
 * 3. LED appears constantly on (but dim due to ~50% duty cycle)
 * 
 * Without 'volatile' on the port pointer, the compiler might optimize
 * this to just led_off(), making the LED appear always off!
 */
#elif DEMO_MODE == 3

    printf("Lab 1 Task 3d: Fast Blink Demo\r\n");
    printf("LED appears constantly on (or dim) because:\r\n");
    printf("  - Loop runs at MHz speed\r\n");
    printf("  - Eye can't perceive changes > 60Hz\r\n\r\n");

    while (1) {
        led_on(&led_green);
        led_off(&led_green);
        /* No delay - runs at full CPU speed! */
    }

/* ============================================================================
 * DEMO 4: Lab 1 Task 4 - Rotating LED Pattern (With Delay)
 * ============================================================================
 * 
 * This shows proper blinking with delays between state changes.
 * LEDs rotate clockwise around the board.
 */
#elif DEMO_MODE == 4

    printf("Lab 1 Task 4: Rotating LED Pattern\r\n");
    printf("LEDs rotate clockwise with 250ms delay\r\n\r\n");

    while (1) {
        led_on(&led_green);
        printf("Green ON\r\n");
        delay_msec(250);
        led_off(&led_green);

        led_on(&led_orange);
        printf("Orange ON\r\n");
        delay_msec(250);
        led_off(&led_orange);

        led_on(&led_red);
        printf("Red ON\r\n");
        delay_msec(250);
        led_off(&led_red);

        led_on(&led_blue);
        printf("Blue ON\r\n");
        delay_msec(250);
        led_off(&led_blue);
    }

/* ============================================================================
 * DEMO 5: Lab 1 Task 6 - Button-Controlled LED
 * ============================================================================
 * 
 * Press the blue button to cycle through LED states.
 * Demonstrates:
 * - Reading GPIO input (PA0)
 * - Edge detection (rising edge = button just pressed)
 * - Debouncing (delay after button press)
 */
#elif DEMO_MODE == 5

    printf("Lab 1 Task 6: Button-Controlled LEDs\r\n");
    printf("Press blue button to cycle through states\r\n\r\n");

    uint8_t led_state = 0;

    while (1) {
        /* Read button state from PA0 */
        /* GPIOA->IDR bit 0 = 1 when button pressed */
        
        /* Detect rising edge (button just pressed) */
        if ((GPIOA->IDR & 0x0001) && ((shadow & 0x0001) == 0)) {
            button_press = 1;
        }
        shadow = GPIOA->IDR;

        if (button_press) {
            button_press = 0;
            led_state = (led_state + 1) % 5;

            /* Turn all LEDs off first */
            led_off(&led_green);
            led_off(&led_orange);
            led_off(&led_red);
            led_off(&led_blue);

            switch (led_state) {
                case 0:
                    printf("State 0: All OFF\r\n");
                    break;
                case 1:
                    led_on(&led_green);
                    printf("State 1: GREEN\r\n");
                    break;
                case 2:
                    led_on(&led_orange);
                    printf("State 2: ORANGE\r\n");
                    break;
                case 3:
                    led_on(&led_red);
                    printf("State 3: RED\r\n");
                    break;
                case 4:
                    led_on(&led_blue);
                    printf("State 4: BLUE\r\n");
                    break;
            }

            /* Debounce delay */
            delay_msec(200);
        }
    }

/* ============================================================================
 * DEMO 6: Lab 2 - PWM Brightness Sweep
 * ============================================================================
 * 
 * All LEDs fade in and out using software PWM.
 * The pwm_driver_update() function is called at 10 kHz by TIM4 interrupt.
 */
#elif DEMO_MODE == 6

    printf("Lab 2: PWM Brightness Sweep\r\n");
    printf("LEDs fade in/out using software PWM\r\n\r\n");

    /* Initialize PWM driver */
    pwm_driver_init(&led_green, &led_orange, &led_red, &led_blue);
    
    /* Initialize timers */
    TMR4_Init_ISR();

    uint8_t brightness = 0;
    int8_t direction = 1;

    while (1) {
        /* Update all LEDs to same brightness */
        pwm_driver_set(GREEN, brightness);
        pwm_driver_set(ORANGE, brightness);
        pwm_driver_set(RED, brightness);
        pwm_driver_set(BLUE, brightness);

        /* Sweep brightness up and down */
        brightness += direction;
        if (brightness >= 100) {
            direction = -1;
            printf("Peak brightness reached, fading down\r\n");
        } else if (brightness == 0) {
            direction = 1;
            printf("Minimum brightness, fading up\r\n");
        }

        /* PWM refresh (called by timer ISR) */
        if (tmr4_flag) {
            tmr4_flag = 0;
            pwm_driver_update();
        }

        delay_msec(20);  /* ~50 brightness updates per second */
    }

/* ============================================================================
 * DEMO 7: Lab 4 - Tilt Display
 * ============================================================================
 * 
 * Tilt the board to light corresponding LEDs.
 * Uses the LIS3DSH 3-axis accelerometer via SPI.
 * 
 * Board orientation → LED response:
 * - Tilt toward Green LED → Green brighter
 * - Tilt toward Red LED → Red brighter
 * - Tilt toward Orange/Blue → Orange/Blue brighter
 */
#elif DEMO_MODE == 7

    printf("Lab 4: Tilt Display\r\n");
    printf("Tilt board to light corresponding LEDs\r\n\r\n");

    /* Initialize PWM, timers, and accelerometer */
    pwm_driver_init(&led_green, &led_orange, &led_red, &led_blue);
    TMR3_Init_ISR();
    TMR4_Init_ISR();

    if (AccInit()) {
        printf("Accelerometer initialized successfully!\r\n");
    } else {
        printf("ERROR: Accelerometer initialization failed!\r\n");
    }

    acc3_t acc_readings;
    uint16_t brightness;

    while (1) {
        /* Sample accelerometer at 32 Hz */
        if (tmr3_flag) {
            tmr3_flag = 0;
            AccRead(&acc_readings);

            /* X-axis controls Green/Red LEDs */
            if (acc_readings.x > 1000) {
                /* Tilted toward Red LED */
                brightness = (acc_readings.x * 100) / 11586;
                if (brightness > 100) brightness = 100;
                pwm_driver_set(RED, brightness);
                pwm_driver_set(GREEN, 0);
            } else if (acc_readings.x < -1000) {
                /* Tilted toward Green LED */
                brightness = (-acc_readings.x * 100) / 11586;
                if (brightness > 100) brightness = 100;
                pwm_driver_set(GREEN, brightness);
                pwm_driver_set(RED, 0);
            } else {
                /* Level in X direction */
                pwm_driver_set(GREEN, 0);
                pwm_driver_set(RED, 0);
            }

            /* Y-axis controls Orange/Blue LEDs */
            if (acc_readings.y > 1000) {
                brightness = (acc_readings.y * 100) / 11586;
                if (brightness > 100) brightness = 100;
                pwm_driver_set(ORANGE, brightness);
                pwm_driver_set(BLUE, 0);
            } else if (acc_readings.y < -1000) {
                brightness = (-acc_readings.y * 100) / 11586;
                if (brightness > 100) brightness = 100;
                pwm_driver_set(BLUE, brightness);
                pwm_driver_set(ORANGE, 0);
            } else {
                pwm_driver_set(ORANGE, 0);
                pwm_driver_set(BLUE, 0);
            }
        }

        /* PWM refresh */
        if (tmr4_flag) {
            tmr4_flag = 0;
            pwm_driver_update();
        }
    }

/* ============================================================================
 * DEMO 8: Lab 4 - Impact Detection
 * ============================================================================
 * 
 * Shake the board to trigger all LEDs!
 * Calculates acceleration magnitude: sqrt(x² + y² + z²)
 * When magnitude exceeds threshold, all LEDs flash.
 */
#elif DEMO_MODE == 8

    printf("Lab 4: Impact Detection\r\n");
    printf("Shake the board to trigger LEDs!\r\n\r\n");

    /* Initialize PWM, timers, and accelerometer */
    pwm_driver_init(&led_green, &led_orange, &led_red, &led_blue);
    TMR3_Init_ISR();
    TMR4_Init_ISR();

    if (AccInit()) {
        printf("Accelerometer initialized successfully!\r\n");
    } else {
        printf("ERROR: Accelerometer initialization failed!\r\n");
    }

    acc3_t acc_readings;
    uint32_t impact;
    uint8_t flash_counter = 0;

    while (1) {
        if (tmr3_flag) {
            tmr3_flag = 0;
            AccRead(&acc_readings);

            /* Calculate impact magnitude (squared) */
            impact = (uint32_t)acc_readings.x * acc_readings.x +
                     (uint32_t)acc_readings.y * acc_readings.y +
                     (uint32_t)acc_readings.z * acc_readings.z;

            /* Threshold for impact detection */
            /* Normal gravity ~16384² = 268M, threshold ~1G = 1073741824 */
            if (impact > ((uint32_t)0x01 << 30)) {
                printf("IMPACT DETECTED! Magnitude: %lu\r\n", impact);
                flash_counter = 20;  /* Flash for 20 PWM cycles */
            }

            /* Flash all LEDs when impact detected */
            if (flash_counter > 0) {
                pwm_driver_set(GREEN, 100);
                pwm_driver_set(ORANGE, 100);
                pwm_driver_set(RED, 100);
                pwm_driver_set(BLUE, 100);
                flash_counter--;
            } else {
                pwm_driver_set(GREEN, 0);
                pwm_driver_set(ORANGE, 0);
                pwm_driver_set(RED, 0);
                pwm_driver_set(BLUE, 0);
            }
        }

        /* PWM refresh */
        if (tmr4_flag) {
            tmr4_flag = 0;
            pwm_driver_update();
        }
    }

/* ============================================================================
 * DEMO 9: Full System - Button Cycles Through All Modes
 * ============================================================================
 * 
 * This is the complete smart-pallet prototype demo.
 * Press button to cycle: OFF → TILT → IMPACT → OFF...
 */
#elif DEMO_MODE == 9

    printf("Full System Demo\r\n");
    printf("Press button to cycle: OFF → TILT → IMPACT → OFF\r\n\r\n");

    /* Initialize everything */
    pwm_driver_init(&led_green, &led_orange, &led_red, &led_blue);
    pwm_driver_set(GREEN, 0);
    pwm_driver_set(ORANGE, 0);
    pwm_driver_set(RED, 0);
    pwm_driver_set(BLUE, 0);

    TMR3_Init_ISR();
    TMR4_Init_ISR();

    if (AccInit()) {
        printf("Accelerometer initialized successfully!\r\n");
    }

    /* State variables */
    static unsigned char state = DISPLAY_OFF;
    acc3_t acc_readings;
    uint16_t brightness = 0;
    uint32_t impact = 0;
    uint8_t impact_flag = 0;

    /* Circular buffer for vibration detection */
    acc3_t cir_buff[16];
    acc3_t* ptr_buff = cir_buff;
    uint8_t full_flag = 0;

    while (1) {
        /* Button edge detection */
        if ((GPIOA->IDR & 0x0001) && ((shadow & 0x0001) == 0)) {
            button_press = 1;
        }
        shadow = GPIOA->IDR;

        /* Accelerometer sampling at 32 Hz */
        if (tmr3_flag) {
            tmr3_flag = 0;
            AccRead(&acc_readings);

            /* Tilt display mode */
            if (state == DISPLAY_TILT) {
                /* X-axis */
                if (acc_readings.x > 1000) {
                    brightness = (acc_readings.x * 100) / 11586;
                    pwm_driver_set(RED, brightness > 100 ? 100 : brightness);
                    pwm_driver_set(GREEN, 0);
                } else if (acc_readings.x < -1000) {
                    brightness = (-acc_readings.x * 100) / 11586;
                    pwm_driver_set(GREEN, brightness > 100 ? 100 : brightness);
                    pwm_driver_set(RED, 0);
                } else {
                    pwm_driver_set(RED, 0);
                    pwm_driver_set(GREEN, 0);
                }

                /* Y-axis */
                if (acc_readings.y > 1000) {
                    brightness = (acc_readings.y * 100) / 11586;
                    pwm_driver_set(ORANGE, brightness > 100 ? 100 : brightness);
                    pwm_driver_set(BLUE, 0);
                } else if (acc_readings.y < -1000) {
                    brightness = (-acc_readings.y * 100) / 11586;
                    pwm_driver_set(BLUE, brightness > 100 ? 100 : brightness);
                    pwm_driver_set(ORANGE, 0);
                } else {
                    pwm_driver_set(ORANGE, 0);
                    pwm_driver_set(BLUE, 0);
                }
            }

            /* Impact detection */
            impact = (uint32_t)acc_readings.x * acc_readings.x +
                     (uint32_t)acc_readings.y * acc_readings.y +
                     (uint32_t)acc_readings.z * acc_readings.z;
            
            if (impact > ((uint32_t)0x01 << 30)) {
                impact_flag = 1;
                printf("Impact detected!\r\n");
            }

            /* State machine */
            switch (state) {
                case DISPLAY_OFF:
                    if (button_press) {
                        button_press = 0;
                        state = DISPLAY_TILT;
                        printf("Mode: TILT DISPLAY\r\n");
                    }
                    break;

                case DISPLAY_TILT:
                    if (button_press) {
                        button_press = 0;
                        state = DISPLAY_IMPACT;
                        pwm_driver_set(GREEN, 0);
                        pwm_driver_set(ORANGE, 0);
                        pwm_driver_set(RED, 0);
                        pwm_driver_set(BLUE, 0);
                        printf("Mode: IMPACT DETECTION\r\n");
                    }
                    break;

                case DISPLAY_IMPACT:
                    if (impact_flag) {
                        impact_flag = 0;
                        pwm_driver_set(GREEN, 100);
                        pwm_driver_set(ORANGE, 100);
                        pwm_driver_set(RED, 100);
                        pwm_driver_set(BLUE, 100);
                    }
                    if (button_press) {
                        button_press = 0;
                        state = DISPLAY_OFF;
                        pwm_driver_set(GREEN, 0);
                        pwm_driver_set(ORANGE, 0);
                        pwm_driver_set(RED, 0);
                        pwm_driver_set(BLUE, 0);
                        printf("Mode: OFF\r\n");
                    }
                    break;
            }
        }

        /* PWM refresh at 10 kHz */
        if (tmr4_flag) {
            tmr4_flag = 0;
            pwm_driver_update();
        }
    }

#else
    #error "Invalid DEMO_MODE. Please select 1-9."
#endif

    /* Should never reach here */
    return 0;
}

/**
 * @brief Error Handler
 */
void Error_Handler(void)
{
    /* Turn on red LED to indicate error */
    *(uint32_t*)0x40020C14 |= (1 << RED);
    while (1) { }
}
