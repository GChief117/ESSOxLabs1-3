# ESS Lab 1: LED Driver for STM32F4 Discovery Board

## Project Overview

This project implements a complete LED driver for the STM32F4 Discovery board as part of the Oxford Embedded Software and Systems (ESS) course Lab 1.

### Board Information
- **MCU**: STM32F407VGT6 (ARM Cortex-M4F, 168 MHz, 1MB Flash, 192KB RAM)
- **LEDs**: 4 user LEDs on Port D
  - Green: PD12
  - Orange: PD13
  - Red: PD14
  - Blue: PD15
- **Button**: Blue user button on PA0

## Project Structure

```
ess_lab1/
├── Core/
│   ├── Inc/
│   │   ├── led_driver.h      # LED driver API (Task 4-5)
│   │   ├── ess_helper.h      # Board initialization
│   │   └── main.h            # Main application header
│   ├── Src/
│   │   ├── main.c            # Demo functions for all tasks
│   │   ├── led_driver.c      # LED driver implementation
│   │   └── ess_helper.c      # Clock & GPIO initialization
│   └── Startup/
│       └── startup_stm32f407vgtx.s
├── Drivers/
│   ├── CMSIS/                # ARM CMSIS headers
│   └── STM32F4xx_HAL_Driver/ # ST HAL library
├── .project                  # STM32CubeIDE project file
├── ess_skeleton.ioc          # STM32CubeMX configuration
└── README.md                 # This file
```

## Lab Tasks Implementation

### Task 2: Direct Register Access
- `demo_task2a_all_leds_on()` - Turn on all LEDs
- `demo_task2b_green_only()` - Green LED only
- `demo_task2c_orange_only()` - Orange LED only
- `demo_task2d_red_only()` - Red LED only
- `demo_task2e_blue_only()` - Blue LED only
- `demo_task2f_identify_leds()` - Sequential LED identification

### Task 3: Simple LED Driver
- `demo_task3c_basic_on_off()` - Basic on/off test
- `demo_task3d_fast_toggle()` - Fast toggle (volatile importance)
- `demo_task3e_visible_blink()` - Visible 1Hz blinking

### Task 4: Generic LED Driver with ADT
- `demo_task4a_all_leds_sequence()` - Sequential pattern
- `demo_task4b_chase_pattern()` - Rotating chase
- `demo_task4c_binary_counter()` - 4-bit binary counter display

### Task 5: Refactoring
The LED driver is now modularized into:
- `led_driver.h` - API definitions
- `led_driver.c` - Implementation

### Task 6: Button Input
- `demo_task6a_button_led()` - Button controls LED
- `demo_task6b_button_toggle()` - Button toggles state
- `demo_task6c_button_cycle()` - Button cycles through colors
- `demo_task6d_button_start_stop()` - Button starts/stops blinking

### Task 7: Debug/Testing
- `demo_task7_printf_test()` - Printf via SWV
- `demo_task7b_full_test()` - Comprehensive verification

## How to Use

### Importing into STM32CubeIDE

1. Open STM32CubeIDE
2. File → Import → General → Existing Projects into Workspace
3. Select the `ess_lab1` folder
4. Click Finish

### Running a Demo

1. Open `Core/Src/main.c`
2. Find the `main()` function at the bottom
3. **Uncomment ONE demo function** to test
4. Build (Ctrl+B or hammer icon)
5. Run (green play button) or Debug (bug icon)

### Example: Testing Task 4a

```c
int main(void)
{
    HAL_Init();
    ess_helper_init();
    
    // Uncomment this line:
    demo_task4a_all_leds_sequence();
    
    while (1) { }
}
```

### Enabling printf Debugging (Task 7)

1. Right-click project → Debug As → Debug Configurations
2. Select your debug config → Debugger tab
3. Check "Serial Wire Viewer (SWV)" Enable
4. Set Core Clock: 168 MHz
5. Click Apply & Close
6. Start debugging (F11)
7. Window → Show View → SWV → SWV ITM Data Console
8. Click the configure button (wrench icon), enable Port 0
9. Click "Start Trace" button
10. Click Resume (F8) to run

## Key Concepts

### Hardware Abstraction Layer (HAL)
The LED driver provides a HAL that:
- Decouples application code from hardware details
- Allows the same code to work with different LEDs
- Uses an Abstract Data Type (ADT) for LED state

### Register Access
- **ODR (0x40020C14)**: Output Data Register - read/write LED states
- **BSRR (0x40020C18)**: Bit Set/Reset Register - atomic operations
- **IDR (0x40020010)**: Input Data Register - read button state

### Volatile Keyword
Critical for embedded systems! Prevents compiler from:
- Optimizing away delay loops
- Caching register values that hardware can change

## LED Driver API

```c
// Initialize an LED
void led_init(LED_t *led, volatile uint32_t *odr, 
              volatile uint32_t *bsrr, uint32_t pin);

// Control functions
void led_on(LED_t *led);
void led_off(LED_t *led);
void led_toggle(LED_t *led);
uint32_t led_is_on(LED_t *led);

// Utilities
void delay_msec(uint32_t msec);
uint32_t button_is_pressed(void);
```

## Troubleshooting

### LEDs not working
1. Check USB connection (ST-LINK should have a solid or blinking red LED)
2. Verify correct board selected in debug configuration
3. Check that `ess_helper_init()` is called before using LEDs

### Build errors
1. Clean project: Project → Clean
2. Rebuild: Project → Build All
3. Check all files are present in project

### printf not showing
1. Verify SWV is enabled in debug configuration
2. Check Core Clock is set to 168 MHz
3. Ensure "Start Trace" is clicked before running
4. Port 0 must be enabled in ITM console configuration

## Author

Gunnar Nelson - Oxford ESS Course, January 2026
