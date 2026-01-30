/**
 * @file acc_driver.h
 * @brief 3-Axis Accelerometer Driver for LIS3DSH (Labs 3-4)
 *
 * Created on: 30 Jan 2022
 * Author: zhuangzhuangdai
 * Enhanced with comments for ESS Lab study
 *
 * ============================================================================
 * ACCELEROMETER OVERVIEW:
 * ============================================================================
 *
 * The STM32F4 Discovery board has a LIS3DSH 3-axis MEMS accelerometer.
 * It measures acceleration in X, Y, and Z directions.
 *
 * COORDINATE SYSTEM (board flat, USB connector toward you):
 *   +X: Toward Red LED
 *   +Y: Toward Orange LED
 *   +Z: Upward (toward you when board is flat)
 *
 * OUTPUT FORMAT:
 *   - 16-bit signed integer per axis (-32768 to +32767)
 *   - At ±2g range: ~16384 counts per g (gravity)
 *   - Flat board: X≈0, Y≈0, Z≈+16384 (1g pointing up)
 *
 * ============================================================================
 * SPI COMMUNICATION (Lab 3):
 * ============================================================================
 *
 * The accelerometer communicates via SPI (Serial Peripheral Interface):
 *   - MOSI (PA7): Master Out, Slave In
 *   - MISO (PA6): Master In, Slave Out
 *   - SCK  (PA5): Serial Clock
 *   - CS   (PE3): Chip Select (active LOW)
 *
 * READ OPERATION:
 *   1. Pull CS LOW
 *   2. Send address byte with bit 7 SET (0x80 | address)
 *   3. Receive data byte(s)
 *   4. Pull CS HIGH
 *
 * WRITE OPERATION:
 *   1. Pull CS LOW
 *   2. Send address byte (bit 7 = 0)
 *   3. Send data byte(s)
 *   4. Pull CS HIGH
 *
 * ============================================================================
 * APPLICATIONS (Lab 4):
 * ============================================================================
 *
 * 1. TILT DETECTION:
 *    - When flat: Z ≈ +16384 (1g)
 *    - Tilt toward X: X increases, Z decreases
 *    - Can calculate tilt angle: θ = atan2(X, Z)
 *
 * 2. IMPACT DETECTION:
 *    - Calculate magnitude: sqrt(X² + Y² + Z²)
 *    - Normal = 16384 (1g)
 *    - Impact causes spike >> 16384
 *
 * 3. VIBRATION DETECTION:
 *    - Collect samples over time
 *    - Calculate variance/standard deviation
 *    - High variance = vibration
 *
 * ============================================================================
 */

#ifndef INC_ACC_DRIVER_H_
#define INC_ACC_DRIVER_H_

#include "spi_driver.h"

/**
 * @brief 3-axis acceleration data structure
 *
 * Each axis is a 16-bit signed integer:
 *   - Range: -32768 to +32767
 *   - At ±2g: ~16384 counts/g
 *   - Flat board: z ≈ +16384
 */
typedef struct {
    int16_t x;   /**< X-axis acceleration (toward Red LED = positive) */
    int16_t y;   /**< Y-axis acceleration (toward Orange LED = positive) */
    int16_t z;   /**< Z-axis acceleration (upward = positive) */
} acc3_t;

/* ============================================================================
 * Accelerometer API
 * ============================================================================ */

/**
 * @brief Initialize the accelerometer
 *
 * Configures:
 *   - SPI1 peripheral
 *   - Accelerometer control registers
 *   - Output data rate and range
 *
 * @return 1 if successful (WHO_AM_I = 0x3F), 0 if failed
 *
 * @example
 *   if (AccInit()) {
 *       printf("Accelerometer OK!\r\n");
 *   } else {
 *       printf("ERROR: Accelerometer not responding\r\n");
 *   }
 */
uint8_t AccInit(void);

/**
 * @brief Read current acceleration values
 *
 * Reads X, Y, Z acceleration data from accelerometer registers.
 * Each value is a 16-bit signed integer.
 *
 * @param reading   Pointer to acc3_t structure to store results
 *
 * @example
 *   acc3_t data;
 *   AccRead(&data);
 *   printf("X=%d, Y=%d, Z=%d\r\n", data.x, data.y, data.z);
 *   
 *   // Calculate magnitude (for impact detection):
 *   uint32_t mag = data.x*data.x + data.y*data.y + data.z*data.z;
 */
void AccRead(acc3_t * reading);

#endif /* INC_ACC_DRIVER_H_ */
