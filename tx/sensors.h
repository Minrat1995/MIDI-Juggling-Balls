/**
 * Sensors Module - TX only
 *
 * Declares sensor init, read, and status functions.
 * Data structures (sensor_data_t etc.) are defined in packet_spec.h
 * which is included here and must stay in sync with rx/packet_spec.h.
 *
 * Sensor configuration:
 *   LSM6DSOX: 416Hz ODR, ±16g accel, ±500dps gyro, BDU enabled
 *   LIS3MDL:  80Hz ODR, ±4 gauss, continuous mode, BDU enabled
 *   H3LIS331: 400Hz ODR, ±400g, BDU enabled
 *   BMP581:   ~218Hz ODR, OSR x4, normal mode, on-chip compensation
 *   FSR:      4 channels on SAADC Ch1-4, disabled until Phase 3 wiring
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>
#include "packet_spec.h"

// ============================================================================
// SENSOR STATUS BITMASK
// ============================================================================

#define SENSOR_LSM6_OK      (1 << 0)
#define SENSOR_LIS3_OK      (1 << 1)
#define SENSOR_H3LIS_OK     (1 << 2)
#define SENSOR_BMP_OK       (1 << 3)
// Bits 4-7: FSR1-4, always 0 until Phase 3 wiring
#define SENSOR_FSR1_OK      (1 << 4)
#define SENSOR_FSR2_OK      (1 << 5)
#define SENSOR_FSR3_OK      (1 << 6)
#define SENSOR_FSR4_OK      (1 << 7)

// All non-FSR sensors working
#define ALL_CORE_SENSORS_OK 0x0F

// ============================================================================
// TEMPERATURE SENTINEL
// ============================================================================

/**
 * Returned by sensors_read_temperature() when BMP581 is unavailable.
 * Value -32768 corresponds to -327.68 C, which is physically impossible.
 * Callers must check for this value before using the result.
 *
 * Example:
 *   int16_t temp = sensors_read_temperature();
 *   if (temp != SENSORS_TEMP_UNAVAILABLE) { use_temp(temp); }
 */
#define SENSORS_TEMP_UNAVAILABLE  ((int16_t)(-32768))

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

/**
 * Initialize I2C bus and all sensors.
 * LSM6DSOX and LIS3MDL are required — returns false if either is absent.
 * H3LIS331 and BMP581 are optional — warnings printed but init continues.
 *
 * @return true on success, false if LSM6 or LIS3 fails
 */
bool sensors_init(void);

/**
 * Get detected I2C addresses (for RTT validation at startup).
 */
void sensors_get_addresses(uint8_t *lsm6, uint8_t *lis3, uint8_t *h3lis, uint8_t *bmp);

/**
 * Read all sensors into data structure.
 * FSR data packed into H3LIS lower 4 bits (zeros until wired).
 *
 * @param data Output structure to populate
 */
void sensors_read(sensor_data_t *data);

/**
 * Verify sensor communication (WHO_AM_I checks).
 *
 * @return true if LSM6 and LIS3 respond correctly
 */
bool sensors_test(void);

/**
 * Runtime sensor status bitmask.
 * Updated on every sensors_read() call.
 *
 * @return Bitmask: 1=working, 0=failed. Bits 4-7 always 0 (FSR not wired).
 */
uint8_t sensors_get_status_bitmask(void);

/**
 * Total I2C error count since boot.
 */
uint16_t sensors_get_i2c_error_count(void);

/**
 * Read BMP581 temperature.
 *
 * Returns temperature in 0.01 C units (2550 = 25.50 C).
 * Returns SENSORS_TEMP_UNAVAILABLE if BMP581 is absent or read fails.
 * Caller must check for SENSORS_TEMP_UNAVAILABLE before using the result.
 */
int16_t sensors_read_temperature(void);

#endif // SENSORS_H
