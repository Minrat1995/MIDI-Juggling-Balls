/**
 * Sensors Module - Multi-sensor interface for juggling ball
 * 
 * Implemented sensors:
 * - LSM6DSOX: 6-axis IMU (accelerometer + gyroscope)
 * - LIS3MDL: 3-axis magnetometer
 * - H3LIS331DL: High-G accelerometer (±400g) with FSR data encoded in lower bits
 * - BMP581: Barometric pressure sensor
 * - 4× FSR: Force-sensitive resistors (encoded into H3LIS331 lower bits)
 * 
 * FSR Encoding (12 bits total):
 * - H3LIS X-axis [3:0]: 4-bit max squeeze intensity (0-15)
 * - H3LIS Y-axis [3:0]: 4-bit FSR contact pattern (which FSRs active)
 * - H3LIS Z-axis [3:0]: 4-bit flags (reserved for future features)
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// SENSOR DATA STRUCTURES
// ============================================================================

/**
 * Combined LSM6DSOX + LIS3MDL 9-axis IMU data (18 bytes)
 * All values are raw 16-bit signed integers from sensors
 */
typedef struct {
    int16_t accel[3];      // X, Y, Z accelerometer (LSM6DSOX)
    int16_t gyro[3];       // X, Y, Z gyroscope (LSM6DSOX)
    int16_t mag[3];        // X, Y, Z magnetometer (LIS3MDL)
} __attribute__((packed)) lsm6_lis3_data_t;

/**
 * Complete sensor reading structure (27 bytes total)
 * 
 * NOTE: Packet size reduced from 31 to 27 bytes by encoding FSR data
 * into unused H3LIS331 lower bits
 */
typedef struct {
    // H3LIS331 High-G Accelerometer + FSR Data (6 bytes)
    // Each axis: [15:4] = 12-bit acceleration, [3:0] = FSR/flags data
    int16_t h3lis_x_fsr_level;     // X-accel + squeeze intensity (0-15)
    int16_t h3lis_y_fsr_pattern;   // Y-accel + FSR contact pattern (4 bits)
    int16_t h3lis_z_flags;         // Z-accel + reserved flags (4 bits)
    
    lsm6_lis3_data_t imu;          // 9-axis IMU data (18 bytes)
    uint8_t pressure[3];           // BMP581 pressure, 24-bit (3 bytes)
} __attribute__((packed)) sensor_data_t;

// Compile-time verification
_Static_assert(sizeof(sensor_data_t) == 27, "sensor_data_t must be exactly 27 bytes");

// ============================================================================
// SENSOR STATUS CODES
// ============================================================================

typedef enum {
    SENSOR_OK = 0,
    SENSOR_ERR_I2C_INIT,
    SENSOR_ERR_LSM6_NOT_FOUND,
    SENSOR_ERR_LIS3_NOT_FOUND,
    SENSOR_ERR_LSM6_CONFIG,
    SENSOR_ERR_LIS3_CONFIG,
    SENSOR_ERR_H3LIS_NOT_FOUND,
    SENSOR_ERR_H3LIS_CONFIG,
    SENSOR_ERR_BMP_NOT_FOUND,
    SENSOR_ERR_BMP_CONFIG
} sensor_status_t;

// ============================================================================
// SENSOR STATUS BITMASK
// ============================================================================

#define SENSOR_LSM6_OK      (1 << 0)
#define SENSOR_LIS3_OK      (1 << 1)
#define SENSOR_H3LIS_OK     (1 << 2)
#define SENSOR_BMP_OK       (1 << 3)
#define SENSOR_FSR1_OK      (1 << 4)
#define SENSOR_FSR2_OK      (1 << 5)
#define SENSOR_FSR3_OK      (1 << 6)
#define SENSOR_FSR4_OK      (1 << 7)

#define ALL_SENSORS_OK      0x0F

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

/**
 * Initialize I2C bus and all sensors
 * 
 * Configures:
 * - LSM6DSOX: 416 Hz ODR, ±16g accel, ±2000dps gyro
 * - LIS3MDL: 80 Hz ODR, ±4 gauss, continuous mode
 * - H3LIS331: 400 Hz ODR, ±400g range, BDU enabled
 * - BMP581: 200 Hz ODR, OSR x4, normal mode
 * - FSR ADC: 10-bit resolution, 4 channels
 * 
 * @return true on success, false if critical sensors (LSM6/LIS3) fail
 */
bool sensors_init(void);

/**
 * Get detected I2C addresses for debugging
 */
void sensors_get_addresses(uint8_t *lsm6, uint8_t *lis3, uint8_t *h3lis, uint8_t *bmp);

/**
 * Read all sensors and encode data
 * 
 * Reads:
 * - LSM6DSOX: Accelerometer and gyroscope
 * - LIS3MDL: Magnetometer
 * - H3LIS331: High-G accelerometer
 * - BMP581: Barometric pressure
 * - FSRs: 4 force sensors (encoded into H3LIS331 lower bits)
 * 
 * FSR encoding:
 * - Finds max FSR value, scales to 0-15 (stored in X-axis lower 4 bits)
 * - Generates contact pattern showing which FSRs active (Y-axis lower 4 bits)
 * - Reserved flags for future features (Z-axis lower 4 bits)
 * 
 * @param data Pointer to sensor_data_t structure to populate
 */
void sensors_read(sensor_data_t *data);

/**
 * Test sensor communication
 * 
 * @return true if all initialized sensors respond correctly
 */
bool sensors_test(void);

/**
 * Get runtime sensor status bitmask
 * 
 * @return Bitmask where 1 = sensor working, 0 = sensor failed
 */
uint8_t sensors_get_status_bitmask(void);

/**
 * Get total I2C error count
 * 
 * @return Number of I2C communication errors since boot
 */
uint16_t sensors_get_i2c_error_count(void);

/**
 * Read temperature from BMP581 sensor
 * 
 * @return Temperature in 0.01°C units (e.g., 2550 = 25.50°C), or 0 if not available
 */
int16_t sensors_read_temperature(void);

#endif // SENSORS_H