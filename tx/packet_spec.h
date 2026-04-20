/**
 * Packet Specification - Shared contract between TX and RX
 *
 * THIS FILE IS IDENTICAL IN tx/ AND rx/ DIRECTORIES.
 * If you change one copy you must change the other.
 * It defines the on-air packet layout, radio constants, and USB framing constants.
 * Neither TX nor RX should define these independently.
 *
 * Packet: 86 bytes at 250Hz (4ms intervals)
 *
 * Header (5 bytes):
 *   ball_id   uint8_t   Ball identifier 1-8
 *   sequence  uint16_t  Wraps at 65535
 *   timestamp uint16_t  RTC ticks since TX boot, wraps at ~65535 ticks (~66.0s)
 *                       NOTE: ticks, not true milliseconds. RTC1 runs at 32768/33
 *                       = 992.97 Hz; each tick = ~1.007ms. For gap-fill interpolation
 *                       treat as ticks. Wrap is ~66.0s, not 65.5s.
 *
 * Data (81 bytes):
 *   data_t0   sensor_data_t (27 bytes)  Current sample
 *   data_t1   sensor_data_t (27 bytes)  t-4ms
 *   data_t2   sensor_data_t (27 bytes)  t-8ms
 *
 * sensor_data_t (27 bytes):
 *   h3lis_x_fsr_level    int16_t  [15:4]=H3LIS X accel, [3:0]=FSR intensity 0-15
 *   h3lis_y_fsr_pattern  int16_t  [15:4]=H3LIS Y accel, [3:0]=FSR contact pattern
 *   h3lis_z_flags        int16_t  [15:4]=H3LIS Z accel, [3:0]=reserved
 *   accel[3]             int16_t  LSM6DSOX ±16g
 *   gyro[3]              int16_t  LSM6DSOX ±500dps
 *   mag[3]               int16_t  LIS3MDL ±4 gauss
 *   pressure[3]          uint8_t  BMP581 24-bit LE, Pa = raw/64
 *
 * Gyro scaling: raw / 32768 * 500 = dps
 * Accel scaling: raw / 32768 * 16 = g
 * Pressure: uint32 = pressure[0] | (pressure[1]<<8) | (pressure[2]<<16); pa = uint32 / 64
 * Temperature (status packet only): raw/65536 = degrees C; stored as int16 in 0.01C units
 *
 * FSR packing note:
 *   H3LIS331 outputs 12-bit values left-aligned in a 16-bit register (bits [15:4]).
 *   Bits [3:0] are always zero from the sensor hardware.
 *   We use those spare lower 4 bits for FSR data without losing any sensor precision.
 *   Limitation: only aggregate peak FSR intensity and per-FSR contact bitmask
 *   are stored, not per-FSR intensity. This matches the OSC output design.
 */

#ifndef PACKET_SPEC_H
#define PACKET_SPEC_H

#include <stdint.h>

// ============================================================================
// RADIO CONSTANTS (must match on both TX and RX)
// ============================================================================

#define RF_CHANNEL              40          // 2440 MHz
#define RADIO_BASE_ADDR         0x12345678
#define RADIO_PREFIX_ADDR       0xAB
#define CRC_POLYNOMIAL          0x00065B    // Nordic nRF proprietary radio CRC-24
                                            // NOTE: this is NOT IBM CRC-24 (0x864CFB).
                                            // Using the wrong polynomial for any
                                            // independent CRC implementation will
                                            // produce no matches.
#define CRC_INIT_VALUE          0x555555
#define PACKET_PAYLOAD_SIZE     86          // Total bytes on air

// Packet type field values
#define PACKET_TYPE_DATA        0x00
#define PACKET_TYPE_STATUS      0xFF

// ============================================================================
// USB FRAMING CONSTANTS
//
// These define the wire format between RX firmware (usb_serial.c) and the
// PC decoder (SerialReader.cpp). All three files must use these definitions.
// Do not redefine them locally.
//
// Frame layout (89 bytes):
//   [0]      USB_SYNC_BYTE_0 (0xAA)
//   [1]      USB_SYNC_BYTE_1 (0x55)
//   [2-87]   radio_packet_t payload (PACKET_PAYLOAD_SIZE = 86 bytes)
//   [88]     XOR checksum of bytes [2-87]
// ============================================================================

#define USB_SYNC_BYTE_0     0xAA
#define USB_SYNC_BYTE_1     0x55
#define USB_FRAME_SIZE      (2 + PACKET_PAYLOAD_SIZE + 1)   // 89 bytes

// ============================================================================
// SENSOR DATA STRUCTURES
// ============================================================================

/**
 * Combined 9-axis IMU (18 bytes)
 * Separate from H3LIS331 which is packed with FSR data.
 */
#pragma pack(push, 1)
typedef struct {
    int16_t accel[3];   // LSM6DSOX X/Y/Z, ±16g, raw
    int16_t gyro[3];    // LSM6DSOX X/Y/Z, ±500dps, raw
    int16_t mag[3];     // LIS3MDL  X/Y/Z, ±4 gauss, raw
} imu_data_t;
#pragma pack(pop)

/**
 * One sensor sample (27 bytes)
 *
 * H3LIS331 data has FSR values packed into the 4 LSBs of each axis word.
 * The H3LIS331 outputs 12-bit values left-aligned in a 16-bit word
 * (actual data in bits [15:4], bits [3:0] always zero from hardware).
 * We use those spare lower 4 bits for FSR data without losing sensor precision.
 *
 * TX packs using: packed = (h3lis_raw & 0xFFF0) | (fsr_nibble & 0x0F)
 * RX decodes using: extract_h3lis_axis() defined below.
 */
#pragma pack(push, 1)
typedef struct {
    int16_t    h3lis_x_fsr_level;    // [15:4]=H3LIS X, [3:0]=peak FSR intensity 0-15
    int16_t    h3lis_y_fsr_pattern;  // [15:4]=H3LIS Y, [3:0]=FSR contact bitmask
    int16_t    h3lis_z_flags;        // [15:4]=H3LIS Z, [3:0]=reserved
    imu_data_t imu;                  // 18 bytes
    uint8_t    pressure[3];          // BMP581 24-bit LE raw, Pa = raw/64
} sensor_data_t;
#pragma pack(pop)

_Static_assert(sizeof(sensor_data_t) == 27, "sensor_data_t must be 27 bytes");

// ============================================================================
// RADIO PACKET STRUCTURE
// ============================================================================

/**
 * On-air data packet (86 bytes)
 * Layout must be byte-identical on TX and RX.
 * t1 = t-4ms, t2 = t-8ms at 250Hz TX rate.
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t       ball_id;    // 1-8
    uint16_t      sequence;   // wraps at 65535
    uint16_t      timestamp;  // RTC ticks since TX boot; wraps at ~65535 ticks (~66.0s).
                              // Each tick = ~1.007ms (RTC1 at 992.97 Hz). NOT true ms.
    sensor_data_t data_t0;    // current sample
    sensor_data_t data_t1;    // t-4ms
    sensor_data_t data_t2;    // t-8ms
} radio_packet_t;
#pragma pack(pop)

_Static_assert(sizeof(radio_packet_t) == 86, "radio_packet_t must be 86 bytes");

// ============================================================================
// STATUS PACKET STRUCTURE (TX -> RTT only, not forwarded over USB)
// ============================================================================

/**
 * Health/diagnostics packet (26 bytes), sent every 2 minutes via RTT.
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t  ball_id;
    uint16_t sequence;
    uint16_t timestamp;
    uint8_t  packet_type;        // always PACKET_TYPE_STATUS (0xFF)
    int16_t  temperature;        // 0.01 C units, from BMP581
    uint16_t battery_voltage;    // millivolts
    uint8_t  battery_percent;    // 0-100
    uint8_t  sensor_health;      // bitmask (see SENSOR_xxx_OK in sensors.h)
    uint32_t total_packets_sent;
    uint32_t uptime_seconds;
    uint16_t radio_timeouts;
    uint16_t i2c_errors;
    uint8_t  reserved;
    uint8_t  checksum;           // XOR of all preceding bytes
} status_packet_t;
#pragma pack(pop)

_Static_assert(sizeof(status_packet_t) == 26, "status_packet_t must be 26 bytes");

// ============================================================================
// H3LIS DECODE HELPER
// ============================================================================

/**
 * Extract the signed 12-bit H3LIS axis value from a packed int16_t.
 *
 * The sensor stores 12-bit data in bits [15:4]; bits [3:0] carry FSR data.
 * Shifting as unsigned (cast to uint16_t first) is well-defined in C.
 * Sign extension from bit 11 is then applied explicitly.
 *
 * This function is used on both TX (debug display) and RX (decode).
 * It is also the correct implementation to replicate in the C++ decoder.
 *
 * @param packed  The raw int16_t field from sensor_data_t
 * @return        Signed 12-bit axis value in a full int16_t
 */
static inline int16_t extract_h3lis_axis(int16_t packed)
{
    uint16_t shifted = (uint16_t)packed >> 4;   // unsigned right shift: well-defined
    return (shifted & 0x0800u)
        ? (int16_t)(shifted | 0xF000u)          // sign-extend from bit 11
        : (int16_t)shifted;
}

#endif // PACKET_SPEC_H