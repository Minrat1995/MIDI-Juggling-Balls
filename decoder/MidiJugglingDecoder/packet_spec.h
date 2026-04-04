/**
 * Packet Specification - Shared contract between TX, RX, and PC decoder.
 *
 * Copied from rx/packet_spec.h. Only change from the C original:
 *   _Static_assert -> static_assert  (C11 -> C++11)
 *
 * If you update the on-air format, update tx/, rx/, and this copy.
 *
 * Packet: 86 bytes at 250Hz (4ms intervals)
 *
 * Header (5 bytes):
 *   ball_id   uint8_t   Ball identifier 1-8
 *   sequence  uint16_t  Wraps at 65535
 *   timestamp uint16_t  Milliseconds since TX boot, wraps at 65.5s
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
 *   accel[3]             int16_t  LSM6DSOX +/-16g
 *   gyro[3]              int16_t  LSM6DSOX +/-500dps
 *   mag[3]               int16_t  LIS3MDL +/-4 gauss
 *   pressure[3]          uint8_t  BMP581 24-bit LE, Pa = raw/64
 *
 * Scaling:
 *   Accel:    raw / 32768.0 * 16.0  = g
 *   Gyro:     raw / 32768.0 * 500.0 = dps
 *   Mag:      raw / 6842.0          = gauss (+/-4 gauss range)
 *   Pressure: uint32 = p[0] | (p[1]<<8) | (p[2]<<16); pa = uint32 / 64
 */

#pragma once

#include <cstdint>
#include <cstdbool>

// ============================================================================
// RADIO CONSTANTS
// ============================================================================

constexpr int     RF_CHANNEL          = 40;
constexpr uint32_t RADIO_BASE_ADDR    = 0x12345678;
constexpr uint8_t  RADIO_PREFIX_ADDR  = 0xAB;
constexpr int     PACKET_PAYLOAD_SIZE = 86;

constexpr uint8_t PACKET_TYPE_DATA   = 0x00;
constexpr uint8_t PACKET_TYPE_STATUS = 0xFF;

// ============================================================================
// SENSOR DATA STRUCTURES
// ============================================================================

#pragma pack(push, 1)

/**
 * Combined 9-axis IMU (18 bytes)
 */
struct imu_data_t {
    int16_t accel[3];   // LSM6DSOX X/Y/Z, +/-16g, raw
    int16_t gyro[3];    // LSM6DSOX X/Y/Z, +/-500dps, raw
    int16_t mag[3];     // LIS3MDL  X/Y/Z, +/-4 gauss, raw
};

/**
 * One sensor sample (27 bytes)
 *
 * H3LIS331 data has FSR values packed into the 4 LSBs of each axis word.
 * TX packs using: packed = (h3lis_raw & 0xFFF0) | (fsr_nibble & 0x0F)
 * PC decodes using: extract_h3lis_axis() defined below.
 */
struct sensor_data_t {
    int16_t    h3lis_x_fsr_level;    // [15:4]=H3LIS X, [3:0]=peak FSR intensity 0-15
    int16_t    h3lis_y_fsr_pattern;  // [15:4]=H3LIS Y, [3:0]=FSR contact bitmask
    int16_t    h3lis_z_flags;        // [15:4]=H3LIS Z, [3:0]=reserved
    imu_data_t imu;                  // 18 bytes
    uint8_t    pressure[3];          // BMP581 24-bit LE raw, Pa = raw/64
};

static_assert(sizeof(sensor_data_t) == 27, "sensor_data_t must be 27 bytes");

/**
 * On-air data packet (86 bytes)
 * t1 = t-4ms, t2 = t-8ms at 250Hz TX rate.
 */
struct radio_packet_t {
    uint8_t       ball_id;    // 1-8
    uint16_t      sequence;   // wraps at 65535
    uint16_t      timestamp;  // TX milliseconds, wraps at 65.5s
    sensor_data_t data_t0;    // current sample
    sensor_data_t data_t1;    // t-4ms
    sensor_data_t data_t2;    // t-8ms
};

static_assert(sizeof(radio_packet_t) == 86, "radio_packet_t must be 86 bytes");

/**
 * Status packet (26 bytes) — TX -> RTT only, not forwarded over USB.
 * Included here for completeness. The PC decoder will never see these.
 */
struct status_packet_t {
    uint8_t  ball_id;
    uint16_t sequence;
    uint16_t timestamp;
    uint8_t  packet_type;        // always PACKET_TYPE_STATUS (0xFF)
    int16_t  temperature;        // 0.01 C units, from BMP581
    uint16_t battery_voltage;    // millivolts
    uint8_t  battery_percent;    // 0-100
    uint8_t  sensor_health;      // bitmask
    uint32_t total_packets_sent;
    uint32_t uptime_seconds;
    uint16_t radio_timeouts;
    uint16_t i2c_errors;
    uint8_t  reserved;
    uint8_t  checksum;           // XOR of all preceding bytes
};

static_assert(sizeof(status_packet_t) == 26, "status_packet_t must be 26 bytes");

#pragma pack(pop)

// ============================================================================
// H3LIS DECODE HELPER
// ============================================================================

/**
 * Extract the signed 12-bit H3LIS axis value from a packed int16_t.
 *
 * The sensor stores 12-bit data in bits [15:4]; bits [3:0] carry FSR data.
 * Shifting as unsigned (cast to uint16_t first) is well-defined in C++.
 * Sign extension from bit 11 is then applied explicitly.
 *
 * Replicated from packet_spec.h on the embedded side — must stay in sync.
 *
 * @param packed  The raw int16_t field from sensor_data_t
 * @return        Signed 12-bit axis value in a full int16_t
 */
inline int16_t extract_h3lis_axis(int16_t packed)
{
    uint16_t shifted = static_cast<uint16_t>(packed) >> 4;
    return (shifted & 0x0800u)
        ? static_cast<int16_t>(shifted | 0xF000u)   // sign-extend from bit 11
        : static_cast<int16_t>(shifted);
}

// ============================================================================
// SCALING HELPERS
// ============================================================================

inline float accel_to_g(int16_t raw)    { return raw / 32768.0f * 16.0f; }
inline float gyro_to_dps(int16_t raw)   { return raw / 32768.0f * 500.0f; }
inline float mag_to_gauss(int16_t raw)  { return raw / 6842.0f; }
inline float h3lis_to_g(int16_t packed) { return extract_h3lis_axis(packed) / 2048.0f * 400.0f; }

inline float pressure_to_pa(const uint8_t p[3])
{
    uint32_t raw = static_cast<uint32_t>(p[0])
                 | (static_cast<uint32_t>(p[1]) << 8)
                 | (static_cast<uint32_t>(p[2]) << 16);
    return static_cast<float>(raw) / 64.0f;
}
