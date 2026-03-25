/**
 * Juggling Ball Wireless Transmitter
 * 
 * Transmits 9-axis IMU data at 125Hz (8ms intervals) using 2.4GHz custom protocol
 * Status packets sent every 2 minutes with battery, temperature, and health data
 * 
 * Features:
 * - LSM6DSOX (accel + gyro) + LIS3MDL (magnetometer)
 * - H3LIS331 (high-G accelerometer) + BMP581 (pressure sensor)
 * - 4× FSR (force-sensitive resistors) encoded into H3LIS331 lower bits
 * - 3-sample redundancy per packet for reliability in chaotic motion
 * - Hardware-timed precision transmission using RTC1
 * - Sub-millisecond latency (no BLE overhead)
 * - Status monitoring with emergency shutdown protection
 * 
 * Hardware: Adafruit Feather nRF52840
 * Sensors: LSM6DSOX + LIS3MDL + H3LIS331 + BMP581 + 4×FSR on I2C/ADC
 * 
 * @author Your Name
 * @date January 2026
 * @version 3.2 - Timing fix for USB-powered operation
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_saadc.h"
#include "sensors.h"

extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);

// ============================================================================
// CONFIGURATION
// ============================================================================

// Hardware pins
#define LED_PIN                 15          // P1.15 (Feather red LED)
#define VBAT_PIN                NRF_SAADC_INPUT_AIN7  // P0.31 via voltage divider

// Ball identification (will be set via BLE provisioning in Phase 3)
#define BALL_ID                 1           // Change to 2, 3, etc. for other balls

// Transmission timing
#define TX_INTERVAL_MS          8           // 125Hz update rate
#define STATUS_INTERVAL_MS      120000      // 2 minutes (120 seconds)

// Emergency shutdown thresholds (only checks done on ball)
#define BATTERY_SHUTDOWN_MV     3300        // 3.3V - Emergency shutdown
#define TEMP_EMERGENCY_SHUTDOWN 6000        // 60.00°C - Emergency shutdown (in 0.01°C units)
#define BATTERY_USB_THRESHOLD   1000        // If below 1V, assume USB-only (no battery)

// Battery percentage thresholds (for estimation)
#define BATTERY_FULL_MV         4200        // 4.2V = 100%
#define BATTERY_EMPTY_MV        3300        // 3.3V = 0%

// Radio parameters (will be set via BLE provisioning in Phase 3)
#define RF_CHANNEL              40          // 2440 MHz
#define TX_POWER                RADIO_TXPOWER_TXPOWER_Pos8dBm

// Radio addressing (must exactly match receiver)
#define RADIO_BASE_ADDR         0x12345678  // Access address base
#define RADIO_PREFIX_ADDR       0xAB        // Access address prefix

// CRC configuration (must exactly match receiver)
#define CRC_POLYNOMIAL          0x00065B    // IBM CRC-24 polynomial  
#define CRC_INIT_VALUE          0x555555

// Packet structure sizes (UPDATED for FSR encoding)
#define SENSOR_DATA_SIZE        27          // Bytes per sensor sample (was 31)
#define PACKET_OVERHEAD         5           // Header: ball_id + sequence + timestamp
#define PACKET_PAYLOAD_SIZE     86          // Total: 5 + (27 * 3) = 86 bytes (was 98)

// Timeout values
#define RADIO_TIMEOUT_US        5000        // 5ms radio operation timeout

// Packet type identifiers
#define PACKET_TYPE_DATA        0x00        // Normal data packet
#define PACKET_TYPE_STATUS      0xFF        // Status packet

// ============================================================================
// TYPE DEFINITIONS
// ============================================================================

/**
 * Radio packet structure - must exactly match receiver
 * Total size: 86 bytes (reduced from 98 via FSR encoding)
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t ball_id;             // Ball identifier (1-8)
    uint16_t sequence;           // Packet sequence number (wraps at 65535)
    uint16_t timestamp;          // Milliseconds since boot (wraps at 65.5s)
    sensor_data_t data_t0;       // Current sensor reading (27 bytes)
    sensor_data_t data_t1;       // Previous reading, t-8ms (27 bytes)
    sensor_data_t data_t2;       // Reading from t-16ms (27 bytes)
} radio_packet_t;
#pragma pack(pop)

/**
 * Status packet structure (22 bytes with padding)
 * Sent every 2 minutes with system health data
 */
#pragma pack(push, 1)
typedef struct {
    // Header (6 bytes)
    uint8_t ball_id;              // Ball identifier (1-8)
    uint16_t sequence;            // Status packet sequence number
    uint16_t timestamp;           // Milliseconds since boot (wraps at 65.5s)
    uint8_t packet_type;          // 0xFF = status packet identifier
    
    // Safety telemetry (6 bytes)
    int16_t temperature;          // BMP581 temperature in 0.01°C units (2550 = 25.50°C)
    uint16_t battery_voltage;     // Battery voltage in millivolts
    uint8_t battery_percent;      // Battery capacity estimate (0-100%)
    uint8_t sensor_health;        // Bitmask of working sensors
    
    // Diagnostics (10 bytes)
    uint32_t total_packets_sent;  // Total data packets transmitted since boot
    uint32_t uptime_seconds;      // System uptime in seconds
    uint16_t radio_timeouts;      // Count of radio TX timeouts
    uint16_t i2c_errors;          // Count of I2C communication errors
    uint8_t reserved;             // Reserved for future use
    uint8_t checksum;             // Simple XOR checksum
} status_packet_t;
#pragma pack(pop)

// Compile-time verification of packet sizes
_Static_assert(sizeof(radio_packet_t) == 86, "radio_packet_t must be 86 bytes");

/**
 * Radio state for error tracking
 */
typedef enum {
    RADIO_STATE_OK = 0,
    RADIO_STATE_TX_TIMEOUT,
    RADIO_STATE_DISABLE_TIMEOUT,
    RADIO_STATE_STARTUP_FAILED
} radio_state_t;

// ============================================================================
// GLOBAL STATE
// ============================================================================

// Packet buffers (protected: only modified between radio operations)
static radio_packet_t tx_packet;
static uint16_t packet_sequence = 0;
static uint16_t status_sequence = 0;

// 3-sample history buffer for redundancy
// [0] = current, [1] = t-8ms, [2] = t-16ms
static sensor_data_t sensor_history[3];

// Error tracking
static volatile radio_state_t radio_status = RADIO_STATE_OK;
static uint32_t tx_timeout_count = 0;

// Status packet tracking
static uint32_t last_status_time_ms = 0;
static uint32_t total_packets_sent = 0;

// ============================================================================
// TIMING
// ============================================================================

/**
 * Initialize RTC1 as a free-running millisecond counter
 * Provides consistent timestamps across all balls for synchronization
 * 
 * Uses 32.768 kHz crystal: 32768 / (32+1) ≈ 993 Hz ≈ 1ms ticks
 */
static void timing_init(void)
{
    // Start Low Frequency Clock (required for RTC)
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
    
    // Configure RTC1 for ~1ms ticks
    NRF_RTC1->PRESCALER = 32;
    
    // Start the counter
    NRF_RTC1->TASKS_START = 1;
    
    // Allow clock to stabilize
    nrf_delay_ms(10);
}

/**
 * Get current timestamp in milliseconds
 * Wraps at 65.5 seconds (receiver handles wrapping correctly)
 */
static inline uint16_t get_timestamp_ms(void)
{
    return (uint16_t)(NRF_RTC1->COUNTER & 0xFFFF);
}

/**
 * Calculate uptime in seconds from RTC counter
 * 
 * @return Uptime in seconds
 */
static uint32_t get_uptime_seconds(void)
{
    // RTC1 counter increments at ~1ms per tick
    // Convert to seconds
    return (uint32_t)(NRF_RTC1->COUNTER / 1000);
}

/**
 * Precise delay using SDK function (calibrated for nRF52)
 */
static inline void delay_ms(uint32_t ms)
{
    nrf_delay_ms(ms);
}

// ============================================================================
// BATTERY MONITORING
// ============================================================================

/**
 * Initialize SAADC for battery voltage monitoring
 * Assumes VBAT connected to P0.31 (AIN7) through voltage divider
 */
static void battery_init(void)
{
    // Configure SAADC
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;
    
    // Configure channel 0 for battery monitoring
    NRF_SAADC->CH[0].CONFIG = 
        (SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos) |
        (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |
        (SAADC_CH_CONFIG_GAIN_Gain1_6 << SAADC_CH_CONFIG_GAIN_Pos) |
        (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
        (SAADC_CH_CONFIG_TACQ_10us << SAADC_CH_CONFIG_TACQ_Pos) |
        (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos);
    
    NRF_SAADC->CH[0].PSELP = VBAT_PIN;
    NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;
    
    // Enable SAADC
    NRF_SAADC->ENABLE = 1;
}

/**
 * Read battery voltage using nRF52840 ADC
 * 
 * @return Battery voltage in millivolts
 */
static uint16_t read_battery_voltage(void)
{
    int16_t adc_value;
    
    // Start sampling
    NRF_SAADC->RESULT.PTR = (uint32_t)&adc_value;
    NRF_SAADC->RESULT.MAXCNT = 1;
    
    NRF_SAADC->TASKS_START = 1;
    while (NRF_SAADC->EVENTS_STARTED == 0);
    NRF_SAADC->EVENTS_STARTED = 0;
    
    NRF_SAADC->TASKS_SAMPLE = 1;
    while (NRF_SAADC->EVENTS_END == 0);
    NRF_SAADC->EVENTS_END = 0;
    
    NRF_SAADC->TASKS_STOP = 1;
    while (NRF_SAADC->EVENTS_STOPPED == 0);
    NRF_SAADC->EVENTS_STOPPED = 0;
    
    // Convert ADC value to millivolts
    uint32_t battery_mv = ((uint32_t)adc_value * 1758) / 1000;
    
    return (uint16_t)battery_mv;
}

/**
 * Estimate battery percentage from voltage
 * 
 * @param voltage_mv Battery voltage in millivolts
 * @return Battery percentage (0-100)
 */
static uint8_t estimate_battery_percent(uint16_t voltage_mv)
{
    if (voltage_mv >= BATTERY_FULL_MV) {
        return 100;
    }
    if (voltage_mv <= BATTERY_EMPTY_MV) {
        return 0;
    }
    
    // Linear interpolation
    uint32_t percent = ((uint32_t)(voltage_mv - BATTERY_EMPTY_MV) * 100) / 
                       (BATTERY_FULL_MV - BATTERY_EMPTY_MV);
    
    return (uint8_t)percent;
}

// ============================================================================
// STATUS PACKET
// ============================================================================

/**
 * Calculate simple XOR checksum for status packet
 */
static uint8_t calculate_checksum(status_packet_t *packet)
{
    uint8_t *bytes = (uint8_t *)packet;
    uint8_t checksum = 0;
    
    for (int i = 0; i < sizeof(status_packet_t) - 1; i++) {
        checksum ^= bytes[i];
    }
    
    return checksum;
}

/**
 * Emergency system shutdown
 */
static void emergency_shutdown(void)
{
    SEGGER_RTT_printf(0, "\r\n!!! ENTERING EMERGENCY SHUTDOWN !!!\r\n");
    SEGGER_RTT_printf(0, "System will enter deep sleep to protect battery.\r\n");
    
    // LED pattern: rapid blink for 2 seconds
    for (int i = 0; i < 10; i++) {
        NRF_P1->OUTSET = (1 << LED_PIN);
        nrf_delay_ms(100);
        NRF_P1->OUTCLR = (1 << LED_PIN);
        nrf_delay_ms(100);
    }
    
    // Disable all peripherals
    NRF_SAADC->ENABLE = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    
    // Enter System OFF mode
    NRF_POWER->SYSTEMOFF = 1;
    
    while(1);
}

/**
 * Build and log status packet
 */
static void transmit_status_packet(bool emergency)
{
    status_packet_t status;
    
    // Header
    status.ball_id = BALL_ID;
    status.sequence = status_sequence++;
    status.timestamp = get_timestamp_ms();
    status.packet_type = PACKET_TYPE_STATUS;
    
    // Safety telemetry
    status.temperature = sensors_read_temperature();
    status.battery_voltage = read_battery_voltage();
    status.battery_percent = estimate_battery_percent(status.battery_voltage);
    status.sensor_health = sensors_get_status_bitmask();
    
    // Diagnostics
    status.total_packets_sent = total_packets_sent;
    status.uptime_seconds = get_uptime_seconds();
    status.radio_timeouts = (uint16_t)tx_timeout_count;
    status.i2c_errors = sensors_get_i2c_error_count();
    status.reserved = 0;
    status.checksum = calculate_checksum(&status);
    
    // Emergency shutdown check
    bool battery_present = (status.battery_voltage > BATTERY_USB_THRESHOLD);
    
    if ((battery_present && status.battery_voltage < BATTERY_SHUTDOWN_MV) || 
        status.temperature >= TEMP_EMERGENCY_SHUTDOWN) {
        
        SEGGER_RTT_printf(0, "\r\n!!! EMERGENCY CONDITION DETECTED !!!\r\n");
        if (battery_present && status.battery_voltage < BATTERY_SHUTDOWN_MV) {
            SEGGER_RTT_printf(0, "Battery critically low: %u mV\r\n", status.battery_voltage);
        }
        if (status.temperature >= TEMP_EMERGENCY_SHUTDOWN) {
            SEGGER_RTT_printf(0, "Temperature critical: %d.%02d C\r\n", 
                status.temperature / 100, status.temperature % 100);
        }
        
        emergency_shutdown();
    }
    
    // Log USB-only mode
    if (!battery_present && status.sequence == 0) {
        SEGGER_RTT_printf(0, "NOTE: USB-only mode (no battery detected)\r\n");
    }
    
    // Log status packet
    SEGGER_RTT_printf(0, "\r\n");
    SEGGER_RTT_printf(0, "================================================\r\n");
    SEGGER_RTT_printf(0, "=== STATUS PACKET #%u ===\r\n", status.sequence);
    SEGGER_RTT_printf(0, "================================================\r\n");
    SEGGER_RTT_printf(0, "Battery: %u mV (%u%%)\r\n", 
        status.battery_voltage, status.battery_percent);
    SEGGER_RTT_printf(0, "Temp: %d.%02d C\r\n", 
        status.temperature / 100, abs(status.temperature % 100));
    SEGGER_RTT_printf(0, "Uptime: %u seconds (%u:%02u:%02u)\r\n", 
        status.uptime_seconds,
        status.uptime_seconds / 3600,
        (status.uptime_seconds % 3600) / 60,
        status.uptime_seconds % 60);
    SEGGER_RTT_printf(0, "Sensors: 0x%02X ", status.sensor_health);
    if (status.sensor_health & SENSOR_LSM6_OK) SEGGER_RTT_printf(0, "LSM6 ");
    if (status.sensor_health & SENSOR_LIS3_OK) SEGGER_RTT_printf(0, "LIS3 ");
    if (status.sensor_health & SENSOR_H3LIS_OK) SEGGER_RTT_printf(0, "H3LIS ");
    if (status.sensor_health & SENSOR_BMP_OK) SEGGER_RTT_printf(0, "BMP");
    SEGGER_RTT_printf(0, "\r\n");
    SEGGER_RTT_printf(0, "Packets: %u total, %u timeouts\r\n", 
        status.total_packets_sent, status.radio_timeouts);
    SEGGER_RTT_printf(0, "I2C Errors: %u\r\n", status.i2c_errors);
    SEGGER_RTT_printf(0, "Checksum: 0x%02X\r\n", status.checksum);
    SEGGER_RTT_printf(0, "================================================\r\n");
    SEGGER_RTT_printf(0, "Next status packet in 120 seconds...\r\n");
    SEGGER_RTT_printf(0, "================================================\r\n\r\n");
    
    last_status_time_ms = get_timestamp_ms();
}

// ============================================================================
// RADIO CONFIGURATION
// ============================================================================

/**
 * Initialize radio for 2Mbps GFSK transmission
 */
static bool radio_init(void)
{
    // Power cycle radio
    NRF_RADIO->POWER = 0;
    nrf_delay_us(10);
    NRF_RADIO->POWER = 1;
    nrf_delay_us(10);

    // Configure radio mode: 2Mbps GFSK
    NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos;

    // Set transmit power
    NRF_RADIO->TXPOWER = TX_POWER;

    // Set RF channel
    NRF_RADIO->FREQUENCY = RF_CHANNEL;

    // Configure packet format (fixed 86-byte payload)
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) |
                       (0 << RADIO_PCNF0_LFLEN_Pos) |
                       (0 << RADIO_PCNF0_S1LEN_Pos);

    NRF_RADIO->PCNF1 = (PACKET_PAYLOAD_SIZE << RADIO_PCNF1_MAXLEN_Pos) |
                       (PACKET_PAYLOAD_SIZE << RADIO_PCNF1_STATLEN_Pos) |
                       (4 << RADIO_PCNF1_BALEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
                       (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);

    // Set access address
    NRF_RADIO->BASE0 = RADIO_BASE_ADDR;
    NRF_RADIO->PREFIX0 = RADIO_PREFIX_ADDR;
    NRF_RADIO->TXADDRESS = 0;
    NRF_RADIO->RXADDRESSES = 1;

    // Configure 24-bit CRC
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
                        (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCPOLY = CRC_POLYNOMIAL;
    NRF_RADIO->CRCINIT = CRC_INIT_VALUE;

    // Set packet buffer pointer
    NRF_RADIO->PACKETPTR = (uint32_t)&tx_packet;

    // Configure hardware shortcuts
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);

    return (NRF_RADIO->FREQUENCY == RF_CHANNEL);
}

// ============================================================================
// PACKET MANAGEMENT
// ============================================================================

/**
 * Update sensor history buffer with new reading
 */
static void update_sensor_history(const sensor_data_t *new_data)
{
    memcpy(&sensor_history[2], &sensor_history[1], sizeof(sensor_data_t));
    memcpy(&sensor_history[1], &sensor_history[0], sizeof(sensor_data_t));
    memcpy(&sensor_history[0], new_data, sizeof(sensor_data_t));
}

/**
 * Build packet with current sensor data + 2 historical samples
 */
static void prepare_packet(void)
{
    tx_packet.ball_id = BALL_ID;
    tx_packet.sequence = packet_sequence++;
    tx_packet.timestamp = get_timestamp_ms();
    
    memcpy(&tx_packet.data_t0, &sensor_history[0], sizeof(sensor_data_t));
    memcpy(&tx_packet.data_t1, &sensor_history[1], sizeof(sensor_data_t));
    memcpy(&tx_packet.data_t2, &sensor_history[2], sizeof(sensor_data_t));
}

/**
 * Transmit packet with timeout protection
 */
static bool transmit_packet(void)
{
    uint32_t timeout_us;
    
    // Clear events
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;

    // Start transmission
    NRF_RADIO->TASKS_TXEN = 1;
    
    // DEBUG: Check if radio starts
    timeout_us = 1000; // 1ms timeout for READY
    while (NRF_RADIO->EVENTS_READY == 0 && timeout_us > 0) {
        nrf_delay_us(10);
        timeout_us -= 10;
    }
    
    if (timeout_us == 0) {
        SEGGER_RTT_printf(0, "RADIO NEVER REACHED READY STATE\r\n");
        radio_status = RADIO_STATE_TX_TIMEOUT;
        tx_timeout_count++;
        return false;
    }

    // Wait for transmission to complete
    timeout_us = RADIO_TIMEOUT_US;
    while (NRF_RADIO->EVENTS_END == 0 && timeout_us > 0) {
        nrf_delay_us(10);
        timeout_us -= 10;
    }

    if (timeout_us == 0) {
        SEGGER_RTT_printf(0, "RADIO TX TIMEOUT (READY ok, but END never fired)\r\n");
        radio_status = RADIO_STATE_TX_TIMEOUT;
        tx_timeout_count++;
        return false;
    }

    // Wait for radio to disable
    timeout_us = RADIO_TIMEOUT_US;
    while (NRF_RADIO->EVENTS_DISABLED == 0 && timeout_us > 0) {
        nrf_delay_us(10);
        timeout_us -= 10;
    }

    if (timeout_us == 0) {
        SEGGER_RTT_printf(0, "RADIO DISABLE TIMEOUT\r\n");
        radio_status = RADIO_STATE_DISABLE_TIMEOUT;
        tx_timeout_count++;
        return false;
    }

    radio_status = RADIO_STATE_OK;
    return true;
}

// ============================================================================
// ERROR HANDLING
// ============================================================================

/**
 * LED blink pattern for fatal error
 */
static void indicate_error_fatal(void)
{
    while (1) {
        NRF_P1->OUTSET = (1 << LED_PIN);
        nrf_delay_ms(100);
        NRF_P1->OUTCLR = (1 << LED_PIN);
        nrf_delay_ms(100);
    }
}

/**
 * LED blink pattern for radio timeout
 */
static void indicate_error_radio(void)
{
    for (int i = 0; i < 3; i++) {
        NRF_P1->OUTSET = (1 << LED_PIN);
        nrf_delay_ms(500);
        NRF_P1->OUTCLR = (1 << LED_PIN);
        nrf_delay_ms(500);
    }
}

/**
 * Attempt radio recovery after timeout
 */
static void recover_radio(void)
{
    SEGGER_RTT_printf(0, "Radio timeout detected, attempting recovery...\r\n");
    
    NRF_RADIO->TASKS_DISABLE = 1;
    nrf_delay_ms(1);
    
    if (!radio_init()) {
        radio_status = RADIO_STATE_STARTUP_FAILED;
        SEGGER_RTT_printf(0, "Radio recovery FAILED\r\n");
        indicate_error_radio();
    } else {
        SEGGER_RTT_printf(0, "Radio recovered successfully\r\n");
    }
}

// ============================================================================
// MAIN PROGRAM
// ============================================================================

int main(void)
{
    sensor_data_t current_sensors;
    uint32_t loop_count = 0;
    uint32_t next_tx_time;
    uint32_t current_time_ms;

    // ===== Initialize High Frequency Crystal =====
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    // ===== LED Setup =====
    NRF_P1->DIRSET = (1 << LED_PIN);
    NRF_P1->OUTCLR = (1 << LED_PIN);

    // ===== Initialize Sensors =====
    SEGGER_RTT_printf(0, "\r\n=== Juggling Ball TX (Ball %d) ===\r\n", BALL_ID);
    SEGGER_RTT_printf(0, "Firmware Version: 3.2 - Timing Fix\r\n");
    SEGGER_RTT_printf(0, "Structure sizes:\r\n");
    SEGGER_RTT_printf(0, "  radio_packet_t: %u (expect 86)\r\n", sizeof(radio_packet_t));
    SEGGER_RTT_printf(0, "  sensor_data_t: %u (expect 27)\r\n", sizeof(sensor_data_t));
    SEGGER_RTT_printf(0, "  status_packet_t: %u bytes\r\n\r\n", sizeof(status_packet_t));

    SEGGER_RTT_printf(0, "Initializing sensors...\r\n");
    
    if (!sensors_init()) {
        SEGGER_RTT_printf(0, "FATAL: Sensor initialization failed\r\n");
        indicate_error_fatal();
    }
    
    uint8_t lsm6_addr, lis3_addr, h3lis_addr, bmp_addr;
    sensors_get_addresses(&lsm6_addr, &lis3_addr, &h3lis_addr, &bmp_addr);
    SEGGER_RTT_printf(0, "Sensors OK: LSM6=0x%02X, LIS3=0x%02X", lsm6_addr, lis3_addr);
    if (h3lis_addr != 0) {
        SEGGER_RTT_printf(0, ", H3LIS=0x%02X", h3lis_addr);
    }
    if (bmp_addr != 0) {
        SEGGER_RTT_printf(0, ", BMP=0x%02X", bmp_addr);
    }
    SEGGER_RTT_printf(0, "\r\n");

    // ===== Initialize Timing =====
    timing_init();

    // ===== Initialize Battery Monitoring =====
    SEGGER_RTT_printf(0, "Initializing battery monitoring...\r\n");
    battery_init();
    uint16_t initial_battery = read_battery_voltage();
    uint8_t initial_percent = estimate_battery_percent(initial_battery);
    
    if (initial_battery < BATTERY_USB_THRESHOLD) {
        SEGGER_RTT_printf(0, "Battery: %u mV (USB-only mode, no battery detected)\r\n", 
            initial_battery);
    } else {
        SEGGER_RTT_printf(0, "Battery: %u mV (%u%%)\r\n", 
            initial_battery, initial_percent);
    }

    // ===== Initialize Radio =====
    SEGGER_RTT_printf(0, "Initializing radio on channel %d...\r\n", RF_CHANNEL);
    if (!radio_init()) {
        SEGGER_RTT_printf(0, "FATAL: Radio initialization failed\r\n");
        indicate_error_fatal();
    }
    SEGGER_RTT_printf(0, "Radio OK\r\n");

    // ===== Initialize History Buffer =====
    memset(sensor_history, 0, sizeof(sensor_history));
    memset(&tx_packet, 0, sizeof(tx_packet));

    // LED on briefly = ready to transmit
    NRF_P1->OUTSET = (1 << LED_PIN);
    nrf_delay_ms(500);
    NRF_P1->OUTCLR = (1 << LED_PIN);
    
    SEGGER_RTT_printf(0, "Starting transmission at 125Hz...\r\n");
    SEGGER_RTT_printf(0, "Status packets every 2 minutes (120 seconds)\r\n");
    SEGGER_RTT_printf(0, "Packet size: 86 bytes (12 bytes saved via FSR encoding)\r\n");
    SEGGER_RTT_printf(0, "First status packet in 120 seconds...\r\n\r\n");

    // ===== Main Transmission Loop =====
    // FIX: Initialize timing properly for first packet
    uint16_t current_time = get_timestamp_ms();
    next_tx_time = current_time + TX_INTERVAL_MS;  // Start 8ms in future
    last_status_time_ms = current_time;
    
    SEGGER_RTT_printf(0, "DEBUG: Starting loop at T=%u, next_tx=%u\r\n\r\n", 
        current_time, next_tx_time);
    
    while (1)
    {
        // DEBUG: Verify loop execution every second
        current_time_ms = get_timestamp_ms();
        if (loop_count == 0 || loop_count % 125 == 0) {
            int32_t time_until_tx = (int32_t)((int16_t)(next_tx_time - current_time_ms));
            SEGGER_RTT_printf(0, "DEBUG: Loop=%u, T=%u, next=%u, diff=%d\r\n", 
                loop_count, current_time_ms, next_tx_time, time_until_tx);
        }
        
        // Read sensors
        sensors_read(&current_sensors);

        // Update history buffer
        update_sensor_history(&current_sensors);

        // Build packet
        prepare_packet();

        // Debug print every 125 loops (once per second at 125Hz)
        if (loop_count % 125 == 0) {
            // Decode H3LIS331 + FSR data for display
            int16_t h3lis_x = current_sensors.h3lis_x_fsr_level >> 4;
            int16_t h3lis_y = current_sensors.h3lis_y_fsr_pattern >> 4;
            int16_t h3lis_z = current_sensors.h3lis_z_flags >> 4;
            
            // Sign-extend 12-bit to 16-bit
            if (h3lis_x & 0x0800) h3lis_x |= 0xF000;
            if (h3lis_y & 0x0800) h3lis_y |= 0xF000;
            if (h3lis_z & 0x0800) h3lis_z |= 0xF000;
            
            // Extract FSR data
            uint8_t fsr_level = current_sensors.h3lis_x_fsr_level & 0x0F;
            uint8_t fsr_pattern = current_sensors.h3lis_y_fsr_pattern & 0x0F;
            uint8_t flags = current_sensors.h3lis_z_flags & 0x0F;
            
            SEGGER_RTT_printf(0, "\r\n[Seq %u, T=%u ms]\r\n", tx_packet.sequence, tx_packet.timestamp);
            
            // H3LIS331 with FSR info
            SEGGER_RTT_printf(0, "H3LIS: X=%6d Y=%6d Z=%6d | FSR: Lvl=%2u Pat=0x%X\r\n", 
                h3lis_x, h3lis_y, h3lis_z, fsr_level, fsr_pattern);
            
            // FSR details
            const char* squeeze_str;
            if (fsr_level == 0) squeeze_str = "None";
            else if (fsr_level <= 5) squeeze_str = "Light";
            else if (fsr_level <= 10) squeeze_str = "Medium";
            else squeeze_str = "Hard";
            
            uint8_t num_active = 0;
            if (fsr_pattern & 0x01) num_active++;
            if (fsr_pattern & 0x02) num_active++;
            if (fsr_pattern & 0x04) num_active++;
            if (fsr_pattern & 0x08) num_active++;
            
            SEGGER_RTT_printf(0, "  Squeeze: %s (%u/15) | Active: %u/4 [%c%c%c%c]\r\n",
                squeeze_str, fsr_level, num_active,
                (fsr_pattern & 0x01) ? '0' : '-',
                (fsr_pattern & 0x02) ? '1' : '-',
                (fsr_pattern & 0x04) ? '2' : '-',
                (fsr_pattern & 0x08) ? '3' : '-');
            
            // Rest of sensors
            SEGGER_RTT_printf(0, "LSM6A: X=%6d Y=%6d Z=%6d\r\n",
                current_sensors.imu.accel[0],
                current_sensors.imu.accel[1],
                current_sensors.imu.accel[2]);
            SEGGER_RTT_printf(0, "LSM6G: X=%6d Y=%6d Z=%6d\r\n",
                current_sensors.imu.gyro[0],
                current_sensors.imu.gyro[1],
                current_sensors.imu.gyro[2]);
            SEGGER_RTT_printf(0, "LIS3M: X=%6d Y=%6d Z=%6d\r\n",
                current_sensors.imu.mag[0],
                current_sensors.imu.mag[1],
                current_sensors.imu.mag[2]);
            
            uint32_t pressure = (uint32_t)(current_sensors.pressure[0] | 
                                          (current_sensors.pressure[1] << 8) | 
                                          (current_sensors.pressure[2] << 16));
            SEGGER_RTT_printf(0, "BMP  : 0x%02X%02X%02X (%u Pa)\r\n",
                current_sensors.pressure[2],
                current_sensors.pressure[1],
                current_sensors.pressure[0],
                pressure);
        }

        // Transmit (with timeout protection)
        bool tx_success = transmit_packet();
        
        // EMERGENCY DEBUG: Print result of first 10 transmissions
        if (loop_count < 10) {
            SEGGER_RTT_printf(0, "TX attempt %u: %s\r\n", 
                loop_count, tx_success ? "SUCCESS" : "FAILED");
        }
        
        // Track successful transmission
        if (tx_success) {
            total_packets_sent++;
        }
        
        // Handle radio errors
        if (!tx_success) {
            SEGGER_RTT_printf(0, "Radio error at loop %u, status=%d\r\n", 
                loop_count, radio_status);
            recover_radio();
        }

        // Check if it's time to send status packet
        uint32_t current_uptime_sec = get_uptime_seconds();
        static uint32_t last_status_uptime_sec = 0;
        
        if ((current_uptime_sec - last_status_uptime_sec) >= 120) {
            transmit_status_packet(false);
            last_status_uptime_sec = current_uptime_sec;
        }

        // LED toggle
        if (NRF_P1->OUT & (1 << LED_PIN)) {
            NRF_P1->OUTCLR = (1 << LED_PIN);
        } else {
            NRF_P1->OUTSET = (1 << LED_PIN);
        }

        // Increment loop counter
        loop_count++;

        // Precision timing: wait until next scheduled transmission
        next_tx_time += TX_INTERVAL_MS;
        current_time_ms = get_timestamp_ms();
        
        // Handle timestamp wrapping and timing safety
        int32_t time_diff = (int32_t)((int16_t)(next_tx_time - current_time_ms));
        
        // Only delay if we're ahead and within reasonable range
        if (time_diff > 0 && time_diff < 100) {
            delay_ms((uint32_t)time_diff);
        } else if (time_diff >= 100) {
            // We're way behind - resync to avoid permanent deadlock
            SEGGER_RTT_printf(0, "WARN: Timing resync (behind by %d ms)\r\n", time_diff);
            next_tx_time = current_time_ms + TX_INTERVAL_MS;
        }
        // If time_diff < 0, we're past deadline - transmit immediately next iteration
    }
}