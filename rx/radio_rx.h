/**
 * Radio Receiver Module
 * 
 * Manages nRF52840 radio hardware for continuous packet reception.
 * Provides low-latency 2.4GHz communication using custom protocol.
 */

#ifndef RADIO_RX_H
#define RADIO_RX_H

#include <stdint.h>
#include <stdbool.h>
#include "sensors.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

// Radio parameters (must exactly match transmitter)
#define RF_CHANNEL              40          // 2440 MHz
#define RADIO_BASE_ADDR         0x12345678  // Access address base
#define RADIO_PREFIX_ADDR       0xAB        // Access address prefix
#define CRC_POLYNOMIAL          0x00065B    // IBM CRC-24
#define CRC_INIT_VALUE          0x555555

// Packet structure sizes
#define SENSOR_DATA_SIZE        31          // Bytes per sensor sample
#define PACKET_OVERHEAD         5           // Header: ball_id + sequence + timestamp
#define PACKET_PAYLOAD_SIZE     98          // Total: 5 + (31 * 3) = 98 bytes

// ============================================================================
// TYPE DEFINITIONS
// ============================================================================

/**
 * Radio packet structure - must exactly match transmitter
 * Total size: 98 bytes
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t ball_id;             // Ball identifier (1-8)
    uint16_t sequence;           // Packet sequence number (wraps at 65535)
    uint16_t timestamp;          // Milliseconds since TX boot (wraps at 65.5s)
    sensor_data_t data_t0;       // Current sensor reading (31 bytes)
    sensor_data_t data_t1;       // Previous reading, t-8ms (31 bytes)
    sensor_data_t data_t2;       // Reading from t-16ms (31 bytes)
} radio_packet_t;
#pragma pack(pop)

// Compile-time verification of packet size
_Static_assert(sizeof(radio_packet_t) == 98, "radio_packet_t must be 98 bytes");

/**
 * Reception statistics for debugging
 */
typedef struct {
    uint32_t total_packets;      // Total packets received
    uint32_t crc_errors;         // CRC failures
    uint32_t radio_end_events;   // Radio END interrupts
    uint32_t radio_crc_ok;       // CRC OK events
    uint32_t radio_crc_fail;     // CRC FAIL events
} radio_stats_t;

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

/**
 * Initialize radio for continuous 2Mbps GFSK reception
 * 
 * Configures hardware shortcuts for automatic packet-to-packet transitions
 * without CPU intervention (zero-latency restart).
 * 
 * @return true on success, false if radio not responding
 */
bool radio_init(void);

/**
 * Start continuous radio reception
 * 
 * Clears events and triggers initial RXEN. After this, hardware shortcuts
 * automatically restart reception after each packet.
 */
void radio_start_rx(void);

/**
 * Get pointer to received packet buffer
 * 
 * @return Pointer to internal packet buffer (read-only)
 * 
 * Note: Buffer is modified in interrupt context. Make atomic copy before processing.
 */
const radio_packet_t* radio_get_packet_buffer(void);

/**
 * Check if new packet has been received
 * 
 * @return true if packet available, false otherwise
 * 
 * Note: Call radio_clear_packet_flag() after processing to reset flag.
 */
bool radio_packet_available(void);

/**
 * Clear the packet received flag
 * 
 * Must be called after processing each packet to acknowledge receipt.
 */
void radio_clear_packet_flag(void);

/**
 * Get reception statistics
 * 
 * @param stats Pointer to structure to populate with current statistics
 */
void radio_get_stats(radio_stats_t *stats);

#endif // RADIO_RX_H
