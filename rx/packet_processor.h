/**
 * Packet Processor Module
 * 
 * Handles packet parsing, ball state tracking, and statistics management.
 * Maintains per-ball statistics including packet loss detection.
 */

#ifndef PACKET_PROCESSOR_H
#define PACKET_PROCESSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "radio_rx.h"

// Maximum number of balls supported
#define MAX_BALLS               8

// ============================================================================
// TYPE DEFINITIONS
// ============================================================================

/**
 * Per-ball tracking state
 * Maintains reception statistics and sequence tracking for each ball
 */
typedef struct {
    uint16_t last_sequence;      // Last received sequence number
    uint32_t packets_received;   // Total packets received
    uint32_t packets_lost;       // Total packets lost (sequence gaps)
    uint32_t rx_timestamp_ms;    // Timestamp of last received packet
    bool initialized;            // True after first packet received
} ball_state_t;

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

/**
 * Initialize packet processor
 * 
 * Clears all ball state and statistics.
 */
void packet_processor_init(void);

/**
 * Process a received packet
 * 
 * Updates ball statistics, detects packet loss via sequence number gaps,
 * and tracks first packet arrival for each ball.
 * 
 * @param packet Pointer to received packet (will be copied internally)
 * @param rx_time_ms Receiver timestamp when packet arrived
 */
void packet_processor_process(const radio_packet_t *packet, uint32_t rx_time_ms);

/**
 * Print statistics summary for all active balls
 * 
 * Outputs via SEGGER RTT (will be replaced with USB output in future).
 * Shows per-ball packet counts, loss rates, and radio statistics.
 */
void packet_processor_print_statistics(void);

/**
 * Get state for a specific ball
 * 
 * @param ball_id Ball identifier (1-8)
 * @param state Pointer to structure to populate with ball state
 * @return true if ball has been seen, false otherwise
 */
bool packet_processor_get_ball_state(uint8_t ball_id, ball_state_t *state);

#endif // PACKET_PROCESSOR_H
