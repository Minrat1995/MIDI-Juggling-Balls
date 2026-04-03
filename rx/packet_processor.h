/**
 * Packet Processor Module
 *
 * Tracks per-ball reception statistics and decodes sensor data for
 * RTT validation output. Phase 1: one ball, RTT only.
 */

#ifndef PACKET_PROCESSOR_H
#define PACKET_PROCESSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "radio_rx.h"

#define MAX_BALLS   8

typedef struct {
    uint16_t last_sequence;
    uint32_t packets_received;
    uint32_t packets_lost;
    uint32_t rx_timestamp_ms;
    bool     initialized;
} ball_state_t;

/**
 * Clear all per-ball state.
 */
void packet_processor_init(void);

/**
 * Process one received packet.
 * Updates sequence tracking, detects gaps, triggers RTT decode output.
 *
 * @param packet     Validated packet (local copy, safe to read)
 * @param rx_time_ms RX timestamp in milliseconds
 */
void packet_processor_process(const radio_packet_t *packet, uint32_t rx_time_ms);

/**
 * Print per-ball statistics and radio counters to RTT.
 * Call once per second from the main loop.
 */
void packet_processor_print_statistics(void);

/**
 * Get state for one ball.
 *
 * @param ball_id  1-8
 * @param state    Output (copied)
 * @return true if ball has been seen
 */
bool packet_processor_get_ball_state(uint8_t ball_id, ball_state_t *state);

#endif // PACKET_PROCESSOR_H
