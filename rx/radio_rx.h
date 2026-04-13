/**
 * Radio Receiver Module
 *
 * Continuous 2Mbps GFSK reception. Hardware shortcuts handle automatic
 * packet-to-packet restart without CPU involvement.
 *
 * Radio constants and packet structures are in packet_spec.h.
 */

#ifndef RADIO_RX_H
#define RADIO_RX_H

#include <stdint.h>
#include <stdbool.h>
#include "packet_spec.h"

// ============================================================================
// TYPE DEFINITIONS
// ============================================================================

typedef struct {
    uint32_t total_packets;     // Packets received with CRC OK
    uint32_t crc_errors;        // CRC failures
    uint32_t end_events;        // Total RADIO END interrupts (good + bad)
    uint32_t isr_overwrites;    // CRC-OK packets dropped because the previous
                                // packet had not yet been consumed by the main
                                // loop. Indicates main loop is taking > 4ms
                                // between radio_packet_available() checks.
                                // Non-zero is a real loss event — investigate
                                // what is blocking the main loop (RTT, USB).
} radio_stats_t;

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

/**
 * Initialize radio for 2Mbps GFSK reception.
 * Must be called before radio_start_rx().
 *
 * Verifies MODE, FREQUENCY, PCNF1.STATLEN, BASE0, CRCPOLY, and CRCINIT
 * readbacks. A mismatch means the peripheral did not accept the configuration.
 *
 * @return true on success
 */
bool radio_init(void);

/**
 * Start continuous reception.
 * Hardware shortcuts keep the radio in RX without CPU scheduling.
 */
void radio_start_rx(void);

/**
 * Check if a new valid packet is waiting.
 *
 * @return true if packet available
 */
bool radio_packet_available(void);

/**
 * Get pointer to the internal receive buffer.
 * Buffer contents are valid only between radio_packet_available() returning
 * true and the next radio interrupt. Make a local copy before calling
 * radio_clear_packet_flag().
 *
 * At 250Hz (4ms between packets) you have 4ms to copy before the next
 * packet may overwrite the buffer. A memcpy of 86 bytes takes ~1us.
 * Do not do blocking work before copying.
 *
 * @return Pointer to received packet (do not cache across interrupt boundary)
 */
const radio_packet_t *radio_get_packet_buffer(void);

/**
 * Acknowledge and clear the packet ready flag.
 * Must be called after copying the packet buffer.
 */
void radio_clear_packet_flag(void);

/**
 * Fill stats structure with current reception counters.
 * Snapshot is taken with IRQs disabled for atomicity.
 */
void radio_get_stats(radio_stats_t *stats);

#endif // RADIO_RX_H
