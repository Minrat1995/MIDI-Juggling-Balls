/**
 * Packet Processor Module Implementation
 */

#include "packet_processor.h"
#include "radio_rx.h"
#include <string.h>

extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);

// ============================================================================
// INTERNAL STATE
// ============================================================================

// Per-ball tracking state
static ball_state_t ball_state[MAX_BALLS];

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

void packet_processor_init(void)
{
    memset(ball_state, 0, sizeof(ball_state));
}

void packet_processor_process(const radio_packet_t *packet, uint32_t rx_time_ms)
{
    uint8_t ball_id = packet->ball_id;
    
    // Validate ball ID
    if (ball_id == 0 || ball_id > MAX_BALLS) {
        SEGGER_RTT_printf(0, "Invalid ball ID: %d\r\n", ball_id);
        return;
    }
    
    ball_state_t *state = &ball_state[ball_id - 1];
    
    // First packet from this ball?
    if (!state->initialized) {
        state->last_sequence = packet->sequence;
        state->packets_received = 1;
        state->packets_lost = 0;
        state->rx_timestamp_ms = rx_time_ms;
        state->initialized = true;
        
        SEGGER_RTT_printf(0, "\r\n*** Ball %d connected, Seq=%u ***\r\n", 
                         ball_id, packet->sequence);
        return;
    }
    
    // Check for sequence gaps (packet loss)
    uint16_t expected_seq = (state->last_sequence + 1) & 0xFFFF;
    uint16_t received_seq = packet->sequence;
    
    if (received_seq != expected_seq) {
        // Calculate gap size (handle wrapping)
        uint16_t gap;
        if (received_seq > expected_seq) {
            gap = received_seq - expected_seq;
        } else {
            gap = (0xFFFF - expected_seq) + received_seq + 1;
        }
        
        // Only count reasonable gaps (filter noise)
        if (gap < 1000) {
            state->packets_lost += gap;
        }
    }
    
    // Update state
    state->last_sequence = received_seq;
    state->packets_received++;
    state->rx_timestamp_ms = rx_time_ms;
}

void packet_processor_print_statistics(void)
{
    radio_stats_t radio_stats;
    radio_get_stats(&radio_stats);
    
    SEGGER_RTT_printf(0, "\r\n=== Reception Statistics ===\r\n");
    SEGGER_RTT_printf(0, "Total packets: %lu, CRC errors: %lu\r\n",
                     radio_stats.total_packets, radio_stats.crc_errors);
    SEGGER_RTT_printf(0, "Radio interrupts: END=%lu, CRC_OK=%lu, CRC_FAIL=%lu\r\n\r\n",
                     radio_stats.radio_end_events, radio_stats.radio_crc_ok, 
                     radio_stats.radio_crc_fail);
    
    // Display per-ball statistics
    for (int i = 0; i < MAX_BALLS; i++) {
        ball_state_t *state = &ball_state[i];
        
        if (state->initialized && state->packets_received > 0) {
            float loss_rate = 0.0f;
            uint32_t total_expected = state->packets_received + state->packets_lost;
            
            if (total_expected > 0) {
                loss_rate = (100.0f * state->packets_lost) / total_expected;
            }
            
            // Print loss rate as "X.XX pct" (SEGGER RTT doesn't handle %f well)
            SEGGER_RTT_printf(0, "Ball %d: RX=%lu Lost=%lu (%lu.%02lu pct) LastSeq=%u\r\n",
                             i + 1,
                             state->packets_received,
                             state->packets_lost,
                             (uint32_t)loss_rate,
                             (uint32_t)((loss_rate - (uint32_t)loss_rate) * 100),
                             state->last_sequence);
        }
    }
    SEGGER_RTT_printf(0, "\r\n");
}

bool packet_processor_get_ball_state(uint8_t ball_id, ball_state_t *state)
{
    if (ball_id == 0 || ball_id > MAX_BALLS || !state) {
        return false;
    }
    
    ball_state_t *internal_state = &ball_state[ball_id - 1];
    
    if (!internal_state->initialized) {
        return false;
    }
    
    memcpy(state, internal_state, sizeof(ball_state_t));
    return true;
}
