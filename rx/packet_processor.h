/**
 * Packet Processor Module
 *
 * Tracks per-ball reception statistics and decodes sensor data for
 * RTT validation output. Phase 1: one ball, RTT only.
 */

#ifndef PACKET_PROCESSOR_H
#define PACKET_PROCESSOR_H

#include <stdint.h>
#include "radio_rx.h"

#define MAX_BALLS   8

/**
 * Clear all per-ball state.
 */
void packet_processor_init(void);

/**
 * Process one received packet.
 * Updates sequence tracking, detects gaps, triggers RTT decode output.
 *
 * @param packet      Validated packet (local copy, safe to read)
 * @param rx_ticks    RX timestamp in RTC ticks (~993Hz)
 */
void packet_processor_process(const radio_packet_t *packet, uint32_t rx_ticks);

/**
 * Print per-ball statistics and radio counters to RTT.
 *
 * CAUTION: This function issues multiple SEGGER_RTT_printf calls. If the
 * RTT up-buffer is full and the RTT channel is configured in blocking mode
 * (SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL), this function can block for an
 * arbitrarily long time. During that block, the radio ISR fires every 4ms
 * and overwrites the single RX buffer — every packet arriving during the
 * stall is silently lost. isr_overwrites in radio_stats_t will increase.
 *
 * Recommended: set SEGGER_RTT_CONFIG_DEFAULT_MODE to
 * SEGGER_RTT_MODE_NO_BLOCK_SKIP in SEGGER_RTT_Conf.h. The tradeoff is
 * occasional loss of RTT output lines rather than loss of radio packets.
 *
 * Call once per second from the main loop.
 */
void packet_processor_print_statistics(void);

#endif // PACKET_PROCESSOR_H
