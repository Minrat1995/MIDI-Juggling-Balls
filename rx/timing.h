/**
 * Timing Module - RTC-based millisecond counter
 * 
 * Provides system-wide millisecond timestamps for packet arrival tracking
 * and statistics intervals.
 * 
 * Uses RTC1 with 32.768 kHz crystal for low-power, always-on timing.
 */

#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>

/**
 * Initialize RTC1 as a 1ms ticker
 * 
 * Configures RTC1 to generate ~1ms ticks using the 32.768 kHz crystal.
 * Must be called after LFCLK is started.
 * 
 * Note: This function starts the Low Frequency Clock automatically.
 */
void timing_init(void);

/**
 * Get current system timestamp in milliseconds
 * 
 * @return Milliseconds since timing_init() was called
 * 
 * Note: Counter is 32-bit and wraps after ~49.7 days of continuous operation.
 */
uint32_t get_timestamp_ms(void);

#endif // TIMING_H
