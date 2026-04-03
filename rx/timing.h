/**
 * Timing Module - RTC1 interrupt-driven millisecond counter
 *
 * Uses 32.768kHz crystal with TICK interrupt (~993Hz ≈ 1ms).
 * Counter is 32-bit, wraps after ~49.7 days.
 */

#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>

void     timing_init(void);
uint32_t get_timestamp_ms(void);

#endif // TIMING_H
