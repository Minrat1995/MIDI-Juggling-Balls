/**
 * Timing Module - RTC1 interrupt-driven tick counter
 *
 * Uses 32.768kHz crystal via nrf_drv_clock. PRESCALER=32 gives:
 *   f = 32768 / (32+1) = 992.97 Hz, period = 1.007ms per tick.
 *
 * This is NOT a millisecond counter. Each tick is ~1.007ms.
 * Use get_rtc_ticks() consistently. Do not divide by 1000 to get seconds;
 * use RTC1_TICKS_PER_SEC instead.
 *
 * Counter is 32-bit, wraps after ~49.7 days of uptime.
 */

#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>
#include <stdbool.h>

// RTC1 runs at 992.97Hz. Use this constant wherever tick->second
// conversion is needed. Do not use 1000 — it makes uptime run 0.71% fast.
#define RTC1_TICKS_PER_SEC  993

/**
 * Initialise RTC1 and request LFCLK via the clock driver.
 *
 * Blocks up to LFCLK_STARTUP_TIMEOUT_MS waiting for the crystal to start.
 * The LFXO takes 200-600ms; a timeout shorter than that will always fire.
 *
 * @return true on success, false if LFCLK did not start within the timeout.
 */
bool     timing_init(void);

/**
 * Return current RTC1 tick count.
 *
 * Each tick is ~1.007ms (992.97Hz). This is NOT milliseconds.
 * Named _ticks to prevent accidental divide-by-1000 conversions.
 */
uint32_t get_rtc_ticks(void);

#endif // TIMING_H
