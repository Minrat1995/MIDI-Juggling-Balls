/**
 * Timing Module Implementation
 */

#include "timing.h"
#include "nrf.h"
#include "nrf_delay.h"

// Free-running millisecond counter (incremented by RTC1 interrupt)
static volatile uint32_t system_time_ms = 0;

void timing_init(void)
{
    // Start Low Frequency Clock (required for RTC)
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
    
    // Configure RTC1: 32.768 kHz / (32+1) ≈ 993 Hz ≈ 1ms
    NRF_RTC1->PRESCALER = 32;
    NRF_RTC1->INTENSET = RTC_INTENSET_TICK_Msk;
    NRF_RTC1->TASKS_START = 1;
    NVIC_EnableIRQ(RTC1_IRQn);
    
    nrf_delay_ms(10);  // Allow clock to stabilize
}

uint32_t get_timestamp_ms(void)
{
    return system_time_ms;
}

/**
 * RTC1 interrupt handler - increments millisecond counter
 */
void RTC1_IRQHandler(void)
{
    if (NRF_RTC1->EVENTS_TICK) {
        NRF_RTC1->EVENTS_TICK = 0;
        system_time_ms++;
    }
}
