/**
 * Timing Module Implementation
 */

#include "timing.h"
#include "nrf.h"
#include "nrf_delay.h"

static volatile uint32_t system_time_ms = 0;

void timing_init(void)
{
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);

    // PRESCALER=32: f = 32768/(32+1) = 992.97 Hz, period = 1.0071 ms
    // This is a 0.71% systematic undercount — system_time_ms runs slow.
    // Over 30 minutes, the counter reads ~12.8 seconds behind wall time.
    // This does not affect radio timing (hardware), packet loss stats,
    // or any real-time decisions in this project.
    //
    // Exact 1ms is not achievable with a 32.768kHz crystal (32768 is not
    // an integer multiple of 1000). If precise wall time is ever needed,
    // use the RTC COUNTER register directly and convert:
    //   ms = (NRF_RTC1->COUNTER * 1000) / 993
    NRF_RTC1->PRESCALER = 32;
    NRF_RTC1->INTENSET  = RTC_INTENSET_TICK_Msk;
    NRF_RTC1->TASKS_START = 1;
    NVIC_SetPriority(RTC1_IRQn, 2);
    NVIC_EnableIRQ(RTC1_IRQn);

    nrf_delay_ms(10);
}

uint32_t get_timestamp_ms(void)
{
    return system_time_ms;
}

void RTC1_IRQHandler(void)
{
    if (NRF_RTC1->EVENTS_TICK) {
        NRF_RTC1->EVENTS_TICK = 0;
        system_time_ms++;
    }
}
