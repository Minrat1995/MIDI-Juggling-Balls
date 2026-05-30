/**
 * Juggling Ball Wireless Transmitter
 *
 * ~199Hz, 86-byte packets over 2.4GHz custom protocol.
 * Status data logged to RTT every 2 minutes (battery, temp, health).
 *
 * Hardware: Adafruit Feather nRF52840
 * Sensors:  LSM6DSOX + LIS3MDL + H3LIS331 + BMP581 + 4xFSR (Phase 3)
 *
 * Sensor ODR vs TX rate at ~199Hz:
 *   LSM6DSOX accel/gyro: 416Hz  - 2.09x TX rate, near-full capture
 *   H3LIS331:            400Hz  - 2.01x TX rate, near-full capture
 *   BMP581:              ~218Hz - 1.10x TX rate, near-full capture, occasional duplicate read (harmless; normal mode registers update atomically)
 *   LIS3MDL:             155Hz  - TX 1.28x faster, ~4 in 5 packets have fresh mag data
 *
 * @version 3.18 + post-v3.18 fixes + batt-log + bmp-addr + batt-vdiv-fix2 + wdt-ctx + sdk-assert + tx-rate-199
 *
 * Changelog from wdt-ctx (tx-rate-199):
 *   - TX_INTERVAL_TICKS changed from 4 to 5.
 *     At 250Hz (4 ticks), 3 balls x 89-byte USB frames = 66,750 bytes/sec, which
 *     exceeds the USB Full Speed bulk endpoint ceiling of 64,000 bytes/sec (one
 *     64-byte transfer per 1ms frame). 3-ball operation was physically impossible.
 *     At ~199Hz (5 ticks, actual 198.6Hz, 5.035ms interval), 3 balls x 89 bytes =
 *     53,133 bytes/sec = 83% of ceiling, leaving 17% headroom for jitter.
 *     Latency budget unchanged within spec: sensor avg wait increases ~2ms -> ~2.5ms.
 *     Sensor capture ratios updated in file header. t1/t2 history slots now represent
 *     t-5ms and t-10ms; packet_spec.h updated accordingly.
 *     RTT banner updated from "250Hz (4ms)" to "~199Hz (5ms)".
 *
 * Changelog from batt-vdiv-fix2 (wdt-ctx):
 *   - g_wdt_context: volatile uint8_t global updated at key points in the main
 *     loop and inside twi_wait() in sensors.c. WDT_IRQHandler writes its value
 *     to GPREGRET in the ~122us window before the WDT reset fires, so the next
 *     boot's banner reports exactly where the stall occurred without needing
 *     J-Link to be connected during the dropout.
 *   - WDT_IRQHandler added. WDT INTENSET.TIMEOUT enabled in wdt_init().
 *     NVIC priority 7 (lowest available) -- the handler only writes one register
 *     and must not preempt time-critical ISRs (TWIM at priority 2).
 *   - Boot banner restructured: RESETREAS and GPREGRET/GPREGRET2 are now read
 *     together before clearing. WDT path decodes g_wdt_context from GPREGRET
 *     and prints a human-readable stall location. HardFault (0xAA) and
 *     init-stage failure codes (GPREGRET2) reported as before.
 *   - Version string updated to wdt-ctx.
 *
 * Changelog from batt-vdiv (batt-vdiv-fix2):
 *   - VBAT_PIN corrected from NRF_SAADC_INPUT_AIN7 (P0.31) to
 *     NRF_SAADC_INPUT_AIN5 (P0.29).
 *     Root cause of all wrong battery readings: P0.31 is the AREF pin on the
 *     Adafruit Feather nRF52840. Per Adafruit documentation, AREF is not
 *     available for use with the ADC. The actual VBAT voltage divider mid-point
 *     is on P0.29 (A6/VDIV), hard-wired to a 150K/150K resistor divider on the
 *     LiPo input. Every previous battery read was sampling the floating AREF
 *     pin, producing the observed 193mV-820mV drifting readings.
 *   - VBAT_VDIV_ENABLE_PIN and all associated P0.14 GPIO drive code removed.
 *     The Feather nRF52840 does not have a switched voltage divider. The
 *     150K/150K divider is hardwired directly between VBAT and P0.29 -- there
 *     is no P-FET, no enable GPIO, and no P0.14 involvement in battery sensing.
 *     The previous "batt-vdiv" fix drove an unrelated pin and had no effect.
 *   - battery_init() comment updated: divider is 150K/150K (not 1MΩ/1MΩ as
 *     previously stated). Ratio is still 1:1 so the 1758 formula multiplier
 *     is unchanged and remains correct.
 *   - Version string updated to batt-vdiv-fix2.
 *
 * Changelog from previous (batt-log+bmp-addr):
 *   - battery_init(): VBAT_VDIV_ENABLE_PIN (P0.14) driven LOW to enable the
 *     Feather's battery voltage divider P-FET before SAADC init.
 *     NOTE: this fix was based on incorrect hardware assumptions. See batt-vdiv-fix2
 *     above for the correct fix. The P-FET and enable pin do not exist on this board.
 *
 * Post-v3.18 functional fixes (main.c) -- post-WDT-reset recovery:
 *   Root cause: after a TWI hang (Errata 89/121) the WDT fires and resets TX.
 *   The WDT resume-counting-from-zero fix (feeds #1 and #2 plus sensors.c feeds)
 *   allows TX to survive its own init sequence. However, sensors_init() can still
 *   fail on the post-WDT-reset boot because the I2C bus may be in a partially-hung
 *   state that clear_bus_init=true does not fully resolve. When sensors_init()
 *   fails it calls indicate_error_fatal(), which itself runs for 500ms before the
 *   WDT fires again, resets TX, and the same cycle repeats — a secondary reset
 *   loop that looks identical to TX being completely dead. Confirmed in hardware:
 *   two test runs (62 min and 5.6 min) both ended with TX permanently silent.
 *
 *   Fix A — indicate_error_fatal() feeds WDT:
 *     The blink loop now feeds the WDT on every iteration. This breaks the secondary
 *     reset loop. TX halts in a permanent visible-fault state (LED blinking at 5Hz)
 *     rather than silently WDT-looping. A blinking LED after a freeze means sensors
 *     failed to re-init and a manual power cycle is needed; no LED means TX is
 *     transmitting normally (heartbeat LED is 1Hz).
 *
 *   Fix B — initial delay increased from 100ms to 500ms:
 *     After a WDT reset, sensor peripherals and the I2C bus may need longer to
 *     stabilise than on a clean power-on. 500ms is conservative. Feed #1 covers
 *     this delay (writing RR[0] on a cold boot with RREN=0 is harmless per PS).
 *
 *   Fix C — sensors_init() retried once with 2s delay:
 *     If sensors_init() fails on the first attempt, wait 2 seconds (two WDT feeds,
 *     1s each) and retry. The 2s wait gives the I2C bus a longer recovery window
 *     than the 500ms initial delay alone provides. If the retry also fails, TX
 *     halts in indicate_error_fatal() (which now feeds WDT — see Fix A).
 *     If the retry succeeds, TX logs "Sensor init succeeded on retry" and continues
 *     into normal operation.
 *
 * Post-v3.18 functional fix (main.c) -- emergency_shutdown() blink loop:
 *   - emergency_shutdown(): wdt_feed() added inside blink loop. The blinks run
 *     for 2000ms; WDT timeout is 500ms with SLEEP=Run (counts through
 *     nrf_delay_ms). Without feeding, the WDT fires at ~blink 3 and resets the
 *     device before NRF_POWER->SYSTEMOFF = 1 is reached, making battery/thermal
 *     emergency shutdown permanently ineffective. Feeding the WDT during
 *     intentional diagnostic work is appropriate — the WDT exists to catch
 *     unintentional stalls, not deliberate sequences.
 *
 * Code-review fixes (main.c):
 *   - HFCLK_STARTUP_TIMEOUT_MS: 10ms -> 100ms.
 *     nRF52840 HFXO datasheet start time is up to 6ms. 10ms left <4ms margin;
 *     the same bug was previously identified and fixed on RX (same value, same
 *     chip). A cold start at the slow end of the datasheet spec could exceed
 *     10ms and fire the GPREGRET2=0x04 fatal halt path. 100ms matches the RX fix.
 *   - emergency_shutdown(): bounded EVENTS_DISABLED poll added between
 *     TASKS_DISABLE and SYSTEMOFF. If the radio is mid-TX when a thermal or
 *     battery emergency fires, entering SYSTEMOFF with DMA active leaves the
 *     peripheral in an undefined state. 1ms timeout (>2x the ~400us on-air time
 *     for an 86-byte packet at 2Mbps). SYSTEMOFF proceeds regardless of timeout
 *     — a thermal/battery emergency cannot wait indefinitely.
 *
 * Post-v3.18 comment fixes (no functional changes):
 *   - LIS3MDL freshness fraction corrected: "~every 2nd packet" -> "~3 in 5 packets"
 *   - transmit_status_packet() renamed log_status_rtt() (function never transmitted
 *     over radio or USB; name implied wire transmission that does not occur)
 *   - WDT SLEEP=Run block comment corrected: cites TWI/WFI stall (Errata 89/121)
 *     as the reason SLEEP=Run is required, not nrf_delay_ms
 *   - WDT SLEEP=Run inline comment in wdt_init(): same correction applied
 *   - indicate_error_fatal(): LED on solid (not 5Hz blink) -- comment corrected
 *   - log_status_rtt(): "USB-only mode" label added when battery_present is false
 *   - BMP581 ODR comment: "(BDU safe)" replaced — BDU is ST-specific; BMP581 is
 *     Bosch with no BDU mechanism
 *   - radio_timeouts comment: corrected "saturates" to "wraps"
 *   - RTT startup banner updated to "v3.18+fixes"
 *
 * Changelog from 3.17:
 *   - wdt_init() moved to after the 500ms LED flash.
 *
 * Changelog from 3.16:
 *   - wdt_init(): Hardware watchdog added. 500ms timeout, SLEEP=Run, HALT=Pause.
 *
 * Changelog from 3.15:
 *   - read_battery_voltage(): EVENTS_STARTED cleared on STARTED timeout path.
 *   - sensors.c fsr_read() #if 0 Phase 3 block: EVENTS_END and EVENTS_STARTED
 *     cleared on all timeout paths.
 *
 * Changelog from 3.14:
 *   - TX_INTERVAL_MS renamed to TX_INTERVAL_TICKS.
 *   - Timing wrap arithmetic: intermediate uint16_t cast at both sites.
 *   - saadc_stop_and_wait(): EVENTS_END = 0 added after EVENTS_STOPPED.
 *
 * Changelog from 3.12:
 *   - log_status_rtt(): temperature display corrected for sub-zero fractional values.
 *   - prepare_packet(): corrected comment on gap-fill interpolation.
 *
 * Changelog from 3.9:
 *   - radio_init(): PREFIX0 added to readback verification.
 *
 * Changelog from 3.6:
 *   - radio_init(): CRCPOLY and CRCINIT added to readback verification.
 *   - sensors_test() added at startup.
 *   - get_timestamp_ms() renamed to get_rtc_ticks().
 *
 * Changelog from 3.3:
 *   - radio_init(): power cycle delay increased 10us -> 1ms.
 *   - transmit_packet(): __DMB() added before TASKS_TXEN.
 *   - recover_radio(): polls EVENTS_DISABLED; removed indicate_error_radio().
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_saadc.h"
#include "sensors.h"
// packet_spec.h included via sensors.h
#include "SEGGER_RTT.h"
#include "app_error.h"     // Required for app_error_fault_handler() override below

extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);

// Forward declaration: indicate_error_fatal() is defined in the ERROR INDICATION
// section below, but is called earlier by timing_init() (LFCLK timeout) and by
// the HFCLK startup block in main(). C requires a declaration before first use.
static void indicate_error_fatal(void);

// Declared here (before IRQ handlers) so WDT_IRQHandler can reference it.
// Full documentation in the GLOBAL STATE section below.
volatile uint8_t g_wdt_context = 0;

// ============================================================================
// HARDFAULT HANDLER
//
// The default Cortex-M4 HardFault vector is a tight infinite loop. Without
// a handler, any fault (stack overflow, bad pointer, illegal instruction)
// leaves TX permanently frozen -- WDT never fires (if pre-wdt_init()) or
// fires and resets but the underlying corruption may cause another fault
// immediately. Either way TX is silent until power cycled.
//
// GPREGRET (general purpose retained register) survives a soft reset but
// not a power cycle. Writing 0xAA here lets main() distinguish a HardFault
// reset from a WDT reset or clean boot on the next boot, giving visibility
// into the fault via RTT the next time J-Link connects.
//
// Root cause being addressed: TWIM EasyDMA writes the caller's stack buffer
// asynchronously after the calling stack frame may have unwound (e.g. on
// twi_wait() timeout the uninit is not instant). DMA writes into freed stack
// space, corrupts return addresses, and the fault occurs on function return.
// The static RX buffer fix in sensors.c (s_rx_buf) eliminates the underlying
// cause; this handler ensures TX recovers even if another fault path exists.
// ============================================================================

void HardFault_Handler(void)
{
    NRF_POWER->GPREGRET = 0xAA;  // survives soft reset, not power cycle
    NVIC_SystemReset();
    while (1);  // unreachable
}

// ============================================================================
// SDK FAULT HANDLER OVERRIDE (sdk-assert)
//
// The SDK's weak default app_error_fault_handler() calls NVIC_SystemReset()
// with no GPREGRET write. Any NRFX_ASSERT() inside the TWI driver (the most
// likely trigger is NRF_ERROR_INVALID_STATE = 0x08, fired when a stale TWIM
// IRQ arrives between nrf_drv_twi_uninit() and nrf_drv_twi_init() in the
// twi_wait() recovery window) previously produced:
//   RESET REASON: soft reset (NVIC_SystemReset)
//   Previous boot: no failure signature (power cycle or clean)
// ...with no recoverable diagnostic information.
//
// This override writes 0xBB to GPREGRET and the fault id low byte to GPREGRET2
// before resetting. The next boot banner then prints:
//   RESET REASON: soft reset (NVIC_SystemReset)
//   *** PREVIOUS BOOT: SDK fault (app_error_fault_handler) id=0x08 ***
//
// NVIC_DisableIRQ / NVIC_ClearPendingIRQ in twi_wait() is the primary fix
// for the stale-IRQ root cause. This handler is the diagnostic backstop.
//
// app_error.h declares the prototype. The SDK links this definition instead
// of its weak default. (void) casts suppress unused-parameter warnings.
// ============================================================================

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    (void)pc; (void)info;
    NRF_POWER->GPREGRET  = 0xBB;
    NRF_POWER->GPREGRET2 = (uint8_t)(id & 0xFF);
    NVIC_SystemReset();
    while (1);  // unreachable
}

/**
 * WDT timeout interrupt handler.
 *
 * The nRF52840 WDT generates this interrupt approximately 4 LFCLK cycles
 * (~122us) before the actual reset. That window is sufficient to write one
 * register. g_wdt_context holds the last-updated location marker from the
 * main loop or twi_wait() recovery path; writing it to GPREGRET lets the
 * next boot's banner decode exactly where the 500ms stall occurred.
 *
 * GPREGRET survives WDT resets (but not power cycles). It is read and
 * cleared early in main() alongside RESETREAS and GPREGRET2.
 *
 * Priority 7 (lowest): the handler only writes one peripheral register and
 * must not preempt time-critical ISRs such as the TWIM IRQ (priority 2).
 */
void WDT_IRQHandler(void)
{
    NRF_POWER->GPREGRET = g_wdt_context;
    // EVENTS_TIMEOUT need not be cleared -- reset follows in ~122us regardless.
}

// ============================================================================
// CONFIGURATION
// ============================================================================

#define LED_PIN     15          // P1.15

// Battery voltage sense pin.
// The Adafruit Feather nRF52840 has a hardwired 150K/150K voltage divider
// between VBAT and P0.29 (A6/VDIV/AIN5). The mid-point of this divider
// is permanently connected to P0.29 -- there is no enable switch or FET.
// Reading AIN5 gives VBAT/2; the formula multiplies by 2 to recover VBAT.
//
// P0.31 (A7/AREF/AIN7) is NOT the battery pin. It is the analog reference
// input for the COMP peripheral and is explicitly not available for ADC use
// per Adafruit documentation. Previous code read AIN7 by mistake, producing
// the floating 193mV-820mV drifting readings observed across all sessions.
#define VBAT_PIN    NRF_SAADC_INPUT_AIN5    // P0.29 -- VBAT/2 mid-point

#define BALL_ID                 1           // Override via BLE provisioning (Phase 3)

#define TX_INTERVAL_TICKS       5           // ~199Hz nominal (actual ~198.6Hz; 5 RTC ticks at 992.97Hz = 5.035ms)
                                            // Reduced from 4 (250Hz) to support 3-ball USB throughput.
                                            // At 250Hz x 3 balls x 89 bytes = 66,750 bytes/sec > USB FS ceiling (64,000).
                                            // At 199Hz x 3 balls x 89 bytes = 53,133 bytes/sec = 83% of ceiling.
#define STATUS_INTERVAL_SEC     120         // Status data logged to RTT every 2 minutes
// RTC1 runs at 32768/(PRESCALER+1) = 32768/33 = 992.97 Hz.
// STATUS_INTERVAL_TICKS avoids a 32-bit divide in the hot loop.
#define STATUS_INTERVAL_TICKS   ((STATUS_INTERVAL_SEC * 32768U) / 33U)  // ~119156

#define BATTERY_SHUTDOWN_MV     3300
#define TEMP_EMERGENCY_SHUTDOWN 6000        // 60.00 C in 0.01 C units
#define BATTERY_USB_THRESHOLD   1000        // Below 1V = USB-only, no battery
#define BATTERY_FULL_MV         4200
// BATTERY_EMPTY_MV and BATTERY_SHUTDOWN_MV are intentionally the same value.
// The shutdown test in log_status_rtt() is strict less-than (<3300mV),
// so exactly 3300mV does not trigger shutdown. estimate_battery_percent()
// returns 0 for <=3300mV. Do not adjust one constant independently of the
// other — changing either moves the zero-percent / shutdown boundary.
#define BATTERY_EMPTY_MV        3300

// RF constants and packet structures are in packet_spec.h (via sensors.h)

#define RADIO_READY_TIMEOUT_US  1000        // 1ms: ramp-up from DISABLED
#define RADIO_TIMEOUT_US        5000        // 5ms: TX completion and DISABLE
#define TX_POWER                RADIO_TXPOWER_TXPOWER_Pos8dBm

#define HFCLK_STARTUP_TIMEOUT_MS    100     // 100ms: HFXO can take up to 6ms per nRF52840 PS;
                                            // 10ms was the original value but <4ms margin caused
                                            // false fatal halts (same bug fixed to 100ms on RX).
#define LFCLK_STARTUP_TIMEOUT_MS    1000    // 1000ms: LFXO (200-600ms typical)

// Watchdog timeout.
// The WDT is fed once per main loop iteration (~5ms at ~199Hz). 500ms gives
// ~100 loop iterations of margin. Any stall longer than 500ms is pathological
// and warrants a reset. The nRF52840 WDT cannot be stopped once started.
#define WDT_TIMEOUT_MS          500         // 500ms: ~100 loop iterations at ~199Hz

// ============================================================================
// RADIO STATE
// ============================================================================

typedef enum {
    RADIO_STATE_OK = 0,
    RADIO_STATE_TX_TIMEOUT,
    RADIO_STATE_DISABLE_TIMEOUT,
    RADIO_STATE_STARTUP_FAILED
} radio_state_t;

// ============================================================================
// GLOBAL STATE
// ============================================================================

static radio_packet_t tx_packet;
static uint16_t packet_sequence = 0;
static uint16_t status_sequence = 0;
static sensor_data_t sensor_history[3]; // [0]=current, [1]=t-5ms, [2]=t-10ms

static volatile radio_state_t radio_status = RADIO_STATE_OK;
static uint32_t tx_timeout_count = 0;
static uint32_t total_packets_sent = 0;

// last_status_rtc_tick: RTC COUNTER value at the last log_status_rtt() call.
// Zero-initialised; the first status log fires at boot + 2min.
static uint32_t last_status_rtc_tick = 0;

// g_wdt_context: declared before the IRQ handlers (above) so WDT_IRQHandler
// can reference it at link time. Updated at key points in the main loop and
// inside twi_wait() in sensors.c. WDT_IRQHandler writes its value to GPREGRET
// in the ~122us window before the WDT reset fires, so the next boot's banner
// reports exactly where the stall occurred without needing J-Link connected
// during the dropout.
//
// Main loop values:
//   0x01 = top of loop, about to sensors_read()
//   0x02 = inside sensors_read() / TWI operations
//   0x03 = transmit_packet() or recover_radio()
//   0x04 = log_status_rtt()
//   0x05 = timing delay (nrf_delay_ms)
//
// twi_wait() timeout path values (sensors.c):
//   0x10 = timeout detected, entering recovery
//   0x11 = polling EVENTS_STOPPED after TASKS_STOP
//   0x12 = inside nrf_drv_twi_uninit()
//   0x13 = inside i2c_bus_recover()
//   0x14 = inside nrf_drv_twi_init() reinit

// ============================================================================
// TIMING  (RTC1, ~1ms ticks)
// ============================================================================

#define RTC1_TICKS_PER_SEC  993U    // 32768/33 = 992.97 Hz, rounded to nearest integer

static void timing_init(void)
{
    // Stop RTC1 and LFCLK unconditionally before configuring.
    // Root cause of COUNTER=0 bug: the TWI driver starts LFCLK, but the SDK
    // clock module may release the clock request immediately after TWI init.
    // LFCLKSTAT still reads "running" briefly (propagation delay) so we skip
    // the restart -- but LFCLK stops before TASKS_START fires and RTC gets
    // no clock input. PRESCALER=32 confirms the register write succeeded, but
    // COUNTER stays 0. Confirmed by diagnostic: PRESCALER=32, COUNTER=0 after
    // 100ms. Fix: always stop everything and restart from scratch.
    NRF_RTC1->TASKS_STOP = 1;
    nrf_delay_ms(2);
    NRF_RTC1->TASKS_CLEAR = 1;
    nrf_delay_ms(2);

    // Stop LFCLK so we own the restart sequence cleanly.
    NRF_CLOCK->TASKS_LFCLKSTOP = 1;
    nrf_delay_ms(5);

    // Write PRESCALER while RTC1 stopped and LFCLK stopped -- guaranteed to take.
    NRF_RTC1->PRESCALER = 32;  // 32768/(32+1) = 992.97 Hz (~1ms per tick)

    // Start LFCLK from crystal and wait for stable.
    // LFXO takes 200-600ms to stabilise. Feed WDT on every iteration:
    // the last WDT feed before timing_init() may have been mid-way through
    // BMP581 Step 6 polling, leaving less than 500ms remaining. Without a
    // feed here the WDT fires during LFCLK startup on the first post-reset
    // boot, causing another WDT reset before TX ever reaches the main loop.
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    uint32_t t = LFCLK_STARTUP_TIMEOUT_MS;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0 && t > 0) {
        NRF_WDT->RR[0] = WDT_RR_RR_Reload;
        nrf_delay_ms(1);
        t--;
    }
    if (t == 0) {
        SEGGER_RTT_printf(0, "FATAL: LFCLK failed to start\r\n");
        NRF_POWER->GPREGRET2 = 0x05;
        indicate_error_fatal();
    }

    // LFCLK stable -- start RTC1.
    NRF_RTC1->TASKS_START = 1;
    nrf_delay_ms(20);

    uint32_t ctr = NRF_RTC1->COUNTER;
    SEGGER_RTT_printf(0, "LFCLK+RTC1: PRESCALER=%lu COUNTER=%lu%s\r\n",
        (unsigned long)NRF_RTC1->PRESCALER, (unsigned long)ctr,
        ctr == 0 ? " *** COUNTER STUCK -- RTC NOT TICKING ***" : " OK");
}

static inline uint16_t get_rtc_ticks(void)
{
    return (uint16_t)(NRF_RTC1->COUNTER & 0xFFFF);
}

static uint32_t get_uptime_seconds(void)
{
    return (uint32_t)(NRF_RTC1->COUNTER / RTC1_TICKS_PER_SEC);
}

// ============================================================================
// WATCHDOG  (nRF52840 WDT peripheral)
//
// 500ms timeout, SLEEP=Run (required for TWI/Errata 89/121 stall detection),
// HALT=Run (WDT counts identically with and without J-Link — see wdt_init() comment).
//
// WDT reset loop protection — feeds placed at eight sites:
//   1. First instruction of main(): covers a boot following a WDT reset, before
//      any init work consumes time. Gives 500ms from this point.
//   1b. Immediately after the 500ms startup delay: the delay consumes the entire
//      500ms window from feed #1, leaving zero budget for the rest of init. This
//      feed gives sensors_init(), HFCLK startup, radio_init(), and the Fix C
//      retry block a fresh 500ms window. Without this feed, TX enters an infinite
//      WDT reset loop after the first WDT-triggered reset: feed #1 fires, 500ms
//      elapses, WDT fires before sensors_init() completes, TX reboots, repeats
//      forever. The red LED never blinks; TX is permanently silent. This is the
//      root cause of the "works for a while, then dies permanently" dropout.
//   2. Immediately before the 500ms LED flash: ensures the flash completes even
//      if sensors_init() consumed most of the 500ms window from feed #1b.
//   3. Inside init_bmp581() in sensors.c: covers the Step 1/4/5 delays and the
//      Step 6 data-ready retry loop (up to 500ms total).
//   4. Inside indicate_error_fatal() blink loop: prevents WDT from firing during
//      the intentional visible-fault state (LED blinking at 5Hz).
//   5. Inside emergency_shutdown() blink loop: prevents WDT from firing before
//      NRF_POWER->SYSTEMOFF = 1 is reached (blink sequence is 2000ms > 500ms).
//   6. Fix C retry block: two 1s feeds while waiting 2s before sensors_init()
//      retry (2s wait is 4x WDT_TIMEOUT_MS without feeds).
//   7. Inside twi_wait() timeout path in sensors.c: feeds WDT immediately when
//      a TWI transaction times out, before uninit/recover/reinit work begins.
//      The recovery sequence has unpredictable duration when the TWIM peripheral
//      is in a deeply hung state; without this feed the WDT fires during a
//      known recovery window rather than an uncontrolled stall.
//
// All feeds are at explicit known-duration delays or known recovery sequences.
// A genuine uncontrolled stall (no nrf_delay_ms, no recovery work) is still
// caught by the WDT as intended.
// ============================================================================

static void wdt_init(void)
{
    // CRV = (timeout_ms * 32768 / 1000) - 1
    // At 500ms: (500 * 32768 / 1000) - 1 = 16383
    NRF_WDT->CRV = (WDT_TIMEOUT_MS * 32768UL / 1000UL) - 1UL;

    NRF_WDT->CONFIG =
        (WDT_CONFIG_SLEEP_Run  << WDT_CONFIG_SLEEP_Pos) |  // count during WFI/sleep
        (WDT_CONFIG_HALT_Run   << WDT_CONFIG_HALT_Pos);    // KEEP RUNNING during J-Link session
        // *** HALT=Run is intentional ***
        // HALT=Pause pauses the WDT whenever J-Link asserts DBGPWRUPREQ, which
        // it does continuously during any active RTT or debug session -- not only
        // when the CPU is at a breakpoint. This means the WDT is effectively
        // disabled any time J-Link is connected, making it impossible to reproduce
        // WDT-triggered dropouts under the debugger. With HALT=Run, the WDT
        // behaves identically with and without J-Link. If a breakpoint is hit
        // during development, feed the WDT manually or use a longer CRV.

    NRF_WDT->RREN = WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos;

    // Enable WDT timeout interrupt so WDT_IRQHandler can write g_wdt_context
    // to GPREGRET in the ~122us window before the reset fires.
    NRF_WDT->INTENSET = WDT_INTENSET_TIMEOUT_Msk;
    NVIC_SetPriority(WDT_IRQn, 7);
    NVIC_EnableIRQ(WDT_IRQn);

    // Start. Cannot be stopped after this point.
    NRF_WDT->TASKS_START = 1;

    // Feed immediately so the first timeout window starts from now.
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;

    SEGGER_RTT_printf(0, "WDT started (timeout %ums)\r\n", WDT_TIMEOUT_MS);
}

static inline void wdt_feed(void)
{
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

// ============================================================================
// BATTERY MONITORING  (SAADC channel 0, 12-bit)
// ============================================================================

static void battery_init(void)
{
    // The Adafruit Feather nRF52840 has a hardwired 150K/150K voltage divider
    // between VBAT and P0.29 (AIN5). The divider is always connected -- there
    // is no enable GPIO or switching FET on this board. Reading AIN5 gives
    // VBAT/2; read_battery_voltage() multiplies by 2 via the 1758 factor.
    //
    // SAADC configuration: gain 1/6, internal 0.6V reference -> full scale 3.6V.
    // At full charge (~4.2V): V_in = 2.1V, ADC = 2389 counts -> 4201mV. ✓
    // At shutdown (~3.3V):    V_in = 1.65V, ADC = 1877 counts -> 3301mV. ✓
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit;
    NRF_SAADC->CH[0].CONFIG =
        (SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos) |
        (SAADC_CH_CONFIG_RESN_Bypass     << SAADC_CH_CONFIG_RESN_Pos) |
        (SAADC_CH_CONFIG_GAIN_Gain1_6    << SAADC_CH_CONFIG_GAIN_Pos) |
        (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
        (SAADC_CH_CONFIG_TACQ_10us       << SAADC_CH_CONFIG_TACQ_Pos) |
        (SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos);
    NRF_SAADC->CH[0].PSELP = VBAT_PIN;
    NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;
    NRF_SAADC->ENABLE = 1;
}

static void saadc_stop_and_wait(void)
{
    NRF_SAADC->TASKS_STOP = 1;
    uint32_t t = 100000;
    while (NRF_SAADC->EVENTS_STOPPED == 0 && t > 0) { t--; }
    NRF_SAADC->EVENTS_STOPPED = 0;
    NRF_SAADC->EVENTS_END = 0;
}

static uint16_t read_battery_voltage(void)
{
    int16_t adc;
    uint32_t timeout;

    NRF_SAADC->RESULT.PTR    = (uint32_t)&adc;
    NRF_SAADC->RESULT.MAXCNT = 1;
    NRF_SAADC->TASKS_START = 1;
    timeout = 100000;
    while (NRF_SAADC->EVENTS_STARTED == 0 && timeout > 0) { timeout--; }
    if (timeout == 0) {
        saadc_stop_and_wait();
        NRF_SAADC->EVENTS_STARTED = 0;
        return 0;
    }
    NRF_SAADC->EVENTS_STARTED = 0;

    NRF_SAADC->TASKS_SAMPLE = 1;
    timeout = 100000;
    while (NRF_SAADC->EVENTS_END == 0 && timeout > 0) { timeout--; }
    if (timeout == 0) { saadc_stop_and_wait(); return 0; }
    NRF_SAADC->EVENTS_END = 0;

    saadc_stop_and_wait();

    if (adc < 0) return 0;
    // mV = adc * (3600mV / 4096) * 2
    // Factor 1758/1000 = 3600*2/4096 rounded to nearest integer.
    // Gain 1/6, 0.6V internal reference -> full scale 3.6V.
    // x2 compensates the 150K/150K (1:1) voltage divider on VBAT.
    return (uint16_t)(((uint32_t)adc * 1758) / 1000);
}

static uint8_t estimate_battery_percent(uint16_t mv)
{
    if (mv >= BATTERY_FULL_MV)  return 100;
    if (mv <= BATTERY_EMPTY_MV) return 0;
    return (uint8_t)(((uint32_t)(mv - BATTERY_EMPTY_MV) * 100) /
                      (BATTERY_FULL_MV - BATTERY_EMPTY_MV));
}

// ============================================================================
// STATUS PACKET
// ============================================================================

static uint8_t calculate_checksum(const status_packet_t *p)
{
    // XOR all bytes except the last (checksum field itself).
    // DEPENDENCY: checksum must remain the final field in status_packet_t.
    const uint8_t *b = (const uint8_t *)p;
    uint8_t cs = 0;
    for (size_t i = 0; i < sizeof(status_packet_t) - 1; i++) cs ^= b[i];
    return cs;
}

static void emergency_shutdown(void)
{
    SEGGER_RTT_printf(0, "\r\n!!! EMERGENCY SHUTDOWN !!!\r\n");
    // Feed WDT during the blink sequence. The blinks take 2000ms; WDT timeout
    // is 500ms with SLEEP=Run. Without feeding here, the WDT fires at ~blink 3
    // and resets the device before SYSTEMOFF is reached, defeating the shutdown.
    for (int i = 0; i < 10; i++) {
        wdt_feed();
        NRF_P1->OUTSET = (1 << LED_PIN); nrf_delay_ms(100);
        NRF_P1->OUTCLR = (1 << LED_PIN); nrf_delay_ms(100);
    }
    NRF_SAADC->ENABLE = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    // Wait for radio to reach DISABLED before entering SYSTEMOFF. If the radio
    // is mid-TX and we enter SYSTEMOFF with DMA active, the peripheral is left
    // in an undefined state. Bounded at 1ms (TX completes in ~400us at 2Mbps);
    // if it does not complete in time we proceed anyway -- SYSTEMOFF is not
    // contingent on peripheral state, and a thermal/battery emergency cannot wait.
    {
        uint32_t t = 1000;
        while (!NRF_RADIO->EVENTS_DISABLED && t > 0) { nrf_delay_us(1); t--; }
        NRF_RADIO->EVENTS_DISABLED = 0;
    }
    NRF_POWER->SYSTEMOFF = 1;
    while (1);
}

static void log_status_rtt(void)
{
    status_packet_t s;
    s.ball_id     = BALL_ID;
    s.sequence    = status_sequence++;
    s.timestamp   = get_rtc_ticks();
    s.packet_type = PACKET_TYPE_STATUS;

    int16_t temp = sensors_read_temperature();
    s.temperature = (temp != SENSORS_TEMP_UNAVAILABLE) ? temp : 0;

    s.battery_voltage    = read_battery_voltage();
    s.battery_percent    = estimate_battery_percent(s.battery_voltage);
    s.sensor_health      = sensors_get_status_bitmask();
    s.total_packets_sent = total_packets_sent;
    s.uptime_seconds     = get_uptime_seconds();
    // radio_timeouts field is uint16_t; tx_timeout_count is uint32_t.
    // The cast wraps (not saturates) at 65535; full count in tx_timeout_count.
    s.radio_timeouts     = (uint16_t)tx_timeout_count;
    s.i2c_errors         = sensors_get_i2c_error_count();
    s.reserved           = 0;
    s.checksum           = calculate_checksum(&s);

    bool battery_present = (s.battery_voltage > BATTERY_USB_THRESHOLD);
    if ((battery_present && s.battery_voltage < BATTERY_SHUTDOWN_MV) ||
        s.temperature >= TEMP_EMERGENCY_SHUTDOWN) {
        if (battery_present && s.battery_voltage < BATTERY_SHUTDOWN_MV)
            SEGGER_RTT_printf(0, "EMERGENCY: Battery %u mV\r\n", s.battery_voltage);
        if (s.temperature >= TEMP_EMERGENCY_SHUTDOWN)
            SEGGER_RTT_printf(0, "EMERGENCY: Temp %s%d.%02d C\r\n",
                s.temperature < 0 ? "-" : "",
                abs(s.temperature) / 100, abs(s.temperature) % 100);
        emergency_shutdown();
    }

    SEGGER_RTT_printf(0, "\r\n=== STATUS #%u ===\r\n", s.sequence);
    if (battery_present) {
        SEGGER_RTT_printf(0, "Battery: %u mV (%u%%)\r\n", s.battery_voltage, s.battery_percent);
    } else {
        SEGGER_RTT_printf(0, "Battery: %u mV (USB-only)\r\n", s.battery_voltage);
    }
    if (temp != SENSORS_TEMP_UNAVAILABLE) {
        SEGGER_RTT_printf(0, "Temp:    %s%d.%02d C\r\n",
            s.temperature < 0 ? "-" : "",
            abs(s.temperature) / 100, abs(s.temperature) % 100);
    } else {
        SEGGER_RTT_printf(0, "Temp:    unavailable (BMP581 absent)\r\n");
    }
    SEGGER_RTT_printf(0, "Uptime:  %u s (%u:%02u:%02u)\r\n",
        s.uptime_seconds,
        s.uptime_seconds / 3600,
        (s.uptime_seconds % 3600) / 60,
        s.uptime_seconds % 60);
    SEGGER_RTT_printf(0, "Sensors: 0x%02X", s.sensor_health);
    if (s.sensor_health & SENSOR_LSM6_OK)  SEGGER_RTT_printf(0, " LSM6");
    if (s.sensor_health & SENSOR_LIS3_OK)  SEGGER_RTT_printf(0, " LIS3");
    if (s.sensor_health & SENSOR_H3LIS_OK) SEGGER_RTT_printf(0, " H3LIS");
    if (s.sensor_health & SENSOR_BMP_OK)   SEGGER_RTT_printf(0, " BMP");
    SEGGER_RTT_printf(0, "\r\n");
    SEGGER_RTT_printf(0, "Packets: %u sent, %u timeouts, %u I2C errors\r\n",
        s.total_packets_sent, s.radio_timeouts, s.i2c_errors);
    SEGGER_RTT_printf(0, "Next status in %u s\r\n\r\n", STATUS_INTERVAL_SEC);
}

// ============================================================================
// RADIO
// ============================================================================

static bool radio_init(void)
{
    NRF_RADIO->POWER = 0; nrf_delay_ms(1);
    NRF_RADIO->POWER = 1; nrf_delay_ms(1);

    if (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) {
        SEGGER_RTT_printf(0, "RADIO: unexpected state 0x%lX after power cycle\r\n",
            NRF_RADIO->STATE);
        return false;
    }

    NRF_RADIO->MODE      = RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos;
    NRF_RADIO->TXPOWER   = TX_POWER;
    NRF_RADIO->FREQUENCY = RF_CHANNEL;
    NRF_RADIO->PCNF0 = 0;
    NRF_RADIO->PCNF1 =
        (PACKET_PAYLOAD_SIZE << RADIO_PCNF1_MAXLEN_Pos)  |
        (PACKET_PAYLOAD_SIZE << RADIO_PCNF1_STATLEN_Pos) |
        (4 << RADIO_PCNF1_BALEN_Pos)                     |
        (RADIO_PCNF1_ENDIAN_Little   << RADIO_PCNF1_ENDIAN_Pos)  |
        (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);

    NRF_RADIO->BASE0       = RADIO_BASE_ADDR;
    NRF_RADIO->PREFIX0     = RADIO_PREFIX_ADDR;
    NRF_RADIO->TXADDRESS   = 0;
    NRF_RADIO->RXADDRESSES = 1;

    NRF_RADIO->CRCCNF  = (RADIO_CRCCNF_LEN_Three    << RADIO_CRCCNF_LEN_Pos) |
                         (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCPOLY = CRC_POLYNOMIAL;
    NRF_RADIO->CRCINIT = CRC_INIT_VALUE;
    NRF_RADIO->PACKETPTR = (uint32_t)&tx_packet;

    NRF_RADIO->SHORTS =
        (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);

    bool mode_ok     = (NRF_RADIO->MODE == (RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos));
    bool freq_ok     = (NRF_RADIO->FREQUENCY == RF_CHANNEL);
    bool payload_ok  = (((NRF_RADIO->PCNF1 >> RADIO_PCNF1_STATLEN_Pos) & 0xFF) == PACKET_PAYLOAD_SIZE);
    bool addr_ok     = (NRF_RADIO->BASE0 == RADIO_BASE_ADDR);
    bool prefix_ok   = (NRF_RADIO->PREFIX0 == RADIO_PREFIX_ADDR);
    bool crc_poly_ok = (NRF_RADIO->CRCPOLY == CRC_POLYNOMIAL);
    bool crc_init_ok = (NRF_RADIO->CRCINIT == CRC_INIT_VALUE);

    if (!mode_ok)     SEGGER_RTT_printf(0, "RADIO: MODE readback mismatch\r\n");
    if (!freq_ok)     SEGGER_RTT_printf(0, "RADIO: FREQUENCY readback mismatch\r\n");
    if (!payload_ok)  SEGGER_RTT_printf(0, "RADIO: PCNF1 STATLEN readback mismatch\r\n");
    if (!addr_ok)     SEGGER_RTT_printf(0, "RADIO: BASE0 readback mismatch\r\n");
    if (!prefix_ok)   SEGGER_RTT_printf(0, "RADIO: PREFIX0 readback mismatch\r\n");
    if (!crc_poly_ok) SEGGER_RTT_printf(0, "RADIO: CRCPOLY readback mismatch\r\n");
    if (!crc_init_ok) SEGGER_RTT_printf(0, "RADIO: CRCINIT readback mismatch\r\n");

    return (mode_ok && freq_ok && payload_ok && addr_ok && prefix_ok && crc_poly_ok && crc_init_ok);
}

static bool transmit_packet(void)
{
    uint32_t t;

    NRF_RADIO->EVENTS_READY    = 0;
    NRF_RADIO->EVENTS_END      = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;

    __DMB();
    NRF_RADIO->TASKS_TXEN = 1;

    t = RADIO_READY_TIMEOUT_US;
    while (!NRF_RADIO->EVENTS_READY && t > 0) { nrf_delay_us(10); t -= 10; }
    if (!t) {
        SEGGER_RTT_printf(0, "RADIO: never reached READY\r\n");
        radio_status = RADIO_STATE_TX_TIMEOUT; tx_timeout_count++;
        return false;
    }

    t = RADIO_TIMEOUT_US;
    while (!NRF_RADIO->EVENTS_END && t > 0) { nrf_delay_us(10); t -= 10; }
    if (!t) {
        SEGGER_RTT_printf(0, "RADIO: TX timeout (READY ok, END never fired)\r\n");
        radio_status = RADIO_STATE_TX_TIMEOUT; tx_timeout_count++;
        return false;
    }

    t = RADIO_TIMEOUT_US;
    while (!NRF_RADIO->EVENTS_DISABLED && t > 0) { nrf_delay_us(10); t -= 10; }
    if (!t) {
        SEGGER_RTT_printf(0, "RADIO: DISABLE timeout\r\n");
        radio_status = RADIO_STATE_DISABLE_TIMEOUT; tx_timeout_count++;
        return false;
    }

    radio_status = RADIO_STATE_OK;
    return true;
}

// ============================================================================
// ERROR INDICATION
// ============================================================================

static void indicate_error_fatal(void)
{
    // LED on solid + WDT feed loop. Indefinite and unambiguous -- impossible
    // to miss unlike a brief blink burst. If you see the red LED on solid and
    // not blinking, sensors_init() or another init step failed after a reset.
    // Power cycle TX to recover. GPREGRET2 on the next J-Link boot will show
    // which init step failed.
    NRF_P1->OUTSET = (1 << LED_PIN);
    while (1) {
        NRF_WDT->RR[0] = WDT_RR_RR_Reload;
        nrf_delay_ms(100);
    }
}

static void recover_radio(void)
{
    SEGGER_RTT_printf(0, "Radio: attempting recovery...\r\n");

    NRF_RADIO->TASKS_DISABLE = 1;
    uint32_t t = 10000;
    while (!NRF_RADIO->EVENTS_DISABLED && t > 0) { nrf_delay_us(1); t--; }
    if (!t) {
        SEGGER_RTT_printf(0, "Radio: TASKS_DISABLE timed out during recovery\r\n");
    }
    NRF_RADIO->EVENTS_DISABLED = 0;

    if (!radio_init()) {
        radio_status = RADIO_STATE_STARTUP_FAILED;
        SEGGER_RTT_printf(0, "Radio: recovery FAILED -- will retry next TX cycle\r\n");
    } else {
        radio_status = RADIO_STATE_OK;
        SEGGER_RTT_printf(0, "Radio: recovered\r\n");
    }
}

// ============================================================================
// PACKET MANAGEMENT
// ============================================================================

static void update_sensor_history(const sensor_data_t *new_data)
{
    memcpy(&sensor_history[2], &sensor_history[1], sizeof(sensor_data_t));
    memcpy(&sensor_history[1], &sensor_history[0], sizeof(sensor_data_t));
    memcpy(&sensor_history[0], new_data, sizeof(sensor_data_t));
}

static void prepare_packet(void)
{
    tx_packet.ball_id   = BALL_ID;
    tx_packet.sequence  = packet_sequence++;
    tx_packet.timestamp = get_rtc_ticks();
    memcpy(&tx_packet.data_t0, &sensor_history[0], sizeof(sensor_data_t));
    memcpy(&tx_packet.data_t1, &sensor_history[1], sizeof(sensor_data_t));
    memcpy(&tx_packet.data_t2, &sensor_history[2], sizeof(sensor_data_t));
}

// ============================================================================
// MAIN
// ============================================================================

int main(void)
{
    // ---- WDT FEED #1 — first instruction, before any other work ----
    //
    // After a WDT reset the watchdog cannot be stopped and resumes counting
    // from zero at the moment of reset. TX init takes well over 500ms (BMP581
    // Step 6 alone can poll for up to 500ms; the LED flash is 500ms). Without
    // this feed the WDT fires again during init, creating an infinite reset
    // loop that TX cannot escape without a manual power cycle.
    //
    // Writing RR[0] on a cold boot (RREN=0, WDT not yet configured) is
    // harmless per nRF52840 PS — the write is ignored when no reload registers
    // are enabled.
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;

    // ---- RTT non-blocking mode — set before any SEGGER_RTT_printf calls ----
    //
    // Explicitly sets channel 0 to NO_BLOCK_SKIP mode at runtime, regardless
    // of what SEGGER_RTT_Conf.h or the SDK configuration set at compile time.
    // In BLOCK_IF_FIFO_FULL mode, SEGGER_RTT_printf spins waiting for buffer
    // space when no J-Link is connected to drain it. The buffer fills within
    // ~1 second at TX print volume; any subsequent print then blocks forever,
    // firing the WDT after 500ms. TX resets, boot banner immediately fills
    // the buffer again, WDT fires again -- permanent silent reset loop.
    // Setting this at runtime is the only reliable fix: compile-time paths
    // through SEGGER_RTT_Conf.h -> SEGGER_RTT.h -> SEGGER_RTT_MODE_DEFAULT
    // may override sdk_config.h or any other header-level setting.
    SEGGER_RTT_SetFlagsUpBuffer(0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    nrf_delay_ms(500);  // Sensor power-on margin. 100ms is sufficient for a clean
                        // power-on NVM load. After a WDT reset, the I2C bus and
                        // sensor peripherals may need longer to fully stabilise.
                        // 500ms is conservative; feed #1 above covers this delay.

    // ---- WDT FEED #1b — immediately after the 500ms startup delay ----
    //
    // The 500ms delay above consumes the entire 500ms window from feed #1,
    // leaving zero budget for everything that follows. Without this feed, TX
    // enters an infinite WDT reset loop after the first WDT-triggered reset:
    //   feed #1 -> 500ms delay exhausts window -> WDT fires in sensors_init()
    //   -> TX reboots -> feed #1 -> 500ms delay -> WDT fires -> repeat forever.
    // The red LED never blinks; TX is permanently silent until power cycled.
    // This is the "works for a while then dies" dropout root cause.
    //
    // This feed gives sensors_init(), HFCLK startup, radio_init(), and the
    // Fix C retry block a fresh 500ms window. The BMP581 feeds inside
    // sensors.c then cover the long delays within init_bmp581().
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;

    sensor_data_t current_sensors;
    uint32_t loop_count = 0;
    uint16_t current_ticks;
    uint16_t next_tx_time;

    // HFCLK required for radio.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    {
        uint32_t t = HFCLK_STARTUP_TIMEOUT_MS;
        while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0 && t > 0) { nrf_delay_ms(1); t--; }
        if (t == 0) {
            NRF_P1->DIRSET = (1 << LED_PIN);
            SEGGER_RTT_printf(0, "FATAL: HFCLK failed to start\r\n");
            NRF_POWER->GPREGRET2 = 0x04;
            indicate_error_fatal();
        }
    }
    NRF_P1->DIRSET = (1 << LED_PIN);
    NRF_P1->OUTCLR = (1 << LED_PIN);

    // Read reset reason and retained registers together before clearing.
    // GPREGRET: WDT context byte (written by WDT_IRQHandler) or 0xAA (HardFault).
    // GPREGRET2: init-stage failure code written before indicate_error_fatal().
    // All three cleared after reading so stale values don't persist.
    {
        uint32_t rr   = NRF_POWER->RESETREAS;
        uint8_t  gpr  = (uint8_t)(NRF_POWER->GPREGRET  & 0xFF);
        uint8_t  gpr2 = (uint8_t)(NRF_POWER->GPREGRET2 & 0xFF);
        NRF_POWER->RESETREAS = rr;
        NRF_POWER->GPREGRET  = 0;
        NRF_POWER->GPREGRET2 = 0;

        if (rr == 0) {
            SEGGER_RTT_printf(0, "RESET REASON: power-on\r\n");
        } else if (rr & POWER_RESETREAS_DOG_Msk) {
            SEGGER_RTT_printf(0, "RESET REASON: WDT (watchdog) -- loop stall detected\r\n");
            // Decode WDT context captured by WDT_IRQHandler ~122us before reset.
            // 0x00 = interrupt did not fire (WDT not yet started, or context not set).
            // 0xAA = would clash with HardFault signature; treat as not captured.
            if (gpr != 0 && gpr != 0xAA) {
                const char *ctx = "unknown";
                switch (gpr) {
                    case 0x01: ctx = "main loop top -- about to sensors_read()"; break;
                    case 0x02: ctx = "sensors_read() / TWI operations";          break;
                    case 0x03: ctx = "transmit_packet() or recover_radio()";     break;
                    case 0x04: ctx = "log_status_rtt()";                         break;
                    case 0x05: ctx = "timing delay -- nrf_delay_ms()";           break;
                    case 0x10: ctx = "twi_wait(): timeout, entering recovery";   break;
                    case 0x11: ctx = "twi_wait(): TASKS_STOP poll";              break;
                    case 0x12: ctx = "twi_wait(): nrf_drv_twi_uninit()";         break;
                    case 0x13: ctx = "twi_wait(): i2c_bus_recover()";            break;
                    case 0x14: ctx = "twi_wait(): nrf_drv_twi_init() reinit";    break;
                }
                SEGGER_RTT_printf(0, "WDT context: 0x%02X -- %s\r\n", gpr, ctx);
            } else {
                SEGGER_RTT_printf(0, "WDT context: not captured (interrupt did not fire)\r\n");
            }
        } else if (rr & POWER_RESETREAS_SREQ_Msk) {
            SEGGER_RTT_printf(0, "RESET REASON: soft reset (NVIC_SystemReset)\r\n");
            if (gpr == 0xBB) {
                SEGGER_RTT_printf(0, "*** PREVIOUS BOOT: SDK fault (app_error_fault_handler) id=0x%02X ***\r\n", gpr2);
            }
        } else if (rr & POWER_RESETREAS_RESETPIN_Msk) {
            SEGGER_RTT_printf(0, "RESET REASON: reset pin\r\n");
        } else if (rr & POWER_RESETREAS_LOCKUP_Msk) {
            SEGGER_RTT_printf(0, "RESET REASON: CPU lockup\r\n");
        } else {
            SEGGER_RTT_printf(0, "RESET REASON: 0x%08lX\r\n", (unsigned long)rr);
        }

        if (gpr == 0xAA) {
            SEGGER_RTT_printf(0, "*** PREVIOUS BOOT: HardFault ***\r\n");
        }
        // GPREGRET2: written before indicate_error_fatal() in init sequence.
        // 0x01=sensors first  0x02=sensors retry  0x03=radio
        // 0x04=HFCLK timeout  0x05=LFCLK timeout
        // Guarded gpr != 0xBB: when an SDK fault fires, GPREGRET2 holds the
        // fault id (already decoded in the SREQ path above), not an init stage.
        if (gpr2 != 0 && gpr != 0xBB) {
            const char *r = "unknown";
            if      (gpr2 == 0x01) r = "sensors_init() first attempt";
            else if (gpr2 == 0x02) r = "sensors_init() retry";
            else if (gpr2 == 0x03) r = "radio_init()";
            else if (gpr2 == 0x04) r = "HFCLK timeout";
            else if (gpr2 == 0x05) r = "LFCLK timeout";
            SEGGER_RTT_printf(0, "*** PREVIOUS BOOT FAILED AT: %s (0x%02X) ***\r\n", r, gpr2);
        }
        // Clean boot: both registers zero AND not an SDK fault (0xBB with gpr2=0
        // would be an SDK fault with id=0, which is valid and must not be
        // misreported as a clean boot).
        if (gpr == 0 && gpr2 == 0) {
            SEGGER_RTT_printf(0, "Previous boot: no failure signature (power cycle or clean)\r\n");
        }
    }

    SEGGER_RTT_printf(0, "\r\n=== Juggling Ball TX (Ball %d) v3.18+fixes+batt-log+bmp-addr+batt-vdiv-fix2+wdt-ctx+sdk-assert+tx-rate-199 ===\r\n", BALL_ID);
    SEGGER_RTT_printf(0, "Packet sizes:\r\n");
    SEGGER_RTT_printf(0, "  radio_packet_t: %u (expect 86)\r\n",     sizeof(radio_packet_t));
    SEGGER_RTT_printf(0, "  sensor_data_t:  %u (expect 27)\r\n",     sizeof(sensor_data_t));
    SEGGER_RTT_printf(0, "  status_packet_t:%u (expect 26)\r\n\r\n", sizeof(status_packet_t));

    SEGGER_RTT_printf(0, "Initializing sensors...\r\n");
    if (!sensors_init()) {
        // First attempt failed. On a post-WDT-reset boot the I2C bus may be in
        // a partially-hung state that clear_bus_init=true does not fully resolve.
        // Wait 2 seconds (feeding WDT) to allow all peripherals to settle, then
        // retry once. Two feeds are required: the 2s wait is 4x WDT_TIMEOUT_MS.
        NRF_POWER->GPREGRET2 = 0x01;
        SEGGER_RTT_printf(0, "WARNING: sensor init failed -- retrying after 2s\r\n");
        NRF_WDT->RR[0] = WDT_RR_RR_Reload;
        nrf_delay_ms(1000);
        NRF_WDT->RR[0] = WDT_RR_RR_Reload;
        nrf_delay_ms(1000);
        if (!sensors_init()) {
            NRF_POWER->GPREGRET2 = 0x02;
            SEGGER_RTT_printf(0, "FATAL: sensor init failed after retry\r\n");
            indicate_error_fatal();
        }
        SEGGER_RTT_printf(0, "Sensor init succeeded on retry\r\n");
    }
    uint8_t a_lsm6, a_lis3, a_h3lis, a_bmp;
    sensors_get_addresses(&a_lsm6, &a_lis3, &a_h3lis, &a_bmp);
    SEGGER_RTT_printf(0, "Sensors: LSM6=0x%02X LIS3=0x%02X", a_lsm6, a_lis3);
    if (a_h3lis) SEGGER_RTT_printf(0, " H3LIS=0x%02X", a_h3lis);
    if (a_bmp)   SEGGER_RTT_printf(0, " BMP=0x%02X",   a_bmp);
    SEGGER_RTT_printf(0, "\r\n");

    if (!sensors_test()) {
        SEGGER_RTT_printf(0, "WARNING: sensors_test() failed -- one or more sensors may have lost comms after init\r\n");
    } else {
        SEGGER_RTT_printf(0, "sensors_test() OK\r\n");
    }

    timing_init();
    battery_init();
    uint16_t init_batt = read_battery_voltage();
    if (init_batt < BATTERY_USB_THRESHOLD) {
        SEGGER_RTT_printf(0, "Battery: %u mV (USB-only mode)\r\n", init_batt);
    } else {
        SEGGER_RTT_printf(0, "Battery: %u mV (%u%%)\r\n",
            init_batt, estimate_battery_percent(init_batt));
    }

    SEGGER_RTT_printf(0, "Initializing radio (channel %d)...\r\n", RF_CHANNEL);
    if (!radio_init()) {
        NRF_POWER->GPREGRET2 = 0x03;
        SEGGER_RTT_printf(0, "FATAL: radio init failed\r\n");
        indicate_error_fatal();
    }
    SEGGER_RTT_printf(0, "Radio OK\r\n");

    memset(sensor_history, 0, sizeof(sensor_history));
    memset(&tx_packet, 0, sizeof(tx_packet));

    // ---- WDT FEED #2 — immediately before 500ms LED flash ----
    //
    // sensors_init() can consume close to the full 500ms window from feed #1
    // (BMP581 Step 6 alone polls for up to 500ms). Feeding here resets the
    // window so the flash completes safely regardless of how long init took.
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;
    NRF_P1->OUTSET = (1 << LED_PIN); nrf_delay_ms(500);
    NRF_P1->OUTCLR = (1 << LED_PIN);

    // Watchdog — started after the LED flash. Once started, cannot be stopped.
    wdt_init();

    SEGGER_RTT_printf(0, "Transmitting at ~199Hz (5ms). Status every %us.\r\n\r\n",
        STATUS_INTERVAL_SEC);

    next_tx_time = get_rtc_ticks() + TX_INTERVAL_TICKS;

    while (1)
    {
        wdt_feed();
        g_wdt_context = 0x01;  // top of loop -- about to sensors_read()

        current_ticks = get_rtc_ticks();

        if (loop_count % 250 == 0) {
            int32_t diff = (int32_t)(int16_t)(uint16_t)(next_tx_time - current_ticks);
            SEGGER_RTT_printf(0, "DEBUG: loop=%u T=%u next=%u diff=%d\r\n",
                loop_count, current_ticks, next_tx_time, diff);
        }

        g_wdt_context = 0x02;  // sensors_read() / TWI operations
        sensors_read(&current_sensors);
        update_sensor_history(&current_sensors);
        prepare_packet();

        if (loop_count % 250 == 0) {
            int16_t hx = extract_h3lis_axis(current_sensors.h3lis_x_fsr_level);
            int16_t hy = extract_h3lis_axis(current_sensors.h3lis_y_fsr_pattern);
            int16_t hz = extract_h3lis_axis(current_sensors.h3lis_z_flags);
            uint8_t fsr_lvl = current_sensors.h3lis_x_fsr_level  & 0x0F;
            uint8_t fsr_pat = current_sensors.h3lis_y_fsr_pattern & 0x0F;

            SEGGER_RTT_printf(0, "\r\n[Seq %u T=%u]\r\n",
                tx_packet.sequence, tx_packet.timestamp);
            SEGGER_RTT_printf(0, "ACCEL: %6d %6d %6d\r\n",
                current_sensors.imu.accel[0],
                current_sensors.imu.accel[1],
                current_sensors.imu.accel[2]);
            SEGGER_RTT_printf(0, "GYRO:  %6d %6d %6d\r\n",
                current_sensors.imu.gyro[0],
                current_sensors.imu.gyro[1],
                current_sensors.imu.gyro[2]);
            SEGGER_RTT_printf(0, "MAG:   %6d %6d %6d\r\n",
                current_sensors.imu.mag[0],
                current_sensors.imu.mag[1],
                current_sensors.imu.mag[2]);
            SEGGER_RTT_printf(0, "H3LIS: %6d %6d %6d  FSR: lvl=%u pat=0x%X\r\n",
                hx, hy, hz, fsr_lvl, fsr_pat);

            uint32_t praw = (uint32_t)current_sensors.pressure[0]        |
                           ((uint32_t)current_sensors.pressure[1] << 8)  |
                           ((uint32_t)current_sensors.pressure[2] << 16);
            SEGGER_RTT_printf(0, "BMP:   %u Pa\r\n", praw / 64);
        }

        g_wdt_context = 0x03;  // transmit_packet() or recover_radio()
        bool ok = transmit_packet();
        if (loop_count < 10) {
            SEGGER_RTT_printf(0, "TX[%u]: %s\r\n", loop_count, ok ? "OK" : "FAIL");
        }
        if (ok) {
            total_packets_sent++;
        } else {
            SEGGER_RTT_printf(0, "Radio error loop=%u status=%d\r\n",
                loop_count, radio_status);
            recover_radio();
        }

        g_wdt_context = 0x04;  // log_status_rtt() (fires every 2 minutes)
        if (((NRF_RTC1->COUNTER - last_status_rtc_tick) & 0xFFFFFFU)
                >= STATUS_INTERVAL_TICKS) {
            log_status_rtt();
            last_status_rtc_tick = NRF_RTC1->COUNTER;
        }

        loop_count++;

        if (loop_count % 250 == 0) {
            if (NRF_P1->OUT & (1 << LED_PIN))
                NRF_P1->OUTCLR = (1 << LED_PIN);
            else
                NRF_P1->OUTSET = (1 << LED_PIN);
        }

        next_tx_time += TX_INTERVAL_TICKS;
        current_ticks = get_rtc_ticks();
        int32_t time_diff = (int32_t)(int16_t)(uint16_t)(next_tx_time - current_ticks);

        g_wdt_context = 0x05;  // timing delay -- nrf_delay_ms()
        if (time_diff > 0 && time_diff < 100) {
            nrf_delay_ms((uint32_t)time_diff);
        } else if (time_diff >= 100 || time_diff < -100) {
            // Silent resync -- do not print here. This fires ~10x per second
            // when the RTC is misconfigured, flooding the RTT buffer and
            // triggering WDT if RTT is in blocking mode. The per-second DEBUG
            // print at loop%250 shows timing drift if it persists.
            next_tx_time = current_ticks + TX_INTERVAL_TICKS;
        }
    }
}