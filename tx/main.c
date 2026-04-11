/**
 * Juggling Ball Wireless Transmitter
 *
 * 250Hz, 86-byte packets over 2.4GHz custom protocol.
 * Status packets every 2 minutes via RTT (battery, temp, health).
 *
 * Hardware: Adafruit Feather nRF52840
 * Sensors:  LSM6DSOX + LIS3MDL + H3LIS331 + BMP581 + 4xFSR (Phase 3)
 *
 * Sensor ODR vs TX rate at 250Hz:
 *   LSM6DSOX accel/gyro: 416Hz  - 1.66x TX rate, near-full capture
 *   H3LIS331:            400Hz  - 1.6x TX rate, near-full capture
 *   BMP581:              ~218Hz - TX slightly faster, occasional duplicate read (BDU safe)
 *   LIS3MDL:             155Hz  - TX 1.6x faster, ~every 2nd packet has fresh mag data
 *
 * @version 3.9
 *
 * Changelog from 3.8:
 *   - CLOCK_STARTUP_TIMEOUT_MS comment corrected: said "10000 iterations at 1ms
 *     each" — the loop decrements from 10 with nrf_delay_ms(1) per iteration,
 *     giving 10 iterations × 1ms = 10ms total. The value was always correct;
 *     only the comment was wrong.
 *   - current_time_ms renamed to current_ticks throughout main(). The variable
 *     holds the return value of get_rtc_ticks() (uint16_t); keeping the _ms
 *     suffix after the v3.7 function rename was an incomplete refactor.
 *   - indicate_error_fatal() forward declaration added. The function is called by
 *     timing_init() (defined early in the file) but was defined later, with no
 *     prior declaration. C99/C11 constraint violation; GCC warns and compiles but
 *     the forward declaration makes the dependency explicit.
 *   - BATTERY_SHUTDOWN_MV / BATTERY_EMPTY_MV: comment added documenting the
 *     deliberate identity (both 3300mV). The shutdown test is strict-less-than so
 *     exactly 3300mV does not shut down; estimate_battery_percent returns 0 at
 *     <=3300mV. Future editors must not adjust one constant independently of the
 *     other without understanding this boundary.
 *   - sensors_init() LSM6/LIS3 status bits: comment added explaining why config
 *     failure returns false without clearing the status bits — the device halts on
 *     any required sensor failure, so cleanup is unnecessary. This explains the
 *     intentional asymmetry with H3LIS/BMP handling.
 *
 * Changelog from 3.7:
 *   - recover_radio(): radio_status set to RADIO_STATE_OK on successful recovery.
 *     Previously left holding the failure code until the next transmit_packet()
 *     call corrected it — causing stale diagnostic state in any debug session
 *     that reads radio_status immediately after recovery.
 *   - prepare_packet(): comment added noting the ~1ms timestamp lag. The timestamp
 *     is taken after sensor reads complete; at 4ms TX interval this is ~25% stale
 *     relative to actual sample time. Phase 2 gap-fill should use sequence number,
 *     not the timestamp field, for interpolation position.
 *   - calculate_checksum(): comment added documenting the field-order dependency.
 *     The loop covers sizeof-1 bytes, implicitly requiring checksum to be the last
 *     field. The _Static_assert guards size, not order; a field inserted after
 *     checksum without updating the loop would silently cover the wrong bytes.
 *
 * Changelog from 3.6:
 *   - radio_init(): CRCPOLY and CRCINIT added to readback verification. These were
 *     previously unchecked. A wrong CRC polynomial produces a complete silent dead
 *     link: TX transmits, RX drops every packet on CRC failure, no error counted.
 *   - sensors_test() added at startup. The function performs WHO_AM_I re-reads after
 *     init, catching a sensor that passes detection but fails to hold communication.
 *     Result is logged but non-fatal (sensors_init() already verified core sensors).
 *   - get_timestamp_ms() renamed to get_rtc_ticks(). The _ms suffix was a persistent
 *     source of confusion: four changelog entries existed solely to re-clarify that
 *     the function returns RTC ticks, not milliseconds. Renaming eliminates all
 *     compensating comments.
 *   - last_status_rtc_tick moved from static local inside main() to file scope.
 *     Static locals inside non-recursive functions are valid C but hide persistent
 *     state from the global state block and cannot be referenced externally.
 *   - Clock startup (HFCLK and LFCLK) now has explicit timeout guards, consistent
 *     with all other peripheral waits. Both previously spun without limit; a crystal
 *     failure at power-on would hang silently with no LED indication.
 *   - calculate_checksum() signature: status_packet_t* -> const status_packet_t*.
 *     The function only reads; the non-const signature was inaccurate.
 *   - read_battery_voltage() early timeout returns: TASKS_STOP now waits for
 *     EVENTS_STOPPED before returning. Previously, TASKS_STOP was issued and the
 *     function returned immediately, leaving the SAADC in an indeterminate stop
 *     state for the next call. This correctly handles the exceptional case (which
 *     is exactly the case these timeout guards exist to handle).
 *
 * Changelog from 3.5:
 *   - current_time_ms: uint32_t -> uint16_t.
 *   - RADIO_READY_TIMEOUT_US added (1000).
 *   - Debug timing print: removed redundant loop_count==0 clause.
 *   - Version banner updated to v3.6.
 *
 * Changelog from 3.4:
 *   - RTC1_TICKS_PER_SEC constant added (993).
 *   - timing_init() nrf_delay_ms(10) comment added.
 *
 * Changelog from 3.3:
 *   - radio_init(): power cycle delay increased 10us -> 1ms.
 *   - transmit_packet(): __DMB() added before TASKS_TXEN.
 *   - recover_radio(): polls EVENTS_DISABLED rather than sleeping 1ms.
 *   - recover_radio(): removed indicate_error_radio() call.
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

extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);

// Forward declaration: indicate_error_fatal() is defined in the ERROR INDICATION
// section below, but is called earlier by timing_init() (LFCLK timeout) and by
// the HFCLK startup block in main(). C requires a declaration before first use.
static void indicate_error_fatal(void);

// ============================================================================
// CONFIGURATION
// ============================================================================

#define LED_PIN                 15          // P1.15
#define VBAT_PIN                NRF_SAADC_INPUT_AIN7  // P0.31

#define BALL_ID                 1           // Override via BLE provisioning (Phase 3)

#define TX_INTERVAL_MS          4           // 250Hz
#define STATUS_INTERVAL_SEC     120         // Status packet every 2 minutes
// RTC1 runs at 32768/(PRESCALER+1) = 32768/33 = 992.97 Hz.
// STATUS_INTERVAL_TICKS avoids a 32-bit divide in the hot loop.
#define STATUS_INTERVAL_TICKS   ((STATUS_INTERVAL_SEC * 32768U) / 33U)  // ~119156

#define BATTERY_SHUTDOWN_MV     3300
#define TEMP_EMERGENCY_SHUTDOWN 6000        // 60.00 C in 0.01 C units
#define BATTERY_USB_THRESHOLD   1000        // Below 1V = USB-only, no battery
#define BATTERY_FULL_MV         4200
// BATTERY_EMPTY_MV and BATTERY_SHUTDOWN_MV are intentionally the same value.
// The shutdown test in transmit_status_packet() is strict less-than (<3300mV),
// so exactly 3300mV does not trigger shutdown. estimate_battery_percent()
// returns 0 for <=3300mV. Do not adjust one constant independently of the
// other — changing either moves the zero-percent / shutdown boundary.
#define BATTERY_EMPTY_MV        3300

// RF constants and packet structures are in packet_spec.h (via sensors.h)

// Separate timeouts for READY and END/DISABLED:
//   RADIO_READY_TIMEOUT_US  — radio ramp-up from DISABLED is fast; 1ms is
//     conservative and avoids waiting for a peripheral that may not respond.
//   RADIO_TIMEOUT_US        — used for END (TX completion) and DISABLED.
//     5ms accounts for a full 86-byte packet + disable sequence at 2Mbps.
#define RADIO_READY_TIMEOUT_US  1000        // 1ms: ramp-up from DISABLED
#define RADIO_TIMEOUT_US        5000        // 5ms: TX completion and DISABLE
#define TX_POWER                RADIO_TXPOWER_TXPOWER_Pos8dBm

// Clock startup timeouts.
// HFXO (16MHz crystal) starts in <1ms; 10ms is conservative.
// LFXO (32.768kHz crystal) startup time on nRF52840 is typically 200-600ms;
// 1000ms provides a safe margin. The original bare-spin had no timeout at all —
// 1000ms is strictly better while still catching a dead crystal.
#define HFCLK_STARTUP_TIMEOUT_MS    10      // 10ms: HFXO
#define LFCLK_STARTUP_TIMEOUT_MS    1000    // 1000ms: LFXO (200-600ms typical)

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
static sensor_data_t sensor_history[3]; // [0]=current, [1]=t-4ms, [2]=t-8ms

static volatile radio_state_t radio_status = RADIO_STATE_OK;
static uint32_t tx_timeout_count = 0;
static uint32_t total_packets_sent = 0;

// last_status_rtc_tick: RTC COUNTER value at the last status packet transmission.
// Stored at file scope (not as static local in main) so it is visible alongside
// other module state. Zero-initialised; the first status packet fires at boot + 2min.
static uint32_t last_status_rtc_tick = 0;

// ============================================================================
// TIMING  (RTC1, ~1ms ticks)
//
// PRESCALER=32: f = 32768/(32+1) = 992.97 Hz, period = 1.0071 ms
//
// RTC1_TICKS_PER_SEC: 993 is the closest integer to 992.97.
//   - get_rtc_ticks() returns raw RTC COUNTER bits [15:0]. Each tick = ~1.007ms.
//     The counter wraps at ~65535 ticks = ~66.0s. For timing arithmetic, always
//     use uint16_t subtraction so wrapping is well-defined.
//   - get_uptime_seconds() divides by RTC1_TICKS_PER_SEC (993), giving <0.003%
//     error. The previous value of 1000 caused 0.71% fast readout (~12.8s
//     ahead after 30 minutes).
//   - Packet timestamp field carries these ticks. For Phase 2 t1/t2 gap-fill
//     interpolation, treat as ticks (not milliseconds). Each tick = ~1.007ms.
// ============================================================================

#define RTC1_TICKS_PER_SEC  993U    // 32768/33 = 992.97 Hz, rounded to nearest integer

static void timing_init(void)
{
    // nrf_drv_twi_init() (called inside sensors_init()) requests the LFCLK via
    // nrf_drv_clock_lfclk_request(), which may have already started it by the
    // time we get here. If we clear EVENTS_LFCLKSTARTED and reissue TASKS_LFCLKSTART
    // on an already-running clock, the event never re-fires and the timeout triggers.
    // Check LFCLKSTAT.STATE first; only go through the start sequence if needed.
    bool lfclk_already_running =
        (NRF_CLOCK->LFCLKSTAT & (CLOCK_LFCLKSTAT_STATE_Running << CLOCK_LFCLKSTAT_STATE_Pos));

    if (!lfclk_already_running) {
        NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
        NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
        NRF_CLOCK->TASKS_LFCLKSTART = 1;

        uint32_t t = LFCLK_STARTUP_TIMEOUT_MS;
        while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0 && t > 0) { nrf_delay_ms(1); t--; }
        if (t == 0) {
            SEGGER_RTT_printf(0, "FATAL: LFCLK failed to start\r\n");
            indicate_error_fatal();
        }
        SEGGER_RTT_printf(0, "LFCLK started\r\n");
    } else {
        SEGGER_RTT_printf(0, "LFCLK already running (started by TWI driver)\r\n");
    }

    NRF_RTC1->PRESCALER = 32;   // 992.97 Hz (~1ms), 0.71% slow — see note above
    NRF_RTC1->TASKS_START = 1;
    // Allow the counter to advance past 0 before get_rtc_ticks() is first called.
    // Without this, a very fast loop iteration after timing_init() could read
    // COUNTER=0, compute next_tx_time=4, and immediately trigger a timing resync.
    nrf_delay_ms(10);
}

// Returns RTC ticks as uint16_t: wraps at 65535 ticks (~66.0s at 993Hz).
// Each tick = ~1.007ms. Wrap arithmetic is correct by uint16_t subtraction.
// The packet timestamp field carries these ticks directly.
static inline uint16_t get_rtc_ticks(void)
{
    return (uint16_t)(NRF_RTC1->COUNTER & 0xFFFF);
}

// Uptime in seconds. Divides by RTC1_TICKS_PER_SEC (993) for <0.003% error.
// 24-bit counter overflows at ~4.7 hours; produces one spurious status packet
// at that point — acceptable for a performance context.
static uint32_t get_uptime_seconds(void)
{
    return (uint32_t)(NRF_RTC1->COUNTER / RTC1_TICKS_PER_SEC);
}

// ============================================================================
// BATTERY MONITORING  (SAADC channel 0, 12-bit)
// ============================================================================

static void battery_init(void)
{
    // 12-bit resolution. FSR reads (Phase 3) must save/restore this.
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

// Helper: issue TASKS_STOP and wait for EVENTS_STOPPED with timeout, then clear.
// Used by read_battery_voltage() both in the normal exit path and in all early
// timeout returns. Without this wait, TASKS_START on the next call can race a
// still-pending STOP, violating the nRF52840 SAADC sequencing requirement.
static void saadc_stop_and_wait(void)
{
    NRF_SAADC->TASKS_STOP = 1;
    uint32_t t = 100000;
    while (NRF_SAADC->EVENTS_STOPPED == 0 && t > 0) { t--; }
    NRF_SAADC->EVENTS_STOPPED = 0;
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
    if (timeout == 0) { saadc_stop_and_wait(); return 0; }
    NRF_SAADC->EVENTS_STARTED = 0;

    NRF_SAADC->TASKS_SAMPLE = 1;
    timeout = 100000;
    while (NRF_SAADC->EVENTS_END == 0 && timeout > 0) { timeout--; }
    if (timeout == 0) { saadc_stop_and_wait(); return 0; }
    NRF_SAADC->EVENTS_END = 0;

    saadc_stop_and_wait();

    if (adc < 0) return 0;
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
    // XOR all bytes except the last (the checksum field itself).
    // DEPENDENCY: this assumes 'checksum' is the final field in status_packet_t.
    // The _Static_assert in packet_spec.h guards the struct size, not field order.
    // If a field is added after 'checksum', this loop silently covers the wrong
    // bytes. Any structural change to status_packet_t must be verified here.
    const uint8_t *b = (const uint8_t *)p;
    uint8_t cs = 0;
    for (size_t i = 0; i < sizeof(status_packet_t) - 1; i++) cs ^= b[i];
    return cs;
}

static void emergency_shutdown(void)
{
    SEGGER_RTT_printf(0, "\r\n!!! EMERGENCY SHUTDOWN !!!\r\n");
    for (int i = 0; i < 10; i++) {
        NRF_P1->OUTSET = (1 << LED_PIN); nrf_delay_ms(100);
        NRF_P1->OUTCLR = (1 << LED_PIN); nrf_delay_ms(100);
    }
    NRF_SAADC->ENABLE = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    NRF_POWER->SYSTEMOFF = 1;
    while (1);
}

static void transmit_status_packet(void)
{
    status_packet_t s;
    s.ball_id     = BALL_ID;
    s.sequence    = status_sequence++;
    s.timestamp   = get_rtc_ticks();
    s.packet_type = PACKET_TYPE_STATUS;

    // Read temperature; check sentinel before storing.
    // SENSORS_TEMP_UNAVAILABLE (-32768) is below any physical temperature
    // and will not trigger emergency shutdown, but we store 0 in the packet
    // to avoid confusing the PC-side display with an impossible value.
    // Decoders should check the sensor_health bitmask (SENSOR_BMP_OK) to
    // determine whether a temperature of 0 means "0 degrees C" or "BMP absent".
    int16_t temp = sensors_read_temperature();
    s.temperature = (temp != SENSORS_TEMP_UNAVAILABLE) ? temp : 0;

    s.battery_voltage    = read_battery_voltage();
    s.battery_percent    = estimate_battery_percent(s.battery_voltage);
    s.sensor_health      = sensors_get_status_bitmask();
    s.total_packets_sent = total_packets_sent;
    s.uptime_seconds     = get_uptime_seconds();
    // tx_timeout_count is uint32_t; radio_timeouts is uint16_t in the wire format.
    // The narrowing is intentional — the field cannot change without breaking the
    // on-air packet format. At 250Hz, uint16_t saturates after ~262 seconds of
    // consecutive radio failures, which is well past the TX freeze detection point.
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
            SEGGER_RTT_printf(0, "EMERGENCY: Temp %d.%02d C\r\n",
                s.temperature / 100, abs(s.temperature % 100));
        emergency_shutdown();
    }

    SEGGER_RTT_printf(0, "\r\n=== STATUS #%u ===\r\n", s.sequence);
    SEGGER_RTT_printf(0, "Battery: %u mV (%u%%)\r\n", s.battery_voltage, s.battery_percent);
    if (temp != SENSORS_TEMP_UNAVAILABLE) {
        SEGGER_RTT_printf(0, "Temp:    %d.%02d C\r\n",
            s.temperature / 100, abs(s.temperature % 100));
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
    // Power-cycle the radio peripheral.
    // Delay after POWER=0/1 is 1ms. 10us was insufficient for the power domain
    // to stabilise; STATE readback then returned non-DISABLED unconditionally.
    NRF_RADIO->POWER = 0; nrf_delay_ms(1);
    NRF_RADIO->POWER = 1; nrf_delay_ms(1);

    // After power-on, radio must be DISABLED before configuration.
    if (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) {
        SEGGER_RTT_printf(0, "RADIO: unexpected state 0x%lX after power cycle\r\n",
            NRF_RADIO->STATE);
        return false;
    }

    NRF_RADIO->MODE      = RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos;
    NRF_RADIO->TXPOWER   = TX_POWER;
    NRF_RADIO->FREQUENCY = RF_CHANNEL;

    NRF_RADIO->PCNF0 = 0; // No S0, no length field, no S1

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

    // Readback verification.
    // CRCPOLY and CRCINIT are included: a wrong CRC polynomial causes the RX to
    // drop every packet on CRC failure with no error logged on either side.
    bool mode_ok    = (NRF_RADIO->MODE ==
                        (RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos));
    bool freq_ok    = (NRF_RADIO->FREQUENCY == RF_CHANNEL);
    bool payload_ok = (((NRF_RADIO->PCNF1 >> RADIO_PCNF1_STATLEN_Pos) & 0xFF)
                        == PACKET_PAYLOAD_SIZE);
    bool addr_ok    = (NRF_RADIO->BASE0 == RADIO_BASE_ADDR);
    bool crc_poly_ok = (NRF_RADIO->CRCPOLY == CRC_POLYNOMIAL);
    bool crc_init_ok = (NRF_RADIO->CRCINIT == CRC_INIT_VALUE);

    if (!mode_ok)     SEGGER_RTT_printf(0, "RADIO: MODE readback mismatch\r\n");
    if (!freq_ok)     SEGGER_RTT_printf(0, "RADIO: FREQUENCY readback mismatch\r\n");
    if (!payload_ok)  SEGGER_RTT_printf(0, "RADIO: PCNF1 STATLEN readback mismatch\r\n");
    if (!addr_ok)     SEGGER_RTT_printf(0, "RADIO: BASE0 readback mismatch\r\n");
    if (!crc_poly_ok) SEGGER_RTT_printf(0, "RADIO: CRCPOLY readback mismatch\r\n");
    if (!crc_init_ok) SEGGER_RTT_printf(0, "RADIO: CRCINIT readback mismatch\r\n");

    return (mode_ok && freq_ok && payload_ok && addr_ok && crc_poly_ok && crc_init_ok);
}

static bool transmit_packet(void)
{
    uint32_t t;

    NRF_RADIO->EVENTS_READY    = 0;
    NRF_RADIO->EVENTS_END      = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;

    // __DMB() flushes the Cortex-M4 write buffer so all stores to tx_packet are
    // visible to the bus fabric before DMA starts. Must appear after the last
    // write to tx_packet (prepare_packet()) and before TASKS_TXEN.
    __DMB();
    NRF_RADIO->TASKS_TXEN = 1;

    // READY timeout (1ms): radio ramp-up from DISABLED is fast. Deliberately
    // shorter than RADIO_TIMEOUT_US (5ms) used for END/DISABLED.
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
    while (1) {
        NRF_P1->OUTSET = (1 << LED_PIN); nrf_delay_ms(100);
        NRF_P1->OUTCLR = (1 << LED_PIN); nrf_delay_ms(100);
    }
}

// indicate_error_radio() removed in v3.3. See changelog.

static void recover_radio(void)
{
    SEGGER_RTT_printf(0, "Radio: attempting recovery...\r\n");

    // Poll EVENTS_DISABLED before power-cycling. Timeout 10ms is conservative;
    // the disable sequence is typically complete in <1ms.
    NRF_RADIO->TASKS_DISABLE = 1;

    uint32_t t = 10000;  // 10ms in 1us steps
    while (!NRF_RADIO->EVENTS_DISABLED && t > 0) { nrf_delay_us(1); t--; }
    if (!t) {
        SEGGER_RTT_printf(0, "Radio: TASKS_DISABLE timed out during recovery\r\n");
    }
    NRF_RADIO->EVENTS_DISABLED = 0;

    if (!radio_init()) {
        radio_status = RADIO_STATE_STARTUP_FAILED;
        SEGGER_RTT_printf(0, "Radio: recovery FAILED — will retry next TX cycle\r\n");
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
    // Timestamp is taken here, after sensors_read() has completed.
    // I2C reads take ~1ms; at 4ms TX interval the timestamp is ~25% stale
    // relative to the actual sensor sample time. For Phase 2 gap-fill
    // interpolation, use the sequence number rather than the timestamp field
    // to determine packet position — sequence is set before any blocking work.
    tx_packet.timestamp = get_rtc_ticks();
    memcpy(&tx_packet.data_t0, &sensor_history[0], sizeof(sensor_data_t));
    memcpy(&tx_packet.data_t1, &sensor_history[1], sizeof(sensor_data_t));
    memcpy(&tx_packet.data_t2, &sensor_history[2], sizeof(sensor_data_t));
    // __DMB() is called in transmit_packet() before TASKS_TXEN, after all
    // writes to tx_packet are complete.
}

// ============================================================================
// MAIN
// ============================================================================

int main(void)
{
    // Allow time for all sensors to complete power-on sequences before firmware
    // touches the I2C bus. The BMP581 NVM load happens in the first few ms
    // after VDD is stable; without this delay the NVM load can fail if there
    // is noise on the rail at power-on. Confirmed necessary in hardware testing.
    nrf_delay_ms(100);

    sensor_data_t current_sensors;
    uint32_t loop_count = 0;
    uint16_t current_ticks;   // return type of get_rtc_ticks()
    uint16_t next_tx_time;

    // HFCLK required for radio. TX does not use nrf_drv_clock (no USB stack),
    // so direct register access is appropriate here.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    {
        uint32_t t = HFCLK_STARTUP_TIMEOUT_MS;
        while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0 && t > 0) { nrf_delay_ms(1); t--; }
        if (t == 0) {
            // LED init not yet done — drive pin directly before halting.
            NRF_P1->DIRSET = (1 << LED_PIN);
            SEGGER_RTT_printf(0, "FATAL: HFCLK failed to start\r\n");
            indicate_error_fatal();
        }
    }
    // LED
    NRF_P1->DIRSET = (1 << LED_PIN);
    NRF_P1->OUTCLR = (1 << LED_PIN);

    // Startup banner
    SEGGER_RTT_printf(0, "\r\n=== Juggling Ball TX (Ball %d) v3.9 ===\r\n", BALL_ID);
    SEGGER_RTT_printf(0, "Packet sizes:\r\n");
    SEGGER_RTT_printf(0, "  radio_packet_t: %u (expect 86)\r\n",     sizeof(radio_packet_t));
    SEGGER_RTT_printf(0, "  sensor_data_t:  %u (expect 27)\r\n",     sizeof(sensor_data_t));
    SEGGER_RTT_printf(0, "  status_packet_t:%u (expect 26)\r\n\r\n", sizeof(status_packet_t));

    // Sensors (must succeed for LSM6 + LIS3)
    SEGGER_RTT_printf(0, "Initializing sensors...\r\n");
    if (!sensors_init()) {
        SEGGER_RTT_printf(0, "FATAL: sensor init failed\r\n");
        indicate_error_fatal();
    }
    uint8_t a_lsm6, a_lis3, a_h3lis, a_bmp;
    sensors_get_addresses(&a_lsm6, &a_lis3, &a_h3lis, &a_bmp);
    SEGGER_RTT_printf(0, "Sensors: LSM6=0x%02X LIS3=0x%02X", a_lsm6, a_lis3);
    if (a_h3lis) SEGGER_RTT_printf(0, " H3LIS=0x%02X", a_h3lis);
    if (a_bmp)   SEGGER_RTT_printf(0, " BMP=0x%02X",   a_bmp);
    SEGGER_RTT_printf(0, "\r\n");

    // WHO_AM_I re-reads after init, verifying sensors still respond post-config.
    // Non-fatal: sensors_init() already confirmed core sensors; this is a
    // belt-and-suspenders check. A failure here is logged but does not halt.
    if (!sensors_test()) {
        SEGGER_RTT_printf(0, "WARNING: sensors_test() failed — one or more sensors may have lost comms after init\r\n");
    } else {
        SEGGER_RTT_printf(0, "sensors_test() OK\r\n");
    }

    // Timing (LFCLK start with timeout inside timing_init)
    timing_init();

    // Battery (sets SAADC to 12-bit; must run after sensors_init)
    battery_init();
    uint16_t init_batt = read_battery_voltage();
    if (init_batt < BATTERY_USB_THRESHOLD) {
        SEGGER_RTT_printf(0, "Battery: %u mV (USB-only mode)\r\n", init_batt);
    } else {
        SEGGER_RTT_printf(0, "Battery: %u mV (%u%%)\r\n",
            init_batt, estimate_battery_percent(init_batt));
    }

    // Radio
    SEGGER_RTT_printf(0, "Initializing radio (channel %d)...\r\n", RF_CHANNEL);
    if (!radio_init()) {
        SEGGER_RTT_printf(0, "FATAL: radio init failed\r\n");
        indicate_error_fatal();
    }
    SEGGER_RTT_printf(0, "Radio OK\r\n");

    memset(sensor_history, 0, sizeof(sensor_history));
    memset(&tx_packet, 0, sizeof(tx_packet));

    // Brief LED flash = ready
    NRF_P1->OUTSET = (1 << LED_PIN); nrf_delay_ms(500);
    NRF_P1->OUTCLR = (1 << LED_PIN);

    SEGGER_RTT_printf(0, "Transmitting at 250Hz (4ms). Status every %us.\r\n\r\n",
        STATUS_INTERVAL_SEC);

    next_tx_time = get_rtc_ticks() + TX_INTERVAL_MS;

    while (1)
    {
        current_ticks = get_rtc_ticks();

        // Debug timing: print once per second (250 loops at 250Hz)
        if (loop_count % 250 == 0) {
            int32_t diff = (int32_t)((int16_t)(next_tx_time - current_ticks));
            SEGGER_RTT_printf(0, "DEBUG: loop=%u T=%u next=%u diff=%d\r\n",
                loop_count, current_ticks, next_tx_time, diff);
        }

        // Read -> history -> packet
        sensors_read(&current_sensors);
        update_sensor_history(&current_sensors);
        prepare_packet();

        // Sensor values printed once per second
        if (loop_count % 250 == 0) {
            // Decode H3LIS using extract_h3lis_axis() from packet_spec.h.
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

        // Transmit
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

        // Status packet every STATUS_INTERVAL_SEC seconds.
        // Subtraction masked to 24 bits handles the RTC counter wrap at 0xFFFFFF.
        if (((NRF_RTC1->COUNTER - last_status_rtc_tick) & 0xFFFFFFU)
                >= STATUS_INTERVAL_TICKS) {
            transmit_status_packet();
            last_status_rtc_tick = NRF_RTC1->COUNTER;
        }

        loop_count++;

        // LED heartbeat: 1Hz (250 loops at 250Hz)
        if (loop_count % 250 == 0) {
            if (NRF_P1->OUT & (1 << LED_PIN))
                NRF_P1->OUTCLR = (1 << LED_PIN);
            else
                NRF_P1->OUTSET = (1 << LED_PIN);
        }

        // Precision timing.
        // next_tx_time and current_ticks are both uint16_t so subtraction
        // wraps correctly at the 65535-tick boundary (~66.0s at 993Hz).
        next_tx_time += TX_INTERVAL_MS;
        current_ticks = get_rtc_ticks();
        int32_t time_diff = (int32_t)((int16_t)(next_tx_time - current_ticks));

        if (time_diff > 0 && time_diff < 100) {
            nrf_delay_ms((uint32_t)time_diff);
        } else if (time_diff >= 100 || time_diff < -100) {
            // Resync on large deviation. Large negative: loop stalled (e.g. slow
            // I2C on first BMP read). Large positive: startup artifact or glitch.
            SEGGER_RTT_printf(0, "WARN: timing resync (diff=%d)\r\n", time_diff);
            next_tx_time = current_ticks + TX_INTERVAL_MS;
        }
        // time_diff 0..-100: slightly behind, transmit immediately next iteration
    }
}
