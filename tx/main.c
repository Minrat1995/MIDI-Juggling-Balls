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
 * @version 3.4
 *
 * Changelog from 3.3:
 *   - radio_init(): power cycle delay increased 10us -> 1ms. 10us was empirically
 *     too short for the radio peripheral power domain to stabilise after POWER=0/1.
 *     If the peripheral is not fully powered when radio_init() reads STATE, the
 *     readback is wrong and init returns false unconditionally.
 *   - transmit_packet(): __DMB() added before TASKS_TXEN. The Cortex-M4 write
 *     buffer can hold pending stores to tx_packet in SRAM. Without a barrier the
 *     DMA transfer can begin before all stores are visible to the bus fabric,
 *     resulting in stale or partially-updated packet data being transmitted.
 *   - recover_radio(): now polls EVENTS_DISABLED rather than sleeping a fixed 1ms.
 *     On a timeout or mid-TX failure the DISABLED event may arrive up to several
 *     hundred us after TASKS_DISABLE; sleeping 1ms was a guess and not reliable.
 *   - recover_radio(): removed indicate_error_radio() call. That function blocked
 *     for 3 seconds per call and returned to the main loop. A failed recover_radio()
 *     caused the loop to call transmit_packet() immediately, fail again, call
 *     recover_radio() again, block 3 more seconds, and so on. The main loop's
 *     1Hz LED heartbeat (toggled every 250 packets) provides sufficient liveness
 *     indication; a frozen heartbeat means the radio is stuck.
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

// ============================================================================
// CONFIGURATION
// ============================================================================

#define LED_PIN                 15          // P1.15
#define VBAT_PIN                NRF_SAADC_INPUT_AIN7  // P0.31

#define BALL_ID                 1           // Override via BLE provisioning (Phase 3)

#define TX_INTERVAL_MS          4           // 250Hz
#define STATUS_INTERVAL_SEC     120         // Status packet every 2 minutes

#define BATTERY_SHUTDOWN_MV     3300
#define TEMP_EMERGENCY_SHUTDOWN 6000        // 60.00 C in 0.01 C units
#define BATTERY_USB_THRESHOLD   1000        // Below 1V = USB-only, no battery
#define BATTERY_FULL_MV         4200
#define BATTERY_EMPTY_MV        3300

// RF constants and packet structures are in packet_spec.h (via sensors.h)

#define RADIO_TIMEOUT_US        5000        // 5ms per operation
#define TX_POWER                RADIO_TXPOWER_TXPOWER_Pos8dBm

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

// ============================================================================
// TIMING  (RTC1, ~1ms ticks)
//
// PRESCALER=32: f = 32768/(32+1) = 992.97 Hz, period = 1.0071 ms
// This is a 0.71% systematic undercount — system_time_ms runs slow by ~0.71%.
// Over 30 minutes, the counter reads ~12.8 seconds behind wall time.
// This does not affect radio timing (hardware), packet loss stats,
// or any real-time decisions. It only affects long-duration elapsed time
// readings if those are ever compared against wall time.
// ============================================================================

static void timing_init(void)
{
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
    NRF_RTC1->PRESCALER = 32;   // 992.97 Hz (~1ms), 0.71% slow — see note above
    NRF_RTC1->TASKS_START = 1;
    nrf_delay_ms(10);
}

// Returns uint16_t: wraps at 65535ms (~65.5s). Wrapping is handled
// correctly by using uint16_t arithmetic in the timing loop.
static inline uint16_t get_timestamp_ms(void)
{
    return (uint16_t)(NRF_RTC1->COUNTER & 0xFFFF);
}

// Uptime in seconds. 24-bit counter overflows at ~4.7 hours;
// this will produce one spurious status packet at that point.
static uint32_t get_uptime_seconds(void)
{
    return (uint32_t)(NRF_RTC1->COUNTER / 1000);
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

static uint16_t read_battery_voltage(void)
{
    int16_t adc;
    NRF_SAADC->RESULT.PTR    = (uint32_t)&adc;
    NRF_SAADC->RESULT.MAXCNT = 1;
    NRF_SAADC->TASKS_START = 1;
    while (NRF_SAADC->EVENTS_STARTED == 0);
    NRF_SAADC->EVENTS_STARTED = 0;
    NRF_SAADC->TASKS_SAMPLE = 1;
    while (NRF_SAADC->EVENTS_END == 0);
    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->TASKS_STOP = 1;
    while (NRF_SAADC->EVENTS_STOPPED == 0);
    NRF_SAADC->EVENTS_STOPPED = 0;

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

static uint8_t calculate_checksum(status_packet_t *p)
{
    uint8_t *b = (uint8_t *)p;
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
    s.timestamp   = get_timestamp_ms();
    s.packet_type = PACKET_TYPE_STATUS;

    // Read temperature; check sentinel before storing.
    // SENSORS_TEMP_UNAVAILABLE (-32768) is below any physical temperature
    // and will not trigger emergency shutdown, but we store 0 in the packet
    // to avoid confusing the PC-side display with an impossible value.
    int16_t temp = sensors_read_temperature();
    s.temperature = (temp != SENSORS_TEMP_UNAVAILABLE) ? temp : 0;

    s.battery_voltage    = read_battery_voltage();
    s.battery_percent    = estimate_battery_percent(s.battery_voltage);
    s.sensor_health      = sensors_get_status_bitmask();
    s.total_packets_sent = total_packets_sent;
    s.uptime_seconds     = get_uptime_seconds();
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
    //
    // Delay after POWER=0 and POWER=1 increased from 10us to 1ms.
    // 10us was insufficient for the peripheral's internal power domain to
    // stabilise before firmware accesses its registers. With 10us, the
    // STATE readback below could return a non-DISABLED value not because
    // the radio was active, but because the register bus hadn't settled,
    // causing radio_init() to return false unconditionally and forcing
    // repeated recovery attempts. 1ms matches Nordic SDK reference examples
    // and provides reliable stabilisation across temperature and supply variation.
    NRF_RADIO->POWER = 0; nrf_delay_ms(1);
    NRF_RADIO->POWER = 1; nrf_delay_ms(1);

    // After power-on, radio must be DISABLED before configuration.
    // A non-DISABLED state here indicates the peripheral did not reset cleanly.
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

    // Verify critical configuration was written correctly.
    // Checks registers with non-trivial values: a bus fault or unclocked
    // peripheral will produce wrong readbacks rather than matching what we wrote.
    bool mode_ok    = (NRF_RADIO->MODE ==
                        (RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos));
    bool freq_ok    = (NRF_RADIO->FREQUENCY == RF_CHANNEL);
    bool payload_ok = (((NRF_RADIO->PCNF1 >> RADIO_PCNF1_STATLEN_Pos) & 0xFF)
                        == PACKET_PAYLOAD_SIZE);
    bool addr_ok    = (NRF_RADIO->BASE0 == RADIO_BASE_ADDR);

    if (!mode_ok)    SEGGER_RTT_printf(0, "RADIO: MODE readback mismatch\r\n");
    if (!freq_ok)    SEGGER_RTT_printf(0, "RADIO: FREQUENCY readback mismatch\r\n");
    if (!payload_ok) SEGGER_RTT_printf(0, "RADIO: PCNF1 STATLEN readback mismatch\r\n");
    if (!addr_ok)    SEGGER_RTT_printf(0, "RADIO: BASE0 readback mismatch\r\n");

    return (mode_ok && freq_ok && payload_ok && addr_ok);
}

static bool transmit_packet(void)
{
    uint32_t t;

    NRF_RADIO->EVENTS_READY    = 0;
    NRF_RADIO->EVENTS_END      = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;

    // __DMB() (Data Memory Barrier) flushes the Cortex-M4 write buffer before
    // the DMA transfer begins. Without this, stores to tx_packet in SRAM may
    // still be pending in the write buffer when TASKS_TXEN triggers the radio
    // DMA, causing stale or partially-updated data to be transmitted.
    // The barrier must appear after the last write to tx_packet (in
    // prepare_packet()) and before TASKS_TXEN.
    __DMB();
    NRF_RADIO->TASKS_TXEN = 1;

    t = 1000;
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

// indicate_error_radio() removed.
//
// It was called from recover_radio() when radio_init() failed after a TX
// timeout. The function blocked for 3 seconds and returned, at which point
// the main loop called transmit_packet() immediately, failed again, called
// recover_radio() again, failed again, and blocked for another 3 seconds.
// This produced a cascade of 3-second blockages rather than recovery.
//
// The 1Hz LED heartbeat toggled in the main loop provides sufficient liveness
// indication. A frozen heartbeat means the radio is stuck. Removing
// indicate_error_radio() allows the main loop to continue cycling, which
// gives recover_radio() repeated chances to succeed (e.g. if the failure
// was a transient bus glitch rather than a hard hardware fault).

static void recover_radio(void)
{
    SEGGER_RTT_printf(0, "Radio: attempting recovery...\r\n");

    // Issue TASKS_DISABLE and wait for EVENTS_DISABLED before reinitialising.
    //
    // Previously: TASKS_DISABLE = 1; nrf_delay_ms(1); radio_init();
    // The 1ms sleep was a guess. If the radio was mid-transmission when a
    // timeout fired, the disable sequence could take longer than 1ms to
    // complete. radio_init() then power-cycled the peripheral and checked
    // STATE == DISABLED; if DISABLED hadn't arrived yet, the readback was
    // wrong and radio_init() returned false, making recovery always fail.
    //
    // Polling EVENTS_DISABLED is the correct approach. The event is set by
    // hardware when the radio has fully disabled. Timeout of 10ms is
    // conservative; the disable sequence is typically complete in <1ms.
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
        // Do not call indicate_error_radio() here. See note above.
        // The main loop will attempt transmit_packet() next iteration,
        // which will fail and call recover_radio() again. This is the
        // desired behaviour: repeated recovery attempts rather than
        // cascading 3-second blockages.
    } else {
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
    tx_packet.timestamp = get_timestamp_ms();
    memcpy(&tx_packet.data_t0, &sensor_history[0], sizeof(sensor_data_t));
    memcpy(&tx_packet.data_t1, &sensor_history[1], sizeof(sensor_data_t));
    memcpy(&tx_packet.data_t2, &sensor_history[2], sizeof(sensor_data_t));
    // Note: __DMB() is called in transmit_packet() before TASKS_TXEN,
    // after all writes to tx_packet are complete.
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
    uint32_t current_time_ms;

    // uint16_t so wrapping arithmetic in the timing section is well-defined
    uint16_t next_tx_time;

    // HFCLK required for radio. TX does not use nrf_drv_clock (no USB stack),
    // so direct register access is appropriate here.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    // LED
    NRF_P1->DIRSET = (1 << LED_PIN);
    NRF_P1->OUTCLR = (1 << LED_PIN);

    // Startup banner
    SEGGER_RTT_printf(0, "\r\n=== Juggling Ball TX (Ball %d) v3.4 ===\r\n", BALL_ID);
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

    // Timing
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

    // Timing initialisation
    next_tx_time = get_timestamp_ms() + TX_INTERVAL_MS;
    static uint32_t last_status_uptime_sec = 0;

    while (1)
    {
        current_time_ms = get_timestamp_ms();

        // Debug timing: print once per second (250 loops at 250Hz)
        if (loop_count == 0 || loop_count % 250 == 0) {
            int32_t diff = (int32_t)((int16_t)(next_tx_time - (uint16_t)current_time_ms));
            SEGGER_RTT_printf(0, "DEBUG: loop=%u T=%u next=%u diff=%d\r\n",
                loop_count, (uint16_t)current_time_ms, next_tx_time, diff);
        }

        // Read -> history -> packet
        sensors_read(&current_sensors);
        update_sensor_history(&current_sensors);
        prepare_packet();

        // Sensor values printed once per second
        if (loop_count % 250 == 0) {
            // Decode H3LIS using extract_h3lis_axis() from packet_spec.h.
            // Uses unsigned right shift (well-defined in C) then explicit sign
            // extension from bit 11 — avoids implementation-defined signed shift.
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

        // Status packet every STATUS_INTERVAL_SEC seconds
        uint32_t uptime = get_uptime_seconds();
        if ((uptime - last_status_uptime_sec) >= STATUS_INTERVAL_SEC) {
            transmit_status_packet();
            last_status_uptime_sec = uptime;
        }

        loop_count++;

        // LED heartbeat: 1Hz (250 loops at 250Hz)
        if (loop_count % 250 == 0) {
            if (NRF_P1->OUT & (1 << LED_PIN))
                NRF_P1->OUTCLR = (1 << LED_PIN);
            else
                NRF_P1->OUTSET = (1 << LED_PIN);
        }

        // Precision timing
        // next_tx_time and current_time_ms are both uint16_t so subtraction
        // wraps correctly at the 65535ms boundary.
        next_tx_time += TX_INTERVAL_MS;
        current_time_ms = get_timestamp_ms();
        int32_t time_diff = (int32_t)((int16_t)(next_tx_time - (uint16_t)current_time_ms));

        if (time_diff > 0 && time_diff < 100) {
            nrf_delay_ms((uint32_t)time_diff);
        } else if (time_diff >= 100 || time_diff < -100) {
            // Resync on any large deviation in either direction.
            // Large positive: startup artifact or clock glitch.
            // Large negative: main loop stalled (e.g. slow I2C on first BMP read).
            // Without this catch, a large negative diff causes the loop to run
            // flat-out trying to catch up, temporarily doubling the TX rate.
            SEGGER_RTT_printf(0, "WARN: timing resync (diff=%d)\r\n", time_diff);
            next_tx_time = (uint16_t)current_time_ms + TX_INTERVAL_MS;
        }
        // time_diff 0..-100: slightly behind, transmit immediately next iteration
    }
}
