/**
 * Juggling Ball Wireless Receiver
 *
 * Receives 86-byte packets at 250Hz from up to 8 balls via 2.4GHz.
 * Forwards packets over USB CDC for the C++ decoder.
 * Decodes and prints sensor values to RTT for Phase 1 validation.
 *
 * Hardware: Adafruit Feather nRF52840 (RX station, USB to PC)
 *
 * USB framing:
 *   Each packet is sent as 89 bytes: 0xAA 0x55 + 86 payload bytes + 1 checksum.
 *   The two-byte sync marker locates packet boundaries. The trailing XOR
 *   checksum (XOR of all 86 payload bytes) validates extraction — any false
 *   sync alignment produces garbage payload data which fails the checksum,
 *   allowing the decoder to discard it and resync cleanly.
 *
 *   Sync bytes and frame size are defined in packet_spec.h (USB_SYNC_BYTE_0,
 *   USB_SYNC_BYTE_1, USB_FRAME_SIZE) and must not be redefined here or in
 *   usb_serial.c.
 *
 * Data flow:
 *   Radio ISR -> rx_packet buffer -> main loop copy -> USB CDC (89 bytes) -> C++ decoder
 *                                                   -> RTT (decoded, 1Hz)
 *
 * Single-buffer overwrite risk:
 *   The radio ISR fires every 4ms and overwrites rx_packet unconditionally.
 *   If the main loop takes longer than 4ms between radio_packet_available()
 *   checks, the previous packet is silently lost. The primary risk is
 *   packet_processor_print_statistics() blocking on a full RTT buffer.
 *   isr_overwrites in radio_stats_t tracks this — a non-zero count means
 *   the main loop is taking > 4ms. Mitigation: set RTT channel to
 *   SEGGER_RTT_MODE_NO_BLOCK_SKIP in SEGGER_RTT_Conf.h.
 *   Phase 3 requires a ring buffer to eliminate this risk entirely.
 *
 * Phase 1 validation checklist (RTT on RX):
 *   1. "Ball 1 first packet" appears when TX is powered on
 *   2. ACCEL shows ~+/-2048 on one axis at rest (1g), others near 0
 *   3. MAG shows non-zero values that shift when ball is rotated
 *   4. BMP shows ~101325 Pa indoors
 *   5. Loss rate <1% benchtop
 *   6. USB: 89-byte framed packets visible in serial monitor or C++ decoder
 *   7. isr_overwrites = 0 in steady state
 *
 * HFCLK note:
 *   We use nrf_drv_clock for HFCLK rather than direct register access.
 *   The USB stack (app_usbd) uses the same clock driver internally.
 *   Mixing direct register writes with the driver abstraction bypasses its
 *   reference counting and can cause indeterminate behaviour. The driver
 *   manages both HFCLK and LFCLK correctly when used exclusively.
 *
 * @version 1.7
 *
 * Changelog from 1.6:
 *   - STATS_INTERVAL_MS renamed STATS_INTERVAL_TICKS, value changed from 1000
 *     to RTC1_TICKS_PER_SEC (993). The _MS name was inconsistent with the
 *     get_rtc_ticks() rename and silently produced a 1.007s interval instead
 *     of 1.000s. Now self-documenting and exact.
 *   - last_stats_time moved from declaration (= 0) to after the init sequence.
 *     usb_serial_init() contains a 2000ms enumeration wait; with last_stats_time = 0
 *     the first stats print fired immediately at loop entry before any packets
 *     could have been received. Now initialised to get_rtc_ticks() after init.
 *   - Redundant #include "radio_rx.h" removed from packet_processor.c (already
 *     included transitively via packet_processor.h).
 *   - APP_ERROR_CHECK on nrf_drv_clock_init() replaced with explicit check.
 *     NRF_ERROR_MODULE_ALREADY_INITIALIZED treated as success (reset-without-
 *     power-cycle scenario). Other failures call indicate_error_fatal() for
 *     consistent RTT output, instead of the SDK fault handler which may be
 *     silent in release builds.
 *   - app_error.h include removed (no longer needed after above change).
 *   - sizeof(radio_packet_t) and sizeof(sensor_data_t) cast to (unsigned) in
 *     RTT printf calls. size_t == unsigned int on ARM but cast is required for
 *     strict correctness with %u.
 *   - Magic number 3 in radio state check replaced with RADIO_STATE_STATE_Rx.
 *   - Unused #include <string.h> removed from radio_rx.c.
 *
 * Changelog from 1.5:
 *   - indicate_error_fatal() added: named fatal handler used for HFCLK and
 *     LFCLK startup failures, consistent with TX pattern.
 *   - HFCLK startup timeout added (HFCLK_STARTUP_TIMEOUT_MS = 10ms). Previously
 *     spun forever if crystal did not start.
 *   - timing_init() now returns bool. Main checks and calls indicate_error_fatal()
 *     if LFCLK does not start within 1000ms.
 *   - get_timestamp_ms() renamed get_rtc_ticks() throughout. RTC1 runs at 993Hz;
 *     each tick is ~1.007ms. The old name caused the same confusion on TX that
 *     prompted the rename there.
 *   - usb_serial_send_framed_packet() sync byte parameters removed. The sync
 *     bytes are protocol constants (0xAA, 0x55) not caller-configurable values.
 *   - USB TX drop counter moved into usb_serial module. Resets on PORT_OPEN so
 *     pre-connection startup drops do not appear in steady-state stats.
 *     Retrieved via usb_serial_get_and_reset_tx_drop_count().
 *   - radio_stats_t gains isr_overwrites field. Printed in 1Hz stats block.
 *     Non-zero means main loop is taking > 4ms — investigate RTT blocking.
 *   - CRCPOLY and CRCINIT readback added to radio_init() (radio_rx.c).
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "radio_rx.h"
#include "packet_processor.h"
#include "timing.h"
#include "usb_serial.h"

extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);

// Stats interval in RTC ticks. RTC1_TICKS_PER_SEC = 993 (see timing.h).
// Using ticks directly avoids any _MS / _ticks unit mismatch at the call site.
#define STATS_INTERVAL_TICKS      RTC1_TICKS_PER_SEC
#define HFCLK_STARTUP_TIMEOUT_MS  10   // HFXO starts in <1ms; 10ms is a generous margin

// USB_SYNC_BYTE_0, USB_SYNC_BYTE_1, and USB_FRAME_SIZE are defined in
// packet_spec.h (included via radio_rx.h -> packet_spec.h).

// ============================================================================
// FATAL ERROR HANDLER
// ============================================================================

// Forward declaration — indicate_error_fatal() is defined below main() but
// called from the init sequence which appears first. Required to satisfy
// C99/C11 constraint that functions are declared before use.
static void indicate_error_fatal(const char *msg);

// ============================================================================
// MAIN
// ============================================================================

int main(void)
{
    ret_code_t ret;

    // Initialise the clock driver. This must happen before anything that
    // uses the clock driver abstraction, including timing_init() and the
    // USB stack.
    //
    // NRF_ERROR_MODULE_ALREADY_INITIALIZED is treated as success: on a
    // reset-without-power-cycle the driver may already be running. Any other
    // non-success code is a genuine fault — use indicate_error_fatal() for
    // consistency with all other fatal init failures (rather than APP_ERROR_CHECK,
    // which calls the SDK fault handler and may not produce RTT output in a
    // release build).
    ret = nrf_drv_clock_init();
    if (ret != NRF_SUCCESS && ret != NRF_ERROR_MODULE_ALREADY_INITIALIZED) {
        indicate_error_fatal("nrf_drv_clock_init failed");
    }

    // Request HFCLK. Required for radio and USB. The clock driver handles
    // the start sequence and tracks the reference count.
    nrf_drv_clock_hfclk_request(NULL);
    {
        uint32_t timeout_ms = 0;
        while (!nrf_drv_clock_hfclk_is_running()) {
            nrf_delay_ms(1);
            if (++timeout_ms > HFCLK_STARTUP_TIMEOUT_MS) {
                indicate_error_fatal("HFCLK did not start");
            }
        }
    }

    // timing_init() requests LFCLK and starts RTC1. Returns false if the
    // LFXO does not start within 1000ms — treat as fatal.
    if (!timing_init()) {
        indicate_error_fatal("LFCLK did not start");
    }

    SEGGER_RTT_printf(0, "\r\n=== Juggling Ball RX v1.7 ===\r\n");
    SEGGER_RTT_printf(0, "Packet sizes:\r\n");
    SEGGER_RTT_printf(0, "  radio_packet_t: %u (expect 86)\r\n", (unsigned)sizeof(radio_packet_t));
    SEGGER_RTT_printf(0, "  sensor_data_t:  %u (expect 27)\r\n", (unsigned)sizeof(sensor_data_t));
    SEGGER_RTT_printf(0, "  USB frame:      %u bytes (2 sync + 86 payload + 1 checksum)\r\n\r\n",
                      (unsigned)USB_FRAME_SIZE);
    SEGGER_RTT_printf(0, "Channel: %d (%d MHz)\r\n", RF_CHANNEL, 2400 + RF_CHANNEL);
    SEGGER_RTT_printf(0, "Listening for up to %d balls\r\n\r\n", MAX_BALLS);

    packet_processor_init();

    // USB init — non-fatal. RTT validation works without USB.
    // Cable must be connected at boot (hot-plug not supported in Phase 1).
    if (!usb_serial_init()) {
        SEGGER_RTT_printf(0, "WARNING: USB init failed — RTT only\r\n\r\n");
    }

    // Radio — fatal if it fails
    if (!radio_init()) {
        indicate_error_fatal("radio_init failed");
    }
    radio_start_rx();

    // Brief settle, then confirm radio is in RX state.
    // RADIO_STATE_STATE_Rx = 3 per nRF52840 PS.
    nrf_delay_ms(10);
    uint32_t radio_state = NRF_RADIO->STATE;
    SEGGER_RTT_printf(0, "Radio state: 0x%02lX (expect 0x03 = RX)\r\n", radio_state);
    if (radio_state != RADIO_STATE_STATE_Rx) {
        SEGGER_RTT_printf(0, "WARNING: radio may not be in RX mode\r\n");
    }
    SEGGER_RTT_printf(0, "Waiting for packets...\r\n\r\n");

    // Initialise stats timer here, after all blocking init has completed.
    // usb_serial_init() contains a 2000ms enumeration wait — if last_stats_time
    // were set at declaration (= 0), the first stats print would fire immediately
    // at loop entry rather than one second into steady-state operation.
    uint32_t last_stats_time = get_rtc_ticks();

    while (1)
    {
        // ---- PRIORITY 1: copy and process any received packet ----
        if (radio_packet_available()) {
            radio_packet_t local_packet;

            // Disable IRQs only for the duration of the copy.
            // The radio hardware is already restarting its next RX cycle.
            // At 250Hz we have 4ms to copy — a memcpy of 86 bytes takes ~1us.
            __disable_irq();
            memcpy(&local_packet, radio_get_packet_buffer(), sizeof(radio_packet_t));
            radio_clear_packet_flag();
            __enable_irq();

            uint32_t rx_ticks = get_rtc_ticks();
            packet_processor_process(&local_packet, rx_ticks);

            // Forward to USB. usb_serial_send_framed_packet() handles all guard
            // conditions internally (port not open, TX buffer busy). Drop count
            // is tracked inside the module and retrieved below during stats print.
            // In normal operation a rising drop count means the TX buffer is
            // consistently full — primary cause is APP_USBD_CDC_ACM_DATA_EPIN_BUFF_SIZE
            // too small (must be >= 89; set to 256 in sdk_config.h).
            usb_serial_send_framed_packet(&local_packet);
        }

        // ---- PRIORITY 2: drive USB event queue ----
        usb_serial_process();

        // ---- PRIORITY 3: print statistics and decoded sensor values (1Hz) ----
        uint32_t now = get_rtc_ticks();
        if (now - last_stats_time >= STATS_INTERVAL_TICKS) {
            last_stats_time = now;
            packet_processor_print_statistics();

            // Retrieve and reset the drop counter. Pre-connection drops (before
            // the host opens the port) are cleared automatically on PORT_OPEN,
            // so this count only reflects post-connection drops.
            uint32_t drops = usb_serial_get_and_reset_tx_drop_count();
            SEGGER_RTT_printf(0, "USB TX drops: %lu\r\n", drops);
        }

        // Sleep until next interrupt (radio END or RTC tick).
        // Radio at 250Hz guarantees wakeup at least every 4ms.
        __WFE();
    }
}

// ============================================================================
// FATAL ERROR HANDLER
// ============================================================================

/**
 * Log a fatal error message to RTT and halt.
 *
 * Used for hardware init failures where continued operation is meaningless.
 * RTT output is best-effort — if RTT is not connected, the message is lost
 * but the halt still occurs. The 1-second delay in the loop keeps power
 * consumption low and makes the stall visible if a debugger is attached later.
 */
static void indicate_error_fatal(const char *msg)
{
    SEGGER_RTT_printf(0, "\r\nFATAL: %s — halted\r\n", msg);
    while (1) {
        nrf_delay_ms(1000);
    }
}
