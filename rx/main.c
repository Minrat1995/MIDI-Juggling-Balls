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
 *   checks, the previous packet is silently lost with no counter. The main
 *   risk is packet_processor_print_statistics() blocking on a full RTT buffer.
 *   usb_tx_drop_count tracks USB TX failures only — it does NOT catch
 *   ISR overwrites. Phase 3 requires a ring buffer to eliminate this risk.
 *
 * Phase 1 validation checklist (RTT on RX):
 *   1. "Ball 1 first packet" appears when TX is powered on
 *   2. ACCEL shows ~+/-2048 on one axis at rest (1g), others near 0
 *   3. MAG shows non-zero values that shift when ball is rotated
 *   4. BMP shows ~101325 Pa indoors
 *   5. Loss rate <1% benchtop
 *   6. USB: 89-byte framed packets visible in serial monitor or C++ decoder
 *
 * HFCLK note:
 *   We use nrf_drv_clock for HFCLK rather than direct register access.
 *   The USB stack (app_usbd) uses the same clock driver internally.
 *   Mixing direct register writes with the driver abstraction bypasses its
 *   reference counting and can cause indeterminate behaviour. The driver
 *   manages both HFCLK and LFCLK correctly when used exclusively.
 *
 * @version 1.5
 *
 * Changelog from 1.4:
 *   - usb_tx_drop_count added: tracks packets where usb_serial_send_framed_packet()
 *     returns false (USB not ready or TX buffer busy). Previously these were
 *     silently discarded with no visibility. The drop count is printed in the
 *     1Hz stats block alongside RF loss stats, making USB vs RF loss
 *     distinguishable from the decoder side.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "app_error.h"
#include "radio_rx.h"
#include "packet_processor.h"
#include "timing.h"
#include "usb_serial.h"

extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);

#define STATS_INTERVAL_MS   1000    // Print stats and decoded sensors every second

// USB_SYNC_BYTE_0, USB_SYNC_BYTE_1, and USB_FRAME_SIZE are defined in
// packet_spec.h (included via radio_rx.h -> packet_spec.h).

int main(void)
{
    uint32_t last_stats_time = 0;
    ret_code_t ret;

    // Count USB TX drops: incremented when usb_serial_send_framed_packet()
    // returns false (TX buffer busy or port not ready). Printed in the 1Hz
    // stats block so USB drops are distinguishable from RF packet loss.
    // Note: drops before the host opens the port (at startup) are expected
    // and not meaningful. In normal operation with the port open, any
    // rising count indicates a TX buffer busy condition.
    uint32_t usb_tx_drop_count = 0;

    // Initialise the clock driver. This must happen before anything that
    // uses the clock driver abstraction, including the USB stack.
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);

    // Request HFCLK. Required for radio and USB. The clock driver handles
    // the start sequence and tracks the reference count.
    nrf_drv_clock_hfclk_request(NULL);
    while (!nrf_drv_clock_hfclk_is_running()) { /* spin */ }

    timing_init();

    SEGGER_RTT_printf(0, "\r\n=== Juggling Ball RX v1.5 ===\r\n");
    SEGGER_RTT_printf(0, "Packet sizes:\r\n");
    SEGGER_RTT_printf(0, "  radio_packet_t: %u (expect 86)\r\n", sizeof(radio_packet_t));
    SEGGER_RTT_printf(0, "  sensor_data_t:  %u (expect 27)\r\n", sizeof(sensor_data_t));
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
        SEGGER_RTT_printf(0, "FATAL: radio init failed\r\n");
        while (1) nrf_delay_ms(1000);
    }
    radio_start_rx();

    // Brief settle, then confirm radio is in RX state
    nrf_delay_ms(10);
    uint32_t radio_state = NRF_RADIO->STATE;
    SEGGER_RTT_printf(0, "Radio state: 0x%02lX (expect 0x03 = RX)\r\n", radio_state);
    if (radio_state != 3) {
        SEGGER_RTT_printf(0, "WARNING: radio may not be in RX mode\r\n");
    }
    SEGGER_RTT_printf(0, "Waiting for packets...\r\n\r\n");

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

            uint32_t rx_time = get_timestamp_ms();
            packet_processor_process(&local_packet, rx_time);

            // Forward to USB. usb_serial_send_framed_packet() handles all guard
            // conditions internally (port not open, TX buffer busy). Count all
            // false returns; drops before the host opens the port are expected
            // at startup. In normal operation, a rising count means the TX
            // buffer is consistently full — primary cause is
            // APP_USBD_CDC_ACM_DATA_EPIN_BUFF_SIZE too small (must be >=89,
            // set to 256 in sdk_config.h).
            if (!usb_serial_send_framed_packet(&local_packet,
                                               USB_SYNC_BYTE_0,
                                               USB_SYNC_BYTE_1)) {
                usb_tx_drop_count++;
            }
        }

        // ---- PRIORITY 2: drive USB event queue ----
        usb_serial_process();

        // ---- PRIORITY 3: print statistics and decoded sensor values (1Hz) ----
        uint32_t now = get_timestamp_ms();
        if (now - last_stats_time >= STATS_INTERVAL_MS) {
            last_stats_time = now;
            packet_processor_print_statistics();
            SEGGER_RTT_printf(0, "USB TX drops: %lu\r\n", usb_tx_drop_count);
        }

        // Sleep until next interrupt (radio END or RTC tick).
        // Radio at 250Hz guarantees wakeup at least every 4ms.
        __WFE();
    }
}
