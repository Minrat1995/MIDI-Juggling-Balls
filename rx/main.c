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
 * Data flow:
 *   Radio ISR -> rx_packet buffer -> main loop copy -> USB CDC (89 bytes) -> C++ decoder
 *                                                   -> RTT (decoded, 1Hz)
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
 * @version 1.4
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

// Two-byte sync marker prepended to every USB packet.
// 0xAA 0x55 cannot be mistaken for a valid ball_id (1-8) at byte 0,
// and the two-byte sequence reduces false-positive alignment to 1/65536.
// An XOR checksum byte is appended after the payload for validation.
// Must match SYNC_BYTE_1/2 and checksum logic in SerialReader.cpp.
#define USB_SYNC_BYTE_0     0xAA
#define USB_SYNC_BYTE_1     0x55

int main(void)
{
    uint32_t last_stats_time = 0;
    ret_code_t ret;

    // Initialise the clock driver. This must happen before anything that
    // uses the clock driver abstraction, including the USB stack.
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);

    // Request HFCLK. Required for radio and USB. The clock driver handles
    // the start sequence and tracks the reference count.
    nrf_drv_clock_hfclk_request(NULL);
    while (!nrf_drv_clock_hfclk_is_running()) { /* spin */ }

    timing_init();

    SEGGER_RTT_printf(0, "\r\n=== Juggling Ball RX v1.4 ===\r\n");
    SEGGER_RTT_printf(0, "Packet sizes:\r\n");
    SEGGER_RTT_printf(0, "  radio_packet_t: %u (expect 86)\r\n", sizeof(radio_packet_t));
    SEGGER_RTT_printf(0, "  sensor_data_t:  %u (expect 27)\r\n", sizeof(sensor_data_t));
    SEGGER_RTT_printf(0, "  USB frame:      89 bytes (2 sync + 86 payload + 1 checksum)\r\n\r\n");
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

            // Forward to USB: 0xAA 0x55 + 86 payload bytes + 1 XOR checksum = 89 bytes.
            // Checksum is computed inside usb_serial_send_framed_packet().
            if (usb_serial_ready()) {
                usb_serial_send_framed_packet(&local_packet,
                                             USB_SYNC_BYTE_0,
                                             USB_SYNC_BYTE_1);
            }
        }

        // ---- PRIORITY 2: drive USB event queue ----
        usb_serial_process();

        // ---- PRIORITY 3: print statistics and decoded sensor values (1Hz) ----
        uint32_t now = get_timestamp_ms();
        if (now - last_stats_time >= STATS_INTERVAL_MS) {
            last_stats_time = now;
            packet_processor_print_statistics();
        }

        // Sleep until next interrupt (radio END or RTC tick).
        // Radio at 250Hz guarantees wakeup at least every 4ms.
        __WFE();
    }
}
