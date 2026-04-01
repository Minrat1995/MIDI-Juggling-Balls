/**
 * Juggling Ball Wireless Receiver - Main Program
 * 
 * Receives 9-axis IMU data from juggling balls at 125Hz via custom 2.4GHz protocol
 * 
 * This is the main orchestration file - hardware modules are in separate files:
 * - radio_rx.c: Radio hardware and interrupt handling
 * - packet_processor.c: Packet parsing and statistics
 * - timing.c: RTC-based millisecond counter
 * 
 * Hardware: Adafruit Feather nRF52840
 * Output: SEGGER RTT Debug Terminal (Debug → Go → View → Terminal)
 *         USB serial output will be added here in future
 * 
 * @author Your Name
 * @date October 2025
 * @version 2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "radio_rx.h"
#include "packet_processor.h"
#include "timing.h"
#include "usb_serial.h"
#include "app_usbd.h"

extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);

// Statistics reporting interval
#define STATS_INTERVAL_MS       1000        // Print stats every second

// NOTE: Interrupt handlers (RADIO_IRQHandler, RTC1_IRQHandler) are now in
// radio_rx.c and timing.c respectively. Do not define them here.

// ============================================================================
// MAIN PROGRAM
// ============================================================================

int main(void)
{
    uint32_t last_stats_time = 0;

    // ===== Initialize High Frequency Crystal =====
    // CRITICAL: Required for radio operation
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    // ===== Initialize Timing System =====
    timing_init();

    // ===== Display Startup Banner =====
    SEGGER_RTT_printf(0, "\r\n====================================\r\n");
    SEGGER_RTT_printf(0, "  Juggling Ball Receiver (Feather)\r\n");
    SEGGER_RTT_printf(0, "====================================\r\n");
    SEGGER_RTT_printf(0, "Channel: 2440 MHz (2400 + %d)\r\n", RF_CHANNEL);
    SEGGER_RTT_printf(0, "Listening for up to %d balls...\r\n\r\n", MAX_BALLS);

    // Verify structure sizes (compile-time check passed, but double-check)
    SEGGER_RTT_printf(0, "Structure sizes:\r\n");
    SEGGER_RTT_printf(0, "  radio_packet_t: %u (expect 98)\r\n", sizeof(radio_packet_t));
    SEGGER_RTT_printf(0, "  sensor_data_t: %u (expect 31)\r\n\r\n", sizeof(sensor_data_t));

    // ===== Initialize Packet Processor =====
    packet_processor_init();

    // ===== Initialize USB Serial =====
    SEGGER_RTT_printf(0, "Initializing USB serial...\r\n");
    
    // Small delay to let hardware settle
    nrf_delay_ms(100);
    
    if (!usb_serial_init()) {
        SEGGER_RTT_printf(0, "WARNING: USB initialization failed (continuing with RTT only)\r\n");
    } else {
        SEGGER_RTT_printf(0, "USB initialized successfully\r\n");
        
        // Give USB MORE time to enumerate and process initial events
        SEGGER_RTT_printf(0, "Waiting for USB enumeration...\r\n");
        for (int i = 0; i < 200; i++) {  // 2 seconds total
            for (int j = 0; j < 10; j++) {
                if (!app_usbd_event_queue_process()) {
                    break;
                }
            }
            nrf_delay_ms(10);
        }
        
        if (usb_serial_ready()) {
            SEGGER_RTT_printf(0, "USB port opened by host\r\n");
        } else {
            SEGGER_RTT_printf(0, "USB ready (waiting for host to open port...)\r\n");
        }
    }

    // ===== Initialize Radio =====
    if (!radio_init()) {
        SEGGER_RTT_printf(0, "FATAL: Radio initialization failed\r\n");
        while (1) {
            nrf_delay_ms(1000);
        }
    }
    SEGGER_RTT_printf(0, "Radio initialized successfully\r\n");

    // ===== Start Continuous Reception =====
    radio_start_rx();
    
    // Verify radio entered RX mode
    nrf_delay_ms(10);
    uint32_t radio_state = NRF_RADIO->STATE;
    SEGGER_RTT_printf(0, "Radio state: 0x%02lX (0x03=RX)\r\n\r\n", radio_state);

    if (radio_state != 3) {
        SEGGER_RTT_printf(0, "WARNING: Radio may not be in RX mode\r\n\r\n");
    }

    // ===== Main Loop - Process Packets and Print Statistics =====
    while (1)
    {
        // PRIORITY 1: Check for received packet FIRST
        if (radio_packet_available()) {
            // Get atomic copy of packet
            const radio_packet_t *packet_ptr = radio_get_packet_buffer();
            radio_packet_t local_packet;
            
            __disable_irq();
            memcpy(&local_packet, (void*)packet_ptr, sizeof(radio_packet_t));
            radio_clear_packet_flag();
            __enable_irq();
            
            // Process packet
            uint32_t rx_time = get_timestamp_ms();
            packet_processor_process(&local_packet, rx_time);
            
            // Send raw packet to USB if connected (non-blocking)
            if (usb_serial_ready()) {
                usb_serial_send_packet(&local_packet);
            }
        }
        
        // PRIORITY 2: Process USB events (but limit iterations)
        // Only process a few events per loop to avoid blocking radio
        for (int i = 0; i < 5; i++) {
            if (!app_usbd_event_queue_process()) {
                break; // No more events
            }
        }

        // PRIORITY 3: Print statistics every second
        uint32_t current_time = get_timestamp_ms();
        if (current_time - last_stats_time >= STATS_INTERVAL_MS) {
            last_stats_time = current_time;
            packet_processor_print_statistics();
        }

        // Sleep until next interrupt (power saving)
        __WFE();
    }
}
