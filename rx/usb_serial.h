/**
 * USB Serial Module
 * 
 * Provides USB CDC virtual serial port for streaming sensor data
 */

#ifndef USB_SERIAL_H
#define USB_SERIAL_H

#include <stdint.h>
#include <stdbool.h>
#include "radio_rx.h"

/**
 * Initialize USB CDC device
 * 
 * Configures USB as virtual serial port.
 * Will appear as /dev/ttyACM0 (Linux) or COMx (Windows).
 * 
 * @return true on success, false on error
 */
bool usb_serial_init(void);

/**
 * Check if USB is connected and ready
 * 
 * @return true if host connected and port open
 */
bool usb_serial_ready(void);

/**
 * Send raw packet data over USB
 * 
 * Format: Binary packet structure (98 bytes)
 * Non-blocking: returns immediately if buffer full
 * 
 * @param packet Pointer to packet to send
 * @return true if sent, false if buffer full
 */
bool usb_serial_send_packet(const radio_packet_t *packet);

/**
 * Send statistics text over USB
 * 
 * Format: ASCII text (same as RTT output)
 * 
 * @param text Null-terminated string
 * @return true if sent, false if buffer full
 */
bool usb_serial_send_text(const char *text);

#endif // USB_SERIAL_H