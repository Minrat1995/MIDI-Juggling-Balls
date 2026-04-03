/**
 * USB Serial Module
 *
 * Streams raw 86-byte radio_packet_t structs over USB CDC virtual serial port.
 * Binary format — the C++ decoder parses the struct directly.
 *
 * Phase 1 limitation: USB cable must be connected at boot.
 * Hot-plug (connecting USB after power-on) is not supported in this version.
 * USB init is non-fatal: RTT validation works fully without USB connected.
 */

#ifndef USB_SERIAL_H
#define USB_SERIAL_H

#include <stdint.h>
#include <stdbool.h>
#include "radio_rx.h"

/**
 * Initialize USB CDC.
 *
 * Cable must be connected at boot — hot-plug is not supported in Phase 1.
 * USB init is non-fatal: RTT validation works fully without USB.
 *
 * @return true on success (false does not prevent RTT operation)
 */
bool usb_serial_init(void);

/**
 * Drive the USB event queue. Call from the main loop, every iteration.
 * Non-blocking; processes pending USB stack events.
 */
void usb_serial_process(void);

/**
 * @return true if host has opened the serial port
 */
bool usb_serial_ready(void);

/**
 * Send one raw packet (86 bytes) over USB.
 * Non-blocking: returns false if USB not ready or TX buffer full.
 *
 * @param packet Pointer to packet to send
 * @return true if write was accepted
 */
bool usb_serial_send_packet(const radio_packet_t *packet);

/**
 * Send a null-terminated ASCII string over USB (for diagnostics).
 *
 * @param text Null-terminated string
 * @return true if write was accepted
 */
bool usb_serial_send_text(const char *text);

#endif // USB_SERIAL_H
