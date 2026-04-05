/**
 * USB Serial Module
 *
 * Streams radio_packet_t structs over USB CDC virtual serial port.
 *
 * USB framing:
 *   Each packet is sent as 89 bytes:
 *     [0]    sync byte 0 (0xAA)
 *     [1]    sync byte 1 (0x55)
 *     [2-87] radio_packet_t payload (86 bytes)
 *     [88]   XOR checksum of bytes [2-87]
 *
 *   The sync header locates packet boundaries. The checksum validates
 *   the extraction — false sync alignments produce garbage payload data
 *   which fails the checksum, allowing clean recovery without packet loss.
 *
 * Phase 1 limitation: USB cable must be connected at boot.
 * Hot-plug (connecting USB after power-on) is not supported in this version.
 * USB init is non-fatal: RTT validation works fully without USB.
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
 * Send one packet over USB with sync header and XOR checksum.
 *
 * Frame layout (89 bytes):
 *   sync0 (1) + sync1 (1) + payload (86) + checksum (1)
 *
 * Checksum is XOR of all 86 payload bytes. The decoder verifies this
 * after extracting the payload — checksum failure indicates false sync
 * alignment and triggers resync without passing garbage to the application.
 *
 * Non-blocking: returns false if USB not ready or TX buffer full.
 * A false return means this packet is silently dropped on the embedded
 * side. The decoder will see it as a sequence gap, same as RF packet loss.
 *
 * @param packet  Pointer to packet to send
 * @param sync0   First sync byte (0xAA)
 * @param sync1   Second sync byte (0x55)
 * @return true if all three writes were accepted
 */
bool usb_serial_send_framed_packet(const radio_packet_t *packet,
                                   uint8_t sync0,
                                   uint8_t sync1);

/**
 * Send a null-terminated ASCII string over USB (for diagnostics).
 *
 * @param text Null-terminated string
 * @return true if write was accepted
 */
bool usb_serial_send_text(const char *text);

#endif // USB_SERIAL_H
