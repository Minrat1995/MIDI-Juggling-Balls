/**
 * USB Serial Module
 *
 * Streams radio_packet_t structs over USB CDC virtual serial port.
 *
 * USB framing:
 *   Each packet is sent as 89 bytes:
 *     [0]    sync byte 0 (USB_SYNC_BYTE_0 = 0xAA)
 *     [1]    sync byte 1 (USB_SYNC_BYTE_1 = 0x55)
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
 * Sync bytes and frame size are taken from packet_spec.h constants
 * (USB_SYNC_BYTE_0, USB_SYNC_BYTE_1, USB_FRAME_SIZE). The decoder is
 * hardcoded to 0xAA 0x55 — these are not caller-configurable.
 *
 * Non-blocking: returns false if USB not ready or TX buffer busy. A false
 * return means this packet is silently dropped. The decoder will see it as
 * a sequence gap, same as RF packet loss. The drop is counted internally
 * and retrievable via usb_serial_get_and_reset_tx_drop_count().
 *
 * @param packet  Pointer to packet to send
 * @return true if the write was accepted by the USB stack
 */
bool usb_serial_send_framed_packet(const radio_packet_t *packet);

/**
 * Send a null-terminated ASCII string over USB (for diagnostics).
 *
 * Truncated to 255 characters. Copies into a static buffer before writing
 * so the caller's string does not need to outlive this call.
 *
 * @param text Null-terminated string
 * @return true if write was accepted
 */
bool usb_serial_send_text(const char *text);

/**
 * Return and atomically reset the TX drop counter.
 *
 * Counts all packets where usb_serial_send_framed_packet() returned false.
 * Resets automatically when the host opens the port (PORT_OPEN event) so
 * pre-connection drops do not appear in steady-state statistics.
 *
 * @return Number of dropped packets since last call or last PORT_OPEN.
 */
uint32_t usb_serial_get_and_reset_tx_drop_count(void);

#endif // USB_SERIAL_H
