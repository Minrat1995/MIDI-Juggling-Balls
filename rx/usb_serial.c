/**
 * USB Serial Module Implementation
 *
 * Streams radio_packet_t structs over USB CDC virtual serial port.
 *
 * USB framing (USB_FRAME_SIZE = 89 bytes per packet):
 *   [0]    sync byte 0 (USB_SYNC_BYTE_0 = 0xAA)
 *   [1]    sync byte 1 (USB_SYNC_BYTE_1 = 0x55)
 *   [2-87] radio_packet_t payload (86 bytes)
 *   [88]   XOR checksum of bytes [2-87]
 *
 * Framing constants (USB_SYNC_BYTE_0/1 and USB_FRAME_SIZE) are defined in
 * packet_spec.h and must not be redefined here.
 *
 * The checksum is XOR of all 86 payload bytes. On the decoder side,
 * after finding the sync header and extracting the payload, the checksum
 * is recomputed and compared. A mismatch means the sync header was a
 * false positive (0xAA 0x55 appearing in payload data), and the decoder
 * discards the frame and resyncs.
 *
 * Phase 1 limitation: USB cable must be connected at boot.
 * Hot-plug (connecting USB after power-on) is not supported in this version.
 * USB init is non-fatal — if it fails, RTT validation still works fully.
 */

#include "usb_serial.h"
#include "app_usbd.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_core.h"
#include "app_usbd_string_desc.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"
#include "nrf_delay.h"
#include <string.h>

extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);

// ============================================================================
// CDC ACM INSTANCE
// ============================================================================

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2
#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                             cdc_acm_user_ev_handler,
                             CDC_ACM_COMM_INTERFACE,
                             CDC_ACM_DATA_INTERFACE,
                             CDC_ACM_COMM_EPIN,
                             CDC_ACM_DATA_EPIN,
                             CDC_ACM_DATA_EPOUT,
                             APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

// ============================================================================
// STATE
// ============================================================================

static volatile bool usb_configured = false;
static volatile bool port_open      = false;

// TX frame buffer for framed packet writes (USB_FRAME_SIZE = 89 bytes).
//
// app_usbd_cdc_acm_write() is non-blocking: it starts a USB DMA transfer and
// returns immediately. The DMA hardware continues reading from the buffer
// pointer asynchronously, potentially across two USB bulk packets (64 bytes,
// then the remainder). If the buffer is a local stack variable, the function
// returns and the stack is reused while the DMA for the second packet is still
// in flight, causing garbage to be transmitted for the last 25 bytes.
//
// Fix: keep the frame buffer as a module-level static so DMA always reads
// live memory regardless of when the second bulk packet fires.
static uint8_t s_tx_frame[USB_FRAME_SIZE];

// TX text buffer for send_text() writes.
//
// Same DMA concern applies: if the caller passes a stack string, and the
// string is longer than one USB full-speed bulk packet (64 bytes), the second
// packet would read freed stack. Fix: copy into this static buffer first.
// Capped at 255 characters; longer strings are truncated silently.
#define TX_TEXT_MAX  256
static char s_tx_text[TX_TEXT_MAX];

// Guards both TX paths. Set before app_usbd_cdc_acm_write(); cleared by
// TX_DONE callback. Both paths share the same USB data endpoint; overlapping
// writes corrupt the ongoing DMA transfer.
static volatile bool s_tx_busy = false;

// Drop counter: incremented whenever send_framed_packet() cannot send.
// Reset on PORT_OPEN so pre-connection startup drops don't pollute stats.
static volatile uint32_t s_tx_drop_count = 0;

// ============================================================================
// CDC ACM EVENT HANDLER
// ============================================================================

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    (void)p_inst;   // parameter mandated by SDK callback signature; not used here
    switch (event) {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
            port_open      = true;
            s_tx_busy      = false;   // clear any stale pending state on reconnect
            s_tx_drop_count = 0;      // reset drop counter — pre-connection drops
                                      // are expected and not meaningful
            SEGGER_RTT_printf(0, "USB: port opened by host\r\n");
            break;
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            port_open = false;
            s_tx_busy = false;
            SEGGER_RTT_printf(0, "USB: port closed\r\n");
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            // All bytes from the last write have been transferred to the USB
            // host. The static frame buffer is now safe to overwrite.
            s_tx_busy = false;
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
            break;
        default:
            break;
    }
}

// ============================================================================
// USB STACK EVENT HANDLER
// ============================================================================

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event) {
        case APP_USBD_EVT_STARTED:
            usb_configured = true;
            SEGGER_RTT_printf(0, "USB: stack started\r\n");
            break;

        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            usb_configured = false;
            port_open      = false;
            SEGGER_RTT_printf(0, "USB: stopped\r\n");
            break;

        case APP_USBD_EVT_POWER_DETECTED:
            if (!nrf_drv_usbd_is_enabled()) {
                app_usbd_enable();
            }
            break;

        case APP_USBD_EVT_POWER_REMOVED:
            app_usbd_stop();
            break;

        case APP_USBD_EVT_POWER_READY:
            app_usbd_start();
            break;

        default:
            break;
    }
}

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

bool usb_serial_init(void)
{
    ret_code_t ret;

    SEGGER_RTT_printf(0, "USB init:\r\n");

    SEGGER_RTT_printf(0, "  Clock driver...\r\n");
    ret = nrf_drv_clock_init();
    if (ret != NRF_SUCCESS && ret != NRF_ERROR_MODULE_ALREADY_INITIALIZED) {
        SEGGER_RTT_printf(0, "  FAILED (0x%04X)\r\n", ret);
        return false;
    }

    SEGGER_RTT_printf(0, "  HFCLK...\r\n");
    nrf_drv_clock_hfclk_request(NULL);
    uint32_t timeout = 10000;
    while (!nrf_drv_clock_hfclk_is_running() && timeout > 0) {
        nrf_delay_us(10);
        timeout--;
    }
    if (timeout == 0) {
        SEGGER_RTT_printf(0, "  FAILED: HFCLK timeout\r\n");
        return false;
    }

    SEGGER_RTT_printf(0, "  Power driver...\r\n");
    static const nrf_drv_power_config_t power_config = {
        .dcdcen   = 0,
        .dcdcenhv = 0
    };
    ret = nrf_drv_power_init(&power_config);
    if (ret != NRF_SUCCESS && ret != NRF_ERROR_MODULE_ALREADY_INITIALIZED) {
        SEGGER_RTT_printf(0, "  FAILED (0x%04X)\r\n", ret);
        return false;
    }

    SEGGER_RTT_printf(0, "  USB stack...\r\n");
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };
    ret = app_usbd_init(&usbd_config);
    if (ret != NRF_SUCCESS) {
        SEGGER_RTT_printf(0, "  FAILED (0x%04X)\r\n", ret);
        return false;
    }

    SEGGER_RTT_printf(0, "  CDC ACM class...\r\n");
    app_usbd_class_inst_t const *cdc = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(cdc);
    if (ret != NRF_SUCCESS) {
        SEGGER_RTT_printf(0, "  FAILED (0x%04X)\r\n", ret);
        return false;
    }

    SEGGER_RTT_printf(0, "  Enabling USB...\r\n");
    app_usbd_enable();

    SEGGER_RTT_printf(0, "  Starting USB...\r\n");
    app_usbd_start();

    // Allow time for USB enumeration
    uint32_t wait_ms = 0;
    while (!usb_serial_ready() && wait_ms < 2000) {
        app_usbd_event_queue_process();
        nrf_delay_ms(10);
        wait_ms += 10;
    }

    if (usb_serial_ready()) {
        SEGGER_RTT_printf(0, "USB: host connected and port open\r\n");
    } else {
        SEGGER_RTT_printf(0, "USB: started (host not yet connected — will connect when port opened)\r\n");
    }

    return true;
}

void usb_serial_process(void)
{
    while (app_usbd_event_queue_process());
}

bool usb_serial_ready(void)
{
    return (usb_configured && port_open);
}

bool usb_serial_send_framed_packet(const radio_packet_t *packet)
{
    if (!usb_serial_ready()) {
        s_tx_drop_count++;
        return false;
    }

    // Guard: drop if the previous DMA transfer is still in flight.
    // s_tx_busy is set here and cleared in TX_DONE. At 250Hz this guard is
    // almost never triggered (DMA completes in ~1ms, period is 4ms), but it
    // prevents buffer corruption in the event of a timing anomaly.
    if (s_tx_busy) {
        s_tx_drop_count++;
        return false;
    }

    // Build the frame into the module-level static buffer.
    //
    // The buffer MUST be static (not a local stack variable). app_usbd_cdc_acm_write
    // is non-blocking: it starts DMA and returns immediately. For frames larger than
    // one USB full-speed bulk packet (64 bytes), the hardware sends the first 64
    // bytes, fires TX_DONE, then sends the remaining 25 bytes. If the buffer is on
    // the stack, the function has already returned and the stack has been reused
    // by the time the second DMA fires, so the last 25 bytes are garbage.
    //
    // Frame layout (USB_FRAME_SIZE = 89 bytes):
    //   [0]    USB_SYNC_BYTE_0 (0xAA)
    //   [1]    USB_SYNC_BYTE_1 (0x55)
    //   [2-87] radio_packet_t payload (86 bytes)
    //   [88]   XOR checksum of bytes [2-87]
    s_tx_frame[0] = USB_SYNC_BYTE_0;
    s_tx_frame[1] = USB_SYNC_BYTE_1;
    memcpy(&s_tx_frame[2], packet, sizeof(radio_packet_t));

    const uint8_t *bytes = (const uint8_t *)packet;
    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(radio_packet_t); i++) {
        checksum ^= bytes[i];
    }
    s_tx_frame[USB_FRAME_SIZE - 1] = checksum;

    s_tx_busy = true;
    ret_code_t ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, s_tx_frame, USB_FRAME_SIZE);
    if (ret != NRF_SUCCESS) {
        s_tx_busy = false;
        s_tx_drop_count++;
        return false;
    }
    return true;
}

bool usb_serial_send_text(const char *text)
{
    if (!usb_serial_ready()) return false;

    // Guard: do not write while a framed-packet DMA is in flight.
    // Both paths share the same USB endpoint; overlapping writes corrupt
    // the ongoing transfer. s_tx_busy is cleared by TX_DONE.
    if (s_tx_busy) return false;

    // Copy into static buffer before writing.
    //
    // app_usbd_cdc_acm_write() is non-blocking and starts DMA. For strings
    // longer than one USB full-speed bulk packet (64 bytes), the second DMA
    // fires after this function returns. If the caller passed a stack pointer,
    // the stack has been reused by then — the last bytes would be garbage.
    // Copying here ensures DMA always reads from live static memory.
    // Strings longer than TX_TEXT_MAX-1 bytes are silently truncated.
    strncpy(s_tx_text, text, TX_TEXT_MAX - 1);
    s_tx_text[TX_TEXT_MAX - 1] = '\0';
    size_t len = strlen(s_tx_text);

    s_tx_busy = true;
    ret_code_t ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, (const uint8_t *)s_tx_text, len);
    if (ret != NRF_SUCCESS) {
        s_tx_busy = false;
        return false;
    }
    return true;
}

uint32_t usb_serial_get_and_reset_tx_drop_count(void)
{
    // Brief IRQ disable for atomic read-and-clear.
    // s_tx_drop_count can be incremented from the main loop context only
    // (both send functions are called from main), so the window is minimal.
    __disable_irq();
    uint32_t n = s_tx_drop_count;
    s_tx_drop_count = 0;
    __enable_irq();
    return n;
}
