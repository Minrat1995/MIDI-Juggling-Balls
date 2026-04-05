/**
 * USB Serial Module Implementation
 *
 * Streams radio_packet_t structs over USB CDC virtual serial port.
 *
 * USB framing (89 bytes per packet):
 *   [0]    sync byte 0 (0xAA)
 *   [1]    sync byte 1 (0x55)
 *   [2-87] radio_packet_t payload (86 bytes)
 *   [88]   XOR checksum of bytes [2-87]
 *
 * The checksum is XOR of all 86 payload bytes. On the decoder side,
 * after finding the sync header and extracting the payload, the checksum
 * is recomputed and compared. A mismatch means the sync header was a
 * false positive (0xAA 0x55 appearing in payload data), and the decoder
 * discards the frame and resyncs. This eliminates the ~20% false-sync
 * loss rate caused by 0xAA 0x55 collisions in sensor data.
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

// ============================================================================
// CDC ACM EVENT HANDLER
// ============================================================================

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    switch (event) {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
            port_open = true;
            SEGGER_RTT_printf(0, "USB: port opened by host\r\n");
            break;
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            port_open = false;
            SEGGER_RTT_printf(0, "USB: port closed\r\n");
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
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

bool usb_serial_send_framed_packet(const radio_packet_t *packet,
                                   uint8_t sync0,
                                   uint8_t sync1)
{
    if (!usb_serial_ready()) return false;

    ret_code_t ret;

    // --- Sync header ---
    uint8_t sync[2] = { sync0, sync1 };
    ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, sync, 2);
    if (ret != NRF_SUCCESS) return false;

    // --- Payload ---
    ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, packet, sizeof(radio_packet_t));
    if (ret != NRF_SUCCESS) return false;

    // --- XOR checksum over all 86 payload bytes ---
    // Recomputed by the decoder after extraction. Any false sync alignment
    // produces garbage payload bytes whose XOR will not match, allowing
    // the decoder to discard the frame and resync without passing bad data
    // to the application.
    const uint8_t *bytes = (const uint8_t *)packet;
    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(radio_packet_t); i++) {
        checksum ^= bytes[i];
    }

    ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, &checksum, 1);
    return (ret == NRF_SUCCESS);
}

bool usb_serial_send_text(const char *text)
{
    if (!usb_serial_ready()) return false;
    ret_code_t ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, text, strlen(text));
    return (ret == NRF_SUCCESS);
}
