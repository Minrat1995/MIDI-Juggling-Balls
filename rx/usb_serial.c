/**
 * USB Serial Module Implementation
 *
 * Streams raw 86-byte radio_packet_t structs over USB CDC virtual serial port.
 *
 * Hot-plug supported via nrf_drv_power_usbevt_enable() — works whether USB
 * is connected before or after power-on, and regardless of the value of
 * APP_USBD_CONFIG_EVENT_QUEUE_ENABLE in sdk_config.h.
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
// USB POWER EVENT HANDLER  (hot-plug support)
// ============================================================================

static void usbd_power_event_handler(nrf_drv_power_usb_evt_t event)
{
    switch (event) {
        case NRF_DRV_POWER_USB_EVT_DETECTED:
            SEGGER_RTT_printf(0, "USB: VBUS detected\r\n");
            if (!nrf_drv_usbd_is_enabled()) {
                app_usbd_enable();
            }
            break;

        case NRF_DRV_POWER_USB_EVT_READY:
            SEGGER_RTT_printf(0, "USB: VBUS ready, starting...\r\n");
            app_usbd_start();
            break;

        case NRF_DRV_POWER_USB_EVT_REMOVED:
            SEGGER_RTT_printf(0, "USB: VBUS removed\r\n");
            app_usbd_stop();
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

    // Register USB power events for hot-plug support.
    // Uses nrf_drv_power_usbevt_enable() which works regardless of
    // APP_USBD_CONFIG_EVENT_QUEUE_ENABLE in sdk_config.h.
    SEGGER_RTT_printf(0, "  USB power events...\r\n");
    static const nrf_drv_power_usbevt_config_t usbevt_config = {
        .handler = usbd_power_event_handler
    };
    ret = nrf_drv_power_usbevt_enable(&usbevt_config);
    if (ret != NRF_SUCCESS) {
        SEGGER_RTT_printf(0, "  FAILED (0x%04X)\r\n", ret);
        return false;
    }

    SEGGER_RTT_printf(0, "USB init OK (waiting for host)\r\n");
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

bool usb_serial_send_packet(const radio_packet_t *packet)
{
    if (!usb_serial_ready()) return false;
    ret_code_t ret = app_usbd_cdc_acm_write(&m_app_cdc_acm,
                                             packet,
                                             sizeof(radio_packet_t));
    return (ret == NRF_SUCCESS);
}

bool usb_serial_send_text(const char *text)
{
    if (!usb_serial_ready()) return false;
    ret_code_t ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, text, strlen(text));
    return (ret == NRF_SUCCESS);
}
