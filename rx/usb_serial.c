/**
 * USB Serial Module Implementation
 * 
 * Fixed to match Nordic's working USB CDC ACM example
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

// CDC ACM interface definitions (matching Nordic's example)
#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2
#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

// USB CDC instance
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

// FIXED: Use correct endpoint numbers and AT protocol (like Nordic example)
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                             cdc_acm_user_ev_handler,
                             CDC_ACM_COMM_INTERFACE,
                             CDC_ACM_DATA_INTERFACE,
                             CDC_ACM_COMM_EPIN,
                             CDC_ACM_DATA_EPIN,
                             CDC_ACM_DATA_EPOUT,
                             APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

static volatile bool usb_configured = false;
static volatile bool port_open = false;

// Event handler for USB CDC
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    switch (event) {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
            port_open = true;
            break;
            
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            port_open = false;
            break;
            
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            // Transmission complete
            break;
            
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
            // Data received (we don't need this for output-only)
            break;
            
        default:
            break;
    }
}

// USB event handler
static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event) {
        case APP_USBD_EVT_DRV_SUSPEND:
            break;
            
        case APP_USBD_EVT_DRV_RESUME:
            break;
            
        case APP_USBD_EVT_STARTED:
            usb_configured = true;
            break;
            
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            usb_configured = false;
            port_open = false;
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

bool usb_serial_init(void)
{
    ret_code_t ret;
    
    extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);
    
    SEGGER_RTT_printf(0, "  Step 1: Init clock driver...\r\n");
    ret = nrf_drv_clock_init();
    if (ret != NRF_SUCCESS && ret != NRF_ERROR_MODULE_ALREADY_INITIALIZED) {
        SEGGER_RTT_printf(0, "  FAILED: Clock init error 0x%04X\r\n", ret);
        return false;
    }
    SEGGER_RTT_printf(0, "  Step 1: OK (ret=0x%04X)\r\n", ret);
    
    SEGGER_RTT_printf(0, "  Step 2: Request HFCLK...\r\n");
    nrf_drv_clock_hfclk_request(NULL);
    
    // Wait for HFCLK with timeout
    uint32_t timeout = 1000;
    while (!nrf_drv_clock_hfclk_is_running() && timeout > 0) {
        nrf_delay_us(100);
        timeout--;
    }
    
    if (timeout == 0) {
        SEGGER_RTT_printf(0, "  FAILED: HFCLK timeout\r\n");
        return false;
    }
    SEGGER_RTT_printf(0, "  Step 2: OK\r\n");
    
    SEGGER_RTT_printf(0, "  Step 3: Init power driver...\r\n");
    // FIXED: Use default config structure instead of NULL (like Nordic example)
    static const nrf_drv_power_config_t power_config = {
        .dcdcen = 0,
        .dcdcenhv = 0
    };
    ret = nrf_drv_power_init(&power_config);
    if (ret != NRF_SUCCESS && ret != NRF_ERROR_MODULE_ALREADY_INITIALIZED) {
        SEGGER_RTT_printf(0, "  FAILED: Power init error 0x%04X\r\n", ret);
        return false;
    }
    SEGGER_RTT_printf(0, "  Step 3: OK\r\n");
    
    // NOTE: Still skipping USB power events as they cause issues in our configuration
    SEGGER_RTT_printf(0, "  Step 4: SKIPPED (will poll USB state)\r\n");
    
    SEGGER_RTT_printf(0, "  Step 5: Init USB stack...\r\n");
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };
    
    ret = app_usbd_init(&usbd_config);
    if (ret != NRF_SUCCESS) {
        SEGGER_RTT_printf(0, "  FAILED: USB stack init error 0x%04X\r\n", ret);
        return false;
    }
    SEGGER_RTT_printf(0, "  Step 5: OK\r\n");
    
    SEGGER_RTT_printf(0, "  Step 6: Register CDC ACM class...\r\n");
    app_usbd_class_inst_t const * class_cdc_acm = 
        app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    if (ret != NRF_SUCCESS) {
        SEGGER_RTT_printf(0, "  FAILED: CDC class append error 0x%04X\r\n", ret);
        return false;
    }
    SEGGER_RTT_printf(0, "  Step 6: OK\r\n");
    
    SEGGER_RTT_printf(0, "  Step 7: Enable USB...\r\n");
    app_usbd_enable();
    SEGGER_RTT_printf(0, "  Step 7: OK\r\n");
    
    SEGGER_RTT_printf(0, "  Step 8: Start USB...\r\n");
    app_usbd_start();
    SEGGER_RTT_printf(0, "  Step 8: OK\r\n");
    
    return true;
}

bool usb_serial_ready(void)
{
    return (usb_configured && port_open);
}

bool usb_serial_send_packet(const radio_packet_t *packet)
{
    if (!usb_serial_ready()) {
        return false;
    }
    
    ret_code_t ret = app_usbd_cdc_acm_write(&m_app_cdc_acm,
                                             packet,
                                             sizeof(radio_packet_t));
    
    return (ret == NRF_SUCCESS);
}

bool usb_serial_send_text(const char *text)
{
    if (!usb_serial_ready()) {
        return false;
    }
    
    size_t len = strlen(text);
    ret_code_t ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, text, len);
    
    return (ret == NRF_SUCCESS);
}
