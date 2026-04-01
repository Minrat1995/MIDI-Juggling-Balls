/**
 * Radio Receiver Module Implementation
 */

#include "radio_rx.h"
#include "nrf.h"
#include "nrf_delay.h"
#include <string.h>

// ============================================================================
// INTERNAL STATE
// ============================================================================

// Radio packet buffer (modified only in interrupt context)
static radio_packet_t rx_packet;
static volatile bool packet_received = false;

// Reception statistics
static uint32_t total_packets_received = 0;
static uint32_t crc_errors = 0;

// Debug counters for interrupt monitoring
static volatile uint32_t radio_end_events = 0;
static volatile uint32_t radio_crc_ok = 0;
static volatile uint32_t radio_crc_fail = 0;

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

bool radio_init(void)
{
    // Power cycle radio for clean initialization
    NRF_RADIO->POWER = 0;
    nrf_delay_us(10);
    NRF_RADIO->POWER = 1;
    nrf_delay_us(10);

    // Configure radio mode
    NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos;

    // Set RF channel
    NRF_RADIO->FREQUENCY = RF_CHANNEL;

    // Configure packet format (fixed 98-byte payload)
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) |
                       (0 << RADIO_PCNF0_LFLEN_Pos) |
                       (0 << RADIO_PCNF0_S1LEN_Pos);

    NRF_RADIO->PCNF1 = (PACKET_PAYLOAD_SIZE << RADIO_PCNF1_MAXLEN_Pos) |
                       (PACKET_PAYLOAD_SIZE << RADIO_PCNF1_STATLEN_Pos) |
                       (4 << RADIO_PCNF1_BALEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
                       (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);

    // Set access address (must match transmitter)
    NRF_RADIO->BASE0 = RADIO_BASE_ADDR;
    NRF_RADIO->PREFIX0 = RADIO_PREFIX_ADDR;
    NRF_RADIO->RXADDRESSES = 1;  // Enable logical address 0

    // Configure 24-bit CRC
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
                        (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCPOLY = CRC_POLYNOMIAL;
    NRF_RADIO->CRCINIT = CRC_INIT_VALUE;

    // Set packet buffer pointer
    NRF_RADIO->PACKETPTR = (uint32_t)&rx_packet;

    // Configure hardware shortcuts for continuous reception
    // Chain: END → DISABLE → RXEN → READY → START (automatic loop)
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos) |
                        (RADIO_SHORTS_DISABLED_RXEN_Enabled << RADIO_SHORTS_DISABLED_RXEN_Pos);

    // Enable interrupts for packet reception and CRC errors
    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk | RADIO_INTENSET_CRCERROR_Msk;
    NVIC_EnableIRQ(RADIO_IRQn);

    // Verify radio is responding
    return (NRF_RADIO->FREQUENCY == RF_CHANNEL);
}

void radio_start_rx(void)
{
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->EVENTS_CRCERROR = 0;
    NRF_RADIO->TASKS_RXEN = 1;
}

const radio_packet_t* radio_get_packet_buffer(void)
{
    return &rx_packet;
}

bool radio_packet_available(void)
{
    return packet_received;
}

void radio_clear_packet_flag(void)
{
    __disable_irq();
    packet_received = false;
    __enable_irq();
}

void radio_get_stats(radio_stats_t *stats)
{
    if (stats) {
        stats->total_packets = total_packets_received;
        stats->crc_errors = crc_errors;
        stats->radio_end_events = radio_end_events;
        stats->radio_crc_ok = radio_crc_ok;
        stats->radio_crc_fail = radio_crc_fail;
    }
}

// ============================================================================
// INTERRUPT HANDLER
// ============================================================================

/**
 * Radio interrupt handler
 * Called when packet received or CRC error occurs
 * Hardware shortcuts automatically restart reception
 */
void RADIO_IRQHandler(void)
{
    // Packet reception complete
    if (NRF_RADIO->EVENTS_END != 0) {
        NRF_RADIO->EVENTS_END = 0;

        // Check CRC status
        if (NRF_RADIO->CRCSTATUS == 1) {
            packet_received = true;
            total_packets_received++;
            radio_crc_ok++;
        }
        
        radio_end_events++;
    }

    // CRC error occurred
    if (NRF_RADIO->EVENTS_CRCERROR != 0) {
        NRF_RADIO->EVENTS_CRCERROR = 0;
        crc_errors++;
        radio_crc_fail++;
    }
    
    // Note: Hardware shortcuts automatically restart RX
}
