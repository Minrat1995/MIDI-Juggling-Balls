/**
 * Radio Receiver Module Implementation
 */

#include "radio_rx.h"
#include "nrf.h"
#include "nrf_delay.h"

extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);

// ============================================================================
// INTERNAL STATE
// ============================================================================

// Single receive buffer.
// The ISR writes here; main loop copies it out within the 4ms window (250Hz).
// If the main loop is delayed longer than 4ms the buffer will be overwritten.
// isr_overwrites tracks this — a non-zero count means packets were silently
// lost due to main loop latency, not RF loss. See radio_stats_t.
// Phase 3 multi-ball requires a ring buffer before running 250Hz x 3 balls.
static radio_packet_t rx_packet;
static volatile bool  packet_received = false;

static volatile uint32_t total_packets  = 0;
static volatile uint32_t crc_errors     = 0;
static volatile uint32_t end_events     = 0;
static volatile uint32_t isr_overwrites = 0;

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

bool radio_init(void)
{
    NRF_RADIO->POWER = 0; nrf_delay_us(10);
    NRF_RADIO->POWER = 1; nrf_delay_us(10);

    // After power-on, radio must be DISABLED before configuration.
    // A non-DISABLED state here indicates the peripheral did not reset cleanly.
    if (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) {
        SEGGER_RTT_printf(0, "RADIO: unexpected state 0x%lX after power cycle\r\n",
            NRF_RADIO->STATE);
        return false;
    }

    NRF_RADIO->MODE      = RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos;
    NRF_RADIO->FREQUENCY = RF_CHANNEL;

    // Fixed-length payload, no length/S0/S1 fields
    NRF_RADIO->PCNF0 = 0;
    NRF_RADIO->PCNF1 =
        (PACKET_PAYLOAD_SIZE << RADIO_PCNF1_MAXLEN_Pos)  |
        (PACKET_PAYLOAD_SIZE << RADIO_PCNF1_STATLEN_Pos) |
        (4 << RADIO_PCNF1_BALEN_Pos)                     |
        (RADIO_PCNF1_ENDIAN_Little   << RADIO_PCNF1_ENDIAN_Pos)  |
        (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);

    NRF_RADIO->BASE0       = RADIO_BASE_ADDR;
    NRF_RADIO->PREFIX0     = RADIO_PREFIX_ADDR;
    NRF_RADIO->RXADDRESSES = 1;

    NRF_RADIO->CRCCNF  = (RADIO_CRCCNF_LEN_Three    << RADIO_CRCCNF_LEN_Pos) |
                         (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCPOLY = CRC_POLYNOMIAL;
    NRF_RADIO->CRCINIT = CRC_INIT_VALUE;

    NRF_RADIO->PACKETPTR = (uint32_t)&rx_packet;

    // Hardware shortcut chain: READY->START, END->DISABLE, DISABLED->RXEN
    // This loops the radio automatically after each packet with zero CPU cost.
    NRF_RADIO->SHORTS =
        (RADIO_SHORTS_READY_START_Enabled   << RADIO_SHORTS_READY_START_Pos)  |
        (RADIO_SHORTS_END_DISABLE_Enabled   << RADIO_SHORTS_END_DISABLE_Pos)  |
        (RADIO_SHORTS_DISABLED_RXEN_Enabled << RADIO_SHORTS_DISABLED_RXEN_Pos);

    // Interrupt on END (covers both CRC-OK and CRC-fail via CRCSTATUS check)
    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
    NVIC_SetPriority(RADIO_IRQn, 1);
    NVIC_EnableIRQ(RADIO_IRQn);

    // Verify critical configuration was written correctly.
    // Checks registers with non-trivial values: a bus fault or unclocked
    // peripheral will produce wrong readbacks rather than matching what was
    // written. CRCPOLY and CRCINIT are included because a mismatch causes
    // 100% packet loss that looks identical to RF dead air.
    bool mode_ok    = (NRF_RADIO->MODE ==
                        (RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos));
    bool freq_ok    = (NRF_RADIO->FREQUENCY == RF_CHANNEL);
    bool payload_ok = (((NRF_RADIO->PCNF1 >> RADIO_PCNF1_STATLEN_Pos) & 0xFF)
                        == PACKET_PAYLOAD_SIZE);
    bool addr_ok    = (NRF_RADIO->BASE0 == RADIO_BASE_ADDR);
    bool crcpoly_ok = (NRF_RADIO->CRCPOLY == CRC_POLYNOMIAL);
    bool crcinit_ok = (NRF_RADIO->CRCINIT == CRC_INIT_VALUE);

    if (!mode_ok)    SEGGER_RTT_printf(0, "RADIO: MODE readback mismatch\r\n");
    if (!freq_ok)    SEGGER_RTT_printf(0, "RADIO: FREQUENCY readback mismatch\r\n");
    if (!payload_ok) SEGGER_RTT_printf(0, "RADIO: PCNF1 STATLEN readback mismatch\r\n");
    if (!addr_ok)    SEGGER_RTT_printf(0, "RADIO: BASE0 readback mismatch\r\n");
    if (!crcpoly_ok) SEGGER_RTT_printf(0, "RADIO: CRCPOLY readback mismatch\r\n");
    if (!crcinit_ok) SEGGER_RTT_printf(0, "RADIO: CRCINIT readback mismatch\r\n");

    return (mode_ok && freq_ok && payload_ok && addr_ok && crcpoly_ok && crcinit_ok);
}

void radio_start_rx(void)
{
    NRF_RADIO->EVENTS_READY    = 0;
    NRF_RADIO->EVENTS_END      = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_RXEN = 1;
}

bool radio_packet_available(void)
{
    return packet_received;
}

const radio_packet_t *radio_get_packet_buffer(void)
{
    return &rx_packet;
}

void radio_clear_packet_flag(void)
{
    packet_received = false;
}

void radio_get_stats(radio_stats_t *stats)
{
    if (!stats) return;
    // Snapshot all four counters atomically to prevent observing a partially
    // updated state. IRQs disabled for ~8 instructions — negligible impact.
    __disable_irq();
    stats->total_packets  = total_packets;
    stats->crc_errors     = crc_errors;
    stats->end_events     = end_events;
    stats->isr_overwrites = isr_overwrites;
    __enable_irq();
}

// ============================================================================
// INTERRUPT HANDLER
// ============================================================================

void RADIO_IRQHandler(void)
{
    if (NRF_RADIO->EVENTS_END) {
        NRF_RADIO->EVENTS_END = 0;
        end_events++;

        if (NRF_RADIO->CRCSTATUS == 1) {
            // CRC OK — new valid packet in rx_packet buffer.
            // If the main loop has not yet consumed the previous packet,
            // it will be silently overwritten. Count this so it is visible
            // in the stats block. A non-zero count means the main loop is
            // blocking for > 4ms — most likely cause is RTT output stalling
            // on a full RTT up-buffer (check SEGGER_RTT_CONFIG_DEFAULT_MODE).
            if (packet_received) {
                isr_overwrites++;
            }
            packet_received = true;
            total_packets++;
        } else {
            // CRC fail — buffer contents are corrupt, discard silently
            crc_errors++;
        }
        // Hardware shortcut has already triggered DISABLE -> RXEN
    }
}
