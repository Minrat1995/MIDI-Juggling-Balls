/**
 * Packet Processor Implementation
 *
 * Phase 1 goal: confirm the full TX->RX pipeline is working before the
 * C++ decoder exists. All sensor fields are decoded and printed to RTT
 * every second so you can validate data integrity by watching RX RTT
 * while moving the ball.
 *
 * Expected values at rest on a bench:
 *   ACCEL: one axis ~+/-2048 (1g depending on orientation), others near 0
 *   GYRO:  all near 0
 *   MAG:   stable non-zero values that shift when ball is rotated
 *   H3LIS: small values at rest, large spikes on sharp impact
 *   BMP:   ~101325 Pa indoors
 *   FSR:   intensity=0, pattern=0x0 (not wired in Phase 1)
 */

#include "packet_processor.h"
#include "radio_rx.h"
#include <string.h>

extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);

// ============================================================================
// INTERNAL STATE
// ============================================================================

static ball_state_t ball_state[MAX_BALLS];

// Last decoded sensor values per ball, updated on every received packet.
// Stored here so print_statistics() can display without re-decoding.
typedef struct {
    int16_t  accel[3];
    int16_t  gyro[3];
    int16_t  mag[3];
    int16_t  h3lis[3];
    uint8_t  fsr_intensity;
    uint8_t  fsr_pattern;
    uint32_t pressure_pa;
} decoded_sensors_t;

static decoded_sensors_t last_decoded[MAX_BALLS];

// ============================================================================
// DECODE HELPERS
// ============================================================================

static void decode_sensor_data(const sensor_data_t *d, decoded_sensors_t *out)
{
    // IMU: direct copy from packet
    out->accel[0] = d->imu.accel[0];
    out->accel[1] = d->imu.accel[1];
    out->accel[2] = d->imu.accel[2];
    out->gyro[0]  = d->imu.gyro[0];
    out->gyro[1]  = d->imu.gyro[1];
    out->gyro[2]  = d->imu.gyro[2];
    out->mag[0]   = d->imu.mag[0];
    out->mag[1]   = d->imu.mag[1];
    out->mag[2]   = d->imu.mag[2];

    // H3LIS331: sensor value is in bits [15:4], FSR data in bits [3:0].
    // extract_h3lis_axis() uses unsigned right shift (well-defined in C) then
    // explicit sign extension from bit 11, avoiding implementation-defined
    // behaviour of signed right shift. Defined in packet_spec.h.
    out->h3lis[0] = extract_h3lis_axis(d->h3lis_x_fsr_level);
    out->h3lis[1] = extract_h3lis_axis(d->h3lis_y_fsr_pattern);
    out->h3lis[2] = extract_h3lis_axis(d->h3lis_z_flags);

    // FSR: 4 LSBs of the packed H3LIS fields
    out->fsr_intensity = (uint8_t)(d->h3lis_x_fsr_level  & 0x0F);
    out->fsr_pattern   = (uint8_t)(d->h3lis_y_fsr_pattern & 0x0F);

    // Pressure: 24-bit little-endian raw value, Pa = raw / 64
    uint32_t raw = (uint32_t)d->pressure[0]         |
                  ((uint32_t)d->pressure[1] << 8)   |
                  ((uint32_t)d->pressure[2] << 16);
    out->pressure_pa = raw / 64;
}

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

void packet_processor_init(void)
{
    memset(ball_state,   0, sizeof(ball_state));
    memset(last_decoded, 0, sizeof(last_decoded));
}

void packet_processor_process(const radio_packet_t *packet, uint32_t rx_time_ms)
{
    uint8_t id = packet->ball_id;
    if (id == 0 || id > MAX_BALLS) {
        SEGGER_RTT_printf(0, "RX: invalid ball_id %d\r\n", id);
        return;
    }

    ball_state_t *s = &ball_state[id - 1];

    // Decode t0 (current sample) for RTT display
    decode_sensor_data(&packet->data_t0, &last_decoded[id - 1]);

    if (!s->initialized) {
        s->last_sequence    = packet->sequence;
        s->packets_received = 1;
        s->packets_lost     = 0;
        s->rx_timestamp_ms  = rx_time_ms;
        s->initialized      = true;
        SEGGER_RTT_printf(0, "\r\n*** Ball %d: first packet seq=%u ***\r\n",
            id, packet->sequence);
        return;
    }

    // Sequence gap detection using 16-bit wrapping subtraction.
    //
    // Threshold of 500 (2 seconds at 250Hz) separates two cases:
    //   gap < 500: treat as lost packets and count them
    //   gap >= 500: treat as TX restart or sequence reset, log and ignore
    //
    // TX restarts appear as large apparent gaps (e.g. 65000+) due to uint16
    // wrapping, so they are always caught by the >= 500 branch.
    // Any sustained RF blackout longer than 2 seconds is a fatal performance
    // event regardless of exact packet count.
    uint16_t expected = (uint16_t)(s->last_sequence + 1);
    uint16_t received = packet->sequence;
    uint16_t gap      = (uint16_t)(received - expected);  // wrapping subtract

    if (gap > 0 && gap < 500) {
        s->packets_lost += gap;
        SEGGER_RTT_printf(0, "Ball %d: %u packet(s) lost before seq %u\r\n",
            id, gap, received);
    } else if (gap >= 500) {
        SEGGER_RTT_printf(0, "Ball %d: large gap %u at seq %u — TX restart?\r\n",
            id, gap, received);
    }

    s->last_sequence    = received;
    s->packets_received++;
    s->rx_timestamp_ms  = rx_time_ms;
}

void packet_processor_print_statistics(void)
{
    radio_stats_t rs;
    radio_get_stats(&rs);

    SEGGER_RTT_printf(0, "\r\n--- RX Stats ---\r\n");
    SEGGER_RTT_printf(0, "Radio: %lu OK  %lu CRC-fail  %lu END events\r\n",
        rs.total_packets, rs.crc_errors, rs.end_events);

    bool any_ball = false;
    for (int i = 0; i < MAX_BALLS; i++) {
        ball_state_t      *s = &ball_state[i];
        decoded_sensors_t *d = &last_decoded[i];

        if (!s->initialized || s->packets_received == 0) continue;
        any_ball = true;

        uint32_t total_expected = s->packets_received + s->packets_lost;

        // Use uint64_t intermediate to prevent overflow.
        // At 250Hz over 3 hours: up to 2,700,000 packets.
        // 2,700,000 * 10000 = 27,000,000,000 which overflows uint32_t (max ~4.29B).
        uint64_t loss_x10000 = (total_expected > 0)
            ? ((uint64_t)s->packets_lost * 10000ULL) / (uint64_t)total_expected
            : 0ULL;

        SEGGER_RTT_printf(0, "\r\nBall %d: RX=%lu  Lost=%lu (%lu.%02lu%%)  Seq=%u\r\n",
            i + 1,
            s->packets_received,
            s->packets_lost,
            (uint32_t)(loss_x10000 / 100),
            (uint32_t)(loss_x10000 % 100),
            s->last_sequence);

        SEGGER_RTT_printf(0, "  ACCEL: %6d %6d %6d\r\n",
            d->accel[0], d->accel[1], d->accel[2]);
        SEGGER_RTT_printf(0, "  GYRO:  %6d %6d %6d\r\n",
            d->gyro[0], d->gyro[1], d->gyro[2]);
        SEGGER_RTT_printf(0, "  MAG:   %6d %6d %6d\r\n",
            d->mag[0], d->mag[1], d->mag[2]);
        SEGGER_RTT_printf(0, "  H3LIS: %6d %6d %6d  FSR: lvl=%u pat=0x%X\r\n",
            d->h3lis[0], d->h3lis[1], d->h3lis[2],
            d->fsr_intensity, d->fsr_pattern);
        SEGGER_RTT_printf(0, "  BMP:   %lu Pa\r\n", d->pressure_pa);
    }

    if (!any_ball) {
        SEGGER_RTT_printf(0, "No balls seen yet\r\n");
    }
    SEGGER_RTT_printf(0, "\r\n");
}

bool packet_processor_get_ball_state(uint8_t ball_id, ball_state_t *state)
{
    if (ball_id == 0 || ball_id > MAX_BALLS || !state) return false;
    ball_state_t *s = &ball_state[ball_id - 1];
    if (!s->initialized) return false;
    memcpy(state, s, sizeof(ball_state_t));
    return true;
}
