/**
 * MIDI Juggling Balls — PC Decoder
 * Phase 1.5: USB read + console decode validation
 *
 * Run with no arguments to auto-detect the RX Feather.
 * Run with a COM port argument to force a specific port:
 *   decoder.exe COM5
 *
 * Expected output at rest (Ball 1):
 *   Ball 1 | seq=XXXX | ACCEL: x=+0.00g y=+0.00g z=+1.00g | ...
 *
 * OSC output is stubbed. Implement OscSender and call it after
 * decoding each packet once this console output looks correct.
 */

#include <windows.h>
#include <cstdio>
#include <cstdint>
#include <cstring>

#include "packet_spec.h"
#include "SerialReader.h"

// ============================================================================
// DECODED SENSOR VALUES (physical units)
// ============================================================================

struct DecodedPacket
{
    uint8_t  ball_id;
    uint16_t sequence;
    uint16_t timestamp_ms;

    // t0 (current sample) decoded to physical units
    float accel_g[3];       // LSM6DSOX, g
    float gyro_dps[3];      // LSM6DSOX, dps
    float mag_gauss[3];     // LIS3MDL, gauss
    float h3lis_g[3];       // H3LIS331, g
    float pressure_pa;      // BMP581, Pa
    uint8_t fsr_intensity;  // 0-15
    uint8_t fsr_pattern;    // bitmask
};

static void decode_packet(const radio_packet_t& raw, DecodedPacket& out)
{
    out.ball_id      = raw.ball_id;
    out.sequence     = raw.sequence;
    out.timestamp_ms = raw.timestamp;

    const sensor_data_t& s = raw.data_t0;

    out.accel_g[0]   = accel_to_g(s.imu.accel[0]);
    out.accel_g[1]   = accel_to_g(s.imu.accel[1]);
    out.accel_g[2]   = accel_to_g(s.imu.accel[2]);

    out.gyro_dps[0]  = gyro_to_dps(s.imu.gyro[0]);
    out.gyro_dps[1]  = gyro_to_dps(s.imu.gyro[1]);
    out.gyro_dps[2]  = gyro_to_dps(s.imu.gyro[2]);

    out.mag_gauss[0] = mag_to_gauss(s.imu.mag[0]);
    out.mag_gauss[1] = mag_to_gauss(s.imu.mag[1]);
    out.mag_gauss[2] = mag_to_gauss(s.imu.mag[2]);

    out.h3lis_g[0]   = h3lis_to_g(s.h3lis_x_fsr_level);
    out.h3lis_g[1]   = h3lis_to_g(s.h3lis_y_fsr_pattern);
    out.h3lis_g[2]   = h3lis_to_g(s.h3lis_z_flags);

    out.pressure_pa  = pressure_to_pa(s.pressure);

    out.fsr_intensity = static_cast<uint8_t>(s.h3lis_x_fsr_level  & 0x0F);
    out.fsr_pattern   = static_cast<uint8_t>(s.h3lis_y_fsr_pattern & 0x0F);
}

// ============================================================================
// OSC OUTPUT STUB
// ============================================================================

// TODO: Replace with OscSender once console output validates correctly.
// Suggested OSC path layout (per project spec):
//   /ball/N/accel   [x y z] g
//   /ball/N/gyro    [x y z] dps
//   /ball/N/mag     [x y z] gauss
//   /ball/N/h3lis   [x y z] g
//   /ball/N/pressure float Pa
//   /ball/N/fsr     intensity(int) pattern(int)
static void send_osc(const DecodedPacket& /*pkt*/)
{
    // Not yet implemented.
}

// ============================================================================
// CONSOLE PRINT (Phase 1.5 validation output)
// ============================================================================

static void print_packet(const DecodedPacket& p)
{
    printf("Ball %d | seq=%5u | ts=%5u ms\n",
           p.ball_id, p.sequence, p.timestamp_ms);

    printf("  ACCEL  %+6.2fg %+6.2fg %+6.2fg\n",
           p.accel_g[0], p.accel_g[1], p.accel_g[2]);

    printf("  GYRO   %+7.1f %+7.1f %+7.1f dps\n",
           p.gyro_dps[0], p.gyro_dps[1], p.gyro_dps[2]);

    printf("  MAG    %+6.3f %+6.3f %+6.3f gauss\n",
           p.mag_gauss[0], p.mag_gauss[1], p.mag_gauss[2]);

    printf("  H3LIS  %+7.1fg %+7.1fg %+7.1fg\n",
           p.h3lis_g[0], p.h3lis_g[1], p.h3lis_g[2]);

    printf("  BMP    %.1f Pa\n", p.pressure_pa);

    printf("  FSR    intensity=%u  pattern=0x%X\n",
           p.fsr_intensity, p.fsr_pattern);
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char* argv[])
{
    printf("=== MIDI Juggling Balls Decoder ===\n\n");

    // --- Resolve port name ---
    std::string portName;

    if (argc >= 2)
    {
        portName = argv[1];
        printf("Using specified port: %s\n", portName.c_str());
    }
    else
    {
        printf("Auto-detecting Adafruit device (VID 0x239A)...\n");
        portName = SerialReader::AutoDetectPort();

        if (portName.empty())
        {
            fprintf(stderr,
                "\nNo Adafruit CDC device found.\n"
                "Check:\n"
                "  1. RX Feather is plugged in and powered\n"
                "  2. USB cable connected before RX boot (hot-plug not supported)\n"
                "  3. nRF52840 Feather CDC driver is installed\n"
                "  4. Device shows in Device Manager under 'Ports (COM & LPT)'\n"
                "\nOr pass a port name explicitly: decoder.exe COM5\n");
            return 1;
        }

        printf("Found: %s\n", portName.c_str());
    }

    // --- Open port ---
    SerialReader reader;
    if (!reader.Open(portName))
    {
        fprintf(stderr, "\nFailed to open %s.\n"
                "If another program (serial monitor, SES RTT viewer) has the "
                "port open, close it first.\n", portName.c_str());
        return 1;
    }

    printf("\nListening for packets. Press Ctrl+C to stop.\n");
    printf("Expected at rest: ACCEL ~1g on one axis, BMP ~101325 Pa\n\n");

    // --- Main loop ---
    uint32_t printedPackets  = 0;
    uint32_t lastStatsTime   = GetTickCount();
    uint32_t lastSeq[9]      = {};    // per-ball, index 1-8
    uint32_t lostPackets[9]  = {};
    uint32_t rxPackets[9]    = {};
    bool     seenBall[9]     = {};

    // Print every Nth packet to avoid flooding the console.
    // At 250Hz this prints ~25 lines/sec — adjust as needed.
    constexpr uint32_t PRINT_EVERY = 10;

    while (true)
    {
        radio_packet_t raw;
        if (reader.TryGetPacket(raw))
        {
            uint8_t id = raw.ball_id;
            if (id >= 1 && id <= 8)
            {
                // Track sequence gaps
                if (seenBall[id])
                {
                    uint16_t expected = static_cast<uint16_t>(lastSeq[id] + 1);
                    uint16_t gap = static_cast<uint16_t>(raw.sequence - expected);
                    if (gap > 0 && gap < 500)
                        lostPackets[id] += gap;
                }
                else
                {
                    printf("Ball %d: first packet seq=%u\n\n", id, raw.sequence);
                    seenBall[id] = true;
                }

                lastSeq[id] = raw.sequence;
                rxPackets[id]++;

                // Decode
                DecodedPacket decoded;
                decode_packet(raw, decoded);

                // OSC output (stub)
                send_osc(decoded);

                // Console output (throttled)
                if (printedPackets % PRINT_EVERY == 0)
                    print_packet(decoded);

                printedPackets++;
            }
        }

        // Print stats every 5 seconds
        uint32_t now = GetTickCount();
        if (now - lastStatsTime >= 5000)
        {
            lastStatsTime = now;
            printf("\n--- Stats ---\n");
            printf("Serial: %u bytes  %u packets  %u resyncs\n",
                   reader.GetBytesReceived(),
                   reader.GetPacketsReceived(),
                   reader.GetResyncCount());

            for (int i = 1; i <= 8; i++)
            {
                if (!seenBall[i]) continue;
                uint32_t total = rxPackets[i] + lostPackets[i];
                float lossRate = total > 0
                    ? 100.0f * lostPackets[i] / static_cast<float>(total)
                    : 0.0f;
                printf("Ball %d: rx=%u  lost=%u  loss=%.2f%%\n",
                       i, rxPackets[i], lostPackets[i], lossRate);
            }
            printf("\n");
        }

        // Yield briefly to avoid spinning at 100% CPU when no packets arrive.
        // At 250Hz a packet arrives every 4ms so this does not add latency.
        Sleep(1);
    }

    reader.Close();
    return 0;
}
