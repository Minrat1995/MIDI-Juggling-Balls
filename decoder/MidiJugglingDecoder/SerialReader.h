/**
 * SerialReader
 *
 * Reads 86-byte radio_packet_t structs from the RX Feather over USB CDC.
 *
 * Design notes:
 *
 * Auto-detection searches Windows device list for VID 0x239A (Adafruit).
 * If multiple Adafruit devices are present, the first matching COM port
 * is used. Call AutoDetectPort() and inspect the result before opening
 * if you need to disambiguate.
 *
 * USB CDC does not guarantee 86-byte aligned reads. A background thread
 * accumulates raw bytes and extracts complete packets using the 0xAA 0x55
 * sync header and XOR checksum defined in packet_spec.h. Sync and framing
 * constants come from packet_spec.h (USB_SYNC_BYTE_0/1, USB_FRAME_SIZE).
 *
 * On misalignment, the buffer is advanced one byte at a time until a valid
 * sync header and matching checksum are found. Misalignment is rare in
 * normal operation but can occur at startup or after a port glitch.
 *
 * Packet queue:
 *   Extracted packets are queued up to MAX_QUEUE_DEPTH entries. If the
 *   consumer (main loop) falls behind and the queue fills, the oldest
 *   packet is dropped to make room for the newest. GetQueueDropCount()
 *   returns the total number of drops since Open(). During single-ball
 *   operation this should remain zero; a non-zero count during multi-ball
 *   testing indicates the consumer loop cannot keep up.
 *
 * TryGetPacket() is safe to call from any thread.
 *
 * The RX firmware sends packets only when usb_serial_ready() is true
 * (host has opened the port). If the port is opened after the RX boots,
 * the first packet will arrive once the host opens it — no data is lost
 * on the embedded side before that point since the firmware buffers nothing.
 */

#pragma once

#include <windows.h>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <cstdint>

#include "packet_spec.h"

class SerialReader
{
public:
    SerialReader()  = default;
    ~SerialReader() { Close(); }

    // Non-copyable, non-movable (owns a thread and HANDLE)
    SerialReader(const SerialReader&)            = delete;
    SerialReader& operator=(const SerialReader&) = delete;

    // ------------------------------------------------------------------
    // Setup
    // ------------------------------------------------------------------

    /**
     * Search for a COM port belonging to an Adafruit device (VID 0x239A).
     * Returns the port name (e.g. "COM5") or empty string if not found.
     * Requires setupapi.lib.
     */
    static std::string AutoDetectPort(uint16_t vid = 0x239A);

    /**
     * Open the serial port and start the background reader thread.
     * portName: "COM5" etc. Ports > 9 are handled automatically.
     * Returns false if the port cannot be opened.
     */
    bool Open(const std::string& portName);

    /**
     * Stop the reader thread and close the port.
     * Safe to call multiple times.
     */
    void Close();

    bool IsOpen() const { return m_hPort != INVALID_HANDLE_VALUE; }

    // ------------------------------------------------------------------
    // Packet retrieval
    // ------------------------------------------------------------------

    /**
     * Non-blocking. Fills packet and returns true if one is available.
     * Returns false if no packet is ready.
     * Safe to call from any thread.
     */
    bool TryGetPacket(radio_packet_t& packet);

    // ------------------------------------------------------------------
    // Diagnostics
    // ------------------------------------------------------------------

    uint32_t GetBytesReceived()   const { return m_bytesReceived.load(); }
    uint32_t GetPacketsReceived() const { return m_packetsReceived.load(); }
    uint32_t GetResyncCount()     const { return m_resyncCount.load(); }

    /**
     * Number of packets dropped from the queue because the consumer loop
     * fell behind. Should remain zero during single-ball operation.
     * A non-zero count during multi-ball testing means the main loop's
     * Sleep(1) / processing budget is insufficient for the incoming rate.
     */
    uint32_t GetQueueDropCount()  const { return m_queueDropCount.load(); }

private:
    void ReaderThread();
    void ProcessBytes(const uint8_t* data, size_t len);

    HANDLE      m_hPort  = INVALID_HANDLE_VALUE;
    std::thread m_thread;
    std::atomic<bool> m_running{ false };

    // Raw byte accumulation buffer (reader thread only — no lock needed)
    std::vector<uint8_t> m_accumBuffer;

    // Completed packets waiting for the consumer
    std::queue<radio_packet_t> m_packetQueue;
    std::mutex                 m_queueMutex;

    std::atomic<uint32_t> m_bytesReceived  { 0 };
    std::atomic<uint32_t> m_packetsReceived{ 0 };
    std::atomic<uint32_t> m_resyncCount    { 0 };
    std::atomic<uint32_t> m_queueDropCount { 0 };

    static constexpr size_t PACKET_SIZE      = sizeof(radio_packet_t); // 86
    static constexpr size_t MAX_ACCUM_BUFFER = 4096; // ~47 packets; runaway guard

    // Queue depth limit. At 250Hz, 500 entries = 2 seconds of backlog.
    // When full, the oldest packet is dropped to admit the newest.
    // Reviewed at Phase 3 multi-ball transition: 3 balls at 250Hz each =
    // 750Hz arrival rate, so the same 500-entry limit covers ~0.67s.
    static constexpr size_t MAX_QUEUE_DEPTH  = 500;
};
