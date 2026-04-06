/**
 * SerialReader implementation
 *
 * USB framing: RX firmware sends 89 bytes per packet:
 *   [0]    0xAA  sync byte 0
 *   [1]    0x55  sync byte 1
 *   [2-87] radio_packet_t payload (86 bytes)
 *   [88]   XOR checksum of bytes [2-87]
 *
 * Packet boundary detection:
 *   Scan for 0xAA 0x55 at buffer[0..1]. If found, extract 86 bytes,
 *   recompute XOR checksum, compare to byte[88]. A mismatch means the
 *   sync header was a false positive (0xAA 0x55 appearing in payload
 *   data) — discard one byte and resync. A match means the frame is
 *   valid and the packet is enqueued.
 *
 * DTR assertion:
 *   EscapeCommFunction(SETDTR) is called after opening the port to
 *   trigger APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN on the RX firmware.
 *   Without this, usb_serial_ready() stays false and no bytes are sent.
 *
 * Auto-detection:
 *   Two-pass search. Pass 1 scans GUID_DEVCLASS_PORTS (devices already
 *   classified as COM ports by Windows). Pass 2 scans all device classes
 *   via DIGCF_ALLCLASSES — catches the nRF52840 CDC ACM device when the
 *   usbser.sys driver has not yet been matched (first connection, missing
 *   INF, or composite USB device where the child interface node has not
 *   been assigned a class yet). Both passes filter by VID 0x239A and look
 *   for a PortName registry value under the device's driver key.
 */

#include "SerialReader.h"

#include <setupapi.h>
#include <devguid.h>
#include <cassert>
#include <cstdio>
#include <cstring>

#pragma comment(lib, "setupapi.lib")

static constexpr uint8_t SYNC_BYTE_1 = 0xAA;
static constexpr uint8_t SYNC_BYTE_2 = 0x55;

// ============================================================================
// AUTO-DETECT
// ============================================================================

/**
 * Scan a device info set for a device matching the given VID that has a
 * COM port name in its driver registry key.
 *
 * Factored out of AutoDetectPort so it can be called with different
 * HDEVINFO sets (Ports class, all classes) without duplicating the
 * per-device enumeration logic.
 *
 * @param devInfo  Device info set to scan. Must not be INVALID_HANDLE_VALUE.
 * @param vid      USB Vendor ID to match (e.g. 0x239A for Adafruit).
 * @return         COM port name (e.g. "COM5") or empty string if not found.
 */
static std::string ScanDeviceInfoSet(HDEVINFO devInfo, uint16_t vid)
{
    if (devInfo == INVALID_HANDLE_VALUE)
        return {};

    char vidPattern[16];
    snprintf(vidPattern, sizeof(vidPattern), "VID_%04X", vid);

    std::string result;

    SP_DEVINFO_DATA devData = {};
    devData.cbSize = sizeof(devData);

    for (DWORD i = 0; SetupDiEnumDeviceInfo(devInfo, i, &devData); ++i)
    {
        char hwId[512] = {};
        if (!SetupDiGetDeviceRegistryPropertyA(
                devInfo, &devData,
                SPDRP_HARDWAREID, nullptr,
                reinterpret_cast<PBYTE>(hwId), sizeof(hwId) - 1, nullptr))
        {
            continue;
        }

        if (strstr(hwId, vidPattern) == nullptr)
            continue;

        HKEY hKey = SetupDiOpenDevRegKey(
            devInfo, &devData,
            DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);

        if (hKey == INVALID_HANDLE_VALUE)
            continue;

        char  portName[32] = {};
        DWORD size = sizeof(portName) - 1;
        DWORD type = REG_SZ;
        RegQueryValueExA(hKey, "PortName", nullptr, &type,
                         reinterpret_cast<LPBYTE>(portName), &size);
        RegCloseKey(hKey);

        if (strncmp(portName, "COM", 3) == 0)
        {
            result = portName;
            break;
        }
    }

    return result;
}

std::string SerialReader::AutoDetectPort(uint16_t vid)
{
    // Pass 1: Ports device class.
    //
    // GUID_DEVCLASS_PORTS is the fast path and covers the common case: the
    // nRF52840 CDC ACM interface enumerated correctly and Windows has matched
    // it to usbser.sys, placing it under the Ports class. This happens on any
    // machine that has connected an nRF52840 Feather before, or on Windows 10+
    // where usbser.sys matches CDC ACM devices without a custom INF.
    {
        HDEVINFO h = SetupDiGetClassDevs(
            &GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);
        std::string r = ScanDeviceInfoSet(h, vid);
        SetupDiDestroyDeviceInfoList(h);
        if (!r.empty()) return r;
    }

    // Pass 2: All device classes.
    //
    // If Pass 1 found nothing, the device may be present but not yet assigned
    // to the Ports class. This happens on first connection before usbser.sys
    // has been matched, or on systems where the CDC ACM driver is installed
    // but the device node hasn't been fully enumerated yet (e.g. the parent
    // composite device node exists but the child interface node's class hasn't
    // been written). DIGCF_ALLCLASSES enumerates every present device
    // regardless of class. We still filter by VID and require a PortName
    // registry value, so only actual COM-port-bearing devices are returned.
    //
    // This pass is slower (scans all devices on the system) but only runs
    // when Pass 1 found nothing, which is the uncommon case.
    {
        HDEVINFO h = SetupDiGetClassDevs(
            nullptr, nullptr, nullptr, DIGCF_PRESENT | DIGCF_ALLCLASSES);
        std::string r = ScanDeviceInfoSet(h, vid);
        SetupDiDestroyDeviceInfoList(h);
        if (!r.empty()) return r;
    }

    return {};
}

// ============================================================================
// OPEN / CLOSE
// ============================================================================

bool SerialReader::Open(const std::string& portName)
{
    std::string path = "\\\\.\\" + portName;

    m_hPort = CreateFileA(
        path.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0, nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);

    if (m_hPort == INVALID_HANDLE_VALUE)
    {
        fprintf(stderr, "SerialReader: failed to open %s (error %lu)\n",
                portName.c_str(), GetLastError());
        return false;
    }

    // USB CDC ignores baud rate but DCB must be configured to avoid
    // ReadFile errors on some Windows versions.
    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(m_hPort, &dcb);
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;
    SetCommState(m_hPort, &dcb);

    // ReadFile blocks for up to 100ms if no data arrives, preventing
    // the reader thread from spinning at 100% CPU when idle.
    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout         = 0;
    timeouts.ReadTotalTimeoutMultiplier  = 0;
    timeouts.ReadTotalTimeoutConstant    = 100;
    SetCommTimeouts(m_hPort, &timeouts);

    // Assert DTR to trigger APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN on the RX
    // firmware. Without this, the RX never sees the port as open and
    // usb_serial_ready() stays false — no bytes are ever sent.
    EscapeCommFunction(m_hPort, SETDTR);

    // Allow the RX USB stack time to process the PORT_OPEN event before
    // the reader thread starts consuming bytes.
    Sleep(200);

    m_accumBuffer.clear();
    m_accumBuffer.reserve(MAX_ACCUM_BUFFER);

    m_running = true;
    m_thread  = std::thread(&SerialReader::ReaderThread, this);

    printf("SerialReader: opened %s (DTR asserted)\n", portName.c_str());
    return true;
}

void SerialReader::Close()
{
    m_running = false;

    if (m_hPort != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_hPort);
        m_hPort = INVALID_HANDLE_VALUE;
    }

    if (m_thread.joinable())
        m_thread.join();
}

// ============================================================================
// READER THREAD
// ============================================================================

void SerialReader::ReaderThread()
{
    constexpr size_t READ_CHUNK = 256;
    uint8_t buf[READ_CHUNK];
    DWORD   bytesRead = 0;

    while (m_running)
    {
        BOOL ok = ReadFile(m_hPort, buf, READ_CHUNK, &bytesRead, nullptr);

        if (!ok)
        {
            if (m_running)
                fprintf(stderr, "SerialReader: ReadFile error %lu - stopping\n",
                        GetLastError());
            break;
        }

        if (bytesRead > 0)
        {
            m_bytesReceived += bytesRead;
            ProcessBytes(buf, bytesRead);
        }
    }
}

// ============================================================================
// PACKET EXTRACTION
// ============================================================================

void SerialReader::ProcessBytes(const uint8_t* data, size_t len)
{
    m_accumBuffer.insert(m_accumBuffer.end(), data, data + len);

    // Runaway guard.
    if (m_accumBuffer.size() > MAX_ACCUM_BUFFER)
    {
        fprintf(stderr, "SerialReader: accumulation buffer overflow - flushing\n");
        m_accumBuffer.clear();
        m_resyncCount++;
        return;
    }

    // Frame layout: SYNC_BYTE_1 SYNC_BYTE_2 [86 payload bytes] [1 checksum] = 89 bytes.
    //
    // Steps:
    //   1. Check buffer[0..1] == 0xAA 0x55
    //   2. Extract 86 payload bytes from buffer[2..87]
    //   3. Compute XOR of those 86 bytes
    //   4. Compare to buffer[88]
    //   5. Mismatch = false sync: discard one byte, increment resync, retry
    //   6. Match = valid frame: enqueue packet
    static constexpr size_t FRAMED_SIZE = 2 + PACKET_SIZE + 1; // 89 bytes

    while (m_accumBuffer.size() >= FRAMED_SIZE)
    {
        if (m_accumBuffer[0] == SYNC_BYTE_1 && m_accumBuffer[1] == SYNC_BYTE_2)
        {
            // Sync header found. Verify checksum before accepting.
            uint8_t computed = 0;
            for (size_t i = 2; i < 2 + PACKET_SIZE; i++)
                computed ^= m_accumBuffer[i];

            uint8_t received = m_accumBuffer[2 + PACKET_SIZE];

            if (computed != received)
            {
                // Checksum mismatch: this 0xAA 0x55 was in the payload data,
                // not a real frame boundary. Discard one byte and resync.
                m_accumBuffer.erase(m_accumBuffer.begin());
                m_resyncCount++;

                if (m_resyncCount <= 10 || m_resyncCount % 1000 == 0)
                {
                    fprintf(stderr,
                        "SerialReader: checksum mismatch (computed=0x%02X received=0x%02X)"
                        " resync #%u\n",
                        computed, received, m_resyncCount.load());
                }
                continue;
            }

            // Checksum OK. Extract the payload.
            radio_packet_t pkt;
            memcpy(&pkt, m_accumBuffer.data() + 2, PACKET_SIZE);
            m_accumBuffer.erase(m_accumBuffer.begin(),
                                m_accumBuffer.begin() + FRAMED_SIZE);

            // Belt-and-suspenders: ball_id must still be 1-8.
            // A valid checksum on a false-aligned frame is possible with
            // probability 1/256 — this catches that residual case.
            if (pkt.ball_id < 1 || pkt.ball_id > 8)
            {
                fprintf(stderr,
                    "SerialReader: valid checksum but bad ball_id %u - discarding\n",
                    pkt.ball_id);
                m_resyncCount++;
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(m_queueMutex);
                m_packetQueue.push(pkt);
            }
            m_packetsReceived++;
        }
        else
        {
            // No sync header at buffer[0]. Discard one byte and retry.
            uint8_t b0 = m_accumBuffer[0];
            uint8_t b1 = m_accumBuffer.size() > 1 ? m_accumBuffer[1] : 0;
            m_accumBuffer.erase(m_accumBuffer.begin());
            m_resyncCount++;

            if (m_resyncCount <= 10 || m_resyncCount % 1000 == 0)
            {
                fprintf(stderr,
                    "SerialReader: resync #%u (expected 0xAA 0x55, got 0x%02X 0x%02X)\n",
                    m_resyncCount.load(), b0, b1);
            }
        }
    }
}

// ============================================================================
// PACKET RETRIEVAL (consumer thread)
// ============================================================================

bool SerialReader::TryGetPacket(radio_packet_t& packet)
{
    std::lock_guard<std::mutex> lock(m_queueMutex);
    if (m_packetQueue.empty())
        return false;

    packet = m_packetQueue.front();
    m_packetQueue.pop();
    return true;
}
