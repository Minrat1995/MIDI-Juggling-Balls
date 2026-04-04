/**
 * SerialReader implementation
 *
 * USB framing: RX firmware sends 88 bytes per packet: 0xAA 0x55 + 86 payload.
 * The decoder finds packet boundaries by scanning for the two-byte sequence.
 * False-positive probability is 1/65536 per position — negligible.
 */

#include "SerialReader.h"

#include <setupapi.h>
#include <devguid.h>
#include <cassert>
#include <cstdio>
#include <cstring>

#pragma comment(lib, "setupapi.lib")

// Sync bytes — must match USB_SYNC_BYTE_0/1 in rx/main.c
static constexpr uint8_t SYNC_BYTE_0 = 0xAA;
static constexpr uint8_t SYNC_BYTE_1 = 0x55;

// ============================================================================
// AUTO-DETECT
// ============================================================================

std::string SerialReader::AutoDetectPort(uint16_t vid)
{
    HDEVINFO devInfo = SetupDiGetClassDevs(
        &GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);

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

    SetupDiDestroyDeviceInfoList(devInfo);
    return result;
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

    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(m_hPort, &dcb);
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;
    SetCommState(m_hPort, &dcb);

    COMMTIMEOUTS timeouts = {};
    timeouts.ReadTotalTimeoutConstant = 100;
    SetCommTimeouts(m_hPort, &timeouts);

    m_accumBuffer.clear();
    m_accumBuffer.reserve(MAX_ACCUM_BUFFER);

    m_running = true;
    m_thread  = std::thread(&SerialReader::ReaderThread, this);

    printf("SerialReader: opened %s\n", portName.c_str());
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
                fprintf(stderr, "SerialReader: ReadFile error %lu — stopping\n",
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

    if (m_accumBuffer.size() > MAX_ACCUM_BUFFER)
    {
        fprintf(stderr, "SerialReader: accumulation buffer overflow — flushing\n");
        m_accumBuffer.clear();
        m_resyncCount++;
        return;
    }

    // Framing: each USB frame is SYNC_BYTE_0 SYNC_BYTE_1 + 86 payload = 88 bytes.
    //
    // Steady state: buffer[0]==0xAA, buffer[1]==0x55 on every iteration.
    // Resync: scan forward for the next 0xAA 0x55 pair.
    //
    // After extracting a packet, the ball_id sanity check (1-8) provides a
    // secondary confirmation. A false sync from coincidental 0xAA 0x55 in
    // payload data has a 1/65536 chance per position, and even then the ball_id
    // check will catch most of those. In practice resync after a dropped USB
    // packet takes at most a handful of frames.
    static constexpr size_t FRAMED_SIZE = 2 + PACKET_SIZE;  // 88 bytes

    while (m_accumBuffer.size() >= FRAMED_SIZE)
    {
        if (m_accumBuffer[0] == SYNC_BYTE_0 && m_accumBuffer[1] == SYNC_BYTE_1)
        {
            // Sync found — extract packet.
            radio_packet_t pkt;
            memcpy(&pkt, m_accumBuffer.data() + 2, PACKET_SIZE);
            m_accumBuffer.erase(m_accumBuffer.begin(),
                                m_accumBuffer.begin() + FRAMED_SIZE);

            if (pkt.ball_id < 1 || pkt.ball_id > 8)
            {
                // Coincidental 0xAA 0x55 in payload — discard and resync.
                fprintf(stderr,
                        "SerialReader: false sync (ball_id=%u) — resyncing\n",
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
            // Not aligned. Scan for the next 0xAA 0x55.
            // Start at index 1 (index 0 is already confirmed not to start a pair).
            size_t syncPos = std::string::npos;
            for (size_t i = 1; i + 1 < m_accumBuffer.size(); ++i)
            {
                if (m_accumBuffer[i]     == SYNC_BYTE_0 &&
                    m_accumBuffer[i + 1] == SYNC_BYTE_1)
                {
                    syncPos = i;
                    break;
                }
            }

            if (syncPos == std::string::npos)
            {
                // No sync pair found. Keep the last (FRAMED_SIZE - 1) bytes
                // in case the pair straddles the next read chunk.
                if (m_accumBuffer.size() > FRAMED_SIZE - 1)
                {
                    size_t discard = m_accumBuffer.size() - (FRAMED_SIZE - 1);
                    m_accumBuffer.erase(m_accumBuffer.begin(),
                                        m_accumBuffer.begin() + discard);
                    m_resyncCount++;
                }
                return;
            }

            fprintf(stderr,
                    "SerialReader: resync #%u — discarding %zu bytes\n",
                    m_resyncCount.load() + 1, syncPos);

            m_accumBuffer.erase(m_accumBuffer.begin(),
                                 m_accumBuffer.begin() + syncPos);
            m_resyncCount++;
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
