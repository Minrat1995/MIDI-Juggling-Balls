/**
 * SerialReader implementation
 *
 * USB framing: RX firmware sends USB_FRAME_SIZE (89) bytes per packet:
 *   [0]    USB_SYNC_BYTE_0 (0xAA)
 *   [1]    USB_SYNC_BYTE_1 (0x55)
 *   [2-87] radio_packet_t payload (86 bytes)
 *   [88]   XOR checksum of bytes [2-87]
 *
 * Framing constants are defined in packet_spec.h and shared with the
 * RX firmware. Do not redefine them here.
 *
 * Packet boundary detection:
 *   Scan for USB_SYNC_BYTE_0/1 at the current buffer position. If found,
 *   extract 86 bytes, recompute XOR checksum, compare to byte[88]. A
 *   mismatch means the sync header was a false positive (0xAA 0x55
 *   appearing in payload data) — advance one byte and retry. A match
 *   means the frame is valid and the packet is enqueued.
 *
 *   All advancement through the accumulation buffer is done via a
 *   read_pos index; the single erase at the end of ProcessBytes removes
 *   all consumed bytes in one operation rather than one byte at a time.
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

/**
 * Enumerate all active COM ports from the Windows serial device map registry
 * key: HKLM\HARDWARE\DEVICEMAP\SERIALCOMM
 *
 * This key is maintained by Windows and lists every COM port that is currently
 * active (driver loaded, device present). It does not require knowing the VID,
 * PID, or device class — it works for any COM port regardless of how Windows
 * has classified the underlying device.
 *
 * Used as a last-resort pass in AutoDetectPort when VID-based matching fails.
 * The nRF52840 Feather can enumerate as a generic "USB Serial Device" without
 * exposing its Adafruit VID to the Ports device class, which defeats both
 * VID-based passes. This pass finds it regardless.
 *
 * @return  Vector of COM port names (e.g. {"COM14", "COM3"}) currently active.
 *          Empty if none found or registry key unavailable.
 */
static std::vector<std::string> EnumerateAllComPorts()
{
    std::vector<std::string> ports;

    HKEY hKey;
    LONG result = RegOpenKeyExA(
        HKEY_LOCAL_MACHINE,
        "HARDWARE\\DEVICEMAP\\SERIALCOMM",
        0, KEY_READ, &hKey);

    if (result != ERROR_SUCCESS)
        return ports;

    DWORD index = 0;
    char  valueName[256];
    char  portName[64];
    DWORD nameLen, dataLen, type;

    while (true)
    {
        nameLen = sizeof(valueName);
        dataLen = sizeof(portName);
        type    = REG_SZ;

        result = RegEnumValueA(hKey, index++,
                               valueName, &nameLen,
                               nullptr, &type,
                               reinterpret_cast<LPBYTE>(portName), &dataLen);

        if (result == ERROR_NO_MORE_ITEMS)
            break;

        if (result == ERROR_SUCCESS && type == REG_SZ &&
            strncmp(portName, "COM", 3) == 0)
        {
            ports.push_back(portName);
        }
    }

    RegCloseKey(hKey);
    return ports;
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
        if (!r.empty())
        {
            printf("Auto-detect: found via Ports class (VID 0x%04X): %s\n",
                   vid, r.c_str());
            return r;
        }
    }

    // Pass 2: All device classes.
    //
    // If Pass 1 found nothing, the device may be present but not yet assigned
    // to the Ports class. DIGCF_ALLCLASSES enumerates every present device
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
        if (!r.empty())
        {
            printf("Auto-detect: found via all-classes scan (VID 0x%04X): %s\n",
                   vid, r.c_str());
            return r;
        }
    }

    // Pass 3: All active COM ports (no VID filtering).
    //
    // Both VID-based passes failed. The device is present but not advertising
    // its Adafruit VID — typically because Windows has loaded the generic
    // usbser.sys driver without associating the VID (this is the known
    // behaviour on the development machine where the Feather enumerates as
    // "USB Serial Device" on COM14).
    //
    // Read HKLM\HARDWARE\DEVICEMAP\SERIALCOMM, which lists every active COM
    // port regardless of device class or VID. If exactly one port is active,
    // use it. If multiple ports are present, list them and require the user
    // to specify explicitly — we cannot distinguish the RX Feather from other
    // COM devices without VID information.
    {
        std::vector<std::string> ports = EnumerateAllComPorts();

        if (ports.empty())
        {
            fprintf(stderr,
                "Auto-detect: no active COM ports found in SERIALCOMM registry.\n");
            return {};
        }

        if (ports.size() == 1)
        {
            printf("Auto-detect: VID 0x%04X not found; using sole active port: %s\n",
                   vid, ports[0].c_str());
            return ports[0];
        }

        // Multiple ports — cannot safely guess which is the RX Feather.
        fprintf(stderr,
            "Auto-detect: VID 0x%04X not found, and multiple COM ports are active.\n"
            "  Cannot determine which is the RX Feather. Active ports:\n",
            vid);
        for (const auto& p : ports)
            fprintf(stderr, "    %s\n", p.c_str());
        fprintf(stderr,
            "  Pass the correct port explicitly: decoder.exe COM14\n");
        return {};
    }
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

    // Cancel any blocking ReadFile before closing the handle.
    //
    // Without CancelSynchronousIo, the sequence is: set m_running=false,
    // CloseHandle, join. If the thread is inside ReadFile when CloseHandle
    // fires, ReadFile returns an error and the thread exits via the !ok path —
    // this works in practice, but CloseHandle on a HANDLE that is still
    // being used in a synchronous API call is technically a race. Issuing
    // CancelSynchronousIo first signals the OS to abort the in-progress
    // ReadFile cleanly before we touch the handle.
    //
    // CancelSynchronousIo returns FALSE with ERROR_NOT_FOUND if the thread
    // is not currently blocked in a synchronous call; that is not an error.
    if (m_thread.joinable())
        CancelSynchronousIo(m_thread.native_handle());

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

    // Frame layout: USB_SYNC_BYTE_0 USB_SYNC_BYTE_1 [86 payload bytes] [1 checksum]
    // = USB_FRAME_SIZE (89) bytes total.
    //
    // read_pos advances through m_accumBuffer without erasing. All consumed
    // bytes are removed in a single erase at the end of the function.
    // This avoids O(n) shifts on every misaligned byte during resync.
    static constexpr size_t FRAMED_SIZE = static_cast<size_t>(USB_FRAME_SIZE);

    size_t read_pos = 0;

    while (read_pos + FRAMED_SIZE <= m_accumBuffer.size())
    {
        if (m_accumBuffer[read_pos]     == USB_SYNC_BYTE_0 &&
            m_accumBuffer[read_pos + 1] == USB_SYNC_BYTE_1)
        {
            // Sync header found. Verify checksum before accepting.
            uint8_t computed = 0;
            for (size_t i = read_pos + 2; i < read_pos + 2 + PACKET_SIZE; i++)
                computed ^= m_accumBuffer[i];

            uint8_t received = m_accumBuffer[read_pos + 2 + PACKET_SIZE];

            if (computed != received)
            {
                // Checksum mismatch: this 0xAA 0x55 was in the payload data,
                // not a real frame boundary. Advance one byte and resync.
                uint32_t count = ++m_resyncCount;
                if (count <= 10 || count % 1000 == 0)
                {
                    fprintf(stderr,
                        "SerialReader: checksum mismatch (computed=0x%02X received=0x%02X)"
                        " resync #%u\n",
                        computed, received, count);
                }
                read_pos++;
                continue;
            }

            // Checksum OK. Extract the payload.
            radio_packet_t pkt;
            memcpy(&pkt, m_accumBuffer.data() + read_pos + 2, PACKET_SIZE);
            read_pos += FRAMED_SIZE;

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
                if (m_packetQueue.size() >= MAX_QUEUE_DEPTH)
                {
                    // Queue is full. Drop the oldest packet to make room for
                    // the newest — current sensor state is more useful than
                    // stale data from 2+ seconds ago.
                    m_packetQueue.pop();
                    uint32_t drops = ++m_queueDropCount;
                    if (drops <= 5 || drops % 100 == 0)
                    {
                        fprintf(stderr,
                            "SerialReader: queue full, oldest packet dropped"
                            " (total drops=%u)\n", drops);
                    }
                }
                m_packetQueue.push(pkt);
            }
            m_packetsReceived++;
        }
        else
        {
            // No sync header at read_pos. Advance one byte and retry.
            uint32_t count = ++m_resyncCount;
            if (count <= 10 || count % 1000 == 0)
            {
                fprintf(stderr,
                    "SerialReader: resync #%u (expected 0xAA 0x55, got 0x%02X 0x%02X)\n",
                    count,
                    m_accumBuffer[read_pos],
                    (read_pos + 1 < m_accumBuffer.size()) ? m_accumBuffer[read_pos + 1] : 0u);
            }
            read_pos++;
        }
    }

    // Remove all consumed bytes in one shot.
    if (read_pos > 0)
        m_accumBuffer.erase(m_accumBuffer.begin(),
                            m_accumBuffer.begin() + static_cast<std::ptrdiff_t>(read_pos));
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
