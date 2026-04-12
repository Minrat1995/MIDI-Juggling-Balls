# MIDI Juggling Balls — Project Reference

**Version:** Post exhaustive TX code review (TX v3.14, RX v1.5, Decoder v1.6, full pipeline validated clean)
**Goal:** Wireless juggling ball → sensor data → sound reinforcing visual performance
**Repo:** https://github.com/Minrat1995/MIDI-Juggling-Balls (private)
**Target:** <20ms perceived latency, <1% packet loss, scalable to 3+ balls

---

## CURRENT STATUS

### What is done
- TX firmware v3.14: exhaustive multi-round code review complete. All identified issues
  resolved. Validated running on hardware (RTT confirmed, all sensors OK, pipeline clean).
  TX has run for 2+ hours without freeze on J-Link RTT — freeze issue may be resolved or
  intermittent; continue monitoring under extended conditions.
  Code considered production-ready pending watchdog addition and RX equivalent review.
  Key fixes across v3.7–v3.14: CRCPOLY/CRCINIT/PREFIX0 readback added to radio_init;
  sensors_test() called at startup; get_timestamp_ms() renamed get_rtc_ticks();
  HFCLK/LFCLK startup timeouts (10ms/1000ms); saadc_stop_and_wait() helper for correct
  SAADC sequencing; saadc_stop_and_wait() clears EVENTS_END (v3.14 — TASKS_STOP can
  generate EVENTS_END per nRF52840 PS; stale event caused immediate-exit on next call);
  LSM6DSOX status bit symmetry; recover_radio() status reset;
  forward declaration for indicate_error_fatal(); duplicate forward declaration removed;
  Phase 3 fsr_read SAADC scan layout corrected (MAXCNT=5, skip CH0 battery sample);
  Phase 3 fsr_read SAADC timeout error paths all leave clean peripheral state;
  raw byte assembly corrected at all sensor read sites — 12 sites in sensors_read()
  (v3.11) plus sensors_read_temperature() and init_bmp581() (v3.12), all well-defined C99/C11;
  temperature RTT display sign corrected for sub-zero fractions (v3.13).
- RX firmware v1.5: end-to-end radio validated, ~1-2% RF packet loss benchtop. **Exhaustive
  code review not yet performed — queue for next session (same process as TX).**
- USB framing: confirmed working end-to-end. 0 resyncs, decoder queue drops = 0
- C++ decoder v1.6: auto-detects COM14 via SERIALCOMM registry fallback, decodes to correct physical values
- Full data pipeline confirmed: TX → radio → RX → USB → decoder → console output
- LIS3MDL confirmed running at 155Hz (FAST_ODR, CTRL_REG1=0xFE)
- All sensors confirmed at startup: LSM6=0x6A LIS3=0x1C H3LIS=0x18 BMP=0x46 (CHIP_ID=0x50)
- Decoder drain-loop fix confirmed: queue_drops=0 in steady state

### Immediate next tasks
1. **Exhaustive code review of RX firmware (same process as TX — two full passes, senior
   engineer scrutiny, produce clean files + deploy + git scripts)**
2. Implement OSC output in decoder (stub already in place)
3. Connect Pure Data patch, ASIO output
4. Success criterion: move ball, hear sensor data drive audio in real time

### Known open issue — TX freeze after ~10-15 minutes
Observed in earlier sessions: TX RTT goes silent, requires manual restart. Most recent
extended run (2+ hours) completed without freezing. Status uncertain — may be resolved,
or may be intermittent. Genuine TWI driver hang (nrf_drv_twi blocking on Errata 89/121)
remains the most likely cause if it recurs. Watchdog (NVIC_SystemReset()) still planned
before Phase 2 extended testing as belt-and-suspenders.
Diagnostic: when freeze occurs, note whether TX RTT is also silent (confirms TX-side cause).

### RF loss pattern observed
Benchtop loss settles in one of two stable states: ~1-2% or <0.5%. Transitions are abrupt
at session start and tend to stay in whichever state they land. Isolated single-packet losses,
not bursts. Consistent with WiFi channel 6 (2437MHz) interference on RF channel 40 (2440MHz).
Mitigation: test channels 20 and 80 if loss is consistently above 1%.

---

## HARDWARE

### TX Ball
- Adafruit Feather nRF52840
- LSM6DSOX (accel + gyro, I2C 0x6A)
- LIS3MDL (magnetometer, I2C 0x1C)
- H3LIS331 (high-G accel, I2C 0x18)
- BMP581 (pressure, I2C 0x46)
- 100nF X7R ceramic decoupling cap on BMP581 VDD — required for reliable NVM load at power-on
- 4× FSR (Phase 3, not yet wired)

### RX Station
- Adafruit Feather nRF52840
- USB to PC only — no sensors
- Enumerates as COM14 on current PC (generic "USB Serial Device" — VID auto-detect
  does not work; always pass COM14 explicitly to decoder)

### Wiring (confirmed correct)
- I2C: SCL=P0.11, SDA=P0.12, pullups present, bus ~12cm
- H3LIS331: CS→3.3V (I2C mode), SDO→GND (fixes address at 0x18)
- BMP581 (Adafruit breakout): SDO→GND = address 0x46. CS left unconnected for I2C.
  Note: unlabelled pin between SCL/SDA on the breakout is SDO
- Battery monitor: P0.31 (AIN7, SAADC channel 0, 12-bit)
- LED: P1.15
- FSR0-3 (Phase 3): P0.03/04/05/28 (AIN1-4, SAADC channels 1-4)

---

## FIRMWARE ARCHITECTURE

### Data flow
```
Ball sensors (250Hz)
  → I2C reads in main loop
  → 86-byte packet built (3 redundant samples)
  → 2.4GHz radio TX (2Mbps GFSK, channel 40)
  → RX Feather radio ISR
  → memcpy to local buffer (IRQs off, ~1us)
  → USB CDC → PC (89-byte framed packets)
      frame: [0xAA][0x55][86-byte radio_packet_t][XOR checksum]
  → C++ decoder → OSC/UDP → Pure Data → ASIO audio
                → RTT (decoded sensor values, 1Hz)
```

### USB framing detail
Each USB frame is 89 bytes:
```
[0]    0xAA  sync byte 0
[1]    0x55  sync byte 1
[2-87] radio_packet_t (86 bytes, on-air layout, no repacking)
[88]   XOR checksum of bytes [2-87]
```
The sync header locates boundaries. The checksum validates extraction —
any false sync alignment produces a checksum mismatch and is discarded.
The entire 89 bytes must be sent in a single `app_usbd_cdc_acm_write()` call
to avoid partial-frame corruption. This requires CDC ACM TX buffer ≥ 89 bytes
(set APP_USBD_CDC_ACM_DATA_EPIN_BUFF_SIZE = 256 in sdk_config.h).

DTR must be asserted by the decoder after opening the COM port to trigger
APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN on the RX. Without DTR, usb_serial_ready()
stays false and no bytes are sent. SerialReader.cpp calls EscapeCommFunction(SETDTR).

### Protocol: OSC primary, MIDI for discrete events only
- MIDI at 250Hz × full sensor suite saturates 31250 baud for even 1 ball
- OSC over UDP loopback: no bandwidth constraint, 32-bit float native
- MIDI retained only for: impact note triggers, FSR contact transitions

---

## PACKET SPECIFICATION (v1.0)

Defined in `packet_spec.h` — identical copy in tx/ and rx/. The decoder has
a separate C++ copy in decoder/MidiJugglingDecoder/ with static_assert instead
of _Static_assert and scaling helpers added. If the on-air format changes,
all three copies must be updated.

### Data packet: 86 bytes at 250Hz (on-air)
```
ball_id    uint8_t   Ball 1-8
sequence   uint16_t  Wraps at 65535
timestamp  uint16_t  RTC ticks since TX boot, wraps at ~65535 ticks (~66.0s)
data_t0    27 bytes  Current sample
data_t1    27 bytes  t-4ms
data_t2    27 bytes  t-8ms
```

Note on timestamp field: the field is named "milliseconds" for convenience but
carries raw RTC ticks. RTC1 runs at 32768/33 = 992.97 Hz, so each tick is
~1.007ms and the counter wraps at ~66.0s (not 65.5s). For Phase 2 t1/t2
gap-fill interpolation, treat the field as ticks, not true milliseconds.

### sensor_data_t: 27 bytes
```
h3lis_x_fsr_level    int16_t  [15:4]=H3LIS X, [3:0]=FSR intensity 0-15
h3lis_y_fsr_pattern  int16_t  [15:4]=H3LIS Y, [3:0]=FSR contact bitmask
h3lis_z_flags        int16_t  [15:4]=H3LIS Z, [3:0]=reserved
accel[3]             int16_t  LSM6DSOX +/-16g raw
gyro[3]              int16_t  LSM6DSOX +/-500dps raw
mag[3]               int16_t  LIS3MDL +/-4 gauss raw
pressure[3]          uint8_t  BMP581 24-bit LE, Pa = raw/64
```

### Data conversion
```c
// Accel:    raw / 32768.0 * 16.0 = g
// Gyro:     raw / 32768.0 * 500.0 = dps
// Mag:      raw / 6842.0 = gauss (+/-4 gauss range)
// Pressure: uint32 = p[0] | (p[1]<<8) | (p[2]<<16); pa = uint32 / 64
// Temp (status packet only): raw / 65536.0 = C; stored as int16 in 0.01C units
// H3LIS:    extract_h3lis_axis(packed) / 2048.0 * 400.0 = g

// H3LIS decode — use extract_h3lis_axis() from packet_spec.h.
// Do NOT use arithmetic right shift (implementation-defined in C/C++).
// The helper uses unsigned shift + explicit sign extension from bit 11.
uint8_t fsr_intensity = h3lis_x_fsr_level & 0x0F;
```

### Status packet: 26 bytes, RTT only, every 2 minutes
```
ball_id, sequence, timestamp, packet_type(0xFF)
temperature (int16, 0.01C), battery_mv (uint16), battery_pct (uint8), sensor_health (uint8)
total_packets_sent (uint32), uptime_seconds (uint32)
radio_timeouts (uint16), i2c_errors (uint16), reserved (uint8), checksum (uint8 XOR)
```

Temperature field: SENSORS_TEMP_UNAVAILABLE (-32768) means BMP581 absent.
Firmware stores 0 in status packet when BMP581 is unavailable to avoid displaying
-327.68°C. Decoders must use the SENSOR_BMP_OK bit in sensor_health to distinguish
"0 degrees C" from "BMP absent" — the temperature field alone is ambiguous.
Status packets are RTT-only and are never forwarded over USB.

---

## RADIO CONFIGURATION (must match TX and RX)
```
Mode:        2Mbps GFSK (Nrf_2Mbit)
Channel:     40 (2440 MHz)
TX Power:    +8 dBm
Base addr:   0x12345678
Prefix:      0xAB
CRC:         24-bit, poly=0x00065B, init=0x555555
Payload:     86 bytes fixed (STATLEN=86, no length field)
Shortcuts:   READY→START, END→DISABLE, DISABLED→RXEN (RX auto-loop)
```

---

## SENSOR CONFIGURATION

### LSM6DSOX
- CTRL1_XL = 0x66 (LSM6_CTRL1_XL_VAL): 416Hz ODR, +/-16g
- CTRL2_G  = 0x64 (LSM6_CTRL2_G_VAL): 416Hz ODR, +/-500dps
  - **0x68 = +/-1000dps — do NOT use. Was a silent bug for multiple sessions.**
- CTRL3_C  = 0x44 (LSM6_CTRL3_C_VAL): BDU enabled, IF_INC enabled
- Auto-increment on I2C burst reads (no special flag needed)

### LIS3MDL
- CTRL_REG1 = 0xFE (LIS3_CTRL_REG1_VAL): ultra-high perf XY, 155Hz (FAST_ODR=1), temp enabled
  - Changed from 0xFC (80Hz). FAST_ODR is bit 1; setting it with OM=11 (UHP) enables 155Hz.
  - **0xFC = 80Hz — do NOT use.**
- CTRL_REG2 = 0x00: +/-4 gauss
- CTRL_REG3 = 0x00: continuous conversion
- CTRL_REG4 = 0x0C: ultra-high perf Z, little-endian
- **Requires | 0x80 on register address for multi-byte I2C burst reads**
- Effective output rate: 155Hz. At 250Hz TX polling, ~every 1.6 packets has fresh mag data
- Duplicate mag readings in packet stream are normal and harmless

### H3LIS331
- CTRL_REG1 = 0x37 (H3LIS_CTRL_REG1_VAL): normal mode, 400Hz, all axes
- CTRL_REG4 = 0xB0 (H3LIS_CTRL_REG4_VAL): BDU enabled, +/-400g
- **Requires | 0x80 on register address for multi-byte I2C burst reads**
- **No register readback verification in sensors_init()** (unlike radio_init and BMP581).
  A silent CTRL_REG4 write failure sets a wrong range without detection.
  Low risk on stable hardware; consider adding readback checks before Phase 2 extended testing.

### BMP581
- On-chip compensation — outputs pre-compensated data directly
- Pa = raw_register_value / 64 (no polynomial, no NVM reads needed)
- **OSR_CONFIG = 0x52 (BMP_OSR_CONFIG_VAL)**: pressure x4, temp x4 oversampling
  - Bit 6 (PRESS_EN) MUST be 1. Without it pressure registers return 0x7F7F7F.
  - **0x12 (PRESS_EN=0) was the original value — confirmed broken on hardware. Do NOT use.**
- ODR_CONFIG = 0x11 (BMP_ODR_CONFIG_VAL): normal mode ~218Hz
- Temp: raw / 65536 = C; firmware stores as int16 in 0.01C units
- NVM error flag (STATUS bit 1) is observed on known-good Adafruit units.
  It does not prevent correct operation — do not treat as fatal.
- Decoupling cap (100nF) required on VDD for reliable NVM load at power-on.
- Register readback verified in init_bmp581() after writing OSR_CONFIG and ODR_CONFIG.

### SAADC
- Channel 0: battery (P0.31 AIN7, 12-bit, owned by main.c)
- Channels 1-4: FSR0-3 (configured but disabled until Phase 3 wiring)
- **FSR reads must save/restore 12-bit resolution — FSR calibration is 10-bit**
- FSR_CONTACT_THRESHOLD = 80, intensity >> 6 scaling: calibrated for 10-bit ADC
- **Phase 3 SAADC scan layout:** CH[0] (battery) remains configured during FSR reads.
  SAADC scans CH[0..4] in order; fsr_read() uses MAXCNT=5 and discards adc_values[0]
  (battery). FSR0..FSR3 are in adc_values[1..4]. Do NOT change MAXCNT to 4 — that
  drops FSR3 and maps the battery sample into FSR0's slot.

### runtime_sensor_status behavior
SENSOR_LSM6_OK is cleared on any single I2C read failure during sensors_read()
(gyro read or accel read), even if the other succeeded. A transient I2C blip
therefore appears as "sensor failed" in the status packet. This is known behavior,
not a bug. Status packets include i2c_errors for correlation.

---

## PC DECODER

### Location
```
C:\Projects\MIDI-Juggling-Balls\decoder\MidiJugglingDecoder\
  MidiJugglingDecoder.sln
  MidiJugglingDecoder.vcxproj
  main.cpp
  SerialReader.h
  SerialReader.cpp
  packet_spec.h        <- C++ version, differs from embedded: static_assert,
                          scaling helpers (accel_to_g, gyro_to_dps etc.)
```

### Running the decoder
```cmd
cd C:\Projects\MIDI-Juggling-Balls\decoder\MidiJugglingDecoder\Debug
MidiJugglingDecoder.exe
```
Auto-detect now works via SERIALCOMM registry fallback: if VID 0x239A is not found (Feather
enumerates as generic USB Serial on this machine), it falls back to the only active COM port
(COM14). Prints a warning when using the fallback. If multiple COM ports are active, it lists
them and exits — pass the port explicitly in that case: `MidiJugglingDecoder.exe COM14`.

### Decoder architecture
- `SerialReader`: background thread, ReadFile loop, 256-byte read chunks
- Accumulation buffer with sync-header + checksum framing
- Finds 0xAA 0x55 sync, extracts 86-byte payload, verifies XOR checksum
- Checksum mismatch = false sync from payload data = discard + resync
- ball_id sanity check (1-8) as belt-and-suspenders after checksum
- `TryGetPacket()`: non-blocking, consumer thread safe
- `main.cpp`: sequence gap validation (gap ≥ 500 = false sync, discard)
- Stats every 5 seconds: bytes, packets, resyncs, per-ball loss, false_syncs
- OSC output: stubbed, implement OscSender after framing confirmed clean

### OSC address space (Phase 1.5, stub implemented)
```
/ball/N/accel    [x y z] g       (+/-16g)
/ball/N/gyro     [x y z] dps     (+/-500dps)
/ball/N/mag      [x y z] gauss   (+/-4 gauss, ~31Hz effective)
/ball/N/h3lis    [x y z] g       (+/-400g — not +/-16g scale)
/ball/N/pressure float Pa
/ball/N/fsr      intensity(int 0-15)  pattern(int bitmask)
```

MIDI (discrete events, Phase 2):
- Ball N → MIDI Channel N (via loopMIDI)
- Impact (H3LIS threshold) → Note On/Off
- FSR contact transitions → CC

---

## TX RTT VALIDATION CHECKLIST (no RX needed)

After flashing TX, connect J-Link and open RTT terminal. Expected startup:
```
=== Juggling Ball TX (Ball 1) v3.13 ===
Packet sizes:
  radio_packet_t: 86 (expect 86)      <- must match
  sensor_data_t:  27 (expect 27)      <- must match
  status_packet_t:26 (expect 26)      <- must match
LSM6DSOX at 0x6A
LIS3MDL at 0x1C
H3LIS331 at 0x18
BMP581 at 0x46 (CHIP_ID=0x50)
  OSR_CONFIG readback=0x52 (expect 0x52)
=== BMP581 OK ===
Battery: XXXX mV (USB-only mode)
Radio OK
Transmitting at 250Hz (4ms). Status every 120s.
```

Every second in the loop:
```
ACCEL:  ~0   ~2048   ~0     <- one axis ~2048 (1g at +/-16g), others near 0
GYRO:   ~0   ~0      ~0     <- near zero at rest
MAG:    stable non-zero values, change with rotation
H3LIS:  small values (-5 to +10), spike on impact
BMP:    ~101325 Pa (102000-103000 Pa typical indoors Wellington NZ)
FSR:    lvl=0 pat=0x0
TX[0..9]: OK             <- first 10 transmissions printed
DEBUG: loop=250 ... diff=0   <- diff must be 0 by loop 250
```

---

## RX RTT VALIDATION CHECKLIST

After flashing RX, power on TX. Both must be running simultaneously.
Note: TX does not need J-Link connected to transmit — USB power is sufficient.
Disconnecting J-Link from TX triggers a reset; wait for TX to reinitialise.

Expected RX output:
```
=== Juggling Ball RX v1.5 ===
  radio_packet_t: 86 (expect 86)
  sensor_data_t:  27 (expect 27)
  USB frame:      89 bytes (2 sync + 86 payload + 1 checksum)
Channel: 40 (2440 MHz)
Radio state: 0x03 (expect 0x03 = RX)
Waiting for packets...

*** Ball 1: first packet seq=XXXX ***   <- appears when TX powered on

--- RX Stats (every second) ---
Radio: N OK  0 CRC-fail  N END events
Ball 1: RX=N  Lost=0 (0.00%)  Seq=XXXX
  ACCEL:   ~0  ~2048   ~0
  GYRO:    ~0    ~0    ~0
  MAG:   stable non-zero
  H3LIS: small values
  BMP:   ~101325 Pa
  FSR:   lvl=0 pat=0x0
```

Acceptable benchtop results:
- Packet loss <1% after first ~5 seconds of RF settling
- CRC errors <0.2%
- Early loss burst on first connection is normal

---

## BUGS FIXED (all sessions)

| Bug | File | Impact |
|-----|------|--------|
| LIS3MDL burst read missing \| 0x80 | sensors.c | All mag data was X-low byte x6 |
| CTRL2_G = 0x68 (+/-1000dps) | sensors.c | Wrong gyro range, wrong RX scaling |
| LSM6DSOX BDU not enabled | sensors.c | Split-sample reads possible at ~1% rate |
| runtime_sensor_status = 0x0F | sensors.c | Falsely reported all sensors OK before detection |
| init_bmp581 status uninitialised | sensors.c | UB if I2C failed before first read |
| BMP581 STATUS nvm_rdy bit inverted | sensors.c | "ready" printed as "BUSY" |
| **BMP581 OSR_CONFIG = 0x12 (PRESS_EN=0)** | sensors.c | **Pressure registers stuck at 0x7F7F7F. Fix: 0x52** |
| BMP pressure display not /64 | main.c (TX) | Showed ~6.5M instead of ~101325 |
| next_tx_time wrong type (uint32_t) | main.c (TX) | Timing arithmetic implementation-defined |
| Negative ADC cast unguarded | main.c (TX) | Wrap-around battery voltage on noise |
| LED toggled at 125Hz | main.c (TX) | Appeared constantly on |
| status_packet_t size wrong (22→26) | main.c (TX) | Wrong struct size in comment |
| TX rate 125Hz → 250Hz | main.c (TX) | Halved sensor avg-wait latency |
| Timing resync only on positive diff | main.c (TX) | Loop ran at ~2x rate after I2C stall |
| sensors_read_temperature returns 0 on fail | sensors.c | 0C ambiguous as error sentinel |
| H3LIS decode uses signed right shift | packet_processor.c | Implementation-defined in C |
| radio_init returns frequency readback only | radio_rx.c | Meaningless check |
| RX packet size 98 vs TX 86 | radio_rx.h | Would never receive any packets |
| RX sensors.h had old sensor_data_t | radio_rx.h | Struct mismatch between sides |
| Packet contract split across files | both sides | Could silently drift — unified in packet_spec.h |
| __attribute__((packed)) mixed with #pragma pack | packet_spec.h | Inconsistent; MSVC incompatible |
| RX HFCLK via direct register + nrf_drv_clock | main.c (RX) | Bypassed driver reference count |
| usb_serial.h claimed hot-plug support | usb_serial.h | Header contradicted implementation |
| Frequency display "2%03d" format | main.c (RX) | Printed 22440 MHz instead of 2440 MHz |
| radio_get_stats() non-atomic volatile reads | radio_rx.c | Could observe partially updated stats |
| Packet loss % overflow for long sessions | packet_processor.c | uint32 overflow after ~57min at 1% loss |
| Gap heuristic threshold 1000 undocumented | packet_processor.c | Reduced to 500, documented |
| radio_rx.h 125Hz comment | radio_rx.h | Stale — corrected to 250Hz/4ms |
| usb_serial_send_packet() three separate writes | usb_serial.c | Partial frame on TX buffer busy — replaced with single atomic 89-byte write |
| radio_init power cycle delay 10us (TX) | main.c (TX) | Too short for peripheral bus stabilisation; caused STATE readback failures and recovery cascade |
| transmit_packet missing __DMB() before TASKS_TXEN (TX) | main.c (TX) | Cortex-M4 write buffer could hold pending stores; DMA might read stale packet data |
| recover_radio fixed 1ms sleep (TX) | main.c (TX) | Insufficient wait for DISABLED; caused always-fail recovery loop |
| indicate_error_radio() in recover_radio (TX) | main.c (TX) | 3s blocking per call cascaded into indefinite loop of blockages |
| **usb_serial_send_framed_packet frame on stack (RX)** | usb_serial.c | **Root cause of 99% decoder loss. DMA reads freed stack memory for last 25 bytes (second USB bulk packet). Fix: static frame buffer + TX_DONE guard** |
| USB framing constants defined in 4 places | main.c (RX), usb_serial.c, SerialReader.cpp | Could silently drift on change. Fix: USB_SYNC_BYTE_0/1 and USB_FRAME_SIZE centralised in packet_spec.h |
| usb_serial_send_text() missing s_tx_busy guard | usb_serial.c | Could corrupt ongoing DMA transfer if called while framed packet in flight |
| SerialReader::Close() races on ReadFile HANDLE | SerialReader.cpp | CloseHandle called while ReadFile may still hold it. Fix: CancelSynchronousIo before CloseHandle |
| SerialReader::ProcessBytes() O(n) erase per resync byte | SerialReader.cpp | Quadratic in sustained resync storm. Fix: read_pos index, single erase at end |
| SerialReader packet queue unbounded | SerialReader.cpp | Unbounded growth under backlog. Fix: MAX_QUEUE_DEPTH=500, drop oldest on overflow |
| RX timing.c LFCLK via direct register writes | timing.c | Bypassed nrf_drv_clock reference counting, contradicting RX design contract. Fix: nrf_drv_clock_lfclk_request() |
| LIS3MDL ODR 80Hz (CTRL_REG1=0xFC) | sensors.c | Sensor refreshed only every ~3rd packet at 250Hz TX rate. Fix: 0xFE (FAST_ODR=1) = 155Hz |
| APP_USBD_CDC_ACM_DATA_EPIN_BUFF_SIZE = 64 (RX) | sdk_config.h | SDK default 64 < 89-byte frame; changed to 256 (necessary but not sufficient — stack bug was the primary cause) |
| AutoDetectPort GUID_DEVCLASS_PORTS only (PC) | SerialReader.cpp | Failed when Feather enumerates as generic USB Serial; added DIGCF_ALLCLASSES pass + SERIALCOMM registry fallback |
| usb_serial_send_framed_packet return value discarded (RX) | main.c (RX) | USB TX drops invisible; added usb_tx_drop_count with RTT output |
| usb_serial.c CDC ACM buffer size uninstrumented (RX) | usb_serial.c | No runtime confirmation of compiled value; added RTT print (since removed once confirmed) |
| USB CDC TX buffer too small for 89-byte frame | sdk_config.h | **FIXED** — APP_USBD_CDC_ACM_DATA_EPIN_BUFF_SIZE = 256 |
| SEGGER_RTT_CONFIG_DEFAULT_MODE undefined in sdk_config.h (TX) | sdk_config.h (TX) | SEGGER_RTT_Conf.h used symbol with no fallback; compiler resolved to 0 by luck. Fix: full SEGGER_RTT_CONFIG section added to sdk_config.h |
| read_battery_voltage() bare polling loops (TX) | main.c (TX) | No timeout guards; SAADC hang would block indefinitely. Fix: 100000-iteration countdown, return 0 on timeout |
| Status interval check used 32-bit divide at 250Hz (TX) | main.c (TX) | get_uptime_seconds() divided COUNTER/1000 every loop. Fix: STATUS_INTERVAL_TICKS direct RTC comparison |
| H3LIS331 burst-read fallback unbounded (TX) | sensors.c (TX) | On burst failure, 6 single-byte reads added ~1.5ms unpredictable latency at tightest timing point. Fix: suppress fallback after 3 consecutive failures, write zeros instead |
| clear_bus_init = false in TWI config (TX) | sensors.c (TX) | SDA stuck low after abnormal reset would cause sensors_init() to fail on next boot. Fix: true — driver clocks bus free at init |
| h3lis_consecutive_failures uint8_t wrap (TX) | sensors.c (TX) | Counter wraps to 0 at 255 (~1s at 250Hz), re-enabling fallback reads and defeating suppression. Fix: cap increment at SUPPRESS_AFTER+1 |
| sensors_read_temperature sign extension UB (TX) | sensors.c (TX) | (int32_t)0xFF000000 exceeds INT_MAX; implementation-defined in C99/C11. Fix: ~(int32_t)0x00FFFFFF (well-defined, identical result) |
| memset(&h3lis_x_fsr_level, 0, 6) fragile (TX) | sensors.c (TX) | Relies on three H3LIS fields being contiguous at exact offset; silent breakage if struct changes. Fix: explicit per-field assignment |
| fsr_read() C block comment guard (TX) | sensors.c (TX) | Partial uncomment could activate code without removing early return. Fix: #if 0 / #endif with explicit removal instruction |
| Sensor config register values inline magic bytes (TX) | sensors.c (TX) | Range/ODR changes require byte reconstruction; previous 0x68/0x64 confusion caused multi-session bug. Fix: named constants (LSM6_CTRL2_G_VAL etc.) |
| get_uptime_seconds() divide by 1000 (TX) | main.c (TX) | RTC1 at 993Hz not 1000Hz; uptime read 0.71% fast (~12.8s/30min). Fix: divide by RTC1_TICKS_PER_SEC=993 |
| timing_init() delay uncommented (TX) | main.c (TX) | 10ms delay purpose unclear; could be deleted by mistake. Fix: comment added explaining counter advance before first get_timestamp_ms() call |
| BMP_ODR_CONFIG_VAL comment said PRESS_EN=1 (TX) | sensors.c (TX) | PRESS_EN is in OSR_CONFIG (0x36), not ODR_CONFIG (0x37). Wrong label on wrong register; dangerous given PRESS_EN debugging history. Fix: corrected comment |
| init_bmp581() readback printed but not checked (TX) | sensors.c (TX) | Silent OSR_CONFIG write failure would leave PRESS_EN=0; init_bmp581() still returned true. Fix: compare readback, return false on mismatch — consistent with radio_init() discipline |
| SENSOR_LSM6_OK not set on accel read success (TX) | sensors.c (TX) | Gyro success set the bit; accel success did not. Fragile to read-order changes. Fix: set SENSOR_LSM6_OK in both success paths |
| Raw byte assembly `(int16_t)(raw[0] \| (raw[1]<<8))` (TX) | sensors.c (TX) | Implementation-defined in C99/C11 when raw[1]>=128 — implicit int promotion shifts into sign bit. Fix applied in v3.11: `(int16_t)((uint16_t)raw[0] \| ((uint16_t)raw[1]<<8))` at all 12 sites (gyro/accel/mag/H3LIS). No behaviour change on GCC/Cortex-M4. |
| sensors_test() never called (TX) | main.c (TX) | Dead public API — post-init WHO_AM_I re-check existed but was never invoked. Fix: called after sensors_init() succeeds; required sensor failure is fatal |
| CRC_POLYNOMIAL comment "IBM CRC-24" wrong (packet_spec.h) | tx\, rx\, decoder\ packet_spec.h | 0x00065B is Nordic nRF proprietary CRC-24, not IBM CRC-24 (0x864CFB). Wrong label would cause independent CRC implementation to accept zero packets. Fix: corrected to "Nordic nRF proprietary radio CRC-24" |
| timestamp comment "milliseconds, 65.5s" stale (packet_spec.h) | tx\, rx\, decoder\ packet_spec.h | RTC1 runs at 992.97Hz; field carries ticks not ms; wrap is ~66.0s not 65.5s. Fix: all three copies updated with ticks annotation and Phase 2 interpolation note |
| stdbool.h / cstdbool unused includes (packet_spec.h) | tx\rx\ packet_spec.h, decoder\ packet_spec.h | bool not used in either version of the file. cstdbool deprecated in C++17 and removed in C++20. Fix: removed from both copies |
| DecodedPacket::timestamp_ms field name (decoder) | main.cpp | Field named _ms, printed as "ms"; carries RTC ticks. Fix: renamed timestamp_ticks, printf updated |
| Ctrl+C calls ExitProcess — no clean shutdown (decoder) | main.cpp | SerialReader destructor never ran; reader thread killed mid-operation; COM port not cleanly released. Fix: SetConsoleCtrlHandler sets g_running=false; main loop exits normally; destructor runs |
| resync_events counter meaning undocumented (decoder) | main.cpp | Counter increments per byte advanced past, not per dropped packet. A single misalignment increments it many times; misleading in stats. Fix: label changed to resync_events with clarifying note in output |
| RegQueryValueExA return value unchecked (decoder) | SerialReader.cpp | Failure silently left portName zero-initialised; device skipped without knowing why. Fix: check return value, continue on failure |
| radio_init PREFIX0 not in readback (TX) | main.c (TX) | PREFIX0 mismatch causes same symptom as wrong CRCPOLY — silent dead link, no error on either side. BASE0 was verified but PREFIX0 was not. Fix: added to readback verification set in radio_init() |
| fsr_read Phase 3 SAADC scan layout wrong (TX) | sensors.c (TX) | CH[0] (battery) configured by battery_init(); SAADC scans CH[0..4] in order. MAXCNT=4 captured CH[0..3]: adc_values[0] was battery (not FSR0), FSR3 (CH4) never sampled. Fix: MAXCNT=5, discard adc_values[0], read FSR0..FSR3 from adc_values[1..4] |
| fsr_read Phase 3 SAADC timeout cleanup incomplete (TX) | sensors.c (TX) | STARTED timeout returned without issuing TASKS_STOP (SAADC left running); END timeout same; STOPPED timeout returned without clearing EVENTS_STOPPED (stale event causes subsequent saadc_stop_and_wait() to return immediately). Fix: all three paths stop cleanly; EVENTS_STOPPED cleared unconditionally |
| Temperature RTT display sign lost for sub-zero fractions (TX) | main.c (TX) | `s.temperature/100` truncates toward zero; values in (−100..0) produce 0, silently dropping the minus sign. e.g. −0.50°C printed as "0.50 C". Fix: sign prefix printed separately using `s.temperature < 0 ? "-" : ""`; magnitude printed as `abs(s.temperature)/100` and `abs(s.temperature)%100`. Applies to both status log and emergency shutdown log. (v3.13) |
| prepare_packet() comment claimed sequence set before blocking work (TX) | main.c (TX) | Comment said "sequence is set before any blocking work" — false; prepare_packet() runs after sensors_read() completes. Misleading for Phase 2 gap-fill implementation. Fix: corrected to "sequence is monotonically increasing and immune to I2C timing jitter". (v3.13) |

---

## FILE LOCATIONS AND COMMIT ROUTINE

**Claude: read this section before generating any file paths or copy commands.**

### SDK working directories (SES compiles from here)

```
TX: C:\nRF5_SDK_17.1.0\examples\proprietary_rf\juggling_ball_tx\pca10056\blank\ses\
RX: C:\nRF5_SDK_17.1.0\examples\proprietary_rf\juggling_ball_rx_feather\pca10056\blank\ses\
```

IMPORTANT: TX directory is `juggling_ball_tx` — NOT `juggling_ball_tx_feather`.
The RX directory IS `juggling_ball_rx_feather`. These are different.

### Git repo structure

```
C:\Projects\MIDI-Juggling-Balls\
├── tx\       main.c  sensors.c  sensors.h  packet_spec.h  sdk_config.h
├── rx\       main.c  radio_rx.c  radio_rx.h  packet_processor.c  packet_processor.h
│             timing.c  timing.h  usb_serial.c  usb_serial.h  packet_spec.h  sdk_config.h
├── decoder\
│   ├── .gitignore   (excludes Debug/ Release/ x64/ .vs/ *.user)
│   └── MidiJugglingDecoder\
│         MidiJugglingDecoder.sln  MidiJugglingDecoder.vcxproj
│         main.cpp  SerialReader.h  SerialReader.cpp  packet_spec.h
└── docs\     project_reference.md  hardware_reference.md
```

The decoder VS2019 project lives inside the repo at:
`C:\Projects\MIDI-Juggling-Balls\decoder\MidiJugglingDecoder\`
The DECODER path variable and the repo decoder path are the same location.
Do not copy decoder files — they are already in the repo. Only copy TX and RX from SDK.

### packet_spec.h — three copies, intentionally different

```
tx\packet_spec.h              On-air contract, C, _Static_assert
rx\packet_spec.h              Must be byte-identical to tx\ copy — fc checks this
decoder\...\packet_spec.h     C++ version: static_assert, constexpr, scaling helpers added
```

The session commit routine fc-checks tx\ vs rx\ only. The decoder copy is
intentionally different (C++ syntax) and is not checked automatically.

### Full commit routine (end of each session)

```cmd
cd C:\Projects\MIDI-Juggling-Balls
set TX_SDK=C:\nRF5_SDK_17.1.0\examples\proprietary_rf\juggling_ball_tx\pca10056\blank\ses
set RX_SDK=C:\nRF5_SDK_17.1.0\examples\proprietary_rf\juggling_ball_rx_feather\pca10056\blank\ses

:: Copy TX files from SDK into repo
copy /Y %TX_SDK%\main.c        tx\
copy /Y %TX_SDK%\sensors.c     tx\
copy /Y %TX_SDK%\sensors.h     tx\
copy /Y %TX_SDK%\packet_spec.h tx\
copy /Y C:\nRF5_SDK_17.1.0\examples\proprietary_rf\juggling_ball_tx\pca10056\blank\config\sdk_config.h tx\

:: Copy RX files from SDK into repo
copy /Y %RX_SDK%\main.c              rx\
copy /Y %RX_SDK%\radio_rx.c          rx\
copy /Y %RX_SDK%\radio_rx.h          rx\
copy /Y %RX_SDK%\packet_processor.c  rx\
copy /Y %RX_SDK%\packet_processor.h  rx\
copy /Y %RX_SDK%\timing.c            rx\
copy /Y %RX_SDK%\timing.h            rx\
copy /Y %RX_SDK%\usb_serial.c        rx\
copy /Y %RX_SDK%\usb_serial.h        rx\
copy /Y %RX_SDK%\packet_spec.h       rx\
copy /Y %RX_SDK%\sdk_config.h        rx\

:: Decoder files are already in the repo — no copy needed
:: docs\project_reference.md is already in the repo — copy from Downloads if updated:
:: copy /Y "C:\Users\Martin\Downloads\project_reference.md" docs\

:: Verify packet_spec.h is identical TX vs RX
fc tx\packet_spec.h rx\packet_spec.h

:: Stage, review, commit
git add tx\ rx\ decoder\ docs\
git status
git commit -m "description"
git push
```

**sdk_config.h is version-controlled** — contains APP_USBD_CDC_ACM_DATA_EPIN_BUFF_SIZE = 256.
Must be committed with any firmware changes.

**packet_spec.h must be identical in tx\ and rx\ at all times.**

---

## PHASE PLAN

### Phase 1 (complete): Single ball, full data flow
- All sensors validated on TX RTT
- End-to-end radio validated: TX → RX → RTT, ~0.7% packet loss benchtop
- USB CDC confirmed: 89-byte framed packets to PC
- C++ decoder confirmed: clean framing, 0 resyncs, loss matches RF loss

### Phase 1.5 (in progress): C++ decoder → OSC → Pure Data
- C++ decoder receiving clean data ✓
- USB framing confirmed end-to-end ✓
- **NEXT: implement OSC output (stub in place)**
- Connect Pure Data patch, ASIO output
- Success criterion: move ball, hear sensor data drive audio in real time

### Phase 2: Sensor-to-sound mapping
- Experiment: height (BMP ~8Pa/m), spin (mag+gyro), impact (H3LIS), squeeze (FSR)
- Choose 2-3 primary mappings, implement in Pure Data
- Calibration stored in flash (layout reserved)
- MIDI discrete events: impact note triggers, FSR contact transitions
- Decision point: if packet loss >1% in performance conditions, implement t1/t2 gap-fill
  - Note: packet timestamp field carries RTC ticks (not true ms); interpolation must use
    tick units. 4ms interval = 4 ticks at 993Hz TX polling.
- **Investigate TX freeze (I2C TWI driver hang) before extended testing**
- **Consider adding register readback verification for H3LIS CTRL_REG4 and LSM6 CTRL2_G**
  before extended testing — silent write failures at wrong range would produce plausible but
  incorrect sensor data.

### Phase 3: Multi-ball (3 balls simultaneous)
- Ring buffer required in RX before running 250Hz x 3 balls (4-entry minimum)
- BLE provisioning to assign ball_id without reflashing
- Wire all 4 FSRs (P0.03/04/05/28)
- Time-slot offset: Ball1=0ms, Ball2=1.33ms, Ball3=2.67ms from TIMER0 (at 250Hz)
- Collision probability unslotted ~28% → near zero with slotting
- BLE + proprietary radio coexistence requires design decision (SoftDevice vs timesharing)
- C++ decoder already routes to /ball/N/ — no structural decoder changes needed
- **Review SerialReader::MAX_QUEUE_DEPTH at transition.** Currently 500 entries = 2s at 250Hz single-ball, ~0.67s at 750Hz three-ball. Monitor GetQueueDropCount() during initial multi-ball testing.
- **Monitor RX ISR overwrite risk.** Single rx_packet buffer is overwritten every 4ms. At 3x250Hz, the main loop has less margin between copies. Confirm usb_tx_drop_count stays near zero before moving to ring buffer phase.

### Phase 4: Performance ready
- 30+ minute stress testing
- Battery 2+ hours
- Physical durability: breadboard → perfboard
- Sound design presets, rehearsal

---

## LATENCY BUDGET

| Stage | Time |
|-------|------|
| Sensor avg wait (250Hz loop) | ~2ms |
| I2C read (all sensors) | ~1ms |
| Radio TX (86 bytes @ 2Mbps) | ~0.37ms |
| USB CDC to PC | ~1ms |
| C++ decode | ~1ms |
| Pure Data ASIO buffer | ~10-20ms (tunable) |
| **Total estimated** | **~15-25ms** |

Target is <20ms perceived. Achievable with ASIO4ALL tuned to 10ms buffer.
Further reduction possible by wiring LSM6DSOX INT1 for interrupt-driven reads (Phase 3).

---

## RISKS (updated post Phase 1.5)

| Risk | Probability | Mitigation |
|------|------------|------------|
| BMP581 pressure fails | Resolved — was PRESS_EN bug, not hardware | |
| TX I2C freeze after ~10-15min | Confirmed occurring | Investigate TWI driver hang (nRF52 Errata 89/121); add NVIC_SystemReset() watchdog |
| Packet loss >1% in performance | 15% | t1/t2 gap-fill in Phase 2; RX antenna placement |
| OSC/mrpeach setup | 20% | Test before Phase 1.5 complete; fallback MIDI patch |
| FSR unreliable | 40% | Adjust threshold, verify 10k pulldown |
| Multi-ball RF collision | 15% | Time-slotting planned; ring buffer required |
| BLE + radio coexistence (Phase 3) | High | Design decision required before Phase 3 start |
| Physical fragility | 35% | Breadboard → perfboard in Phase 3 |
| Body occlusion (performance) | 30% | Elevate RX antenna; consider diversity RX in Phase 4 |

---

## TROUBLESHOOTING

### Decoder shows 0 bytes received
1. Check RX RTT for "USB: port opened by host" — if absent, DTR assertion failed
2. Confirm COM port in Device Manager (currently COM14 — auto-detect falls back to it)
3. RX must be plugged in before boot — hot-plug not supported
4. Only one program can own the COM port — close any serial monitors first

### Decoder shows high loss with many resyncs
If resyncs are climbing and almost no packets are accepted:
1. Confirm sdk_config.h has APP_USBD_CDC_ACM_DATA_EPIN_BUFF_SIZE = 256
2. Confirm the define is actually compiled in: check RX RTT startup for "CDC ACM TX buf: 256 bytes"
   (if this line is absent, the diagnostic was removed in v1.5 — add it back temporarily)
3. Do a full Clean + Build in SES — normal rebuild does not recompile cached objects
4. Confirm usb_serial.c frame buffer is `static uint8_t s_tx_frame[89]` not a local variable
   (local variable = DMA reads freed stack for second USB bulk packet = garbage last 25 bytes)

### Decoder shows ~20% loss with "bad ball_id 170" messages
Cause: old RX firmware (v1.2 or v1.3 with three-write bug) — reflects in stream.
Fix: reflash RX with v1.5 firmware.

### TX freezes after ~10-15 minutes
RTT goes silent, radio ISR stops. Most likely nrf_drv_twi blocking hang.
Workaround: restart TX. Before restarting, note whether TX RTT is also silent
(confirms TX-side cause vs RX radio ISR failure).

### BMP581 shows 0 Pa or 130557 Pa (0x7F7F7F)
1. Check OSR_CONFIG readback in RTT — must be 0x52 (BMP_OSR_CONFIG_VAL), not 0x12
2. PRESS_EN (bit 6) must be set. 0x12 leaves it 0 — pressure never runs
3. Check SDO wire to GND (fixes address at 0x46)
4. Fit 100nF decoupling cap on VDD if not already present

### BMP581 NVM error warning at startup
Normal on Adafruit BMP581 breakouts. Confirmed on multiple units.
Does not affect pressure output if PRESS_EN is set correctly. Not actionable.

### No packets received (RX)
1. Ensure TX is powered on and has completed init (allow 2-3s after TX boot)
2. Disconnecting J-Link from TX triggers a reset — wait for reinit
3. Check RF_CHANNEL = 40 on both sides
4. Check RADIO_BASE_ADDR and RADIO_PREFIX_ADDR match exactly
5. Packet size must be 86 bytes on both sides (packet_spec.h)

### High RF packet loss (>5%)
1. Check radio timeout count in TX status packet (2-minute RTT print)
2. Try RF channels 20, 60, 80
3. Keep RX elevated and forward-facing to reduce body occlusion

### I2C errors accumulating
- Verify SCL/SDA not swapped (P0.11/P0.12)
- Verify pullups present (2.2k-4.7k to 3.3V)
- Try 100kHz I2C if errors persist at 400kHz

---

## KEY REMINDERS FOR NEW SESSIONS

### File paths — read before generating any commands
```
TX SDK:   C:\nRF5_SDK_17.1.0\examples\proprietary_rf\juggling_ball_tx\pca10056\blank\ses\
RX SDK:   C:\nRF5_SDK_17.1.0\examples\proprietary_rf\juggling_ball_rx_feather\pca10056\blank\ses\
Repo:     C:\Projects\MIDI-Juggling-Balls\
Decoder:  C:\Projects\MIDI-Juggling-Balls\decoder\MidiJugglingDecoder\
```
TX is `juggling_ball_tx`. RX is `juggling_ball_rx_feather`. They differ.
Decoder is inside the repo — files are not copied, they are edited in place.
project_reference.md lives at docs\ inside the repo.
Full commit routine is in the FILE LOCATIONS AND COMMIT ROUTINE section above.

- **TX firmware is v3.14.** Any reference to v3.13 or earlier is obsolete.
- **indicate_error_fatal() requires a forward declaration** at the top of main.c.
  It is called by timing_init() and the HFCLK startup block, both of which appear
  before its definition. Without the forward declaration, C99/C11 constraint violation.
- **LFCLK_STARTUP_TIMEOUT_MS = 1000ms.** The LFXO (32.768kHz crystal) takes 200-600ms
  to start. A timeout of 10ms or less will always fire and halt the device. HFXO is
  fast (<1ms); HFCLK_STARTUP_TIMEOUT_MS = 10ms is correct.
- **LFCLK may already be running when timing_init() is called.** nrf_drv_twi_init()
  (called inside sensors_init()) requests LFCLK via the driver. timing_init() checks
  LFCLKSTAT.STATE and skips the start sequence if the clock is already running —
  clearing EVENTS_LFCLKSTARTED and reissuing TASKS_LFCLKSTART on a running clock
  causes the event to never re-fire and the timeout to trigger.
- **git add requires files to be inside the repo working tree.** SDK path
  (C:\nRF5_SDK_17.1.0\...) is outside the repo. Always copy SDK → repo tx\ first,
  then git add tx\. See commit routine in FILE LOCATIONS AND COMMIT ROUTINE.
- **get_timestamp_ms() is now get_rtc_ticks().** Any reference to get_timestamp_ms()
  is obsolete. The rename was made to eliminate persistent confusion in changelogs.
- **saadc_stop_and_wait() clears both EVENTS_STOPPED and EVENTS_END.** Both the normal
  and timeout-exit paths of read_battery_voltage() use it. Do not replace it with inline
  TASKS_STOP without also waiting for EVENTS_STOPPED — the nRF52840 SAADC requires the
  stop to complete before TASKS_START can be issued again. EVENTS_END must also be cleared:
  per nRF52840 PS, TASKS_STOP can generate EVENTS_END; a stale set event would cause the
  EVENTS_END wait loop on the next call to exit immediately without a valid conversion.
- **RX exhaustive code review is pending.** Do not assume RX code quality matches TX.
  Same two-pass senior review process applies before Phase 2 extended testing.
- **TX v3.14 is the version to use.** main.c and sensors.c are in repo tx\ and SDK ses\.

- **radio_init() readback covers: MODE, FREQUENCY, PCNF1.STATLEN, BASE0, PREFIX0, CRCPOLY, CRCINIT.** PREFIX0 was added in v3.10. A mismatch on any of these produces a silent dead link.
- **Phase 3 fsr_read SAADC: MAXCNT=5, discard adc_values[0] (battery, CH0).** FSR0..FSR3 are in adc_values[1..4]. Do not revert to MAXCNT=4.
- **On-air packet size: 86 bytes. USB frame: 89 bytes (sync + payload + checksum).**
- **USB framing constants are in packet_spec.h.** USB_SYNC_BYTE_0 (0xAA), USB_SYNC_BYTE_1 (0x55), USB_FRAME_SIZE (89). Do not redefine locally in usb_serial.c, main.c (RX), or SerialReader.cpp.
- **packet_spec.h must be identical in tx\ and rx\.** Run fc to verify before committing.
- **sdk_config.h is version-controlled** and must be committed with firmware changes.
- **APP_USBD_CDC_ACM_DATA_EPIN_BUFF_SIZE = 256** in sdk_config.h. Default 64 is too small.
- **usb_serial frame buffer is static.** Do not change it back to a local variable — DMA reads it after the function returns (second USB bulk packet fires after function exit).
- **usb_serial_send_text() checks s_tx_busy.** Both send paths share the USB endpoint; overlapping writes corrupt the transfer.
- **RX LFCLK via nrf_drv_clock_lfclk_request(), not direct registers.** Direct writes bypass driver reference counting and can cause RTC1 to stop if the USB stack releases LFCLK.
- **BMP581 OSR_CONFIG must be 0x52 (BMP_OSR_CONFIG_VAL).** Bit 6 = PRESS_EN. 0x12 = pressure disabled.
- **BMP581 NVM error is normal.** Observed on all tested Adafruit units. Not a fault.
- **BMP581 needs 100nF decoupling cap on VDD.** Required for reliable power-on.
- **BMP581: pa = raw/64.** On-chip compensation. No NVM, no polynomial.
- **LIS3MDL CTRL_REG1 = 0xFE (LIS3_CTRL_REG1_VAL, 155Hz, FAST_ODR=1).** Previous value 0xFC = 80Hz. Burst read needs | 0x80. LSM6DSOX does not. H3LIS331 also needs | 0x80.
- **LSM6 CTRL2_G = 0x64 (LSM6_CTRL2_G_VAL, +/-500dps).** 0x68 = +/-1000dps. This was a silent multi-session bug. Use the named constant.
- **Gyro is +/-500dps.** Decoder and Pure Data must use this for scaling.
- **H3LIS is +/-400g.** Do not confuse with LSM6 +/-16g scale.
- **H3LIS decode: use extract_h3lis_axis().** Do not use arithmetic right shift.
- **Sensor config values are named constants in sensors.c** (LSM6_CTRL1_XL_VAL, LSM6_CTRL2_G_VAL, etc.). Use these when modifying ODR or range. Do not reconstruct bytes manually.
- **Packet timestamp field carries RTC ticks, not true milliseconds.** RTC1 runs at 993Hz; each tick = ~1.007ms. For Phase 2 gap-fill interpolation, treat as ticks. Counter wraps at ~66.0s.
- **get_uptime_seconds() divides by RTC1_TICKS_PER_SEC = 993**, not 1000. The old divide-by-1000 made uptime run 0.71% fast. If you see this constant changed back to 1000, revert it.
- **runtime_sensor_status SENSOR_LSM6_OK clears on any single I2C read failure** during sensors_read(). A transient blip appears as "sensor failed" in the status packet. This is normal — correlate with i2c_errors count.
- **H3LIS and LSM6 register init is not readback-verified** (unlike radio_init and BMP581, which now both validate readbacks). Plan to add before Phase 2 extended testing.
- **OSC is primary protocol.** MIDI only for discrete events.
- **USB init is non-fatal on RX.** RTT validation works without USB.
- **TX does not need J-Link to transmit.** USB power is sufficient after flashing.
- **Disconnecting J-Link from TX resets it.** Allow 2-3s reinit before expecting packets on RX.
- **Decoder auto-detects COM14** via SERIALCOMM fallback. No argument needed if only one COM port active.
- **Decoder requires DTR assertion.** SerialReader.cpp calls EscapeCommFunction(SETDTR).
- **USB single-write is mandatory.** Three separate writes corrupt the stream on TX buffer busy.
- **CDC ACM TX buffer must be 256 bytes.** Set in sdk_config.h, requires Clean+Build in SES.
- **SerialReader queue drops (GetQueueDropCount) should be zero** during single-ball operation. Non-zero at Phase 3 multi-ball means consumer loop cannot keep up — review MAX_QUEUE_DEPTH and Sleep(1) budget.
- **Decoder resync_events counts misaligned bytes, not dropped packets.** A single bad burst at startup can produce hundreds of resync_events with zero packet loss. Only worry if resyncs are non-zero in steady state.
- **Decoder Ctrl+C exits cleanly** via g_running flag and console control handler. SerialReader destructor runs and joins the reader thread. If you replace the main loop structure, preserve this — without it the COM port may not release until the process is killed.
- **DecodedPacket::timestamp_ticks** — field was previously named timestamp_ms. It carries RTC ticks, not milliseconds. Any OSC sender or logging code built on top must use ticks units for interpolation.
- **CRC_POLYNOMIAL = 0x00065B is Nordic nRF proprietary CRC-24**, not IBM CRC-24 (0x864CFB). If implementing an independent CRC checker (e.g. Python analysis script), use 0x00065B with init 0x555555. Searching "IBM CRC-24" gives the wrong polynomial.
- **SDK not in git.** nRF5 SDK v17.1.0, download separately from Nordic.
- **SES .emProject not in git.** Contains absolute paths, machine-specific.
- **VS2019 .sln and .vcxproj ARE in git.** These are safe to commit (no absolute paths).
