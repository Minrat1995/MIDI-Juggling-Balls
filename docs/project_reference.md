# MIDI Juggling Balls — Project Reference

**Version:** Post Phase 1 hardware validation (TX v3.3, RX v1.2)
**Goal:** Wireless juggling ball → sensor data → sound reinforcing visual performance
**Repo:** https://github.com/Minrat1995/MIDI-Juggling-Balls (private)
**Target:** <20ms perceived latency, <1% packet loss, scalable to 3+ balls

---

## CURRENT STATUS

### What is done
- TX firmware v3.3: all sensors validated on hardware, 250Hz confirmed stable
- RX firmware v1.2: end-to-end radio validated, 0.33% packet loss benchtop
- Both flashed and confirmed working against the same packet contract
- Git repo at `C:\Projects\MIDI-Juggling-Balls\` with tx/ and rx/ folders
- Hardware wiring confirmed correct (see HARDWARE section)
- Decoupling capacitor (100nF X7R ceramic) fitted on BMP581 VDD

### Immediate next tasks
1. Build C++ decoder for OSC output to Pure Data
2. Connect USB CDC binary stream, confirm 86-byte packets received on PC
3. OSC → Pure Data → ASIO audio (Phase 1.5)
4. First sensor-to-sound mapping (Phase 2)

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
  → USB CDC → PC (binary, 86 bytes/packet)
  → C++ decoder → OSC/UDP → Pure Data → ASIO audio
                → RTT (decoded sensor values, 1Hz)
```

### Protocol: OSC primary, MIDI for discrete events only
- MIDI at 250Hz × full sensor suite saturates 31250 baud for even 1 ball
- OSC over UDP loopback: no bandwidth constraint, 32-bit float native
- MIDI retained only for: impact note triggers, FSR contact transitions

---

## PACKET SPECIFICATION (v1.0)

Defined in `packet_spec.h` — identical copy in tx/ and rx/. If you change one,
change the other. This is the on-air contract between TX and RX.

### Data packet: 86 bytes at 250Hz
```
ball_id    uint8_t   Ball 1-8
sequence   uint16_t  Wraps at 65535
timestamp  uint16_t  TX milliseconds, wraps at 65.5s
data_t0    27 bytes  Current sample
data_t1    27 bytes  t-4ms
data_t2    27 bytes  t-8ms
```

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

// H3LIS decode — use extract_h3lis_axis() from packet_spec.h.
// Do NOT use arithmetic right shift (implementation-defined in C).
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
Firmware stores 0 in status packet when unavailable to avoid confusing PC-side display.

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
- CTRL1_XL = 0x66: 416Hz ODR, +/-16g
- CTRL2_G  = 0x64: 416Hz ODR, +/-500dps
- CTRL3_C  = 0x44: BDU enabled, IF_INC enabled
- Auto-increment on I2C burst reads (no special flag needed)

### LIS3MDL
- CTRL_REG1 = 0xFC: ultra-high perf XY, 80Hz, temp enabled
- CTRL_REG2 = 0x00: +/-4 gauss
- CTRL_REG3 = 0x00: continuous conversion
- CTRL_REG4 = 0x0C: ultra-high perf Z, little-endian
- **Requires | 0x80 on register address for multi-byte I2C burst reads**
- Effective output rate ~31Hz at 250Hz TX polling (polled every ~8th cycle)
- Duplicate mag readings in packet stream are normal and harmless

### H3LIS331
- CTRL_REG1 = 0x37: normal mode, 400Hz, all axes
- CTRL_REG4 = 0xB0: BDU enabled, +/-400g
- **Requires | 0x80 on register address for multi-byte I2C burst reads**

### BMP581
- On-chip compensation — outputs pre-compensated data directly
- Pa = raw_register_value / 64 (no polynomial, no NVM reads needed)
- **OSR_CONFIG = 0x52**: pressure x4, temp x4 oversampling
  - Bit 6 (PRESS_EN) MUST be 1. Without it pressure registers return 0x7F7F7F.
  - 0x12 (PRESS_EN=0) was the original value — confirmed broken on hardware.
- ODR_CONFIG = 0x11: normal mode ~218Hz
- Temp: raw / 65536 = C; firmware stores as int16 in 0.01C units
- NVM error flag (STATUS bit 1) is observed on known-good Adafruit units.
  It does not prevent correct operation — do not treat as fatal.
- Decoupling cap (100nF) required on VDD for reliable NVM load at power-on.

### SAADC
- Channel 0: battery (P0.31 AIN7, 12-bit, owned by main.c)
- Channels 1-4: FSR0-3 (configured but disabled until Phase 3 wiring)
- **FSR reads must save/restore 12-bit resolution — FSR calibration is 10-bit**
- FSR_CONTACT_THRESHOLD = 80, intensity >> 6 scaling: calibrated for 10-bit ADC

---

## TX RTT VALIDATION CHECKLIST (no RX needed)

After flashing TX, connect J-Link and open RTT terminal. Expected startup:
```
=== Juggling Ball TX (Ball 1) v3.3 ===
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

Troubleshooting:
- BMP shows 0 Pa: check OSR_CONFIG readback = 0x52 (PRESS_EN bit), check SDO to GND
- BMP NVM error warning: normal on Adafruit breakouts, does not affect output
- MAG shows same value all 3 axes: | 0x80 fix not in build — rebuild
- Any sensor at 0x00: I2C error — check wiring, pullups
- TX FAIL on first 10: radio issue — check HFCLK started
- diff drifting negative: timing resync will fire, check for I2C stalls

---

## RX RTT VALIDATION CHECKLIST

After flashing RX, power on TX. Both must be running simultaneously.
Note: TX does not need J-Link connected to transmit — USB power is sufficient.
Disconnecting J-Link from TX triggers a reset; wait for TX to reinitialise.

Expected RX output:
```
=== Juggling Ball RX v1.2 ===
  radio_packet_t: 86 (expect 86)
  sensor_data_t:  27 (expect 27)
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

Acceptable benchtop results (confirmed in hardware testing):
- Packet loss <1% after first ~5 seconds of RF settling
- CRC errors <0.2%
- Early loss burst on first connection is normal (RF/AGC settling)

Troubleshooting:
- "No balls seen yet" after TX running: TX may be reinitialising — wait 5s after TX boot
- High CRC errors, zero OK: access address mismatch — verify BASE_ADDR and PREFIX match
- MAG all identical: TX still has old firmware — reflash TX
- Loss >5%: try RF channels 20, 60, 80; keep RX elevated

---

## OSC ADDRESS SPACE (Phase 1.5 onward)

C++ decoder emits OSC bundles to localhost:9000
```
/ball/N/accel/x|y|z     float, g (+/-16g range)
/ball/N/gyro/x|y|z      float, dps (+/-500dps range)
/ball/N/mag/x|y|z       float, gauss (~31Hz effective at 250Hz TX)
/ball/N/pressure        float, Pa (on-chip compensated)
/ball/N/fsr/intensity   float 0.0-1.0 (aggregate peak, not per-FSR)
/ball/N/fsr/contact     int 0 or 1
/ball/N/hg/x|y|z        float, g (+/-400g range — note: NOT +/-16g scale)
```

MIDI (discrete events only, via loopMIDI):
- Ball N → MIDI Channel N
- Impact (H3LIS threshold) → Note On/Off
- FSR contact transitions → CC

FSR packing note: only aggregate peak intensity and per-FSR contact bitmask
are available. Per-FSR intensity is not available in the current packet format.

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
| **BMP581 OSR_CONFIG = 0x12 (PRESS_EN=0)** | sensors.c | **Pressure registers stuck at 0x7F7F7F. Confirmed on two units. Fix: 0x52** |
| BMP pressure display not /64 | main.c (TX) | Showed ~6.5M instead of ~101325 |
| next_tx_time wrong type (uint32_t) | main.c (TX) | Timing arithmetic implementation-defined |
| Negative ADC cast unguarded | main.c (TX) | Wrap-around battery voltage on noise |
| LED toggled at 125Hz | main.c (TX) | Appeared constantly on |
| status_packet_t size wrong (22→26) | main.c (TX) | Wrong struct size claimed in comment |
| TX rate 125Hz → 250Hz | main.c (TX) | Halved sensor avg-wait latency |
| Timing resync only on positive diff | main.c (TX) | Loop ran at ~2x rate after I2C stall |
| sensors_read_temperature returns 0 on fail | sensors.c | 0C ambiguous as error sentinel |
| H3LIS decode uses signed right shift | packet_processor.c | Implementation-defined in C |
| radio_init returns frequency readback only | radio_rx.c, main.c (TX) | Meaningless check |
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

---

## FILE LOCATIONS

### SDK working directories (SES compiles from here)
```
TX: C:\nRF5_SDK_17.1.0\examples\proprietary_rf\juggling_ball_tx\pca10056\blank\ses\
RX: C:\nRF5_SDK_17.1.0\examples\proprietary_rf\juggling_ball_rx_feather\pca10056\blank\ses\
```

### Git repo
```
C:\Projects\MIDI-Juggling-Balls\
├── tx\     main.c  sensors.c  sensors.h  packet_spec.h  sdk_config.h
├── rx\     main.c  radio_rx.c  radio_rx.h  packet_processor.c  packet_processor.h
│           timing.c  timing.h  usb_serial.c  usb_serial.h  packet_spec.h  sdk_config.h
├── docs\   project_reference.md  hardware_reference.md
└── README.md
```

### Commit routine (end of each session)
```
cd C:\Projects\MIDI-Juggling-Balls
set TX_SDK=C:\nRF5_SDK_17.1.0\examples\proprietary_rf\juggling_ball_tx\pca10056\blank\ses
set RX_SDK=C:\nRF5_SDK_17.1.0\examples\proprietary_rf\juggling_ball_rx_feather\pca10056\blank\ses
copy /Y %TX_SDK%\main.c       tx\
copy /Y %TX_SDK%\sensors.c    tx\
copy /Y %TX_SDK%\sensors.h    tx\
copy /Y %TX_SDK%\packet_spec.h tx\
copy /Y %RX_SDK%\main.c       rx\
copy /Y %RX_SDK%\radio_rx.c   rx\
copy /Y %RX_SDK%\radio_rx.h   rx\
copy /Y %RX_SDK%\packet_processor.c rx\
copy /Y %RX_SDK%\packet_processor.h rx\
copy /Y %RX_SDK%\timing.c     rx\
copy /Y %RX_SDK%\timing.h     rx\
copy /Y %RX_SDK%\usb_serial.c rx\
copy /Y %RX_SDK%\usb_serial.h rx\
copy /Y %RX_SDK%\packet_spec.h rx\
fc tx\packet_spec.h rx\packet_spec.h
git add tx\ rx\ docs\
git commit -m "description"
git push
```

**packet_spec.h must be identical in tx\ and rx\ at all times.**
The fc command above verifies this before every commit.

---

## PHASE PLAN

### Phase 1 (complete): Single ball, full data flow
- All sensors validated on TX RTT
- End-to-end radio validated: TX → RX → RTT, 0.33% packet loss benchtop
- USB CDC confirmed: 86-byte binary packets to PC

### Phase 1.5 (current): C++ decoder → OSC → Pure Data
- Build C++ decoder (Visual Studio 2019)
- Parse 86-byte binary stream from USB CDC
- Emit OSC bundles to localhost:9000 via mrpeach
- Connect Pure Data patch, ASIO output
- Success criterion: move ball, hear sensor data drive audio in real time

### Phase 2: Sensor-to-sound mapping
- Experiment: height (BMP ~8Pa/m), spin (mag+gyro), impact (H3LIS), squeeze (FSR)
- Choose 2-3 primary mappings, implement in Pure Data
- Calibration stored in flash (layout reserved)
- MIDI discrete events: impact note triggers, FSR contact transitions
- Decision point: if packet loss >1% in performance conditions, implement t1/t2 gap-fill

### Phase 3: Multi-ball (3 balls simultaneous)
- Ring buffer required in RX before running 250Hz x 3 balls (4-entry minimum)
- BLE provisioning to assign ball_id without reflashing
- Wire all 4 FSRs (P0.03/04/05/28)
- Time-slot offset: Ball1=0ms, Ball2=1.33ms, Ball3=2.67ms from TIMER0 (at 250Hz)
- Collision probability unslotted ~28% → near zero with slotting
- BLE + proprietary radio coexistence requires design decision (SoftDevice vs timesharing)
- C++ already routes to /ball/N/ — no structural decoder changes needed

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

## RISKS (updated post Phase 1)

| Risk | Probability | Mitigation |
|------|------------|------------|
| BMP581 pressure fails | Resolved — was PRESS_EN bug, not hardware | |
| Packet loss >1% in performance | 15% | t1/t2 gap-fill in Phase 2; RX antenna placement |
| OSC/mrpeach setup | 20% | Test before Phase 1.5; fallback MIDI patch |
| FSR unreliable | 40% | Adjust threshold, verify 10k pulldown |
| Multi-ball RF collision | 15% | Time-slotting planned; ring buffer required |
| BLE + radio coexistence (Phase 3) | High | Design decision required before Phase 3 start |
| Physical fragility | 35% | Breadboard → perfboard in Phase 3 |
| Body occlusion (performance) | 30% | Elevate RX antenna; consider diversity RX in Phase 4 |

---

## TROUBLESHOOTING

### BMP581 shows 0 Pa or 130557 Pa (0x7F7F7F)
1. Check OSR_CONFIG readback in RTT — must be 0x52, not 0x12
2. PRESS_EN (bit 6) must be set. 0x12 leaves it 0 — pressure never runs
3. Check SDO wire to GND (fixes address at 0x46)
4. Fit 100nF decoupling cap on VDD if not already present

### BMP581 NVM error warning at startup
- Normal on Adafruit BMP581 breakouts. Confirmed on multiple units.
- Does not affect pressure output if PRESS_EN is set correctly.
- Not actionable — ignore.

### No packets received (RX)
1. Ensure TX is powered on and has completed init (allow 2-3s after TX boot)
2. Disconnecting J-Link from TX triggers a reset — wait for reinit
3. Check RF_CHANNEL = 40 on both sides
4. Check RADIO_BASE_ADDR and RADIO_PREFIX_ADDR match exactly
5. Packet size must be 86 bytes on both sides (packet_spec.h)

### High packet loss (>5%)
1. Check radio timeout count in TX status packet (2-minute RTT print)
2. Try RF channels 20, 60, 80
3. Keep RX elevated and forward-facing to reduce body occlusion

### Magnetometer readings all identical
- Was a firmware bug (missing | 0x80) — fixed in v3.3. Reflash TX.

### Timing resync warnings in TX RTT
- Indicates main loop stalled >100ms (e.g. slow I2C during BMP init)
- Single occurrence at startup is normal
- Repeated occurrences: check I2C error count in status packet

### Audio latency too high
1. Reduce Pure Data ASIO buffer (20ms → 10ms)
2. Reduce block size (128 → 64 samples)
3. Check C++ decoder is not blocking on serial read

### I2C errors accumulating
- Verify SCL/SDA not swapped (P0.11/P0.12)
- Verify pullups present (2.2k-4.7k to 3.3V)
- Try 100kHz I2C if errors persist at 400kHz

---

## KEY REMINDERS FOR NEW SESSIONS

- **TX rate is 250Hz (4ms).** Any reference to 125Hz/8ms is obsolete.
- **Packet size: 86 bytes.** Any reference to 98 bytes is obsolete.
- **packet_spec.h must be identical in tx/ and rx/.** Run fc to verify before committing.
- **BMP581 OSR_CONFIG must be 0x52.** Bit 6 = PRESS_EN. 0x12 = pressure disabled.
- **BMP581 NVM error is normal.** Observed on all tested Adafruit units. Not a fault.
- **BMP581 needs 100nF decoupling cap on VDD.** Required for reliable power-on.
- **BMP581: pa = raw/64.** On-chip compensation. No NVM, no polynomial.
- **LIS3MDL burst read needs | 0x80.** LSM6DSOX does not.
- **Gyro is +/-500dps.** RX decoder and Pure Data must use this for scaling.
- **H3LIS is +/-400g.** Do not confuse with LSM6 +/-16g scale in Pure Data.
- **H3LIS decode: use extract_h3lis_axis().** Do not use arithmetic right shift.
- **OSC is primary protocol.** MIDI only for discrete events.
- **USB init is non-fatal on RX.** RTT validation works without USB.
- **TX does not need J-Link to transmit.** USB power is sufficient after flashing.
- **Disconnecting J-Link from TX resets it.** Allow 2-3s reinit before expecting packets on RX.
- **SDK not in git.** nRF5 SDK v17.1.0, download separately from Nordic.
- **SES .emProject not in git.** Contains absolute paths, machine-specific.
