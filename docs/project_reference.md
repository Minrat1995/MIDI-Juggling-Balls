# MIDI Juggling Balls — Project Reference

**Version:** TX v3.18+fixes+batt-log+bmp-addr+batt-vdiv-fix2+wdt-ctx+sdk-assert+tx-rate-199+i2c-250k+twi-recovery2 / RX v1.11
**Goal:** Wireless juggling ball → sensor data → sound reinforcing visual performance
**Repo:** https://github.com/Minrat1995/MIDI-Juggling-Balls (private)
**Target:** <20ms perceived latency, <1% packet loss, scalable to 3+ balls

---

## CURRENT STATUS

### What is working

- Perfboard bench assembly complete: STEMMA QT chain assembled, all four sensors confirmed
  in RTT banner. GND resistance 1.2Ω end-to-end on STEMMA chain.
- **Direct GND wire added: BMP581 GND pad → Feather GND pin (star topology). This is the
  fix that resolved standalone dropout. 1+ hour confirmed stable standalone run achieved.**
- All four sensors at correct addresses: LSM6DSOX 0x6A, LIS3MDL 0x1C, H3LIS331 0x18, BMP581 0x47.
- Battery reads correctly: 4159–4161mV confirmed on a charged cell (AIN5 fix — see below).
- Non-blocking TWI with per-transaction timeout in sensors.c. TWI0_USE_EASY_DMA=1 confirmed.
- TASKS_STOP + EVENTS_STOPPED poll before uninit in twi_wait().
- twi-recovery2: POWER=0/1 deep lockup recovery, NVIC disable/clear around uninit, SDA
  pre-flight check at sensors_read() entry. All flashed and running.
- I2C clock reduced from 400kHz to 250kHz (i2c-250k). TX rate reduced to ~199Hz
  (TX_INTERVAL_TICKS=5, 5.035ms) to stay within USB Full Speed ceiling at 3 balls.
- app_error_fault_handler() non-weak override: writes 0xBB to GPREGRET, fault id to GPREGRET2.
  SREQ banner path correctly decodes 0xBB and prints SDK fault id.
- WDT context diagnostic: g_wdt_context updated at key loop/recovery points; WDT_IRQHandler
  writes value to GPREGRET ~122us before reset; next boot banner decodes location of stall.
- WDT HALT=Run, SLEEP=Run. Feeds at 8 sites in main.c, feeds in sensors.c twi_wait() and
  init_bmp581(). WDT interrupt enabled (INTENSET.TIMEOUT, priority 7).
- HardFault_Handler: writes GPREGRET=0xAA, NVIC_SystemReset().
- GPREGRET/GPREGRET2 failure encoding covers: HardFault (0xAA), SDK assert (0xBB), init
  failures (0x01–0x05 in GPREGRET2), and WDT context (0x01–0x14 in GPREGRET).
- DMA-safe s_rx_buf in sensors.c. RTT non-blocking confirmed. RTC1 ticking (diff=0 confirmed).
- indicate_error_fatal(): LED solid on + WDT feed loop indefinitely.
- RX firmware v1.11: comprehensive multi-pass code review applied. Full pipeline confirmed
  working. Key improvements: main loop reordered so usb_serial_process() runs before send
  (was causing every-other-packet drops via s_tx_busy not cleared in time); decode deferred
  from hot path to 1Hz stats display; CLOCK_CONFIG_LF_SRC = 1 added to sdk_config.h (was
  defaulting to RC oscillator, making all timing constants wrong); RTT buffer increased from
  512 to 2048 bytes (sufficient for 8 balls); NVIC_EnableIRQ moved to after readback
  verification in radio_init(); USB Full Speed ceiling documented.
- Full data pipeline confirmed: TX → radio → RX → USB → decoder → console output.

### TX dropout: RESOLVED

Root cause confirmed and fixed. See dropout history below for full record.

#### Dropout history (closed — issue resolved)

**Previous sessions:** dropouts presented as RESET REASON: power-on (brownout) due to
high-resistance breadboard GND. Perfboard fixed the brownout — GND resistance dropped
from 3–7Ω to 1.2Ω.

**Perfboard sessions — progression of observed failure modes:**

1. **RESET REASON: WDT, no failure signature.** WDT firing in running main loop.
   twi_wait() timeout path called nrf_drv_twi_uninit() which blocked indefinitely with
   DMA in-flight. Fix: TASKS_STOP + EVENTS_STOPPED poll before uninit.

2. **RESET REASON: soft reset, no failure signature.** SDK's weak app_error_fault_handler()
   called NVIC_SystemReset() with no GPREGRET write. I2C noise → EVENTS_ERROR → TWIM state
   machine → NRFX_ASSERT() → silent reset. Fix: non-weak override writes 0xBB/fault-id.

3. **J-Link confirmed to prevent dropout** (1+ hour clean with J-Link, seconds–minutes
   without). J-Link's SWD GND provides a low-impedance return path, reducing GND bounce
   during I2C transactions. Root cause confirmed as GND impedance.

4. **RESET REASON: WDT, context not captured.** WDT interrupt did not fire, indicating
   PRIMASK was set at time of stall — stall occurred inside an SDK critical section with
   global interrupts disabled. Software alone cannot prevent this.

5. **Hardware fix applied: direct GND wire, BMP581 GND pad → Feather GND pin (star
   topology, 28–30 AWG stranded).** Result: 1+ hour confirmed stable standalone run.
   Issue closed.

**Root cause summary:** STEMMA QT chain GND path had ~1.2Ω end-to-end resistance. Transient
I2C currents caused GND bounce that settled within 2.5μs at 400kHz but was marginal. The
direct GND wire reduced the return path impedance to ~0.1Ω, eliminating the bounce.
Software mitigations (250kHz I2C, TASKS_STOP, NVIC disable/clear, POWER=0/1, SDA
pre-flight) remain in place as belt-and-braces for the PCB design.

#### Hardware GND topology (current, floating prototype)

- STEMMA QT chain: Feather → LSM6DSOX → LIS3MDL → H3LIS331 → BMP581 (star 3V3/GND via chain)
- Direct GND wire: BMP581 GND pad → Feather GND pin (28–30 AWG stranded, ~2–5cm)
- Assembly: floating components, bubble wrap between layers, no perfboard mount yet
- LIS3MDL kept away from LiPo to minimise DC magnetic bias on mag readings
- Note: relative battery movement during throws may cause low-frequency mag noise;
  acceptable for prototype (mag is lowest-priority sensor for sound mapping)

#### Next hardware milestone

Star GND topology: add individual direct GND wires from LSM6DSOX and LIS3MDL to Feather
GND if any dropout recurs. Not required based on current stable run — hold in reserve.
The custom PCB (Phase 4) will implement full star GND pour, making individual wires
unnecessary.

---

## THIS SESSION'S CHANGES (for commit)

### TX firmware — main.c

**SDK assert override (sdk-assert):**
- app_error_fault_handler() non-weak override added. SDK weak default calls
  NVIC_SystemReset() with no GPREGRET write, producing soft reset with no signature.
  Override writes 0xBB to GPREGRET and fault id low byte to GPREGRET2, then resets.
- app_error.h included in include block (required for prototype).
- Boot banner SREQ path checks for 0xBB and reports SDK fault id.
- GPREGRET2 init-stage decode guarded against gpr==0xBB to prevent false decode.
- Clean-boot check (`gpr==0 && gpr2==0`) documented against 0xBB+gpr2=0 edge case.

**TX rate reduction (tx-rate-199):**
- TX_INTERVAL_TICKS changed 4 → 5. At 250Hz × 3 balls × 89 bytes = 66,750 bytes/sec,
  which exceeds the USB Full Speed bulk endpoint ceiling (64,000 bytes/sec). At ~199Hz
  (5 ticks, 5.035ms): 3 balls × 89 bytes = 53,133 bytes/sec = 83% of ceiling.
- Sensor ODR vs TX rate ratios updated in file header.
- WDT margin comment updated (~100 loop iterations at ~199Hz).
- sensor_history comment updated: t-5ms, t-10ms.
- RTT banner updated: "~199Hz (5ms)".
- Version string updated in both @version header and RTT printf banner.

**Code review fixes (previous session):**
- runtime_sensor_status reset to 0x00 at sensors_init() entry (sensors.c). Previously
  stale status bits from a failed first attempt could persist into Fix C retry.

**Code review fixes (this session — senior review pass):**
- HFCLK_STARTUP_TIMEOUT_MS: 10ms → 100ms (main.c). nRF52840 HFXO can take up to 6ms;
  10ms left <4ms margin. Same bug was previously fixed to 100ms on RX.
- emergency_shutdown(): bounded EVENTS_DISABLED poll (1ms) added between TASKS_DISABLE
  and SYSTEMOFF (main.c). Prevents entering SYSTEMOFF with radio DMA potentially active.
- sensors_read() entry guard: lis3_addr == 0 added alongside lsm6_addr == 0 (sensors.c).
  LIS3MDL is required; guard now returns zeros rather than calling i2c_read_regs(0,...).
- sdk_config.h: CLOCK_CONFIG_LF_SRC = 1 added. TX timing_init() overrides the SDK clock
  module's LFCLK selection, so this was not causing failures; added for correctness and
  to remove the dependency on timing_init() being reached before any LFCLK-sensitive code.
- project_reference.md PACKET SPECIFICATION table: data_t1/data_t2 corrected from
  t-4ms/t-8ms to t-5ms/t-10ms (missed when packet_spec.h was updated).

### Hardware

**Direct GND wire added:**
- 28–30 AWG stranded wire, BMP581 GND pad → Feather GND pin.
- Provides parallel low-impedance GND return path (~0.1Ω) alongside STEMMA chain (~1.2Ω).
- Result: first confirmed 1+ hour stable standalone run. Dropout issue resolved.

### packet_spec.h

- File header rate updated: 250Hz (4ms) → ~199Hz (5.035ms intervals).
- data_t1/data_t2 descriptions updated: t-4ms/t-8ms → t-5ms/t-10ms (three locations).
- Must be copied to both tx\ and rx\ in repo (shared contract).

---

## OUTSTANDING ITEMS

### 1. Commit current firmware and run full commit routine

Files to commit:
- tx\main.c    — sdk-assert + tx-rate-199 + code-review fixes (HFCLK timeout, emergency_shutdown DISABLE wait)
- tx\sensors.c — twi-recovery2 + i2c-250k + wdt-ctx markers + runtime_sensor_status reset + code-review fix (lis3_addr guard)
- tx\sdk_config.h — CLOCK_CONFIG_LF_SRC = 1 added
- tx\packet_spec.h — t1/t2 timing updated for ~199Hz
- rx\packet_spec.h — must match tx\packet_spec.h exactly (fc check in commit routine)

Expected banner after flash:
```
=== Juggling Ball TX (Ball 1) v3.18+fixes+batt-log+bmp-addr+batt-vdiv-fix2+wdt-ctx+sdk-assert+tx-rate-199 ===
Transmitting at ~199Hz (5ms). Status every 120s.
```

### 2. Revert TX power 0dBm → +8dBm

Was reduced as a diagnostic step during dropout investigation. Dropout resolved — restore
TX_POWER to RADIO_TXPOWER_TXPOWER_Pos8dBm in main.c before Phase 1.5 testing.

### 3. Proceed to Phase 1.5: OSC output and Pure Data connection

Hardware stable. Pipeline confirmed. Next milestone: OSC output from PC decoder into
Pure Data for initial sensor-to-sound mapping experiments.

---

## PHASE PLAN

### Phase 1 (complete)
- All sensors validated on TX RTT
- End-to-end radio validated: TX → RX → RTT, <0.5% packet loss benchtop
- USB CDC confirmed: 89-byte framed packets to PC
- C++ decoder confirmed: clean framing, 0 resyncs

### Phase 1.5 (current): OSC output and Pure Data connection
- Non-blocking TWI, DMA safety, WDT feeds, RTC fix, diagnostics all in place ✓
- Battery reading confirmed correct ✓
- Perfboard assembly complete ✓
- Software mitigations for I2C noise applied ✓
- Direct GND wire added (BMP581 → Feather GND) ✓
- **TX dropout resolved: 1+ hour confirmed stable standalone run ✓**
- TX rate reduced to ~199Hz (TX_INTERVAL_TICKS=5): USB ceiling resolved for 3 balls ✓
- **CURRENT ACTION: commit firmware, revert TX power to +8dBm, begin OSC output**

### Phase 2: Sensor-to-sound mapping
- Experiment: height (BMP ~8Pa/m), spin (mag+gyro), impact (H3LIS), squeeze (FSR)
- MIDI discrete events: impact note triggers, FSR contact transitions
- Decision point: if packet loss >1% in performance, implement t1/t2 gap-fill
- **Ring buffer required before adding a second ball.** At 2 balls × ~199Hz the inter-packet
  interval is ~5ms. USB DMA time is ~2ms; there is margin, but a ring buffer is still
  required for robust multi-ball handling.
- **USB throughput ceiling:** resolved for 3 balls at ~199Hz (53,133 bytes/sec = 83% of
  USB Full Speed ceiling). Ring buffer and revised framing still required for Phase 2.

### Phase 3: Multi-ball (3 balls simultaneous)
- BLE provisioning to assign ball_id
- Wire all 4 FSRs
- Resolved USB throughput ceiling from Phase 2

### Phase 4: Performance ready
- 30+ minute stress testing
- Battery 2+ hours
- Custom PCB from JLCPCB/PCBWay (~USD$5–15 for 5 boards) once perfboard design stable.
  Eliminates inter-board wiring entirely, allows form factor designed around ball interior.

---

## LATENCY BUDGET

| Stage | Time |
|-------|------|
| Sensor avg wait (~199Hz loop) | ~2.5ms |
| I2C read (all sensors, 250kHz) | ~2.1ms |
| Radio TX (86 bytes @ 2Mbps) | ~0.37ms |
| USB CDC to PC | ~1ms |
| C++ decode | ~1ms |
| Pure Data ASIO buffer | ~10–20ms (tunable) |
| **Total estimated** | **~16–26ms** |

Note: I2C read time increased from ~1ms to ~2.1ms following 400kHz → 250kHz reduction.
Total latency budget unchanged within spec.

---

## BUGS FIXED (all sessions — condensed)

| Bug | File | Impact |
|-----|------|--------|
| nrf_drv_twi blocking mode on hang (TX) | sensors.c | handler=NULL = blocking; TWI hang stalls forever. Fix: non-blocking + twi_wait() timeout |
| TWI0_USE_EASY_DMA absent (TX) | sdk_config.h | Legacy blocking backend used regardless of handler. Fix: TWI_ENABLED=1, TWI0_USE_EASY_DMA=1, TWIM_ENABLED=1 |
| WDT feed missing in twi_wait() timeout path (TX) | sensors.c | Recovery work exceeds 500ms; WDT fires during known recovery window. Fix: feeds before and after uninit |
| TASKS_STOP missing before uninit in twi_wait() (TX) | sensors.c | uninit() may block indefinitely if DMA in-flight. Fix: TASKS_STOP + EVENTS_STOPPED poll before uninit |
| POWER=0/1 missing for deep lockup in twi_wait() (TX) | sensors.c | TASKS_STOP insufficient for Errata 89/121 deep lockup. Fix: POWER cycle when EVENTS_STOPPED times out |
| NVIC not disabled around uninit in twi_wait() (TX) | sensors.c | Stale TWIM IRQ in UNINITIALIZED state → NRFX_ASSERT → soft reset. Fix: NVIC disable/clear |
| SDA pre-flight check absent (TX) | sensors.c | Stuck bus on sensors_read() entry causes immediate EVENTS_ERROR. Fix: SDA pin check + recovery before first transaction |
| DMA-unsafe RX buffer in i2c_read_regs() (TX) | sensors.c | DMA writes into freed stack on timeout, causing HardFault. Fix: s_rx_buf static buffer + memcpy |
| RTT buffer blocking without J-Link (TX) | main.c | SEGGER_RTT_printf blocks when buffer full; WDT fires. Fix: runtime SetFlagsUpBuffer(NO_BLOCK_SKIP) |
| RTC COUNTER stuck at 0 (TX) | main.c | LFCLK released by SDK after TWI init; RTC gets no clock. Fix: explicit LFCLK stop/restart in timing_init() |
| WDT fires during LFCLK startup wait (TX) | main.c | LFXO startup 200-600ms; no WDT feed in loop. Fix: WDT feed on every iteration |
| WDT feed missing after 500ms startup delay (TX) | main.c | Delay consumes entire feed #1 window; WDT fires in sensors_init(). Fix: feed #1b added after delay |
| No HardFault handler (TX) | main.c | HardFault = permanent CPU hang, no recovery. Fix: HardFault_Handler writes GPREGRET=0xAA, resets |
| No failure diagnostics across resets (TX) | main.c | Could not distinguish WDT/HardFault/brownout. Fix: GPREGRET/GPREGRET2 encode failure site |
| No WDT stall location diagnostic (TX) | main.c | WDT reset gives no information on where stall occurred. Fix: g_wdt_context + WDT_IRQHandler |
| SDK assert produces clean soft reset (TX) | main.c | NRFX_ASSERT fires app_error_fault_handler (weak), which resets with no signature. Fix: non-weak override writes 0xBB |
| Battery pin wrong: AIN7 (AREF) instead of AIN5 (TX) | main.c | P0.31 is AREF, not available for ADC. All previous battery reads were floating. Fix: AIN5 (P0.29) |
| Spurious VBAT_VDIV_ENABLE_PIN on P0.14 (TX) | main.c | Feather has no FET on battery divider. P0.14 GPIO drive had no effect. Fix: removed entirely |
| I2C at 400kHz too fast for GND noise margin (TX) | sensors.c | GND bounce at 1.2Ω doesn't settle within 2.5μs bit period. Fix: 250kHz (4μs bit period) |
| WDT HALT=Pause (TX) | main.c | Behaviour differs between J-Link and standalone. Fix: HALT=Run for consistent behaviour |
| indicate_error_fatal() blink too brief to observe (TX) | main.c | 4-second blink burst missed. Fix: LED solid on + WDT feed loop indefinitely |
| Timing resync RTT flood (TX) | main.c | Broken RTC caused ~10 RTT prints/sec; flooded buffer. Fix: print removed |
| radio_recoveries inflated by pre-session watchdog fires (RX) | radio_rx.c | Counter useless as diagnostic. Fix: s_session_active flag |
| usb_serial_process() called after send — TX_DONE not cleared (RX) | main.c | Every other packet dropped via s_tx_busy guard. Fix: process() moved to top of loop |
| CLOCK_CONFIG_LF_SRC absent from sdk_config.h (RX) | sdk_config.h | SDK defaulted to RC oscillator (±500ppm); all timing constants assumed crystal. Fix: LF_SRC = 1 |
| NVIC_EnableIRQ before readback in radio_init() (RX) | radio_rx.c | Readback failure left ISR armed on misconfigured peripheral. Fix: NVIC_EnableIRQ gated on success |
| radio_recover() called radio_start_rx() after failed radio_init() (RX) | radio_rx.c | Started hardware without armed ISR; packets received silently. Fix: early return on init failure |
| usb_serial_process() called if USB init failed (RX) | usb_serial.c | Called into uninitialised USB stack state. Fix: s_usb_init_ok guard |
| SEGGER_RTT_CONFIG_DEFAULT_MODE absent (RX) | sdk_config.h | RTT defaulted to blocking mode; full buffer caused main loop stall and radio packet loss. Fix: mode=0 added |
| RTT buffer 512 bytes — overflows at 2+ balls (RX) | sdk_config.h | Stats print silently dropped at 2+ balls. Fix: 2048 bytes |
| HFCLK startup timeout 10ms too tight (RX) | main.c | nRF52840 HFXO can take up to 6ms; <4ms margin caused false fatal halts. Fix: 100ms |
| send_text() zero-length locks s_tx_busy permanently (RX) | usb_serial.c | NRF_SUCCESS returned with no DMA; TX_DONE never fires. Fix: early return on len==0 |
| BMP581 address wrong: 0x46 instead of 0x47 (TX) | sensors.c | Sensor not found at init. Fix: 0x47 is factory default |
| External pull-up resistors (hardware design) | wiring | Unnecessary — onboard 10kΩ on each breakout = ~3.3kΩ effective |
| Bulk decoupling cap undersized (hardware) | wiring | 10µF replaced with 330µF 25V electrolytic at Feather 3.3V/GND |
| Breadboard ground wiring (hardware) | wiring | 3–7Ω contact resistance causes brownout resets. Fix: perfboard build (complete) |
| GND impedance on STEMMA QT chain (hardware) | wiring | 1.2Ω chain GND causes I2C noise → TWIM faults → crashes. Fix: direct BMP581→Feather GND wire (~0.1Ω parallel path). 1+ hour stable run confirmed |
| TX rate 250Hz exceeds USB FS ceiling at 3 balls (TX) | main.c | 250Hz × 3 × 89 = 66,750 bytes/sec > 64,000 ceiling. Fix: TX_INTERVAL_TICKS=5 (~199Hz); 3 balls = 83% of ceiling |
| runtime_sensor_status not reset at sensors_init() entry (TX) | sensors.c | Stale status bits from failed first attempt persist into Fix C retry. Fix: reset to 0x00 at init entry |
| HFCLK startup timeout 10ms too tight (TX) | main.c | Same as RX bug (fixed earlier). nRF52840 HFXO can take up to 6ms; 10ms left <4ms margin, risking GPREGRET2=0x04 false halt. Fix: 100ms |
| emergency_shutdown() enters SYSTEMOFF without waiting for radio DISABLE (TX) | main.c | Radio could be mid-TX with DMA active when SYSTEMOFF is reached. Fix: bounded 1ms EVENTS_DISABLED poll before SYSTEMOFF |
| sensors_read() entry guard missing lis3_addr == 0 check (TX) | sensors.c | If lis3_addr were 0 while twi_initialized=true, i2c_read_regs(0,...) would run and NACK on address 0. Fix: lis3_addr == 0 added to guard |
| CLOCK_CONFIG_LF_SRC absent from TX sdk_config.h (TX) | sdk_config.h | TX timing_init() overrides SDK LFCLK selection, so not causing failures. Added for correctness and to remove dependency on timing_init() override. Fix: CLOCK_CONFIG_LF_SRC = 1 |

---

## GPREGRET DIAGNOSTIC ENCODING

### GPREGRET (register 0)
| Value | Meaning |
|-------|---------|
| 0x00 | No failure — clean boot or power-cycle |
| 0xAA | HardFault on previous boot (NVIC_SystemReset from HardFault_Handler) |
| 0xBB | SDK assert/error (app_error_fault_handler override) — see GPREGRET2 for fault id |
| 0x01 | WDT stall: top of main loop, about to sensors_read() |
| 0x02 | WDT stall: inside sensors_read() / TWI operations |
| 0x03 | WDT stall: transmit_packet() or recover_radio() |
| 0x04 | WDT stall: log_status_rtt() |
| 0x05 | WDT stall: timing delay nrf_delay_ms() |
| 0x10 | WDT stall: twi_wait() timeout detected, entering recovery |
| 0x11 | WDT stall: twi_wait() TASKS_STOP poll |
| 0x12 | WDT stall: twi_wait() nrf_drv_twi_uninit() |
| 0x13 | WDT stall: twi_wait() i2c_bus_recover() |
| 0x14 | WDT stall: twi_wait() nrf_drv_twi_init() reinit |

### GPREGRET2 (register 1)
| Value | Meaning |
|-------|---------|
| 0x00 | No init failure |
| 0x01 | sensors_init() first attempt failed |
| 0x02 | sensors_init() retry (Fix C) failed |
| 0x03 | radio_init() failed |
| 0x04 | HFCLK startup timed out |
| 0x05 | LFCLK startup timed out |
| 0x01 (when GPREGRET=0xBB) | NRF_FAULT_ID_SDK_ASSERT — ASSERT() in SDK driver |
| 0x02 (when GPREGRET=0xBB) | NRF_FAULT_ID_SDK_ERROR — APP_ERROR_CHECK() failure |

Both registers survive WDT and soft resets. Cleared by power-on reset (brownout).
If both read 0x00 after a dropout, it was a brownout — no software code ran.

---

## TX RTT VALIDATION CHECKLIST (no RX needed)

After flashing TX, connect J-Link and open RTT terminal. Expected startup:
```
RESET REASON: power-on
Previous boot: no failure signature (power cycle or clean)
=== Juggling Ball TX (Ball 1) v3.18+fixes+batt-log+bmp-addr+batt-vdiv-fix2+wdt-ctx+sdk-assert+tx-rate-199 ===
Packet sizes:
  radio_packet_t: 86 (expect 86)
  sensor_data_t:  27 (expect 27)
  status_packet_t:26 (expect 26)
I2C bus recovery: SDA was OK (no hang)
LSM6DSOX at 0x6A
LIS3MDL at 0x1C
H3LIS331 at 0x18
BMP581 at 0x47 (CHIP_ID=0x50)
  OSR_CONFIG readback=0x52 (expect 0x52)
  WARNING: no data ready (STATUS=0x01)   <- normal on this unit
  [1] raw=0x6XXXXX = ~102000 Pa
=== BMP581 OK ===
LFCLK+RTC1: PRESCALER=32 COUNTER=19 OK  <- COUNTER must be non-zero
Battery: 4XXX mV (XX%)                  <- must show realistic mV, NOT "USB-only" when LiPo connected
Radio OK
WDT started (timeout 500ms)
Transmitting at ~199Hz (5ms). Status every 120s.
DEBUG: loop=250 T=1257 next=1257 diff=0  <- diff must be 0 by loop 250
```

After WDT reset (if occurs), expected additional lines:
```
RESET REASON: WDT (watchdog) -- loop stall detected
WDT context: 0xXX -- <location string>   <- key diagnostic output
```

After SDK assert reset:
```
RESET REASON: soft reset -- SDK assert/error
SDK fault: id=0x01 -- NRF_FAULT_ID_SDK_ASSERT (ASSERT() in SDK driver)
```

---

## RX RTT VALIDATION CHECKLIST

Expected RX output with TX transmitting:
```
=== Juggling Ball RX v1.11 ===
...
*** Ball 1: first packet seq=XXXX ***

--- RX Stats (every second) ---
Radio: N OK  0 CRC-fail  N END events  0 ISR-overwrites  0 recoveries

Ball 1: RX=N  Lost=0 (0.00%)  Seq=XXXX  0.0s ago
  ACCEL / GYRO / MAG / H3LIS / BMP as expected
  FSR:   lvl=0 pat=0x0
```

radio_recoveries should be 0 during an active session. Non-zero = peripheral genuinely hung mid-session.

USB TX drops should be 0 in steady state at 1 ball × 250Hz. Non-zero drops indicate the main loop
is taking >4ms per iteration. Primary cause: RTT output stalling (confirm SEGGER_RTT_CONFIG_DEFAULT_MODE = 0
in sdk_config.h and that SEGGER_RTT_Conf.h uses #ifndef guards so the value is not overridden).

---

## PERFBOARD BUILD COMPONENTS

### Purchased — DigiKey order (arrived)

| Item | DigiKey # | Adafruit # | Notes |
|------|-----------|------------|-------|
| Adafruit FeatherWing Proto | 1528-1622-ND | 2884 | Wiring termination layer, stacks on Feather |
| Feather Stacking Female Headers | — | 2886 | 12+16 pin set — required for 2884 to stack |
| Bakelite Perfboard ×10 | 1528-2171-ND | 2670 | Scissors-cuttable sensor carriers |
| M2.5 Nylon Standoff/Screw Kit | 1528-2339-ND | 3299 | Includes 5mm, 6mm, 8mm, 10mm, 12mm |
| STEMMA QT 50mm cables ×5 | 1528-4399-ND | 4399 | JST SH 4-pin, prototype phase wiring |
| Klein 11057 wire stripper | 1742-1270-ND | — | 22-32AWG |
| Kapton tape 10mm | 1188-KAPTON-TAPE10MM-ND | — | Strain relief and battery retention |
| ESD wrist strap | DKS-ESD-WRISTSTRAP-ND | — | Handle Feather and sensor boards safely |

### To purchase locally

| Item | Supplier | Product | Notes |
|------|----------|---------|-------|
| 30AWG silicone wire, 5 colours | Jaycar NZ | WH3026 | Black, red, green, yellow, white — 10m each |
| Conformal coating | RS Components NZ | Electrolube AFA200 | Acrylic, aromatic-free, reworkable with IPA |
| Double-sided foam tape | Hardware store | 3M 4008 or 4013 | ~1mm thick, battery retention |
| Small steel ruler | Hardware store/stationery | — | For measuring/scoring Bakelite |

### Already owned

| Item | Notes |
|------|-------|
| Feather nRF52840 | |
| 4517 LSM6DSOX + LIS3MDL (9DoF) | One physical board, two I2C devices |
| H3LIS331 (Adafruit 4627) | |
| BMP581 (Adafruit 5716) | |
| LiPo 603450, 3.7V 1100mAh | Inspect for swelling. Charge fully before use. |
| 330µF 25V electrolytic | Already on perfboard |
| Adafruit 1608 Perma-Proto | Fallback sensor carrier if Bakelite approach fails |
| Bubble wrap | Ball prototype casing |
| Lead-free solder | |
| Flux pen | |

---

## HARDWARE

### TX Ball

**Processor/radio:** Adafruit Feather nRF52840

**Sensor boards (3 physical boards, 4 I2C devices)**
- Adafruit 4517: LSM6DSOX (accel + gyro, I2C 0x6A) + LIS3MDL (magnetometer, I2C 0x1C)
- Adafruit 4627: H3LIS331 (high-G accel, I2C 0x18)
- Adafruit 5716: BMP581 (pressure, I2C 0x47)

**Power**
- LiPo battery: 3.7V 1100mAh, JST connector. MCP73831 charges via USB.
  Inspect for swelling before use. Confirm JST polarity before connecting.
- 330µF 25V electrolytic at Feather 3.3V/GND. Polarity confirmed.

**Battery monitoring**
- SAADC channel 0: AIN5 (P0.29), hardwired 150K/150K divider (always connected, no FET)
- Conversion: adc_counts × 1758 / 1000 = millivolts
- USB-only threshold: 1000mV. Full charge: ~4200mV.
- Shutdown threshold: 3300mV (triggers emergency_shutdown)
- Battery reads ~559mV when disconnected (floating pin — normal, below USB threshold)

**Capacitors**
- 330µF 25V electrolytic: one, at Feather 3.3V/GND near JST connector
- 100nF per sensor: onboard on all Adafruit breakouts — no separate parts required

**Other:** 4× FSR (Phase 3, not yet wired). TX power currently 0dBm (**revert to +8dBm before Phase 1.5 testing**).

### Wiring — perfboard (prototype phase: STEMMA QT)

**I2C:** SCL=P0.11, SDA=P0.12
**STEMMA QT chain:** Feather (soldered cut cable) → 4517 → H3LIS331 → BMP581
**I2C pull-ups:** none external required. Three boards onboard = ~3.3kΩ effective.
**Address configuration:** all sensors use factory default addresses, no config pins needed.

**Wire colour scheme (perfboard final build — Jaycar WH3026 30AWG):**
- Red: 3.3V | Black: GND | Yellow: SCL | Green: SDA | White: misc/address/FSR
- Note: green=SDA (not blue — Jaycar WH3026 has no blue). Different from breadboard scheme.

**Power topology:** dedicated wire from 2884 3V3 strip to each sensor VCC;
dedicated wire from 2884 GND strip to each sensor GND.

**LED:** P1.15 (~1Hz toggle = normal; solid on = init failed; off = brownout loop)
**FSR0–3 (Phase 3):** P0.03/04/05/28 (AIN1–4, SAADC channels 1–4)
**SWD/J-Link post-assembly:** bubblewrap is removable for reprogramming.

### RX Station
- Adafruit Feather nRF52840, USB to PC only
- COM14 on current PC (pass explicitly to decoder — VID auto-detect unreliable)

### Stack architecture

```
Layer 1 (top):    Feather nRF52840 + 2884 FeatherWing Proto (stacked via 2886 headers)
                  ↕ 10mm M2.5 nylon standoffs (spanning battery)
                  Battery: LiPo 603450 (foam-taped to Feather underside)
                  ↕ M2.5 standoffs (continuation)
Layer 2 (middle): 4517 9DoF + H3LIS331 (on Bakelite carrier cut from 2670)
                  ↕ 5mm M2.5 nylon standoffs
Layer 3 (bottom): BMP581 (on Bakelite carrier cut from 2670)

Bounding box: 50.8 × 34.0 × ~42mm
```

---

## FIRMWARE ARCHITECTURE

### Data flow
```
Ball sensors (250Hz)
  → I2C reads in main loop (250kHz, ~2.1ms total)
  → 86-byte packet built (3 redundant samples)
  → 2.4GHz radio TX (2Mbps GFSK, channel 40)
  → RX Feather radio ISR
  → usb_serial_process() [drain TX_DONE from previous packet, clear s_tx_busy]
  → memcpy to local buffer (RADIO IRQ masked, ~1us)
  → usb_serial_send_framed_packet() [immediately after copy — minimum latency]
  → packet_processor_process() [sequence tracking, raw sample store]
  → USB CDC → PC (89-byte framed packets)
      frame: [0xAA][0x55][86-byte radio_packet_t][XOR checksum]
  → C++ decoder → OSC/UDP → Pure Data → ASIO audio
                → RTT (decoded sensor values, 1Hz — decoded on demand not on every packet)
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
The entire 89 bytes must be sent in a single app_usbd_cdc_acm_write() call
to avoid partial-frame corruption. This requires CDC ACM TX buffer >= 89 bytes
(set APP_USBD_CDC_ACM_DATA_EPIN_BUFF_SIZE = 256 in sdk_config.h).

DTR must be asserted by the decoder after opening the COM port to trigger
APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN on the RX. Without DTR, usb_serial_ready()
stays false and no bytes are sent. SerialReader.cpp calls EscapeCommFunction(SETDTR).

### Protocol: OSC primary, MIDI for discrete events only
- MIDI at 250Hz × full sensor suite saturates 31250 baud for even 1 ball
- OSC over UDP loopback: no bandwidth constraint, 32-bit float native
- MIDI retained only for: impact note triggers, FSR contact transitions

### Watchdog (TX)
nRF52840 WDT, 500ms timeout. SLEEP=Run, HALT=Run. Cannot be stopped once started.
Started after the 500ms LED flash in main(). WDT interrupt enabled (INTENSET.TIMEOUT,
priority 7) — WDT_IRQHandler writes g_wdt_context to GPREGRET before reset.

WDT feeds in main.c (8 sites):
  1. First instruction — covers post-WDT-reset boot
  1b. Immediately after 500ms startup delay — fresh window for sensors_init()
  2. Before 500ms LED flash — sensors_init() may have consumed window from #1b
  3. Inside LFCLK startup wait loop
  4. Fix C retry block (two 1s feeds during 2s retry)
  5. Inside indicate_error_fatal() — LED solid without WDT firing
  6. Inside emergency_shutdown() — 2000ms blink > 500ms timeout

WDT feeds in sensors.c:
  7. init_bmp581() — at Step 1/4/5 delays and Step 6 retry loop
  8a. twi_wait() timeout — before TASKS_STOP/recovery
  8b. twi_wait() timeout — after EVENTS_STOPPED poll
  8c. twi_wait() timeout — after nrf_drv_twi_uninit()

---

## PACKET SPECIFICATION (v1.0)

Defined in packet_spec.h — identical copy in tx/ and rx/. Decoder has a C++ copy.
If on-air format changes, all three copies must be updated.

### Data packet: 86 bytes at ~199Hz
```
ball_id    uint8_t   Ball 1-8
sequence   uint16_t  Wraps at 65535
timestamp  uint16_t  RTC ticks since TX boot, wraps at ~65535 ticks (~66.0s)
data_t0    27 bytes  Current sample
data_t1    27 bytes  t-5ms
data_t2    27 bytes  t-10ms
```

Timestamp: raw RTC ticks, not ms. RTC1 at 992.97 Hz → each tick ~1.007ms, wraps ~66.0s.
For Phase 2 t1/t2 gap-fill interpolation, treat the field as ticks.

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
// Temp:     raw / 65536.0 = C; stored as int16 in 0.01C units (status packet only)
// H3LIS:    extract_h3lis_axis(packed) / 2048.0 * 400.0 = g
uint8_t fsr_intensity = h3lis_x_fsr_level & 0x0F;
```

### Status packet: 26 bytes, RTT only, every 2 minutes
```
ball_id, sequence, timestamp, packet_type(0xFF)
temperature (int16, 0.01C), battery_mv (uint16), battery_pct (uint8), sensor_health (uint8)
total_packets_sent (uint32), uptime_seconds (uint32)
radio_timeouts (uint16), i2c_errors (uint16), reserved (uint8), checksum (uint8 XOR)
```

Temperature field: SENSORS_TEMP_UNAVAILABLE (-32768) means BMP581 absent; stored as 0.
Status packets are RTT-only, never forwarded over USB.

---

## RADIO CONFIGURATION (must match TX and RX)
```
Mode:        2Mbps GFSK (Nrf_2Mbit)
Channel:     40 (2440 MHz)
TX Power:    currently 0dBm (**revert to +8dBm before Phase 1.5 testing**)
Base addr:   0x12345678 | Prefix: 0xAB
CRC:         24-bit, poly=0x00065B (Nordic proprietary — NOT IBM CRC-24 0x864CFB), init=0x555555
Payload:     86 bytes fixed (STATLEN=86, no length field)
Shortcuts:   READY->START, END->DISABLE, DISABLED->RXEN (RX auto-loop)
```

---

## SENSOR CONFIGURATION

### LSM6DSOX
- CTRL1_XL=0x66: 416Hz, +/-16g, LPF2_XL_EN=1
- CTRL2_G=0x64: 416Hz, +/-500dps (**0x68 = +/-1000dps — do NOT use**)
- CTRL3_C=0x44: BDU enabled, IF_INC enabled

### LIS3MDL
- CTRL_REG1=0xFE: UHP, 155Hz FAST_ODR (**0xFC = 80Hz — do NOT use**)
- CTRL_REG2=0x00: +/-4 gauss | CTRL_REG3=0x00: continuous | CTRL_REG4=0x0C: UHP Z, LE
- **Requires | 0x80 on register address for multi-byte burst reads**

### H3LIS331
- CTRL_REG1=0x37: normal mode, 400Hz, all axes
- CTRL_REG4=0xB0: BDU enabled, +/-400g
- **Requires | 0x80 on register address for multi-byte burst reads**

### BMP581
- **OSR_CONFIG=0x52**: PRESS_EN=1, OSR_P=x4, OSR_T=x4 (**0x12=PRESS_EN=0 — causes 0x7F7F7F. Do NOT use.**)
- ODR_CONFIG=0x11: normal mode ~218Hz
- Pa = raw_register_value / 64 (on-chip compensation)
- NVM error flag normal on Adafruit units. Step 6 data-ready poll always times out — normal.

### SAADC
- Channel 0: battery (P0.29 AIN5, 12-bit, hardwired 150K/150K divider)
- Channels 1-4: FSR0-3 (configured but disabled until Phase 3)
- Phase 3 scan: MAXCNT=5, discard adc_values[0] (battery). Do NOT use MAXCNT=4.
- saadc_stop_and_wait() tight loop: 100000-iteration loop with no delay inside — runs
  ~10–15ms worst case. Harmless in Phase 2, but add a WDT feed before Phase 3 when SAADC
  is used more heavily.

---

## OSC ADDRESS SPACE (Phase 1.5, stub implemented)
```
/ball/N/accel    [x y z] g       (+/-16g)
/ball/N/gyro     [x y z] dps     (+/-500dps)
/ball/N/mag      [x y z] gauss   (+/-4 gauss)
/ball/N/h3lis    [x y z] g       (+/-400g)
/ball/N/pressure float Pa
/ball/N/fsr      intensity(int 0-15)  pattern(int bitmask)
```

---

## PC DECODER

### Location
```
C:\Projects\MIDI-Juggling-Balls\decoder\MidiJugglingDecoder\
  MidiJugglingDecoder.sln / .vcxproj / main.cpp
  SerialReader.h / SerialReader.cpp
  packet_spec.h   <- C++ version: static_assert, scaling helpers
```

### Running
```cmd
cd C:\Projects\MIDI-Juggling-Balls\decoder\MidiJugglingDecoder\Debug
MidiJugglingDecoder.exe
```
Auto-detect via SERIALCOMM registry. Falls back to COM14 if only one active port.

---

## FILE LOCATIONS AND COMMIT ROUTINE

**Claude: read this section before generating any file paths or copy commands.**

### SDK working directories (SES compiles from here)

```
TX: C:\nRF5_SDK_17.1.0\examples\proprietary_rf\juggling_ball_tx\pca10056\blank\ses\
RX: C:\nRF5_SDK_17.1.0\examples\proprietary_rf\juggling_ball_rx_feather\pca10056\blank\ses\
```

IMPORTANT: TX directory is juggling_ball_tx — NOT juggling_ball_tx_feather.
The RX directory IS juggling_ball_rx_feather. These are different.

### Git repo structure

```
C:\Projects\MIDI-Juggling-Balls\
├── tx\       main.c  sensors.c  sensors.h  packet_spec.h  sdk_config.h
├── rx\       main.c  radio_rx.c  radio_rx.h  packet_processor.c  packet_processor.h
│             timing.c  timing.h  usb_serial.c  usb_serial.h  packet_spec.h  sdk_config.h
├── decoder\
│   └── MidiJugglingDecoder\
│         MidiJugglingDecoder.sln  MidiJugglingDecoder.vcxproj
│         main.cpp  SerialReader.h  SerialReader.cpp  packet_spec.h
└── docs\     project_reference.md  hardware_reference.md
```

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

:: Copy RX files from SDK into repo (only if RX changed this session)
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

:: Verify packet_spec.h is identical TX vs RX
fc tx\packet_spec.h rx\packet_spec.h

:: Stage, review, commit
git add tx\ rx\ docs\
git status
git commit -m "<version string>: <summary of changes>"
git push
```

---

## SESSION REMINDERS

- TX is juggling_ball_tx. RX is juggling_ball_rx_feather. They differ.
- TWI0_USE_EASY_DMA=1 in TX sdk_config.h is required. Without it the event handler is
  silently ignored and all transactions block.
- packet_spec.h must be identical in tx\ and rx\ at all times.
- sdk_config.h is version-controlled — commit with firmware changes.
- APP_USBD_CDC_ACM_DATA_EPIN_BUFF_SIZE=256 in RX sdk_config.h (default 64 too small).
- LIS3MDL and H3LIS331 burst reads need | 0x80. LSM6DSOX does not.
- LSM6 CTRL2_G=0x64 (+/-500dps). 0x68=+/-1000dps. Use the named constant.
- BMP581 OSR_CONFIG must be 0x52 (PRESS_EN=1). 0x12 disables pressure.
- BMP581 NVM error is normal. Step 6 data-ready timeout is normal on this unit.
- H3LIS decode: use extract_h3lis_axis(). Do not use arithmetic right shift.
- get_rtc_ticks() returns uint16_t on TX, uint32_t on RX (different implementations).
- Battery on TX: AIN5 (P0.29), 150K/150K hardwired divider, no enable FET or GPIO.
  Reads ~559mV when disconnected (floating) — below 1000mV USB-only threshold, normal.
- I2C now at 250kHz (was 400kHz). TX rate now ~199Hz (TX_INTERVAL_TICKS=5, was 4/250Hz). If TX rate falls below ~199Hz check DEBUG diff values.
- Disconnecting J-Link from TX resets it. Allow 2-3s reinit before expecting RX packets.
- WDT feeds in main.c and sensors.c are required and must not be removed.
- WDT HALT=Run. WDT interrupt enabled (priority 7). Do not revert either.
- CRC_POLYNOMIAL=0x00065B is Nordic nRF proprietary, not IBM CRC-24 (0x864CFB).
- TX power currently 0dBm. **Revert to +8dBm before Phase 1.5 testing** (was reduced during dropout investigation).
- USB throughput ceiling: 89-byte frames. At ~199Hz: 1 ball = 17,711 bytes/sec (28%), 2 balls = 35,422 (55%), 3 balls = 53,133 (83%). Ceiling resolved for 3-ball operation. Ring buffer still required for Phase 2 multi-ball handling.
- Direct GND wire in place: BMP581 GND → Feather GND (28–30 AWG stranded). Do not remove.
- If dropout recurs: add direct GND wires from LSM6DSOX and LIS3MDL to Feather GND (full star topology). Not currently needed.
- CLOCK_CONFIG_LF_SRC = 1 in both TX and RX sdk_config.h. RX: required (SDK clock module
  controls LFCLK; without it the SDK defaults to RC oscillator, making all timing constants
  wrong). TX: added for correctness; timing_init() overrides the SDK's LFCLK selection
  unconditionally, but having the SDK start with Xtal removes the dependency on that override.
- HFCLK_STARTUP_TIMEOUT_MS = 100ms in TX main.c. nRF52840 HFXO can take up to 6ms; the
  previous value of 10ms left <4ms margin. Same bug was previously fixed on RX.
- SEGGER_RTT_CONFIG_DEFAULT_MODE = 0 in RX sdk_config.h. Verify SEGGER_RTT_Conf.h in the
  SDK uses #ifndef guards so this value is not overridden. Blocking RTT mode causes packet loss.
- SEGGER_RTT_CONFIG_BUFFER_SIZE_UP = 2048 in RX sdk_config.h (sufficient for 8 balls).
- usb_serial_process() must be called before usb_serial_send_framed_packet() in the RX main
  loop. If called after, TX_DONE is not processed before the next send attempt and every
  other packet is dropped via the s_tx_busy guard.
- DecodedPacket::timestamp_ticks carries RTC ticks, not milliseconds.
- SDK not in git. nRF5 SDK v17.1.0. SES .emProject not in git (absolute paths).
- VS2019 .sln and .vcxproj ARE in git.
- ESD wrist strap: wear when handling Feather and sensor boards.
- Conformal coating (AFA200): mask USB-C, JST, test pads. Reworkable with IPA.
- Battery: inspect for swelling. Confirm JST polarity. Charge fully before use.
- Three physical sensor boards, four I2C devices. Wire colours: R=3V3, Blk=GND,
  Yel=SCL, Grn=SDA (NOT blue — Jaycar WH3026 has no blue).
- saadc_stop_and_wait() tight loop has no WDT feed — add one before Phase 3 SAADC use.

### LED states (TX)
- LED blinking ~1Hz: normal operation
- LED on solid: indicate_error_fatal() reached — init failed after reset
- LED off: genuine power-on reset loop (brownout) — firmware never starts

### Files to upload for next TX debugging session
1. main.c (TX) — current output version
2. sensors.c (TX) — current output version
3. sensors.h (TX)
4. packet_spec.h (TX)
5. sdk_config.h (TX)
6. project_reference.md (this file)

RX files are NOT needed unless RX behaviour changes.
