/**
 * Sensors Module Implementation
 *
 * Fixes applied across review and hardware validation sessions:
 *   - sensors_read(): raw byte assembly corrected at all 12 sites (gyro×3,
 *     accel×3, mag×3, H3LIS×3). Previous form (int16_t)(raw[0] | (raw[1]<<8))
 *     is implementation-defined in C99/C11 when raw[1] >= 128.
 *     Corrected form: (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1]<<8)).
 *   - LIS3MDL burst read: | 0x80 on register address (was reading X-low 6x)
 *   - CTRL2_G: 0x64 for +/-500dps (was 0x68 = +/-1000dps)
 *   - CTRL3_C: BDU + IF_INC enabled on LSM6DSOX (prevents split-sample reads)
 *   - runtime_sensor_status initialised 0x00 (was 0x0F)
 *   - init_bmp581: status/reg_val initialised to 0 (was UB if I2C failed early)
 *   - BMP581 STATUS bit 0 nvm_rdy interpretation corrected (1=ready, not busy)
 *   - BMP581 OSR_CONFIG: 0x52 not 0x12 -- bit 6 (PRESS_EN) must be set
 *   - sensors_read_temperature: returns SENSORS_TEMP_UNAVAILABLE on failure
 *   - Named constants added for all sensor register config values.
 *   - h3lis_consecutive_failures: increment capped at H3LIS_FALLBACK_SUPPRESS_AFTER+1.
 *   - sensors_read_temperature sign extension: ~(int32_t)0x00FFFFFF.
 *   - H3LIS/pressure zero-on-failure: explicit per-field assignment.
 *   - fsr_read disabled code converted to #if 0 / #endif.
 *   - fsr_read (#if 0 Phase 3 block): MAXCNT corrected to 5; timeout paths clean.
 *   - sensors_read() LSM6DSOX: local lsm6_ok flag for explicit AND semantics.
 *
 * Post-v3.18 fixes (sensors.c):
 *   - init_bmp581() Steps 4 and 5: OSR_CONFIG and ODR_CONFIG readbacks validated.
 *   - init_bmp581() Step 7 failure message generalised.
 *   - H3LIS FSR packing: unsigned domain arithmetic (C99/C11 conformance).
 *   - LSM6_CTRL1_XL_VAL comment: FS_XL[3:2] corrected to "01"; LPF2_XL_EN documented.
 *   - init_bmp581() Step 7: trailing nrf_delay_ms(50) skipped on final attempt.
 *
 * Post-v3.18 fixes (sensors.c) -- I2C bus recovery:
 *   - i2c_bus_recover() added and called at the start of sensors_init().
 *     After a WDT reset (triggered by a TWI peripheral hang per Errata 89/121),
 *     the peripheral registers are cleared but a sensor may still be holding SDA
 *     low mid-transaction. clear_bus_init=true in the TWI config clocks SCL up
 *     to 9 times but worst-case state alignment can require more. i2c_bus_recover()
 *     uninits the TWI peripheral (releasing pin control), configures SCL and SDA
 *     as open-drain GPIO, clocks SCL up to 18 times until SDA is released, sends
 *     an explicit STOP condition, then releases both pins. nrf_drv_twi_init()
 *     reconfigures them as TWI pins. Logs whether SDA was stuck, giving RTT
 *     confirmation that the hang-and-recovery path was taken.
 *     Safe on a clean boot: SDA is already high, only the GPIO configuration and
 *     STOP condition run before nrf_drv_twi_init() reconfigures the pins.
 *
 *   - init_bmp581() feeds WDT at each long delay point and inside the Step 6
 *     data-ready retry loop. After a WDT reset the watchdog cannot be stopped and
 *     resumes counting from zero immediately. BMP581 init alone can consume close
 *     to 500ms: Step 1 soft reset wait (50ms), Step 4 delay (10ms), Step 5 delay
 *     (100ms), and Step 6 data-ready poll (up to 50 x 10ms = 500ms). Without feeds
 *     at these points, the WDT fires during BMP581 init on every post-WDT-reset
 *     boot, creating an infinite reset loop.
 *     Feeds are placed only at explicit nrf_delay_ms() calls -- never around I2C
 *     operations. A genuine TWI hang (Errata 89/121), which has no nrf_delay_ms
 *     around it, is still caught by the WDT as intended.
 *     NRF_WDT->RR[0] write is accessible in sensors.c via nrf.h (already included
 *     transitively through nrf_drv_twi.h -> nrf_drv_twi_two_wire.h -> nrf.h).
 *     Writing RR[0] on a cold boot (RREN=0) is harmless per nRF52840 PS.
 *
 * Post-v3.18 fixes (sensors.c) -- non-blocking TWI with per-transaction timeout:
 *   This is the primary fix for the persistent TX stall after a TWI hang (Errata
 *   89/121). All previous fix rounds (WDT feeds, bus recovery, Fix C retry) address
 *   state left by a previous hang but cannot prevent a new one. The only escape is
 *   bounding every transaction with a timeout so the call can return false and the
 *   caller continues.
 *
 *   Root cause: nrf_drv_twi_tx/rx with handler=NULL runs in blocking mode. When
 *   the TWIM peripheral hangs, the driver spins forever inside the SDK. WDT fires,
 *   resets TX, sensors_init() retries, another transaction hangs -- unescapable loop.
 *
 *   Changes:
 *   1. k_twi_config: TWI config extracted to module level (was a local in
 *      sensors_init). Required so twi_wait()'s recovery path can call
 *      nrf_drv_twi_init() without holding a dangling pointer to a stack local.
 *   2. Non-blocking mode: nrf_drv_twi_init() now passes twi_event_handler (non-NULL).
 *      nrf_drv_twi_tx/rx return immediately after starting DMA; the event handler
 *      fires from the TWIM IRQ when each transfer completes or errors.
 *   3. twi_event_handler(): sets volatile twi_xfer_done and twi_xfer_error from TWIM
 *      IRQ context (APP_IRQ_PRIORITY_HIGH = priority 2), preempting the spin loop.
 *   4. twi_wait(): spin-waits on twi_xfer_done with nrf_delay_us(1) per iteration
 *      and TWI_TIMEOUT_US = 5000 (5ms) countdown. At 400kHz the longest expected
 *      transaction (6-byte burst) is ~285us; 5ms is ~17x margin. On timeout: feeds
 *      WDT (see item 9 below), uninits TWIM, calls i2c_bus_recover() for GPIO-level
 *      recovery, reinits with handler. Returns false regardless -- the failed
 *      transfer's data must not be used.
 *   5. i2c_write_reg(): clears twi_xfer_done/error before tx, calls twi_wait().
 *   6. i2c_read_regs(): two-phase non-blocking pattern. Phase 1 TX uses s_reg_buf
 *      (module-level static) as EasyDMA source; Phase 2 RX uses the caller's buffer.
 *      Each phase gets a separate twi_wait() call with its own 5ms timeout.
 *   7. i2c_bus_recover() no longer clears sensor addresses. This is intentional:
 *      when called from twi_wait() during sensors_read() in the main loop, not
 *      clearing addresses allows the next sensors_read() call to retry the same
 *      sensors after bus recovery. If the bus is healthy after reinit, real data
 *      resumes immediately. If repeated hangs occur, each attempt costs at most
 *      TWI_TIMEOUT_US + recovery overhead -- TX stays alive. Sensor address
 *      clearing is now done explicitly at the top of sensors_init(), where
 *      re-detection always follows immediately.
 *   8. twi_timeout_count: module-private counter, incremented on each timeout,
 *      logged to RTT, and folded into i2c_error_count for status packet visibility.
 *   9. twi_wait() timeout path: WDT fed immediately on timeout detection, before
 *      uninit/recover/reinit work. The recovery sequence (uninit + bus_recover +
 *      reinit) has unpredictable duration when the TWIM peripheral is in a deeply
 *      hung state -- the SDK driver may block internally waiting for peripheral
 *      state acknowledgment. Without a WDT feed at this point the 500ms window
 *      from the top-of-loop feed can be exhausted during recovery, firing the WDT
 *      and resetting TX. This is the same rationale as feeds in init_bmp581() and
 *      indicate_error_fatal(): feeding during a known recovery window is correct;
 *      not feeding is what causes spurious resets.
 *
 * Post-v3.18 fixes (sensors.c) -- improved TWI recovery (twi-recovery2):
 *   Three targeted improvements to the twi_wait() recovery path and
 *   sensors_read() entry:
 *
 *   1. POWER=0/1 peripheral reset on EVENTS_STOPPED timeout.
 *      When TASKS_STOP itself does not complete within TWI_TIMEOUT_US, the
 *      TWIM peripheral is in deep lockup per Errata 89/121 -- the peripheral
 *      is locked waiting for the I2C bus and TASKS_STOP cannot interrupt it.
 *      The nRF52840 PS identifies POWER=0/1 as the definitive recovery: it
 *      resets all peripheral registers including the DMA state machine.
 *      Without this, uninit() is called with the peripheral still active,
 *      which either blocks or leaves DMA writing to freed memory (HardFault).
 *      After POWER=1, uninit() finds ENABLE already cleared and returns
 *      immediately, cleaning up driver state only.
 *      NRF_TWIM_Type in SDK 17.1.0 does not expose POWER as a named field.
 *      Access via direct pointer cast to offset 0xFFC (valid per nRF52840 PS:
 *      all peripherals have POWER at 0xFFC). Cast to volatile uint32_t *.
 *
 *   2. NVIC_DisableIRQ / NVIC_ClearPendingIRQ around uninit.
 *      The TWIM IRQ (SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn) must not fire
 *      between nrf_drv_twi_uninit() and nrf_drv_twi_init(). If it does, the
 *      SDK driver sees an interrupt in UNINITIALIZED state and hits
 *      NRFX_ASSERT() -> app_error_fault_handler -> soft reset. This is the
 *      most likely mechanism behind the "RESET REASON: soft reset" dropouts
 *      observed before sdk-assert fix. NVIC_ClearPendingIRQ ensures any IRQ
 *      that queued during the timeout path is discarded before reinit.
 *      nrf_drv_twi_init() re-enables the IRQ internally.
 *
 *   3. SDA pre-flight check at sensors_read() entry.
 *      If SDA is low before any transaction begins, a sensor is already
 *      holding the bus. Every subsequent TWIM transaction will immediately
 *      produce EVENTS_ERROR, which -- before the sdk-assert fix -- caused
 *      a soft reset via NRFX_ASSERT(). With the sdk-assert fix it causes
 *      repeated i2c_error_count increments and all-zeros sensor data.
 *      Catching this before the first transaction and running bus recovery
 *      (GPIO SCL clocking + STOP + reinit) eliminates both failure modes.
 *      nrf_gpio_pin_read() on a TWI-configured pin returns the physical line
 *      state -- the SDK configures the input buffer as connected during init.
 *
 * Post-v3.18 fixes (sensors.c) -- I2C speed reduction (i2c-250k):
 *   k_twi_config frequency changed from NRF_DRV_TWI_FREQ_400K to
 *   NRF_DRV_TWI_FREQ_250K. At 400kHz (2.5us bit period), GND bounce from
 *   transient current on the sensor chain may not fully settle before the
 *   I2C sampling point, causing intermittent EVENTS_ERROR / SDK assert resets
 *   (observed as RESET REASON: soft reset with no failure signature prior to
 *   sdk-assert fix). At 250kHz (4us bit period) settling time doubles.
 *   sensors_read() I2C time increases ~1.3ms -> ~2.1ms; 250Hz TX rate
 *   maintained with ~1.3ms slack in the 4ms interval.
 *   If dropouts persist: step to NRF_DRV_TWI_FREQ_100K + TX_INTERVAL_TICKS=7.
 *
 * Post-v3.18 fixes (sensors.c) -- TASKS_STOP before uninit in twi_wait():
 *   Root cause of WDT reset in running main loop (confirmed in hardware):
 *   nrf_drv_twi_uninit() waits internally for TWIM ENABLE=0 to take effect.
 *   Per nRF52840 PS, ENABLE=0 does not take effect until any in-flight DMA
 *   transfer completes. With a sensor holding SDA low mid-transaction, DMA
 *   may never complete and uninit blocks indefinitely -- exhausting the 500ms
 *   WDT window between the pre-uninit feed and the post-uninit feed that were
 *   already in place. Symptom: RESET REASON: WDT, no GPREGRET2 init-failure
 *   signature (stall occurred in the running main loop, not during init).
 *
 *   Fix: issue TWIM TASKS_STOP before calling nrf_drv_twi_uninit(). TASKS_STOP
 *   requests the peripheral to abort the current transfer and assert
 *   EVENTS_STOPPED. EVENTS_STOPPED firing means DMA has ended; uninit then finds
 *   the peripheral already halted and returns immediately. TASKS_STOP completion
 *   does not depend on the I2C slave releasing SDA -- it is a peripheral-side
 *   operation. The EVENTS_STOPPED poll is bounded at 5ms (consistent with
 *   TWI_TIMEOUT_US). A WDT feed is added after the poll so recovery continues
 *   with a full 500ms window regardless of how long STOP took.
 *
 * Post-v3.18 fixes (sensors.c) -- HardFault / DMA pointer safety:
 *   - s_rx_buf[8] added as module-level static EasyDMA receive buffer.
 *   - i2c_read_regs() Phase 2 now DMA's into s_rx_buf, then memcpy's to the
 *     caller's buffer after twi_wait() confirms completion. Previously DMA
 *     wrote directly into the caller's stack buffer. On a twi_wait() timeout,
 *     nrf_drv_twi_uninit() is not instantaneous -- DMA could still be writing
 *     when the calling stack frame unwound, corrupting return addresses and
 *     causing a HardFault. Without a HardFault handler the CPU loops forever:
 *     no WDT (if pre-wdt_init()), no LED blink, TX permanently silent until
 *     power cycled. s_rx_buf is always valid regardless of stack state.
 *
 * Code-review fix (sensors.c):
 *   - sensors_read() entry guard: lis3_addr == 0 added alongside lsm6_addr == 0.
 *     LIS3MDL is a required sensor (sensors_init() returns false if absent), so
 *     lis3_addr == 0 in a running system means an unrecoverable init gap. The
 *     guard now returns zeros rather than calling i2c_read_regs(0, ...) which
 *     would generate a NACK transaction to address 0 on the bus.
 *
 * Changelog from 3.11 (sensors.c only):
 *   - sensors_read_temperature(): uint32_t cast applied before shift in 24-bit
 *     byte assembly.
 *   - init_bmp581() pressure validation read: same uint32_t cast fix applied.
 */

#include "sensors.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_saadc.h"
#include <string.h>

extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);

// g_wdt_context is defined in main.c. Updated here inside twi_wait() so that
// WDT_IRQHandler can capture the exact recovery step if the WDT fires during
// a TWI hang. See main.c for the full value table.
extern volatile uint8_t g_wdt_context;

// ============================================================================
// I2C CONFIGURATION
// ============================================================================

#define I2C_SCL_PIN     11      // P0.11
#define I2C_SDA_PIN     12      // P0.12

#define LSM6DSOX_ADDR_1 0x6A
#define LSM6DSOX_ADDR_2 0x6B
#define LIS3MDL_ADDR_1  0x1C
#define LIS3MDL_ADDR_2  0x1E
#define H3LIS331_ADDR   0x18
#define BMP581_ADDR_1   0x47    // factory default (SDO floating / ADDR jumper open) — try first
#define BMP581_ADDR_2   0x46    // SDO pulled to GND — fallback

// ============================================================================
// FSR CONFIGURATION
// ============================================================================

#define FSR0_ADC_PIN    NRF_SAADC_INPUT_AIN1  // P0.03
#define FSR1_ADC_PIN    NRF_SAADC_INPUT_AIN2  // P0.04
#define FSR2_ADC_PIN    NRF_SAADC_INPUT_AIN3  // P0.05
#define FSR3_ADC_PIN    NRF_SAADC_INPUT_AIN4  // P0.28

#define FSR_CONTACT_THRESHOLD   80

// ============================================================================
// LSM6DSOX REGISTERS
// ============================================================================

#define LSM6_WHO_AM_I   0x0F
#define LSM6_CTRL1_XL   0x10
#define LSM6_CTRL2_G    0x11
#define LSM6_CTRL3_C    0x12
#define LSM6_OUTX_L_G   0x22
#define LSM6_OUTX_L_A   0x28
#define LSM6DSOX_ID     0x6C

#define LSM6_CTRL1_XL_VAL   0x66    // ODR[7:4]=0110=416Hz, FS_XL[3:2]=01=+/-16g,
                                    // LPF2_XL_EN(bit1)=1 -- enables 104Hz LPF2 on
                                    // accel output via reset-default CTRL8_XL
#define LSM6_CTRL2_G_VAL    0x64    // ODR[7:4]=0110=416Hz, FS_G[3:2]=01=+/-500dps
                                    // NOTE: 0x68 = +/-1000dps -- do NOT use (was a bug)
#define LSM6_CTRL3_C_VAL    0x44    // BDU(bit6)=1, IF_INC(bit2)=1

// ============================================================================
// LIS3MDL REGISTERS
// ============================================================================

#define LIS3_WHO_AM_I   0x0F
#define LIS3_CTRL_REG1  0x20
#define LIS3_CTRL_REG2  0x21
#define LIS3_CTRL_REG3  0x22
#define LIS3_CTRL_REG4  0x23
#define LIS3_OUT_X_L    0x28
#define LIS3MDL_ID      0x3D

#define LIS3_CTRL_REG1_VAL  0xFE    // TEMP_EN=1, OM=11(UHP), DO=111, FAST_ODR=1 -> 155Hz
                                    // NOTE: 0xFC = 80Hz (FAST_ODR=0) -- do NOT use
#define LIS3_CTRL_REG2_VAL  0x00    // FS=00 -> +/-4 gauss
#define LIS3_CTRL_REG3_VAL  0x00    // continuous conversion mode
#define LIS3_CTRL_REG4_VAL  0x0C    // OMZ=11(UHP), BLE=0(LE)

// ============================================================================
// H3LIS331 REGISTERS
// ============================================================================

#define H3LIS_WHO_AM_I  0x0F
#define H3LIS_CTRL_REG1 0x20
#define H3LIS_CTRL_REG4 0x23
#define H3LIS_OUT_X_L   0x28
#define H3LIS331_ID     0x32

#define H3LIS_CTRL_REG1_VAL 0x37    // normal mode, 400Hz ODR, X/Y/Z enabled
#define H3LIS_CTRL_REG4_VAL 0xB0    // BDU(bit7)=1, FS[5:4]=11=+/-400g

// ============================================================================
// BMP581 REGISTERS
// ============================================================================

#define BMP_CHIP_ID     0x01
#define BMP_STATUS      0x28
#define BMP_PRESS_XLSB  0x20
#define BMP_TEMP_XLSB   0x1D
#define BMP_OSR_CONFIG  0x36
#define BMP_ODR_CONFIG  0x37
#define BMP_CMD         0x7E

#define BMP_CMD_SOFT_RESET  0xB6
#define BMP581_ID           0x50
#define BMP581_ID_ALT       0x51

// BMP_OSR_CONFIG_VAL = 0x52: PRESS_EN=1, OSR_P=x4, OSR_T=x4
// NOTE: 0x12 = PRESS_EN=0 -- do NOT use (causes 0x7F7F7F pressure output)
#define BMP_OSR_CONFIG_VAL  0x52

// 0x11 = MODE=NORMAL, ODR=maximum (~218Hz)
// PRESS_EN is in OSR_CONFIG (0x36), NOT this register.
#define BMP_ODR_CONFIG_VAL  0x11

// ============================================================================
// TWI TIMEOUT
//
// Per-phase timeout for each nrf_drv_twi_tx/rx call. nrf_delay_us(1) per
// iteration gives 1us resolution. At 400kHz the longest expected transaction
// is a 6-byte burst read at ~285us. 5000us = 5ms provides ~17x margin.
//
// A two-phase read (i2c_read_regs) can stall for at most ~10ms on full
// double-timeout before bus recovery and reinit execute. This is well within
// the WDT window. The main loop feeds the WDT before sensors_read(), so a
// timeout during sensors_read() does not trigger a WDT reset -- provided the
// wdt_feed() inside twi_wait()'s timeout path keeps the window fresh (see
// the non-blocking TWI changelog above, item 9).
// ============================================================================

#define TWI_TIMEOUT_US  5000U

// ============================================================================
// GLOBAL STATE
// ============================================================================

// TWI configuration at module level so twi_wait()'s inline recovery path can
// call nrf_drv_twi_init() without holding a dangling pointer to a local in
// sensors_init(). Values are identical to what sensors_init() used to declare
// locally; only the scope has changed.
//
// Frequency: 250kHz (was 400kHz). Reduced to improve I2C noise margin.
// At 400kHz each bit period is 2.5us; GND bounce from transient current on the
// sensor chain has a decay time constant of ~1-2us and may not fully settle
// before the sampling point, causing intermittent EVENTS_ERROR / SDK assert
// resets. At 250kHz the bit period doubles to 4us, giving 2x more settling
// time. sensors_read() total I2C time increases from ~1.3ms to ~2.1ms, leaving
// ~1.3ms slack within the 4ms TX interval -- 250Hz is maintained.
// If dropouts persist, step to NRF_DRV_TWI_FREQ_100K with TX_INTERVAL_TICKS=7
// (~142Hz) for maximum noise margin.
static const nrf_drv_twi_config_t k_twi_config = {
    .scl                = I2C_SCL_PIN,
    .sda                = I2C_SDA_PIN,
    .frequency          = NRF_DRV_TWI_FREQ_250K,
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
    .clear_bus_init     = true
};

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);
static bool twi_initialized = false;

// Non-blocking TWI transfer flags.
// twi_xfer_done: set by twi_event_handler() (TWIM IRQ) on each transfer
//   completion or error. Cleared before each nrf_drv_twi_tx/rx call so
//   twi_wait() always starts with a clean state.
// twi_xfer_error: set alongside twi_xfer_done when the event is not
//   NRF_DRV_TWI_EVT_DONE (e.g. address NACK, data NACK).
// Both volatile: written from TWIM IRQ context, read from main loop context.
static volatile bool twi_xfer_done  = false;
static volatile bool twi_xfer_error = false;

// EasyDMA-safe buffer for the register address byte in i2c_read_regs() Phase 1.
// TWIM EasyDMA reads the source pointer asynchronously. A module-level static
// is unambiguous and never goes out of scope. Single-byte; no concurrency
// issue since all I2C calls are serialised in the main loop.
static uint8_t s_reg_buf = 0;

// EasyDMA-safe receive buffer for i2c_read_regs() Phase 2.
//
// TWIM EasyDMA writes the destination pointer asynchronously after
// nrf_drv_twi_rx() returns. If the caller passes a stack-allocated buffer
// (e.g. raw[6] in sensors_read()), and twi_wait() returns on a timeout
// path that calls nrf_drv_twi_uninit(), uninit is not instantaneous --
// the TWIM peripheral may still be mid-DMA. When the calling stack frame
// unwinds before DMA completes, DMA writes into freed stack space, corrupting
// return addresses or saved registers. The resulting HardFault produces a
// permanent CPU hang (no WDT, no LED) indistinguishable from TX being dead.
//
// Using a module-level static buffer eliminates this race entirely: the
// buffer is always valid regardless of stack state. After twi_wait() returns
// (success or timeout), the data is memcpy'd into the caller's buffer.
// The memcpy is a few nanoseconds on 6 bytes; not latency-relevant.
//
// Size 8: largest single read is 6 bytes (LSM6/LIS3/H3LIS burst), BMP uses
// 3 bytes. 8 gives one byte of margin and keeps alignment simple.
// All I2C calls are serialised in the main loop -- no concurrency issue.
#define S_RX_BUF_SIZE 8
static uint8_t s_rx_buf[S_RX_BUF_SIZE];

// Count of TWI transactions that timed out. Logged to RTT on each event and
// folded into i2c_error_count so it is visible in status packets.
static uint32_t twi_timeout_count = 0;

static uint8_t  lsm6_addr  = 0;
static uint8_t  lis3_addr  = 0;
static uint8_t  h3lis_addr = 0;
static uint8_t  bmp_addr   = 0;

static uint8_t  runtime_sensor_status = 0x00;
static uint16_t i2c_error_count = 0;
// i2c_error_count wraps at 65535. i2c_read_regs() can increment it up to twice
// per call (TX phase + RX phase). Diagnostic-only; WDT handles pathological
// I2C failure cases.
// fsr_initialized: guards the Phase 3 SAADC sequence against fsr_read() being
// called before fsr_init(). Write-only in Phase 2 build. Do not remove.
static bool     fsr_initialized  = false;

#define H3LIS_FALLBACK_SUPPRESS_AFTER  3
static uint8_t h3lis_consecutive_failures = 0;

// Compile-time check: TWI0_USE_EASY_DMA must be 1 to route nrf_drv_twi to the
// TWIM EasyDMA backend. Without this, nrf_drv_twi_init() silently uses the
// legacy blocking TWI backend -- the twi_event_handler is never called and
// every transaction blocks forever on a peripheral hang (Errata 89/121).
// If this check fails, add the following to sdk_config.h:
//   #define TWI_ENABLED 1
//   #define TWI0_USE_EASY_DMA 1
//   #define TWIM_ENABLED 1
#if !defined(TWI0_USE_EASY_DMA) || (TWI0_USE_EASY_DMA == 0)
#error "TWI0_USE_EASY_DMA must be 1 in sdk_config.h for non-blocking TWIM operation"
#endif

// ============================================================================
// TWI EVENT HANDLER (non-blocking mode)
// ============================================================================

/**
 * TWIM transfer complete/error callback.
 *
 * Called from TWIM IRQ context (APP_IRQ_PRIORITY_HIGH, priority 2).
 * Preempts the twi_wait() spin loop and sets twi_xfer_done within 1-2
 * spin iterations of the actual transfer completing.
 *
 * NRF_DRV_TWI_EVT_DONE means all bytes transferred without error.
 * Any other event type (address NACK, data NACK) sets twi_xfer_error = true.
 */
static void twi_event_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
{
    (void)p_context;
    twi_xfer_error = (p_event->type != NRF_DRV_TWI_EVT_DONE);
    twi_xfer_done  = true;
}

// ============================================================================
// LOW-LEVEL I2C
// ============================================================================

// Forward declaration: twi_wait() calls i2c_bus_recover() which is defined
// later in the file (just before sensors_init()). The forward declaration
// lets the compiler verify the signature before the first call site.
static void i2c_bus_recover(void);

/**
 * Wait for the current non-blocking TWI transfer to complete.
 *
 * Spins on twi_xfer_done with nrf_delay_us(1) per iteration. The TWIM IRQ
 * (APP_IRQ_PRIORITY_HIGH) preempts the spin and sets twi_xfer_done. Typical
 * latency from actual transfer end to flag observation: 1-2 spin iterations.
 *   Single-byte write:  ~55us
 *   6-byte burst read:  ~285us
 *
 * On timeout (TWI_TIMEOUT_US elapsed with twi_xfer_done still false):
 *   0. WDT fed immediately. The recovery sequence (STOP + uninit + bus_recover +
 *      reinit) has unpredictable duration. Without this feed the 500ms WDT window
 *      from the top-of-loop feed can be exceeded during recovery, firing the WDT
 *      and resetting TX. Feeding here gives recovery a fresh 500ms window.
 *      This is the same rationale as feeds in init_bmp581() and
 *      indicate_error_fatal(): feeding during a known recovery window is
 *      correct; not feeding causes spurious resets.
 *   1. Increments twi_timeout_count and i2c_error_count, logs to RTT.
 *   2. TWIM TASKS_STOP issued; EVENTS_STOPPED polled with 5ms bound. This
 *      terminates any in-flight DMA before uninit is called. Without this step,
 *      nrf_drv_twi_uninit() waits for ENABLE=0 to take effect, which per
 *      nRF52840 PS requires DMA to complete first. With a sensor holding SDA
 *      low, DMA never completes and uninit blocks indefinitely -- exhausting the
 *      500ms WDT window. TASKS_STOP is a peripheral-side operation and does not
 *      depend on the I2C slave releasing SDA.
 *   3. WDT fed after STOP poll (fresh 500ms window for uninit).
 *   4. nrf_drv_twi_uninit() -- stops the peripheral (fast, DMA already ended).
 *   5. WDT fed after uninit.
 *   6. twi_initialized = false.
 *   7. i2c_bus_recover() -- GPIO SCL clocking and STOP condition.
 *      Does NOT clear sensor addresses -- see design note below.
 *   8. nrf_drv_twi_init() + enable with non-blocking handler.
 *      Success: twi_initialized = true, subsequent I2C calls may work.
 *      Failure: twi_initialized = false, sensors_read() returns all zeros on
 *      the next call (entry guard), TX continues transmitting until the
 *      peripheral recovers on a subsequent call.
 *   9. Returns false -- the timed-out transfer produced no valid data.
 *
 * Design note -- sensor addresses not cleared on timeout:
 *   When a timeout occurs during sensors_read() in the running main loop,
 *   preserving the sensor addresses allows the next sensors_read() call to
 *   retry at the same addresses after bus recovery. If the bus is healthy
 *   after reinit, real data resumes immediately. If repeated hangs occur,
 *   each attempt costs at most TWI_TIMEOUT_US + recovery overhead (~10ms) --
 *   TX stays alive and recovers automatically without any intervention.
 *
 *   If addresses were cleared here, the lsm6_addr == 0 guard at the entry
 *   of sensors_read() would cause all-zeros output for the remainder of the
 *   session (main.c never calls sensors_init() after startup). That is
 *   better than TX being permanently dead, but worse than self-recovery.
 *
 *   When a timeout occurs during sensors_init() (via i2c_write_reg or
 *   i2c_read_regs called during the config phase), sensors_init() returns
 *   false because the write/read failed. Fix C in main.c retries
 *   sensors_init(), which clears addresses explicitly at its own entry.
 */
static bool twi_wait(void)
{
    for (uint32_t t = TWI_TIMEOUT_US; t > 0; t--) {
        if (twi_xfer_done) {
            if (twi_xfer_error) {
                i2c_error_count++;
                return false;
            }
            return true;
        }
        nrf_delay_us(1);
    }

    // Transfer did not complete within TWI_TIMEOUT_US -- TWIM peripheral hung.
    //
    // Feed WDT before recovery work. The recovery sequence (STOP + uninit +
    // bus_recover + reinit) has unpredictable duration. Without this feed, the
    // 500ms WDT window from the top-of-loop wdt_feed() can be exhausted during
    // recovery, firing the WDT and resetting TX.
    //
    // This is a known recovery window, not an uncontrolled stall. Feeding here
    // is correct for the same reason feeds exist in init_bmp581() and
    // indicate_error_fatal(): the WDT exists to catch unexpected stalls, not
    // deliberate recovery sequences.
    //
    // Writing RR[0] when WDT has not yet been started (RREN=0, early boot path
    // if twi_wait() is reached before wdt_init()) is harmless per nRF52840 PS.
    g_wdt_context = 0x10;  // timeout detected, entering recovery
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;

    twi_timeout_count++;
    i2c_error_count++;
    SEGGER_RTT_printf(0, "TWI: timeout #%lu -- stop, uninit, recover, reinit\r\n",
        (unsigned long)twi_timeout_count);

    // Issue TASKS_STOP to terminate any in-flight DMA before calling uninit.
    //
    // Root cause of WDT reset in running main loop (confirmed in hardware):
    // nrf_drv_twi_uninit() waits internally for TWIM ENABLE=0 to take effect.
    // Per nRF52840 PS, ENABLE=0 does not take effect until in-flight DMA
    // completes. With a sensor holding SDA low mid-transaction, DMA may never
    // complete and uninit blocks indefinitely -- exhausting the 500ms WDT window
    // between the pre-uninit feed above and the post-uninit feed below.
    //
    // TASKS_STOP requests the TWIM peripheral to abort the current transfer and
    // assert EVENTS_STOPPED. Once EVENTS_STOPPED fires, DMA is finished. uninit
    // then finds the peripheral already halted and returns immediately.
    //
    // TASKS_STOP completion does not depend on the I2C slave releasing SDA: it
    // is a peripheral-side operation. The EVENTS_STOPPED poll is bounded at
    // TWI_TIMEOUT_US (5ms) -- consistent with the per-transaction timeout above.
    g_wdt_context = 0x11;  // TASKS_STOP poll
    NRF_TWIM0->TASKS_STOP = 1;
    {
        uint32_t stop_t = TWI_TIMEOUT_US;
        while (!NRF_TWIM0->EVENTS_STOPPED && stop_t > 0U) {
            nrf_delay_us(1);
            stop_t--;
        }
        NRF_TWIM0->EVENTS_STOPPED = 0;

        if (stop_t == 0) {
            // EVENTS_STOPPED never fired -- peripheral is in deep lockup (Errata 89/121).
            // TASKS_STOP itself got stuck. The only definitive recovery per the
            // nRF52840 PS is a POWER=0/1 cycle which resets all peripheral
            // registers including the DMA state machine. TASKS_STOP alone is
            // insufficient because the peripheral is locked waiting for the bus.
            SEGGER_RTT_printf(0, "TWI: EVENTS_STOPPED timeout -- POWER cycle\r\n");
            NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;
            // POWER register is at offset 0xFFC on all nRF52840 peripherals per PS.
            // NRF_TWIM_Type does not expose it as a named field in SDK 17.1.0 headers.
            // Direct pointer cast is the correct access method.
            volatile uint32_t * const twim0_power =
                (volatile uint32_t *)((uint32_t)NRF_TWIM0 + 0xFFCu);
            *twim0_power = 0;
            nrf_delay_us(10);
            *twim0_power = 1;
            nrf_delay_us(5);
            // Peripheral registers are now reset to defaults. uninit() below
            // will find ENABLE already cleared and just clean up driver state.
        }
    }
    // Feed WDT after STOP poll + optional POWER cycle. Both are bounded but
    // we use a full feed here so uninit and bus recovery have a fresh window.
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;

    // Disable TWIM IRQ before uninit to prevent a stale IRQ firing between
    // uninit and reinit. If an IRQ fires while the driver is in UNINITIALIZED
    // state it hits NRFX_ASSERT() -> app_error_fault_handler -> soft reset.
    // nrf_drv_twi_init() re-enables the IRQ internally via NVIC_EnableIRQ.
    NVIC_DisableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);

    // nrf_drv_twi_uninit() should return immediately -- peripheral is stopped
    // (or POWER-cycled). Feed WDT after uninit so the rest of recovery has a
    // full 500ms window regardless of how long uninit took.
    g_wdt_context = 0x12;  // nrf_drv_twi_uninit()
    nrf_drv_twi_uninit(&m_twi);

    // Clear any IRQ that queued while NVIC was enabled during the timeout path.
    NVIC_ClearPendingIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;
    twi_initialized = false;

    // GPIO-level bus recovery: clocks SCL up to 18 times, sends STOP.
    // Sensor addresses are NOT cleared -- see design note in function doc.
    // The twi_initialized guard inside i2c_bus_recover() is a no-op here
    // since we already set it false (no double uninit).
    g_wdt_context = 0x13;  // i2c_bus_recover()
    i2c_bus_recover();

    // Reinit with non-blocking handler. On success subsequent calls may work.
    g_wdt_context = 0x14;  // nrf_drv_twi_init() reinit
    ret_code_t err = nrf_drv_twi_init(&m_twi, &k_twi_config, twi_event_handler, NULL);
    if (err == NRF_SUCCESS) {
        nrf_drv_twi_enable(&m_twi);
        twi_initialized = true;
        SEGGER_RTT_printf(0, "TWI: reinit OK after timeout\r\n");
    } else {
        SEGGER_RTT_printf(0, "TWI: reinit FAILED (0x%04X) after timeout\r\n",
            (unsigned)err);
    }

    return false;
}

/**
 * Write one byte to a sensor register.
 *
 * In non-blocking mode, nrf_drv_twi_tx() starts DMA and returns immediately.
 * twi_wait() spins until the TWIM IRQ fires or TWI_TIMEOUT_US elapses.
 *
 * Stack safety: the 2-byte buffer {reg, value} is on this function's stack.
 * twi_wait() is called from within this function, so the stack frame remains
 * live throughout the DMA transfer. EasyDMA reads from a valid address. ✓
 *
 * @return true if write succeeded, false on NACK, I2C error, or timeout.
 */
static bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    twi_xfer_done  = false;
    twi_xfer_error = false;
    ret_code_t err = nrf_drv_twi_tx(&m_twi, addr, data, 2, false);
    if (err != NRF_SUCCESS) {
        i2c_error_count++;
        return false;
    }
    return twi_wait();
}

/**
 * Read one or more consecutive registers from a sensor.
 *
 * Two-phase non-blocking transfer:
 *   Phase 1 (TX): send register address with xfer_pending=true (no STOP).
 *     s_reg_buf (module-level static) is the EasyDMA source -- always valid.
 *   Phase 2 (RX): DMA writes into s_rx_buf (module-level static), never into
 *     the caller's buffer directly. On success, data is memcpy'd to caller.
 *
 * Why s_rx_buf instead of the caller's buffer directly:
 *   TWIM EasyDMA writes asynchronously after nrf_drv_twi_rx() returns. If
 *   the caller passes a stack buffer and twi_wait() times out, uninit() is
 *   not instantaneous -- DMA may still be writing when the calling stack
 *   frame unwinds. DMA then writes into freed stack space, corrupting return
 *   addresses. The resulting HardFault hangs TX permanently (no WDT if pre-
 *   wdt_init(), no LED). s_rx_buf is always valid regardless of stack state.
 *
 * Each phase gets a separate twi_wait() with its own TWI_TIMEOUT_US countdown.
 * i2c_error_count may increment up to twice per call (once per failed phase).
 *
 * @return true if both phases completed without error, false otherwise.
 */
static bool i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (len > S_RX_BUF_SIZE) {
        // Caller requests more bytes than s_rx_buf can hold. This is a
        // compile-time logic error -- increase S_RX_BUF_SIZE if new sensors
        // require longer burst reads. Fail safe rather than overflow.
        i2c_error_count++;
        return false;
    }

    // Phase 1: TX register address. xfer_pending=true holds the bus (no STOP)
    // so Phase 2 issues a repeated start, as required by I2C sensor protocol.
    s_reg_buf = reg;
    twi_xfer_done  = false;
    twi_xfer_error = false;
    ret_code_t err = nrf_drv_twi_tx(&m_twi, addr, &s_reg_buf, 1, true);
    if (err != NRF_SUCCESS) { i2c_error_count++; return false; }
    if (!twi_wait()) return false;

    // Phase 2: RX into s_rx_buf. EasyDMA always writes a valid static address.
    // On timeout, uninit() may not stop DMA immediately -- s_rx_buf absorbs
    // any stray writes safely without corrupting the caller's stack.
    twi_xfer_done  = false;
    twi_xfer_error = false;
    err = nrf_drv_twi_rx(&m_twi, addr, s_rx_buf, len);
    if (err != NRF_SUCCESS) { i2c_error_count++; return false; }
    if (!twi_wait()) return false;

    // Transfer complete and verified. Copy to caller's buffer now that DMA
    // is done and twi_wait() has confirmed the IRQ fired cleanly.
    memcpy(data, s_rx_buf, len);
    return true;
}

static bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *value)
{
    return i2c_read_regs(addr, reg, value, 1);
}

// ============================================================================
// FSR
// ============================================================================

static void fsr_init(void)
{
    for (int ch = 1; ch <= 4; ch++) {
        NRF_SAADC->CH[ch].CONFIG =
            (SAADC_CH_CONFIG_RESP_Bypass     << SAADC_CH_CONFIG_RESP_Pos) |
            (SAADC_CH_CONFIG_RESN_Bypass     << SAADC_CH_CONFIG_RESN_Pos) |
            (SAADC_CH_CONFIG_GAIN_Gain1_6    << SAADC_CH_CONFIG_GAIN_Pos) |
            (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
            (SAADC_CH_CONFIG_TACQ_10us       << SAADC_CH_CONFIG_TACQ_Pos) |
            (SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos);
        NRF_SAADC->CH[ch].PSELN = SAADC_CH_PSELN_PSELN_NC;
    }
    NRF_SAADC->CH[1].PSELP = FSR0_ADC_PIN;
    NRF_SAADC->CH[2].PSELP = FSR1_ADC_PIN;
    NRF_SAADC->CH[3].PSELP = FSR2_ADC_PIN;
    NRF_SAADC->CH[4].PSELP = FSR3_ADC_PIN;

    fsr_initialized = true;
    SEGGER_RTT_printf(0, "FSR ADC channels configured (disabled until Phase 3 wiring)\r\n");
}

static bool fsr_read(uint16_t *fsr_out)
{
    memset(fsr_out, 0, 4 * sizeof(uint16_t));
    return false;

#if 0  /* Phase 3: remove the early return above and this #if 0 / #endif.
          BEFORE ENABLING: verify the resolution save/restore is present.
          BEFORE ENABLING: confirm all three SAADC timeout paths clear EVENTS_END:
            - STARTED timeout: EVENTS_END cleared ✓
            - END timeout: EVENTS_END cleared ✓
            - STOPPED timeout: EVENTS_END cleared ✓
          CHANNEL SCAN LAYOUT: MAXCNT=5, discard adc_values[0] (battery CH0).
          FSR0..FSR3 are in adc_values[1..4]. Do NOT use MAXCNT=4.  */

    if (!fsr_initialized) {
        memset(fsr_out, 0, 4 * sizeof(uint16_t));
        return false;
    }

    uint32_t saved_res = NRF_SAADC->RESOLUTION;
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_10bit;

    int16_t adc_values[5];
    NRF_SAADC->RESULT.PTR    = (uint32_t)adc_values;
    NRF_SAADC->RESULT.MAXCNT = 5;
    NRF_SAADC->TASKS_START = 1;

    uint32_t timeout = 100000;
    while (NRF_SAADC->EVENTS_STARTED == 0 && timeout > 0) { timeout--; }
    if (timeout == 0) {
        NRF_SAADC->TASKS_STOP = 1;
        uint32_t t2 = 100000;
        while (NRF_SAADC->EVENTS_STOPPED == 0 && t2 > 0) { t2--; }
        NRF_SAADC->EVENTS_STOPPED = 0;
        NRF_SAADC->EVENTS_END = 0;
        NRF_SAADC->EVENTS_STARTED = 0;
        NRF_SAADC->RESOLUTION = saved_res;
        memset(fsr_out, 0, 4 * sizeof(uint16_t));
        return false;
    }
    NRF_SAADC->EVENTS_STARTED = 0;
    NRF_SAADC->TASKS_SAMPLE = 1;

    timeout = 100000;
    while (NRF_SAADC->EVENTS_END == 0 && timeout > 0) { timeout--; }
    if (timeout == 0) {
        NRF_SAADC->TASKS_STOP = 1;
        uint32_t t2 = 100000;
        while (NRF_SAADC->EVENTS_STOPPED == 0 && t2 > 0) { t2--; }
        NRF_SAADC->EVENTS_STOPPED = 0;
        NRF_SAADC->EVENTS_END = 0;
        NRF_SAADC->RESOLUTION = saved_res;
        memset(fsr_out, 0, 4 * sizeof(uint16_t));
        return false;
    }
    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->TASKS_STOP = 1;

    timeout = 100000;
    while (NRF_SAADC->EVENTS_STOPPED == 0 && timeout > 0) { timeout--; }
    NRF_SAADC->EVENTS_STOPPED = 0;
    if (timeout == 0) {
        NRF_SAADC->EVENTS_END = 0;
        NRF_SAADC->RESOLUTION = saved_res;
        memset(fsr_out, 0, 4 * sizeof(uint16_t));
        return false;
    }
    NRF_SAADC->RESOLUTION = saved_res;

    for (int i = 0; i < 4; i++) {
        int16_t v = adc_values[i + 1];
        if      (v < 0)    fsr_out[i] = 0;
        else if (v > 1023) fsr_out[i] = 1023;
        else               fsr_out[i] = (uint16_t)v;
    }
    return true;

#endif
}

// ============================================================================
// SENSOR DETECTION
// ============================================================================

static uint8_t detect_lsm6dsox(void)
{
    uint8_t who;
    if (i2c_read_reg(LSM6DSOX_ADDR_1, LSM6_WHO_AM_I, &who) && who == LSM6DSOX_ID)
        return LSM6DSOX_ADDR_1;
    if (i2c_read_reg(LSM6DSOX_ADDR_2, LSM6_WHO_AM_I, &who) && who == LSM6DSOX_ID)
        return LSM6DSOX_ADDR_2;
    return 0;
}

static uint8_t detect_lis3mdl(void)
{
    uint8_t who;
    if (i2c_read_reg(LIS3MDL_ADDR_1, LIS3_WHO_AM_I, &who) && who == LIS3MDL_ID)
        return LIS3MDL_ADDR_1;
    if (i2c_read_reg(LIS3MDL_ADDR_2, LIS3_WHO_AM_I, &who) && who == LIS3MDL_ID)
        return LIS3MDL_ADDR_2;
    return 0;
}

static uint8_t detect_h3lis331(void)
{
    uint8_t who;
    if (i2c_read_reg(H3LIS331_ADDR, H3LIS_WHO_AM_I, &who)) {
        if (who == H3LIS331_ID) return H3LIS331_ADDR;
        SEGGER_RTT_printf(0, "Device at 0x18: WHO_AM_I=0x%02X (expected 0x32)\r\n", who);
    }
    return 0;
}

static uint8_t detect_bmp581(void)
{
    uint8_t chip_id;
    if (i2c_read_reg(BMP581_ADDR_1, BMP_CHIP_ID, &chip_id)) {
        if (chip_id == BMP581_ID || chip_id == BMP581_ID_ALT) {
            SEGGER_RTT_printf(0, "BMP581 at 0x%02X (CHIP_ID=0x%02X)\r\n",
                BMP581_ADDR_1, chip_id);
            return BMP581_ADDR_1;
        }
        SEGGER_RTT_printf(0, "Device at 0x%02X: CHIP_ID=0x%02X (expected 0x%02X or 0x%02X)\r\n",
            BMP581_ADDR_1, chip_id, BMP581_ID, BMP581_ID_ALT);
    }
    if (i2c_read_reg(BMP581_ADDR_2, BMP_CHIP_ID, &chip_id)) {
        if (chip_id == BMP581_ID || chip_id == BMP581_ID_ALT) {
            SEGGER_RTT_printf(0, "BMP581 at 0x%02X (CHIP_ID=0x%02X)\r\n",
                BMP581_ADDR_2, chip_id);
            return BMP581_ADDR_2;
        }
        SEGGER_RTT_printf(0, "Device at 0x%02X: CHIP_ID=0x%02X (expected 0x%02X or 0x%02X)\r\n",
            BMP581_ADDR_2, chip_id, BMP581_ID, BMP581_ID_ALT);
    }
    return 0;
}

// ============================================================================
// BMP581 INITIALISATION
// ============================================================================

static bool init_bmp581(void)
{
    uint8_t status  = 0;
    uint8_t reg_val = 0;

    SEGGER_RTT_printf(0, "\r\n=== BMP581 Init ===\r\n");

    SEGGER_RTT_printf(0, "Step 1: Soft reset...\r\n");
    if (!i2c_write_reg(bmp_addr, BMP_CMD, BMP_CMD_SOFT_RESET)) {
        SEGGER_RTT_printf(0, "  FAILED: write reset\r\n");
        return false;
    }
    nrf_delay_ms(50);
    // WDT feed after the 50ms soft-reset wait. On a WDT-restart boot the
    // watchdog is already running from the moment of reset. This feed prevents
    // the WDT from firing during BMP581 init and creating an infinite reset loop.
    // Safe: this is a known-duration explicit delay, not an I2C blocking call.
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;

    uint8_t chip_id;
    if (!i2c_read_reg(bmp_addr, BMP_CHIP_ID, &chip_id)) {
        SEGGER_RTT_printf(0, "  FAILED: no response after reset\r\n");
        return false;
    }
    SEGGER_RTT_printf(0, "  OK (CHIP_ID=0x%02X)\r\n", chip_id);

    SEGGER_RTT_printf(0, "Step 2: Register dump:\r\n");
    if (i2c_read_reg(bmp_addr, BMP_OSR_CONFIG, &reg_val))
        SEGGER_RTT_printf(0, "  OSR_CONFIG=0x%02X\r\n", reg_val);
    if (i2c_read_reg(bmp_addr, BMP_ODR_CONFIG, &reg_val))
        SEGGER_RTT_printf(0, "  ODR_CONFIG=0x%02X\r\n", reg_val);
    if (i2c_read_reg(bmp_addr, BMP_STATUS, &status))
        SEGGER_RTT_printf(0, "  STATUS=0x%02X\r\n", status);

    SEGGER_RTT_printf(0, "Step 3: STATUS analysis:\r\n");
    SEGGER_RTT_printf(0, "  nvm_rdy=%s nvm_err=%s drdy=%s\r\n",
        (status & 0x01) ? "ready" : "NOT READY",
        (status & 0x02) ? "ERROR" : "ok",
        (status & 0x20) ? "ready" : "not ready");
    if (status & 0x02) {
        SEGGER_RTT_printf(0, "  WARNING: NVM error flag set (observed on known-good units)\r\n");
    }

    SEGGER_RTT_printf(0, "Step 4: OSR config (PRESS_EN=1, pres x4, temp x4)...\r\n");
    if (!i2c_write_reg(bmp_addr, BMP_OSR_CONFIG, BMP_OSR_CONFIG_VAL)) {
        SEGGER_RTT_printf(0, "  FAILED\r\n"); return false;
    }
    if (!i2c_read_reg(bmp_addr, BMP_OSR_CONFIG, &reg_val)) {
        SEGGER_RTT_printf(0, "  FAILED: OSR_CONFIG readback I2C error\r\n"); return false;
    }
    SEGGER_RTT_printf(0, "  OSR_CONFIG readback=0x%02X (expect 0x%02X)\r\n",
        reg_val, BMP_OSR_CONFIG_VAL);
    if (reg_val != BMP_OSR_CONFIG_VAL) {
        SEGGER_RTT_printf(0, "  FAILED: OSR_CONFIG mismatch -- PRESS_EN may not be set\r\n");
        return false;
    }
    nrf_delay_ms(10);
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;  // feed after Step 4 delay

    SEGGER_RTT_printf(0, "Step 5: NORMAL mode (~218Hz)...\r\n");
    if (!i2c_write_reg(bmp_addr, BMP_ODR_CONFIG, BMP_ODR_CONFIG_VAL)) {
        SEGGER_RTT_printf(0, "  FAILED\r\n"); return false;
    }
    if (!i2c_read_reg(bmp_addr, BMP_ODR_CONFIG, &reg_val)) {
        SEGGER_RTT_printf(0, "  FAILED: ODR_CONFIG readback I2C error\r\n"); return false;
    }
    SEGGER_RTT_printf(0, "  ODR_CONFIG readback=0x%02X (expect 0x%02X)\r\n",
        reg_val, BMP_ODR_CONFIG_VAL);
    if (reg_val != BMP_ODR_CONFIG_VAL) {
        SEGGER_RTT_printf(0, "  FAILED: ODR_CONFIG mismatch\r\n");
        return false;
    }
    nrf_delay_ms(100);
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;  // feed after Step 5 delay

    SEGGER_RTT_printf(0, "Step 6: Waiting for data ready...\r\n");
    bool data_ready = false;
    for (int retry = 50; retry > 0; retry--) {
        // Feed WDT on every retry. This loop runs up to 50 times with 10ms
        // delays = up to 500ms total, which equals the WDT timeout. Without
        // this feed the WDT fires during Step 6 on a WDT-restart boot.
        NRF_WDT->RR[0] = WDT_RR_RR_Reload;
        if (i2c_read_reg(bmp_addr, BMP_STATUS, &status) && (status & 0x20)) {
            SEGGER_RTT_printf(0, "  DATA READY (STATUS=0x%02X)\r\n", status);
            data_ready = true;
            break;
        }
        nrf_delay_ms(10);
    }
    if (!data_ready) {
        SEGGER_RTT_printf(0, "  WARNING: no data ready (STATUS=0x%02X)\r\n", status);
    }

    SEGGER_RTT_printf(0, "Step 7: Pressure test reads...\r\n");
    uint8_t press_data[3];
    for (int attempt = 0; attempt < 5; attempt++) {
        if (i2c_read_regs(bmp_addr, BMP_PRESS_XLSB, press_data, 3)) {
            uint32_t raw = (uint32_t)press_data[0] |
                          ((uint32_t)press_data[1] << 8) |
                          ((uint32_t)press_data[2] << 16);
            uint32_t pa  = raw / 64;
            SEGGER_RTT_printf(0, "  [%d] raw=0x%06X = %u Pa\r\n", attempt + 1, raw, pa);
            if (raw != 0x7F7F7F && pa >= 30000 && pa <= 110000) {
                SEGGER_RTT_printf(0, "=== BMP581 OK ===\r\n\r\n");
                return true;
            }
        }
        // Skip the inter-attempt delay after the final attempt -- wastes 50ms
        // before returning false with no further reads to benefit.
        if (attempt < 4) nrf_delay_ms(50);
    }

    SEGGER_RTT_printf(0, "=== BMP581 FAILED ===\r\n");
    SEGGER_RTT_printf(0, "All 5 pressure reads returned invalid data (0x7F7F7F, I2C error, or out-of-range)\r\n");
    SEGGER_RTT_printf(0, "Possible causes: PRESS_EN not set (check readback 0x%02X), wiring, defective module\r\n\r\n",
        BMP_OSR_CONFIG_VAL);
    return false;
}

// ============================================================================
// I2C BUS RECOVERY
// ============================================================================

/**
 * GPIO-level I2C bus recovery.
 *
 * After a WDT reset following a TWI peripheral hang (nRF52840 Errata 89/121),
 * a sensor may still be holding SDA low. The peripheral registers are cleared
 * by the reset, but the sensor is still mid-transaction on the physical bus.
 * clear_bus_init=true in the TWI config provides up to 9 clocks, but
 * worst-case state alignment can require more.
 *
 * This function:
 *   1. Uninits the TWI peripheral if running (releases SCL/SDA pin control)
 *   2. Configures SCL and SDA as open-drain GPIO with input buffer connected
 *   3. Clocks SCL up to 18 times, stopping early if SDA goes high
 *   4. Sends a STOP condition (SDA low->high while SCL high)
 *   5. Releases both pins high -- nrf_drv_twi_init() reconfigures them
 *
 * Does NOT clear sensor addresses.
 *   sensors_init() clears addresses explicitly before calling here, so that
 *   re-detection always follows. twi_wait() does not clear addresses so that
 *   sensors_read() can retry after a mid-operation bus recovery without needing
 *   a full re-init -- see design note in twi_wait() for the full rationale.
 *
 * Safe to call on a clean boot: if SDA is already high, only steps 1-2 and
 * 5 execute. nrf_drv_twi_init() reconfigures the pins normally.
 *
 * Called from two sites:
 *   1. sensors_init(): before every init attempt. twi_initialized may be true
 *      (Fix C retry after a partial init) or false (first boot). The uninit
 *      guard below handles both cases.
 *   2. twi_wait() timeout path: after TASKS_STOP, nrf_drv_twi_uninit() and
 *      twi_initialized=false are already set. The uninit guard below is a
 *      no-op in that case (correct -- prevents a double uninit).
 *      twi_wait() feeds the WDT before calling here, so no WDT feed is
 *      needed inside this function for the twi_wait() call path.
 */
static void i2c_bus_recover(void)
{
    // Uninit TWI peripheral if it was previously initialized, releasing
    // control of SCL and SDA back to GPIO.
    if (twi_initialized) {
        nrf_drv_twi_uninit(&m_twi);
        twi_initialized = false;
    }

    // Configure SCL and SDA as open-drain outputs with input buffer connected.
    // S0D1 drive: drives low strongly, releases high (open-drain). Matches I2C
    // electrical spec. Input buffer connected so we can read SDA state.
    nrf_gpio_cfg(I2C_SCL_PIN,
                 NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_CONNECT,
                 NRF_GPIO_PIN_PULLUP,
                 NRF_GPIO_PIN_S0D1,
                 NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(I2C_SDA_PIN,
                 NRF_GPIO_PIN_DIR_OUTPUT,
                 NRF_GPIO_PIN_INPUT_CONNECT,
                 NRF_GPIO_PIN_PULLUP,
                 NRF_GPIO_PIN_S0D1,
                 NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_pin_set(I2C_SCL_PIN);
    nrf_gpio_pin_set(I2C_SDA_PIN);
    nrf_delay_us(10);

    // Clock SCL up to 18 times to release a stuck slave.
    // A slave needs at most 9 clocks to finish a byte and release SDA. We use
    // 18 to handle worst-case state alignment (e.g. slave mid-byte on both a
    // read and an ACK). Stop early if SDA goes high (slave released the line).
    bool sda_was_stuck = false;
    for (int i = 0; i < 18; i++) {
        if (nrf_gpio_pin_read(I2C_SDA_PIN)) {
            break;  // SDA released, no further clocks needed
        }
        sda_was_stuck = true;
        nrf_gpio_pin_clear(I2C_SCL_PIN);
        nrf_delay_us(5);
        nrf_gpio_pin_set(I2C_SCL_PIN);
        nrf_delay_us(5);
    }

    // Send a STOP condition: SDA transitions low -> high while SCL is high.
    // This terminates any in-progress transaction on the bus, returning all
    // slaves to idle state regardless of what they were doing.
    nrf_gpio_pin_clear(I2C_SDA_PIN);
    nrf_delay_us(5);
    nrf_gpio_pin_set(I2C_SCL_PIN);
    nrf_delay_us(5);
    nrf_gpio_pin_set(I2C_SDA_PIN);
    nrf_delay_us(10);

    SEGGER_RTT_printf(0, "I2C bus recovery: SDA was %s\r\n",
        sda_was_stuck ? "STUCK -- recovered" : "OK (no hang)");
}

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

bool sensors_init(void)
{
    // Clear sensor addresses and status bitmask before bus recovery and re-detection.
    // runtime_sensor_status reset prevents stale bits from a failed first attempt
    // persisting into a Fix C retry (main.c calls sensors_init() twice on failure).
    // Done here rather than inside i2c_bus_recover() so that bus recovery called
    // from twi_wait() during sensors_read() does NOT clear addresses or status --
    // allowing normal operation to resume after bus recovery without a full re-init.
    lsm6_addr  = 0;
    lis3_addr  = 0;
    h3lis_addr = 0;
    bmp_addr   = 0;
    runtime_sensor_status = 0x00;

    // Recover the I2C bus before initialising the TWI peripheral.
    // On a post-WDT-reset boot a sensor may be holding SDA low mid-transaction.
    // This clears that physical bus state before handing the pins to the TWI
    // driver. On a clean boot SDA is already high and only the GPIO config and
    // STOP condition execute -- nrf_drv_twi_init() reconfigures the pins.
    i2c_bus_recover();

    // Pass twi_event_handler (non-NULL) to select non-blocking mode.
    // In non-blocking mode, nrf_drv_twi_tx/rx return immediately after starting
    // DMA. The handler fires from the TWIM IRQ when each transfer completes or
    // errors. i2c_write_reg/i2c_read_regs call twi_wait() to spin-wait with a
    // TWI_TIMEOUT_US per-phase timeout.
    //
    // Previously NULL was passed here (blocking mode). In blocking mode,
    // nrf_drv_twi_tx/rx spin forever when the TWIM peripheral hangs (Errata
    // 89/121). This was the root cause of TX permanently stalling after a
    // TWI hang.
    ret_code_t err = nrf_drv_twi_init(&m_twi, &k_twi_config, twi_event_handler, NULL);
    if (err != NRF_SUCCESS) {
        SEGGER_RTT_printf(0, "ERROR: I2C init failed\r\n");
        return false;
    }
    nrf_drv_twi_enable(&m_twi);
    twi_initialized = true;
    nrf_delay_ms(20);

    lsm6_addr = detect_lsm6dsox();
    if (lsm6_addr == 0) {
        SEGGER_RTT_printf(0, "ERROR: LSM6DSOX not found\r\n");
        return false;
    }
    SEGGER_RTT_printf(0, "LSM6DSOX at 0x%02X\r\n", lsm6_addr);
    runtime_sensor_status |= SENSOR_LSM6_OK;

    lis3_addr = detect_lis3mdl();
    if (lis3_addr == 0) {
        SEGGER_RTT_printf(0, "ERROR: LIS3MDL not found\r\n");
        return false;
    }
    SEGGER_RTT_printf(0, "LIS3MDL at 0x%02X\r\n", lis3_addr);
    runtime_sensor_status |= SENSOR_LIS3_OK;

    h3lis_addr = detect_h3lis331();
    if (h3lis_addr == 0) {
        SEGGER_RTT_printf(0, "WARNING: H3LIS331 not found\r\n");
    } else {
        SEGGER_RTT_printf(0, "H3LIS331 at 0x%02X\r\n", h3lis_addr);
        runtime_sensor_status |= SENSOR_H3LIS_OK;
    }

    bmp_addr = detect_bmp581();
    if (bmp_addr == 0) {
        SEGGER_RTT_printf(0, "WARNING: BMP581 not found\r\n");
    } else {
        runtime_sensor_status |= SENSOR_BMP_OK;
    }

    // LSM6DSOX configuration
    if (!i2c_write_reg(lsm6_addr, LSM6_CTRL1_XL, LSM6_CTRL1_XL_VAL)) {
        SEGGER_RTT_printf(0, "ERROR: LSM6DSOX CTRL1_XL failed\r\n");
        return false;
    }
    if (!i2c_write_reg(lsm6_addr, LSM6_CTRL2_G, LSM6_CTRL2_G_VAL)) {
        SEGGER_RTT_printf(0, "ERROR: LSM6DSOX CTRL2_G failed\r\n");
        return false;
    }
    if (!i2c_write_reg(lsm6_addr, LSM6_CTRL3_C, LSM6_CTRL3_C_VAL)) {
        SEGGER_RTT_printf(0, "ERROR: LSM6DSOX CTRL3_C failed\r\n");
        return false;
    }
    SEGGER_RTT_printf(0, "LSM6DSOX: 416Hz, +/-16g, +/-500dps, BDU enabled\r\n");

    // LIS3MDL configuration
    if (!i2c_write_reg(lis3_addr, LIS3_CTRL_REG1, LIS3_CTRL_REG1_VAL)) {
        SEGGER_RTT_printf(0, "ERROR: LIS3MDL CTRL_REG1 failed\r\n");
        return false;
    }
    if (!i2c_write_reg(lis3_addr, LIS3_CTRL_REG2, LIS3_CTRL_REG2_VAL)) {
        SEGGER_RTT_printf(0, "ERROR: LIS3MDL CTRL_REG2 failed\r\n");
        return false;
    }
    if (!i2c_write_reg(lis3_addr, LIS3_CTRL_REG3, LIS3_CTRL_REG3_VAL)) {
        SEGGER_RTT_printf(0, "ERROR: LIS3MDL CTRL_REG3 failed\r\n");
        return false;
    }
    if (!i2c_write_reg(lis3_addr, LIS3_CTRL_REG4, LIS3_CTRL_REG4_VAL)) {
        SEGGER_RTT_printf(0, "ERROR: LIS3MDL CTRL_REG4 failed\r\n");
        return false;
    }
    SEGGER_RTT_printf(0, "LIS3MDL: 155Hz (FAST_ODR), +/-4 gauss, continuous\r\n");

    // H3LIS331 configuration (optional)
    if (h3lis_addr != 0) {
        if (!i2c_write_reg(h3lis_addr, H3LIS_CTRL_REG1, H3LIS_CTRL_REG1_VAL)) {
            SEGGER_RTT_printf(0, "WARNING: H3LIS331 CTRL_REG1 failed\r\n");
            h3lis_addr = 0;
            runtime_sensor_status &= ~SENSOR_H3LIS_OK;
        }
    }
    if (h3lis_addr != 0) {
        if (!i2c_write_reg(h3lis_addr, H3LIS_CTRL_REG4, H3LIS_CTRL_REG4_VAL)) {
            SEGGER_RTT_printf(0, "WARNING: H3LIS331 CTRL_REG4 failed\r\n");
            h3lis_addr = 0;
            runtime_sensor_status &= ~SENSOR_H3LIS_OK;
        }
    }
    if (h3lis_addr != 0) {
        SEGGER_RTT_printf(0, "H3LIS331: +/-400g, 400Hz, BDU enabled\r\n");
    }

    // BMP581 configuration (optional)
    if (bmp_addr != 0) {
        if (!init_bmp581()) {
            SEGGER_RTT_printf(0, "WARNING: BMP581 init failed -- continuing without pressure\r\n");
            bmp_addr = 0;
            runtime_sensor_status &= ~SENSOR_BMP_OK;
        }
    }

    fsr_init();

    nrf_delay_ms(10);
    return true;
}

void sensors_get_addresses(uint8_t *lsm6, uint8_t *lis3, uint8_t *h3lis, uint8_t *bmp)
{
    if (lsm6)  *lsm6  = lsm6_addr;
    if (lis3)  *lis3  = lis3_addr;
    if (h3lis) *h3lis = h3lis_addr;
    if (bmp)   *bmp   = bmp_addr;
}

void sensors_read(sensor_data_t *data)
{
    uint8_t  raw[6];
    int16_t  h3lis_raw[3];
    uint16_t fsr_raw[4];

    if (!twi_initialized || lsm6_addr == 0 || lis3_addr == 0) {
        // Both lsm6_addr and lis3_addr must be non-zero: they are required sensors
        // and sensors_init() returns false if either is absent. In the main loop
        // twi_wait() recovery path addresses are preserved (not cleared), so if
        // a required sensor is somehow 0 here that is an unrecoverable init gap --
        // return zeros rather than attempt I2C to address 0 and NACK.
        memset(data, 0, sizeof(sensor_data_t));
        return;
    }

    // SDA pre-flight check: if SDA is low before any transaction starts, a
    // sensor is already holding the bus and every TWIM transaction will hit
    // EVENTS_ERROR -> SDK assert -> soft reset. Catch this here and run
    // bus recovery before attempting any I2C operation.
    // SCL must be high (idle) before reading SDA; it is, because the TWIM
    // peripheral is idle between loop iterations.
    // nrf_gpio_pin_read() works on TWI-configured pins: the input buffer is
    // connected (NRF_GPIO_PIN_INPUT_CONNECT is set by nrf_drv_twi_init via
    // the SDK pin config). Reading a TWI pin returns the current line state.
    if (!nrf_gpio_pin_read(I2C_SDA_PIN)) {
        SEGGER_RTT_printf(0, "sensors_read: SDA low before tx -- bus recovery\r\n");
        i2c_error_count++;
        i2c_bus_recover();
        // Reinit after GPIO-level recovery. Addresses preserved (see design
        // note in twi_wait). If reinit fails, twi_initialized stays false and
        // the guard at the top of the next sensors_read() call returns zeros.
        ret_code_t err = nrf_drv_twi_init(&m_twi, &k_twi_config,
                                           twi_event_handler, NULL);
        if (err == NRF_SUCCESS) {
            nrf_drv_twi_enable(&m_twi);
            twi_initialized = true;
        } else {
            twi_initialized = false;
            memset(data, 0, sizeof(sensor_data_t));
            return;
        }
    }

    // LSM6DSOX gyro and accel. lsm6_ok is true only if BOTH reads succeed.
    bool lsm6_ok = true;

    if (i2c_read_regs(lsm6_addr, LSM6_OUTX_L_G, raw, 6)) {
        data->imu.gyro[0] = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
        data->imu.gyro[1] = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
        data->imu.gyro[2] = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));
    } else {
        memset(data->imu.gyro, 0, sizeof(data->imu.gyro));
        lsm6_ok = false;
    }

    if (i2c_read_regs(lsm6_addr, LSM6_OUTX_L_A, raw, 6)) {
        data->imu.accel[0] = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
        data->imu.accel[1] = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
        data->imu.accel[2] = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));
    } else {
        memset(data->imu.accel, 0, sizeof(data->imu.accel));
        lsm6_ok = false;
    }

    if (lsm6_ok) runtime_sensor_status |=  SENSOR_LSM6_OK;
    else         runtime_sensor_status &= ~SENSOR_LSM6_OK;

    // LIS3MDL magnetometer. | 0x80 required for auto-increment on multi-byte read.
    if (lis3_addr != 0) {
        if (i2c_read_regs(lis3_addr, LIS3_OUT_X_L | 0x80, raw, 6)) {
            data->imu.mag[0] = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
            data->imu.mag[1] = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
            data->imu.mag[2] = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));
            runtime_sensor_status |= SENSOR_LIS3_OK;
        } else {
            memset(data->imu.mag, 0, sizeof(data->imu.mag));
            runtime_sensor_status &= ~SENSOR_LIS3_OK;
        }
    } else {
        memset(data->imu.mag, 0, sizeof(data->imu.mag));
    }

    // H3LIS331 + FSR encoding. | 0x80 required for auto-increment.
    if (h3lis_addr != 0) {
        bool h3lis_ok = i2c_read_regs(h3lis_addr, H3LIS_OUT_X_L | 0x80, raw, 6);

        if (!h3lis_ok) {
            if (h3lis_consecutive_failures < H3LIS_FALLBACK_SUPPRESS_AFTER) {
                h3lis_ok = true;
                for (int axis = 0; axis < 3; axis++) {
                    uint8_t lo, hi;
                    uint8_t base = H3LIS_OUT_X_L + (axis * 2);
                    if (!i2c_read_reg(h3lis_addr, base, &lo) ||
                        !i2c_read_reg(h3lis_addr, base + 1, &hi)) {
                        h3lis_ok = false;
                        break;
                    }
                    raw[axis * 2]     = lo;
                    raw[axis * 2 + 1] = hi;
                }
            }
            if (!h3lis_ok) {
                if (h3lis_consecutive_failures < H3LIS_FALLBACK_SUPPRESS_AFTER + 1)
                    h3lis_consecutive_failures++;
            }
        }

        if (h3lis_ok) {
            h3lis_consecutive_failures = 0;
            h3lis_raw[0] = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
            h3lis_raw[1] = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
            h3lis_raw[2] = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));

            (void)fsr_read(fsr_raw);

            uint16_t max_fsr = 0;
            for (int i = 0; i < 4; i++) {
                if (fsr_raw[i] > max_fsr) max_fsr = fsr_raw[i];
            }

            uint8_t fsr_intensity = (max_fsr >> 6) & 0x0F;
            uint8_t fsr_pattern   = 0;
            if (fsr_raw[0] > FSR_CONTACT_THRESHOLD) fsr_pattern |= 0x01;
            if (fsr_raw[1] > FSR_CONTACT_THRESHOLD) fsr_pattern |= 0x02;
            if (fsr_raw[2] > FSR_CONTACT_THRESHOLD) fsr_pattern |= 0x04;
            if (fsr_raw[3] > FSR_CONTACT_THRESHOLD) fsr_pattern |= 0x08;

            // Unsigned domain arithmetic: (int16_t)0xFFF0 exceeds INT16_MAX;
            // narrowing cast is implementation-defined in C99/C11.
            data->h3lis_x_fsr_level   = (int16_t)(((uint16_t)h3lis_raw[0] & 0xFFF0u) | (uint16_t)fsr_intensity);
            data->h3lis_y_fsr_pattern = (int16_t)(((uint16_t)h3lis_raw[1] & 0xFFF0u) | (uint16_t)fsr_pattern);
            // Lower 4 bits of z-axis field are reserved; mask clears them.
            data->h3lis_z_flags       = (int16_t)((uint16_t)h3lis_raw[2] & 0xFFF0u);
            runtime_sensor_status |= SENSOR_H3LIS_OK;
        } else {
            data->h3lis_x_fsr_level   = 0;
            data->h3lis_y_fsr_pattern = 0;
            data->h3lis_z_flags       = 0;
            runtime_sensor_status &= ~SENSOR_H3LIS_OK;
        }
    } else {
        data->h3lis_x_fsr_level   = 0;
        data->h3lis_y_fsr_pattern = 0;
        data->h3lis_z_flags       = 0;
    }

    // BMP581 pressure (raw in 1/64 Pa; Pa = raw/64)
    if (bmp_addr != 0) {
        uint8_t press[3];
        if (i2c_read_regs(bmp_addr, BMP_PRESS_XLSB, press, 3)) {
            data->pressure[0] = press[0];
            data->pressure[1] = press[1];
            data->pressure[2] = press[2];
            runtime_sensor_status |= SENSOR_BMP_OK;
        } else {
            memset(data->pressure, 0, 3);
            runtime_sensor_status &= ~SENSOR_BMP_OK;
        }
    } else {
        memset(data->pressure, 0, 3);
    }
}

bool sensors_test(void)
{
    uint8_t id;
    bool ok = true;
    if (!twi_initialized) return false;

    if (lsm6_addr == 0 ||
        !i2c_read_reg(lsm6_addr, LSM6_WHO_AM_I, &id) || id != LSM6DSOX_ID) {
        SEGGER_RTT_printf(0, "LSM6DSOX test FAILED\r\n");
        ok = false;
    }
    if (lis3_addr == 0 ||
        !i2c_read_reg(lis3_addr, LIS3_WHO_AM_I, &id) || id != LIS3MDL_ID) {
        SEGGER_RTT_printf(0, "LIS3MDL test FAILED\r\n");
        ok = false;
    }
    if (h3lis_addr != 0) {
        if (!i2c_read_reg(h3lis_addr, H3LIS_WHO_AM_I, &id) || id != H3LIS331_ID) {
            SEGGER_RTT_printf(0, "H3LIS331 test FAILED\r\n");
            ok = false;
        }
    }
    if (bmp_addr != 0) {
        if (!i2c_read_reg(bmp_addr, BMP_CHIP_ID, &id) ||
            (id != BMP581_ID && id != BMP581_ID_ALT)) {
            SEGGER_RTT_printf(0, "BMP581 test FAILED\r\n");
            ok = false;
        }
    }
    return ok;
}

uint8_t  sensors_get_status_bitmask(void)   { return runtime_sensor_status; }
uint16_t sensors_get_i2c_error_count(void)  { return i2c_error_count; }

int16_t sensors_read_temperature(void)
{
    if (bmp_addr == 0) return SENSORS_TEMP_UNAVAILABLE;

    uint8_t t[3];
    if (!i2c_read_regs(bmp_addr, BMP_TEMP_XLSB, t, 3))
        return SENSORS_TEMP_UNAVAILABLE;

    int32_t raw = (int32_t)((uint32_t)t[0] | ((uint32_t)t[1] << 8) | ((uint32_t)t[2] << 16));

    // Sign-extend from bit 23. ~(int32_t)0x00FFFFFF is well-defined;
    // (int32_t)0xFF000000 is not (exceeds INT_MAX in C99/C11).
    if (raw & 0x800000) raw |= ~(int32_t)0x00FFFFFF;

    // BMP581: 1/65536 C per LSB -> 0.01 C units
    return (int16_t)((raw * 100) / 65536);
}