/**
 * Sensors Module Implementation
 *
 * Fixes applied across review and hardware validation sessions:
 *   - LIS3MDL burst read: | 0x80 on register address (was reading X-low 6x)
 *   - CTRL2_G: 0x64 for +/-500dps (was 0x68 = +/-1000dps)
 *   - CTRL3_C: BDU + IF_INC enabled on LSM6DSOX (prevents split-sample reads)
 *   - runtime_sensor_status initialised 0x00 (was 0x0F — falsely reported all OK)
 *   - init_bmp581: status/reg_val initialised to 0 (was UB if I2C failed early)
 *   - BMP581 STATUS bit 0 nvm_rdy interpretation corrected (1=ready, not busy)
 *   - BMP581 OSR_CONFIG: 0x52 not 0x12 — bit 6 (PRESS_EN) must be set to
 *     enable pressure measurements. Without it the output registers stay at
 *     0x7F7F7F indefinitely regardless of other configuration. Confirmed in
 *     hardware testing: two units failed identically until this was corrected.
 *   - sensors_read_temperature: returns SENSORS_TEMP_UNAVAILABLE on failure
 *     instead of 0 (0 is a valid temperature and is ambiguous as an error)
 *   - fsr_init comment corrected: SAADC not yet enabled when fsr_init() runs
 *   - fsr_read: SAADC resolution save/restore scaffolded for Phase 3 wiring
 */

#include "sensors.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_saadc.h"
#include <string.h>

extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);

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
#define BMP581_ADDR_1   0x46
#define BMP581_ADDR_2   0x47

// ============================================================================
// FSR CONFIGURATION
// ============================================================================

// FSR ADC pins — channels 1-4 (battery uses channel 0 in main.c)
// SAADC owned by main.c. battery_init() sets 12-bit resolution.
// fsr_read() must save/restore resolution: FSR calibration is for 10-bit.
#define FSR0_ADC_PIN    NRF_SAADC_INPUT_AIN1  // P0.03
#define FSR1_ADC_PIN    NRF_SAADC_INPUT_AIN2  // P0.04
#define FSR2_ADC_PIN    NRF_SAADC_INPUT_AIN3  // P0.05
#define FSR3_ADC_PIN    NRF_SAADC_INPUT_AIN4  // P0.28

// Contact threshold and intensity encoding calibrated for 10-bit ADC (0-1023)
#define FSR_CONTACT_THRESHOLD   80

// ============================================================================
// LSM6DSOX REGISTERS
// ============================================================================

#define LSM6_WHO_AM_I   0x0F
#define LSM6_CTRL1_XL   0x10    // Accel ODR/range
#define LSM6_CTRL2_G    0x11    // Gyro ODR/range
#define LSM6_CTRL3_C    0x12    // BDU, IF_INC, etc.
#define LSM6_OUTX_L_G   0x22
#define LSM6_OUTX_L_A   0x28
#define LSM6DSOX_ID     0x6C

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

// ============================================================================
// H3LIS331 REGISTERS
// ============================================================================

#define H3LIS_WHO_AM_I  0x0F
#define H3LIS_CTRL_REG1 0x20
#define H3LIS_CTRL_REG4 0x23
#define H3LIS_OUT_X_L   0x28
#define H3LIS331_ID     0x32

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

// ============================================================================
// GLOBAL STATE
// ============================================================================

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);
static bool twi_initialized = false;

static uint8_t lsm6_addr  = 0;
static uint8_t lis3_addr  = 0;
static uint8_t h3lis_addr = 0;
static uint8_t bmp_addr   = 0;

// Initialised to 0x00: do not report sensors OK before detection runs
static uint8_t  runtime_sensor_status = 0x00;
static uint16_t i2c_error_count = 0;
static bool     fsr_initialized  = false;

// ============================================================================
// LOW-LEVEL I2C
// ============================================================================

static bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    ret_code_t err = nrf_drv_twi_tx(&m_twi, addr, data, 2, false);
    if (err != NRF_SUCCESS) {
        i2c_error_count++;
        return false;
    }
    return true;
}

static bool i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    ret_code_t err;
    err = nrf_drv_twi_tx(&m_twi, addr, &reg, 1, true);
    if (err != NRF_SUCCESS) { i2c_error_count++; return false; }
    err = nrf_drv_twi_rx(&m_twi, addr, data, len);
    if (err != NRF_SUCCESS) { i2c_error_count++; return false; }
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
    // Configure channels 1-4. SAADC is NOT yet enabled here — battery_init()
    // in main.c runs after sensors_init() and enables it at 12-bit resolution.
    // Channel config writes are valid before SAADC enable.
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
    // DISABLED: FSRs not yet wired.
    //
    // BEFORE UNCOMMENTING: add resolution save/restore around the sample:
    //   uint32_t saved_res = NRF_SAADC->RESOLUTION;
    //   NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_10bit;
    //   ... sample channels 1-4 into int16_t adc_values[4] ...
    //   NRF_SAADC->RESOLUTION = saved_res;
    //
    // FSR_CONTACT_THRESHOLD (80) and >>6 intensity scaling are calibrated for
    // 10-bit (0-1023). battery_init() sets 12-bit; without save/restore the
    // threshold becomes 4x too sensitive and intensity encoding is wrong.
    //
    // Wiring targets:
    //   FSR0 -> P0.03 (SAADC Ch1)
    //   FSR1 -> P0.04 (SAADC Ch2)
    //   FSR2 -> P0.05 (SAADC Ch3)
    //   FSR3 -> P0.28 (SAADC Ch4)
    memset(fsr_out, 0, 4 * sizeof(uint16_t));
    return false;

    /* UNCOMMENT WHEN FSRs ARE WIRED:

    if (!fsr_initialized) {
        memset(fsr_out, 0, 4 * sizeof(uint16_t));
        return false;
    }

    uint32_t saved_res = NRF_SAADC->RESOLUTION;
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_10bit;

    int16_t adc_values[4];
    NRF_SAADC->RESULT.PTR    = (uint32_t)adc_values;
    NRF_SAADC->RESULT.MAXCNT = 4;
    NRF_SAADC->TASKS_START = 1;

    uint32_t timeout = 100000;
    while (NRF_SAADC->EVENTS_STARTED == 0 && timeout > 0) { timeout--; }
    if (timeout == 0) {
        NRF_SAADC->RESOLUTION = saved_res;
        memset(fsr_out, 0, 4 * sizeof(uint16_t));
        return false;
    }
    NRF_SAADC->EVENTS_STARTED = 0;
    NRF_SAADC->TASKS_SAMPLE = 1;

    timeout = 100000;
    while (NRF_SAADC->EVENTS_END == 0 && timeout > 0) { timeout--; }
    if (timeout == 0) {
        NRF_SAADC->RESOLUTION = saved_res;
        memset(fsr_out, 0, 4 * sizeof(uint16_t));
        return false;
    }
    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->TASKS_STOP = 1;

    timeout = 100000;
    while (NRF_SAADC->EVENTS_STOPPED == 0 && timeout > 0) { timeout--; }
    if (timeout == 0) {
        NRF_SAADC->RESOLUTION = saved_res;
        memset(fsr_out, 0, 4 * sizeof(uint16_t));
        return false;
    }
    NRF_SAADC->EVENTS_STOPPED = 0;
    NRF_SAADC->RESOLUTION = saved_res;

    for (int i = 0; i < 4; i++) {
        if      (adc_values[i] < 0)    fsr_out[i] = 0;
        else if (adc_values[i] > 1023) fsr_out[i] = 1023;
        else                           fsr_out[i] = (uint16_t)adc_values[i];
    }
    return true;

    END OF COMMENTED CODE */
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
    }
    if (i2c_read_reg(BMP581_ADDR_2, BMP_CHIP_ID, &chip_id)) {
        if (chip_id == BMP581_ID || chip_id == BMP581_ID_ALT) {
            SEGGER_RTT_printf(0, "BMP581 at 0x%02X (CHIP_ID=0x%02X)\r\n",
                BMP581_ADDR_2, chip_id);
            return BMP581_ADDR_2;
        }
    }
    return 0;
}

// ============================================================================
// BMP581 INITIALISATION
// ============================================================================

static bool init_bmp581(void)
{
    // Initialise to 0: safe to print even if I2C reads fail before assignment
    uint8_t status  = 0;
    uint8_t reg_val = 0;

    SEGGER_RTT_printf(0, "\r\n=== BMP581 Init ===\r\n");

    SEGGER_RTT_printf(0, "Step 1: Soft reset...\r\n");
    if (!i2c_write_reg(bmp_addr, BMP_CMD, BMP_CMD_SOFT_RESET)) {
        SEGGER_RTT_printf(0, "  FAILED: write reset\r\n");
        return false;
    }
    nrf_delay_ms(50);

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

    // nvm_rdy (bit 0): 1 = NVM subsystem ready (datasheet confirmed)
    // nvm_err (bit 1): 1 = NVM error (calibration load failed)
    // NOTE: nvm_err has been observed on genuine working Adafruit BMP581
    // breakouts during hardware testing. It does not prevent correct operation
    // provided pressure reads return valid data. Do not treat it as fatal.
    SEGGER_RTT_printf(0, "Step 3: STATUS analysis:\r\n");
    SEGGER_RTT_printf(0, "  nvm_rdy=%s nvm_err=%s drdy=%s\r\n",
        (status & 0x01) ? "ready" : "NOT READY",
        (status & 0x02) ? "ERROR" : "ok",
        (status & 0x20) ? "ready" : "not ready");
    if (status & 0x02) {
        SEGGER_RTT_printf(0, "  WARNING: NVM error flag set (observed on known-good units)\r\n");
    }

    // OSR_CONFIG = 0x52:
    //   bit 6 (PRESS_EN) = 1  — MUST be set to enable pressure measurement.
    //                           Without this bit, output registers stay at
    //                           0x7F7F7F regardless of all other config.
    //   bits [4:2] (OSR_T)    = 001 = temperature oversampling x2 (effectively x4 with OSR_P)
    //   bits [1:0] (OSR_P)    = 10  = pressure oversampling x4
    // 0x52 = 0101 0010
    SEGGER_RTT_printf(0, "Step 4: OSR config (PRESS_EN=1, pres x4, temp x4)...\r\n");
    if (!i2c_write_reg(bmp_addr, BMP_OSR_CONFIG, 0x52)) {
        SEGGER_RTT_printf(0, "  FAILED\r\n"); return false;
    }
    if (i2c_read_reg(bmp_addr, BMP_OSR_CONFIG, &reg_val))
        SEGGER_RTT_printf(0, "  OSR_CONFIG readback=0x%02X (expect 0x52)\r\n", reg_val);
    nrf_delay_ms(10);

    SEGGER_RTT_printf(0, "Step 5: NORMAL mode (~218Hz)...\r\n");
    if (!i2c_write_reg(bmp_addr, BMP_ODR_CONFIG, 0x11)) {
        SEGGER_RTT_printf(0, "  FAILED\r\n"); return false;
    }
    if (i2c_read_reg(bmp_addr, BMP_ODR_CONFIG, &reg_val))
        SEGGER_RTT_printf(0, "  ODR_CONFIG readback=0x%02X (expect 0x11)\r\n", reg_val);
    nrf_delay_ms(100);

    SEGGER_RTT_printf(0, "Step 6: Waiting for data ready...\r\n");
    bool data_ready = false;
    for (int retry = 50; retry > 0; retry--) {
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
            uint32_t raw = press_data[0] |
                          (press_data[1] << 8) |
                          (press_data[2] << 16);
            uint32_t pa  = raw / 64;
            SEGGER_RTT_printf(0, "  [%d] raw=0x%06X = %u Pa\r\n", attempt + 1, raw, pa);
            if (raw != 0x7F7F7F && pa >= 30000 && pa <= 110000) {
                SEGGER_RTT_printf(0, "=== BMP581 OK ===\r\n\r\n");
                return true;
            }
        }
        nrf_delay_ms(50);
    }

    SEGGER_RTT_printf(0, "=== BMP581 FAILED ===\r\n");
    SEGGER_RTT_printf(0, "All reads returned 0x7F7F7F\r\n");
    SEGGER_RTT_printf(0, "Possible causes: PRESS_EN not set (check 0x52), wiring, defective module\r\n\r\n");
    return false;
}

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

bool sensors_init(void)
{
    const nrf_drv_twi_config_t twi_config = {
        .scl                = I2C_SCL_PIN,
        .sda                = I2C_SDA_PIN,
        .frequency          = NRF_DRV_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init     = false
    };

    ret_code_t err = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    if (err != NRF_SUCCESS) {
        SEGGER_RTT_printf(0, "ERROR: I2C init failed\r\n");
        return false;
    }
    nrf_drv_twi_enable(&m_twi);
    twi_initialized = true;
    nrf_delay_ms(20);

    // Required sensors
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

    // Optional sensors
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

    // ---- LSM6DSOX configuration ----

    // CTRL1_XL: 416Hz ODR, +/-16g
    // 0x66 = 0110 0110: ODR[7:4]=0110=416Hz, FS_XL[3:2]=11=+/-16g
    if (!i2c_write_reg(lsm6_addr, LSM6_CTRL1_XL, 0x66)) {
        SEGGER_RTT_printf(0, "ERROR: LSM6DSOX CTRL1_XL failed\r\n");
        return false;
    }

    // CTRL2_G: 416Hz ODR, +/-500dps
    // 0x64 = 0110 0100: ODR[7:4]=0110=416Hz, FS_G[3:2]=01=+/-500dps
    if (!i2c_write_reg(lsm6_addr, LSM6_CTRL2_G, 0x64)) {
        SEGGER_RTT_printf(0, "ERROR: LSM6DSOX CTRL2_G failed\r\n");
        return false;
    }

    // CTRL3_C: BDU=1, IF_INC=1
    // 0x44 = 0100 0100: BDU(bit6)=1 prevents split-sample reads,
    //                   IF_INC(bit2)=1 enables register auto-increment for burst reads
    if (!i2c_write_reg(lsm6_addr, LSM6_CTRL3_C, 0x44)) {
        SEGGER_RTT_printf(0, "ERROR: LSM6DSOX CTRL3_C failed\r\n");
        return false;
    }
    SEGGER_RTT_printf(0, "LSM6DSOX: 416Hz, +/-16g, +/-500dps, BDU enabled\r\n");

    // ---- LIS3MDL configuration ----

    // CTRL_REG1: ultra-high perf XY, 155Hz (FAST_ODR enabled), temp enabled
    // 0xFE = 1111 1110: TEMP_EN=1, OM=11 (UHP), DO=111, FAST_ODR=1 (bit1), ST=00
    // FAST_ODR=1 with OM=11 (UHP) enables 155Hz. Without FAST_ODR (0xFC), ODR is 80Hz.
    if (!i2c_write_reg(lis3_addr, LIS3_CTRL_REG1, 0xFE)) {
        SEGGER_RTT_printf(0, "ERROR: LIS3MDL CTRL_REG1 failed\r\n");
        return false;
    }
    // CTRL_REG2: +/-4 gauss
    if (!i2c_write_reg(lis3_addr, LIS3_CTRL_REG2, 0x00)) {
        SEGGER_RTT_printf(0, "ERROR: LIS3MDL CTRL_REG2 failed\r\n");
        return false;
    }
    // CTRL_REG3: continuous conversion
    if (!i2c_write_reg(lis3_addr, LIS3_CTRL_REG3, 0x00)) {
        SEGGER_RTT_printf(0, "ERROR: LIS3MDL CTRL_REG3 failed\r\n");
        return false;
    }
    // CTRL_REG4: ultra-high perf Z, little-endian
    if (!i2c_write_reg(lis3_addr, LIS3_CTRL_REG4, 0x0C)) {
        SEGGER_RTT_printf(0, "ERROR: LIS3MDL CTRL_REG4 failed\r\n");
        return false;
    }
    SEGGER_RTT_printf(0, "LIS3MDL: 155Hz (FAST_ODR), +/-4 gauss, continuous\r\n");

    // ---- H3LIS331 configuration (optional) ----

    if (h3lis_addr != 0) {
        // CTRL_REG1: normal mode, 400Hz, all axes enabled
        if (!i2c_write_reg(h3lis_addr, H3LIS_CTRL_REG1, 0x37)) {
            SEGGER_RTT_printf(0, "WARNING: H3LIS331 CTRL_REG1 failed\r\n");
            h3lis_addr = 0;
            runtime_sensor_status &= ~SENSOR_H3LIS_OK;
        }
    }
    if (h3lis_addr != 0) {
        // CTRL_REG4: BDU=1, +/-400g, little-endian
        // 0xB0 = 1011 0000: BDU(bit7)=1, FS[5:4]=11=+/-400g
        if (!i2c_write_reg(h3lis_addr, H3LIS_CTRL_REG4, 0xB0)) {
            SEGGER_RTT_printf(0, "WARNING: H3LIS331 CTRL_REG4 failed\r\n");
            h3lis_addr = 0;
            runtime_sensor_status &= ~SENSOR_H3LIS_OK;
        }
    }
    if (h3lis_addr != 0) {
        SEGGER_RTT_printf(0, "H3LIS331: +/-400g, 400Hz, BDU enabled\r\n");
    }

    // ---- BMP581 configuration (optional) ----

    if (bmp_addr != 0) {
        if (!init_bmp581()) {
            SEGGER_RTT_printf(0, "WARNING: BMP581 init failed — continuing without pressure\r\n");
            bmp_addr = 0;
            runtime_sensor_status &= ~SENSOR_BMP_OK;
        }
    }

    // ---- FSR channel configuration ----
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

    if (!twi_initialized || lsm6_addr == 0) {
        memset(data, 0, sizeof(sensor_data_t));
        return;
    }

    // LSM6DSOX gyro (auto-increment, no special flag needed on LSM6DSOX)
    if (i2c_read_regs(lsm6_addr, LSM6_OUTX_L_G, raw, 6)) {
        data->imu.gyro[0] = (int16_t)(raw[0] | (raw[1] << 8));
        data->imu.gyro[1] = (int16_t)(raw[2] | (raw[3] << 8));
        data->imu.gyro[2] = (int16_t)(raw[4] | (raw[5] << 8));
        runtime_sensor_status |= SENSOR_LSM6_OK;
    } else {
        memset(data->imu.gyro, 0, sizeof(data->imu.gyro));
        runtime_sensor_status &= ~SENSOR_LSM6_OK;
    }

    // LSM6DSOX accel
    if (i2c_read_regs(lsm6_addr, LSM6_OUTX_L_A, raw, 6)) {
        data->imu.accel[0] = (int16_t)(raw[0] | (raw[1] << 8));
        data->imu.accel[1] = (int16_t)(raw[2] | (raw[3] << 8));
        data->imu.accel[2] = (int16_t)(raw[4] | (raw[5] << 8));
    } else {
        memset(data->imu.accel, 0, sizeof(data->imu.accel));
        runtime_sensor_status &= ~SENSOR_LSM6_OK;
    }

    // LIS3MDL magnetometer
    // IMPORTANT: | 0x80 sets the MSB of the sub-address, enabling register
    // auto-increment for multi-byte I2C reads. Without it, LIS3MDL reads the
    // same register (X-low) 6 times. LSM6DSOX does not require this flag.
    if (lis3_addr != 0) {
        if (i2c_read_regs(lis3_addr, LIS3_OUT_X_L | 0x80, raw, 6)) {
            data->imu.mag[0] = (int16_t)(raw[0] | (raw[1] << 8));
            data->imu.mag[1] = (int16_t)(raw[2] | (raw[3] << 8));
            data->imu.mag[2] = (int16_t)(raw[4] | (raw[5] << 8));
            runtime_sensor_status |= SENSOR_LIS3_OK;
        } else {
            memset(data->imu.mag, 0, sizeof(data->imu.mag));
            runtime_sensor_status &= ~SENSOR_LIS3_OK;
        }
    } else {
        memset(data->imu.mag, 0, sizeof(data->imu.mag));
    }

    // H3LIS331 + FSR encoding
    // | 0x80 required for auto-increment on H3LIS331 (same as LIS3MDL)
    if (h3lis_addr != 0) {
        bool h3lis_ok = i2c_read_regs(h3lis_addr, H3LIS_OUT_X_L | 0x80, raw, 6);

        if (!h3lis_ok) {
            // Fallback: single-byte reads (slower, avoids bus issues)
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

        if (h3lis_ok) {
            h3lis_raw[0] = (int16_t)(raw[0] | (raw[1] << 8));
            h3lis_raw[1] = (int16_t)(raw[2] | (raw[3] << 8));
            h3lis_raw[2] = (int16_t)(raw[4] | (raw[5] << 8));

            fsr_read(fsr_raw);

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

            // Explicit mask before OR: do not rely on sensor hardware guaranteeing
            // zeros in bits [3:0]. The mask makes the packing intention explicit.
            data->h3lis_x_fsr_level   = (h3lis_raw[0] & (int16_t)0xFFF0) | (int16_t)fsr_intensity;
            data->h3lis_y_fsr_pattern = (h3lis_raw[1] & (int16_t)0xFFF0) | (int16_t)fsr_pattern;
            data->h3lis_z_flags       = (h3lis_raw[2] & (int16_t)0xFFF0) | 0;
            runtime_sensor_status |= SENSOR_H3LIS_OK;
        } else {
            memset(&data->h3lis_x_fsr_level, 0, 6);
            runtime_sensor_status &= ~SENSOR_H3LIS_OK;
        }
    } else {
        memset(&data->h3lis_x_fsr_level, 0, 6);
    }

    // BMP581 pressure (raw is in 1/64 Pa units; Pa = raw/64)
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
        if (!i2c_read_reg(h3lis_addr, H3LIS_WHO_AM_I, &id) || id != H3LIS331_ID)
            SEGGER_RTT_printf(0, "H3LIS331 test FAILED\r\n");
    }
    if (bmp_addr != 0) {
        if (!i2c_read_reg(bmp_addr, BMP_CHIP_ID, &id) ||
            (id != BMP581_ID && id != BMP581_ID_ALT))
            SEGGER_RTT_printf(0, "BMP581 test FAILED\r\n");
    }
    return ok;
}

uint8_t  sensors_get_status_bitmask(void)   { return runtime_sensor_status; }
uint16_t sensors_get_i2c_error_count(void)  { return i2c_error_count; }

int16_t sensors_read_temperature(void)
{
    // Return sentinel if BMP581 is absent or read fails.
    // SENSORS_TEMP_UNAVAILABLE (-32768) = -327.68 C, physically impossible.
    // Callers must check for this value before using the result.
    if (bmp_addr == 0) return SENSORS_TEMP_UNAVAILABLE;

    uint8_t t[3];
    if (!i2c_read_regs(bmp_addr, BMP_TEMP_XLSB, t, 3))
        return SENSORS_TEMP_UNAVAILABLE;

    int32_t raw = (int32_t)(t[0] | (t[1] << 8) | (t[2] << 16));
    if (raw & 0x800000) raw |= (int32_t)0xFF000000; // sign-extend 24->32 bit

    // BMP581: 1/65536 C per LSB -> 0.01 C units
    return (int16_t)((raw * 100) / 65536);
}
