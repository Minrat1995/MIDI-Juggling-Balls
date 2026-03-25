/**
 * Sensors Module Implementation - WITH I2C HANG DIAGNOSTICS
 * 
 * All sensors integrated with emergency debug output to identify I2C hangs
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

#define I2C_SCL_PIN     11
#define I2C_SDA_PIN     12

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

// FSR ADC pins - using channels 1-4 (battery uses channel 0)
#define FSR0_ADC_PIN    NRF_SAADC_INPUT_AIN1  // P0.03 - Channel 1
#define FSR1_ADC_PIN    NRF_SAADC_INPUT_AIN2  // P0.04 - Channel 2
#define FSR2_ADC_PIN    NRF_SAADC_INPUT_AIN3  // P0.05 - Channel 3
#define FSR3_ADC_PIN    NRF_SAADC_INPUT_AIN4  // P0.28 - Channel 4

#define FSR_CONTACT_THRESHOLD   80

// ============================================================================
// LSM6DSOX REGISTERS
// ============================================================================

#define LSM6_WHO_AM_I   0x0F
#define LSM6_CTRL1_XL   0x10
#define LSM6_CTRL2_G    0x11
#define LSM6_OUTX_L_G   0x22
#define LSM6_OUTX_L_A   0x28
#define LSM6_OUT_TEMP_L 0x20
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
#define BMP_PRESS_LSB   0x21
#define BMP_PRESS_MSB   0x22
#define BMP_TEMP_XLSB   0x1D
#define BMP_TEMP_LSB    0x1E
#define BMP_TEMP_MSB    0x1F
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

static uint8_t lsm6_addr = 0;
static uint8_t lis3_addr = 0;
static uint8_t h3lis_addr = 0;
static uint8_t bmp_addr = 0;

static uint8_t runtime_sensor_status = 0x0F;
static uint16_t i2c_error_count = 0;
static bool fsr_initialized = false;

// ============================================================================
// LOW-LEVEL I2C FUNCTIONS
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
    if (err != NRF_SUCCESS) {
        i2c_error_count++;
        return false;
    }
    
    err = nrf_drv_twi_rx(&m_twi, addr, data, len);
    if (err != NRF_SUCCESS) {
        i2c_error_count++;
        return false;
    }
    return true;
}

static bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *value)
{
    return i2c_read_regs(addr, reg, value, 1);
}

// ============================================================================
// FSR FUNCTIONS
// ============================================================================

static void fsr_init(void)
{
    // NOTE: Battery monitoring uses SAADC channel 0 (configured in main.c)
    // FSRs use channels 1-4 to avoid conflict
    
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_10bit;
    
    // Configure FSR channels 1-4 (channel 0 already used by battery)
    for (int ch = 1; ch <= 4; ch++) {
        NRF_SAADC->CH[ch].CONFIG = 
            (SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos) |
            (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |
            (SAADC_CH_CONFIG_GAIN_Gain1_6 << SAADC_CH_CONFIG_GAIN_Pos) |
            (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
            (SAADC_CH_CONFIG_TACQ_10us << SAADC_CH_CONFIG_TACQ_Pos) |
            (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos);
        NRF_SAADC->CH[ch].PSELN = SAADC_CH_PSELN_PSELN_NC;
    }
    
    // Map FSR pins to channels 1-4
    NRF_SAADC->CH[1].PSELP = FSR0_ADC_PIN;
    NRF_SAADC->CH[2].PSELP = FSR1_ADC_PIN;
    NRF_SAADC->CH[3].PSELP = FSR2_ADC_PIN;
    NRF_SAADC->CH[4].PSELP = FSR3_ADC_PIN;
    
    // Note: SAADC already enabled by battery_init() in main.c
    fsr_initialized = true;
    
    SEGGER_RTT_printf(0, "FSR ADC initialized (channels 1-4, avoiding battery on ch0)\r\n");
}

static bool fsr_read(uint16_t *fsr_out)
{
    // DISABLED: FSR sensors not yet wired
    // When ready, uncomment the code below and wire to:
    //   FSR0 → P0.03 (Channel 1)
    //   FSR1 → P0.04 (Channel 2)
    //   FSR2 → P0.05 (Channel 3)
    //   FSR3 → P0.28 (Channel 4)
    memset(fsr_out, 0, 4 * sizeof(uint16_t));
    return false;
    
    /* UNCOMMENT WHEN FSRs ARE WIRED:
    
    if (!fsr_initialized) {
        memset(fsr_out, 0, 4 * sizeof(uint16_t));
        return false;
    }
    
    // Read FSRs from channels 1-4 (battery uses channel 0)
    int16_t adc_values[4];
    
    NRF_SAADC->RESULT.PTR = (uint32_t)adc_values;
    NRF_SAADC->RESULT.MAXCNT = 4;
    
    NRF_SAADC->TASKS_START = 1;
    
    // Timeout protection (10ms max wait)
    uint32_t timeout = 100000;
    while (NRF_SAADC->EVENTS_STARTED == 0 && timeout > 0) {
        timeout--;
    }
    if (timeout == 0) {
        memset(fsr_out, 0, 4 * sizeof(uint16_t));
        return false;
    }
    NRF_SAADC->EVENTS_STARTED = 0;
    
    NRF_SAADC->TASKS_SAMPLE = 1;
    
    timeout = 100000;
    while (NRF_SAADC->EVENTS_END == 0 && timeout > 0) {
        timeout--;
    }
    if (timeout == 0) {
        memset(fsr_out, 0, 4 * sizeof(uint16_t));
        return false;
    }
    NRF_SAADC->EVENTS_END = 0;
    
    NRF_SAADC->TASKS_STOP = 1;
    
    timeout = 100000;
    while (NRF_SAADC->EVENTS_STOPPED == 0 && timeout > 0) {
        timeout--;
    }
    if (timeout == 0) {
        memset(fsr_out, 0, 4 * sizeof(uint16_t));
        return false;
    }
    NRF_SAADC->EVENTS_STOPPED = 0;
    
    // Convert and clamp values
    for (int i = 0; i < 4; i++) {
        if (adc_values[i] < 0) {
            fsr_out[i] = 0;
        } else if (adc_values[i] > 1023) {
            fsr_out[i] = 1023;
        } else {
            fsr_out[i] = (uint16_t)adc_values[i];
        }
    }
    
    return true;
    
    END OF COMMENTED CODE */
}

// ============================================================================
// SENSOR DETECTION
// ============================================================================

static uint8_t detect_lsm6dsox(void)
{
    uint8_t who_am_i;
    
    if (i2c_read_reg(LSM6DSOX_ADDR_1, LSM6_WHO_AM_I, &who_am_i)) {
        if (who_am_i == LSM6DSOX_ID) return LSM6DSOX_ADDR_1;
    }
    
    if (i2c_read_reg(LSM6DSOX_ADDR_2, LSM6_WHO_AM_I, &who_am_i)) {
        if (who_am_i == LSM6DSOX_ID) return LSM6DSOX_ADDR_2;
    }
    
    return 0;
}

static uint8_t detect_lis3mdl(void)
{
    uint8_t who_am_i;
    
    if (i2c_read_reg(LIS3MDL_ADDR_1, LIS3_WHO_AM_I, &who_am_i)) {
        if (who_am_i == LIS3MDL_ID) return LIS3MDL_ADDR_1;
    }
    
    if (i2c_read_reg(LIS3MDL_ADDR_2, LIS3_WHO_AM_I, &who_am_i)) {
        if (who_am_i == LIS3MDL_ID) return LIS3MDL_ADDR_2;
    }
    
    return 0;
}

static uint8_t detect_h3lis331(void)
{
    uint8_t who_am_i;
    
    if (i2c_read_reg(H3LIS331_ADDR, H3LIS_WHO_AM_I, &who_am_i)) {
        if (who_am_i == H3LIS331_ID) return H3LIS331_ADDR;
        SEGGER_RTT_printf(0, "Device at 0x18 has WHO_AM_I=0x%02X (expected 0x32)\r\n", who_am_i);
    }
    
    return 0;
}

static uint8_t detect_bmp581(void)
{
    uint8_t chip_id;
    
    if (i2c_read_reg(BMP581_ADDR_1, BMP_CHIP_ID, &chip_id)) {
        if (chip_id == BMP581_ID || chip_id == BMP581_ID_ALT) {
            SEGGER_RTT_printf(0, "BMP581 found at 0x%02X (CHIP_ID=0x%02X)\r\n", 
                BMP581_ADDR_1, chip_id);
            return BMP581_ADDR_1;
        }
    }
    
    if (i2c_read_reg(BMP581_ADDR_2, BMP_CHIP_ID, &chip_id)) {
        if (chip_id == BMP581_ID || chip_id == BMP581_ID_ALT) {
            SEGGER_RTT_printf(0, "BMP581 found at 0x%02X (CHIP_ID=0x%02X)\r\n", 
                BMP581_ADDR_2, chip_id);
            return BMP581_ADDR_2;
        }
    }
    
    return 0;
}

// ============================================================================
// BMP581 INITIALIZATION
// ============================================================================

static bool init_bmp581(void)
{
    uint8_t status, reg_val;
    int retry;
    
    SEGGER_RTT_printf(0, "\r\n=== BMP581 Initialization (Enhanced Diagnostics) ===\r\n");
    
    SEGGER_RTT_printf(0, "Step 1: Soft reset...\r\n");
    if (!i2c_write_reg(bmp_addr, BMP_CMD, BMP_CMD_SOFT_RESET)) {
        SEGGER_RTT_printf(0, "  FAILED: Could not write reset command\r\n");
        return false;
    }
    nrf_delay_ms(50);
    
    uint8_t chip_id;
    if (!i2c_read_reg(bmp_addr, BMP_CHIP_ID, &chip_id)) {
        SEGGER_RTT_printf(0, "  FAILED: Device not responding after reset\r\n");
        return false;
    }
    SEGGER_RTT_printf(0, "  Device responsive (CHIP_ID=0x%02X)\r\n", chip_id);
    
    uint8_t regs[3];
    SEGGER_RTT_printf(0, "Step 2: Initial register dump:\r\n");
    if (i2c_read_reg(bmp_addr, 0x36, &regs[0])) {
        SEGGER_RTT_printf(0, "  0x36 (OSR_CONFIG) = 0x%02X\r\n", regs[0]);
    }
    if (i2c_read_reg(bmp_addr, 0x37, &regs[1])) {
        SEGGER_RTT_printf(0, "  0x37 (ODR_CONFIG) = 0x%02X\r\n", regs[1]);
    }
    if (i2c_read_reg(bmp_addr, 0x28, &regs[2])) {
        SEGGER_RTT_printf(0, "  0x28 (STATUS) = 0x%02X\r\n", regs[2]);
    }
    
    if (i2c_read_reg(bmp_addr, BMP_STATUS, &status)) {
        SEGGER_RTT_printf(0, "Step 3: STATUS register analysis:\r\n");
        SEGGER_RTT_printf(0, "  Raw value: 0x%02X\r\n", status);
        SEGGER_RTT_printf(0, "  Bit 0 (nvm_rdy): %s\r\n", (status & 0x01) ? "BUSY" : "ready");
        SEGGER_RTT_printf(0, "  Bit 1 (nvm_err): %s\r\n", (status & 0x02) ? "ERROR!" : "ok");
        SEGGER_RTT_printf(0, "  Bit 5 (drdy_press): %s\r\n", (status & 0x20) ? "ready" : "not ready");
        
        if (status & 0x02) {
            SEGGER_RTT_printf(0, "  *** NVM ERROR DETECTED! ***\r\n");
            SEGGER_RTT_printf(0, "  This indicates calibration data failed to load.\r\n");
            SEGGER_RTT_printf(0, "  Sensor may be defective or need full Bosch API.\r\n");
        }
    }
    
    SEGGER_RTT_printf(0, "Step 4: Configure OSR=0x12...\r\n");
    if (!i2c_write_reg(bmp_addr, 0x36, 0x12)) {
        SEGGER_RTT_printf(0, "  FAILED\r\n");
        return false;
    }
    
    if (i2c_read_reg(bmp_addr, 0x36, &reg_val)) {
        SEGGER_RTT_printf(0, "  OSR_CONFIG readback = 0x%02X\r\n", reg_val);
    }
    nrf_delay_ms(10);
    
    SEGGER_RTT_printf(0, "Step 5: Setting NORMAL mode (ODR=200Hz)...\r\n");
    if (!i2c_write_reg(bmp_addr, 0x37, 0x11)) {
        SEGGER_RTT_printf(0, "  FAILED\r\n");
        return false;
    }
    
    if (i2c_read_reg(bmp_addr, 0x37, &reg_val)) {
        SEGGER_RTT_printf(0, "  ODR_CONFIG readback = 0x%02X (expect 0x11)\r\n", reg_val);
    }
    nrf_delay_ms(100);
    
    SEGGER_RTT_printf(0, "Step 6: Waiting for data ready...\r\n");
    retry = 50;
    bool data_ready = false;
    
    while (retry > 0) {
        if (i2c_read_reg(bmp_addr, BMP_STATUS, &status)) {
            if (status & 0x20) {
                SEGGER_RTT_printf(0, "  DATA READY (STATUS=0x%02X)\r\n", status);
                data_ready = true;
                break;
            }
        }
        nrf_delay_ms(10);
        retry--;
    }
    
    if (!data_ready) {
        SEGGER_RTT_printf(0, "  WARNING: No data ready signal (STATUS=0x%02X)\r\n", status);
    }
    
    SEGGER_RTT_printf(0, "Step 7: Reading pressure data (5 attempts)...\r\n");
    uint8_t press_data[3];
    
    for (int attempt = 0; attempt < 5; attempt++) {
        if (i2c_read_regs(bmp_addr, BMP_PRESS_XLSB, press_data, 3)) {
            uint32_t pressure = press_data[0] | (press_data[1] << 8) | (press_data[2] << 16);
            SEGGER_RTT_printf(0, "  Attempt %d: 0x%02X%02X%02X = %u Pa\r\n",
                attempt + 1, press_data[2], press_data[1], press_data[0], pressure);
            
            if (pressure != 0x7F7F7F && pressure >= 30000 && pressure <= 110000) {
                SEGGER_RTT_printf(0, "=== BMP581 Init SUCCESS ===\r\n\r\n");
                return true;
            }
        }
        nrf_delay_ms(50);
    }
    
    SEGGER_RTT_printf(0, "\r\n=== BMP581 Init FAILED ===\r\n");
    SEGGER_RTT_printf(0, "All attempts returned 0x7F7F7F (invalid data marker)\r\n");
    SEGGER_RTT_printf(0, "\r\nPossible causes:\r\n");
    SEGGER_RTT_printf(0, "  1. NVM error (bit 1 of STATUS = 1)\r\n");
    SEGGER_RTT_printf(0, "  2. Sensor not entering NORMAL mode\r\n");
    SEGGER_RTT_printf(0, "  3. Hardware issue (check wiring/power)\r\n");
    SEGGER_RTT_printf(0, "  4. May need full Bosch BMP5 API for initialization\r\n");
    SEGGER_RTT_printf(0, "\r\nSuggested next steps:\r\n");
    SEGGER_RTT_printf(0, "  - Check if green LED on BMP581 breakout is lit\r\n");
    SEGGER_RTT_printf(0, "  - Verify I2C pullups present (2.2k-4.7k to 3.3V)\r\n");
    SEGGER_RTT_printf(0, "  - Try swapping BMP581 module (may be defective)\r\n");
    SEGGER_RTT_printf(0, "  - Consider using BMP390 instead (simpler init)\r\n\r\n");
    
    return false;
}

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

bool sensors_init(void)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_config = {
        .scl                = I2C_SCL_PIN,
        .sda                = I2C_SDA_PIN,
        .frequency          = NRF_DRV_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init     = false
    };
    
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    if (err_code != NRF_SUCCESS) {
        SEGGER_RTT_printf(0, "ERROR: I2C initialization failed\r\n");
        return false;
    }
    
    nrf_drv_twi_enable(&m_twi);
    twi_initialized = true;
    nrf_delay_ms(20);
    
    lsm6_addr = detect_lsm6dsox();
    if (lsm6_addr == 0) {
        SEGGER_RTT_printf(0, "ERROR: LSM6DSOX not found\r\n");
        runtime_sensor_status &= ~SENSOR_LSM6_OK;
        return false;
    }
    SEGGER_RTT_printf(0, "Found LSM6DSOX at 0x%02X\r\n", lsm6_addr);
    runtime_sensor_status |= SENSOR_LSM6_OK;
    
    lis3_addr = detect_lis3mdl();
    if (lis3_addr == 0) {
        SEGGER_RTT_printf(0, "ERROR: LIS3MDL not found\r\n");
        runtime_sensor_status &= ~SENSOR_LIS3_OK;
        return false;
    }
    SEGGER_RTT_printf(0, "Found LIS3MDL at 0x%02X\r\n", lis3_addr);
    runtime_sensor_status |= SENSOR_LIS3_OK;
    
    h3lis_addr = detect_h3lis331();
    if (h3lis_addr == 0) {
        SEGGER_RTT_printf(0, "WARNING: H3LIS331 not found\r\n");
        runtime_sensor_status &= ~SENSOR_H3LIS_OK;
    } else {
        SEGGER_RTT_printf(0, "Found H3LIS331 at 0x%02X\r\n", h3lis_addr);
        runtime_sensor_status |= SENSOR_H3LIS_OK;
    }
    
    bmp_addr = detect_bmp581();
    if (bmp_addr == 0) {
        SEGGER_RTT_printf(0, "WARNING: BMP581 not found\r\n");
        runtime_sensor_status &= ~SENSOR_BMP_OK;
    } else {
        runtime_sensor_status |= SENSOR_BMP_OK;
    }
    
    if (!i2c_write_reg(lsm6_addr, LSM6_CTRL1_XL, 0x66)) {
        SEGGER_RTT_printf(0, "ERROR: LSM6DSOX accel config failed\r\n");
        return false;
    }
    
    if (!i2c_write_reg(lsm6_addr, LSM6_CTRL2_G, 0x68)) {
        SEGGER_RTT_printf(0, "ERROR: LSM6DSOX gyro config failed\r\n");
        return false;
    }
    
    if (!i2c_write_reg(lis3_addr, LIS3_CTRL_REG1, 0xFC)) {
        SEGGER_RTT_printf(0, "ERROR: LIS3MDL config failed\r\n");
        return false;
    }
    
    if (!i2c_write_reg(lis3_addr, LIS3_CTRL_REG2, 0x00)) {
        SEGGER_RTT_printf(0, "ERROR: LIS3MDL scale config failed\r\n");
        return false;
    }
    
    if (!i2c_write_reg(lis3_addr, LIS3_CTRL_REG3, 0x00)) {
        SEGGER_RTT_printf(0, "ERROR: LIS3MDL mode config failed\r\n");
        return false;
    }
    
    if (!i2c_write_reg(lis3_addr, LIS3_CTRL_REG4, 0x0C)) {
        SEGGER_RTT_printf(0, "ERROR: LIS3MDL Z-axis config failed\r\n");
        return false;
    }
    
    if (h3lis_addr != 0) {
        if (!i2c_write_reg(h3lis_addr, H3LIS_CTRL_REG1, 0x37)) {
            SEGGER_RTT_printf(0, "WARNING: H3LIS331 CTRL_REG1 config failed\r\n");
            h3lis_addr = 0;
            runtime_sensor_status &= ~SENSOR_H3LIS_OK;
        }
        
        if (h3lis_addr != 0 && !i2c_write_reg(h3lis_addr, H3LIS_CTRL_REG4, 0xB0)) {
            SEGGER_RTT_printf(0, "WARNING: H3LIS331 CTRL_REG4 config failed\r\n");
            h3lis_addr = 0;
            runtime_sensor_status &= ~SENSOR_H3LIS_OK;
        }
        
        if (h3lis_addr != 0) {
            SEGGER_RTT_printf(0, "H3LIS331 configured: ±400g, 400Hz, BDU enabled\r\n");
        }
    }
    
    if (bmp_addr != 0) {
        if (!init_bmp581()) {
            SEGGER_RTT_printf(0, "WARNING: BMP581 initialization failed\r\n");
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
    if (lsm6) *lsm6 = lsm6_addr;
    if (lis3) *lis3 = lis3_addr;
    if (h3lis) *h3lis = h3lis_addr;
    if (bmp) *bmp = bmp_addr;
}

void sensors_read(sensor_data_t *data)
{
    uint8_t raw_data[6];
    int16_t h3lis_raw[3];
    uint16_t fsr_raw[4];
    
    if (!twi_initialized || lsm6_addr == 0) {
        memset(data, 0, sizeof(sensor_data_t));
        return;
    }
    
    // Read LSM6DSOX Gyroscope
    if (i2c_read_regs(lsm6_addr, LSM6_OUTX_L_G, raw_data, 6)) {
        data->imu.gyro[0] = (int16_t)(raw_data[0] | (raw_data[1] << 8));
        data->imu.gyro[1] = (int16_t)(raw_data[2] | (raw_data[3] << 8));
        data->imu.gyro[2] = (int16_t)(raw_data[4] | (raw_data[5] << 8));
        runtime_sensor_status |= SENSOR_LSM6_OK;
    } else {
        memset(data->imu.gyro, 0, sizeof(data->imu.gyro));
        runtime_sensor_status &= ~SENSOR_LSM6_OK;
    }
    
    // Read LSM6DSOX Accelerometer
    if (i2c_read_regs(lsm6_addr, LSM6_OUTX_L_A, raw_data, 6)) {
        data->imu.accel[0] = (int16_t)(raw_data[0] | (raw_data[1] << 8));
        data->imu.accel[1] = (int16_t)(raw_data[2] | (raw_data[3] << 8));
        data->imu.accel[2] = (int16_t)(raw_data[4] | (raw_data[5] << 8));
    } else {
        memset(data->imu.accel, 0, sizeof(data->imu.accel));
        runtime_sensor_status &= ~SENSOR_LSM6_OK;
    }
    
    // Read LIS3MDL Magnetometer
    if (lis3_addr != 0) {
        if (i2c_read_regs(lis3_addr, LIS3_OUT_X_L, raw_data, 6)) {
            data->imu.mag[0] = (int16_t)(raw_data[0] | (raw_data[1] << 8));
            data->imu.mag[1] = (int16_t)(raw_data[2] | (raw_data[3] << 8));
            data->imu.mag[2] = (int16_t)(raw_data[4] | (raw_data[5] << 8));
            runtime_sensor_status |= SENSOR_LIS3_OK;
        } else {
            memset(data->imu.mag, 0, sizeof(data->imu.mag));
            runtime_sensor_status &= ~SENSOR_LIS3_OK;
        }
    } else {
        memset(data->imu.mag, 0, sizeof(data->imu.mag));
    }
    
    // Read H3LIS331 + Encode FSR Data
    if (h3lis_addr != 0) {
        // Try multi-byte read with auto-increment
        uint8_t reg_addr = H3LIS_OUT_X_L | 0x80;
        
        if (i2c_read_regs(h3lis_addr, reg_addr, raw_data, 6)) {
            h3lis_raw[0] = (int16_t)(raw_data[0] | (raw_data[1] << 8));
            h3lis_raw[1] = (int16_t)(raw_data[2] | (raw_data[3] << 8));
            h3lis_raw[2] = (int16_t)(raw_data[4] | (raw_data[5] << 8));
            
            // Read FSR sensors (currently disabled, returns zeros)
            fsr_read(fsr_raw);
            
            // Find maximum FSR value
            uint16_t max_fsr = 0;
            for (int i = 0; i < 4; i++) {
                if (fsr_raw[i] > max_fsr) {
                    max_fsr = fsr_raw[i];
                }
            }
            
            // Scale to 4-bit intensity (0-15)
            uint8_t fsr_intensity = (max_fsr >> 6) & 0x0F;
            
            // Generate contact pattern
            uint8_t fsr_pattern = 0;
            if (fsr_raw[0] > FSR_CONTACT_THRESHOLD) fsr_pattern |= 0x01;
            if (fsr_raw[1] > FSR_CONTACT_THRESHOLD) fsr_pattern |= 0x02;
            if (fsr_raw[2] > FSR_CONTACT_THRESHOLD) fsr_pattern |= 0x04;
            if (fsr_raw[3] > FSR_CONTACT_THRESHOLD) fsr_pattern |= 0x08;
            
            // Flags (reserved)
            uint8_t flags = 0;
            
            // Pack into H3LIS data
            data->h3lis_x_fsr_level = (h3lis_raw[0] & 0xFFF0) | fsr_intensity;
            data->h3lis_y_fsr_pattern = (h3lis_raw[1] & 0xFFF0) | fsr_pattern;
            data->h3lis_z_flags = (h3lis_raw[2] & 0xFFF0) | flags;
            
            runtime_sensor_status |= SENSOR_H3LIS_OK;
        } else {
            // Multi-byte read failed - try single-byte reads as fallback
            bool success = true;
            for (int axis = 0; axis < 3; axis++) {
                uint8_t low_byte, high_byte;
                uint8_t base_reg = H3LIS_OUT_X_L + (axis * 2);
                
                if (!i2c_read_reg(h3lis_addr, base_reg, &low_byte) ||
                    !i2c_read_reg(h3lis_addr, base_reg + 1, &high_byte)) {
                    success = false;
                    break;
                }
                h3lis_raw[axis] = (int16_t)(low_byte | (high_byte << 8));
            }
            
            if (success) {
                // Read FSR and encode (same as above)
                fsr_read(fsr_raw);
                
                uint16_t max_fsr = 0;
                for (int i = 0; i < 4; i++) {
                    if (fsr_raw[i] > max_fsr) max_fsr = fsr_raw[i];
                }
                
                uint8_t fsr_intensity = (max_fsr >> 6) & 0x0F;
                uint8_t fsr_pattern = 0;
                if (fsr_raw[0] > FSR_CONTACT_THRESHOLD) fsr_pattern |= 0x01;
                if (fsr_raw[1] > FSR_CONTACT_THRESHOLD) fsr_pattern |= 0x02;
                if (fsr_raw[2] > FSR_CONTACT_THRESHOLD) fsr_pattern |= 0x04;
                if (fsr_raw[3] > FSR_CONTACT_THRESHOLD) fsr_pattern |= 0x08;
                
                data->h3lis_x_fsr_level = (h3lis_raw[0] & 0xFFF0) | fsr_intensity;
                data->h3lis_y_fsr_pattern = (h3lis_raw[1] & 0xFFF0) | fsr_pattern;
                data->h3lis_z_flags = (h3lis_raw[2] & 0xFFF0) | 0;
                
                runtime_sensor_status |= SENSOR_H3LIS_OK;
            } else {
                memset(&data->h3lis_x_fsr_level, 0, 6);
                runtime_sensor_status &= ~SENSOR_H3LIS_OK;
            }
        }
    } else {
        memset(&data->h3lis_x_fsr_level, 0, 6);
    }
    
    // Read BMP581 Pressure
    if (bmp_addr != 0) {
        uint8_t press_data[3];
        if (i2c_read_regs(bmp_addr, BMP_PRESS_XLSB, press_data, 3)) {
            data->pressure[0] = press_data[0];
            data->pressure[1] = press_data[1];
            data->pressure[2] = press_data[2];
            runtime_sensor_status |= SENSOR_BMP_OK;
        } else {
            memset(data->pressure, 0, sizeof(data->pressure));
            runtime_sensor_status &= ~SENSOR_BMP_OK;
        }
    } else {
        memset(data->pressure, 0, sizeof(data->pressure));
    }
}

bool sensors_test(void)
{
    uint8_t id_value;
    bool all_ok = true;
    
    if (!twi_initialized) {
        return false;
    }
    
    if (lsm6_addr != 0) {
        if (!i2c_read_reg(lsm6_addr, LSM6_WHO_AM_I, &id_value) || id_value != LSM6DSOX_ID) {
            SEGGER_RTT_printf(0, "LSM6DSOX test FAILED\r\n");
            all_ok = false;
        }
    } else {
        all_ok = false;
    }
    
    if (lis3_addr != 0) {
        if (!i2c_read_reg(lis3_addr, LIS3_WHO_AM_I, &id_value) || id_value != LIS3MDL_ID) {
            SEGGER_RTT_printf(0, "LIS3MDL test FAILED\r\n");
            all_ok = false;
        }
    } else {
        all_ok = false;
    }
    
    if (h3lis_addr != 0) {
        if (!i2c_read_reg(h3lis_addr, H3LIS_WHO_AM_I, &id_value) || id_value != H3LIS331_ID) {
            SEGGER_RTT_printf(0, "H3LIS331 test FAILED\r\n");
        }
    }
    
    if (bmp_addr != 0) {
        if (!i2c_read_reg(bmp_addr, BMP_CHIP_ID, &id_value) || 
            (id_value != BMP581_ID && id_value != BMP581_ID_ALT)) {
            SEGGER_RTT_printf(0, "BMP581 test FAILED\r\n");
        }
    }
    
    return all_ok;
}

uint8_t sensors_get_status_bitmask(void)
{
    return runtime_sensor_status;
}

uint16_t sensors_get_i2c_error_count(void)
{
    return i2c_error_count;
}

int16_t sensors_read_temperature(void)
{
    if (bmp_addr == 0) {
        return 0;
    }
    
    uint8_t temp_data[3];
    if (!i2c_read_regs(bmp_addr, BMP_TEMP_XLSB, temp_data, 3)) {
        return 0;
    }
    
    int32_t raw_temp = (int32_t)(temp_data[0] | (temp_data[1] << 8) | (temp_data[2] << 16));
    
    if (raw_temp & 0x800000) {
        raw_temp |= 0xFF000000;
    }
    
    int16_t temp_celsius_hundredths = (int16_t)((raw_temp * 100) / 65536);
    
    return temp_celsius_hundredths;
}