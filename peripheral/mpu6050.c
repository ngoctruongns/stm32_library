/**
 * @file  mpu6050.c
 * @brief MPU-6050 6-axis IMU driver (accelerometer + gyroscope).
 *
 * Implemented over STM32 LL I2C API (blocking, polling).
 *
 * Hardware connections (I2C3 on STM32F407):
 *   I2C3 SCL  → PA8  (AF4, open-drain)
 *   I2C3 SDA  → PC9  (AF4, open-drain)
 *   AD0       → GND  → I2C address = 0x68
 */

#include "mpu6050.h"
#include "main.h"
#include "log_helper.h"

// ---------------------------------------------------------------------------
// Register map
// ---------------------------------------------------------------------------
#define REG_SMPLRT_DIV      0x19U
#define REG_CONFIG          0x1AU   // DLPF_CFG
#define REG_GYRO_CONFIG     0x1BU   // FS_SEL
#define REG_ACCEL_CONFIG    0x1CU   // AFS_SEL
#define REG_ACCEL_XOUT_H    0x3BU   // first of 14 burst bytes
#define REG_TEMP_OUT_H      0x41U
#define REG_GYRO_XOUT_H     0x43U
#define REG_PWR_MGMT_1      0x6BU   // SLEEP, CLKSEL
#define REG_WHO_AM_I        0x75U

// PWR_MGMT_1 bits
#define PWR1_DEVICE_RESET   (1U << 7U)
#define PWR1_SLEEP          (1U << 6U)
#define PWR1_CLKSEL_GYRO_X  0x01U   // use PLL with X-axis gyro as clock source

// ACCEL_CONFIG AFS_SEL bits [4:3]
#define ACCEL_FS_SEL_SHIFT  3U

// GYRO_CONFIG FS_SEL bits [4:3]
#define GYRO_FS_SEL_SHIFT   3U

// ---------------------------------------------------------------------------
// Sensitivity constants
// ---------------------------------------------------------------------------
// LSB per g for each accelerometer full-scale range
static const float k_accel_sensitivity[4] = {
    16384.0f,   // ±2g
    8192.0f,    // ±4g
    4096.0f,    // ±8g
    2048.0f     // ±16g
};

// LSB per °/s for each gyroscope full-scale range
static const float k_gyro_sensitivity[4] = {
    131.0f,     // ±250 °/s
    65.5f,      // ±500 °/s
    32.8f,      // ±1000 °/s
    16.4f       // ±2000 °/s
};

#define I2C_TIMEOUT_MS  10U

// ---------------------------------------------------------------------------
// Low-level I2C helpers (LL API, blocking)
// ---------------------------------------------------------------------------

static bool i2c_wait_flag(I2C_TypeDef *i2c,
                          uint32_t     flag,
                          bool         expected_set)
{
    uint32_t t0 = get_ms_tick_count();
    while (((i2c->SR1 & flag) != 0U) != expected_set) {
        if ((get_ms_tick_count() - t0) >= I2C_TIMEOUT_MS) {
            return false;
        }
        if (LL_I2C_IsActiveFlag_BERR(i2c) || LL_I2C_IsActiveFlag_ARLO(i2c)) {
            LL_I2C_ClearFlag_BERR(i2c);
            LL_I2C_ClearFlag_ARLO(i2c);
            return false;
        }
    }
    return true;
}

/**
 * @brief  Write one byte to a register.
 */
static bool i2c_write_reg(I2C_TypeDef *i2c, uint8_t dev_addr, uint8_t reg, uint8_t value)
{
    // START condition
    LL_I2C_GenerateStartCondition(i2c);
    if (!i2c_wait_flag(i2c, I2C_SR1_SB, true)) { return false; }

    // Send device address + WRITE
    LL_I2C_TransmitData8(i2c, (uint8_t)((dev_addr << 1U) | 0U));
    if (!i2c_wait_flag(i2c, I2C_SR1_ADDR, true)) { return false; }
    (void)i2c->SR2;  // clear ADDR flag by reading SR2

    // Send register address
    if (!i2c_wait_flag(i2c, I2C_SR1_TXE, true)) { return false; }
    LL_I2C_TransmitData8(i2c, reg);

    // Send data byte
    if (!i2c_wait_flag(i2c, I2C_SR1_TXE, true)) { return false; }
    LL_I2C_TransmitData8(i2c, value);

    // Wait for BTF (both shift reg and DR empty) then STOP
    if (!i2c_wait_flag(i2c, I2C_SR1_BTF, true)) { return false; }
    LL_I2C_GenerateStopCondition(i2c);
    return true;
}

/**
 * @brief  Read one byte from a register.
 */
static bool i2c_read_reg(I2C_TypeDef *i2c, uint8_t dev_addr, uint8_t reg, uint8_t *value)
{
    // START + write register address
    LL_I2C_GenerateStartCondition(i2c);
    if (!i2c_wait_flag(i2c, I2C_SR1_SB, true)) { return false; }

    LL_I2C_TransmitData8(i2c, (uint8_t)((dev_addr << 1U) | 0U));
    if (!i2c_wait_flag(i2c, I2C_SR1_ADDR, true)) { return false; }
    (void)i2c->SR2;

    if (!i2c_wait_flag(i2c, I2C_SR1_TXE, true)) { return false; }
    LL_I2C_TransmitData8(i2c, reg);
    if (!i2c_wait_flag(i2c, I2C_SR1_BTF, true)) { return false; }

    // Repeated START + read
    LL_I2C_GenerateStartCondition(i2c);
    if (!i2c_wait_flag(i2c, I2C_SR1_SB, true)) { return false; }

    // Disable ACK before sending address so we NACK the single byte
    LL_I2C_AcknowledgeNextData(i2c, LL_I2C_NACK);

    LL_I2C_TransmitData8(i2c, (uint8_t)((dev_addr << 1U) | 1U));
    if (!i2c_wait_flag(i2c, I2C_SR1_ADDR, true)) { return false; }
    (void)i2c->SR2;

    LL_I2C_GenerateStopCondition(i2c);

    if (!i2c_wait_flag(i2c, I2C_SR1_RXNE, true)) { return false; }
    *value = LL_I2C_ReceiveData8(i2c);

    // Re-enable ACK for future transfers
    LL_I2C_AcknowledgeNextData(i2c, LL_I2C_ACK);
    return true;
}

/**
 * @brief  Burst-read N bytes starting at reg into buf.
 */
static bool i2c_read_burst(I2C_TypeDef *i2c,
                            uint8_t      dev_addr,
                            uint8_t      reg,
                            uint8_t     *buf,
                            uint8_t      len)
{
    if ((buf == NULL) || (len == 0U)) { return false; }

    // START + write register address
    LL_I2C_GenerateStartCondition(i2c);
    if (!i2c_wait_flag(i2c, I2C_SR1_SB, true)) { return false; }

    LL_I2C_TransmitData8(i2c, (uint8_t)((dev_addr << 1U) | 0U));
    if (!i2c_wait_flag(i2c, I2C_SR1_ADDR, true)) { return false; }
    (void)i2c->SR2;

    if (!i2c_wait_flag(i2c, I2C_SR1_TXE, true)) { return false; }
    LL_I2C_TransmitData8(i2c, reg);
    if (!i2c_wait_flag(i2c, I2C_SR1_BTF, true)) { return false; }

    // Repeated START
    LL_I2C_GenerateStartCondition(i2c);
    if (!i2c_wait_flag(i2c, I2C_SR1_SB, true)) { return false; }

    // Enable ACK before addressing in read mode
    LL_I2C_AcknowledgeNextData(i2c, LL_I2C_ACK);

    LL_I2C_TransmitData8(i2c, (uint8_t)((dev_addr << 1U) | 1U));
    if (!i2c_wait_flag(i2c, I2C_SR1_ADDR, true)) { return false; }
    (void)i2c->SR2;

    for (uint8_t i = 0U; i < len; i++) {
        if (i == (len - 1U)) {
            // Last byte: disable ACK and issue STOP before reading
            LL_I2C_AcknowledgeNextData(i2c, LL_I2C_NACK);
            LL_I2C_GenerateStopCondition(i2c);
        }
        if (!i2c_wait_flag(i2c, I2C_SR1_RXNE, true)) { return false; }
        buf[i] = LL_I2C_ReceiveData8(i2c);
    }

    // Re-enable ACK
    LL_I2C_AcknowledgeNextData(i2c, LL_I2C_ACK);
    return true;
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static inline int16_t combine_bytes(uint8_t hi, uint8_t lo)
{
    return (int16_t)((uint16_t)hi << 8U | (uint16_t)lo);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool MPU6050_Init(MPU6050_Handle  *hdev,
                  I2C_TypeDef     *i2c,
                  uint8_t          addr,
                  MPU6050_AccelFS  accel_fs,
                  MPU6050_GyroFS   gyro_fs,
                  MPU6050_DLPF     dlpf)
{
    hdev->i2c         = i2c;
    hdev->addr        = addr;
    hdev->accel_fs    = accel_fs;
    hdev->gyro_fs     = gyro_fs;
    hdev->initialized = false;

    // Short settling delay after power-on
    LL_mDelay(100U);

    // Verify device identity
    uint8_t who_am_i = 0U;
    if (!i2c_read_reg(i2c, addr, REG_WHO_AM_I, &who_am_i)) {
        LOG_ERR("MPU6050: WHO_AM_I read failed (I2C timeout / no ACK)");
        return false;
    }
    LOG_DBG("MPU6050: WHO_AM_I=0x%02X (exp 0x%02X)", who_am_i, MPU6050_WHO_AM_I_VALUE);
    if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
        LOG_ERR("MPU6050: unexpected device ID 0x%02X", who_am_i);
        return false;
    }

    // Reset device to clean state
    if (!i2c_write_reg(i2c, addr, REG_PWR_MGMT_1, PWR1_DEVICE_RESET)) {
        LOG_ERR("MPU6050: device reset write failed");
        return false;
    }
    LL_mDelay(100U);   // reset takes up to 100 ms

    // Wake device and select PLL with X-axis gyro clock (recommended)
    if (!i2c_write_reg(i2c, addr, REG_PWR_MGMT_1, PWR1_CLKSEL_GYRO_X)) {
        LOG_ERR("MPU6050: PWR_MGMT_1 write failed");
        return false;
    }

    // DLPF configuration
    if (!i2c_write_reg(i2c, addr, REG_CONFIG, (uint8_t)dlpf)) {
        LOG_ERR("MPU6050: CONFIG (DLPF) write failed");
        return false;
    }

    // Sample rate divider — 0 means sample rate = gyro rate / (1 + 0) = 1kHz
    if (!i2c_write_reg(i2c, addr, REG_SMPLRT_DIV, 0x00U)) {
        LOG_ERR("MPU6050: SMPLRT_DIV write failed");
        return false;
    }

    // Gyroscope full-scale range
    uint8_t gyro_cfg = (uint8_t)((uint8_t)gyro_fs << GYRO_FS_SEL_SHIFT);
    if (!i2c_write_reg(i2c, addr, REG_GYRO_CONFIG, gyro_cfg)) {
        LOG_ERR("MPU6050: GYRO_CONFIG write failed");
        return false;
    }
    uint8_t rb_gyro = 0U;
    i2c_read_reg(i2c, addr, REG_GYRO_CONFIG, &rb_gyro);
    LOG_DBG("MPU6050: GYRO_CONFIG wrote=0x%02X read=0x%02X", gyro_cfg, rb_gyro);

    // Accelerometer full-scale range
    uint8_t accel_cfg = (uint8_t)((uint8_t)accel_fs << ACCEL_FS_SEL_SHIFT);
    if (!i2c_write_reg(i2c, addr, REG_ACCEL_CONFIG, accel_cfg)) {
        LOG_ERR("MPU6050: ACCEL_CONFIG write failed");
        return false;
    }
    uint8_t rb_accel = 0U;
    i2c_read_reg(i2c, addr, REG_ACCEL_CONFIG, &rb_accel);
    LOG_DBG("MPU6050: ACCEL_CONFIG wrote=0x%02X read=0x%02X", accel_cfg, rb_accel);

    hdev->initialized = true;
    LOG_INF("MPU6050: initialized (accel_fs=%d gyro_fs=%d dlpf=%d)",
            (int)accel_fs, (int)gyro_fs, (int)dlpf);
    return true;
}

bool MPU6050_ReadRaw(MPU6050_Handle *hdev, MPU6050_RawData *raw)
{
    if (!hdev->initialized || (raw == NULL)) { return false; }

    // Burst-read 14 bytes starting at ACCEL_XOUT_H:
    //   [0-1] AX_H/L  [2-3] AY_H/L  [4-5] AZ_H/L
    //   [6-7] TEMP_H/L
    //   [8-9] GX_H/L  [10-11] GY_H/L  [12-13] GZ_H/L
    uint8_t buf[14] = {0U};
    if (!i2c_read_burst(hdev->i2c, hdev->addr, REG_ACCEL_XOUT_H, buf, 14U)) {
        return false;
    }

    raw->ax       = combine_bytes(buf[0],  buf[1]);
    raw->ay       = combine_bytes(buf[2],  buf[3]);
    raw->az       = combine_bytes(buf[4],  buf[5]);
    raw->temp_raw = combine_bytes(buf[6],  buf[7]);
    raw->gx       = combine_bytes(buf[8],  buf[9]);
    raw->gy       = combine_bytes(buf[10], buf[11]);
    raw->gz       = combine_bytes(buf[12], buf[13]);
    return true;
}

bool MPU6050_ReadScaled(MPU6050_Handle *hdev, MPU6050_Data *data)
{
    if ((hdev == NULL) || (data == NULL)) { return false; }

    MPU6050_RawData raw;
    if (!MPU6050_ReadRaw(hdev, &raw)) { return false; }

    float accel_sens = k_accel_sensitivity[(uint8_t)hdev->accel_fs];
    float gyro_sens  = k_gyro_sensitivity[(uint8_t)hdev->gyro_fs];

    data->ax_g    = (float)raw.ax / accel_sens;
    data->ay_g    = (float)raw.ay / accel_sens;
    data->az_g    = (float)raw.az / accel_sens;

    // Temperature formula from MPU-6050 datasheet:
    //   Temperature (°C) = (TEMP_OUT / 340) + 36.53
    data->temp_c  = ((float)raw.temp_raw / 340.0f) + 36.53f;

    data->gx_dps  = (float)raw.gx / gyro_sens;
    data->gy_dps  = (float)raw.gy / gyro_sens;
    data->gz_dps  = (float)raw.gz / gyro_sens;
    return true;
}

void MPU6050_Sleep(MPU6050_Handle *hdev)
{
    if (!hdev->initialized) { return; }
    uint8_t val = 0U;
    if (i2c_read_reg(hdev->i2c, hdev->addr, REG_PWR_MGMT_1, &val)) {
        val |= PWR1_SLEEP;
        i2c_write_reg(hdev->i2c, hdev->addr, REG_PWR_MGMT_1, val);
    }
}

void MPU6050_Wake(MPU6050_Handle *hdev)
{
    if (!hdev->initialized) { return; }
    uint8_t val = 0U;
    if (i2c_read_reg(hdev->i2c, hdev->addr, REG_PWR_MGMT_1, &val)) {
        val &= (uint8_t)(~PWR1_SLEEP);
        i2c_write_reg(hdev->i2c, hdev->addr, REG_PWR_MGMT_1, val);
    }
}
