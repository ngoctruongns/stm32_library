/**
 * @file  mpu6050.h
 * @brief MPU-6050 6-axis IMU driver (accelerometer + gyroscope).
 *
 * Implemented over STM32 LL I2C API (blocking, polling).
 *
 * Hardware connections (I2C3 on STM32F407):
 *   I2C3 SCL  → PA8  (AF4, open-drain)
 *   I2C3 SDA  → PC9  (AF4, open-drain)
 *   AD0       → GND  → I2C address = 0x68
 *              (AD0 = VCC → 0x69)
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

// ---------------------------------------------------------------------------
// I2C address (7-bit)
// AD0 pin LOW  → 0x68
// AD0 pin HIGH → 0x69
// ---------------------------------------------------------------------------
#define MPU6050_I2C_ADDR_LOW    0x68U
#define MPU6050_I2C_ADDR_HIGH   0x69U

#define MPU6050_WHO_AM_I_VALUE  0x68U   // fixed device ID

// ---------------------------------------------------------------------------
// Accelerometer full-scale range
// ---------------------------------------------------------------------------
typedef enum {
    MPU6050_ACCEL_FS_2G  = 0,  // ±2g,  sensitivity 16384 LSB/g
    MPU6050_ACCEL_FS_4G  = 1,  // ±4g,  sensitivity  8192 LSB/g
    MPU6050_ACCEL_FS_8G  = 2,  // ±8g,  sensitivity  4096 LSB/g
    MPU6050_ACCEL_FS_16G = 3   // ±16g, sensitivity  2048 LSB/g
} MPU6050_AccelFS;

// ---------------------------------------------------------------------------
// Gyroscope full-scale range
// ---------------------------------------------------------------------------
typedef enum {
    MPU6050_GYRO_FS_250DPS  = 0,  // ±250 °/s, sensitivity 131.0 LSB/(°/s)
    MPU6050_GYRO_FS_500DPS  = 1,  // ±500 °/s, sensitivity  65.5 LSB/(°/s)
    MPU6050_GYRO_FS_1000DPS = 2,  // ±1000°/s, sensitivity  32.8 LSB/(°/s)
    MPU6050_GYRO_FS_2000DPS = 3   // ±2000°/s, sensitivity  16.4 LSB/(°/s)
} MPU6050_GyroFS;

// ---------------------------------------------------------------------------
// Digital low-pass filter (DLPF) bandwidth
// ---------------------------------------------------------------------------
typedef enum {
    MPU6050_DLPF_260HZ = 0,
    MPU6050_DLPF_184HZ = 1,
    MPU6050_DLPF_94HZ  = 2,
    MPU6050_DLPF_44HZ  = 3,
    MPU6050_DLPF_21HZ  = 4,
    MPU6050_DLPF_10HZ  = 5,
    MPU6050_DLPF_5HZ   = 6
} MPU6050_DLPF;

// ---------------------------------------------------------------------------
// Raw sensor data (signed 16-bit per axis)
// ---------------------------------------------------------------------------
typedef struct {
    int16_t ax;  // accelerometer X
    int16_t ay;
    int16_t az;
    int16_t temp_raw;
    int16_t gx;  // gyroscope X
    int16_t gy;
    int16_t gz;
} MPU6050_RawData;

// ---------------------------------------------------------------------------
// Scaled sensor data
// ---------------------------------------------------------------------------
typedef struct {
    float ax_g;    // acceleration in g
    float ay_g;
    float az_g;
    float temp_c;  // temperature in °C
    float gx_dps;  // angular rate in °/s
    float gy_dps;
    float gz_dps;
} MPU6050_Data;

// ---------------------------------------------------------------------------
// Driver handle
// ---------------------------------------------------------------------------
typedef struct {
    I2C_TypeDef    *i2c;
    uint8_t         addr;        // 7-bit I2C address (shifted left by 1 for LL API)
    MPU6050_AccelFS accel_fs;
    MPU6050_GyroFS  gyro_fs;
    bool            initialized;
} MPU6050_Handle;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * @brief  Initialise the MPU-6050.
 *         Verifies WHO_AM_I, wakes device, sets DLPF, full-scale ranges.
 * @param  hdev     Pointer to driver handle.
 * @param  i2c      I2C peripheral to use (e.g. I2C3).
 * @param  addr     7-bit I2C address (MPU6050_I2C_ADDR_LOW or _HIGH).
 * @param  accel_fs Accelerometer full-scale range.
 * @param  gyro_fs  Gyroscope full-scale range.
 * @param  dlpf     Digital low-pass filter setting.
 * @return true on success, false on communication or identification error.
 */
bool MPU6050_Init(MPU6050_Handle  *hdev,
                  I2C_TypeDef     *i2c,
                  uint8_t          addr,
                  MPU6050_AccelFS  accel_fs,
                  MPU6050_GyroFS   gyro_fs,
                  MPU6050_DLPF     dlpf);

/**
 * @brief  Read raw 16-bit sensor values for accel, temperature, and gyro.
 * @param  hdev  Initialised driver handle.
 * @param  raw   Output struct.
 * @return true on success.
 */
bool MPU6050_ReadRaw(MPU6050_Handle *hdev, MPU6050_RawData *raw);

/**
 * @brief  Read sensor data scaled to physical units.
 * @param  hdev  Initialised driver handle.
 * @param  data  Output struct (g, °C, °/s).
 * @return true on success.
 */
bool MPU6050_ReadScaled(MPU6050_Handle *hdev, MPU6050_Data *data);

/**
 * @brief  Put MPU-6050 into sleep (low-power) mode.
 */
void MPU6050_Sleep(MPU6050_Handle *hdev);

/**
 * @brief  Wake MPU-6050 from sleep mode.
 */
void MPU6050_Wake(MPU6050_Handle *hdev);

#ifdef __cplusplus
}
#endif
