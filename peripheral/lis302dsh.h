#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

// ---------------------------------------------------------------------------
// Hardware mapping (STM32F407 Discovery — on-board LIS3DSH)
// NOTE: Later Discovery board revisions use LIS3DSH (WHO_AM_I=0x3F) instead
//       of the original LIS302DL (0x3B). This driver targets LIS3DSH.
// ---------------------------------------------------------------------------
// SPI1:  PA5=SCK, PA6=MISO, PA7=MOSI  (AF5, Mode 0: CPOL=0, CPHA=0)
// CS:    PE3 (SPI1_CS_Pin / SPI1_CS_GPIO_Port) — active low
// INT1:  PE0 (MEMS_INT1)
// INT2:  PE1 (MEMS_INT2)

#define LIS302DL_CS_PORT    SPI1_CS_GPIO_Port
#define LIS302DL_CS_PIN     SPI1_CS_Pin

#define LIS302DL_WHO_AM_I_VALUE    0x3FU   // LIS3DSH device ID

// ---------------------------------------------------------------------------
// Full-scale range
// ---------------------------------------------------------------------------
typedef enum {
    LIS302DL_FS_2G = 0,    // ±2g, sensitivity 0.06 mg/LSB (16-bit)
    LIS302DL_FS_8G = 1     // ±8g, sensitivity 0.24 mg/LSB (16-bit)
} LIS302DL_FullScale;

// ---------------------------------------------------------------------------
// Output data rate
// ---------------------------------------------------------------------------
typedef enum {
    LIS302DL_ODR_100HZ = 0,
    LIS302DL_ODR_400HZ = 1
} LIS302DL_DataRate;

// ---------------------------------------------------------------------------
// Raw accelerometer data (signed 16-bit per axis — LIS3DSH output)
// ---------------------------------------------------------------------------
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} LIS302DL_RawData;

// ---------------------------------------------------------------------------
// Scaled accelerometer data (milli-g)
// ---------------------------------------------------------------------------
typedef struct {
    float x_mg;
    float y_mg;
    float z_mg;
} LIS302DL_Data;

// ---------------------------------------------------------------------------
// Driver handle
// ---------------------------------------------------------------------------
typedef struct {
    SPI_TypeDef       *spi;
    LIS302DL_FullScale fs;
    LIS302DL_DataRate  odr;
    bool               initialized;
} LIS302DL_Handle;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * @brief  Initialise the LIS3DSH sensor on the STM32F407 Discovery board.
 *         Verifies WHO_AM_I, powers on, enables all axes, sets ODR and FS.
 * @param  hdev   Pointer to driver handle.
 * @param  spi    SPI peripheral to use (SPI1).
 * @param  odr    Output data rate.
 * @param  fs     Full-scale range.
 * @return true on success, false on communication or identification error.
 */
bool LIS302DL_Init(LIS302DL_Handle   *hdev,
                   SPI_TypeDef       *spi,
                   LIS302DL_DataRate  odr,
                   LIS302DL_FullScale fs);

/**
 * @brief  Check if new data is available on all three axes.
 * @return true if XYZ data ready.
 */
bool LIS302DL_DataReady(LIS302DL_Handle *hdev);

/**
 * @brief  Read raw signed 16-bit output for each axis.
 * @param  hdev  Initialised driver handle.
 * @param  raw   Output struct.
 * @return true on success.
 */
bool LIS302DL_ReadRaw(LIS302DL_Handle *hdev, LIS302DL_RawData *raw);

/**
 * @brief  Read acceleration scaled to milli-g.
 *         Sensitivity: 0.06 mg/digit (±2g) or 0.24 mg/digit (±8g).
 * @param  hdev  Initialised driver handle.
 * @param  data  Output struct (x_mg, y_mg, z_mg).
 * @return true on success.
 */
bool LIS302DL_ReadMg(LIS302DL_Handle *hdev, LIS302DL_Data *data);

/**
 * @brief  Put sensor into power-down mode.
 */
void LIS302DL_PowerDown(LIS302DL_Handle *hdev);

/**
 * @brief  Wake sensor from power-down mode (restores ODR and FS settings).
 */
void LIS302DL_PowerOn(LIS302DL_Handle *hdev);

#ifdef __cplusplus
}
#endif
