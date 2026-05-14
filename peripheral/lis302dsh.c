/**
 * @file  lis302dsh.c
 * @brief LIS302DL MEMS 3-axis accelerometer driver.
 *
 * Implemented over STM32 LL SPI API (blocking, software CS).
 *
 * Hardware connections on STM32F407 Discovery (on-board sensor):
 *   SPI1 SCK   → PA5  (AF5)
 *   SPI1 MISO  → PA6  (AF5)
 *   SPI1 MOSI  → PA7  (AF5)
 *   CS         → PE3  (active low, software-controlled)
 *   INT1       → PE0  (MEMS_INT1, optional)
 *   INT2       → PE1  (MEMS_INT2, optional)
 *
 * SPI1 is used in Mode 3 (CPOL=High, CPHA=2Edge), 8-bit, MSB first.
 * The driver forces this mode during LIS302DL_Init() to match LIS3DSH timing.
 */

#include "lis302dsh.h"
#include "main.h"
#include "log_helper.h"

// ---------------------------------------------------------------------------
// Register map (LIS3DSH)
// ---------------------------------------------------------------------------
#define REG_WHO_AM_I        0x0FU
#define REG_CTRL_REG4       0x20U   // ODR, BDU, axis enable
#define REG_CTRL_REG5       0x24U   // full scale, anti-alias filter
#define REG_STATUS_REG      0x27U
#define REG_OUT_X_L         0x28U   // 16-bit little-endian output
#define REG_OUT_Y_L         0x2AU
#define REG_OUT_Z_L         0x2CU

// CTRL_REG4 bit masks (LIS3DSH)
#define CTRL4_ODR_100HZ (0x60U)  // ODR[7:4]=0110, 100 Hz
#define CTRL4_ODR_400HZ (0x70U)  // ODR[7:4]=0111, 400 Hz
#define CTRL4_BDU       (1U << 3U)   // Block Data Update: freeze output regs until MSB read
#define CTRL4_ZEN       (1U << 2U)
#define CTRL4_YEN       (1U << 1U)
#define CTRL4_XEN       (1U << 0U)

// CTRL_REG5 full-scale values (bits[5:3])
#define CTRL5_FS_2G     (0x00U)  // ±2g
#define CTRL5_FS_8G     (0x18U)  // ±8g  (FS=011 → bits[5:3]=011)

// STATUS_REG bit masks
#define STATUS_ZYXDA    (1U << 3U)   // X/Y/Z data available
#define STATUS_ZYXOR    (1U << 6U)   // X/Y/Z data overrun

// SPI frame bits
#define SPI_READ        (1U << 7U)   // read flag
#define SPI_WRITE       (0U << 7U)   // write flag (explicit for clarity)
#define SPI_MULTI       (1U << 6U)   // auto-increment (multi-byte)

// Sensitivity (mg per LSB, 16-bit two's complement output)
#define SENSITIVITY_2G  0.06f
#define SENSITIVITY_8G  0.24f

#define SPI_TIMEOUT_MS  5U

// ---------------------------------------------------------------------------
// Low-level SPI helpers
// ---------------------------------------------------------------------------

static inline void cs_assert(void)
{
    LL_GPIO_ResetOutputPin(LIS302DL_CS_PORT, LIS302DL_CS_PIN);
}

static inline void cs_deassert(void)
{
    LL_GPIO_SetOutputPin(LIS302DL_CS_PORT, LIS302DL_CS_PIN);
}

static void spi_flush_rx(SPI_TypeDef *spi)
{
    while (LL_SPI_IsActiveFlag_RXNE(spi)) {
        (void)LL_SPI_ReceiveData8(spi);
    }
}

static bool spi_wait_not_busy(SPI_TypeDef *spi)
{
    uint32_t t0 = get_ms_tick_count();
    while (LL_SPI_IsActiveFlag_BSY(spi)) {
        if ((get_ms_tick_count() - t0) >= SPI_TIMEOUT_MS) { return false; }
    }
    return true;
}

static bool spi_txrx_byte(SPI_TypeDef *spi, uint8_t tx, uint8_t *rx)
{
    uint32_t t0 = get_ms_tick_count();
    while (!LL_SPI_IsActiveFlag_TXE(spi)) {
        if ((get_ms_tick_count() - t0) >= SPI_TIMEOUT_MS) { return false; }
    }
    LL_SPI_TransmitData8(spi, tx);

    t0 = get_ms_tick_count();
    while (!LL_SPI_IsActiveFlag_RXNE(spi)) {
        if ((get_ms_tick_count() - t0) >= SPI_TIMEOUT_MS) { return false; }
    }
    *rx = LL_SPI_ReceiveData8(spi);
    return true;
}

static bool spi_write_reg(SPI_TypeDef *spi, uint8_t reg, uint8_t value)
{
    uint8_t dummy;
    if (!spi_wait_not_busy(spi)) { return false; }
    spi_flush_rx(spi);
    cs_assert();
    bool ok = spi_txrx_byte(spi, (uint8_t)(SPI_WRITE | (reg & 0x3FU)), &dummy);
    if (ok) {
        ok = spi_txrx_byte(spi, value, &dummy);
    }
    ok = ok && spi_wait_not_busy(spi);
    cs_deassert();
    return ok;
}

static bool spi_read_reg(SPI_TypeDef *spi, uint8_t reg, uint8_t *value)
{
    for (uint32_t attempt = 0U; attempt < 2U; ++attempt) {
        uint8_t dummy;
        if (!spi_wait_not_busy(spi)) { return false; }
        spi_flush_rx(spi);
        cs_assert();
        bool ok = spi_txrx_byte(spi, (uint8_t)(SPI_READ | (reg & 0x3FU)), &dummy);
        if (ok) {
            ok = spi_txrx_byte(spi, 0x00U, value);
        }
        ok = ok && spi_wait_not_busy(spi);
        cs_deassert();
        if (ok) {
            return true;
        }
    }
    return false;
}

/* Read one axis (L then H) without auto-increment dependency. */
static bool spi_read_axis16(SPI_TypeDef *spi, uint8_t reg_l, int16_t *value, uint8_t *lo, uint8_t *hi)
{
    uint8_t l = 0U;
    uint8_t h = 0U;
    bool ok = spi_read_reg(spi, reg_l, &l);
    if (ok) { ok = spi_read_reg(spi, (uint8_t)(reg_l + 1U), &h); }
    if (ok) {
        *value = (int16_t)((uint16_t)h << 8U | (uint16_t)l);
        if (lo != NULL) { *lo = l; }
        if (hi != NULL) { *hi = h; }
    }
    return ok;
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static uint8_t build_ctrl_reg4(LIS302DL_DataRate odr, bool active)
{
    uint8_t reg = CTRL4_XEN | CTRL4_YEN | CTRL4_ZEN | CTRL4_BDU;
    if (active) {
        reg |= (odr == LIS302DL_ODR_400HZ) ? CTRL4_ODR_400HZ : CTRL4_ODR_100HZ;
    }
    return reg;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool LIS302DL_Init(LIS302DL_Handle   *hdev,
                   SPI_TypeDef       *spi,
                   LIS302DL_DataRate  odr,
                   LIS302DL_FullScale fs)
{
    hdev->spi         = spi;
    hdev->odr         = odr;
    hdev->fs          = fs;
    hdev->initialized = false;

    // Ensure SPI uses LIS3DSH-compatible SPI mode (Mode 3)
    LL_SPI_Disable(spi);
    LL_SPI_SetClockPolarity(spi, LL_SPI_POLARITY_HIGH);
    LL_SPI_SetClockPhase(spi, LL_SPI_PHASE_2EDGE);

    // Ensure SPI is enabled
    if (!LL_SPI_IsEnabled(spi)) {
        LL_SPI_Enable(spi);
    }

    // CS deasserted initially
    cs_deassert();

    // Short settling delay — LIS3DSH turn-on time: 1/ODR + 6.6 ms = ~17 ms at 100 Hz
    LL_mDelay(20U);

    // Verify device identity
    uint8_t who_am_i = 0U;
    if (!spi_read_reg(spi, REG_WHO_AM_I, &who_am_i)) {
        LOG_DBG("LIS302DL: SPI read WHO_AM_I failed (timeout/MISO error)");
        return false;
    }
    LOG_DBG("LIS302DL: WHO_AM_I=0x%02X (exp 0x%02X)", who_am_i, LIS302DL_WHO_AM_I_VALUE);
    if (who_am_i != LIS302DL_WHO_AM_I_VALUE) { return false; }

    // Power on, set ODR, enable all axes (CTRL_REG4)
    uint8_t ctrl4 = build_ctrl_reg4(odr, true);
    if (!spi_write_reg(spi, REG_CTRL_REG4, ctrl4)) {
        LOG_DBG("LIS302DL: write CTRL_REG4 failed");
        return false;
    }
    // Readback to verify write succeeded
    uint8_t rb4 = 0U;
    spi_read_reg(spi, REG_CTRL_REG4, &rb4);
    LOG_DBG("LIS302DL: CTRL_REG4 wrote=0x%02X read=0x%02X", ctrl4, rb4);
    if (rb4 != ctrl4) { return false; }

    // Full-scale selection (CTRL_REG5)
    uint8_t ctrl5 = (fs == LIS302DL_FS_8G) ? CTRL5_FS_8G : CTRL5_FS_2G;
    if (!spi_write_reg(spi, REG_CTRL_REG5, ctrl5)) {
        LOG_DBG("LIS302DL: write CTRL_REG5 failed");
        return false;
    }
    uint8_t rb5 = 0U;
    spi_read_reg(spi, REG_CTRL_REG5, &rb5);
    LOG_DBG("LIS302DL: CTRL_REG5 wrote=0x%02X read=0x%02X", ctrl5, rb5);

    hdev->initialized = true;
    return true;
}

bool LIS302DL_DataReady(LIS302DL_Handle *hdev)
{
    if (!hdev->initialized) { return false; }
    uint8_t status = 0U;
    if (!spi_read_reg(hdev->spi, REG_STATUS_REG, &status)) { return false; }
    return (bool)(status & STATUS_ZYXDA);
}

bool LIS302DL_ReadRaw(LIS302DL_Handle *hdev, LIS302DL_RawData *raw)
{
    if (!hdev->initialized || (raw == NULL)) { return false; }

    uint8_t status_before = 0xFFU;
    uint8_t status_after = 0xFFU;
    uint8_t bytes[6] = {0U};
    (void)spi_read_reg(hdev->spi, REG_STATUS_REG, &status_before);

    bool ok = spi_read_axis16(hdev->spi, REG_OUT_X_L, &raw->x, &bytes[0], &bytes[1]);
    if (ok) { ok = spi_read_axis16(hdev->spi, REG_OUT_Y_L, &raw->y, &bytes[2], &bytes[3]); }
    if (ok) { ok = spi_read_axis16(hdev->spi, REG_OUT_Z_L, &raw->z, &bytes[4], &bytes[5]); }
    if (!ok) { return false; }

    (void)spi_read_reg(hdev->spi, REG_STATUS_REG, &status_after);

    static bool has_prev = false;
    static int16_t prev_x = 0;
    static int16_t prev_y = 0;
    static int16_t prev_z = 0;
    static uint32_t unchanged_count = 0U;

    bool unchanged = has_prev && (raw->x == prev_x) && (raw->y == prev_y) && (raw->z == prev_z);
    if (unchanged) {
        unchanged_count++;
    } else {
        unchanged_count = 0U;
    }

    LOG_VERB("LIS302DL DBG st=0x%02X->0x%02X raw=[%02X %02X %02X %02X %02X %02X] xyz_raw=%d,%d,%d same=%lu",
            status_before,
            status_after,
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5],
            (int)raw->x, (int)raw->y, (int)raw->z,
            (unsigned long)unchanged_count);

    prev_x = raw->x;
    prev_y = raw->y;
    prev_z = raw->z;
    has_prev = true;

    return true;
}

bool LIS302DL_ReadMg(LIS302DL_Handle *hdev, LIS302DL_Data *data)
{
    if (data == NULL) { return false; }

    LIS302DL_RawData raw;
    if (!LIS302DL_ReadRaw(hdev, &raw)) { return false; }

    float sens = (hdev->fs == LIS302DL_FS_8G) ? SENSITIVITY_8G : SENSITIVITY_2G;
    data->x_mg = (float)raw.x * sens;
    data->y_mg = (float)raw.y * sens;
    data->z_mg = (float)raw.z * sens;
    return true;
}

void LIS302DL_PowerDown(LIS302DL_Handle *hdev)
{
    if (!hdev->initialized) { return; }
    spi_write_reg(hdev->spi, REG_CTRL_REG4, build_ctrl_reg4(hdev->odr, false));
}

void LIS302DL_PowerOn(LIS302DL_Handle *hdev)
{
    if (!hdev->initialized) { return; }
    spi_write_reg(hdev->spi, REG_CTRL_REG4, build_ctrl_reg4(hdev->odr, true));
}
