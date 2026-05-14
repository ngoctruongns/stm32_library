# Peripheral Library for STM32F407

Collection of peripheral drivers and control utilities used by the diff-drive robot firmware on STM32F407 Discovery.

## 📋 Module Overview

```
Library/peripheral/
├── peri_config.h       ← Compile-time configuration constants
├── peri_control.h/c    ← Top-level peripheral orchestration loop
├── buzzer.h/c          ← Buzzer driver (solid / blink / beep modes)
├── uart_lib.h/c        ← UART2/UART3 communication + DMA transport
├── lis302dsh.h/c        ← LIS302DL MEMS 3-axis accelerometer driver (on-board, SPI1)
└── vl53l0x.h/c        ← VL53L0X time-of-flight distance sensor driver (I2C3)
```

---

## ⚙️ Configuration — `peri_config.h`

Central header for tunable constants shared across peripheral modules.

| Macro | Default | Description |
|---|---|---|
| `BUZZER_BLINK_ON_DURATION_MS` | 500 | ON duration for blink mode |
| `BUZZER_BLINK_OFF_DURATION_MS` | 500 | OFF duration for blink mode |
| `BUZZER_BEEP_ON_DURATION_MS` | 200 | ON duration for beep mode |
| `JOYSTICK_LOOP_INTERVAL_MS` | 50 | PS2X poll interval (ms) |
| `CONTROL_CMD_TIMEOUT_MS` | 200 | Safety timeout: stop if no valid command |

---

## 🔧 Module Details

### 1️⃣ Peripheral Control — `peri_control.h/c`

Top-level orchestrator called from `main.c`. Coordinates all peripherals in a single control loop.

**Key functions:**

| Function | Description |
|---|---|
| `peripheral_init()` | Initialise all peripherals (buzzer, diff-drive, WS2812, PS2X) |
| `peripheral_control_loop()` | Main polling loop — joystick → UART3 fallback → motor control |
| `peripheral_tim10_interrupt_handler()` | TIM10 ISR hook (1 kHz tick tasks) |

**Control priority:**
1. PS2X joystick (highest)
2. UART3 `CMD_VEL` packet (fallback, valid within `CONTROL_CMD_TIMEOUT_MS`)
3. Motor stop (safety default)

---

### 2️⃣ Buzzer — `buzzer.h/c`

GPIO-driven active buzzer with three operating modes.

**Modes (`BuzzerType`):**

| Mode | Behaviour |
|---|---|
| `BUZZER_TYPE_SOLID` | Always on/off as set |
| `BUZZER_TYPE_BLINK` | Toggles between ON and OFF at configurable durations |
| `BUZZER_TYPE_BEEP` | Single beep pulse, then goes silent |

**Key functions:**

| Function | Description |
|---|---|
| `Buzzer_Init(type, state)` | Initialise with given mode and initial state |
| `Buzzer_SetState(type, state)` | Change mode/state at runtime |
| `Buzzer_Update()` | Call periodically to drive blink/beep timing |
| `Buzzer_Stop()` | Immediately silence buzzer |

**Usage example:**
```c
Buzzer_Init(BUZZER_TYPE_BEEP, BUZZER_ON);   // single beep on boot

// In main loop or 1 kHz timer:
Buzzer_Update();
```

---

### 3️⃣ UART Library — `uart_lib.h/c`

Handles UART2 (debug/logging) and UART3 (robot communication with Raspberry Pi) using DMA.

**UART3 transport:**
- RX: DMA1 Stream1 (circular, channel 4)
- TX: DMA1 Stream3 (normal, channel 4)
- Protocol: binary data packets defined in `serial_comm/`

**Key functions:**

| Function | Description |
|---|---|
| `uart3_comm_init()` | Initialise UART3 DMA buffers and enable reception |
| `uart3_comm_poll()` | Process received bytes, decode packets |
| `uart3_get_latest_cmd_vel(cmd, rx_time_ms)` | Get last valid `CmdVelType` packet and its timestamp |
| `usart2_interrupt_handler(data)` | USART2 byte-received ISR hook |
| `usart3_idle_interrupt_handler()` | USART3 IDLE line ISR hook |
| `usart3_dma_rx_interrupt_handler()` | DMA RX half/complete ISR hook |
| `usart3_dma_tx_interrupt_handler()` | DMA TX complete ISR hook |

---

### 4️⃣ LIS3DSH MEMS Accelerometer — `lis302dsh.h/c`

Driver for the on-board ST LIS3DSH 3-axis MEMS accelerometer on the STM32F407 Discovery board.
Implemented over the STM32 LL SPI API (blocking, software CS).

**Specifications:**
- Axes: X, Y, Z (signed 16-bit output per axis)
- Full scale: ±2g (sensitivity 0.06 mg/digit) or ±8g (0.24 mg/digit)
- Output data rate: 100 Hz or 400 Hz
- Interface: SPI Mode 0, 8-bit, MSB first
- WHO_AM_I register: `0x3F`

**Hardware connections (on-board, no external wiring needed):**

| Signal | STM32F407 Pin | Notes |
|---|---|---|
| SPI1 SCK | **PA5** (AF5) | |
| SPI1 MISO | **PA6** (AF5) | |
| SPI1 MOSI | **PA7** (AF5) | |
| CS | **PE3** (`SPI1_CS_Pin`) | Active low, SW-controlled |
| INT1 | **PE0** (MEMS_INT1) | Optional, free-fall / wake-up |
| INT2 | **PE1** (MEMS_INT2) | Optional |

**Key types:**

| Type | Description |
|---|---|
| `LIS302DL_FullScale` | `LIS302DL_FS_2G` / `LIS302DL_FS_8G` |
| `LIS302DL_DataRate` | `LIS302DL_ODR_100HZ` / `LIS302DL_ODR_400HZ` |
| `LIS302DL_RawData` | Signed 16-bit x/y/z |
| `LIS302DL_Data` | Float x_mg/y_mg/z_mg (milli-g) |

**Key functions:**

| Function | Description |
|---|---|
| `LIS302DL_Init(hdev, spi, odr, fs)` | Init: verify WHO_AM_I, power on, configure ODR and FS |
| `LIS302DL_DataReady(hdev)` | Returns true when XYZ data available |
| `LIS302DL_ReadRaw(hdev, raw)` | Read raw signed 16-bit output |
| `LIS302DL_ReadMg(hdev, data)` | Read acceleration in milli-g |
| `LIS302DL_PowerDown(hdev)` | Enter power-down mode |
| `LIS302DL_PowerOn(hdev)` | Wake from power-down |

**Usage example:**
```c
#include "lis302dsh.h"

static LIS302DL_Handle lis302dl;

// Initialise once:
if (!LIS302DL_Init(&lis302dl, SPI1, LIS302DL_ODR_100HZ, LIS302DL_FS_2G)) {
    // Handle init error (wrong board or SPI issue)
}

// Poll in control loop:
if (LIS302DL_DataReady(&lis302dl)) {
    LIS302DL_Data accel;
    LIS302DL_ReadMg(&lis302dl, &accel);
    // accel.x_mg, accel.y_mg, accel.z_mg in milli-g
}
```

---

### 5️⃣ VL53L0X Distance Sensor — `vl53l0x.h/c`

Driver for the ST VL53L0X time-of-flight (ToF) sensor for obstacle detection.
Implemented over the STM32 LL I2C API (blocking, timeout-guarded).

**Specifications:**
- Range: 30 – 2000 mm (reliable)
- Interface: I2C, default address `0x29` (7-bit)
- Return value `0xFFFF` (`VL53L0X_OUT_OF_RANGE`) when > 2 m or error

**Timing budget presets (`VL53L0X_TimingBudget`):**

| Preset | Budget | Accuracy |
|---|---|---|
| `VL53L0X_TIMING_HIGH_SPEED` | ~20 ms | ±5 % |
| `VL53L0X_TIMING_DEFAULT` | ~33 ms | standard |
| `VL53L0X_TIMING_HIGH_ACCURACY` | 200 ms | best |

**Key functions:**

| Function | Description |
|---|---|
| `VL53L0X_Init(hdev, i2c, budget)` | Initialise sensor: ID check, SPAD calibration, set timing budget |
| `VL53L0X_ReadRangeSingleMM(hdev)` | Single-shot blocking measurement (returns mm) |
| `VL53L0X_StartContinuous(hdev, period_ms)` | Start continuous ranging (0 = back-to-back) |
| `VL53L0X_ReadRangeContinuousMM(hdev)` | Non-blocking read in continuous mode |
| `VL53L0X_StopContinuous(hdev)` | Stop continuous ranging |
| `VL53L0X_SetAddress(hdev, new_addr)` | Change I2C address (multi-sensor bus sharing) |

**Hardware wiring (STM32F407 Discovery — I2C3):**

| VL53L0X Pin | STM32F407 Pin | Notes |
|---|---|---|
| VIN / VDD | 3.3 V | Do **not** use 5 V |
| GND | GND | |
| SCL | **PA8** (I2C3_SCL, AF4) | 4.7 kΩ pull-up to 3.3 V |
| SDA | **PC9** (I2C3_SDA, AF4) | 4.7 kΩ pull-up to 3.3 V |
| XSHUT | Any GPIO (optional) | LOW = power-off sensor |
| GPIO1 | Any GPIO (optional) | Data-ready interrupt |

**Usage example:**
```c
#include "vl53l0x.h"

static VL53L0X_Handle vl53l0x;

// Initialise once:
if (!VL53L0X_Init(&vl53l0x, I2C3, VL53L0X_TIMING_DEFAULT)) {
    // Handle init error
}

// Single-shot read (blocking, ~33 ms):
uint16_t dist_mm = VL53L0X_ReadRangeSingleMM(&vl53l0x);
if (dist_mm != VL53L0X_OUT_OF_RANGE) {
    // Valid distance in dist_mm
}

// Continuous mode (non-blocking in control loop):
VL53L0X_StartContinuous(&vl53l0x, 50);   // 50 ms/measurement
// In loop:
uint16_t d = VL53L0X_ReadRangeContinuousMM(&vl53l0x);
```

---

## 🔗 Dependencies

| Module | Depends on |
|---|---|
| `peri_control` | `buzzer`, `uart_lib`, `diff_drive` (motor_control), `ws2812` (led_ws2812), `PS2X_lib` |
| `buzzer` | `main.h` (LL GPIO), `peri_config.h` |
| `uart_lib` | `main.h` (LL USART/DMA), `serial_comm/` |
| `lis302dl` | `main.h` (LL SPI/GPIO) |
| `vl53l0x` | `main.h` (LL I2C) |
