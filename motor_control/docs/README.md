# Motor DC Control Library - STM32F407

DC 12V motor control library with PID controller on STM32F407 MCU.

## 📋 Architecture Overview

The system is divided into 4 main classes working in coordination:

```
┌─────────────────┐
│   Motor Class   │  ← Highest level controller class
└────────┬────────┘
         │
    ┌────┴────┬──────────┬─────────────┐
    │          │          │             │
    ▼          ▼          ▼             ▼
┌────────┐ ┌──────┐ ┌───────┐ ┌──────────┐
│Encoder │ │ PID  │ │Driver │ │STM32 HAL │
│(TIM2)  │ │ Ctrl │ │(L298) │ │Hardware  │
└────────┘ └──────┘ └───────┘ └──────────┘
```

### Classes Overview

#### 1️⃣ **Encoder**
- Reads encoder values from Timer 2 (Quadrature mode)
- Computes: revolutions, angle, angular velocity, RPM
- Updates speed periodically

#### 2️⃣ **PIDController**
- Standard PID algorithm implementation
- Anti-Windup: Integral accumulation limit prevents overshoot
- Output limiting: Constrains control signal
- Formula: `u(t) = Kp*e(t) + Ki*∫e(dt) + Kd*de/dt`

#### 3️⃣ **MotorDriver**
- Controls L298N driver circuit
- PWM (Timer 3 Channel 1) → Enable pin
- IN1, IN2 (GPIO) → Select rotation direction (forward/backward/stop)
- Simple API: `setSpeed(-100 to 100)` or `setDirection()` + `setPower()`

#### 4️⃣ **Motor** (Main Controller)
- Integrates all 3 classes above
- High-level API: `setTargetRPM()`, `update()`, `getCurrentRPM()`
- Runs PID control loop for speed regulation

## 🔧 Hardware Configuration

### Timer Setup (STM32CubeMX)

```
TIM2 (Encoder):
  - Quadrature Encoder Interface (XOR of both inputs)
  - Internal Clock
  - ETR Pin: PA0 (encoder input)
  - Auto Reload Value: 65535 (max 16-bit counter)

TIM3 (PWM):
  - Channel 1: PA6 (PWM output → L298 Enable pin)
  - Mode: PWM1 (Edge-aligned)
  - Prescaler: 84 - 1 = 83  (For ~1kHz PWM at 84MHz)
  - Auto Reload Value: 1000  (must match MOTOR_PWM_MAX)
```

### GPIO Setup (STM32CubeMX)

```
PA4:  Output (L298 IN1)
PA5:  Output (L298 IN2)
PA6:  Alternate Function (TIM3_CH1 - PWM)
```

### L298N Connections

```
┌─────────────────────────────────┐
│  STM32F407                      │
├─────────────────────────────────┤
│ PA6  ──────────┬──→ Enable  (L298)
│ PA4  ──────────┬──→ IN1     (L298)
│ PA5  ──────────┬──→ IN2     (L298)
│ GND  ──────────┬──→ GND     (L298)
│ 5V   ──────────┬──→ 5V      (L298)
└─────────────────────────────────┘

L298N Output:
├──→ Motor Pin 1
├──→ Motor Pin 2

Encoder → Timer 2 Input Capture
```

## 📝 Usage

### 1. Initialization in main.c

```c
#include "motor_example.cpp"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();    // Encoder
    MX_TIM3_Init();    // PWM
    MX_TIM4_Init();    // Update timer (optional, 100Hz)

    // Initialize motor system
    motor_system_init(&htim2, &htim3);

    // Start update timer (if using interrupt)
    HAL_TIM_Base_Start_IT(&htim4);

    while (1)
    {
        // Outside main loop:
        // - Set motor speed: motor_set_speed_rpm(500);
        // - Read values: rpm = motor_get_current_rpm();
    }

    motor_system_deinit();
    return 0;
}
```

### 2. Periodic Update (100 Hz)

**Option A: Timer Interrupt**
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)  // 100 Hz timer
    {
        motor_system_update();
    }
}
```

**Option B: Manual in main loop**
```c
uint32_t last_update = HAL_GetTick();
while (1)
{
    if (HAL_GetTick() - last_update >= 10)  // 10ms = 100Hz
    {
        motor_system_update();
        last_update = HAL_GetTick();
    }
}
```

### 3. Motor Control

```c
// Set speed 500 RPM forward
motor_set_speed_rpm(500.0f);

// Or rotate backward
motor_set_speed_rpm(-300.0f);

// Stop immediately
motor_stop();

// Read values
float current_rpm = motor_get_current_rpm();
float current_power = motor_get_current_power();
float ang_vel = motor_get_current_angular_velocity();
```

## ⚙️ Tuning PID Parameters (motor_config.h)

Default parameters:
```c
#define PID_KP 2.0f      // Proportional gain
#define PID_KI 0.5f      // Integral gain
#define PID_KD 0.1f      // Derivative gain
```

### PID Tuning Guide

1. **Start with P only**: Set Ki=0, Kd=0, adjust Kp
   - Too slow: increase Kp
   - Overshoot: decrease Kp

2. **Add Integral**: Gradually increase Ki
   - Fixes steady-state error
   - Avoid excessive gain to prevent overshoot

3. **Add Derivative**: Gradually increase Kd
   - Smooths control process
   - Reduces overshoot

**Tuning Examples**:
```c
// Fast response (aggressive):
#define PID_KP 3.5f
#define PID_KI 0.8f
#define PID_KD 0.2f

// Balanced:
#define PID_KP 2.0f
#define PID_KI 0.5f
#define PID_KD 0.1f

// Smooth, stable:
#define PID_KP 1.0f
#define PID_KI 0.2f
#define PID_KD 0.05f
```

## 📊 API Reference

### Motor Class

```cpp
// Initialize
Motor(driver, encoder, pid_controller);
HAL_StatusTypeDef init();

// Control
void setTargetRPM(float rpm);
void setTargetAngularVelocity(float rad_per_sec);
void update(float dt);
void stop();

// Read
float getCurrentRPM();
float getCurrentAngularVelocity();
float getCurrentPower();
float getSpeedError();
void resetEncoder();
```

### MotorDriver Class

```cpp
void setDirection(MotionDirection direction);  // FORWARD/BACKWARD/STOP
void setPower(float power);                    // 0-100%
void setSpeed(float speed);                    // -100 to 100
void brake();
float getSpeed();
MotionDirection getDirection();
```

### Encoder Class

```cpp
HAL_StatusTypeDef init();
int32_t getRawCount();
void resetCount();
float getRevolutions();
float getAngleDegree();
float getAngleRadian();
void updateSpeed(float dt);
float getAngularVelocity();
float getRPM();
```

### PIDController Class

```cpp
void setGains(float kp, float ki, float kd);
void setIntegralLimit(float max_integral);
void setOutputLimit(float out_min, float out_max);
float compute(float setpoint, float feedback, float dt);
void reset();
```

## 🐛 Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| Motor won't spin | PWM not started | Check `tim3->Instance->CCER` EN1 bit |
| Reverse rotation | Wrong encoder/IN1/IN2 connection | Swap connections or direction |
| Unstable speed | Bad PID tuning | Reduce Kp, increase update frequency |
| No encoder reading | Timer 2 not started | Check `HAL_TIM_Encoder_Start()` |

## 💡 Suggested Improvements

1. **Soft Starting**: Ramp-up speed gradually
2. **Current Limiting**: Protect motor from overcurrent
3. **Temperature Monitoring**: Detect overheating
4. **Dual Motor Sync**: Synchronize motors for differential drive
5. **Multi-driver Support**: Support other drivers besides L298

For more details, see IMPROVEMENTS_SUGGESTIONS.md and PID_TUNING_GUIDE.md

---

**Version**: 1.0
**Last Updated**: Feb 2026
