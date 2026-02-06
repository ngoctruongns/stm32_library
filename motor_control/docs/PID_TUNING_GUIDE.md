# PID Parameter Tuning Guide for DC Motor

Complete guide to tuning PID controller parameters for optimal motor performance.

## 1. Theory Basics

### PID Control Formula
```
u(t) = Kp*e(t) + Ki*∫e(dt) + Kd*de/dt
```

Where:
- `e(t)` = error = setpoint - feedback
- `u(t)` = control signal (output)
- `Kp` = Proportional gain (0.1 - 10)
- `Ki` = Integral gain (0.01 - 2)
- `Kd` = Derivative gain (0 - 1)

### Effect of Each Component

| Component | Increase Kp | Increase Ki | Increase Kd |
|-----------|-------------|-------------|------------|
| Rise Time | Decrease | Decrease | Minimal |
| Overshoot | Increase | Increase | Decrease |
| Settling Time | Decrease | Decrease | Decrease |
| Steady-State Error | Decrease | Decrease | Minimal |
| Stability | Decrease | Decrease | Increase |

## 2. Tuning Procedure (Ziegler-Nichols Method)

### Step 1: Prepare Initial Setup
```c
// Set all gains to zero
#define PID_KP 0.0f
#define PID_KI 0.0f
#define PID_KD 0.0f
```

### Step 2: Increase Kp Until Oscillation
```c
// Test sequential values
#define PID_KP 0.5f    // Too slow?
#define PID_KP 1.0f    // Still slow?
#define PID_KP 2.0f    // Starting to oscillate?
#define PID_KP 3.0f    // Clear oscillation

// Record Kp value at sustained oscillation: Ku
// Example: Ku = 3.0 (motor starts oscillating at this Kp)
// Measure oscillation period: Tu = 0.5s (use oscilloscope or logging)
```

### Step 3: Calculate from Ku and Tu
```
Ziegler-Nichols Classic Rules:
  Kp = 0.6 * Ku
  Ki = 1.2 * Ku / Tu
  Kd = 0.075 * Ku * Tu

Example: If Ku=3.0, Tu=0.5s:
  Kp = 0.6 * 3.0 = 1.8
  Ki = 1.2 * 3.0 / 0.5 = 7.2
  Kd = 0.075 * 3.0 * 0.5 = 0.1125
```

### Step 4: Manual Fine-tuning
After applying formula, fine-tune based on performance:

```c
// Initial from formula
#define PID_KP 1.8f
#define PID_KI 7.2f
#define PID_KD 0.1125f

// Still has overshoot → reduce Ki
#define PID_KI 5.0f

// Response too slow → increase Kp
#define PID_KP 2.2f

// Still oscillating → increase Kd
#define PID_KD 0.2f
```

## 3. Practical Tuning (Without Oscilloscope)

### Step 1: P Only (Set Ki=0, Kd=0)
```c
#define PID_KP 2.0f
#define PID_KI 0.0f
#define PID_KD 0.0f
```

Observe when ramping from 0 → 500 RPM:
- Settles too slow (>1s)? → increase Kp
- Large overshoot (>130%)? → decrease Kp
- Oscillates? → decrease Kp more

**Optimal**: Settles in 0.3-0.5s, <10% overshoot

### Step 2: Add Integral (Increase Ki)
```c
#define PID_KI 0.5f
```

- Steady-state error (motor runs slower than setpoint)? → increase Ki
- Worse response? → decrease Ki

**Optimal**: Nearly zero steady-state error

### Step 3: Add Derivative (Increase Kd)
```c
#define PID_KD 0.1f
```

- Still large overshoot? → increase Kd
- Response too "soft"? → decrease Kd

**Optimal**: <5% overshoot, smooth response

## 4. Ready-Made Tuning Sets

### Aggressive (Fast Response)
```c
#define PID_KP 3.5f
#define PID_KI 0.8f
#define PID_KD 0.2f
```
- Fast settle time
- Higher overshoot risk
- Suitable for high-speed requirements

### Moderate (Balanced)
```c
#define PID_KP 2.0f
#define PID_KI 0.5f
#define PID_KD 0.1f
```
- Good compromise
- Moderate response speed
- Acceptable overshoot
- **Default recommendation**

### Conservative (Smooth & Stable)
```c
#define PID_KP 1.0f
#define PID_KI 0.2f
#define PID_KD 0.05f
```
- Very smooth response
- Longer settle time
- Minimal overshoot
- Suitable for sensitive applications

## 5. Anti-Windup Tuning

Prevents integral accumulation when motor can't reach target:

```c
// motor_config.h
#define PID_INTEGRAL_LIMIT 50.0f  // Accumulated error limit

// Check by logging:
// Integral contribution = PID_KI * integral_sum
// Should not exceed output_max
```

If overshoot occurs after starting:
```c
// Reduce integral limit
#define PID_INTEGRAL_LIMIT 30.0f
```

## 6. Output Limiting

```c
// motor_config.h
#define PID_OUT_MIN -100.0f
#define PID_OUT_MAX 100.0f
```

Output constrained to -100 to 100% (PWM limits).

If motor can't reach target speed:
- Check PWM max value is sufficient
- Verify motor specifications
- Check power supply voltage (12V adequate?)

## 7. Debug & Monitoring

### Add Logging
```c
// In main loop
if (count % 100 == 0)  // Print every 100 updates
{
    printf("Target: %.1f, Current: %.1f, Error: %.1f, "
           "P: %.2f, I: %.2f, D: %.2f, Out: %.1f\n",
           target_rpm,
           motor_get_current_rpm(),
           getSpeedError(),
           g_pid->getP(),
           g_pid->getI(),
           g_pid->getD(),
           motor_get_current_power());
}
```

### Performance Metrics

1. **Rise Time**: Time from 0 → 90% setpoint
2. **Overshoot**: Percentage above setpoint
3. **Settling Time**: Time to stabilize within ±5%
4. **Steady-State Error**: Final error at stable state

**Good targets**:
- Rise Time: < 500ms
- Overshoot: < 10%
- Settling Time: < 1000ms
- SSE: < 2%

## 8. Common Issues

### Motor Too Slow
```
Cause: Kp too low
Solution:
  1. Increase Kp: 2.0 → 2.5 → 3.0
  2. If still slow, check:
     - PWM max value
     - Motor load
     - PID_OUT_MAX
```

### Oscillation (Motor Speed Fluctuates)
```
Cause: Kp too high OR Kd too low
Solution:
  1. Decrease Kp: 3.0 → 2.5 → 2.0
  2. Increase Kd: 0.1 → 0.15 → 0.2
  3. Verify update frequency ≥ 50Hz
```

### Large Overshoot
```
Cause: Ki too high OR Anti-Windup too loose
Solution:
  1. Decrease Ki: 0.8 → 0.5 → 0.3
  2. Decrease PID_INTEGRAL_LIMIT
  3. Increase Kd for damping
```

### Can't Reach Target RPM
```
Cause: Motor load too high OR insufficient power
Solution:
  1. Verify motor max RPM specs
  2. Check L298 connections
  3. Increase Ki for steady-state compensation
  4. Verify 12V power supply
```

## 9. Quick Reference Table

| Problem | Fix |
|---------|-----|
| Response slow | ↑ Kp, ↑ Ki |
| Overshoot | ↓ Kp, ↓ Ki, ↑ Kd |
| Oscillation | ↓ Kp, ↑ Kd |
| Steady-state error | ↑ Ki |
| Noise in output | ↓ Kd |
| Instability | ↓ Kp, ↓ Ki, ↑ Kd |

## 10 Additional Notes

- Update PID at 100Hz or higher for better control
- Encoder resolution affects Kd sensitivity (low res → Kd more sensitive)
- Verify feedback sign (forward motion = positive feedback?)
- Re-tune if temperature changes significantly
- Motor characteristics vary with load

---

**Tip**: Start conservative, gradually make tuning aggressive!
