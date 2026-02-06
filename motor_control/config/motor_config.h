/**
 * @file motor_config.h
 * @brief DC motor parameter configuration file
 *
 * Edit these values to match your hardware specifications
 */

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// ============= ENCODER CONFIGURATION =============
/** Number of pulses per revolution of encoder */
#define ENCODER_PPR 20

// ============= PID CONFIGURATION =============
/** Proportional coefficient */
#define PID_KP 2.0f

/** Integral coefficient */
#define PID_KI 0.5f

/** Derivative coefficient */
#define PID_KD 0.1f

/** Anti-Windup integral limit */
#define PID_INTEGRAL_LIMIT 50.0f

/** Output min/max limits */
#define PID_OUT_MIN -100.0f
#define PID_OUT_MAX 100.0f

// ============= MOTOR DRIVER CONFIGURATION =============
/** Maximum PWM value (must match TMAR of Timer 3) */
#define MOTOR_PWM_MAX 1000

/** Controller update frequency (Hz) - from main loop */
#define MOTOR_UPDATE_FREQ 100  // 100 Hz = dt = 0.01s

/** Motor maximum speed (RPM) */
#define MOTOR_MAX_RPM 1000

// ============= GPIO L298 CONFIGURATION =============
// These pins must be configured in STM32CubeMX
#define MOTOR_IN1_PORT GPIOA
#define MOTOR_IN1_PIN GPIO_PIN_4

#define MOTOR_IN2_PORT GPIOA
#define MOTOR_IN2_PIN GPIO_PIN_5

// ============= TIMER CONFIGURATION =============
// Timer 2: Encoder (Quadrature mode)
// Timer 3: PWM (Channel 1 for Enable pin)

#endif // MOTOR_CONFIG_H
