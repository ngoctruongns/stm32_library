/**
 * @file motor_config.h
 * @brief DC motor parameter configuration file
 *
 * Edit these values to match your hardware specifications
 */

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "main.h"

// ============= ENCODER CONFIGURATION =============
/** Number of pulses per revolution of encoder */
#define ENCODER_PPR 330

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
#define MOTOR_PWM_FREQ_HZ   1000 // 1 kHz PWM frequency, see config with CubueMX timer settings
#define MOTOR_PWM_DUTY_MAX  80.0f // Max duty cycle percentage (0-100%)

/** Controller update frequency (Hz) - from main loop */
#define MOTOR_UPDATE_FREQ   100  // 100 Hz = dt = 0.01s

/** Motor maximum speed (RPM) */
#define MOTOR_MAX_RPM       200   // Motor JBG-520, 330 RPM

/** Flip left motor direction (true if motor is wired in reverse) */
#define MOTOR_LEFT_FLIP       true

/** Flip right motor direction (true if motor is wired in reverse) */
#define MOTOR_RIGHT_FLIP      false

/*** Dead zone for small motor speeds (percentage) */
#define MOTOR_DEAD_ZONE       5.0f

// ============= GPIO L298 CONFIGURATION =============
// Left Motor  -> IN1, IN2
// Right Motor -> IN3, IN4
#define MOTOR_LEFT_IN1_PIN      L298_IN1_Pin
#define MOTOR_LEFT_IN1_PORT     L298_IN1_GPIO_Port
#define MOTOR_LEFT_IN2_PIN      L298_IN2_Pin
#define MOTOR_LEFT_IN2_PORT     L298_IN2_GPIO_Port
#define MOTOR_RIGHT_IN1_PIN     L298_IN3_Pin
#define MOTOR_RIGHT_IN1_PORT    L298_IN3_GPIO_Port
#define MOTOR_RIGHT_IN2_PIN     L298_IN4_Pin
#define MOTOR_RIGHT_IN2_PORT    L298_IN4_GPIO_Port

// ============= TIMER CONFIGURATION =============
// Timer 2: Encoder Left (Quadrature mode)
// Timer 5: Encoder Right (Quadrature mode)
#define ENCODER_LEFT_TIMER      TIM2
#define ENCODER_RIGHT_TIMER     TIM5

// Timer 3: PWM Motor Left (Channel 1 for Enable pin)
// Timer 3: PWM Motor Right (Channel 4 for Enable pin)
#define MOTOR_PWM_TIMER         TIM3
#define MOTOR_PWM_LEFT_CHANNEL  LL_TIM_CHANNEL_CH1
#define MOTOR_PWM_RIGHT_CHANNEL LL_TIM_CHANNEL_CH4

// Timer 10: Control loop update timer (100 Hz)
#define CONTROL_LOOP_TIMER      TIM10

// ============= DIFFERENTIAL DRIVE CONFIGURATION =============
/** Distance between the two wheels (meters) */
#define WHEEL_BASE          0.252f
/** Wheel diameter in meters */
#define WHEEL_DIAMETER      0.067f


#endif // MOTOR_CONFIG_H
