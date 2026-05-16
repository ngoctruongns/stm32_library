#pragma once

// Define peripheral configuration parameters
// Buzzer configuration
#define BUZZER_BLINK_ON_DURATION_MS 500
#define BUZZER_BLINK_OFF_DURATION_MS 500
#define BUZZER_BEEP_ON_DURATION_MS 200

// GPIO configuration
#define BUZZER_PORT BUZZER_GPIO_Port
#define BUZZER_PIN BUZZER_Pin

// Joystick loop interval
#define JOYSTICK_LOOP_INTERVAL_MS 50U

// Safety timeout: stop motors if no valid control command from joystick/UART3
#define CONTROL_CMD_TIMEOUT_MS   200U

// Sensor data logging interval
#define SENSOR_LOG_INTERVAL_MS   100U

// IMU odometry read interval (50 Hz) — must match UART3_FEEDBACK_PERIOD_MS
#define IMU_ODOM_INTERVAL_MS     20U