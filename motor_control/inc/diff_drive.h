#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Max linear velocity (m/s) - for safety, set to a reasonable value
#define MAX_LINEAR_VEL      0.3f

typedef enum
{
    MOTOR_L = 0,
    MOTOR_R,
    MOTOR_MAX
} MotorNameT;

/**
 * @brief Initialize the differential drive system
 *
 * This function initializes both left and right motors, including their drivers,
 * encoders, and PID controllers. It should be called once at system startup.
 */
void diff_drive_init();

/**
 * @brief Set the velocity of the differential drive robot
 *
 * This function sets the desired linear and angular velocities for the robot.
 * The velocities are converted to individual wheel speeds using differential drive kinematics.
 *
 * @param linear_vel Linear velocity in meters per second (m/s)
 * @param angular_vel Angular velocity in radians per second (rad/s)
 */
void diff_drive_set_velocity(float linear_vel, float angular_vel);

/**
 * @brief Update the differential drive control loop
 *
 * This function updates the PID controllers for both motors. It should be called
 * periodically in the main control loop to maintain desired velocities.
 *
 * @param dt Time elapsed since last update in seconds
 */
void diff_drive_update(float dt);

/**
 * @brief Stop the differential drive robot
 *
 * This function immediately stops the robot by setting both linear and angular
 * velocities to zero.
 */
void diff_drive_stop();

/**
 * @brief Get the raw encoder count for a specific motor
 *
 * @param motor Motor name (MOTOR_L or MOTOR_R)
 * @return Raw encoder count
 */
int32_t getEncoderCount(MotorNameT motor);

/**
 * @brief Get the current RPM for a specific motor
 *
 * @param motor Motor name (MOTOR_L or MOTOR_R)
 * @return Current RPM
 */
float getCurrentRPM(MotorNameT motor);

/**
 * @brief Get the target RPM setpoint for a specific motor
 *
 * @param motor Motor name (MOTOR_L or MOTOR_R)
 * @return Target RPM setpoint
 */
float getTargetRPM(MotorNameT motor);

#ifdef __cplusplus
}
#endif
