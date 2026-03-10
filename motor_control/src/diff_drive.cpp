
#include "motor_ctl.h"
#include "diff_drive.h"

// Global declarations for components
// Motor driver objects
MotorDriver leftDriver(MOTOR_PWM_TIMER, MOTOR_PWM_LEFT_CHANNEL, MOTOR_LEFT_IN1_PORT,
                      MOTOR_LEFT_IN1_PIN, MOTOR_LEFT_IN2_PORT, MOTOR_LEFT_IN2_PIN, MOTOR_LEFT_FLIP);
MotorDriver rightDriver(MOTOR_PWM_TIMER, MOTOR_PWM_RIGHT_CHANNEL, MOTOR_RIGHT_IN1_PORT,
                       MOTOR_RIGHT_IN1_PIN, MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN, MOTOR_RIGHT_FLIP);
// Encoder objects
Encoder leftEnc(ENCODER_LEFT_TIMER, ENCODER_PPR, MOTOR_LEFT_FLIP);
Encoder rightEnc(ENCODER_RIGHT_TIMER, ENCODER_PPR, MOTOR_RIGHT_FLIP);

// PID controller objects
PIDController leftPID(PID_KP, PID_KI, PID_KD);
PIDController rightPID(PID_KP, PID_KI, PID_KD);

// Motors control system
Motor leftMotor(&leftDriver, &leftEnc, &leftPID);
Motor rightMotor(&rightDriver, &rightEnc, &rightPID);

extern "C" {

/**
 * @brief Initialize differential drive system
 */
void diff_drive_init()
{
    // Initialize left motor
    if (leftMotor.init() != 0) {
        // Handle error - for now, just continue
    }

    // Initialize right motor
    if (rightMotor.init() != 0) {
        // Handle error - for now, just continue
    }
}

/**
 * @brief Set velocity for differential drive
 * @param linear_vel Linear velocity (m/s)
 * @param angular_vel Angular velocity (rad/s)
 */
void diff_drive_set_velocity(float linear_vel, float angular_vel)
{
    // Calculate wheel velocities
    float left_vel = linear_vel - angular_vel * (WHEEL_BASE / 2.0f);
    float right_vel = linear_vel + angular_vel * (WHEEL_BASE / 2.0f);

    // Convert to RPM
    // RPM = (velocity_mps * 60) / (PI * wheel_diameter)
    float left_rpm = (left_vel * 60.0f) / (3.14159f * WHEEL_DIAMETER);
    float right_rpm = (right_vel * 60.0f) / (3.14159f * WHEEL_DIAMETER);

    // Clamp wheel RPMs to configured maximum
    if (left_rpm > MOTOR_MAX_RPM) left_rpm = MOTOR_MAX_RPM;
    if (left_rpm < -MOTOR_MAX_RPM) left_rpm = -MOTOR_MAX_RPM;
    if (right_rpm > MOTOR_MAX_RPM) right_rpm = MOTOR_MAX_RPM;
    if (right_rpm < -MOTOR_MAX_RPM) right_rpm = -MOTOR_MAX_RPM;

    // Set target RPM for motors
    leftMotor.setTargetRPM(left_rpm);
    rightMotor.setTargetRPM(right_rpm);
}

/**
 * @brief Set PID gains for both left and right motors
 */
void diff_drive_set_pid(float kp, float ki, float kd)
{
    leftPID.setGains(kp, ki, kd);
    rightPID.setGains(kp, ki, kd);
    leftPID.reset();
    rightPID.reset();
}

/**
 * @brief Update differential drive control loop
 * @param dt Delta time since last update (seconds)
 */
void diff_drive_update(void)
{
    // Update PID controllers for both motors
    leftMotor.update();
    rightMotor.update();
}

/**
 * @brief Stop differential drive (set velocity to zero)
 */
void diff_drive_stop()
{
    // Set velocity to zero
    diff_drive_set_velocity(0.0f, 0.0f);
}

int32_t getEncoderCount(MotorNameT motor)
{
    if (motor == MOTOR_L) {
        return leftEnc.getRawCount();
    } else if (motor == MOTOR_R) {
        return rightEnc.getRawCount();
    } else {
        return 0; // Invalid motor, return 0
    }
}

float getCurrentRPM(MotorNameT motor)
{
    if (motor == MOTOR_L) {
        return leftEnc.getRPM();
    } else if (motor == MOTOR_R) {
        return rightEnc.getRPM();
    } else {
        return 0.0f; // Invalid motor, return 0
    }
}

float getTargetRPM(MotorNameT motor)
{
    if (motor == MOTOR_L) {
        return leftMotor.getTargetRPM();
    } else if (motor == MOTOR_R) {
        return rightMotor.getTargetRPM();
    } else {
        return 0.0f; // Invalid motor, return 0
    }
}

void setMotorRPM(MotorNameT motor, float rpm)
{
    if (motor == MOTOR_L) {
        leftMotor.setTargetRPM(rpm);
    } else if (motor == MOTOR_R) {
        rightMotor.setTargetRPM(rpm);
    }
}

/******************* END FILE *********************/
} // extern "C"
