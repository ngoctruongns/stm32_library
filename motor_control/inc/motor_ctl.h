#ifndef MOTOR_CTL_H
#define MOTOR_CTL_H

#include "motor_driver.h"
#include "encoder.h"
#include "PID_ctl.h"

/**
 * @class Motor
 * @brief High-level DC motor control class
 *
 * This is the main controller class integrating Encoder, PIDController and MotorDriver
 * to achieve DC motor speed control via PID
 */
class Motor
{
public:
    /**
     * @brief Initialize Motor Controller
     * @param driver Pointer to Motor Driver (will manage lifecycle)
     * @param encoder Pointer to Encoder (will manage lifecycle)
     * @param pid_controller Pointer to PID Controller (will manage lifecycle)
     */
    Motor(MotorDriver *driver, Encoder *encoder, PIDController *pid_controller);

    /**
     * @brief Initialize entire motor system
     * @return 0 on success, -1 on error
     */
    int32_t init(void);

    /**
     * @brief Set desired speed setpoint
     * @param rpm Desired speed (RPM)
     */
    void setTargetRPM(float rpm);

    /**
     * @brief Set desired speed via angular velocity
     * @param rad_per_sec Desired angular velocity (rad/s)
     */
    void setTargetAngularVelocity(float rad_per_sec);

    /**
     * @brief Update PID controller (should be called in main loop)
     */
    void update(void);

    /**
     * @brief Get current speed
     * @return Current speed (RPM)
     */
    float getCurrentRPM(void) const;

    /**
     * @brief Get current angular velocity
     * @return Angular velocity (rad/s)
     */
    float getCurrentAngularVelocity(void) const;

    /**
     * @brief Get speed error
     * @return Error value (RPM)
     */
    float getSpeedError(void) const;

    /**
     * @brief Reset encoder
     */
    void resetEncoder(void);

    /**
     * @brief Stop motor
     */
    void stop(void);

    /**
     * @brief Release resources
     */
    void deinit(void);

    /**
     * @brief Get pointer to driver
     */
    MotorDriver *getDriver(void) { return _driver; }

    /**
     * @brief Get pointer to encoder
     */
    Encoder *getEncoder(void) { return _encoder; }

    /**
     * @brief Get pointer to PID controller
     */
    PIDController *getPIDController(void) { return _pid_controller; }

    /**
     * @brief Get target RPM setpoint
     * @return Target RPM
     */
    float getTargetRPM(void) const { return _target_rpm; }

private:
    MotorDriver *_driver;
    Encoder *_encoder;
    PIDController *_pid_controller;
    float _target_rpm;
};

#endif // MOTOR_CTL_H
