#include "motor_ctl.h"

Motor::Motor(MotorDriver *driver, Encoder *encoder,
             PIDController *pid_controller)
    : _driver(driver), _encoder(encoder), _pid_controller(pid_controller),
      _target_rpm(0.0f)
{
}

int32_t Motor::init(void)
{
    if (_driver == nullptr || _encoder == nullptr || _pid_controller == nullptr)
        return -1;

    // Initialize driver
    int32_t status = _driver->init();
    if (status != 0)
        return status;

    // Initialize encoder
    status = _encoder->init();
    if (status != 0)
        return status;

    // Set low-pass filter alpha for encoder RPM
    _encoder->setRPMAlpha(ENCODER_RPM_ALPHA);

    // Reset PID
    _pid_controller->reset();
    _pid_controller->setIntegralLimit(PID_INTEGRAL_LIMIT);
    _pid_controller->setAlphaEMA(PID_ALPHA_EMA);

    return 0;
}

void Motor::setTargetRPM(float rpm)
{
    // enforce maximum allowed speed
    if (rpm > MOTOR_MAX_RPM) {
        rpm = MOTOR_MAX_RPM;
    } else if (rpm < -MOTOR_MAX_RPM) {
        rpm = -MOTOR_MAX_RPM;
    }

    _target_rpm = rpm;
}

void Motor::setTargetAngularVelocity(float rad_per_sec)
{
    // Convert from rad/s to RPM
    // 1 revolution = 2π rad, so: RPM = rad/s * 60 / 2π
    float rpm = rad_per_sec * 60.0f / 6.28318530718f;

    setTargetRPM(rpm);
}

void Motor::update(void)
{
    // Update speed from encoder
    _encoder->updateSpeed();

    // Get current speed
    float current_rpm = _encoder->getRPM();

    // Compute PID output
    float pid_output = _pid_controller->compute(_target_rpm, current_rpm);

    // Update motor power based on PID output
    _driver->setSpeed(pid_output);
}

float Motor::getCurrentRPM(void) const
{
    return _encoder->getRPM();
}

float Motor::getCurrentAngularVelocity(void) const
{
    return _encoder->getAngularVelocity();
}

float Motor::getSpeedError(void) const
{
    return _target_rpm - _encoder->getRPM();
}

void Motor::resetEncoder(void)
{
    _encoder->resetCount();
    _pid_controller->reset();
}

void Motor::stop(void)
{
    _target_rpm = 0.0f;
    _driver->brake();
    _pid_controller->reset();
}

void Motor::deinit(void)
{
    stop();
    _driver->deinit();
}
