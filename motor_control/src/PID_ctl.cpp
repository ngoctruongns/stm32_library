#include "PID_ctl.h"

PIDController::PIDController(float kp, float ki, float kd)
    : _kp(kp), _ki(ki), _kd(kd), _p_term(0.0f), _i_term(0.0f),
      _d_term(0.0f), _last_error(0.0f), _integral_sum(0.0f),
      _max_integral(100.0f), _out_min(-100.0f), _out_max(100.0f)
{
}

void PIDController::setGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setIntegralLimit(float max_integral)
{
    _max_integral = max_integral;
}

void PIDController::setOutputLimit(float out_min, float out_max)
{
    _out_min = out_min;
    _out_max = out_max;
}

float PIDController::compute(float setpoint, float feedback, float dt)
{
    if (dt <= 0.0f)
        return 0.0f;

    // P term
    float error = setpoint - feedback;
    _p_term = _kp * error;

    // I term (with Anti-Windup)
    _integral_sum += error * dt;
    if (_integral_sum > _max_integral)
        _integral_sum = _max_integral;
    if (_integral_sum < -_max_integral)
        _integral_sum = -_max_integral;
    _i_term = _ki * _integral_sum;

    // D term
    float error_rate = (error - _last_error) / dt;
    _d_term = _kd * error_rate;
    _last_error = error;

    // Compute output
    float output = _p_term + _i_term + _d_term;

    // Limit output
    if (output > _out_max)
        output = _out_max;
    if (output < _out_min)
        output = _out_min;

    return output;
}

void PIDController::reset(void)
{
    _p_term = 0.0f;
    _i_term = 0.0f;
    _d_term = 0.0f;
    _last_error = 0.0f;
    _integral_sum = 0.0f;
}
