#include "PID_ctl.h"

PIDController::PIDController(float kp, float ki, float kd)
    : _kp(kp), _ki(ki), _kd(kd), _p_term(0.0f), _i_term(0.0f), _d_term(0.0f), _last_feedback(0.0f),
      _integral_sum(0.0f), _max_integral(100.0f), _d_alpha(0.1f), _d_filtered_rate(0.0f),
      _setpoint_filtered(0.0f), _setpoint_slope(5.0f), _out_min(-100.0f), _out_max(100.0f),
      _sp_deadzone(5.0f)
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

void PIDController::setAlphaEMA(float alpha)
{
    _d_alpha = alpha;
}

void PIDController::setOutputLimit(float out_min, float out_max)
{
    _out_min = out_min;
    _out_max = out_max;
}

void PIDController::setSetpoint(float sp)
{
    _setpoint_filtered = sp;
}

void PIDController::setSetpointSlope(float slope)
{
    _setpoint_slope = slope;
}

void PIDController::setSetpointDeadzone(float deadzone)
{
    _sp_deadzone = deadzone;
}

float PIDController::compute(float setpoint, float feedback)
{
    // Check if setpoint is within deadzone then return 0 to avoid oscillation at low speed
    if (setpoint > -_sp_deadzone && setpoint < _sp_deadzone) {
        reset();
        return 0.0f;
    }

    // Ramp setpoint by filtered value to avoid sudden jumps
    float setpoint_delta = setpoint - _setpoint_filtered;
    float clamped_delta = setpoint_delta;
    if (clamped_delta > _setpoint_slope)
        clamped_delta = _setpoint_slope;
    else if (clamped_delta < -_setpoint_slope)
        clamped_delta = -_setpoint_slope;
    _setpoint_filtered += clamped_delta;

    /************  P term ************/
    float error = _setpoint_filtered - feedback;
    _p_term = _kp * error;

    /************  I term ************/
    _integral_sum += error;
    _i_term = _ki * _integral_sum;

    // I term with Anti-Windup
    if (_i_term > _max_integral) _i_term = _max_integral;
    if (_i_term < -_max_integral) _i_term = -_max_integral;

    // Back-calculation
    if (_ki != 0.0f) // Avoid division by zero
        _integral_sum = _i_term / _ki;

    /************  D term ************/
    // Used Feedback to avoid "Derivative Kick" when change Setpoint
    float raw_error_rate = _last_feedback - feedback;
    _last_feedback = feedback;

    // Low-pass filter (EMA)
    _d_filtered_rate = (_d_alpha * raw_error_rate) + (1.0f - _d_alpha) * _d_filtered_rate;
    _d_term = _kd * _d_filtered_rate;

    // Compute output
    float output = _p_term + _i_term + _d_term;

    // Limit output
    if (output > _out_max)
        output = _out_max;
    if (output < _out_min)
        output = _out_min;

    // Ensure output direction matches setpoint direction
    if (setpoint > 0.0f && output < 0.0f)
        output = 0.0f;
    else if (setpoint < 0.0f && output > 0.0f)
        output = 0.0f;
    else if (setpoint == 0.0f)
        output = 0.0f;

    return output;
}

void PIDController::reset(void)
{
    _p_term = 0.0f;
    _i_term = 0.0f;
    _d_term = 0.0f;
    _d_filtered_rate = 0.0f;
    _integral_sum = 0.0f;
    _last_feedback = 0.0f;
    _setpoint_filtered = 0.0f;
}
