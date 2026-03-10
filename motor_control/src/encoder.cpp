#include "encoder.h"
#include <math.h>

#define PI 3.14159265359f

Encoder::Encoder(TIM_TypeDef *tim, uint16_t ppr, bool flip)
    : _tim(tim), _last_count(0),
      _angular_velocity(0.0f), _rpm(0.0f), _flip(flip), _rpm_alpha(1.0f)
{
    _ppr = ppr * 4; // Quadrature mode counts 4 times per pulse
}

int32_t Encoder::init(void)
{
    if (_tim == nullptr)
        return -1;

    // Reset counter to 0
    LL_TIM_SetCounter(_tim, 0);

    // Enable counter
    LL_TIM_EnableCounter(_tim);

    // Enable PID control loop timer
    LL_TIM_EnableCounter(CONTROL_LOOP_TIMER);

    return 0;
}

void Encoder::setRPMAlpha(float alpha)
{
    if (alpha < 0.0f) {
        _rpm_alpha = 0.0f;
    } else if (alpha > 1.0f) {
        _rpm_alpha = 1.0f;
    } else {
        _rpm_alpha = alpha;
    }
}

int32_t Encoder::getRawCount(void)
{
    // Check flip and return count accordingly
    int32_t count = (int32_t)LL_TIM_GetCounter(_tim);
    return _flip ? -count : count;
}

void Encoder::resetCount(void)
{
    LL_TIM_SetCounter(_tim, 0);
    _last_count = 0;
    _angular_velocity = 0.0f;
    _rpm = 0.0f;
}

float Encoder::getRevolutions(void)
{
    int32_t counts = getRawCount();
    return (float)counts / _ppr;
}

float Encoder::getAngleDegree(void)
{
    return getRevolutions() * 360.0f;
}

float Encoder::getAngleRadian(void)
{
    return getRevolutions() * 2.0f * PI;
}

// Update speed with RPM
void Encoder::updateSpeed(void)
{
    int32_t current_count = getRawCount();
    int32_t delta_count = current_count - _last_count;
    _last_count = current_count;

    // Calculate angular velocity: rad/s
    // delta_count / ppr = number of revolutions
    // number of revolutions * 2π = radians
    float delta_revolutions = (float)delta_count / _ppr;
    _angular_velocity = (delta_revolutions * 2.0f * PI) * PID_CONTROL_FREQ;

    // Calculate RPM
    // RPM = (pulses / ppr / dt) * 60
    float raw_rpm = delta_revolutions * 60.0f * PID_CONTROL_FREQ;

    // Low-pass filter (EMA) on RPM to reduce noise before PID
    // _rpm_alpha == 1.0f -> no filtering, 0.0f -> keep previous value
    _rpm = (_rpm_alpha * raw_rpm) + ((1.0f - _rpm_alpha) * _rpm);
}

float Encoder::getAngularVelocity(void)
{
    return _angular_velocity;
}

float Encoder::getRPM(void)
{
    return _rpm;
}
