#include "../inc/encoder.h"
#include <math.h>

#define PI 3.14159265359f

Encoder::Encoder(TIM_TypeDef *tim_encoder, uint16_t ppr)
    : _tim_encoder(tim_encoder), _ppr(ppr), _last_count(0),
      _angular_velocity(0.0f), _rpm(0.0f)
{
}

int32_t Encoder::init(void)
{
    if (_tim_encoder == nullptr)
        return -1;

    // Initialize TIM2 as encoder timer
    // Configure as encoder mode with both edges
    LL_TIM_SetEncoderMode(_tim_encoder, LL_TIM_ENCODERMODE_X4);

    // Set auto-reload value to max (for 16-bit timer)
    LL_TIM_SetAutoReload(_tim_encoder, 0xFFFF);

    // Reset counter to 0
    LL_TIM_SetCounter(_tim_encoder, 0);

    // Enable counter
    LL_TIM_EnableCounter(_tim_encoder);

    return 0;
}

int32_t Encoder::getRawCount(void)
{
    return (int32_t)LL_TIM_GetCounter(_tim_encoder);
}

void Encoder::resetCount(void)
{
    LL_TIM_SetCounter(_tim_encoder, 0);
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

void Encoder::updateSpeed(float dt)
{
    if (dt <= 0.0f)
        return;

    int32_t current_count = getRawCount();
    int32_t delta_count = current_count - _last_count;
    _last_count = current_count;

    // Calculate angular velocity: rad/s
    // delta_count / ppr = number of revolutions
    // number of revolutions * 2π = radians
    float delta_revolutions = (float)delta_count / _ppr;
    _angular_velocity = (delta_revolutions * 2.0f * PI) / dt;

    // Calculate RPM
    // RPM = (pulses / ppr / dt) * 60
    _rpm = (delta_revolutions / dt) * 60.0f;
}

float Encoder::getAngularVelocity(void)
{
    return _angular_velocity;
}

float Encoder::getRPM(void)
{
    return _rpm;
}
