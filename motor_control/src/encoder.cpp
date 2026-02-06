#include "../inc/encoder.h"
#include <math.h>

#define PI 3.14159265359f

Encoder::Encoder(TIM_HandleTypeDef *htim_encoder, uint16_t ppr)
    : _htim_encoder(htim_encoder), _ppr(ppr), _last_count(0),
      _angular_velocity(0.0f), _rpm(0.0f)
{
}

HAL_StatusTypeDef Encoder::init(void)
{
    if (_htim_encoder == nullptr)
        return HAL_ERROR;

    // Start timer in Quadrature Encoder mode
    return HAL_TIM_Encoder_Start(_htim_encoder, TIM_CHANNEL_ALL);
}

int32_t Encoder::getRawCount(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(_htim_encoder);
}

void Encoder::resetCount(void)
{
    __HAL_TIM_SET_COUNTER(_htim_encoder, 0);
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
