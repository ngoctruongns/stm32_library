#include "../inc/motor_driver.h"
#include <stdlib.h>

MotorDriver::MotorDriver(TIM_HandleTypeDef *htim, uint32_t pwm_channel,
                         GPIO_TypeDef *gpio_port, uint16_t in1_pin,
                         uint16_t in2_pin, uint32_t pwm_max)
    : _htim(htim), _pwm_channel(pwm_channel), _gpio_port(gpio_port),
      _in1_pin(in1_pin), _in2_pin(in2_pin), _pwm_max(pwm_max),
      _current_speed(0.0f), _direction(MOTOR_STOP)
{
}

HAL_StatusTypeDef MotorDriver::init(void)
{
    if (_htim == nullptr || _gpio_port == nullptr)
        return HAL_ERROR;

    // Start PWM on channel
    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(_htim, _pwm_channel);
    if (status != HAL_OK)
        return status;

    // Reset PWM to 0
    _updatePWM(0);

    // Default is stop
    setDirection(MOTOR_STOP);

    return HAL_OK;
}

HAL_StatusTypeDef MotorDriver::startPWM(void)
{
    return HAL_TIM_PWM_Start(_htim, _pwm_channel);
}

HAL_StatusTypeDef MotorDriver::stopPWM(void)
{
    return HAL_TIM_PWM_Stop(_htim, _pwm_channel);
}

void MotorDriver::setDirection(MotionDirection direction)
{
    _direction = direction;

    switch (direction)
    {
    case MOTOR_FORWARD:
        HAL_GPIO_WritePin(_gpio_port, _in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(_gpio_port, _in2_pin, GPIO_PIN_RESET);
        break;

    case MOTOR_BACKWARD:
        HAL_GPIO_WritePin(_gpio_port, _in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(_gpio_port, _in2_pin, GPIO_PIN_SET);
        break;

    case MOTOR_STOP:
    default:
        // Both pins reset - motor stops
        HAL_GPIO_WritePin(_gpio_port, _in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(_gpio_port, _in2_pin, GPIO_PIN_RESET);
        break;
    }
}

void MotorDriver::setPower(float power)
{
    // Limit power from 0 to 100
    if (power > 100.0f)
        power = 100.0f;
    if (power < 0.0f)
        power = 0.0f;

    _current_speed = power;

    // Calculate PWM value from power %
    uint32_t pulse = (uint32_t)((_pwm_max * power) / 100.0f);
    _updatePWM(pulse);
}

void MotorDriver::setSpeed(float speed)
{
    // Speed from -100 to 100
    if (speed > 100.0f)
        speed = 100.0f;
    if (speed < -100.0f)
        speed = -100.0f;

    _current_speed = speed;

    if (speed > 0.0f)
    {
        setDirection(MOTOR_FORWARD);
        setPower(speed);
    }
    else if (speed < 0.0f)
    {
        setDirection(MOTOR_BACKWARD);
        setPower(-speed);
    }
    else
    {
        setDirection(MOTOR_STOP);
        _updatePWM(0);
    }
}

void MotorDriver::brake(void)
{
    setDirection(MOTOR_STOP);
    _updatePWM(0);
    _current_speed = 0.0f;
}

void MotorDriver::deinit(void)
{
    brake();
    HAL_TIM_PWM_Stop(_htim, _pwm_channel);
}

void MotorDriver::_updatePWM(uint32_t pulse)
{
    __HAL_TIM_SET_COMPARE(_htim, _pwm_channel, pulse);
}
