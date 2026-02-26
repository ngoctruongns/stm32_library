#include "../inc/motor_driver.h"
#include <stdlib.h>

MotorDriver::MotorDriver(TIM_TypeDef *tim, GPIO_TypeDef *gpio_port,
                         uint32_t in1_pin, uint32_t in2_pin, uint32_t pwm_max)
    : _tim(tim), _gpio_port(gpio_port), _in1_pin(in1_pin), _in2_pin(in2_pin),
      _pwm_max(pwm_max), _current_speed(0.0f), _direction(MOTOR_STOP)
{
}

int32_t MotorDriver::init(void)
{
    if (_tim == nullptr || _gpio_port == nullptr)
        return -1;

    // Determine which channel and timer to use
    // TIM3 Channel 1 is used for PWM
    // Set compare value to 0 initially
    LL_TIM_OC_SetCompareCH1(_tim, 0);

    // Enable timer counter
    LL_TIM_EnableCounter(_tim);

    // Default is stop
    setDirection(MOTOR_STOP);

    return 0;
}

int32_t MotorDriver::startPWM(void)
{
    if (_tim == nullptr)
        return -1;

    // Enable PWM output
    LL_TIM_CC_EnableChannel(_tim, LL_TIM_CHANNEL_CH1);
    return 0;
}

int32_t MotorDriver::stopPWM(void)
{
    if (_tim == nullptr)
        return -1;

    // Disable PWM output
    LL_TIM_CC_DisableChannel(_tim, LL_TIM_CHANNEL_CH1);
    return 0;
}

void MotorDriver::setDirection(MotionDirection direction)
{
    _direction = direction;

    switch (direction)
    {
    case MOTOR_FORWARD:
        LL_GPIO_SetOutputPin(_gpio_port, _in1_pin);
        LL_GPIO_ResetOutputPin(_gpio_port, _in2_pin);
        break;

    case MOTOR_BACKWARD:
        LL_GPIO_ResetOutputPin(_gpio_port, _in1_pin);
        LL_GPIO_SetOutputPin(_gpio_port, _in2_pin);
        break;

    case MOTOR_STOP:
    default:
        // Both pins reset - motor stops
        LL_GPIO_ResetOutputPin(_gpio_port, _in1_pin);
        LL_GPIO_ResetOutputPin(_gpio_port, _in2_pin);
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
    stopPWM();
}

void MotorDriver::_updatePWM(uint32_t pulse)
{
    LL_TIM_OC_SetCompareCH1(_tim, pulse);
}
