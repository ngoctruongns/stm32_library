#include "motor_driver.h"
#include "tim.h"

MotorDriver::MotorDriver(TIM_TypeDef *tim, uint32_t channel,
                         GPIO_TypeDef *in1_port, uint32_t in1_pin,
                         GPIO_TypeDef *in2_port, uint32_t in2_pin, bool flip) :
    _tim(tim), _channel(channel),
    _in1_port(in1_port), _in1_pin(in1_pin),
    _in2_port(in2_port), _in2_pin(in2_pin),
    _current_speed(0.0f), _direction(MOTOR_STOP), _flip(flip)
{
}

int32_t MotorDriver::init(void)
{
    if (_tim == nullptr || _in1_port == nullptr || _in2_port == nullptr)
        return -1;

    // Get timer PWM max value from timer auto reload register
    if (_tim != nullptr) {
        _pwm_max = (LL_TIM_GetAutoReload(_tim) + 1) * MOTOR_PWM_DUTY_MAX / 100.0f;
    }

    // Default is stop
    setDirection(MOTOR_STOP);

    // Start PWM
    LL_TIM_CC_EnableChannel(_tim, _channel);

    return 0;
}

int32_t MotorDriver::stopPWM(void)
{
    if (_tim == nullptr)
        return -1;

    // Disable PWM output on configured channel
    LL_TIM_CC_DisableChannel(_tim, _channel);
    return 0;
}

void MotorDriver::setDirection(MotionDirection direction)
{
    // Check with current direction and flip if needed
    if (_flip) {
        if (direction == MOTOR_FORWARD) {
            direction = MOTOR_BACKWARD;
        } else if (direction == MOTOR_BACKWARD) {
            direction = MOTOR_FORWARD;
        }
    }

    _direction = direction;

    switch (direction)
    {
    case MOTOR_FORWARD:
        LL_GPIO_SetOutputPin(_in1_port, _in1_pin);
        LL_GPIO_ResetOutputPin(_in2_port, _in2_pin);
        break;

    case MOTOR_BACKWARD:
        LL_GPIO_ResetOutputPin(_in1_port, _in1_pin);
        LL_GPIO_SetOutputPin(_in2_port, _in2_pin);
        break;

    case MOTOR_STOP:
    default:
        // Both pins reset - motor stops
        LL_GPIO_ResetOutputPin(_in1_port, _in1_pin);
        LL_GPIO_ResetOutputPin(_in2_port, _in2_pin);
        break;
    }
}

void MotorDriver::setPower(float power)
{
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

    // Set dead zone for small speed
    if (speed > -MOTOR_DEAD_ZONE && speed < MOTOR_DEAD_ZONE) {
        speed = 0.0f;
    }

    _current_speed = speed;

    if (speed > 0.0f) {
        setDirection(MOTOR_FORWARD);
        setPower(speed);
    } else if (speed < 0.0f) {
        setDirection(MOTOR_BACKWARD);
        setPower(-speed);
    } else {
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

void MotorDriver::_updatePWM(uint32_t pwm)
{
    TIMx_setPWM(_tim, _channel, pwm);
}
