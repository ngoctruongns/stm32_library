#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "stm32f4xx.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_gpio.h"
#include <stdint.h>

/**
 * @enum MotionDirection
 * @brief Motor motion direction
 */
typedef enum
{
    MOTOR_STOP = 0,   // Stop
    MOTOR_FORWARD,    // Forward
    MOTOR_BACKWARD    // Backward
} MotionDirection;

/**
 * @class MotorDriver
 * @brief Class to control DC motor via L298 driver
 *
 * PWM is supplied to Enable pin (Timer 3 CH1)
 * IN1 and IN2 are used to select rotation direction
 */
class MotorDriver
{
public:
    /**
     * @brief Initialize Motor Driver for L298 circuit
     * @param tim Pointer to Timer (TIM3)
     * @param gpio_port GPIO Port containing IN1, IN2
     * @param in1_pin IN1 pin (direction control 1)
     * @param in2_pin IN2 pin (direction control 2)
     * @param pwm_max Maximum PWM value (timer auto load value)
     */
    MotorDriver(TIM_TypeDef *tim, GPIO_TypeDef *gpio_port, uint32_t in1_pin,
                uint32_t in2_pin, uint32_t pwm_max = 1000);

    /**
     * @brief Initialize motor driver
     * @return 0 on success, -1 on error
     */
    int32_t init(void);

    /**
     * @brief Start PWM
     * @return 0 on success, -1 on error
     */
    int32_t startPWM(void);

    /**
     * @brief Stop PWM
     * @return 0 on success, -1 on error
     */
    int32_t stopPWM(void);

    /**
     * @brief Set motion direction
     * @param direction Motion direction (FORWARD, BACKWARD, STOP)
     */
    void setDirection(MotionDirection direction);

    /**
     * @brief Set motor power (0-100%)
     * @param power Power as percentage (0 ~ 100)
     */
    void setPower(float power);

    /**
     * @brief Set motor speed with direction
     * @param speed Speed from -100 to 100 (negative = backward, positive = forward)
     */
    void setSpeed(float speed);

    /**
     * @brief Stop motor immediately (brake)
     */
    void brake(void);

    /**
     * @brief Get current motor speed (-100 to 100)
     * @return Current speed
     */
    float getSpeed(void) const { return _current_speed; }

    /**
     * @brief Get current motion direction
     * @return Motion direction
     */
    MotionDirection getDirection(void) const { return _direction; }

    /**
     * @brief Release resources
     */
    void deinit(void);

private:
    TIM_TypeDef *_tim;
    GPIO_TypeDef *_gpio_port;
    uint16_t _in1_pin;
    uint16_t _in2_pin;
    uint32_t _pwm_max;
    float _current_speed;
    MotionDirection _direction;

    /**
     * @brief Helper function to update PWM
     */
    void _updatePWM(uint32_t pulse);
};

#endif // MOTOR_DRIVER_H
