#ifndef ENCODER_H
#define ENCODER_H

#include "motor_config.h"

/**
 * @class Encoder
 * @brief Class to read encoder values from Timer 2
 *
 * Timer 2 is configured in Quadrature (XOR) mode
 * to read incremental/decremental encoder pulses from DC motor
 */
class Encoder
{
public:
    /**
     * @brief Initialize Encoder
     * @param tim Pointer to Timer configuration
     * @param ppr Pulses Per Revolution - pulses per revolution
     * @param flip Whether to flip direction (if encoder is mounted in reverse)
     */
    Encoder(TIM_TypeDef *tim, uint16_t ppr,  bool flip = false);

    /**
     * @brief Initialize encoder timer
     * @return 0 on success, -1 on error
     */
    int32_t init(void);

    /**
     * @brief Read current counter value from timer
     * @return int32_t Counter value (can be negative if rotating backward)
     */
    int32_t getRawCount(void);

    /**
     * @brief Reset counter value to 0
     */
    void resetCount(void);

    /**
     * @brief Get number of revolutions since last reset
     * @return Number of revolutions (can be negative)
     */
    float getRevolutions(void);

    /**
     * @brief Get rotation angle since last reset (degrees)
     * @return Rotation angle (degrees)
     */
    float getAngleDegree(void);

    /**
     * @brief Get rotation angle since last reset (radians)
     * @return Rotation angle (radians)
     */
    float getAngleRadian(void);

    /**
     * @brief Update encoder speed (should be called periodically)
     * @param dt Delta time since last call (seconds)
     */
    void updateSpeed(float dt);

    /**
     * @brief Get current angular velocity
     * @return Angular velocity (rad/s)
     */
    float getAngularVelocity(void);

    /**
     * @brief Get speed in revolutions per minute
     * @return RPM
     */
    float getRPM(void);

private:
    TIM_TypeDef *_tim;
    uint16_t _ppr;                    // Pulses Per Revolution
    int32_t _last_count;              // Last counter value
    float _angular_velocity;          // rad/s
    float _rpm;                       // RPM
    bool _flip;                       // Flip direction if encoder is mounted in reverse
};

#endif // ENCODER_H
