#ifndef PID_CTL_H
#define PID_CTL_H

#include <stdint.h>
#include <string.h>

/**
 * @class PIDController
 * @brief Class implementing PID control algorithm
 *
 * Formula: u(t) = Kp*e(t) + Ki*∫e(dt) + Kd*de/dt
 * Where e(t) is the error between desired and current value
 */
class PIDController
{
public:
    /**
     * @brief Initialize PID controller
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    PIDController(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f);

    /**
     * @brief Set PID gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setGains(float kp, float ki, float kd);

    /**
     * @brief Set limit for accumulated error (Anti-Windup)
     * @param max_integral Maximum integral value
     */
    void setIntegralLimit(float max_integral);

    /**
     * @brief Set output limits
     * @param out_min Minimum output value
     * @param out_max Maximum output value
     */
    void setOutputLimit(float out_min, float out_max);

    /**
     * @brief Compute and return control value
     * @param setpoint Desired setpoint
     * @param feedback Current feedback value
     * @param dt Delta time since last call (seconds)
     * @return Control signal (output)
     */
    float compute(float setpoint, float feedback, float dt);

    /**
     * @brief Reset internal states
     */
    void reset(void);

    /**
     * @brief Get P term value
     * @return P value
     */
    float getP(void) const { return _p_term; }

    /**
     * @brief Get I term value
     * @return I value
     */
    float getI(void) const { return _i_term; }

    /**
     * @brief Get D term value
     * @return D value
     */
    float getD(void) const { return _d_term; }

    /**
     * @brief Get current error value
     * @return Error value
     */
    float getError(void) const { return _last_error; }

private:
    float _kp, _ki, _kd;              // PID gains
    float _p_term, _i_term, _d_term;  // P, I, D terms
    float _last_error;                // Error from last call
    float _integral_sum;              // Accumulated error sum
    float _max_integral;              // Integral limit
    float _out_min, _out_max;         // Output limits
};

#endif // PID_CTL_H
