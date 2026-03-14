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
     * @brief Set alpha factor for D term low-pass filter
     * @param alpha factor value (0.1 -0.3)
     */
    void setAlphaEMA(float alpha);

    /**
     * @brief Set output limits
     * @param out_min Minimum output value
     * @param out_max Maximum output value
     */
    void setOutputLimit(float out_min, float out_max);

    /**
     * @brief Set filtered setpoint for ramp filtering
     * @param sp Desired setpoint
     */
    void setSetpoint(float sp);

    /**
     * @brief Set maximum ramp step for setpoint filter
     * @param slope Maximum change of setpoint_filtered per call
     */
    void setSetpointSlope(float slope);

    /**
     * @brief Set deadzone for setpoint to avoid oscillation at low speed
     * @param deadzone Deadzone range around zero (e.g. 5 RPM)
     */
    void setSetpointDeadzone(float deadzone);

    /**
     * @brief Compute and return control value
     * @param setpoint Desired setpoint
     * @param feedback Current feedback value
     * @return Control signal (output)
     */
    float compute(float setpoint, float feedback);

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

private:
    float _kp, _ki, _kd;              // PID gains
    float _p_term, _i_term, _d_term;  // P, I, D terms
    float _last_feedback;             // Feedback from last call
    float _integral_sum;              // Accumulated error sum
    float _max_integral;              // Integral limit
    float _d_alpha;                   // Low-pass filter factor for D term
    float _d_filtered_rate;           // Error rate filter for D term
    float _setpoint_filtered;         // Filtered setpoint for ramping
    float _setpoint_slope;            // Max step for filtered setpoint per compute call
    float _out_min, _out_max;         // Output limits
    float _sp_deadzone;               // Deadzone for setpoint (RPM)
};

#endif // PID_CTL_H
