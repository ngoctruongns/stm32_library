/**
 * @file motor_example.cpp
 * @brief Example usage of DC motor control system with PID
 *
 * This is an example demonstrating how to use Motor, MotorDriver, Encoder, PIDController classes
 *
 * Usage: Copy this code into main.c or create a separate task to run
#include "../inc/motor_ctl.h"
#include "../config/motor_config.h"

// Global declarations for components
static Encoder *g_encoder = nullptr;
static PIDController *g_pid = nullptr;
static MotorDriver *g_driver = nullptr;
static Motor *g_motor = nullptr;

/**
 * @brief Initialize motor system
 *
 * Call this function once in main() or during startup
 * Assumes:
 * - htim2 is Timer 2 (for encoder, Quadrature mode)
 * - htim3 is Timer 3 Channel 1 (for PWM)
 * - GPIOA is configured: PA4 (IN1), PA5 (IN2)
 */
void motor_system_init(TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3)
{
    // 1. Tạo đối tượng Encoder
    g_encoder = new Encoder(htim2, ENCODER_PPR);
    if (g_encoder == nullptr)
        return;

    // 2. Tạo đối tượng PID Controller
    g_pid = new PIDController(PID_KP, PID_KI, PID_KD);
    if (g_pid == nullptr)
    {
        delete g_encoder;
        return;
    }
    g_pid->setIntegralLimit(PID_INTEGRAL_LIMIT);
    g_pid->setOutputLimit(PID_OUT_MIN, PID_OUT_MAX);

    // 3. Tạo đối tượng Motor Driver
    g_driver = new MotorDriver(htim3, TIM_CHANNEL_1,
                                MOTOR_IN1_PORT, MOTOR_IN1_PIN,
                                MOTOR_IN2_PORT, MOTOR_IN2_PIN,
                                MOTOR_PWM_MAX);
    if (g_driver == nullptr)
    {
        delete g_encoder;
        delete g_pid;
        return;
    }

    // 4. Tạo đối tượng Motor (điều khiển chính)
    g_motor = new Motor(g_driver, g_encoder, g_pid);
    if (g_motor == nullptr)
    {
        delete g_encoder;
        delete g_pid;
        delete g_driver;
        return;
    }

    // 5. Khởi tạo hệ thống
    HAL_StatusTypeDef status = g_motor->init();
    if (status != HAL_OK)
    {
        // Error handling
        motor_system_deinit();
    }
}

/**
 * @brief Cập nhật bộ điều khiển motor
 *
 * Gọi hàm này định kỳ trong vòng lặp chính hoặc trong timer interrupt
 * Tần suất gọi nên phù hợp với MOTOR_UPDATE_FREQ (mặc định 100 Hz = 10ms)
 */
void motor_system_update(void)
{
    if (g_motor != nullptr)
    {
        float dt = 1.0f / MOTOR_UPDATE_FREQ;  // Delta time
        g_motor->update(dt);
    }
}

/**
 * @brief Set target speed for motor (RPM)
 * @param rpm Desired speed (RPM)
 */
void motor_set_speed_rpm(float rpm)
{
    if (g_motor != nullptr)
    {
        // Limit speed
        if (rpm > MOTOR_MAX_RPM)
            rpm = MOTOR_MAX_RPM;
        if (rpm < -MOTOR_MAX_RPM)
            rpm = -MOTOR_MAX_RPM;

        g_motor->setTargetRPM(rpm);
    }
}

/**
 * @brief Set target speed via angular velocity (rad/s)
 * @param rad_per_sec Desired angular velocity (rad/s)
 */
void motor_set_speed_rad(float rad_per_sec)
{
    if (g_motor != nullptr)
    {
        g_motor->setTargetAngularVelocity(rad_per_sec);
    }
}

/**
 * @brief Get current motor speed (RPM)
 * @return Current speed
 */
float motor_get_current_rpm(void)
{
    if (g_motor != nullptr)
        return g_motor->getCurrentRPM();
    return 0.0f;
}

/**
 * @brief Get current angular velocity (rad/s)
 * @return Angular velocity
 */
float motor_get_current_angular_velocity(void)
{
    if (g_motor != nullptr)
        return g_motor->getCurrentAngularVelocity();
    return 0.0f;
}

/**
 * @brief Get current output power (%)
 * @return Output power
 */
float motor_get_current_power(void)
{
    if (g_motor != nullptr)
        return g_motor->getCurrentPower();
    return 0.0f;
}

/**
 * @brief Stop motor
 */
void motor_stop(void)
{
    if (g_motor != nullptr)
    {
        g_motor->stop();
    }
}

/**
 * @brief Release motor system resources
 */
void motor_system_deinit(void)
{
    if (g_motor != nullptr)
    {
        delete g_motor;
        g_motor = nullptr;
    }
    if (g_driver != nullptr)
    {
        delete g_driver;
        g_driver = nullptr;
    }
    if (g_pid != nullptr)
    {
        delete g_pid;
        g_pid = nullptr;
    }
    if (g_encoder != nullptr)
    {
        delete g_encoder;
        g_encoder = nullptr;
    }
}

// ============= VÍ DỤ SỬ DỤNG TRONG MAIN =============
/*

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Gọi mỗi 10ms từ timer interrupt (100 Hz)
    if (htim->Instance == TIM4)  // Giả sử dùng Timer 4 để cập nhật
    {
        motor_system_update();
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();   // Encoder
    MX_TIM3_Init();   // PWM
    MX_TIM4_Init();   // Cập nhật điều khiển (100 Hz)
    MX_USART2_UART_Init();

    // Khởi tạo motor system
    motor_system_init(&htim2, &htim3);

    // Bắt đầu timer cập nhật
    HAL_TIM_Base_Start_IT(&htim4);

    while (1)
    {
        // Ví dụ: Thiết lập tốc độ 500 RPM tiến
        motor_set_speed_rpm(500.0f);

        // Hoặc quay lùi 300 RPM
        // motor_set_speed_rpm(-300.0f);

        // Dừng motor
        // motor_stop();

        // Đọc giá trị hiện tại
        float rpm = motor_get_current_rpm();
        float power = motor_get_current_power();

        // Làm thêm các xử lý khác...
    }

    motor_system_deinit();
    return 0;
}

*/
