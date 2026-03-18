#include "peri_config.h"
#include "peri_control.h"
#include "main.h"

#include "PS2X_lib.h"
#include "buzzer.h"
#include "diff_drive.h"
#include "log_helper.h"
#include "ws2812.h"

void peripheral_init(void)
{
    // Buzzer ON to init
    Buzzer_Init(BUZZER_TYPE_BEEP, BUZZER_ON);

    diff_drive_init();
    WS2812_Init();

    set_log_level(LOG_LEVEL_INF);

    WS2812_SetSolidColor(LED_RED);

    for (int i = 0; i < 10; i++) {
        if (ps2x_init() == PS2X_SUCCESS) {
            LOG_DBG("PS2X Controller Initialized Successfully\r\n");
            break;
        } else if (i == 9) {
            LOG_ERR("Failed to Initialize PS2X Controller\r\n");
        }
    }

    LL_mDelay(100);
    WS2812_SetRainbow(100);
}

void peripheral_control_loop(void)
{
    float linear_vel = 0.0f;
    float angular_vel = 0.0f;
    static uint32_t last_ps2_update_ms = 0;

    WS2812_loopControl();
    Buzzer_Update();

    uint32_t now = get_ms_tick_count();
    if ((now - last_ps2_update_ms) >= JOYSTICK_LOOP_INTERVAL_MS) {
        last_ps2_update_ms = now;
        PS2X_CommandData cmd_data = ps2x_update_data();

        // If PS2X data is valid, update motor velocity and peripheral states
        if (cmd_data.status != PS2X_SUCCESS) {
            return;
        }

        linear_vel = cmd_data.linear_vel;
        angular_vel = cmd_data.angular_vel;
        diff_drive_set_velocity(linear_vel, angular_vel);

        // If buzzer or LED state change, update them
        if (cmd_data.buzzer_on) {
            Buzzer_Init(BUZZER_TYPE_BEEP, BUZZER_ON);
        }

        if (cmd_data.led_change) {
            WS2812_ChangeColorRing();
        }
    }

}

void peripheral_tim10_interrupt_handler(void)
{
    diff_drive_update();
    LL_GPIO_TogglePin(PID_CLK_GPIO_Port, PID_CLK_Pin);
}
