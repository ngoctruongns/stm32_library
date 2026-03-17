// Implementation of buzzer control functions
#include "buzzer.h"

#include "main.h"
#include "peri_config.h"

static BuzzerConfig buzzer_cfg = {
    .type = BUZZER_TYPE_SOLID,
    .state = BUZZER_OFF,
    .on_duration_ms = BUZZER_BLINK_ON_DURATION_MS,
    .off_duration_ms = BUZZER_BLINK_OFF_DURATION_MS,
    .last_toggle_time_ms = 0U,
};

static void buzzer_apply_state(BuzzerState state)
{
    if (state == BUZZER_ON) {
        LL_GPIO_SetOutputPin(BUZZER_PORT, BUZZER_PIN);
    } else {
        LL_GPIO_ResetOutputPin(BUZZER_PORT, BUZZER_PIN);
    }
}

void Buzzer_Init(BuzzerType type, BuzzerState state)
{
    buzzer_cfg.type = (type < BUZZER_TYPE_MAX) ? type : BUZZER_TYPE_SOLID;
    buzzer_cfg.state = (state == BUZZER_ON) ? BUZZER_ON : BUZZER_OFF;
    buzzer_cfg.on_duration_ms = BUZZER_BLINK_ON_DURATION_MS;
    buzzer_cfg.off_duration_ms = BUZZER_BLINK_OFF_DURATION_MS;
    buzzer_cfg.last_toggle_time_ms = get_ms_tick_count();

    if (buzzer_cfg.type == BUZZER_TYPE_BEEP) {
        buzzer_cfg.on_duration_ms = BUZZER_BEEP_ON_DURATION_MS;
        buzzer_cfg.off_duration_ms = BUZZER_BEEP_ON_DURATION_MS;
    }

    buzzer_apply_state(buzzer_cfg.state);
}

void Buzzer_SetState(BuzzerType type, BuzzerState state)
{
    if (type < BUZZER_TYPE_MAX) {
        buzzer_cfg.type = type;
    }

    buzzer_cfg.state = (state == BUZZER_ON) ? BUZZER_ON : BUZZER_OFF;
    buzzer_cfg.last_toggle_time_ms = get_ms_tick_count();
    buzzer_apply_state(buzzer_cfg.state);
}

void Buzzer_Update(void)
{
    uint32_t now = get_ms_tick_count();

    switch (buzzer_cfg.type) {
        case BUZZER_TYPE_SOLID:
            buzzer_apply_state(buzzer_cfg.state);
            break;

        case BUZZER_TYPE_BLINK: {
            uint32_t duration = (buzzer_cfg.state == BUZZER_ON) ? buzzer_cfg.on_duration_ms
                                                                : buzzer_cfg.off_duration_ms;
            if ((duration > 0U) && ((now - buzzer_cfg.last_toggle_time_ms) >= duration)) {
                buzzer_cfg.state = (buzzer_cfg.state == BUZZER_ON) ? BUZZER_OFF : BUZZER_ON;
                buzzer_cfg.last_toggle_time_ms = now;
                buzzer_apply_state(buzzer_cfg.state);
            }
            break;
        }

        case BUZZER_TYPE_BEEP:
            if (buzzer_cfg.state == BUZZER_ON) {
                if ((buzzer_cfg.on_duration_ms > 0U) &&
                    ((now - buzzer_cfg.last_toggle_time_ms) >= buzzer_cfg.on_duration_ms)) {
                    buzzer_cfg.state = BUZZER_OFF;
                    buzzer_cfg.last_toggle_time_ms = now;
                    buzzer_apply_state(BUZZER_OFF);
                }
            } else {
                buzzer_apply_state(BUZZER_OFF);
            }
            break;

        default:
            buzzer_cfg.type = BUZZER_TYPE_SOLID;
            buzzer_cfg.state = BUZZER_OFF;
            buzzer_apply_state(BUZZER_OFF);
            break;
    }
}

void Buzzer_SetBlinkDurations(uint32_t on_duration_ms, uint32_t off_duration_ms)
{
    buzzer_cfg.on_duration_ms = (on_duration_ms > 0U) ? on_duration_ms : 1U;
    buzzer_cfg.off_duration_ms = (off_duration_ms > 0U) ? off_duration_ms : 1U;
}

void Buzzer_SetBeepDurations(uint32_t on_duration_ms)
{ buzzer_cfg.on_duration_ms = (on_duration_ms > 0U) ? on_duration_ms : BUZZER_BEEP_ON_DURATION_MS; }

void Buzzer_Stop(void)
{
    buzzer_cfg.state = BUZZER_OFF;
    buzzer_cfg.last_toggle_time_ms = get_ms_tick_count();
    buzzer_apply_state(BUZZER_OFF);
}
