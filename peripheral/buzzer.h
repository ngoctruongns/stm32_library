#pragma once

#include <stdint.h>
#include "velocity_control.h"  // shared BUZZER_TYPE_* enum values

// Buzzer states
typedef enum {
    BUZZER_OFF = 0,
    BUZZER_ON
} BuzzerState;

// BuzzerType typedef defined in velocity_control.h

// Struct data for buzzer configuration
typedef struct {
    BuzzerType type;
    BuzzerState state;
    uint32_t on_duration_ms;   // Duration for ON state in BLINK or BEEP mode
    uint32_t off_duration_ms;  // Duration for OFF state in BLINK or BEEP mode
    uint32_t last_toggle_time_ms; // Last time the buzzer state was toggled
} BuzzerConfig;

// Initialize buzzer
void Buzzer_Init(BuzzerType type, BuzzerState state);
// Set buzzer state and type
void Buzzer_SetState(BuzzerType type, BuzzerState state);
void Buzzer_Update(void);
void Buzzer_SetBlinkDurations(uint32_t on_duration_ms, uint32_t off_duration_ms);
void Buzzer_SetBeepDurations(uint32_t on_duration_ms);
void Buzzer_Stop(void);