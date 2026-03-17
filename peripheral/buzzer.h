#pragma once

#include <stdint.h>

// Buzzer states
typedef enum {
    BUZZER_OFF = 0,
    BUZZER_ON
} BuzzerState;

// Buzzer types
typedef enum {
    BUZZER_TYPE_SOLID = 0,
    BUZZER_TYPE_BLINK,
    BUZZER_TYPE_BEEP,
    BUZZER_TYPE_MAX
} BuzzerType;

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