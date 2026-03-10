#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool is_blank_string(const char *s);

bool str_to_float(const char *str, float *out);

// Parses a command string in the format "PID <kp> <ki> <kd>" (case-insensitive)
bool parse_pid_command(const char *cmd, float *kp, float *ki, float *kd);

#ifdef __cplusplus
}
#endif
