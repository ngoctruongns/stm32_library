#include "string_helper.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

bool is_blank_string(const char *s)
{
    if (s == NULL) {
        return true;
    }
    while (*s) {
        if (!isspace((unsigned char)*s)) {
            return false;
        }
        s++;
    }
    return true;
}

bool str_to_float(const char *str, float *out)
{
    if (str == NULL || out == NULL) {
        return false;
    }

    // allow use of standard parser, but keep fallback detection
    char *endptr = NULL;
    float value = strtof(str, &endptr);
    if (str == endptr) {
        return false;
    }

    // skip trailing spaces
    while (*endptr && isspace((unsigned char)*endptr)) {
        endptr++;
    }

    if (*endptr != '\0') {
        return false;
    }

    *out = value;
    return true;
}

bool parse_pid_command(const char *cmd, float *kp, float *ki, float *kd)
{
    if (cmd == NULL || kp == NULL || ki == NULL || kd == NULL) {
        return false;
    }

    if (is_blank_string(cmd)) {
        return false;
    }

    const char *p = cmd;
    // Skip leading spaces
    while (*p && isspace((unsigned char)*p)) p++;

    // Accept PID or pid
    if (!(strncmp(p, "PID", 3u) == 0 || strncmp(p, "pid", 3u) == 0)) {
        return false;
    }
    p += 3;

    // Skip spaces
    while (*p && isspace((unsigned char)*p)) p++;

    char token[32];

    // kp
    int idx = 0;
    while (*p && !isspace((unsigned char)*p) && idx < (int)sizeof(token) - 1) {
        token[idx++] = *p++;
    }
    token[idx] = '\0';
    if (!str_to_float(token, kp)) {
        return false;
    }

    // skip spaces
    while (*p && isspace((unsigned char)*p)) p++;

    // ki
    idx = 0;
    while (*p && !isspace((unsigned char)*p) && idx < (int)sizeof(token) - 1) {
        token[idx++] = *p++;
    }
    token[idx] = '\0';
    if (!str_to_float(token, ki)) {
        return false;
    }

    // skip spaces
    while (*p && isspace((unsigned char)*p)) p++;

    // kd
    idx = 0;
    while (*p && !isspace((unsigned char)*p) && idx < (int)sizeof(token) - 1) {
        token[idx++] = *p++;
    }
    token[idx] = '\0';
    if (!str_to_float(token, kd)) {
        return false;
    }

    // end of string allow trailing spaces
    while (*p && isspace((unsigned char)*p)) p++;
    if (*p != '\0') {
        return false;
    }

    return true;
}
