#pragma once

#include <stdio.h>

#define LOG_LEVEL_OFF 0
#define LOG_LEVEL_ERR 1
#define LOG_LEVEL_WRN 2
#define LOG_LEVEL_INF 3
#define LOG_LEVEL_DBG 4

#define LOG_LEVEL_DEFAULT LOG_LEVEL_INF

int get_log_level(void);
void set_log_level(int level);
void print_buff_hex(int level, void *buff, int length);

#define log_printf(level, fmt, args...) \
    do { \
        if (get_log_level() >= level) { \
            printf(fmt, ##args); \
        } \
    } while (0)

#define LOG_ERR(fmt, args...)   log_printf(LOG_LEVEL_ERR, "E:" fmt "\n", ##args)
#define LOG_WRN(fmt, args...)   log_printf(LOG_LEVEL_WRN, "W:" fmt "\n", ##args)
#define LOG_INF(fmt, args...)   log_printf(LOG_LEVEL_INF, "I:" fmt "\n", ##args)
#define LOG_DBG(fmt, args...)   log_printf(LOG_LEVEL_DBG, "D:" fmt "\n", ##args)
#define PRINT_DBG(fmt, args...) log_printf(LOG_LEVEL_DBG, fmt, ##args)

// Print buffer data in hex format
#define LOG_HEX_INF(buff, length) print_buff_hex(LOG_LEVEL_INF, buff, length)
#define LOG_HEX_DBG(buff, length) print_buff_hex(LOG_LEVEL_DBG, buff, length)
