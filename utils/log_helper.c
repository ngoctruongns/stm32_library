#include <stdint.h>

#include "log_helper.h"

static int log_level = LOG_LEVEL_DEFAULT;

int get_log_level(void)
{
    return log_level;
}

void set_log_level(int level)
{
    if (level > LOG_LEVEL_DBG) {
        level = LOG_LEVEL_DBG;
    } else if (level < LOG_LEVEL_OFF) {
        level = LOG_LEVEL_OFF;
    }

    log_level = level;
}

void print_buff_hex(int level, void *buff, int length)
{
    if (get_log_level() < level)
        return;

    uint8_t *data = (uint8_t *) buff;

    for (int i = 0; i < length; i++) {
        if (i > 0 && i % 16 == 0) {
            printf("\n");
        }
        printf("%02X ", data[i]);
    }
    printf("\r\n");
}
