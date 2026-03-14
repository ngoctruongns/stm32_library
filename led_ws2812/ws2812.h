#ifndef __WS2812_H
#define __WS2812_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} ws2812_color_t;

void WS2812_Init(void);

void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b);

void WS2812_Clear(void);

void WS2812_Update(void);

void WS2812_Blink(uint8_t r, uint8_t g, uint8_t b, uint32_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif