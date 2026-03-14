#include "ws2812.h"

int led_blink_example(void)
{
    WS2812_Init();

    while(1)
    {
        WS2812_Blink(255,0,0,200);
    }
}