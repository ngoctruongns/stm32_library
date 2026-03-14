#ifndef __WS2812_H
#define __WS2812_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

// Define the color structure for WS2812 LEDs
#define LED_RED     (ws2812_color_t){255, 0, 0}
#define LED_GREEN   (ws2812_color_t){0, 255, 0}
#define LED_BLUE    (ws2812_color_t){0, 0, 255}
#define LED_OFF     (ws2812_color_t){0, 0, 0}
#define LED_WHITE   (ws2812_color_t){255, 255, 255}
#define LED_YELLOW  (ws2812_color_t){255, 255, 0}
#define LED_CYAN    (ws2812_color_t){0, 255, 255}
#define LED_MAGENTA (ws2812_color_t){255, 0, 255}

/* Define structure for WS2812 LEDs */

// LED color structure
typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} ws2812_color_t;

// LED display type
typedef enum
{
    LED_TYPE_OFF = 0,
    LED_TYPE_SOLID,
    LED_TYPE_BLINK,
    LED_TYPE_RAINBOW,
    LED_TYPE_BREATH,
    LED_TYPE_CHASE,
    LED_TYPE_SCANNER
} led_display_type_t;

// Params type for solid color
typedef struct
{
    ws2812_color_t color;
} led_solid_params_t;

// Params type for blinking pattern
typedef struct
{
    ws2812_color_t color;
    uint16_t delay_ms;
} led_blink_params_t;

// Params type for breathing effect
typedef struct
{
    ws2812_color_t color;
    uint16_t breath_period_ms;
} led_breath_params_t;

// Params type for rainbow pattern (can be extended with speed, direction, etc.)
typedef struct
{
    uint16_t rainbow_period_ms;
} led_rainbow_params_t;

// Main config structure for LED display
typedef union {
    led_solid_params_t solid;
    led_blink_params_t blink;
    led_rainbow_params_t rainbow;
    led_breath_params_t breath;
    // Add more params for other patterns as needed
} led_display_config_t;

/***  LED interface functions ***/
// Initialization function to set up GPIO, Timer, and DMA for WS2812 control
void WS2812_Init(void);

// Loop control for patterns like blinking, breathing, etc. (to be called in main loop)
void WS2812_loopControl(void);

// Interface functions to set different LED display patterns
void WS2812_SetSolidColor(ws2812_color_t color);
void WS2812_SetBlink(ws2812_color_t color, uint16_t delay_ms);
void WS2812_SetRainbow(uint16_t speed_ms);
void WS2812_SetBreath(ws2812_color_t color, uint16_t breath_period_ms);
// Add more functions for other patterns as needed

#ifdef __cplusplus
}
#endif

#endif