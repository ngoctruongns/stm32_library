#include "main.h"
#include "ws2812.h"
#include "led_config.h"
#include "log_helper.h"

/* * TIM1 Setup: APB2 = 168MHz, ARR = 209 -> PWM freq = 800kHz
 * WS2812 Timing Requirements:
 * T0H: 0.35us (+/- 150ns) -> ~58-60 ticks (Used 64)
 * T1H: 0.70us (+/- 150ns) -> ~117-120 ticks (Used 128)
 */
#define DUTY_0 64
#define DUTY_1 128

#define BITS_PER_LED 24
#define RESET_SLOTS  50 /* > 50us to trigger Reset latch */
#define BUFFER_SIZE ((WS2812_LED_COUNT * BITS_PER_LED) + RESET_SLOTS)
#define HUE_STEP (255 / WS2812_LED_COUNT) // Hue step between LEDs

/***                Static function prototype          ***/
/*********************************************************/
static void WS2812_Encode(void);
static void WS2812_SetPixel(uint16_t index, ws2812_color_t color);
static void WS2812_SetAll(ws2812_color_t color);
static void WS2812_Clear(void);
static void WS2812_Update(void);

static void WS2812_LedBlink(ws2812_color_t color, uint16_t delay_ms);
static void WS2812_Rainbow(uint16_t speed_ms);
static void WS2812_Breath(ws2812_color_t color, uint16_t breath_period_ms);

/*****                Global variables              ******/
/*********************************************************/

/* PWM Buffer: 24 bits per LED + Reset slots (all zeros) */
static uint16_t pwm_buffer[BUFFER_SIZE];
static ws2812_color_t led_buffer[WS2812_LED_COUNT];
static volatile uint8_t ws2812_dma_busy = 0;

led_display_type_t gLedCurrType = LED_TYPE_OFF;
led_display_config_t gLedCfg;

/***  Static function for setting LED display patterns ***/
/*********************************************************/

/**
 * @brief Encodes RGB data into PWM duty cycles for DMA transfer
 * WS2812 expects data in G-R-B order.
 */
static void WS2812_Encode(void)
{
    uint32_t idx = 0;

    for (uint16_t i = 0; i < WS2812_LED_COUNT; i++)
    {
        /* Compose 24-bit color in GRB order */
        uint32_t color_data = (led_buffer[i].g << 16) | (led_buffer[i].r << 8) | led_buffer[i].b;

        // Send MSB first
        for (int8_t bit = 23; bit >= 0; bit--)
        {
            if (color_data & (1 << bit))
                pwm_buffer[idx++] = DUTY_1;
            else
                pwm_buffer[idx++] = DUTY_0;
        }
    }

    /* Fill Reset slots with 0 to keep the line LOW */
    for (uint16_t i = 0; i < RESET_SLOTS; i++)
    {
        pwm_buffer[idx++] = 0;
    }
}

static void WS2812_SetPixel(uint16_t index, ws2812_color_t color)
{
    if (index < WS2812_LED_COUNT)
    {
        // Constrain color values to max brightness
        led_buffer[index].r = (uint8_t)((color.r * BRIGHTNESS_MAX) / 255);
        led_buffer[index].g = (uint8_t)((color.g * BRIGHTNESS_MAX) / 255);
        led_buffer[index].b = (uint8_t)((color.b * BRIGHTNESS_MAX) / 255);
    }
}

static void WS2812_SetAll(ws2812_color_t color)
{
    for (uint16_t i = 0; i < WS2812_LED_COUNT; i++)
    {
        WS2812_SetPixel(i, color);
    }
}

static void WS2812_Clear(void)
{
    WS2812_SetAll(LED_OFF);
}

/**
 * @brief Starts the DMA transfer to update the LED strip
 */
static void WS2812_Update(void)
{
    if (ws2812_dma_busy)
    {
        return;
    }

    ws2812_dma_busy = 1;

    /* 1. Encode LED buffer to PWM values */
    WS2812_Encode();

    /* 2. Disable DMA Stream to reconfigure length */
    LL_DMA_DisableStream(WS2812_DMA, WS2812_DMA_STREAM);
    while (LL_DMA_IsEnabledStream(WS2812_DMA, WS2812_DMA_STREAM));

    /* 3. Clear all DMA status flags for Stream 1 */
    LL_DMA_ClearFlag_TC1(WS2812_DMA);
    LL_DMA_ClearFlag_HT1(WS2812_DMA);
    LL_DMA_ClearFlag_TE1(WS2812_DMA);

    /* 4. Reload DMA length for Normal mode */
    LL_DMA_SetDataLength(WS2812_DMA, WS2812_DMA_STREAM, BUFFER_SIZE);

    /* 5. Enable DMA Stream and Timer DMA Request */
    LL_DMA_EnableStream(WS2812_DMA, WS2812_DMA_STREAM);

    /* 6. Start Timer Counter from zero */
    LL_TIM_SetCounter(WS2812_TIM, 0);
    LL_TIM_EnableCounter(WS2812_TIM);
}

static void WS2812_LedBlink(ws2812_color_t color, uint16_t delay_ms)
{
    static uint32_t last_toggle_time = 0;
    static uint8_t state = 0;
    uint32_t current_time = get_ms_tick_count();

    if ((current_time - last_toggle_time) >= delay_ms)
    {
        state = !state;
        if (state)
        {
            WS2812_SetAll(color);
        }
        else
        {
            WS2812_Clear();
        }
        WS2812_Update();
        last_toggle_time = current_time;
    }
}

static ws2812_color_t Wheel(uint8_t wheel_pos)
{
    ws2812_color_t color;
    wheel_pos = 255 - wheel_pos;
    if (wheel_pos < 85) {
        color.r = 255 - wheel_pos * 3;
        color.g = 0;
        color.b = wheel_pos * 3;
    } else if (wheel_pos < 170) {
        wheel_pos -= 85;
        color.r = 0;
        color.g = wheel_pos * 3;
        color.b = 255 - wheel_pos * 3;
    } else {
        wheel_pos -= 170;
        color.r = wheel_pos * 3;
        color.g = 255 - wheel_pos * 3;
        color.b = 0;
    }
    return color;
}

/**
 * @brief Rainbow cycle effect for WS2812
 * @param speed_ms Delay between color shifts (lower is faster)
 */
static void WS2812_Rainbow(uint16_t speed_ms)
{
    static uint8_t j = 0; // State variable to track color shift over time
    static uint32_t last_update = 0;
    uint32_t current_time = get_ms_tick_count();

    // Update only when the speed interval has passed (non-blocking)
    if (current_time - last_update < speed_ms) {
        return;
    }
    last_update = current_time;

    for (uint16_t i = 0; i < WS2812_LED_COUNT; i++) {
        // Calculate hue for each LED to create a gradient across the strip
        // The "256 / WS2812_LED_COUNT" spreads the full rainbow across all pixels
        uint8_t hue = (uint8_t)((i * 256 / WS2812_LED_COUNT) + j) & 0xFF;
        WS2812_SetPixel(i, Wheel(hue));
    }

    // Push data to the LED strip
    WS2812_Update();

    // Increment j to shift the rainbow colors in the next frame
    j += 3;
}

/**
 * @brief Breathing effect for WS2812
 * @note  This function should be called within a loop or a Timer callback
 * with a fixed update interval (e.g., every 20ms).
 */
static void WS2812_Breath(ws2812_color_t color, uint16_t breath_period_ms)
{
    // Get system uptime in milliseconds
    uint32_t current_time = get_ms_tick_count();

    // 1. Calculate the current position within the period (0 to breath_period_ms - 1)
    uint32_t phase = current_time % breath_period_ms;


    uint32_t amplitude = (phase * 511) / breath_period_ms;
    uint8_t brightness;

    if (amplitude < 256) {
        brightness = (uint8_t)amplitude;       // Fading in (0 -> 255)
    } else {
        brightness = (uint8_t)(511 - amplitude); // Fading out (255 -> 0)
    }

    // 3. Scale the color based on brightness (Using multiplication and bit shifting instead of division)
    // Formula: (Color * Brightness) / 256
    ws2812_color_t breathing_color;
    breathing_color.r = (uint8_t)((color.r * brightness)/ 255);
    breathing_color.g = (uint8_t)((color.g * brightness)/ 255);
    breathing_color.b = (uint8_t)((color.b * brightness)/ 255);

    // print for debugging
    // printf("Breath: %d, R:%d,G:%d,B:%d\r\n", brightness, breathing_color.r, breathing_color.g, breathing_color.b);

    WS2812_SetAll(breathing_color);
    WS2812_Update();
}

/***         LED display interface function implement  ***/
/*********************************************************/

/**
 * @brief Hardware-specific initialization for TIM1 and GPIO
 */
void WS2812_Init(void)
{
    /* Ensure GPIO is set to Very High Speed for sharp PWM edges */
    LL_GPIO_SetPinSpeed(LED_PORT, LED_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);

    /* Clear DMA flags to prevent immediate trigger or error */
    LL_DMA_ClearFlag_TC1(WS2812_DMA);
    LL_DMA_ClearFlag_TE1(WS2812_DMA);

    /* 4. Configure DMA transfer parameters */
    LL_DMA_SetMemoryAddress(WS2812_DMA, WS2812_DMA_STREAM, (uint32_t)pwm_buffer);
    LL_DMA_SetDataLength(WS2812_DMA, WS2812_DMA_STREAM, BUFFER_SIZE);

    /* Enable DMA interrupts */
    LL_DMA_EnableIT_TC(WS2812_DMA, WS2812_DMA_STREAM);
    LL_DMA_EnableIT_TE(WS2812_DMA, WS2812_DMA_STREAM);

    LL_TIM_EnableDMAReq_CC1(WS2812_TIM);

    /* Enable Preload for Compare Register */
    LL_TIM_OC_EnablePreload(WS2812_TIM, WS2812_TIM_CHANNEL);

    /* CRITICAL for TIM1: Main Output Enable (MOE) bit in BDTR register */
    LL_TIM_EnableAllOutputs(WS2812_TIM);

    /* Enable Capture/Compare Channel 1 */
    LL_TIM_CC_EnableChannel(WS2812_TIM, WS2812_TIM_CHANNEL);

    // LL_TIM_OC_SetCompareCH1(WS2812_TIM, 105); // Set duty 50% (ARR=209) for debugging
    LL_TIM_DisableCounter(WS2812_TIM);
}

void WS2812_DMA_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC1(WS2812_DMA))
    {
        LL_DMA_ClearFlag_TC1(WS2812_DMA);
        LL_TIM_DisableCounter(WS2812_TIM);
        ws2812_dma_busy = 0;
    }

    if (LL_DMA_IsActiveFlag_TE1(WS2812_DMA))
    {
        LL_DMA_ClearFlag_TE1(WS2812_DMA);
        LL_TIM_DisableCounter(WS2812_TIM);
        ws2812_dma_busy = 0;
        LOG_DBG("WS2812 DMA Transfer Error\r\n");
    }
}

void WS2812_SetSolidColor(ws2812_color_t color)
{
    gLedCurrType = LED_TYPE_SOLID;
    gLedCfg.solid.color = color;

    WS2812_SetAll(color);
    WS2812_Update();
}

void WS2812_SetBlink(ws2812_color_t color, uint16_t delay_ms)
{
    gLedCurrType = LED_TYPE_BLINK;
    gLedCfg.blink.color = color;
    gLedCfg.blink.delay_ms = delay_ms;
}

void WS2812_SetRainbow(uint16_t speed_ms)
{
    gLedCurrType = LED_TYPE_RAINBOW;
    gLedCfg.rainbow.rainbow_period_ms = speed_ms;
}

void WS2812_SetBreath(ws2812_color_t color, uint16_t breath_period_ms)
{
    gLedCurrType = LED_TYPE_BREATH;
    gLedCfg.breath.color = color;
    gLedCfg.breath.breath_period_ms = breath_period_ms;
}

// Loop control for patterns like blinking, breathing, etc. (to be called in main loop)
void WS2812_loopControl(void)
{
    // Handler current display pattern
    switch (gLedCurrType) {
        case LED_TYPE_SOLID:
            // No loop control needed for solid color
            break;
        case LED_TYPE_BLINK:
            WS2812_LedBlink(gLedCfg.blink.color, gLedCfg.blink.delay_ms);
            break;
        case LED_TYPE_RAINBOW:
            WS2812_Rainbow(gLedCfg.rainbow.rainbow_period_ms);
            break;
        case LED_TYPE_BREATH:
            WS2812_Breath(gLedCfg.breath.color, gLedCfg.breath.breath_period_ms);
            break;
        default:
            break;
    }
}
