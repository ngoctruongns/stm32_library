#include "ws2812.h"
#include "led_config.h"
#include "main.h"

/* * TIM1 Setup: APB2 = 168MHz, ARR = 209 -> PWM freq = 800kHz
 * WS2812 Timing Requirements:
 * T0H: 0.35us (+/- 150ns) -> ~58-60 ticks (Used 64)
 * T1H: 0.70us (+/- 150ns) -> ~117-120 ticks (Used 128)
 */
#define DUTY_0 64
#define DUTY_1 128

#define BITS_PER_LED 24
#define RESET_SLOTS  50 /* > 50us to trigger Reset latch */

static ws2812_color_t led_buffer[WS2812_LED_COUNT];

/* PWM Buffer: 24 bits per LED + Reset slots (all zeros) */
static uint16_t pwm_buffer[(WS2812_LED_COUNT * BITS_PER_LED) + RESET_SLOTS];

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

    /* Enable Preload for Compare Register */
    LL_TIM_OC_EnablePreload(WS2812_TIM, WS2812_TIM_CHANNEL);

    /* CRITICAL for TIM1: Main Output Enable (MOE) bit in BDTR register */
    LL_TIM_EnableAllOutputs(WS2812_TIM);

    /* Enable Capture/Compare Channel 1 */
    LL_TIM_CC_EnableChannel(WS2812_TIM, WS2812_TIM_CHANNEL);

    // LL_TIM_OC_SetCompareCH1(TIM1, 105); // Set duty 50% (ARR=209) for debugging
    LL_TIM_EnableCounter(TIM1);

    // Configure the destination address (Peripheral) for DMA
    LL_DMA_SetPeriphAddress(WS2812_DMA, WS2812_DMA_STREAM, (uint32_t)&WS2812_TIM->CCR1);
}

void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index < WS2812_LED_COUNT)
    {
        led_buffer[index].r = r;
        led_buffer[index].g = g;
        led_buffer[index].b = b;
    }
}

void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint16_t i = 0; i < WS2812_LED_COUNT; i++)
    {
        WS2812_SetPixel(i, r, g, b);
    }
}

void WS2812_Clear(void)
{
    WS2812_SetAll(0, 0, 0);
}

/**
 * @brief Starts the DMA transfer to update the LED strip
 */
void WS2812_Update(void)
{
    // Check for Transfer Error flag
    if (LL_DMA_IsActiveFlag_TE1(DMA2)) {
        // Nếu nhảy vào đây, nghĩa là cấu hình địa chỉ hoặc Bus DMA có vấn đề
        LL_DMA_ClearFlag_TE1(DMA2);
        // Toggle LED LD4
        LL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
        return;
    }
    /* 1. Encode LED buffer to PWM values */
    WS2812_Encode();

    /* 2. Disable DMA Stream to reconfigure length */
    LL_DMA_DisableStream(WS2812_DMA, WS2812_DMA_STREAM);
    while (LL_DMA_IsEnabledStream(WS2812_DMA, WS2812_DMA_STREAM));

    /* 3. Clear all DMA status flags for Stream 1 */
    LL_DMA_ClearFlag_TC1(WS2812_DMA);
    LL_DMA_ClearFlag_HT1(WS2812_DMA);
    LL_DMA_ClearFlag_TE1(WS2812_DMA);

    /* 4. Configure DMA transfer parameters */
    LL_DMA_SetMemoryAddress(WS2812_DMA, WS2812_DMA_STREAM, (uint32_t)pwm_buffer);
    LL_DMA_SetDataLength(WS2812_DMA, WS2812_DMA_STREAM, (WS2812_LED_COUNT * BITS_PER_LED) + RESET_SLOTS);

    /* 5. Enable DMA Stream and Timer DMA Request */
    LL_DMA_EnableStream(WS2812_DMA, WS2812_DMA_STREAM);
    LL_TIM_EnableDMAReq_CC1(WS2812_TIM);

    /* 6. Start Timer Counter from zero */
    LL_TIM_SetCounter(WS2812_TIM, 0);
    LL_TIM_EnableCounter(WS2812_TIM);

    /* 7. Wait for Transfer Complete (Blocking for debugging) */
    uint32_t timeout = 1000000;
    while (!LL_DMA_IsActiveFlag_TC1(WS2812_DMA) && timeout--);

    /* 8. Clean up to prevent continuous PWM output after data is sent */
    LL_TIM_DisableCounter(WS2812_TIM);
    LL_TIM_DisableDMAReq_CC1(WS2812_TIM);
    LL_DMA_ClearFlag_TC1(WS2812_DMA);
}

void WS2812_Blink(uint8_t r, uint8_t g, uint8_t b, uint32_t delay_ms)
{
    WS2812_SetAll(r, g, b);
    WS2812_Update();
    LL_mDelay(delay_ms);
    WS2812_Clear();
    WS2812_Update();
    LL_mDelay(delay_ms);
}