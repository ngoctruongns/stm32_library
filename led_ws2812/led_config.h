#pragma once


#define WS2812_LED_COUNT     30

// GPIO configuration for WS2812 data line:  PE9 ------> TIM1_CH1
#define LED_PORT GPIOE
#define LED_PIN LL_GPIO_PIN_9

// Timer and PWM for WS2812 control
#define WS2812_TIM TIM1
#define WS2812_TIM_CHANNEL LL_TIM_CHANNEL_CH1

// DMA config for WS2812 control (TIM1_CH1 -> DMA2 Stream1 Channel6)
#define WS2812_DMA DMA2
#define WS2812_DMA_STREAM LL_DMA_STREAM_1
#define WS2812_DMA_CHANNEL LL_DMA_CHANNEL_6