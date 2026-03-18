#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// UART2/UART3 communication API (command parsing + UART3 DMA transport)
void uart3_comm_init(void);
void uart3_comm_poll(void);

void usart2_interrupt_handler(uint8_t data);
void usart3_idle_interrupt_handler(void);
void usart3_dma_rx_interrupt_handler(void);
void usart3_dma_tx_interrupt_handler(void);

#ifdef __cplusplus
}
#endif