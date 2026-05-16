#pragma once

#include <stdint.h>
#include "velocity_control.h"

#ifdef __cplusplus
extern "C" {
#endif

// UART2/UART3 communication API (command parsing + UART3 DMA transport)
void uart3_comm_init(void);
void uart3_comm_poll(void);
uint8_t uart3_get_latest_cmd_vel(struct CmdVelType *cmd, uint32_t *rx_time_ms);

// Update cached IMU raw register values used in the next odometry packet.
// Call at 50 Hz from the control loop after reading MPU-6050.
void uart3_update_imu_raw(int16_t ax, int16_t ay, int16_t az,
                           int16_t gx, int16_t gy, int16_t gz);

void usart2_interrupt_handler(uint8_t data);
void usart3_idle_interrupt_handler(void);
void usart3_dma_rx_interrupt_handler(void);
void usart3_dma_tx_interrupt_handler(void);

#ifdef __cplusplus
}
#endif