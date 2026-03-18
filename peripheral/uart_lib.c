#include "uart_lib.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"

#include "diff_drive.h"
#include "log_helper.h"
#include "process_data_packet.h"
#include "string_helper.h"
#include "velocity_control.h"

#define UART2_CMD_BUFFER_LEN 128
static char uart2_cmd_buffer[UART2_CMD_BUFFER_LEN];
static uint16_t uart2_cmd_index = 0;

#define UART3_RX_DMA_BUFFER_LEN 256
#define UART3_TX_DMA_BUFFER_LEN 128
static uint8_t uart3_rx_dma_buffer[UART3_RX_DMA_BUFFER_LEN];
static volatile uint16_t uart3_rx_last_pos = 0;
static uint8_t uart3_tx_dma_buffer[UART3_TX_DMA_BUFFER_LEN];
static volatile uint8_t uart3_tx_busy = 0;
static uint32_t uart3_last_enc_tx_ms = 0;

static void process_pid_command(const char *cmd)
{
	float kp, ki, kd;
	if (parse_pid_command(cmd, &kp, &ki, &kd)) {
		diff_drive_set_pid(kp, ki, kd);
		printf("PID update success!\r\n");
	} else {
		if (cmd != NULL && cmd[0] != '\0') {
			printf("PID parse error: '%s'\r\n", cmd);
		}
	}
}

static void process_uart2_command(const char *cmd)
{
	if (cmd == NULL) {
		return;
	}

	while (isspace((unsigned char)*cmd)) {
		cmd++;
	}

	if (*cmd == '\0') {
		return;
	}

	if (toupper((unsigned char)cmd[0]) == 'L' && toupper((unsigned char)cmd[1]) == 'O' &&
		toupper((unsigned char)cmd[2]) == 'G' && isspace((unsigned char)cmd[3])) {
		const char *level_str = &cmd[4];
		char *endptr = NULL;
		long level = strtol(level_str, &endptr, 10);

		if (level_str == endptr) {
			printf("Invalid LOG command. Use: LOG <0..4>\\r\\n");
			return;
		}

		set_log_level((int)level);
		printf("Log level set to %d\\r\\n", get_log_level());
		return;
	}

	process_pid_command(cmd);
}

static void uart3_handle_decoded_packet(const uint8_t *payload, uint8_t payload_len)
{
	if (payload == NULL || payload_len < 1U) {
		return;
	}

	switch (payload[0]) {
		case CMD_VEL_COMMAND:
			if (payload_len == sizeof(struct CmdVelType)) {
				const struct CmdVelType *cmd = (const struct CmdVelType *)payload;
				setMotorRPM(MOTOR_L, (float)cmd->left_rpm);
				setMotorRPM(MOTOR_R, (float)cmd->right_rpm);
			}
			break;

		case PID_CONFIG_COMMAND:
			if (payload_len == sizeof(struct PIDConfigType)) {
				const struct PIDConfigType *pid_cfg = (const struct PIDConfigType *)payload;
				diff_drive_set_pid(pid_cfg->Kp, pid_cfg->Ki, pid_cfg->Kd);
			}
			break;

		default:
			break;
	}
}

static void uart3_process_rx_block(const uint8_t *data, uint16_t len)
{
	uint8_t decoded_payload[BUFFER_SIZE] = {0};

	for (uint16_t i = 0; i < len; i++) {
		uint8_t decoded_len = handleRxByteConcurrent(data[i], decoded_payload);
		if (decoded_len > 0U) {
			uart3_handle_decoded_packet(decoded_payload, decoded_len);
		}
	}
}

static void uart3_scan_dma_rx_buffer(void)
{
	uint16_t dma_pos = UART3_RX_DMA_BUFFER_LEN - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);
	if (dma_pos >= UART3_RX_DMA_BUFFER_LEN) {
		dma_pos = 0U;
	}

	if (dma_pos == uart3_rx_last_pos) {
		return;
	}

	if (dma_pos > uart3_rx_last_pos) {
		uart3_process_rx_block(&uart3_rx_dma_buffer[uart3_rx_last_pos], dma_pos - uart3_rx_last_pos);
	} else {
		uart3_process_rx_block(&uart3_rx_dma_buffer[uart3_rx_last_pos],
							   UART3_RX_DMA_BUFFER_LEN - uart3_rx_last_pos);
		if (dma_pos > 0U) {
			uart3_process_rx_block(&uart3_rx_dma_buffer[0], dma_pos);
		}
	}

	uart3_rx_last_pos = dma_pos;
}

static uint8_t uart3_tx_dma_start(const uint8_t *data, uint16_t len)
{
	if (data == NULL || len == 0U || uart3_tx_busy != 0U) {
		return 0U;
	}

	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
	while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_3)) {
	}

	LL_DMA_ClearFlag_HT3(DMA1);
	LL_DMA_ClearFlag_TC3(DMA1);
	LL_DMA_ClearFlag_TE3(DMA1);
	LL_DMA_ClearFlag_DME3(DMA1);
	LL_DMA_ClearFlag_FE3(DMA1);

	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_3, LL_USART_DMA_GetRegAddr(USART3));
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_3, (uint32_t)data);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, len);

	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_3);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_3);

	uart3_tx_busy = 1U;
	LL_USART_EnableDMAReq_TX(USART3);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
	return 1U;
}

static void uart3_send_encoder_feedback(void)
{
	struct WheelEncType enc_data;
	enc_data.type = WHEEL_ENC_COMMAND;
	enc_data.left_enc = getEncoderCount(MOTOR_L);
	enc_data.right_enc = getEncoderCount(MOTOR_R);

	uint8_t frame_len = encoderAllPackage((const uint8_t *)&enc_data, sizeof(enc_data), uart3_tx_dma_buffer);
	(void)uart3_tx_dma_start(uart3_tx_dma_buffer, frame_len);
}

void uart3_comm_init(void)
{
	uart3_rx_last_pos = 0U;
	uart3_tx_busy = 0U;
	uart3_last_enc_tx_ms = get_ms_tick_count();

	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
	while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_1)) {
	}

	LL_DMA_ClearFlag_HT1(DMA1);
	LL_DMA_ClearFlag_TC1(DMA1);
	LL_DMA_ClearFlag_TE1(DMA1);
	LL_DMA_ClearFlag_DME1(DMA1);
	LL_DMA_ClearFlag_FE1(DMA1);

	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, LL_USART_DMA_GetRegAddr(USART3));
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)uart3_rx_dma_buffer);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, UART3_RX_DMA_BUFFER_LEN);

	LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_1);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_1);

	LL_USART_EnableDMAReq_RX(USART3);
	LL_USART_EnableIT_IDLE(USART3);

	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
}

void uart3_comm_poll(void)
{
	uint32_t now = get_ms_tick_count();
	if ((now - uart3_last_enc_tx_ms) >= 20U) {
		uart3_last_enc_tx_ms = now;
		if (uart3_tx_busy == 0U) {
			uart3_send_encoder_feedback();
		}
	}
}

void usart3_idle_interrupt_handler(void)
{
	uart3_scan_dma_rx_buffer();
}

void usart3_dma_rx_interrupt_handler(void)
{
	uart3_scan_dma_rx_buffer();
}

void usart3_dma_tx_interrupt_handler(void)
{
	LL_USART_DisableDMAReq_TX(USART3);
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
	uart3_tx_busy = 0U;
}

void usart2_interrupt_handler(uint8_t data)
{
	if (data == '\r') {
		// ignore CR, parse on LF
		return;
	}

	if (data == '\n') {
		// complete command detected
		uart2_cmd_buffer[uart2_cmd_index] = '\0';
		if (uart2_cmd_index > 0) {
			process_uart2_command(uart2_cmd_buffer);
		}
		uart2_cmd_index = 0;
		return;
	}

	if (uart2_cmd_index < UART2_CMD_BUFFER_LEN - 1) {
		uart2_cmd_buffer[uart2_cmd_index++] = (char)data;
	} else {
		// overflow, reset buffer to avoid hang
		uart2_cmd_index = 0;
		memset(uart2_cmd_buffer, 0, UART2_CMD_BUFFER_LEN);
		printf("UART2 command buffer overflow\r\n");
	}
}
