/* PS2X_lib.c - PS2 controller support using SPI2 */
#include "PS2X_lib.h"
#include "main.h"
#include "spi.h"
#include "log_helper.h"
#include "diff_drive.h"

// Buffer for PS2 command and response
static uint8_t ps2_cmd[PS2X_BUFF_SIZE]  = {0x01, 0x42};
static uint8_t ps2_resp[PS2X_BUFF_SIZE] = {0};
// static uint8_t enter_config[]={0x01,0x43,0x00,0x01,0x00};
// static uint8_t set_mode[]={0x01,0x44,0x00,0x01,0x03,0x00,0x00,0x00,0x00};
// static uint8_t set_bytes_large[]={0x01,0x4F,0x00,0xFF,0xFF,0x03,0x00,0x00,0x00};
// static uint8_t exit_config[]={0x01,0x43,0x00,0x00,0x5A,0x5A,0x5A,0x5A,0x5A};

PS2X_Data ps2_data;

// Static function prototypes
static int ps2x_transfer_block(const uint8_t *tx, uint8_t *rx, uint16_t len);
static int ps2x_send_command(const uint8_t *cmd, uint8_t *resp, uint16_t len);

/* Static function Implementations */

static int ps2x_transfer_block(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    bool isConnected = false;

    for (uint16_t i = 0; i < len; ++i) {
        uint8_t out = tx ? tx[i] : 0xFF;
        delay_us(10); // Short delay between bytes if needed by PS2 timing
        uint8_t in = SPIx_transferOneByte(PS2X_SPI, out);
        if (rx)
            rx[i] = in;
        if (i == 0 && in != 0xFF) {
            return PS2X_ERR_COMM; // First byte should be 0xFF
        }
        // Check if all bytes are 0xFF, which indicates no controller connected
        if (i > 0 && in != 0xFF) {
            isConnected = true;
        }
    }
    return isConnected ? (int)len : PS2X_ERR_COMM;
}

static int ps2x_send_command(const uint8_t *cmd, uint8_t *resp, uint16_t len)
{
    // PS2 protocol: drive CS low, send bytes, then raise CS.

    PS2X_CS_LOW();
    int ret = ps2x_transfer_block(cmd, resp, len);
    PS2X_CS_HIGH();
    return ret;
}

/* Public API Implementations */

// Init PS2X, connect and read type of PS2X controller, if OK return 0
int ps2x_init(void)
{
    int ret = ps2x_send_command(ps2_cmd, ps2_resp, PS2X_BUFF_SIZE);
    if (ret <= 0) {
        LOG_DBG("Failed to communicate with PS2 controller");
        return PS2X_ERR_COMM;
    }

    // Check PS2 mode byte (0x41 for digital, 0x73 for analog, 0x79 for pressure) in response
    if (ps2_resp[1] != 0x41 && ps2_resp[1] != 0x73 && ps2_resp[1] != 0x79) {
        LOG_DBG("Unsupported PS2 mode: 0x%02X", ps2_resp[1]);
        return PS2X_ERR_MODE; // Unsupported mode
    }

    // Check response for valid controller type (0x5A is common for DualShock)
    if (ps2_resp[2] != 0x5A) {
        LOG_DBG("Unsupported PS2 controller type: 0x%02X", ps2_resp[2]);
        return PS2X_ERR_TYPE;
    }

    return PS2X_SUCCESS;
}

void ps2x_read_gamepad(void)
{
    // Config to analog mode
    // ps2x_send_command(enter_config, ps2_resp, sizeof(enter_config));
    // delay_us(10);
    // ps2x_send_command(set_mode, ps2_resp, sizeof(set_mode));
    // delay_us(10);
    // ps2x_send_command(set_bytes_large, ps2_resp, sizeof(set_bytes_large));
    // delay_us(10);
    // ps2x_send_command(exit_config, ps2_resp, sizeof(exit_config));
    // delay_us(10);

    // Read PS2 controller state and print button states
    ps2x_send_command(ps2_cmd, ps2_resp, PS2X_BUFF_SIZE);

    // Printf PS2 response for debugging
    if(ps2_resp[2] == 0x5A) {
        LOG_HEX_DBG(ps2_resp, PS2X_BUFF_SIZE);
    }

    // Fill PS2X_Data struct based on response
    for (int i = 0; i < PS2X_BUFF_SIZE -1; ++i) {
        ps2_data.raw[i] = ps2_resp[i + 1]; // Skip first byte (0xFF)
    }

}

// Get PS2X data state
PS2X_Packet ps2x_getAllData(void)
{
    return ps2_data.state;
}

// Get PS2X mode (digital, analog, pressure)
uint8_t ps2x_getMode(void)
{
    return ps2_data.state.mode;
}

// Check if any of the buttons in button_mask are pressed
// Data when no button press are:
// Mode Digital (0x41): FF FF FF FF FF FF
// Mode Analog (0x73):  FF FF 80 7F 80 7F

// Get PS2X button state, return 1 if pressed, 0 if not pressed
bool ps2x_isButtonPressed(void)
{
    return (ps2_data.raw[PS2X_IDX_BTN1] != 0xFF || ps2_data.raw[PS2X_IDX_BTN2] != 0xFF);
}

// Get PS2X joystick active, return 1 if active, 0 if not active (joystick in neutral position)
bool ps2x_isJoystickActive(void)
{
    return (ps2_data.raw[PS2X_IDX_RX] != 0x80 || ps2_data.raw[PS2X_IDX_RY] != 0x7F ||
            ps2_data.raw[PS2X_IDX_LX] != 0x80 || ps2_data.raw[PS2X_IDX_LY] != 0x7F);
}

// Update PS2X data by reading from controller, return 0 if success, negative value if error
PS2X_CommandData ps2x_update_data(void)
{
    PS2X_CommandData cmd_data = {0};

    ps2x_read_gamepad();
    PS2X_Packet ps2_state = ps2x_getAllData();

    if (ps2_state.btn2.l1 == PS2X_BTN_ACTIVE || ps2_state.btn2.r1 == PS2X_BTN_ACTIVE ||
        ps2_state.btn2.l2 == PS2X_BTN_ACTIVE || ps2_state.btn2.r2 == PS2X_BTN_ACTIVE) {
        int16_t ly = 127 - ps2_state.ly;
        float linear_vel = ((float)ly / 128.0f) * MAX_LINEAR_VEL;
        if (linear_vel < 0.05f && linear_vel > -0.05f) {
            linear_vel = 0.0f;
        }

        int16_t rx = 128 - ps2_state.rx;
        float angular_vel = ((float)rx / 128.0f) * MAX_ANGULAR_VEL;
        if (angular_vel < 0.1f && angular_vel > -0.1f) {
            angular_vel = 0.0f;
        }

        LOG_DBG("LY:%d,RX:%d\r\n", ly, rx);
        cmd_data.status = PS2X_SUCCESS;
        cmd_data.linear_vel = linear_vel;
        cmd_data.angular_vel = angular_vel;
    }

    // Triangle button to turn on buzzer
    if (ps2_state.btn2.triangle == PS2X_BTN_ACTIVE) {
        cmd_data.status = PS2X_SUCCESS;
        cmd_data.buzzer_on = true;
    }

    // Circle button to change LED color
    if (ps2_state.btn2.circle == PS2X_BTN_ACTIVE) {
        cmd_data.status = PS2X_SUCCESS;
        cmd_data.led_change = true;
    }

    return cmd_data;
}