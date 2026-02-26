/* PS2X_example.c
 * Example usage of Library/PS2X_lib
 * Call PS2X_example_run() from main after clocks and GPIO are initialized.
 */

#include "PS2X_lib.h"
#include "main.h"

/* Example: send a short PS2 command and read response.
 * Standard 'enter config' style commands use sequences like {0x01, 0x42, ...}
 * Here we send 0x01, 0x42 and read two response bytes into `resp`.
 */
void PS2X_example_run(void)
{
    /* Try initialize PS2X peripheral */
    for (int i =0; i<10; i++)
    {

        if (ps2x_init() == PS2X_SUCCESS) {
            // ps2x_config_mode(PS2X_MODE_PRESSURES);
            printf("PS2X Controller Initialized Successfully\r\n");
            break;
        } else if (i == 9) {
            printf("Failed to Initialize PS2X Controller\r\n");
        }
    }

    while (1)
    {
        /* Communication with PS2 controller */
        ps2x_read_gamepad();

        // Get data from PS2X and print for debugging
        PS2X_State ps2_state = ps2x_getAllData();
        printf("PS2X Mode: 0x%02X, Type: 0x%02X\r\n", ps2_state.mode, ps2_state.type);
        printf("Joystick Left: X=%d Y=%d\r\n", ps2_state.lx, ps2_state.ly);
        printf("Joystick Right: X=%d Y=%d\r\n", ps2_state.rx, ps2_state.ry);
        if (ps2_state.btn1.up == PS2X_BTN_ACTIVE) {
            printf("Up button is pressed\r\n");
        }

        LL_mDelay(500);
    }
}
