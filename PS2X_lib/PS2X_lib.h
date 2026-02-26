/* PS2X_lib.h - PS2 controller support using SPI2 */
#ifndef PS2X_LIB_H
#define PS2X_LIB_H

#include <stdint.h>
#include <stdbool.h>

// Config CS pin for SPI
#define PS2X_SPI            SPI2
#define PS2X_BUFF_SIZE      9
#define PS2X_CS_GPIO_Port   SPI2_CS_GPIO_Port
#define PS2X_CS_Pin         SPI2_CS_Pin

// Define PS2X button active state (0 when pressed, 1 when not pressed)
#define PS2X_BTN_ACTIVE     0
#define PS2X_BTN_INACTIVE   1

typedef enum {
    PS2X_MODE_DIGITAL = 0x41,
    PS2X_MODE_ANALOG = 0x73,
    PS2X_MODE_PRESSURES = 0x79
} PS2X_Mode;

// Index in PS2X raw data (8 bytes)
typedef enum {
    PS2X_IDX_MODE = 0,
    PS2X_IDX_TYPE = 1,
    PS2X_IDX_BTN1 = 2,
    PS2X_IDX_BTN2 = 3,
    PS2X_IDX_RX   = 4,
    PS2X_IDX_RY   = 5,
    PS2X_IDX_LX   = 6,
    PS2X_IDX_LY   = 7
} PS2X_Index;

// Define as bitfield for PS2X mode configuration
typedef struct {
    uint8_t select : 1; // 0x01
    uint8_t l3 : 1;     // 0x02
    uint8_t r3 : 1;     // 0x04
    uint8_t start : 1;  // 0x08
    uint8_t up : 1;     // 0x10
    uint8_t right : 1;  // 0x20
    uint8_t down : 1;   // 0x40
    uint8_t left : 1;   // 0x80
} PS2X_Button1;

typedef struct {
    uint8_t l2 : 1;       // 0x01
    uint8_t r2 : 1;       // 0x02
    uint8_t l1 : 1;       // 0x04
    uint8_t r1 : 1;       // 0x08
    uint8_t triangle : 1; // 0x10
    uint8_t circle : 1;   // 0x20
    uint8_t cross : 1;    // 0x40
    uint8_t square : 1;   // 0x80
} PS2X_Button2;

typedef struct {
    uint8_t mode; // 0x41 for digital, 0x73 for analog, 0x79 for pressure
    uint8_t type; // 0x5A for DualShock, other values for different controllers
    PS2X_Button1 btn1;
    PS2X_Button2 btn2;
    uint8_t rx; // Right stick X-axis
    uint8_t ry; // Right stick Y-axis
    uint8_t lx; // Left stick X-axis
    uint8_t ly; // Left stick Y-axis
} PS2X_State;   // Total 8 bytes of data

typedef union {
    PS2X_State state;
    uint8_t raw[PS2X_BUFF_SIZE -1]; // Exclude first byte which is 0xFF
} PS2X_Data;

// Define Error codes
#define PS2X_SUCCESS        (0)
#define PS2X_ERR_COMM       (-1)
#define PS2X_ERR_TYPE       (-2)
#define PS2X_ERR_MODE       (-3)

// GPIO control CS pin if defined
#ifdef PS2X_CS_GPIO_Port
#include "stm32f4xx_ll_gpio.h"

#define PS2X_CS_LOW()  LL_GPIO_ResetOutputPin(PS2X_CS_GPIO_Port, PS2X_CS_Pin)
#define PS2X_CS_HIGH() LL_GPIO_SetOutputPin(PS2X_CS_GPIO_Port, PS2X_CS_Pin)
#else
#define PS2X_CS_LOW()  ((void)0)
#define PS2X_CS_HIGH() ((void)0)
#endif

int ps2x_init(void);
void ps2x_read_gamepad(void);
PS2X_State ps2x_getAllData(void);
uint8_t ps2x_getMode(void);
bool ps2x_isButtonPressed(void);
bool ps2x_isJoystickActive(void);


#endif /* PS2X_LIB_H */
