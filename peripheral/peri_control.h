#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Peripheral control functions for main
void peripheral_init(void);
void peripheral_control_loop(void);
void peripheral_tim10_interrupt_handler(void);

#ifdef __cplusplus
}
#endif
