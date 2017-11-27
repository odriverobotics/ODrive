#ifndef COMMANDS_H
#define COMMANDS_H

// TODO: resolve assert
#define assert(expr)

#ifdef __cplusplus

#include <functional>
#include <limits>
#include "crc.hpp"

// Select which protocol to run on USB (see README for more details)
#define USB_PROTOCOL_NATIVE
// #define USB_PROTOCOL_NATIVE_STREAM_BASED
// #define USB_PROTOCOL_LEGACY
// #define USB_PROTOCOL_NONE

// Select which protocol to run on UART (see README for more details)
// #define UART_PROTOCOL_NATIVE
// #define UART_PROTOCOL_LEGACY
#define UART_PROTOCOL_NONE

// Use GPIO 1/2 for step/dir input instead of UART
// #define USE_GPIO_MODE_STEP_DIR


typedef enum {
    GPIO_MODE_NONE,
    GPIO_MODE_UART,
    GPIO_MODE_STEP_DIR,
} GpioMode_t;

extern "C" {
#endif

void init_communication(void);
void communication_task(void const * argument);
void set_cmd_buffer(uint8_t *buf, uint32_t len);
void usb_update_thread();
void USB_receive_packet(const uint8_t *buffer, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* COMMANDS_H */
