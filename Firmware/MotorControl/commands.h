#ifndef COMMANDS_H
#define COMMANDS_H

// TODO: resolve assert
#define assert(expr)

#ifdef __cplusplus

#include <functional>
#include <limits>
#include "crc.hpp"

// Select which protocol to run on UART (see README for more details)
//#define UART_PROTOCOL_NEW
#define UART_PROTOCOL_LEGACY
//#define UART_PROTOCOL_NONE

// Select which protocol to run on USB (see README for more details)
#define USB_PROTOCOL_NEW
//#define USB_PROTOCOL_NEW_STREAM_BASED
//#define USB_PROTOCOL_LEGACY
//#define USB_PROTOCOL_NONE



typedef enum {
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
