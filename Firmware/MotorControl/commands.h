#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>
#include <stdlib.h>

// TODO: resolve assert
#define assert(expr)

// Select which protocol to run on USB (see README for more details)
#define USB_PROTOCOL_NATIVE
// #define USB_PROTOCOL_NATIVE_STREAM_BASED
// #define USB_PROTOCOL_LEGACY
#define USB_PROTOCOL_STDOUT // this can coexist with the other protocols
// #define USB_PROTOCOL_NONE

// Select which protocol to run on UART (see README for more details)
// #define UART_PROTOCOL_NATIVE
// #define UART_PROTOCOL_LEGACY
#define UART_PROTOCOL_STDOUT // this can coexist with the other protocols
// #define UART_PROTOCOL_NONE

// Use GPIO 1/2 for step/dir input instead of UART
// #define USE_GPIO_MODE_STEP_DIR


// must match order of file_table
typedef enum {
    FILENO_STDIN,
    FILENO_STDOUT,
    FILENO_STDERR,
#if !defined(USB_PROTOCOL_NONE)
    FILENO_USB,
#endif
#if !defined(UART_PROTOCOL_NONE)
    FILENO_UART,
#endif
    FILENO_NUM
} FileNumber_t;


#ifdef __cplusplus
#include "protocol.hpp"
extern StreamSink* file_table[FILENO_NUM];
#endif


#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GPIO_MODE_NONE,
    GPIO_MODE_UART,
    GPIO_MODE_STEP_DIR,
} GpioMode_t;


void init_communication(void);
void communication_task(void const * argument);
void set_cmd_buffer(uint8_t *buf, uint32_t len);
void usb_update_thread();
void USB_receive_packet(const uint8_t *buffer, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* COMMANDS_H */
