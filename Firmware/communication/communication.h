#ifndef COMMANDS_H
#define COMMANDS_H

// TODO: resolve assert
#define assert(expr)

#ifdef __cplusplus

#include <functional>
#include <limits>
#include "crc.hpp"

extern "C" {
#endif

void init_deferred_interrupts(void);
void init_communication(void);
void communication_task(void * ctx);
void set_cmd_buffer(uint8_t *buf, uint32_t len);
void usb_deferred_interrupt_thread(void * ctx);
void USB_receive_packet(const uint8_t *buffer, size_t length);

extern uint64_t serial_number;
extern char serial_number_str[13];

#ifdef __cplusplus
}
#endif

#endif /* COMMANDS_H */
