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

void init_communication(void);
void communication_task(void const * argument);
void set_cmd_buffer(uint8_t *buf, uint32_t len);
void usb_update_thread();
void USB_receive_packet(const uint8_t *buffer, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* COMMANDS_H */
