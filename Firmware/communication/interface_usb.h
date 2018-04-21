#ifndef __INTERFACE_USB_HPP
#define __INTERFACE_USB_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void usb_process_packet(uint8_t *buf, uint32_t len);
void serve_on_usb(void);

#ifdef __cplusplus
}
#endif

#endif // __INTERFACE_USB_HPP
