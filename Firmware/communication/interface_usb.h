#ifndef __INTERFACE_USB_HPP
#define __INTERFACE_USB_HPP


#ifdef __cplusplus
extern "C" {
#endif

#include <cmsis_os.h>
#include <stdint.h>

extern osThreadId usb_thread;
extern const uint32_t stack_size_usb_thread;

typedef struct {
    uint32_t rx_cnt;
    uint32_t tx_cnt;
    uint32_t tx_overrun_cnt;
} USBStats_t;

extern USBStats_t usb_stats_;

void usb_rx_process_packet(uint8_t *buf, uint32_t len, uint8_t endpoint_pair);
void start_usb_server(void);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
#include <fibre/../../stream_utils.hpp>
extern fibre::BufferedStreamSink<64> usb_cdc_stdout_sink;
extern bool usb_cdc_stdout_pending;
#endif

#endif // __INTERFACE_USB_HPP
