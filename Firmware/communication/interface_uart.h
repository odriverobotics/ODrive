#ifndef __INTERFACE_UART_HPP
#define __INTERFACE_UART_HPP


#ifdef __cplusplus
extern "C" {
#endif

#include <cmsis_os.h>
#include "usart.h"

extern osThreadId uart_thread;
extern const uint32_t stack_size_uart_thread;

void start_uart_server(UART_HandleTypeDef* huart);
void uart_poll(void);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
#include <fibre/../../stream_utils.hpp>
extern fibre::BufferedStreamSink<64> uart0_stdout_sink;
extern bool uart0_stdout_pending;
#endif

#endif // __INTERFACE_UART_HPP
