#ifndef __INTERFACE_UART_HPP
#define __INTERFACE_UART_HPP

#ifdef __cplusplus
#include "fibre/protocol.hpp"
#include "CircularBuffer.hpp"
extern StreamSink* uart4_stream_output_ptr;

extern "C" {
#endif

#include <cmsis_os.h>
#include <usart.h>


#define UART_TX_BUFFER_SIZE (64)

extern osThreadId uart_thread;
extern const uint32_t stack_size_uart_thread;

void start_uart_server(void);

#ifdef __cplusplus
}
#endif

class UARTSender
    : public StreamSink
{
public:
    UARTSender(UART_HandleTypeDef * pHuart);
    virtual ~UARTSender() = default;
    int process_bytes(const uint8_t* buffer, size_t length, size_t* processed_bytes);
    size_t get_free_space(void);

private:
    UART_HandleTypeDef * m_pHuart;
    uint8_t tx_buf_[UART_TX_BUFFER_SIZE];
    uint8_t TXBuffer[UART_TX_BUFFER_SIZE];
    CCBBuffer<uint8_t> TXCircularBuffer;
};

#endif // __INTERFACE_UART_HPP
