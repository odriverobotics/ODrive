#ifndef __INTERFACE_UART_HPP
#define __INTERFACE_UART_HPP

#include "fibre/protocol.hpp"
extern StreamSink* uart4_stream_output_ptr;

#include <cmsis_os.h>
#include "stm32_usart.hpp" // TODO: replace with generic usart header

#define UART_TX_BUFFER_SIZE 64
#define UART_RX_BUFFER_SIZE 64

class UARTInterface {
public:
    class UARTSender : public StreamSink {
    public:
        UARTSender(STM32_USART_t* uart) : uart_(uart) {
            osSemaphoreDef(sem_tx_def);
            sem_tx_ = osSemaphoreCreate(osSemaphore(sem_tx_def), 1);
        }

        int process_bytes(const uint8_t* buffer, size_t length, size_t* processed_bytes);

        void handle_tx_done() { osSemaphoreRelease(sem_tx_); };

        size_t get_free_space() { return SIZE_MAX; }

    private:
        STM32_USART_t* uart_;
        uint8_t tx_buf_[UART_TX_BUFFER_SIZE];
        osSemaphoreId sem_tx_;
    };

    UARTInterface(STM32_USART_t* uart) :
        uart(uart),
        stream_output(uart),
        packet_output(stream_output),
        channel(packet_output),
        stream_input(channel) {}

    bool start_server();

    STM32_USART_t* uart;
    UARTSender stream_output;
    StreamBasedPacketSink packet_output;
    BidirectionalPacketBasedChannel channel;
    StreamToPacketSegmenter stream_input;
    osThreadId uart_thread;

private:
    void server_thread();

    // DMA open loop continous circular buffer
    // 1ms delay periodic, chase DMA ptr around
    uint8_t rx_buf_[UART_RX_BUFFER_SIZE];
};

#endif // __INTERFACE_UART_HPP
