
#include "interface_uart.h"

#include "ascii_protocol.hpp"

#include <MotorControl/utils.hpp>

#include <fibre/protocol.hpp>
#include <usart.h>
#include <cmsis_os.h>
#include <freertos_vars.h>

#define UART_TX_BUFFER_SIZE 64
#define UART_RX_BUFFER_SIZE 64

// DMA open loop continous circular buffer
// 1ms delay periodic, chase DMA ptr around
static uint8_t dma_rx_buffer[UART_RX_BUFFER_SIZE];
static uint32_t dma_last_rcv_idx;

// FIXME: the stdlib doesn't know about CMSIS threads, so this is just a global variable
// static thread_local uint32_t deadline_ms = 0;

osThreadId uart_thread;
extern UART_HandleTypeDef* uart0;
static UART_HandleTypeDef* huart_ = uart0; // defined in board.cpp.
const uint32_t stack_size_uart_thread = 4096;  // Bytes


class UARTSender : public StreamSink {
public:
    int process_bytes(const uint8_t* buffer, size_t length, size_t* processed_bytes) {
        // Loop to ensure all bytes get sent
        while (length) {
            size_t chunk = length < UART_TX_BUFFER_SIZE ? length : UART_TX_BUFFER_SIZE;
            // wait for USB interface to become ready
            // TODO: implement ring buffer to get a more continuous stream of data
            // if (osSemaphoreWait(sem_uart_dma, deadline_to_timeout(deadline_ms)) != osOK)
            if (osSemaphoreWait(sem_uart_dma, PROTOCOL_SERVER_TIMEOUT_MS) != osOK)
                return -1;
            // transmit chunk
            memcpy(tx_buf_, buffer, chunk);
            if (HAL_UART_Transmit_DMA(huart_, tx_buf_, chunk) != HAL_OK)
                return -1;
            buffer += chunk;
            length -= chunk;
            if (processed_bytes)
                *processed_bytes += chunk;
        }
        return 0;
    }

    size_t get_free_space() { return SIZE_MAX; }
private:
    uint8_t tx_buf_[UART_TX_BUFFER_SIZE];
} uart_stream_output;
StreamSink* uart_stream_output_ptr = &uart_stream_output;

StreamBasedPacketSink uart_packet_output(uart_stream_output);
BidirectionalPacketBasedChannel uart_channel(uart_packet_output);
StreamToPacketSegmenter uart_stream_input(uart_channel);

static void uart_server_thread(void * ctx) {
    (void) ctx;

    for (;;) {
        // Check for UART errors and restart receive DMA transfer if required
        if (huart_->RxState != HAL_UART_STATE_BUSY_RX) {
            HAL_UART_AbortReceive(huart_);
            HAL_UART_Receive_DMA(huart_, dma_rx_buffer, sizeof(dma_rx_buffer));
            dma_last_rcv_idx = 0;
        }
        // Fetch the circular buffer "write pointer", where it would write next
        uint32_t new_rcv_idx = UART_RX_BUFFER_SIZE - huart_->hdmarx->Instance->NDTR;
        if (new_rcv_idx > UART_RX_BUFFER_SIZE) { // defensive programming
            continue;
        }

        // deadline_ms = timeout_to_deadline(PROTOCOL_SERVER_TIMEOUT_MS);
        // Process bytes in one or two chunks (two in case there was a wrap)
        if (new_rcv_idx < dma_last_rcv_idx) {
            uart_stream_input.process_bytes(dma_rx_buffer + dma_last_rcv_idx,
                    UART_RX_BUFFER_SIZE - dma_last_rcv_idx, nullptr); // TODO: use process_all
            ASCII_protocol_parse_stream(dma_rx_buffer + dma_last_rcv_idx,
                    UART_RX_BUFFER_SIZE - dma_last_rcv_idx, uart_stream_output);
            dma_last_rcv_idx = 0;
        }
        if (new_rcv_idx > dma_last_rcv_idx) {
            uart_stream_input.process_bytes(dma_rx_buffer + dma_last_rcv_idx,
                    new_rcv_idx - dma_last_rcv_idx, nullptr); // TODO: use process_all
            ASCII_protocol_parse_stream(dma_rx_buffer + dma_last_rcv_idx,
                    new_rcv_idx - dma_last_rcv_idx, uart_stream_output);
            dma_last_rcv_idx = new_rcv_idx;
        }

        // The thread is woken up by the control loop at 8kHz. This should be
        // enough for most applications.
        // At 1Mbaud/s that corresponds to at most 12.5 bytes which can arrive
        // during the sleep period.
        osThreadSuspend(nullptr);
    }
}

// TODO: allow multiple UART server instances
void start_uart_server() {
    // DMA is set up to receive in a circular buffer forever.
    // We dont use interrupts to fetch the data, instead we periodically read
    // data out of the circular buffer into a parse buffer, controlled by a state machine
    HAL_UART_Receive_DMA(huart_, dma_rx_buffer, sizeof(dma_rx_buffer));
    dma_last_rcv_idx = 0;

    // Start UART communication thread
    osThreadDef(uart_server_thread_def, uart_server_thread, osPriorityNormal, 0, stack_size_uart_thread / sizeof(StackType_t) /* the ascii protocol needs considerable stack space */);
    uart_thread = osThreadCreate(osThread(uart_server_thread_def), NULL);
}

void uart_poll() {
    osThreadResume(uart_thread);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    osSemaphoreRelease(sem_uart_dma);
}
