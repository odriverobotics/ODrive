
#include "interface_uart.h"

#include "ascii_protocol.hpp"

#include <MotorControl/utils.hpp>

#include <fibre/async_stream.hpp>
#include <fibre/../../legacy_protocol.hpp>
#include <usart.h>
#include <cmsis_os.h>
#include <freertos_vars.h>
#include <odrive_main.h>

#define UART_TX_BUFFER_SIZE 64
#define UART_RX_BUFFER_SIZE 64

// DMA open loop continous circular buffer
// 1ms delay periodic, chase DMA ptr around
static uint8_t dma_rx_buffer[UART_RX_BUFFER_SIZE];
static uint32_t dma_last_rcv_idx;

osThreadId uart_thread = 0;
static UART_HandleTypeDef* huart_ = nullptr;
const uint32_t stack_size_uart_thread = 4096;  // Bytes

namespace fibre {

class Stm32UartTxStream : public AsyncStreamSink {
public:
    Stm32UartTxStream(UART_HandleTypeDef* huart) : huart_(huart) {}

    void start_write(cbufptr_t buffer, TransferHandle* handle, Callback<void, WriteResult> completer) final;
    void cancel_write(TransferHandle transfer_handle) final;
    void did_finish();

    UART_HandleTypeDef *huart_;
    Callback<void, WriteResult> completer_;
    const uint8_t* tx_end_ = nullptr;
};

class Stm32UartRxStream : public AsyncStreamSource {
public:
    void start_read(bufptr_t buffer, TransferHandle* handle, Callback<void, ReadResult> completer) final;
    void cancel_read(TransferHandle transfer_handle) final;
    void did_receive(uint8_t* buffer, size_t length);

    Callback<void, ReadResult> completer_;
    bufptr_t rx_buf_ = {nullptr, nullptr};
};

}

using namespace fibre;

void Stm32UartTxStream::start_write(cbufptr_t buffer, TransferHandle* handle, Callback<void, WriteResult> completer) {
    size_t chunk = std::min(buffer.size(), (size_t)UART_TX_BUFFER_SIZE);

    completer_ = completer;
    tx_end_ = buffer.begin() + chunk;

    if (handle) {
        *handle = reinterpret_cast<TransferHandle>(this);
    }

    if (HAL_UART_Transmit_DMA(huart_, const_cast<uint8_t*>(buffer.begin()), chunk) != HAL_OK) {
        completer_ = nullptr;
        tx_end_ = nullptr;
        completer.invoke({kStreamError, buffer.begin()});
    }
}

void Stm32UartTxStream::cancel_write(TransferHandle transfer_handle) {
    // not implemented
}

void Stm32UartTxStream::did_finish() {
    const uint8_t* tx_end = tx_end_;
    tx_end_ = nullptr;
    completer_.invoke_and_clear({kStreamOk, tx_end});
}

void Stm32UartRxStream::start_read(bufptr_t buffer, TransferHandle* handle, Callback<void, ReadResult> completer) {
    completer_ = completer;
    rx_buf_ = buffer;
    if (handle) {
        *handle = reinterpret_cast<TransferHandle>(this);
    }
}

void Stm32UartRxStream::cancel_read(TransferHandle transfer_handle) {
    // not implemented
}

void Stm32UartRxStream::did_receive(uint8_t* buffer, size_t length) {
    // This can be called even if there was no RX operation in progress

    bufptr_t rx_buf = rx_buf_;

    if (completer_ && rx_buf.begin()) {
        rx_buf_ = {nullptr, nullptr};
        size_t chunk = std::min(length, rx_buf.size());
        memcpy(rx_buf.begin(), buffer, chunk);
        completer_.invoke_and_clear({kStreamOk, rx_buf.begin() + chunk});
    }
}

Stm32UartTxStream uart_tx_stream(huart_);
Stm32UartRxStream uart_rx_stream;

LegacyProtocolStreamBased fibre_over_uart(&uart_rx_stream, &uart_tx_stream);

fibre::AsyncStreamSinkMultiplexer<2> uart_tx_multiplexer(uart_tx_stream);
fibre::BufferedStreamSink<64> uart0_stdout_sink(uart_tx_multiplexer); // Used in communication.cpp
AsciiProtocol ascii_over_uart(&uart_rx_stream, &uart_tx_multiplexer);

bool uart0_stdout_pending = false;

static void uart_server_thread(void * ctx) {
    (void) ctx;

    if (odrv.config_.uart0_protocol == ODrive::STREAM_PROTOCOL_TYPE_FIBRE) {
        fibre_over_uart.start({});
    } else if (odrv.config_.uart0_protocol == ODrive::STREAM_PROTOCOL_TYPE_ASCII
            || odrv.config_.uart0_protocol == ODrive::STREAM_PROTOCOL_TYPE_ASCII_AND_STDOUT) {
        ascii_over_uart.start();
    }

    for (;;) {
        osEvent event = osMessageGet(uart_event_queue, osWaitForever);

        if (event.status != osEventMessage) {
            continue;
        }

        switch (event.value.v) {
            case 1: {
                // This event is triggered by the control loop at 8kHz. This should be
                // enough for most applications.
                // At 1Mbaud/s that corresponds to at most 12.5 bytes which can arrive
                // during the sleep period.

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

                // Process bytes in one or two chunks (two in case there was a wrap)
                if (new_rcv_idx < dma_last_rcv_idx) {
                    uart_rx_stream.did_receive(dma_rx_buffer + dma_last_rcv_idx,
                            UART_RX_BUFFER_SIZE - dma_last_rcv_idx);
                    dma_last_rcv_idx = 0;
                }
                if (new_rcv_idx > dma_last_rcv_idx) {
                    uart_rx_stream.did_receive(dma_rx_buffer + dma_last_rcv_idx,
                            new_rcv_idx - dma_last_rcv_idx);
                    dma_last_rcv_idx = new_rcv_idx;
                }
            } break;

            case 2: {
                uart_tx_stream.did_finish();
            } break;

            case 3: { // stdout has data
                uart0_stdout_pending = false;
                uart0_stdout_sink.maybe_start_async_write();
            } break;
        }
    }
}

// TODO: allow multiple UART server instances
void start_uart_server(UART_HandleTypeDef* huart) {
    huart_ = huart;
    uart_tx_stream.huart_ = huart;

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
    if (uart_thread) { // the thread is only started if UART is enabled
        osMessagePut(uart_event_queue, 1, 0);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == huart_) {
        osMessagePut(uart_event_queue, 2, 0);
    }
}
