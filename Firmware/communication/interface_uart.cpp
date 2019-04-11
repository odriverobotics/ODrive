
#include "interface_uart.h"

#include "ascii_protocol.hpp"

#include <MotorControl/utils.h>

#include <fibre/protocol.hpp>
#include <stm32_usart.hpp>
#include <cmsis_os.h>
#include <freertos_vars.h>
#include <odrive_main.h>

// FIXME: the stdlib doesn't know about CMSIS threads, so this is just a global variable
// static thread_local uint32_t deadline_ms = 0;

int UARTInterface::UARTSender::process_bytes(const uint8_t* buffer, size_t length, size_t* processed_bytes) {
    if (!uart_) {
        return -1;
    }

    // Loop to ensure all bytes get sent
    while (length) {
        size_t chunk = length < UART_TX_BUFFER_SIZE ? length : UART_TX_BUFFER_SIZE;
        // wait for UART interface to become ready
        // TODO: implement ring buffer to get a more continuous stream of data
        // if (osSemaphoreWait(sem_tx_, deadline_to_timeout(deadline_ms)) != osOK)
        if (osSemaphoreWait(sem_tx_, PROTOCOL_SERVER_TIMEOUT_MS) != osOK)
            return -1;
        // transmit chunk
        memcpy(tx_buf_, buffer, chunk);
        if (!uart_->start_tx(tx_buf_, chunk,
                [](void* ctx){ if (ctx) ((UARTSender*)ctx)->handle_tx_done(); }, this
            )) {
            return -1;
        }
        
        buffer += chunk;
        length -= chunk;
        if (processed_bytes)
            *processed_bytes += chunk;
    }
    return 0;
}

bool UARTInterface::start_server() {
    if (!uart) {
        return false;
    }

    // DMA is set up to recieve in a circular buffer forever.
    // We dont use interrupts to fetch the data, instead we periodically read
    // data out of the circular buffer into a parse buffer, controlled by a state machine
    if (!uart->start_rx(rx_buf_, sizeof(rx_buf_))) {
        return false;
    }

    // Start UART communication thread
    osThreadDef(server_thread_def,
            [](void* ctx){ if (ctx) ((UARTInterface*)ctx)->server_thread(); },
            RTOS_PRIO_UART, 0, 1024 /* the ascii protocol needs considerable stack space */);
    uart_thread = osThreadCreate(osThread(server_thread_def), this);

    return false;
}

void UARTInterface::server_thread() {
    if (!uart) {
        return;
    }

    bool needs_restart = false;

    for (;;) {
        // Check for UART errors and restart recieve DMA transfer if required
        needs_restart = (needs_restart && !uart->check_error());
        if (needs_restart) {
            needs_restart = !uart->stop_rx() &&
                            !uart->start_rx(rx_buf_, sizeof(rx_buf_));
        } else {
            // deadline_ms = timeout_to_deadline(PROTOCOL_SERVER_TIMEOUT_MS);
            // Process bytes in one or two chunks (two in case there was a wrap)
            for (size_t i = 0; i < 2; ++i) {
                uint8_t* data;
                size_t length;
                if (uart->get_rx_data(&data, &length)) {
                    stream_input.process_bytes(data, length, nullptr); // TODO: use process_all
                    //ASCII_protocol_parse_stream(data, length, stream_output);
                }
            }
        }

        osDelay(1);
    }
}
